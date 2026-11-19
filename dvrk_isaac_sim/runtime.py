"""Isaac Sim runtime and multi-arm simulation loop for dVRK robots."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import time
from typing import Mapping, Sequence

import numpy as np

from dvrk_arm_description import RobotConfig
from dvrk_simulator_base.cartesian_command import CartesianCommand, resolve_cartesian_command
from dvrk_simulator_base.command_mailbox import CommandMailboxes
from dvrk_simulator_base.arm_controller import ArmController
from dvrk_simulator_base.publication_frames import with_publication_frames
from dvrk_simulator_base.scene import SceneConfig, resolve_asset_uri
from dvrk_simulator_base.snapshots import ArmSnapshot, OperatingStateSnapshot
from dvrk_simulator_base.types import JointState, Pose

from .camera import CameraOptions, IsaacCamera
from .kinematics import CRTKECM, CRTKPSM
from .python_runtime import default_generated_root
from .usd_visual import CRTKUSDVisual


@dataclass(frozen=True)
class RuntimeOptions:
    headless: bool = False
    renderer: str = "RaytracedLighting"
    simulation_rate_hz: float = 120.0
    render_rate_hz: float = 30.0
    generated_root: Path | None = None
    camera_options: CameraOptions | None = None
    collision_backend: str = "newton"
    newton_python: str | None = None
    newton_device: str = "auto"

    def __post_init__(self) -> None:
        if self.collision_backend not in {"newton", "none"}:
            raise ValueError(f"unsupported collision backend: {self.collision_backend!r}")
        if not isinstance(self.newton_device, str) or not self.newton_device.strip():
            raise ValueError("newton_device must be a non-empty string")


def _generated_variant(generated_dir: Path, robot: RobotConfig) -> tuple[Path, Path]:
    variant = (
        robot.instrument or "420006"
        if robot.type == "PSM"
        else robot.endoscope or "Si_straight"
    )
    asset_dir = generated_dir.expanduser().resolve() / f"{robot.name}_{variant}"
    return asset_dir / robot.name / f"{robot.name}.usda", asset_dir / "kinematics.json"


def _reference_usd(path: Path | None, prim_path: str, position=None, orientation_xyzw=None) -> None:
    if path is None:
        return
    if not path.is_file():
        raise FileNotFoundError(f"USD asset not found: {path}")
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    prim = stage.DefinePrim(prim_path, "Xform")
    prim.GetReferences().AddReference(str(path.resolve()))
    if position is not None or orientation_xyzw is not None:
        from pxr import Gf, UsdGeom

        xform = UsdGeom.Xformable(prim)
        ops = xform.GetOrderedXformOps()
        if position is not None:
            translate_op = next((op for op in ops if op.GetOpType() == UsdGeom.XformOp.TypeTranslate), None)
            if translate_op is None:
                translate_op = xform.AddTranslateOp()
            if translate_op.GetAttr().GetTypeName() == "float3":
                translate_op.Set(Gf.Vec3f(*[float(v) for v in position]))
            else:
                translate_op.Set(Gf.Vec3d(*[float(v) for v in position]))
        if orientation_xyzw is not None:
            quat = Gf.Quatd(float(orientation_xyzw[3]), Gf.Vec3d(*[float(v) for v in orientation_xyzw[:3]]))
            orient_op = next((op for op in ops if op.GetOpType() == UsdGeom.XformOp.TypeOrient), None)
            if orient_op is None:
                orient_op = xform.AddOrientOp()
            if orient_op.GetAttr().GetTypeName() == "quatf":
                orient_op.Set(Gf.Quatf(quat))
            else:
                orient_op.Set(quat)


def _view_pose_from_ecm_tip(tip_pose: Pose) -> Pose:
    # Isaac camera axes are forward, left, up. Haply's dVRK view axes
    # are left, up, forward, with the same origin at the lens.
    return Pose(tip_pose.position, tip_pose.orientation[:, [1, 2, 0]])


def _setup_scene_lighting() -> None:
    import omni.usd
    from pxr import Gf, UsdGeom, UsdLux

    stage = omni.usd.get_context().get_stage()
    dome = UsdLux.DomeLight.Define(stage, "/World/Lighting/DomeLight")
    dome.CreateColorAttr(Gf.Vec3f(0.22, 0.22, 0.22))
    dome.CreateIntensityAttr(450.0)
    key = UsdLux.DistantLight.Define(stage, "/World/Lighting/KeyLight")
    key.CreateColorAttr(Gf.Vec3f(1.0, 0.96, 0.90))
    key.CreateIntensityAttr(1800.0)
    key.CreateAngleAttr(0.5)
    UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(35.0, -25.0, -30.0))


class IsaacSimArm(ArmController):
    """Manages kinematics, visual synchronization, and command mailboxes for an arm."""

    def __init__(
        self,
        config: RobotConfig,
        commands: CommandMailboxes,
        manifest_path: Path | None,
    ) -> None:
        self.config = config
        self.commands = commands
        self.manifest_path = manifest_path

        if config.type == "PSM":
            self.model = CRTKPSM(config, kinematics_manifest=manifest_path)
        elif config.type == "ECM":
            self.model = CRTKECM(config, kinematics_manifest=manifest_path)
        else:
            raise ValueError(f"unsupported robot type: {config.type}")

        self.visual: CRTKUSDVisual | None = None
        self.physics_sync = None
        super().__init__(config, commands)
        self.jaw_position = self.jaw_setpoint

    def initialize_visual(self, *, sync_collisions: bool = False) -> None:
        if self.manifest_path is not None and self.manifest_path.is_file():
            import omni.usd

            stage = omni.usd.get_context().get_stage()
            if stage.GetPrimAtPath(f"/World/{self.config.name}").IsValid():
                self.visual = CRTKUSDVisual(self.config.name, self.manifest_path)
                measured = self.model.measured_js()
                self.visual.update(measured.names, measured.position,
                                   self.jaw_position if self.config.type == "PSM" else None)
                if sync_collisions and self.config.type == "PSM":
                    if self.model._urdf_chain is None:
                        raise RuntimeError(
                            f"{self.config.name}: Newton collision sync requires a URDF manifest"
                        )
                    from .usd_physics_links import PhysicsLinkSync

                    self.physics_sync = PhysicsLinkSync(
                        self.config.name,
                        self.manifest_path,
                        self.model._urdf_chain,
                        self.model._base_transform,
                    )
                    self.physics_sync.update(
                        measured.names, measured.position, self.jaw_position
                    )

    def prepare_step(
        self,
        dt: float,
        now: float,
        ecm_pose: Pose | None = None,
        *,
        has_ecm: bool = False,
    ) -> None:
        self.advance_commands(
            now, lambda target, seed: self.model.compute_ik(target),
            lambda target: resolve_cartesian_command(target, self.config, ecm_pose, has_ecm=has_ecm),
            measured_position=self.model.measured_js().position,
        )
        self.model.servo_jp(self.joint_setpoint)
        self.jaw_position = self.jaw_setpoint

        self.model.step(dt)

        if self.visual is not None:
            self.visual.update(
                self.model.measured_js().names,
                self.model.measured_js().position,
                self.jaw_position if self.config.type == "PSM" else None,
            )

    def snapshot(self, simulation_time: float, sequence: int) -> ArmSnapshot:
        measured_js = self.model.measured_js()
        measured_cp = self.model.measured_cp()
        state_snapshot = OperatingStateSnapshot(
            self.operating_state.state,
            is_homed=self.operating_state.is_homed,
            is_busy=(
                self.joint_trajectory is not None
                or self.jaw_trajectory is not None
                or self.move_failure_pending
            ),
        )
        arm_snapshot = ArmSnapshot(
            sequence=sequence,
            simulation_time=simulation_time,
            valid=True,
            measured_js=measured_js,
            setpoint_js=JointState(measured_js.names, self.joint_setpoint, self.joint_velocity),
            measured_cp_world=measured_cp,
            setpoint_cp_world=self.model.compute_fk(self.joint_setpoint),
            measured_cv_world=self.model.measured_cv(),
            jaw_measured=self.jaw_position if self.config.type == "PSM" else None,
            jaw_setpoint=self.jaw_setpoint if self.config.type == "PSM" else None,
            operating_state=state_snapshot,
            operating_state_event=self.operating_state_event_pending,
        )
        self.operating_state_event_pending = False
        self.move_failure_pending = False
        return arm_snapshot


class IsaacSimRuntime:
    """Multi-arm simulation runtime backed by NVIDIA Isaac Sim."""

    def __init__(
        self,
        scene_robots: Sequence[RobotConfig],
        options: RuntimeOptions,
        commands: Mapping[str, CommandMailboxes],
        scene_config: SceneConfig | None = None,
    ) -> None:
        self.options = options
        self.commands = commands
        self.scene_config = scene_config
        self.scene_robots = tuple(scene_robots)
        self.generated_root = options.generated_root or default_generated_root()

        self.simulation_app = None
        self.camera: IsaacCamera | None = None
        self.collision_backend = None
        self.command_warnings: list[tuple[str, str]] = []
        self._simulation_time = 0.0
        self._sequence = 0
        self._connected = False
        self._next_render = 0.0
        self.timeline = None

        self.arms: dict[str, IsaacSimArm] = {}

    def initialize(self) -> dict[str, ArmSnapshot]:
        from isaacsim import SimulationApp

        self.simulation_app = SimulationApp({
            "headless": self.options.headless,
            "renderer": self.options.renderer,
            "minimal_shading_mode": 2,
            "anti_aliasing": 2 if self.options.renderer == "MinimalRendering" else 3,
            "multi_gpu": False,
            "disable_viewport_updates": self.options.headless,
        })
        self._connected = True

        from isaacsim.core.utils.extensions import enable_extension

        if (
            self.options.camera_options is not None
            and "rtsp" in self.options.camera_options.transports
        ):
            enable_extension("isaacsim.streaming.rtsp")

        self.simulation_app.update()

        from .assets import convert_model
        from .collision_decomposition import collision_cache_valid
        from .collision_primitives import has_instrument_collisions
        from .urdf_kinematics import MANIFEST_FORMAT
        from ament_index_python.packages import get_package_share_directory

        collision_profile = (
            Path(get_package_share_directory("dvrk_isaac_sim"))
            / "share/collision_primitives/instrument_collision.yaml"
        )
        if self.options.collision_backend == "newton" and not collision_profile.is_file():
            raise FileNotFoundError(f"Newton collision profile not found: {collision_profile}")

        for robot in self.scene_robots:
            asset_path, manifest_path = _generated_variant(self.generated_root, robot)
            needs_conversion = not asset_path.is_file() or not manifest_path.is_file()
            if not needs_conversion:
                try:
                    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
                except (OSError, json.JSONDecodeError):
                    needs_conversion = True
                else:
                    # Stale manifests predate kinematic frame conventions such
                    # as the ECM tip/view change and must be regenerated.
                    needs_conversion = int(manifest.get("format", 0)) < MANIFEST_FORMAT
                    if not needs_conversion and self.options.collision_backend == "newton" and robot.type == "PSM":
                        needs_conversion = not (
                            has_instrument_collisions(manifest)
                            and collision_cache_valid(manifest, collision_profile)
                        )
            if needs_conversion:
                convert_model(model=robot.name, instrument=robot.instrument or "420006",
                              endoscope=robot.endoscope or "Si_straight",
                              output_dir=self.generated_root, force=True,
                              collision_profile=collision_profile if self.options.collision_backend == "newton" and robot.type == "PSM" else None,
                              require_instrument_collisions=self.options.collision_backend == "newton" and robot.type == "PSM")
            _reference_usd(
                asset_path,
                f"/World/{robot.name}",
                robot.base_position,
                robot.base_orientation_xyzw,
            )
            self.arms[robot.name] = IsaacSimArm(robot, self.commands[robot.name], manifest_path)

        if self.scene_config is not None:
            self._load_scene_objects()

        _setup_scene_lighting()
        self.simulation_app.update()

        for arm in self.arms.values():
            arm.initialize_visual(sync_collisions=self.options.collision_backend == "newton")

        if self.options.collision_backend == "newton":
            from .collision_backend import make_collision_backend
            from .newton_runtime import resolve_newton_python
            import omni.usd

            stage = omni.usd.get_context().get_stage()
            physics_links = {
                name: arm.physics_sync
                for name, arm in self.arms.items()
                if arm.physics_sync is not None
            }
            self.collision_backend = make_collision_backend(
                "newton",
                stage=stage,
                physics_links=physics_links,
                has_environment=bool(self.scene_config and self.scene_config.objects),
                newton_python=Path(resolve_newton_python(self.options.newton_python)),
                newton_device=self.options.newton_device,
                initial_time=self._simulation_time,
            )
        else:
            import omni.usd
            from pxr import UsdPhysics

            stage = omni.usd.get_context().get_stage()
            for prim in stage.Traverse():
                if prim.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr().Set(False)
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr().Set(False)

        if self.options.camera_options is not None and self.options.camera_options.enabled:
            self.camera = IsaacCamera(self.options.camera_options)

        from omni.timeline import get_timeline_interface
        self.timeline = get_timeline_interface()
        self.timeline.set_time_codes_per_second(self.options.simulation_rate_hz)
        self.timeline.set_target_framerate(self.options.render_rate_hz)
        self.timeline.play()
        ecm = next((arm for arm in self.arms.values() if arm.config.type == "ECM"), None)
        if self.camera is not None:
            if ecm is not None:
                self.camera.set_pose(ecm.model.measured_cp())
            self.camera.start_streaming()

        snapshots = {
            name: arm.snapshot(self._simulation_time, self._sequence)
            for name, arm in self.arms.items()
        }
        return with_publication_frames(
            snapshots, [arm.config for arm in self.arms.values()],
            ecm_view_pose=(
                _view_pose_from_ecm_tip(snapshots[ecm.config.name].measured_cp_world)
                if ecm is not None else None
            ),
        )

    def _load_scene_objects(self) -> None:
        """Import exercise objects with their declared fixed/dynamic behavior."""
        from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig
        import omni.usd
        from pxr import Gf, Sdf, UsdGeom, UsdPhysics

        if self.scene_config.objects:
            stage = omni.usd.get_context().get_stage()
            UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
            UsdGeom.Xform.Define(stage, "/World/Environment")
        for item in self.scene_config.objects:
            settings = item.as_dict()
            prim_path = f"/World/Environment/{item.name}"
            if item.shape is not None:
                stage = omni.usd.get_context().get_stage()
                size = tuple(float(value) for value in settings["size"])
                color = tuple(float(value) for value in settings.get("color", (0.7, 0.7, 0.7, 1.0)))
                if item.shape == "cone_bowl":
                    from .environment_geometry import spawn_cone_bowl

                    root = spawn_cone_bowl(stage, Sdf.Path(prim_path), size, color)
                    xform = UsdGeom.Xformable(root.GetPrim())
                elif item.shape == "box":
                    cube = UsdGeom.Cube.Define(stage, prim_path)
                    cube.CreateSizeAttr(1.0)
                    cube.CreateDisplayColorAttr([Gf.Vec3f(*color[:3])])
                    cube.CreateDisplayOpacityAttr([color[3]])
                    xform = UsdGeom.Xformable(cube.GetPrim())
                    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
                    root = cube
                elif item.shape == "sphere":
                    sphere = UsdGeom.Sphere.Define(stage, prim_path)
                    sphere.CreateRadiusAttr(size[0] * 0.5)
                    sphere.CreateDisplayColorAttr([Gf.Vec3f(*color[:3])])
                    sphere.CreateDisplayOpacityAttr([color[3]])
                    UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
                    xform = UsdGeom.Xformable(sphere.GetPrim())
                    root = sphere
                else:
                    raise ValueError(f"unsupported scene object shape: {item.shape}")
                xform.AddTranslateOp().Set(Gf.Vec3d(*item.position))
                # AddOrientOp() authors a float-precision op; setting a Gf.Quatd
                # on it is a USD type mismatch that Kit only logs, silently
                # dropping the configured rotation.
                xform.AddOrientOp().Set(
                    Gf.Quatf(
                        float(item.orientation_xyzw[3]),
                        Gf.Vec3f(*[float(v) for v in item.orientation_xyzw[:3]]),
                    )
                )
                if item.shape == "box":
                    # Scale must come after translate/orient in xformOpOrder so
                    # it only sizes the cube instead of scaling its placement.
                    xform.AddScaleOp().Set(Gf.Vec3f(*size))
                if not item.fixed:
                    rigid = UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
                    rigid.CreateKinematicEnabledAttr().Set(False)
                    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr().Set(
                        float(settings["mass"])
                    )
                continue

            asset_path = resolve_asset_uri(item.asset)
            if asset_path.suffix.lower() == ".urdf":
                destination = self.generated_root / "objects" / item.name
                destination.mkdir(parents=True, exist_ok=True)
                config = URDFImporterConfig()
                config.urdf_path = str(asset_path)
                config.usd_path = str(destination)
                config.fix_base = item.fixed
                config.merge_fixed_joints = True
                converted = URDFImporter(config).import_urdf()
                if not converted:
                    raise RuntimeError(f"could not import scene object {item.name}")
                asset_path = Path(converted)
            elif asset_path.suffix.lower() not in {".usd", ".usda", ".usdc"}:
                raise ValueError(f"unsupported Isaac scene object asset: {asset_path}")
            _reference_usd(asset_path, prim_path,
                           item.position, item.orientation_xyzw)

    def step(self) -> dict[str, ArmSnapshot]:
        dt = 1.0 / self.options.simulation_rate_hz
        now = time.monotonic()
        self._simulation_time += dt
        self._sequence += 1

        ecm = next((arm for arm in self.arms.values() if arm.config.type == "ECM"), None)
        ecm_pose = ecm.model.measured_cp() if ecm is not None else None
        ecm_view_pose = _view_pose_from_ecm_tip(ecm_pose) if ecm_pose is not None else None

        for name, arm in self.arms.items():
            reference = ecm_view_pose if arm.config.type == "PSM" else ecm_pose
            arm.prepare_step(dt, now, reference, has_ecm=ecm is not None)
            self.command_warnings.extend((name, text) for text in arm.command_warnings)
            arm.command_warnings.clear()

        for arm in self.arms.values():
            if arm.physics_sync is not None:
                measured = arm.model.measured_js()
                arm.physics_sync.update(
                    measured.names, measured.position,
                    arm.jaw_position if arm.config.type == "PSM" else None,
                )
        if self.collision_backend is not None:
            self.collision_backend.advance(
                self._simulation_time,
                playing=True,
                render_period=dt,
            )
            self.collision_backend.render()

        if self.simulation_app is not None and now >= self._next_render:
            if self.camera is not None and ecm is not None:
                self.camera.set_pose(ecm.model.measured_cp())
            if self.timeline is not None:
                self.timeline.set_current_time(self._simulation_time)
            self.simulation_app.update()
            if self.camera is not None:
                self.camera.capture(self._simulation_time)
            self._next_render = max(self._next_render + 1.0 / self.options.render_rate_hz,
                                    time.monotonic())

        snapshots = {
            name: arm.snapshot(self._simulation_time, self._sequence)
            for name, arm in self.arms.items()
        }
        return with_publication_frames(
            snapshots, [arm.config for arm in self.arms.values()],
            ecm_view_pose=(
                _view_pose_from_ecm_tip(snapshots[ecm.config.name].measured_cp_world)
                if ecm is not None else None
            ),
        )

    def is_connected(self) -> bool:
        if self.simulation_app is None or not self._connected:
            return False
        return not self.simulation_app.is_exiting()

    def take_camera_rate_hz(self) -> float:
        if self.camera is None:
            return 0.0
        return self.camera.take_camera_rate_hz()

    def shutdown(self) -> None:
        self._connected = False
        if self.collision_backend is not None:
            self.collision_backend.close()
            self.collision_backend = None
        if self.timeline is not None:
            self.timeline.stop()
            self.timeline = None
        if self.camera is not None:
            self.camera.close()
            self.camera = None
        # Kit can exit the interpreter from close(). The Isaac worker closes
        # the application only after the shared loop has sent its stopped ack.

    def close_application(self, exit_code: int = 0) -> None:
        if self.simulation_app is not None:
            self.simulation_app.close(wait_for_replicator=False, exit_code=exit_code)
            self.simulation_app = None
