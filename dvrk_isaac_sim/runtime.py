"""Isaac Sim runtime and multi-arm simulation loop for dVRK robots."""

from __future__ import annotations

from dataclasses import dataclass
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
        if position is not None:
            xform.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in position]))
        if orientation_xyzw is not None:
            quat = Gf.Quatd(float(orientation_xyzw[3]), Gf.Vec3d(*[float(v) for v in orientation_xyzw[:3]]))
            xform.AddOrientOp().Set(Gf.Quatf(quat))


def _setup_scene_lighting() -> None:
    import omni.usd
    from pxr import Sdf, UsdLux

    stage = omni.usd.get_context().get_stage()
    distant_light = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/DistantLight"))
    distant_light.CreateIntensityAttr(3000.0)
    distant_light.AddRotateXYZOp().Set((-60.0, 30.0, 0.0))
    dome_light = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/DomeLight"))
    dome_light.CreateIntensityAttr(500.0)


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
        super().__init__(config, commands)
        self.jaw_position = self.jaw_setpoint

    def initialize_visual(self) -> None:
        if self.manifest_path is not None and self.manifest_path.is_file():
            import omni.usd

            stage = omni.usd.get_context().get_stage()
            if stage.GetPrimAtPath(f"/World/{self.config.name}").IsValid():
                self.visual = CRTKUSDVisual(self.config.name, self.manifest_path)
                measured = self.model.measured_js()
                self.visual.update(measured.names, measured.position,
                                   self.jaw_position if self.config.type == "PSM" else None)

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

        for robot in self.scene_robots:
            asset_path, manifest_path = _generated_variant(self.generated_root, robot)
            if not asset_path.is_file() or not manifest_path.is_file():
                convert_model(model=robot.name, instrument=robot.instrument or "420006",
                              endoscope=robot.endoscope or "Si_straight",
                              output_dir=self.generated_root, force=True)
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
            arm.initialize_visual()

        if self.options.camera_options is not None and self.options.camera_options.enabled:
            self.camera = IsaacCamera(self.options.camera_options)

        from omni.timeline import get_timeline_interface
        self.timeline = get_timeline_interface()
        self.timeline.set_time_codes_per_second(self.options.simulation_rate_hz)
        self.timeline.set_target_framerate(self.options.render_rate_hz)
        self.timeline.play()
        if self.camera is not None:
            ecm = next((arm for arm in self.arms.values() if arm.config.type == "ECM"), None)
            if ecm is not None:
                self.camera.set_pose(ecm.model.measured_cp())
            self.camera.start_streaming()

        snapshots = {
            name: arm.snapshot(self._simulation_time, self._sequence)
            for name, arm in self.arms.items()
        }
        snapshots = with_publication_frames(
            snapshots, [arm.config for arm in self.arms.values()]
        )
        return snapshots

    def _load_scene_objects(self) -> None:
        """Import exercise objects with their declared fixed/dynamic behavior."""
        from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig
        import omni.usd
        from pxr import UsdPhysics

        if self.scene_config.objects:
            stage = omni.usd.get_context().get_stage()
            UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        for item in self.scene_config.objects:
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
            _reference_usd(asset_path, f"/World/Objects/{item.name}",
                           item.position, item.orientation_xyzw)

    def step(self) -> dict[str, ArmSnapshot]:
        dt = 1.0 / self.options.simulation_rate_hz
        now = time.monotonic()
        self._simulation_time += dt
        self._sequence += 1

        ecm = next((arm for arm in self.arms.values() if arm.config.type == "ECM"), None)
        ecm_pose = ecm.model.measured_cp() if ecm is not None else None
        has_ecm = ecm is not None

        for name, arm in self.arms.items():
            arm.prepare_step(dt, now, ecm_pose, has_ecm=has_ecm)
            self.command_warnings.extend((name, text) for text in arm.command_warnings)
            arm.command_warnings.clear()

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
        snapshots = with_publication_frames(
            snapshots, [arm.config for arm in self.arms.values()]
        )
        return snapshots

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
