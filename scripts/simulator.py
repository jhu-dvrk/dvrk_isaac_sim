#!/usr/bin/env python3
"""Isaac Sim 6.0 ROS 2/CRTK simulator and integration test.

This script intentionally uses no USD robot assets yet. It validates that:

* Isaac Sim starts with the ROS 2 bridge;
* the sourced ROS 2 Python environment is visible inside Isaac Sim;
* crtk_msgs custom messages can be imported;
* PSM1 and ECM publish CRTK topics using simulation time; and
* joint and Cartesian commands are delivered to the kinematic models.

Run with Isaac Sim's Python interpreter, not the system Python. Runtime settings are
loaded from config; use --config to select a saved YAML file.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import sys
import threading
import time
from typing import Any

import numpy as np

# Permit running the source script directly with Isaac Sim's Python.
_package_root = Path(__file__).resolve().parents[1]
if str(_package_root) not in sys.path:
    sys.path.insert(0, str(_package_root))
from _isaac_sim_build import require_isaac_sim_build

require_isaac_sim_build(__file__)
from dvrk_isaac_sim.scene import load_scene, load_simulator_config, resolve_scene_path, SceneRobot


def _arguments() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=root / "share/isaac_sim.yaml",
                        help="YAML file containing simulator settings")
    parser.add_argument("--headless", action="store_true", default=None,
                        help="override config and run without an Isaac Sim window")
    parser.add_argument("--duration", type=float, default=None,
                        help="override config simulation duration in seconds")
    parser.add_argument("--scene", type=Path, default=None,
                        help="override config and select a scene YAML")
    parser.add_argument(
        "--renderer",
        choices=("MinimalRendering", "RaytracedLighting", "RealTimePathTracing", "PathTracing"),
        default=None,
        help="override the renderer selected by the simulator config",
    )
    args = parser.parse_args()

    config_path = args.config.expanduser().resolve()
    simulator_config = load_simulator_config(config_path)
    args.config = config_path
    args.simulator_config = simulator_config
    args.scene_config = resolve_scene_path(
        config_path, args.scene if args.scene is not None else simulator_config.scene
    )
    args.scene_model = load_scene(args.scene_config)
    args.generated_dir = simulator_config.generated_dir
    args.renderer = simulator_config.renderer if args.renderer is None else args.renderer
    args.headless = simulator_config.headless if args.headless is None else args.headless
    args.duration = simulator_config.duration if args.duration is None else args.duration
    if os.environ.get("DVRK_SIMULATOR_FORCE_HEADLESS") == "true":
        args.headless = True
    if renderer := os.environ.get("DVRK_SIMULATOR_RENDERER"):
        args.renderer = renderer
    test_timeout = os.environ.get("DVRK_SIMULATOR_TEST_TIMEOUT")
    args.run_crtk_integration_test = test_timeout is not None
    if test_timeout is not None:
        args.headless = True
        args.duration = float(test_timeout)
    args.simulation_rate_hz = simulator_config.simulation_rate_hz
    args.render_rate_hz = simulator_config.render_rate_hz
    args.scene_camera = args.scene_model.camera.as_dict()
    args.camera = args.scene_model.camera.mode
    if args.renderer not in {
        "MinimalRendering", "RaytracedLighting", "RealTimePathTracing", "PathTracing"
    }:
        raise ValueError(f"{config_path}: unsupported renderer {args.renderer}")
    return args


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
            xform.AddTranslateOp(opSuffix="base").Set(Gf.Vec3d(*position))
        if orientation_xyzw is not None:
            x, y, z, w = orientation_xyzw
            # AddOrientOp currently causes a native USD shutdown in Isaac Sim
            # 6.0 when appended to imported robot xform stacks.  Author the
            # equivalent XYZ Euler op in degrees instead.
            roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
            pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
            yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            xform.AddRotateXYZOp(opSuffix="base").Set(
                Gf.Vec3d(*np.degrees([roll, pitch, yaw])))
    print(f"Referenced USD asset {path} at {prim_path}", flush=True)


def _setup_scene_lighting() -> None:
    """Add neutral local lighting so dark instruments remain visible."""
    import omni.usd
    from pxr import Gf, UsdLux, UsdGeom

    stage = omni.usd.get_context().get_stage()
    dome = UsdLux.DomeLight.Define(stage, "/World/Lighting/DomeLight")
    dome.CreateColorAttr(Gf.Vec3f(0.22, 0.22, 0.22))
    dome.CreateIntensityAttr(450.0)
    key = UsdLux.DistantLight.Define(stage, "/World/Lighting/KeyLight")
    key.CreateColorAttr(Gf.Vec3f(1.0, 0.96, 0.90))
    key.CreateIntensityAttr(1800.0)
    key.CreateAngleAttr(0.5)
    UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3d(35.0, -25.0, -30.0))


def _configure_fixed_timestep(timeline, simulation_rate_hz: float,
                              render_rate_hz: float) -> None:
    """Configure independent simulation time codes and render pacing."""
    # These timeline controls are available in Isaac Sim 6.0.  Keep the
    # guarded form so configuration-only tooling can still import this file.
    timeline.set_time_codes_per_second(float(simulation_rate_hz))
    if hasattr(timeline, "set_target_framerate"):
        timeline.set_target_framerate(float(render_rate_hz))


def _ros_time(seconds: float):
    from builtin_interfaces.msg import Time

    seconds = max(0.0, float(seconds))
    result = Time()
    result.sec = int(seconds)
    result.nanosec = int(round((seconds - result.sec) * 1e9))
    if result.nanosec >= 1_000_000_000:
        result.sec += 1
        result.nanosec -= 1_000_000_000
    return result


@dataclass(frozen=True)
class _RenderState:
    joint_names: tuple[str, ...]
    joint_position: np.ndarray
    jaw_position: float | None
    camera_pose: Any | None


@dataclass(frozen=True)
class _ControlSnapshot:
    simulation_time: float
    captured_at: float
    states: dict[str, _RenderState]
    command_metrics: dict[str, tuple[int, int, int, int, float | None]]


class _ControlLoop:
    """Run ROS callbacks and kinematics independently of blocking rendering."""

    def __init__(self, executor, ordered_nodes, clock_publisher, clock_type,
                 rate_hz: float, initial_time: float,
                 component_lock: threading.RLock | None = None) -> None:
        self._executor = executor
        self._ordered_nodes = ordered_nodes
        self._clock_publisher = clock_publisher
        self._clock_type = clock_type
        self._period = 1.0 / rate_hz
        self._stop = threading.Event()
        self._state_lock = threading.Lock()
        self.component_lock = component_lock or threading.RLock()
        self._playing = True
        self._simulation_time = initial_time
        self._updates = 0
        self._steps = 0
        self._ros_spins = 0
        self._exception: BaseException | None = None
        self._snapshot = self._capture_snapshot()
        self._ros_thread = threading.Thread(
            target=self._spin_ros, name="dvrk-isaac-ros", daemon=True
        )
        self._thread = threading.Thread(
            target=self._run, name="dvrk-isaac-control", daemon=True
        )

    def _capture_snapshot(self) -> _ControlSnapshot:
        states = {}
        command_metrics = {}
        for _, component, visual, camera in self._ordered_nodes:
            measured = component.model.measured_js()
            states[component.config.name] = _RenderState(
                measured.names,
                measured.position,
                component.jaw_position,
                component.model.measured_cp() if camera is not None else None,
            )
            command_metrics[component.config.name] = component.command_metrics()
        return _ControlSnapshot(
            self._simulation_time, time.monotonic(), states, command_metrics
        )

    def start(self) -> None:
        self._ros_thread.start()
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._ros_thread.is_alive():
            self._ros_thread.join(timeout=5.0)
        if self._thread.is_alive():
            self._thread.join(timeout=5.0)

    def set_playing(self, playing: bool) -> None:
        with self._state_lock:
            self._playing = bool(playing)

    def snapshot(self) -> _ControlSnapshot:
        with self._state_lock:
            return self._snapshot

    def metrics(self) -> tuple[int, int, int, float]:
        with self._state_lock:
            return self._updates, self._steps, self._ros_spins, self._simulation_time

    def raise_if_failed(self) -> None:
        with self._state_lock:
            error = self._exception
        if error is not None:
            raise RuntimeError("CRTK control loop failed") from error

    def _spin_ros(self) -> None:
        """Service ROS independently of both control and render cadence."""
        try:
            while not self._stop.is_set():
                self._executor.spin_once(timeout_sec=0.01)
                with self._state_lock:
                    self._ros_spins += 1
        except BaseException as error:
            with self._state_lock:
                self._exception = error
            self._stop.set()

    def _run(self) -> None:
        next_step = time.monotonic()
        try:
            while not self._stop.is_set():
                now = time.monotonic()
                wait = next_step - now
                if wait > 0.0:
                    self._stop.wait(wait)
                    continue

                # If Python or the OS delayed this thread, advance enough fixed
                # steps to keep simulation time aligned with wall time. A single
                # larger model step is equivalent for the velocity-limited
                # kinematic interpolation and avoids a catch-up CPU spiral.
                step_count = 1 + int(max(0.0, now - next_step) / self._period)
                next_step += step_count * self._period

                with self._state_lock:
                    playing = self._playing
                    if playing:
                        self._simulation_time += step_count * self._period
                    current_time = self._simulation_time

                stamp = _ros_time(current_time)
                with self.component_lock:
                    for _, component, _, _ in self._ordered_nodes:
                        component.set_simulation_stamp(stamp)

                    if playing:
                        clock = self._clock_type()
                        clock.clock = stamp
                        self._clock_publisher.publish(clock)

                    dt = step_count * self._period if playing else 0.0
                    for _, component, _, _ in self._ordered_nodes:
                        component.process_pending_commands()
                        component.model.step(dt)
                        component.publish(stamp, valid=playing)

                    snapshot = self._capture_snapshot()

                with self._state_lock:
                    self._snapshot = snapshot
                    self._updates += 1
                    if playing:
                        self._steps += step_count
        except BaseException as error:
            with self._state_lock:
                self._exception = error
            self._stop.set()


def _generated_variant(args: argparse.Namespace, entry: SceneRobot) -> tuple[Path, Path]:
    variant = (entry.instrument or "420006" if entry.type == "PSM"
               else entry.endoscope or "Si_straight")
    asset_dir = args.generated_dir.expanduser().resolve() / f"{entry.name}_{variant}"
    return asset_dir / entry.name / f"{entry.name}.usda", asset_dir / "kinematics.json"


def main() -> int:
    args = _arguments()

    # Isaac Sim must be initialized before importing most Isaac modules.
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({
        "headless": args.headless,
        "renderer": args.renderer,
        # Textured diffuse plus FXAA preserves the instrument materials while
        # leaving GPU headroom for WiVRN's compositor and second encoder.
        "minimal_shading_mode": 2,
        "anti_aliasing": 2 if args.renderer == "MinimalRendering" else 3,
        # This process currently renders on one selected GPU. Avoid enabling
        # the multi-GPU path on single-GPU workstations and laptops.
        "multi_gpu": False,
        # Headless camera/RTSP runs use explicit render products. Updating the
        # otherwise invisible 1280x720 editor viewport wastes GPU time and can
        # make the camera miss its wall-clock target.
        "disable_viewport_updates": args.headless,
    })
    nodes = []
    executor = None
    control_loop = None
    try:
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.ros2.bridge")
        enable_extension("isaacsim.ros2.nodes")
        if "rtsp" in args.scene_camera.get("transports", ["ros_raw"]):
            enable_extension("isaacsim.streaming.rtsp")
        simulation_app.update()
        scene_entries = args.scene_model.robots
        if scene_entries:
            for entry in scene_entries:
                asset, _ = _generated_variant(args, entry)
                robot_type = entry.type
                if robot_type == "PSM":
                    frame = entry.frame
                    _reference_usd(asset, f"/World/{entry.name}",
                                   frame.get("position"), frame.get("orientation_xyzw"))
        # The ECM is intentionally represented by kinematics and its camera
        # only; never add the endoscope/ECM mesh to the stage.

        _setup_scene_lighting()

        import rclpy
        from rclpy.node import Node
        from crtk_msgs.msg import OperatingState
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
        from rosgraph_msgs.msg import Clock
        from omni.timeline import get_timeline_interface
        from pxr import Sdf

        # This import is intentional: it is the custom-message preflight check.
        print(f"Loaded custom ROS 2 message: {OperatingState.__module__}.OperatingState", flush=True)

        package_root = Path(__file__).resolve().parents[1]
        if str(package_root) not in sys.path:
            sys.path.insert(0, str(package_root))

        from dvrk_arm_description import load_robot_config
        from dvrk_isaac_sim.kinematics import CRTKECM, CRTKPSM
        from dvrk_isaac_sim.ros_interface import CRTKROSComponent
        from dvrk_isaac_sim.usd_visual import CRTKUSDVisual

        rclpy.init()
        from rclpy.executors import SingleThreadedExecutor
        executor = SingleThreadedExecutor()

        cameras = []
        component_lock = threading.RLock()

        def add_component(namespace: str, config_path: Path, frame: dict | None = None,
                          manifest: Path | None = None, instrument: str | None = None):
            frame = frame or {}
            config = load_robot_config(
                config_path,
                base_position=frame.get("position"),
                base_orientation_xyzw=frame.get("orientation_xyzw"),
                instrument=instrument,
            )
            if config.type == "PSM":
                model = CRTKPSM(config, kinematics_manifest=manifest)
            elif config.type == "ECM":
                model = CRTKECM(config, kinematics_manifest=manifest)
            else:
                raise ValueError(f"Unsupported robot type: {config.type}")
            node = Node(f"dvrk_isaac_sim_{config.name}", namespace=f"/{namespace}")
            executor.add_node(node)
            component = CRTKROSComponent(
                node, config, model, _ros_time(1.0 / args.simulation_rate_hz),
                component_lock,
            )
            import omni.usd

            stage = omni.usd.get_context().get_stage()
            visual = (
                CRTKUSDVisual(config.name, manifest)
                if stage.GetPrimAtPath(f"/World/{config.name}").IsValid()
                else None
            )
            camera = None
            if config.type == "ECM" and args.camera != "off":
                from dvrk_isaac_sim.camera import IsaacCameraPublisher
                camera = IsaacCameraPublisher(node, config, args.camera, args.scene_camera)
                cameras.append(camera)
            nodes.append((node, component, visual, camera))

        if scene_entries:
            for entry in scene_entries:
                asset, manifest = _generated_variant(args, entry)
                add_component(entry.name, entry.config_path, entry.frame, manifest, entry.instrument)

        # PSM Cartesian CRTK topics are expressed in the moving ECM optical
        # frame, as on a dVRK system. Wire this after all components exist
        # because scene YAML may list the PSMs before the ECM.
        ecm_component = next((component for _, component, _, _ in nodes
                              if component.config.type == "ECM"), None)
        if ecm_component is not None:
            # PSM Cartesian topics use dVRK view axes, derived from the
            # current ECM optical FK rather than raw optical-camera axes.
            ecm_view_frame = f"{ecm_component.config.name}_view"
            for _, component, _, _ in nodes:
                if component.config.type == "PSM":
                    component.set_cartesian_reference(ecm_component.model, ecm_view_frame)

        # Let Fabric Scene Delegate mirror the referenced robot hierarchy and
        # the identity-valued CRTK xform operations once.  Subsequent joint
        # motion writes runtime local matrices directly to Fabric instead of
        # repeatedly invalidating the USD stage and Hydra scene.
        simulation_app.update()
        fabric_visual = None
        for _, _, visual, _ in nodes:
            if visual is not None:
                visual.enable_fabric()
                if fabric_visual is None:
                    fabric_visual = visual
        if fabric_visual is not None:
            print("Robot visual transforms: Isaac Fabric runtime updates enabled", flush=True)

        # Start from the importer-authored zero-joint pose.  The configured
        # home state is applied below through move_jp, once the USD/Fabric
        # hierarchy and Isaac timeline are ready.
        for _, component, _, _ in nodes:
            component.model.prepare_startup_move()
        with Sdf.ChangeBlock():
            for _, component, visual, _ in nodes:
                if visual is not None:
                    measured = component.model.measured_js()
                    visual.update(
                        measured.names, measured.position,
                        component.jaw_position,
                    )
        if fabric_visual is not None:
            fabric_visual.flush()

        # Advance ECM first so every PSM reads the current, not previous-step,
        # camera pose when converting Cartesian state and commands.
        ordered_nodes = sorted(
            nodes, key=lambda item: 0 if item[1].config.type == "ECM" else 1
        )

        timeline = get_timeline_interface()
        _configure_fixed_timestep(
            timeline, args.simulation_rate_hz, args.render_rate_hz
        )
        timeline.play()
        # Route the startup pose through the same move path used by ROS clients.
        # and ROS move_jp callbacks.  Besides publishing the normal busy edge,
        # this makes startup initialization observable to CRTK clients rather
        # than silently changing only the internal model state.
        with component_lock:
            for _, component, _, _ in nodes:
                if not component.command_joint_position(component.config.home_position):
                    raise RuntimeError(
                        f"{component.config.name}: failed to apply startup move_jp"
                    )
                print(
                    f"{component.config.name}: startup move_jp -> "
                    f"{component.config.home_position.tolist()}",
                    flush=True,
                )
        for _, component, _, _ in nodes:
            component.publish_tool_type()
        clock_publisher = nodes[0][0].create_publisher(Clock, "/clock", 10)
        diagnostics_publisher = nodes[0][0].create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )

        def publish_runtime_diagnostics(
            simulation_time: float,
            control_hz: float,
            simulation_hz: float,
            render_hz: float,
            camera_hz: float,
            real_time_factor: float,
            state_age_ms: float,
        ) -> None:
            status = DiagnosticStatus()
            status.name = "dvrk_isaac_sim/runtime"
            status.hardware_id = "dvrk_isaac_sim"
            status.level = (
                DiagnosticStatus.OK
                if control_hz >= 0.9 * args.simulation_rate_hz else DiagnosticStatus.WARN
            )
            status.message = "running" if status.level == DiagnosticStatus.OK else "control rate below target"
            status.values = [
                KeyValue(key="control_hz", value=f"{control_hz:.1f}"),
                KeyValue(key="simulation_hz", value=f"{simulation_hz:.1f}"),
                KeyValue(key="render_hz", value=f"{render_hz:.1f}"),
                KeyValue(key="camera_hz", value=f"{camera_hz:.1f}"),
                KeyValue(key="real_time_factor", value=f"{real_time_factor:.3f}"),
                KeyValue(key="state_age_ms", value=f"{state_age_ms:.1f}"),
                KeyValue(key="arms", value=str(len(nodes))),
            ]
            message = DiagnosticArray()
            message.header.stamp = _ros_time(simulation_time)
            message.status = [status]
            diagnostics_publisher.publish(message)
        if args.run_crtk_integration_test:
            print("Isaac Sim CRTK integration test running", flush=True)
            for entry in scene_entries:
                print(f"  {entry.name} topics: /{entry.name}/measured_js, /{entry.name}/measured_cp, /{entry.name}/servo_jp", flush=True)

        initial_time = max(0.0, float(timeline.get_current_time()))
        control_loop = _ControlLoop(
            executor, ordered_nodes, clock_publisher, Clock,
            args.simulation_rate_hz, initial_time, component_lock,
        )
        control_loop.start()

        render_period = 1.0 / args.render_rate_hz
        next_render = time.monotonic()
        last_status = next_render
        last_updates = 0
        last_steps = 0
        last_ros_spins = 0
        last_simulation_time = initial_time
        rendered_frames = 0
        active_camera_frames = 0
        render_time_total = 0.0
        render_time_max = 0.0
        scene_update_time_total = 0.0
        scene_update_time_max = 0.0
        control_age_before_total = 0.0
        control_age_after_total = 0.0

        while simulation_app.is_running():
            control_loop.raise_if_failed()
            now = time.monotonic()
            if now < next_render:
                time.sleep(next_render - now)

            playing = bool(timeline.is_playing())
            control_loop.set_playing(playing)
            snapshot = control_loop.snapshot()
            current_time = snapshot.simulation_time
            snapshots = snapshot.states
            timeline.set_current_time(current_time)

            # Apply only the latest control state before rendering. Batch all
            # robot transform edits into one USD change notice; emitting a
            # Hydra invalidation for every individual joint makes a moving
            # three-PSM scene dramatically slower than a stationary scene.
            scene_update_start = time.monotonic()
            with Sdf.ChangeBlock():
                for _, component, visual, _ in ordered_nodes:
                    state = snapshots.get(component.config.name)
                    if state is not None and visual is not None:
                        visual.update(
                            state.joint_names, state.joint_position,
                            state.jaw_position,
                        )
            if fabric_visual is not None:
                fabric_visual.flush()

            # Camera helpers may query the stage while setting their poses, so
            # keep them outside the Sdf change block.
            for _, component, _, camera in ordered_nodes:
                state = snapshots.get(component.config.name)
                if state is None:
                    continue
                if camera is not None and playing and state.camera_pose is not None:
                    camera.set_pose(state.camera_pose)
            scene_update_elapsed = time.monotonic() - scene_update_start
            scene_update_time_total += scene_update_elapsed
            scene_update_time_max = max(
                scene_update_time_max, scene_update_elapsed
            )

            render_start = time.monotonic()
            control_age_before_total += render_start - snapshot.captured_at
            simulation_app.update()
            render_elapsed = time.monotonic() - render_start
            control_age_after_total += time.monotonic() - snapshot.captured_at
            rendered_frames += 1
            render_time_total += render_elapsed
            render_time_max = max(render_time_max, render_elapsed)

            # ROS image publication is subscriber-gated. The RTSP writer is
            # attached once and captures this wall-clock-paced render product.
            for _, _, _, camera in ordered_nodes:
                if camera is not None and playing:
                    camera.publish(current_time)
                    active_camera_frames += 1
            render_end = time.monotonic()
            next_render += render_period
            if next_render < render_end:
                next_render = render_end

            if render_end - last_status >= 1.0:
                elapsed = render_end - last_status
                updates, steps, ros_spins, simulation_time = control_loop.metrics()
                update_hz = (updates - last_updates) / elapsed
                step_hz = (steps - last_steps) / elapsed
                ros_hz = (ros_spins - last_ros_spins) / elapsed
                render_hz = rendered_frames / elapsed
                camera_hz = active_camera_frames / elapsed
                real_time_factor = (
                    (simulation_time - last_simulation_time) / elapsed
                )
                average_render_ms = (
                    1000.0 * render_time_total / rendered_frames
                    if rendered_frames else 0.0
                )
                average_control_age_before_ms = (
                    1000.0 * control_age_before_total / rendered_frames
                    if rendered_frames else 0.0
                )
                average_control_age_after_ms = (
                    1000.0 * control_age_after_total / rendered_frames
                    if rendered_frames else 0.0
                )
                command_text = "; ".join(
                    f"{name}=rx:{received}/apply:{applied}/"
                    f"coalesce:{coalesced}/reject:{rejected}/"
                    f"age:{age_ms:.1f}ms"
                    for name, (
                        received, applied, coalesced, rejected, age_ms
                    ) in snapshot.command_metrics.items()
                    if received and age_ms is not None
                )
                publish_runtime_diagnostics(
                    simulation_time, update_hz, step_hz, render_hz, camera_hz,
                    real_time_factor, average_control_age_after_ms,
                )
                print(
                    "Simulator performance: "
                    f"real-time-factor={real_time_factor:.3f}; "
                    f"control={update_hz:.1f} Hz; "
                    f"simulation-steps={step_hz:.1f} Hz; "
                    f"ros={ros_hz:.1f} spin/s; "
                    f"render={render_hz:.1f} Hz; "
                    f"camera={camera_hz:.1f} Hz; "
                    f"scene-update={1000.0 * scene_update_time_total / rendered_frames:.1f} ms avg/"
                    f"{1000.0 * scene_update_time_max:.1f} ms max; "
                    f"render-time={average_render_ms:.1f} ms avg/"
                    f"{1000.0 * render_time_max:.1f} ms max; "
                    f"control-state-age={average_control_age_before_ms:.1f} ms submit/"
                    f"{average_control_age_after_ms:.1f} ms rendered"
                    + (f"; commands[{command_text}]" if command_text else ""),
                    flush=True,
                )
                last_status = render_end
                last_updates = updates
                last_steps = steps
                last_ros_spins = ros_spins
                last_simulation_time = simulation_time
                rendered_frames = 0
                active_camera_frames = 0
                render_time_total = 0.0
                render_time_max = 0.0
                scene_update_time_total = 0.0
                scene_update_time_max = 0.0
                control_age_before_total = 0.0
                control_age_after_total = 0.0

            if args.duration > 0.0 and playing and current_time >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    except Exception:
        # Isaac Sim can close its window immediately after a Python exception.
        # Print the full traceback before cleanup so asset/conversion failures
        # remain actionable from the launch console.
        import traceback
        traceback.print_exc()
        raise
    finally:
        if control_loop is not None:
            control_loop.stop()
        if executor is not None:
            executor.shutdown()
        if "rclpy" in locals() and rclpy.ok():
            for node, _, _, _ in nodes:
                node.destroy_node()
            rclpy.shutdown()
        simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
