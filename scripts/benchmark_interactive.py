#!/usr/bin/env python3
"""Benchmark interactive full-cart control, rendering, and RTSP delivery.

This is intentionally an external ROS 2 client.  It exercises DDS delivery
and the simulator's executor/control/render hand-off rather than calling the
backend in-process.  It is opt-in: run it on the target GPU after building and
sourcing the same ROS environment used to launch Isaac Sim.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from dvrk_arm_description import JointConfig, load_robot_config
from dvrk_isaac_sim.scene import load_scene, load_simulator_config, resolve_scene_path


@dataclass(frozen=True)
class ArmSpec:
    name: str
    joints: tuple[JointConfig, ...]


class BenchmarkMonitor:
    """Native ROS client that drives bounded joint setpoints for every arm."""

    def __init__(self, specs: list[ArmSpec]) -> None:
        import rclpy
        from rclpy.node import Node
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import JointState

        self.rclpy = rclpy
        self.node = Node("dvrk_isaac_sim_interactive_benchmark")
        self._JointState = JointState
        self.specs = {spec.name: spec for spec in specs}
        self.latest: dict[str, np.ndarray] = {}
        self.initial: dict[str, np.ndarray] = {}
        self.max_displacement = {spec.name: 0.0 for spec in specs}
        self.first_motion_latency_s = {spec.name: None for spec in specs}
        self.command_started_at: float | None = None
        self.received = {spec.name: 0 for spec in specs}
        self.sent = {spec.name: 0 for spec in specs}
        self.timestamp_regressions = {spec.name: 0 for spec in specs}
        self._last_stamp: dict[str, int] = {}
        self.clock_messages = 0
        self.clock_regressions = 0
        self._last_clock = -1
        self.publishers = {}
        for spec in specs:
            self.publishers[spec.name] = self.node.create_publisher(
                JointState, f"/{spec.name}/servo_jp", 1
            )
            self.node.create_subscription(
                JointState, f"/{spec.name}/measured_js",
                lambda message, name=spec.name: self._joint_state(name, message), 10,
            )
        self.node.create_subscription(Clock, "/clock", self._clock, 10)

    def _joint_state(self, name: str, message) -> None:
        positions = np.asarray(message.position, dtype=float)
        if positions.shape != (len(self.specs[name].joints),):
            return
        stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        previous = self._last_stamp.get(name)
        if previous is not None and stamp < previous:
            self.timestamp_regressions[name] += 1
        self._last_stamp[name] = stamp
        self.latest[name] = positions
        self.received[name] += 1
        if name not in self.initial:
            self.initial[name] = positions.copy()
        else:
            self.max_displacement[name] = max(
                self.max_displacement[name],
                float(np.max(np.abs(positions - self.initial[name]))),
            )
            if (self.command_started_at is not None
                    and self.first_motion_latency_s[name] is None
                    and self.max_displacement[name] > 1e-5):
                self.first_motion_latency_s[name] = time.monotonic() - self.command_started_at

    def _clock(self, message) -> None:
        stamp = int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec)
        if self._last_clock >= 0 and stamp < self._last_clock:
            self.clock_regressions += 1
        self._last_clock = stamp
        self.clock_messages += 1

    @property
    def ready(self) -> bool:
        return len(self.latest) == len(self.specs) and self.clock_messages > 0

    def publish_commands(self, elapsed: float) -> None:
        """Send small limit-clamped sinusoids, preserving each initial pose."""
        for name, spec in self.specs.items():
            current = self.initial.get(name)
            if current is None:
                continue
            message = self._JointState()
            message.name = [joint.name for joint in spec.joints]
            values = []
            for index, joint in enumerate(spec.joints):
                # 2 mm for prismatic joints and 2 degrees for revolute joints,
                # reduced automatically near a configured limit.
                requested = 0.002 if joint.type == "prismatic" else math.radians(2.0)
                headroom = min(current[index] - joint.lower, joint.upper - current[index])
                amplitude = max(0.0, min(requested, 0.4 * headroom))
                # Start at a non-zero bounded step.  Starting a sine wave at
                # zero measures its intentional ramp, not ROS/control latency.
                values.append(float(current[index] + amplitude * math.cos(2.0 * math.pi * 0.5 * elapsed)))
            message.position = values
            self.publishers[name].publish(message)
            self.sent[name] += 1

    def publish_hold(self) -> None:
        """Warm DDS discovery without changing a robot's current pose."""
        for name, spec in self.specs.items():
            current = self.latest.get(name)
            if current is None:
                continue
            message = self._JointState()
            message.name = [joint.name for joint in spec.joints]
            message.position = [float(value) for value in current]
            self.publishers[name].publish(message)

    def begin_commands(self) -> None:
        """Reset measurement counters after all state publishers are ready."""
        self.initial = {name: position.copy() for name, position in self.latest.items()}
        self.max_displacement = {name: 0.0 for name in self.specs}
        self.first_motion_latency_s = {name: None for name in self.specs}
        self.received = {name: 0 for name in self.specs}
        self.sent = {name: 0 for name in self.specs}
        self.timestamp_regressions = {name: 0 for name in self.specs}
        self._last_stamp = {}
        self.clock_messages = 0
        self.clock_regressions = 0
        self._last_clock = -1
        self.command_started_at = time.monotonic()

    def close(self) -> None:
        self.node.destroy_node()


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "share" / "isaac_sim.yaml")
    parser.add_argument("--scene", default="ECM_PSM1_PSM2_PSM3_stereo_rtsp.yaml")
    parser.add_argument(
        "--renderer",
        choices=("MinimalRendering", "RaytracedLighting", "RealTimePathTracing", "PathTracing"),
        help="override the renderer for this benchmark run",
    )
    parser.add_argument(
        "--camera-scale", type=float, default=1.0,
        help="scale scene camera width and height for this run without changing the source scene",
    )
    parser.add_argument("--duration", type=float, default=30.0, help="active command duration in seconds")
    parser.add_argument("--command-rate-hz", type=float, default=60.0)
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    parser.add_argument("--min-state-rate-hz", type=float, default=30.0)
    parser.add_argument("--max-command-response-ms", type=float, default=500.0)
    parser.add_argument("--min-rtsp-fps", type=float, default=1.0)
    parser.add_argument("--no-rtsp-probe", action="store_true")
    parser.add_argument(
        "--rtsp-decoder", choices=("nvh264dec", "avdec_h264"), default="nvh264dec",
        help="decoder used by the local GStreamer RTSP measurement client",
    )
    parser.add_argument(
        "--disable-rtsp", action="store_true",
        help="remove the RTSP graph from a temporary benchmark scene",
    )
    parser.add_argument("--output", type=Path, help="write JSON metrics to this path")
    args = parser.parse_args()
    if (args.duration <= 0.0 or args.command_rate_hz <= 0.0
            or args.startup_timeout <= 0.0 or args.max_command_response_ms <= 0.0):
        parser.error("duration, command-rate-hz, startup-timeout, and max-command-response-ms must be positive")
    if not 0.0 < args.camera_scale <= 1.0:
        parser.error("camera-scale must be in (0, 1]")
    return args


def _rtsp_command(scene, decoder: str) -> list[str] | None:
    settings = scene.camera.as_dict()
    if "rtsp" not in settings.get("transports", []):
        return None
    rtsp = settings.get("rtsp", {})
    port = int(rtsp.get("port", 8554))
    mount = str(rtsp.get("mount_path", "/ECM"))
    command = [
        "gst-launch-1.0", "-v", "rtspsrc", f"location=rtsp://127.0.0.1:{port}{mount}",
        "protocols=udp", "latency=0", "drop-on-latency=true", "!", "rtph264depay",
        "wait-for-keyframe=true", "!", "h264parse", "!", decoder,
    ]
    if decoder == "nvh264dec":
        command.extend(["max-display-delay=0", "!"])
    else:
        command.append("!")
    return command + [
        "queue", "max-size-buffers=1", "max-size-bytes=0",
        "max-size-time=0", "leaky=downstream", "!", "fpsdisplaysink",
        "video-sink=fakesink", "text-overlay=false", "sync=false", "fps-update-interval=1000",
    ]


def _stop(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5.0)


def _benchmark_scene(scene_path: Path, scene, scale: float,
                     disable_rtsp: bool) -> Path | None:
    """Create a temporary scene for camera/RTSP A/B experiments."""
    if scale == 1.0 and not disable_rtsp:
        return None
    document = yaml.safe_load(scene_path.read_text(encoding="utf-8"))
    camera = document["scene"]["camera"]
    if scale != 1.0:
        for name in ("width", "height"):
            if name not in camera:
                raise ValueError(f"{scene_path}: camera.{name} is required for --camera-scale")
            camera[name] = max(2, int(round(int(camera[name]) * scale)))
    if disable_rtsp:
        camera["transports"] = [
            transport for transport in camera.get("transports", [])
            if transport != "rtsp"
        ]
        camera.pop("rtsp", None)
    # The temporary scene lives outside the package, so make its robot YAML
    # paths absolute before the normal scene resolver loads it.
    for configured, entry in zip(document["scene"].get("robots", []), scene.robots):
        configured["config"] = str(entry.config_path)
    temporary = tempfile.NamedTemporaryFile(
        prefix="dvrk-isaac-scaled-camera-", suffix=".yaml", delete=False, mode="w", encoding="utf-8"
    )
    yaml.safe_dump(document, temporary, sort_keys=False)
    temporary.close()
    return Path(temporary.name)


def main() -> int:
    args = _arguments()
    config_path = args.config.expanduser().resolve()
    simulator = load_simulator_config(config_path)
    scene_path = resolve_scene_path(config_path, args.scene)
    scene = load_scene(scene_path)
    expected = {"ECM", "PSM1", "PSM2", "PSM3"}
    found = {entry.name for entry in scene.robots}
    if found != expected:
        raise ValueError(f"benchmark requires {sorted(expected)}, scene has {sorted(found)}")
    specs = [ArmSpec(entry.name, load_robot_config(entry.config_path).joints) for entry in scene.robots]
    if not simulator.isaac_sim_dir:
        raise ValueError(f"{config_path}: isaac_sim_dir is required")

    temporary_scene = _benchmark_scene(
        scene_path, scene, args.camera_scale, args.disable_rtsp
    )
    launch_scene_path = temporary_scene or scene_path
    launch = [
        "ros2", "launch", "dvrk_isaac_sim", "simulator.launch.py",
        f"config:={config_path}", f"scene:={launch_scene_path}",
    ]
    print("Launching: " + " ".join(str(item) for item in launch), flush=True)
    launch_log = tempfile.NamedTemporaryFile(prefix="dvrk-isaac-benchmark-", suffix=".log", delete=False)
    launch_environment = os.environ.copy()
    launch_environment["DVRK_SIMULATOR_FORCE_HEADLESS"] = "true"
    if args.renderer:
        launch_environment["DVRK_SIMULATOR_RENDERER"] = args.renderer
    launch_process = subprocess.Popen(
        launch, stdout=launch_log, stderr=subprocess.STDOUT, text=True,
        env=launch_environment,
    )
    monitor = None
    rtsp_process = None
    rtsp_log = None
    rtsp_output = ""
    result: dict[str, object] = {
        "scene": scene.name, "renderer": args.renderer or simulator.renderer,
        "camera_scale": args.camera_scale, "rtsp_enabled": not args.disable_rtsp,
        "success": False,
    }
    try:
        import rclpy
        rclpy.init()
        monitor = BenchmarkMonitor(specs)
        deadline = time.monotonic() + args.startup_timeout
        while time.monotonic() < deadline and not monitor.ready:
            if launch_process.poll() is not None:
                raise RuntimeError(f"simulator exited during startup; log: {launch_log.name}")
            rclpy.spin_once(monitor.node, timeout_sec=0.1)
        if not monitor.ready:
            raise TimeoutError(f"timed out waiting for all state topics; log: {launch_log.name}")
        # Fast DDS may report zero subscription matches briefly even while a
        # writer is usable.  Warm it with harmless hold commands instead of
        # making its discovery accounting part of the benchmark result.
        warmup_deadline = time.monotonic() + 1.0
        while time.monotonic() < warmup_deadline:
            monitor.publish_hold()
            rclpy.spin_once(monitor.node, timeout_sec=0.01)

        rtsp_command = (
            None if args.no_rtsp_probe or args.disable_rtsp
            else _rtsp_command(scene, args.rtsp_decoder)
        )
        if rtsp_command is not None:
            # Do not leave GStreamer on a bounded PIPE: verbose startup and
            # diagnostics could fill it and make a healthy stream appear slow.
            rtsp_log = tempfile.NamedTemporaryFile(
                prefix="dvrk-isaac-rtsp-", suffix=".log", delete=False
            )
            rtsp_process = subprocess.Popen(
                rtsp_command, stdout=rtsp_log, stderr=subprocess.STDOUT, text=True
            )
        monitor.begin_commands()
        command_start = monitor.command_started_at
        assert command_start is not None
        next_command = command_start
        while time.monotonic() - command_start < args.duration:
            if launch_process.poll() is not None:
                raise RuntimeError(f"simulator exited during benchmark; log: {launch_log.name}")
            now = time.monotonic()
            if now >= next_command:
                monitor.publish_commands(now - command_start)
                next_command += 1.0 / args.command_rate_hz
            rclpy.spin_once(monitor.node, timeout_sec=min(0.01, max(0.0, next_command - time.monotonic())))

        elapsed = time.monotonic() - command_start
        state_rates = {name: count / elapsed for name, count in monitor.received.items()}
        rtsp_alive = rtsp_process is None or rtsp_process.poll() is None
        if rtsp_process is not None:
            _stop(rtsp_process)
            rtsp_log.close()
            rtsp_output = Path(rtsp_log.name).read_text(encoding="utf-8", errors="replace")
        # fpsdisplaysink reports a running ``average`` after each interval.
        # Use its final value for a stable regression metric, not the maximum
        # instantaneous ``current`` rate.
        average_fps_values = [
            float(value) for value in re.findall(r"average:\s*([0-9.]+)", rtsp_output)
        ]
        current_fps_values = [
            float(value) for value in re.findall(r"(?:fps|current):\s*([0-9.]+)", rtsp_output)
        ]
        rtsp_fps = average_fps_values[-1] if average_fps_values else max(current_fps_values, default=0.0)
        failures = []
        for name in sorted(monitor.specs):
            if monitor.sent[name] == 0 or monitor.max_displacement[name] <= 1e-5:
                failures.append(f"{name} did not respond to servo_jp")
            if state_rates[name] < args.min_state_rate_hz:
                failures.append(f"{name} state rate {state_rates[name]:.1f} Hz is below {args.min_state_rate_hz:.1f} Hz")
            if monitor.timestamp_regressions[name]:
                failures.append(f"{name} has timestamp regressions")
            latency_s = monitor.first_motion_latency_s[name]
            if latency_s is None or latency_s * 1000.0 > args.max_command_response_ms:
                response_text = "not observed" if latency_s is None else f"{1000.0 * latency_s:.1f} ms"
                failures.append(
                    f"{name} command response {response_text} "
                    f"exceeds {args.max_command_response_ms:.1f} ms"
                )
        if monitor.clock_messages == 0 or monitor.clock_regressions:
            failures.append("/clock did not advance monotonically")
        if rtsp_process is not None and (not rtsp_alive or rtsp_fps < args.min_rtsp_fps):
            failures.append(f"RTSP decoded {rtsp_fps:.1f} fps, expected at least {args.min_rtsp_fps:.1f}")
        result.update({
            "duration_s": elapsed, "state_rate_hz": state_rates, "commands_sent": monitor.sent,
            "max_joint_displacement": monitor.max_displacement,
            "first_motion_latency_ms": {
                name: None if value is None else 1000.0 * value
                for name, value in monitor.first_motion_latency_s.items()
            },
            "timestamp_regressions": monitor.timestamp_regressions,
            "clock_messages": monitor.clock_messages, "clock_regressions": monitor.clock_regressions,
            "command_subscription_matches": {
                name: publisher.get_subscription_count()
                for name, publisher in monitor.publishers.items()
            },
            "rtsp_fps": rtsp_fps if not args.no_rtsp_probe and not args.disable_rtsp else None,
            "rtsp_decoder": args.rtsp_decoder if not args.no_rtsp_probe and not args.disable_rtsp else None,
            "rtsp_log": rtsp_log.name if rtsp_log is not None else None,
            "launch_log": launch_log.name, "failures": failures, "success": not failures,
        })
    finally:
        _stop(rtsp_process)
        if rtsp_log is not None and not rtsp_log.closed:
            rtsp_log.close()
        if monitor is not None:
            monitor.close()
        if "rclpy" in locals() and rclpy.ok():
            rclpy.shutdown()
        _stop(launch_process)
        launch_log.close()
        if temporary_scene is not None:
            temporary_scene.unlink(missing_ok=True)
    report = json.dumps(result, indent=2, sort_keys=True)
    print(report, flush=True)
    if args.output:
        args.output.expanduser().write_text(report + "\n", encoding="utf-8")
    return 0 if result.get("success") else 1


if __name__ == "__main__":
    raise SystemExit(main())
