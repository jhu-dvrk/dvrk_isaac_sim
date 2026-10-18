"""Runtime coordination objects shared by the Isaac simulator entry point."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Any, Callable

import numpy as np


@dataclass(frozen=True)
class RenderState:
    joint_names: tuple[str, ...]
    joint_position: np.ndarray
    jaw_position: float | None
    camera_pose: Any | None


@dataclass(frozen=True)
class ControlSnapshot:
    simulation_time: float
    captured_at: float
    states: dict[str, RenderState]
    command_metrics: dict[str, tuple[int, int, int, int, float | None]]


@dataclass
class ComponentRuntime:
    node: Any
    component: Any
    visual: Any | None = None
    camera: Any | None = None
    jaw_controller: Any | None = None

    @property
    def name(self) -> str:
        return self.component.config.name

    @property
    def type(self) -> str:
        return self.component.config.type

    def control_step(self, dt: float, stamp, playing: bool) -> None:
        self.component.set_simulation_stamp(stamp)
        self.component.process_pending_commands()
        if self.jaw_controller is not None:
            self.jaw_controller.step(dt)
        else:
            self.component.step_jaw(dt)
        self.component.model.step(dt)
        self.component.publish(stamp, valid=playing)

    def capture_state(self) -> RenderState:
        measured = self.component.model.measured_js()
        return RenderState(
            measured.names,
            measured.position,
            self.component.jaw_position,
            self.component.model.measured_cp() if self.camera is not None else None,
        )

    def command_metrics(self) -> tuple[int, int, int, int, float | None]:
        return self.component.command_metrics()

    def prepare_startup_move(self) -> None:
        self.component.model.prepare_startup_move()

    def command_home_position(self) -> bool:
        return self.component.command_joint_position(self.component.config.home_position)

    def publish_tool_type(self) -> None:
        self.component.publish_tool_type()

    def apply_state(self, state: RenderState | None, physics_links: dict[str, Any]) -> None:
        if state is None:
            return
        if self.visual is not None:
            self.visual.update(state.joint_names, state.joint_position, state.jaw_position)
        physics = physics_links.get(self.name)
        if physics is not None:
            physics.update(state.joint_names, state.joint_position, state.jaw_position)

    def apply_current_state(self, physics_links: dict[str, Any]) -> None:
        self.apply_state(self.capture_state(), physics_links)

    def update_camera_pose(self, state: RenderState | None, playing: bool) -> None:
        if self.camera is not None and playing and state is not None and state.camera_pose is not None:
            self.camera.set_pose(state.camera_pose)


class ControlLoop:
    """Run ROS callbacks and component controllers independently of rendering."""

    def __init__(
        self,
        executor,
        runtimes: list[ComponentRuntime],
        clock_publisher,
        clock_type,
        rate_hz: float,
        initial_time: float,
        ros_time: Callable[[float], Any],
        component_lock: threading.RLock | None = None,
    ) -> None:
        self._executor = executor
        self._runtimes = runtimes
        self._clock_publisher = clock_publisher
        self._clock_type = clock_type
        self._period = 1.0 / rate_hz
        self._ros_time = ros_time
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

    def _capture_snapshot(self) -> ControlSnapshot:
        states = {runtime.name: runtime.capture_state() for runtime in self._runtimes}
        command_metrics = {
            runtime.name: runtime.command_metrics() for runtime in self._runtimes
        }
        return ControlSnapshot(
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

    def snapshot(self) -> ControlSnapshot:
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

                step_count = 1 + int(max(0.0, now - next_step) / self._period)
                next_step += step_count * self._period

                with self._state_lock:
                    playing = self._playing
                    if playing:
                        self._simulation_time += step_count * self._period
                    current_time = self._simulation_time

                stamp = self._ros_time(current_time)
                with self.component_lock:
                    if playing:
                        clock = self._clock_type()
                        clock.clock = stamp
                        self._clock_publisher.publish(clock)

                    dt = step_count * self._period if playing else 0.0
                    for runtime in self._runtimes:
                        runtime.control_step(dt, stamp, playing)

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
