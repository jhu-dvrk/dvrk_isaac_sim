"""Isaac Sim GUI for monitoring and commanding virtual CRTK arms."""

from __future__ import annotations

import math
import threading
from typing import Any

import numpy as np

from .kinematics import Pose


def _rpy_from_matrix(rotation: np.ndarray) -> tuple[float, float, float]:
    """Extract roll, pitch, yaw using the simulator's XYZ convention."""
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
        yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    else:
        roll = 0.0
        yaw = math.atan2(float(-rotation[0, 1]), float(rotation[1, 1]))
    return roll, pitch, yaw


class IsaacCRTKWindow:
    """Live CRTK monitor and joint/state command window.

    This module is imported only for non-headless Isaac Sim runs.  It uses
    display units of degrees for revolute joints and millimetres for insertion
    joints, while commands sent to the model remain radians/metres.
    """

    _STATE_COMMANDS = ("enable", "disable", "pause", "resume", "home", "unhome", "fault", "clear_fault")
    _STATE_COMMAND_PLACEHOLDER = "..."

    def __init__(self, components: list[Any],
                 component_lock: threading.RLock | None = None) -> None:
        import omni.ui as ui

        self._ui = ui
        self._components = components
        self._component_lock = component_lock or threading.RLock()
        self._fields: dict[str, list[Any]] = {}
        self._dirty: set[str] = set()
        self._refreshing = False
        self._refresh_counter = 0
        self._performance: dict[str, str] = {}
        self._performance_labels: dict[str, Any] = {}
        self._window = ui.Window("dVRK CRTK Monitor", width=520, height=760)
        with self._window.frame:
            with ui.ScrollingFrame(horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF):
                with ui.VStack(spacing=8, height=0):
                    self._add_performance_panel()
                    for component in components:
                        self._add_component(component)
        self.update()

    def _add_performance_panel(self) -> None:
        """Create a compact display-only summary of the simulator loops."""
        ui = self._ui
        with ui.CollapsableFrame("Performance", collapsed=False):
            with ui.VStack(spacing=3, height=0):
                for key, title in (
                    ("control", "Kinematic control"),
                    ("simulation", "Simulation steps"),
                    ("ros", "ROS executor"),
                    ("render", "Render"),
                    ("camera", "Camera"),
                    ("timing", "Timing"),
                ):
                    with ui.HStack(spacing=5):
                        ui.Label(f"{title}:", width=130)
                        self._performance_labels[key] = ui.Label("collecting…", width=330)

    def set_performance(self, values: dict[str, str]) -> None:
        """Receive rate/timing values sampled by the simulator main loop.

        The UI only stores already-aggregated strings: it does not inspect ROS
        or simulator state and therefore cannot add work to those loops.
        """
        self._performance = dict(values)

    def _add_component(self, component: Any) -> None:
        ui = self._ui
        name = component.config.name
        asset = component.config.raw.get("robot", {}).get("asset", {})
        instrument = asset.get("instrument") if isinstance(asset, dict) else None
        display_name = f"{name} ({instrument})" if instrument else name
        with ui.CollapsableFrame(display_name, collapsed=False):
            with ui.VStack(spacing=4, height=0):
                with ui.HStack(spacing=5):
                    state_label = ui.Label("", width=150)
                    homed_label = ui.Label("", width=100)
                    state_model = ui.ComboBox(
                        0, self._STATE_COMMAND_PLACEHOLDER, *self._STATE_COMMANDS, width=180
                    )
                    state_model.model.add_item_changed_fn(
                        lambda _model, _item, arm=name, combo=state_model: self._combo_command(arm, combo)
                    )
                with ui.CollapsableFrame("Joint control", collapsed=True):
                    with ui.VStack(spacing=4, height=0):
                        fields = []
                        for joint in component.config.joints:
                            with ui.HStack(height=26, spacing=5):
                                ui.Label(joint.name, width=100)
                                measured = ui.Label("", width=95)
                                model = ui.SimpleFloatModel(0.0)
                                field = ui.FloatField(model=model, width=105)
                                ui.Label("deg" if joint.type == "revolute" else "mm", width=35)
                                fields.append((joint, measured, model, field))
                                model.add_value_changed_fn(lambda _model, arm=name: self._mark_dirty(arm))
                        ui.Button("Apply", clicked_fn=lambda arm=name: self._apply_joints(arm))
                with ui.CollapsableFrame("Cartesian control", collapsed=True):
                    with ui.VStack(spacing=4, height=0):
                        cartesian_fields = []
                        for label, unit in (("X", "mm"), ("Y", "mm"), ("Z", "mm"),
                                            ("Roll", "deg"), ("Pitch", "deg"), ("Yaw", "deg")):
                            with ui.HStack(height=26, spacing=5):
                                ui.Label(label, width=100)
                                model = ui.SimpleFloatModel(0.0)
                                ui.FloatField(model=model, width=105)
                                ui.Label(unit, width=35)
                                cartesian_fields.append(model)
                                model.add_value_changed_fn(lambda _model, arm=name: self._mark_dirty(arm))
                        ui.Button("Apply", clicked_fn=lambda arm=name: self._apply_cartesian(arm))
                jaw = None
                if component.config.type == "PSM":
                    with ui.CollapsableFrame("Jaw", collapsed=True):
                        with ui.VStack(spacing=4, height=0):
                            with ui.HStack(height=26, spacing=5):
                                ui.Label("Jaw", width=100)
                                jaw_measured = ui.Label("", width=95)
                                jaw_model = ui.SimpleFloatModel(0.0)
                                jaw_field = ui.FloatField(model=jaw_model, width=105)
                                ui.Label("deg", width=35)
                                jaw_model.add_value_changed_fn(lambda _model, arm=name: self._mark_dirty(arm))
                                jaw = (jaw_measured, jaw_model, jaw_field)
                            ui.Button("Apply", clicked_fn=lambda arm=name: self._apply_jaw(arm))
                self._fields[name] = {
                    "state": state_label, "homed": homed_label, "joints": fields,
                    "cartesian": cartesian_fields, "jaw": jaw,
                }

    def _mark_dirty(self, name: str) -> None:
        if not self._refreshing:
            self._dirty.add(name)

    def _component(self, name: str) -> Any | None:
        return next((item for item in self._components if item.config.name == name), None)

    def _command_state(self, name: str, command: str) -> None:
        component = self._component(name)
        if component is not None:
            with self._component_lock:
                component.command_state(command)

    def _combo_command(self, name: str, combo: Any) -> None:
        # Isaac Sim omni.ui ComboBox value models expose the selected item
        # index. Convert it to the CRTK command string before dispatch.
        index = combo.model.get_item_value_model().as_int
        try:
            if 1 <= index <= len(self._STATE_COMMANDS):
                self._command_state(name, self._STATE_COMMANDS[index - 1])
        finally:
            # State commands are one-shot actions, not a persistent selection.
            combo.model.get_item_value_model().set_value(0)

    def _apply_joints(self, name: str) -> None:
        component = self._component(name)
        if component is None:
            return
        values = []
        for joint, _measured, model, _field in self._fields[name]["joints"]:
            value = model.as_float
            values.append(math.radians(value) if joint.type == "revolute" else value / 1000.0)
        with self._component_lock:
            accepted = component.command_joint_position(values)
        if accepted:
            self._dirty.discard(name)

    def _apply_jaw(self, name: str) -> None:
        component = self._component(name)
        controls = self._fields[name].get("jaw")
        if component is None or controls is None:
            return
        _measured, model, _field = controls
        with self._component_lock:
            accepted = component.command_jaw_position(math.radians(model.as_float))
        if accepted:
            self._dirty.discard(name)

    def _apply_cartesian(self, name: str) -> None:
        component = self._component(name)
        controls = self._fields[name].get("cartesian") if name in self._fields else None
        if component is None or controls is None:
            return
        values = [model.as_float for model in controls]
        position = np.asarray(values[:3], dtype=float) / 1000.0
        roll, pitch, yaw = (math.radians(value) for value in values[3:])
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        orientation = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ])
        with self._component_lock:
            accepted = component.command_cartesian_position(Pose(position, orientation))
        if accepted:
            self._dirty.discard(name)

    def update(self) -> None:
        """Refresh labels and target fields from the current simulation state."""
        self._refresh_counter += 1
        if self._refresh_counter % 6 != 0:
            return
        self._refreshing = True
        try:
            with self._component_lock:
                self._update_components()
            for key, label in self._performance_labels.items():
                label.text = self._performance.get(key, "collecting…")
        finally:
            self._refreshing = False

    def _update_components(self) -> None:
        for component in self._components:
            name = component.config.name
            controls = self._fields.get(name)
            if controls is None:
                continue
            controls["state"].text = f"State: {component.operating_state}"
            controls["homed"].text = f"Homed: {'Yes' if component.is_homed else 'No'}"
            measured = component.model.measured_js()
            goal = component.model.goal_js()
            for index, (joint, measured_label, model, _field) in enumerate(controls["joints"]):
                measured_value = float(measured.position[index])
                goal_value = float(goal.position[index])
                if joint.type == "revolute":
                    measured_value = math.degrees(measured_value)
                    goal_value = math.degrees(goal_value)
                    unit = "deg"
                else:
                    measured_value *= 1000.0
                    goal_value *= 1000.0
                    unit = "mm"
                measured_label.text = f"{measured_value:8.2f} {unit}"
                if name not in self._dirty:
                    model.set_value(goal_value)
            cartesian = component.model.measured_cp()
            cartesian_values = [
                float(cartesian.position[0]) * 1000.0,
                float(cartesian.position[1]) * 1000.0,
                float(cartesian.position[2]) * 1000.0,
                *(math.degrees(value) for value in _rpy_from_matrix(cartesian.orientation)),
            ]
            if name not in self._dirty:
                for model, value in zip(controls["cartesian"], cartesian_values):
                    model.set_value(value)
            if controls["jaw"] is not None:
                jaw_measured, jaw_model, _jaw_field = controls["jaw"]
                jaw_position = component.jaw_position
                if jaw_position is not None:
                    jaw_value = math.degrees(float(jaw_position))
                    jaw_measured.text = f"{jaw_value:8.2f} deg"
                    if name not in self._dirty:
                        jaw_model.set_value(jaw_value)

    def close(self) -> None:
        self._window.visible = False
