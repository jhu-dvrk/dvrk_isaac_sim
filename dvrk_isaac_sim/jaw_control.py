"""Bounded logical jaw PID used before full dynamic jaw articulation."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class JawPIDConfig:
    lower: float = -0.349066
    upper: float = 1.39626
    kp: float = 90.0
    ki: float = 0.0
    kd: float = 6.0
    max_effort: float = 18.0
    inertia: float = 1.0
    max_velocity: float = 1.2
    target_rate: float = 1.5
    integral_limit: float = 0.25
    effort_deadband: float = 0.0


@dataclass(frozen=True)
class JawPIDState:
    position: float
    velocity: float
    target: float
    commanded_target: float
    effort: float
    integral: float
    saturated: bool


def _finite(value: float, fallback: float) -> float:
    return float(value) if math.isfinite(float(value)) else float(fallback)


class JawPIDController:
    """Finite-effort PID for the logical dVRK jaw coordinate.

    This controller intentionally lives in the logical jaw coordinate rather
    than on the two mimic links. It provides bounded, damped target tracking for
    the kinematic collider path. A later dynamic Newton jaw can consume the same
    target/gain parameters and replace only the final integration step.
    """

    def __init__(self, config: JawPIDConfig):
        self.config = config
        self._position = self._clip(0.0)
        self._velocity = 0.0
        self._target = self._position
        self._commanded_target = self._position
        self._integral = 0.0
        self._effort = 0.0
        self._saturated = False

    @property
    def state(self) -> JawPIDState:
        return JawPIDState(
            position=self._position,
            velocity=self._velocity,
            target=self._target,
            commanded_target=self._commanded_target,
            effort=self._effort,
            integral=self._integral,
            saturated=self._saturated,
        )

    def set_command(self, target: float) -> None:
        self._commanded_target = self._clip(target)

    def reset(self, position: float | None = None) -> None:
        if position is not None:
            self._position = self._clip(position)
        self._velocity = 0.0
        self._target = self._position
        self._commanded_target = self._position
        self._integral = 0.0
        self._effort = 0.0
        self._saturated = False

    def hold(self) -> None:
        """Make the current measured opening the new target and clear stored I."""
        self._target = self._position
        self._commanded_target = self._position
        self._integral = 0.0
        self._effort = 0.0
        self._saturated = False

    def step(self, dt: float) -> JawPIDState:
        dt = float(dt)
        if dt <= 0.0:
            self._effort = 0.0
            self._saturated = False
            return self.state

        rate = max(0.0, float(self.config.target_rate))
        if rate > 0.0:
            delta = self._commanded_target - self._target
            self._target += max(-rate * dt, min(rate * dt, delta))
        else:
            self._target = self._commanded_target
        self._target = self._clip(self._target)

        error = self._target - self._position
        proposed_integral = self._integral + error * dt
        limit = max(0.0, float(self.config.integral_limit))
        if limit > 0.0:
            proposed_integral = max(-limit, min(limit, proposed_integral))
        else:
            proposed_integral = 0.0

        raw_effort = (
            float(self.config.kp) * error
            + float(self.config.ki) * proposed_integral
            - float(self.config.kd) * self._velocity
        )
        deadband = max(0.0, float(self.config.effort_deadband))
        if abs(raw_effort) < deadband:
            raw_effort = 0.0
        max_effort = max(0.0, float(self.config.max_effort))
        effort = max(-max_effort, min(max_effort, raw_effort)) if max_effort > 0.0 else 0.0
        self._saturated = not math.isclose(effort, raw_effort, rel_tol=0.0, abs_tol=1e-12)

        # Conditional integration avoids winding up against effort or joint
        # limits while still permitting Ki to remove a genuine steady error.
        at_lower = self._position <= self.config.lower and effort < 0.0
        at_upper = self._position >= self.config.upper and effort > 0.0
        if not self._saturated and not at_lower and not at_upper:
            self._integral = proposed_integral

        inertia = max(1e-9, float(self.config.inertia))
        velocity = self._velocity + (effort / inertia) * dt
        max_velocity = max(0.0, float(self.config.max_velocity))
        if max_velocity > 0.0:
            velocity = max(-max_velocity, min(max_velocity, velocity))

        previous_position = self._position
        position = self._clip(previous_position + velocity * dt)
        if (
            (self._target - previous_position) != 0.0
            and (self._target - previous_position) * (self._target - position) < 0.0
        ):
            position = self._target
            velocity = 0.0
        if position in (self.config.lower, self.config.upper):
            velocity = 0.0
        if abs(position - self._target) < 1e-6 and abs(velocity) < 1e-5:
            position = self._target
            velocity = 0.0

        self._position = position
        self._velocity = velocity
        self._effort = effort
        return self.state

    def _clip(self, value: float) -> float:
        value = _finite(value, 0.0)
        return max(float(self.config.lower), min(float(self.config.upper), value))


def jaw_controller_from_config(config) -> JawPIDController:
    jaw_config = config.raw.get("robot", {}).get("jaw", {})
    if not isinstance(jaw_config, dict):
        jaw_config = {}
    control_config = jaw_config.get("pid", {})
    if not isinstance(control_config, dict):
        control_config = {}
    lower = float(jaw_config.get("lower", -0.349066))
    upper = float(jaw_config.get("upper", 1.39626))
    return JawPIDController(JawPIDConfig(
        lower=lower,
        upper=upper,
        kp=float(control_config.get("kp", 90.0)),
        ki=float(control_config.get("ki", 0.0)),
        kd=float(control_config.get("kd", 6.0)),
        max_effort=float(control_config.get("max_effort", 18.0)),
        inertia=float(control_config.get("inertia", 1.0)),
        max_velocity=float(control_config.get("max_velocity", 1.2)),
        target_rate=float(control_config.get("target_rate", 1.5)),
        integral_limit=float(control_config.get("integral_limit", 0.25)),
        effort_deadband=float(control_config.get("effort_deadband", 0.0)),
    ))
