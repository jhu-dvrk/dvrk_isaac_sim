"""Pure-Python CRTK-style kinematics for the virtual PSM and ECM."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable

import numpy as np

from dvrk_arm_description import RobotConfig
from dvrk_simulator_base.rotations import quaternion_matrix_xyzw as _quaternion_matrix_xyzw
from dvrk_simulator_base.types import IKResult, JointState, Pose, Twist
from .urdf_kinematics import UrdfKinematicChain


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])



def _rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    return np.array([
        [c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
        [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
        [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)],
    ])


def _transform(rotation: np.ndarray, translation: Iterable[float]) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


class CRTKComponent:
    """Backend-independent component exposing CRTK-style state and commands."""

    def __init__(self, config: RobotConfig, adaptor_offset: float,
                 adaptor_rpy: tuple[float, float, float],
                 kinematics_manifest: str | Path | None = None):
        self.config = config
        self._q = config.home_position.copy()
        self._qdot = np.zeros(len(config.joints), dtype=float)
        self._target_q = self._q.copy()
        self._joint_names = tuple(joint.name for joint in config.joints)
        self._joint_types = tuple(joint.type for joint in config.joints)
        self._velocity_limits = np.asarray(
            [joint.velocity for joint in config.joints], dtype=float
        )
        self._lower_limits = np.asarray(
            [joint.lower for joint in config.joints], dtype=float
        )
        self._upper_limits = np.asarray(
            [joint.upper for joint in config.joints], dtype=float
        )
        self._adaptor_offset = float(adaptor_offset)
        self._adaptor_rpy = adaptor_rpy
        self._joint_origins = self._make_joint_origins()
        self._joint_axes_cache = self._joint_axes()
        self._joint_origin_transforms = tuple(
            _transform(_rpy_matrix(*origin_rpy), origin_xyz)
            for origin_xyz, origin_rpy in self._joint_origins
        )
        self._base_rotation = _quaternion_matrix_xyzw(
            self.config.base_orientation_xyzw
        )
        self._base_transform = _transform(
            self._base_rotation, self.config.base_position
        )
        self._cached_q: np.ndarray | None = None
        self._cached_transform: np.ndarray | None = None
        self._cached_jacobian: np.ndarray | None = None
        manifest = (Path(kinematics_manifest).expanduser().resolve()
                    if kinematics_manifest is not None else None)
        self._urdf_chain = (UrdfKinematicChain(manifest)
                            if manifest is not None and manifest.is_file() else None)
        if self._urdf_chain is not None and self._urdf_chain.active_joints != self._joint_names:
            raise ValueError(
                f"URDF manifest joints {self._urdf_chain.active_joints} do not match "
                f"configured joints {self._joint_names}"
            )

    def _make_joint_origins(self) -> tuple[tuple[np.ndarray, tuple[float, float, float]], ...]:
        origins = [(np.zeros(3), (0.0, 0.0, 0.0)) for _ in self.config.joints]
        if len(origins) >= 3:
            origins[2] = (np.array([0.0, 0.0, self._adaptor_offset]), self._adaptor_rpy)
        return tuple(origins)

    def _joint_axes(self) -> tuple[np.ndarray, ...]:
        raise NotImplementedError

    def _forward_with_jacobian(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        if q.shape != (len(self.config.joints),):
            raise ValueError("joint position has the wrong size")
        if self._urdf_chain is not None:
            transform, jacobian = self._urdf_chain.forward(
                q, self._joint_names
            )
            transform = self._base_transform @ transform
            jacobian[:3, :] = self._base_rotation @ jacobian[:3, :]
            jacobian[3:, :] = self._base_rotation @ jacobian[3:, :]
            return transform, jacobian
        transform = self._base_transform.copy()
        axes_world = []
        origins_world = []
        for index, (joint_type, axis, origin_transform) in enumerate(zip(
                self._joint_types, self._joint_axes_cache,
                self._joint_origin_transforms)):
            transform = transform @ origin_transform
            origins_world.append(transform[:3, 3].copy())
            axes_world.append(transform[:3, :3] @ axis)
            if joint_type == "revolute":
                transform = transform @ _transform(_rotation(axis, q[index]), [0.0, 0.0, 0.0])
            else:
                transform = transform @ _transform(np.eye(3), axis * q[index])

        position = transform[:3, 3]
        jacobian = np.zeros((6, len(self.config.joints)))
        for index, (axis, origin, joint_type) in enumerate(
                zip(axes_world, origins_world, self._joint_types)):
            if joint_type == "revolute":
                jacobian[:3, index] = np.cross(axis, position - origin)
                jacobian[3:, index] = axis
            else:
                jacobian[:3, index] = axis
        return transform, jacobian

    def _measured_forward_with_jacobian(self) -> tuple[np.ndarray, np.ndarray]:
        """Return cached FK/Jacobian state for the current measured joints."""
        if self._cached_q is None or not np.array_equal(self._cached_q, self._q):
            transform, jacobian = self._forward_with_jacobian(self._q)
            self._cached_q = self._q.copy()
            self._cached_transform = transform
            self._cached_jacobian = jacobian
        assert self._cached_transform is not None
        assert self._cached_jacobian is not None
        return self._cached_transform, self._cached_jacobian

    def measured_js(self) -> JointState:
        return JointState(self._joint_names, self._q.copy(), self._qdot.copy())

    def measured_cp(self, frame: str | None = None) -> Pose:
        if frame not in (None, self.config.tool_frame, self.config.adaptor_frame):
            raise ValueError(f"unknown frame {frame!r}")
        transform, _ = self._measured_forward_with_jacobian()
        return Pose(transform[:3, 3].copy(), transform[:3, :3].copy())

    def measured_cv(self, frame: str | None = None) -> Twist:
        _, jacobian = self._measured_forward_with_jacobian()
        velocity = jacobian @ self._qdot
        return Twist(velocity[:3].copy(), velocity[3:].copy())

    def goal_js(self) -> JointState:
        """Return the current move/servo joint goal using CRTK naming."""
        return JointState(self._joint_names, self._target_q.copy(), np.zeros_like(self._target_q))

    def is_busy(self) -> bool:
        return bool(np.any(np.abs(self._q - self._target_q) > 1e-9))

    def move_jp(self, joint_position: Iterable[float]) -> None:
        target = self._validate_joint_position(joint_position)
        self._target_q = target

    def prepare_startup_move(self) -> None:
        """Set the measured state to the zero pose before a startup move.

        Isaac Sim uses the normal ``move_jp`` path to reach the configured
        home pose.  Keeping measured and target joints equal at construction
        time would make that startup command a no-op, so the simulator uses
        this method immediately before issuing its initial command.
        """
        zero = np.zeros_like(self._q)
        zero = self._validate_joint_position(zero)
        self._q = zero
        self._target_q = zero.copy()
        self._qdot = np.zeros_like(zero)
        self._cached_q = None
        self._cached_transform = None
        self._cached_jacobian = None

    def servo_jp(self, joint_position: Iterable[float]) -> None:
        self.move_jp(joint_position)

    def step(self, dt: float) -> None:
        if dt < 0.0:
            raise ValueError("dt must be non-negative")
        delta = self._target_q - self._q
        max_delta = self._velocity_limits * dt
        applied = np.clip(delta, -max_delta, max_delta)
        self._qdot = applied / dt if dt > 0.0 else np.zeros_like(applied)
        self._q += applied
        if np.allclose(self._q, self._target_q):
            self._qdot[:] = 0.0

    def compute_fk(self, q: Iterable[float] | None = None, frame: str | None = None) -> Pose:
        values = self._q if q is None else self._validate_joint_position(q)
        if frame not in (None, self.config.tool_frame, self.config.adaptor_frame):
            raise ValueError(f"unknown frame {frame!r}")
        transform, _ = (
            self._measured_forward_with_jacobian()
            if q is None else self._forward_with_jacobian(values)
        )
        return Pose(transform[:3, 3].copy(), transform[:3, :3].copy())

    def compute_jacobian(self, q: Iterable[float] | None = None, frame: str | None = None) -> np.ndarray:
        values = self._q if q is None else self._validate_joint_position(q)
        if frame not in (None, self.config.tool_frame, self.config.adaptor_frame):
            raise ValueError(f"unknown frame {frame!r}")
        _, jacobian = (
            self._measured_forward_with_jacobian()
            if q is None else self._forward_with_jacobian(values)
        )
        return jacobian.copy()

    def compute_ik(self, target: Pose, seed: Iterable[float] | None = None, max_iterations: int = 100) -> IKResult:
        """Solve Cartesian IK, including orientation when the chain has six DOFs."""
        q = self._q.copy() if seed is None else self._validate_joint_position(seed)
        use_orientation = len(self.config.joints) >= 6
        tolerance = 1e-5
        for iteration in range(max_iterations):
            transform, jacobian = self._forward_with_jacobian(q)
            pose = Pose(transform[:3, 3], transform[:3, :3])
            position_error = target.position - pose.position
            if use_orientation:
                # First-order world-frame rotation error. This is compatible
                # with the angular part of the spatial Jacobian.
                orientation_error = 0.5 * (
                    np.cross(pose.orientation[:, 0], target.orientation[:, 0])
                    + np.cross(pose.orientation[:, 1], target.orientation[:, 1])
                    + np.cross(pose.orientation[:, 2], target.orientation[:, 2])
                )
                error = np.concatenate((position_error, orientation_error))
            else:
                error = position_error
                jacobian = jacobian[:3, :]
            error_norm = float(np.linalg.norm(error))
            if error_norm < tolerance:
                return IKResult(
                    q, True, iterations=iteration, position_error=error_norm,
                    message="pose converged" if use_orientation else "position converged",
                )
            step = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + 1e-6 * np.eye(jacobian.shape[0]), error)
            q = self._clip_joint_position(q + 0.5 * step)
        pose = self.compute_fk(q)
        position_error = target.position - pose.position
        if use_orientation:
            orientation_error = 0.5 * (
                np.cross(pose.orientation[:, 0], target.orientation[:, 0])
                + np.cross(pose.orientation[:, 1], target.orientation[:, 1])
                + np.cross(pose.orientation[:, 2], target.orientation[:, 2])
            )
            error_norm = float(np.linalg.norm(np.concatenate((position_error, orientation_error))))
        else:
            error_norm = float(np.linalg.norm(position_error))
        return IKResult(
            q, error_norm < tolerance, iterations=max_iterations,
            position_error=error_norm,
            message="pose IK did not converge" if use_orientation else "position IK did not converge",
        )

    def move_cp(self, target: Pose) -> IKResult:
        result = self.compute_ik(target)
        if result.success:
            self.move_jp(result.position)
        return result

    def _clip_joint_position(self, position: np.ndarray) -> np.ndarray:
        return np.clip(position, self._lower_limits, self._upper_limits)

    def _validate_joint_position(self, position: Iterable[float]) -> np.ndarray:
        result = np.asarray(list(position), dtype=float)
        if result.shape != (len(self.config.joints),):
            raise ValueError("joint position has the wrong size")
        if np.any(result < self._lower_limits) or np.any(result > self._upper_limits):
            raise ValueError("joint position exceeds configured limits")
        return result


class CRTKPSM(CRTKComponent):
    """Six-DOF virtual PSM: RCM plus roll, wrist pitch, and wrist yaw."""

    def __init__(self, config: RobotConfig,
                 kinematics_manifest: str | Path | None = None):
        if config.type != "PSM" or len(config.joints) != 6:
            raise ValueError("CRTKPSM requires six configured joints")
        super().__init__(
            config, adaptor_offset=0.4826,
            adaptor_rpy=(math.pi, 0.0, -math.pi / 2.0),
            kinematics_manifest=kinematics_manifest,
        )

    def _make_joint_origins(self) -> tuple[tuple[np.ndarray, tuple[float, float, float]], ...]:
        origins = list(super()._make_joint_origins())
        # Approximate the 420006 instrument chain. The converter/visual keeps
        # the instrument-specific mesh, while this provides a useful generic
        # wrist length for Cartesian FK and measured_cp.
        if len(origins) >= 6:
            origins[3] = (np.array([0.0, 0.0, 0.4670]), (0.0, 0.0, 0.0))
            origins[4] = (np.zeros(3), (-math.pi / 2.0, -math.pi / 2.0, 0.0))
            origins[5] = (np.array([0.0107, 0.0, 0.0]), (-math.pi / 2.0, -math.pi / 2.0, 0.0))
        return tuple(origins)

    def _joint_axes(self) -> tuple[np.ndarray, ...]:
        return (
            np.array([0.0, -1.0, 0.0]),
            np.array([-1.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 1.0]),
            np.array([0.0, 0.0, 1.0]),
            np.array([0.0, 0.0, 1.0]),
            np.array([0.0, 0.0, 1.0]),
        )


class CRTKECM(CRTKComponent):
    """Four-DOF virtual ECM: yaw, pitch, insertion, and roll."""

    def __init__(self, config: RobotConfig,
                 kinematics_manifest: str | Path | None = None):
        if config.type != "ECM" or len(config.joints) != 4:
            raise ValueError("CRTKECM requires a four-joint ECM configuration")
        super().__init__(
            config, adaptor_offset=0.3829,
            adaptor_rpy=(math.pi, 0.0, math.pi / 2.0),
            kinematics_manifest=kinematics_manifest,
        )

    def _joint_axes(self) -> tuple[np.ndarray, ...]:
        return (
            np.array([0.0, -1.0, 0.0]),
            np.array([-1.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 1.0]),
            np.array([0.0, 0.0, 1.0]),
        )


def main() -> None:
    """Small integration-test entry point installed with the ROS 2 package."""
    print("dvrk_isaac_sim kinematic core is available; launch integration is not implemented yet")
