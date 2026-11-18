"""Backend-independent rotation conversions with explicit ROS ordering."""

from __future__ import annotations

import numpy as np


def quaternion_matrix_xyzw(quaternion: np.ndarray) -> np.ndarray:
    """Convert a ROS-order quaternion ``(x, y, z, w)`` to a rotation matrix."""
    x, y, z, w = np.asarray(quaternion, dtype=float) / np.linalg.norm(quaternion)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rotation_to_quaternion_xyzw(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a rotation matrix to a normalized ROS-order quaternion."""
    rotation = np.asarray(rotation, dtype=float)
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = 2.0 * np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
        w = (rotation[2, 1] - rotation[1, 2]) / scale
        x = 0.25 * scale
        y = (rotation[0, 1] + rotation[1, 0]) / scale
        z = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = 2.0 * np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])
        w = (rotation[0, 2] - rotation[2, 0]) / scale
        x = (rotation[0, 1] + rotation[1, 0]) / scale
        y = 0.25 * scale
        z = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = 2.0 * np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])
        w = (rotation[1, 0] - rotation[0, 1]) / scale
        x = (rotation[0, 2] + rotation[2, 0]) / scale
        y = (rotation[1, 2] + rotation[2, 1]) / scale
        z = 0.25 * scale
    result = np.asarray([x, y, z, w], dtype=float)
    result /= np.linalg.norm(result)
    return tuple(float(value) for value in result)


def rotation_to_quaternion_wxyz(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a rotation matrix to Isaac's scalar-first ``(w, x, y, z)``."""
    x, y, z, w = rotation_to_quaternion_xyzw(rotation)
    return (w, x, y, z)
