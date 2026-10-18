"""Small homogeneous-transform helpers shared across simulator modules."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return the XYZ fixed-axis roll/pitch/yaw rotation matrix."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def rotation_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    """Return the right-handed rotation matrix for an axis-angle pair."""
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm == 0.0:
        raise ValueError("rotation axis must be non-zero")
    x, y, z = axis / norm
    c, s = math.cos(angle), math.sin(angle)
    return np.array([
        [c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
        [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
        [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)],
    ])


def transform_matrix(rotation: np.ndarray, translation: Iterable[float]) -> np.ndarray:
    """Return a 4x4 homogeneous transform from rotation and translation."""
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def transform_from_rpy(
    translation: Iterable[float],
    rpy: Iterable[float],
) -> np.ndarray:
    """Return a homogeneous transform from URDF-style xyz/rpy values."""
    roll, pitch, yaw = (float(value) for value in rpy)
    xyz = [float(value) for value in translation]
    return transform_matrix(rpy_matrix(roll, pitch, yaw), xyz)
