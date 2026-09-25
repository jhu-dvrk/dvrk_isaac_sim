"""Conservative kinematic contact guard for fast instrument motion."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Iterable

import numpy as np

from .scene import SceneProp
from .urdf_kinematics import _rpy_matrix, _transform


@dataclass(frozen=True)
class _CollisionCloud:
    source_link: str
    local_points: np.ndarray


@dataclass(frozen=True)
class _AabbObstacle:
    name: str
    minimum: np.ndarray
    maximum: np.ndarray


def _package_uri_to_path(uri: str) -> Path | None:
    if uri.startswith("package://"):
        package_and_rel = uri[len("package://") :]
        package, _, relative = package_and_rel.partition("/")
        try:
            from ament_index_python.packages import get_package_share_directory
        except Exception:
            return None
        return Path(get_package_share_directory(package)) / relative
    path = Path(uri)
    return path if path.is_absolute() else None


def _load_obj_vertices(path: Path) -> np.ndarray:
    vertices = []
    for line in path.read_text(errors="ignore").splitlines():
        if line.startswith("v "):
            parts = line.split()
            if len(parts) >= 4:
                vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
    if not vertices:
        raise ValueError(f"{path}: OBJ contains no vertices")
    return np.asarray(vertices, dtype=float)


def _corners(minimum: np.ndarray, maximum: np.ndarray) -> np.ndarray:
    return np.asarray(
        [
            [x, y, z]
            for x in (minimum[0], maximum[0])
            for y in (minimum[1], maximum[1])
            for z in (minimum[2], maximum[2])
        ],
        dtype=float,
    )


def _prop_aabb(prop: SceneProp, margin: float) -> _AabbObstacle:
    center = np.asarray(prop.position, dtype=float)
    half = 0.5 * np.asarray(prop.size, dtype=float)
    # Environment props are currently authored as axis-aligned cubes/tables in
    # the high-speed contact test scenes.  Inflate the AABB by margin so the
    # guard stops just before visual penetration.
    return _AabbObstacle(prop.name, center - half - margin, center + half + margin)


def _aabb_overlap(a_min: np.ndarray, a_max: np.ndarray, b_min: np.ndarray, b_max: np.ndarray) -> bool:
    return bool(np.all(a_min <= b_max) and np.all(b_min <= a_max))


class KinematicContactGuard:
    """Clamp kinematic joint steps before collision geometry crosses props.

    The guard uses the manifest's collision meshes as point clouds and checks a
    conservative swept AABB between the previous and proposed joint states.  It
    is deliberately engine-independent, so the same clamp affects both visual
    and flattened collision transforms.
    """

    def __init__(
        self,
        manifest_path: str | Path,
        kinematic_chain,
        props: Iterable[SceneProp],
        *,
        margin: float = 0.0005,
        iterations: int = 12,
    ) -> None:
        self._chain = kinematic_chain
        self._margin = float(margin)
        self._iterations = int(iterations)
        self._obstacles = tuple(
            _prop_aabb(prop, self._margin)
            for prop in props
            if prop.kind in {"cube", "table"} and (prop.static or prop.dynamic)
        )
        self._clouds = self._load_clouds(Path(manifest_path))

    @property
    def enabled(self) -> bool:
        return bool(self._clouds and self._obstacles)

    def clamp(self, joint_names: tuple[str, ...], previous: np.ndarray, proposed: np.ndarray) -> np.ndarray:
        if not self.enabled or not self._state_crosses_obstacle(joint_names, previous, proposed):
            return proposed
        low = 0.0
        high = 1.0
        for _ in range(self._iterations):
            mid = 0.5 * (low + high)
            candidate = previous + (proposed - previous) * mid
            if self._state_crosses_obstacle(joint_names, previous, candidate):
                high = mid
            else:
                low = mid
        return previous + (proposed - previous) * max(0.0, low - 1e-4)

    def _load_clouds(self, manifest_path: Path) -> tuple[_CollisionCloud, ...]:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        collision = manifest.get("collision") if isinstance(manifest, dict) else None
        if not isinstance(collision, dict):
            return ()
        clouds = []
        for item in collision.get("items", []):
            if not isinstance(item, dict):
                continue
            geometry = item.get("geometry")
            if not isinstance(geometry, dict) or geometry.get("type") != "mesh":
                continue
            path = _package_uri_to_path(str(geometry.get("filename", "")))
            if path is None or not path.is_file():
                continue
            points = _load_obj_vertices(path)
            scale = np.asarray(geometry.get("scale", [1.0, 1.0, 1.0]), dtype=float)
            if scale.shape != (3,) or np.any(scale == 0.0):
                scale = np.ones(3, dtype=float)
            points = points * scale
            origin = _transform(
                _rpy_matrix(*[float(value) for value in item.get("origin_rpy", [0.0, 0.0, 0.0])]),
                [float(value) for value in item.get("origin_xyz", [0.0, 0.0, 0.0])],
            )
            homogeneous = np.c_[points, np.ones(len(points))]
            local_points = (origin @ homogeneous.T).T[:, :3]
            link = str(item.get("source_link", "")).strip()
            if link:
                clouds.append(_CollisionCloud(link, local_points))
        return tuple(clouds)

    def _state_crosses_obstacle(
        self,
        joint_names: tuple[str, ...],
        previous: np.ndarray,
        proposed: np.ndarray,
    ) -> bool:
        previous_poses = self._chain.forward_all_links(previous, joint_names)
        proposed_poses = self._chain.forward_all_links(proposed, joint_names)
        for cloud in self._clouds:
            if cloud.source_link not in previous_poses or cloud.source_link not in proposed_poses:
                continue
            start = self._world_points(previous_poses[cloud.source_link], cloud.local_points)
            end = self._world_points(proposed_poses[cloud.source_link], cloud.local_points)
            swept = np.vstack((start, end))
            swept_min = swept.min(axis=0)
            swept_max = swept.max(axis=0)
            for obstacle in self._obstacles:
                if _aabb_overlap(swept_min, swept_max, obstacle.minimum, obstacle.maximum):
                    return True
        return False

    @staticmethod
    def _world_points(transform: np.ndarray, local_points: np.ndarray) -> np.ndarray:
        homogeneous = np.c_[local_points, np.ones(len(local_points))]
        return (transform @ homogeneous.T).T[:, :3]
