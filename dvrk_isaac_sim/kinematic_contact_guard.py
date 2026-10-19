"""Conservative kinematic contact guard for fast instrument motion."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Iterable

import numpy as np

from .scene import SceneProp
from .environment_geometry import cone_bowl_segments
from .rotations import quaternion_matrix_xyzw
from .transforms import transform_from_rpy as _transform_from_rpy


@dataclass(frozen=True)
class _CollisionCloud:
    source_link: str
    local_points: np.ndarray


@dataclass(frozen=True)
class _AabbObstacle:
    name: str
    minimum: np.ndarray
    maximum: np.ndarray


@dataclass(frozen=True)
class _ConvexObstacle:
    points: np.ndarray
    axes: np.ndarray
    minimum: np.ndarray
    maximum: np.ndarray


def _cone_obstacles(prop: SceneProp, margin: float):
    rotation = quaternion_matrix_xyzw(np.asarray(prop.orientation_xyzw))
    obstacles = []
    for vertices, faces in cone_bowl_segments(prop.size):
        points = np.asarray(vertices) @ rotation.T + prop.position
        triangles = points[np.asarray(faces)]
        normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        normals /= np.linalg.norm(normals, axis=1)[:, None]
        axes = np.vstack((np.eye(3), normals))
        projected = points @ axes.T
        obstacles.append(_ConvexObstacle(points, axes, projected.min(axis=0) - margin,
                                         projected.max(axis=0) + margin))
    return tuple(obstacles)


def _convex_overlap(points: np.ndarray, obstacle: _ConvexObstacle) -> bool:
    projected = points @ obstacle.axes.T
    return bool(np.all(projected.max(axis=0) >= obstacle.minimum)
                and np.all(projected.min(axis=0) <= obstacle.maximum))


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


def _box_points(size) -> np.ndarray:
    half = 0.5 * np.asarray(size, dtype=float)
    return np.asarray(
        [
            [x, y, z]
            for x in (-half[0], half[0])
            for y in (-half[1], half[1])
            for z in (-half[2], half[2])
        ],
        dtype=float,
    )


def _cylinder_points(radius: float, length: float, segments: int = 24) -> np.ndarray:
    angles = np.linspace(0.0, 2.0 * np.pi, int(segments), endpoint=False)
    half = 0.5 * float(length)
    rings = [
        [float(radius) * np.cos(angle), float(radius) * np.sin(angle), z]
        for z in (-half, half)
        for angle in angles
    ]
    return np.asarray(rings + [[0.0, 0.0, -half], [0.0, 0.0, half]], dtype=float)


def _capsule_points(radius: float, length: float, segments: int = 24) -> np.ndarray:
    radius = float(radius)
    half = 0.5 * max(float(length) - 2.0 * radius, 0.0)
    angles = np.linspace(0.0, 2.0 * np.pi, int(segments), endpoint=False)
    rings = [
        [radius * np.cos(angle), radius * np.sin(angle), z]
        for z in (-half, half)
        for angle in angles
    ]
    caps = [[0.0, 0.0, -half - radius], [0.0, 0.0, half + radius]]
    caps.extend([radius * np.cos(angle), radius * np.sin(angle), -half] for angle in angles)
    caps.extend([radius * np.cos(angle), radius * np.sin(angle), half] for angle in angles)
    return np.asarray(rings + caps, dtype=float)


def _sphere_points(radius: float, segments: int = 16) -> np.ndarray:
    radius = float(radius)
    points = [
        [radius, 0.0, 0.0],
        [-radius, 0.0, 0.0],
        [0.0, radius, 0.0],
        [0.0, -radius, 0.0],
        [0.0, 0.0, radius],
        [0.0, 0.0, -radius],
    ]
    angles = np.linspace(0.0, 2.0 * np.pi, int(segments), endpoint=False)
    points.extend([radius * np.cos(angle), radius * np.sin(angle), 0.0] for angle in angles)
    return np.asarray(points, dtype=float)


def _geometry_points(geometry: dict) -> np.ndarray | None:
    geometry_type = geometry.get("type")
    if geometry_type == "convex_mesh":
        points = np.asarray(geometry.get("points", []), dtype=float)
        return points if points.ndim == 2 and points.shape[1] == 3 and len(points) else None
    if geometry_type == "mesh":
        path = _package_uri_to_path(str(geometry.get("filename", "")))
        if path is None or not path.is_file():
            return None
        points = _load_obj_vertices(path)
        scale = np.asarray(geometry.get("scale", [1.0, 1.0, 1.0]), dtype=float)
        if scale.shape != (3,) or np.any(scale == 0.0):
            scale = np.ones(3, dtype=float)
        return points * scale
    if geometry_type == "box":
        size = np.asarray(geometry.get("size", [0.0, 0.0, 0.0]), dtype=float)
        if size.shape != (3,) or np.any(size <= 0.0):
            return None
        return _box_points(size)
    if geometry_type == "cylinder":
        radius = float(geometry.get("radius", 0.0))
        length = float(geometry.get("length", 0.0))
        if radius <= 0.0 or length <= 0.0:
            return None
        return _cylinder_points(radius, length)
    if geometry_type == "capsule":
        radius = float(geometry.get("radius", 0.0))
        length = float(geometry.get("length", 0.0))
        if radius <= 0.0 or length <= 0.0:
            return None
        return _capsule_points(radius, length)
    if geometry_type == "sphere":
        radius = float(geometry.get("radius", 0.0))
        if radius <= 0.0:
            return None
        return _sphere_points(radius)
    return None


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
    conservative swept AABB for box props and swept point-cloud projections
    against the individual cone walls. It is engine-independent, so the same
    clamp affects both visual and flattened collision transforms.
    """

    def __init__(
        self,
        manifest_path: str | Path,
        kinematic_chain,
        props: Iterable[SceneProp],
        *,
        margin: float = 0.0005,
        iterations: int = 12,
        static_only: bool = False,
        base_transform: np.ndarray | None = None,
    ) -> None:
        self._chain = kinematic_chain
        self._margin = float(margin)
        self._iterations = int(iterations)
        self._base_transform = np.eye(4) if base_transform is None else np.asarray(base_transform)
        props = tuple(props)
        self._cone_walls = tuple(
            wall for prop in props if prop.kind == "cone_bowl" and prop.static
            for wall in _cone_obstacles(prop, self._margin)
        )
        self._obstacles = tuple(
            _prop_aabb(prop, self._margin)
            for prop in props
            if prop.kind in {"cube", "sphere", "table"} and (prop.static or prop.dynamic)
            and (not static_only or prop.static)
        )
        self._clouds = self._load_clouds(Path(manifest_path))

    @property
    def enabled(self) -> bool:
        return bool(self._clouds and (self._obstacles or self._cone_walls))

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
            if not isinstance(geometry, dict):
                continue
            points = _geometry_points(geometry)
            if points is None:
                continue
            origin = _transform_from_rpy(
                item.get("origin_xyz", [0.0, 0.0, 0.0]),
                item.get("origin_rpy", [0.0, 0.0, 0.0]),
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
            start = self._world_points(self._base_transform @ previous_poses[cloud.source_link], cloud.local_points)
            end = self._world_points(self._base_transform @ proposed_poses[cloud.source_link], cloud.local_points)
            swept = np.vstack((start, end))
            swept_min = swept.min(axis=0)
            swept_max = swept.max(axis=0)
            for obstacle in self._obstacles:
                if _aabb_overlap(swept_min, swept_max, obstacle.minimum, obstacle.maximum):
                    return True
            for wall in self._cone_walls:
                if _convex_overlap(swept, wall):
                    return True
        return False

    @staticmethod
    def _world_points(transform: np.ndarray, local_points: np.ndarray) -> np.ndarray:
        homogeneous = np.c_[local_points, np.ones(len(local_points))]
        return (transform @ homogeneous.T).T[:, :3]
