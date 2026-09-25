"""Diagnostics for imported and flattened collision mesh frames."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np


def load_collision_items(manifest_path: Path) -> tuple[str, list[dict]]:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    collision = manifest.get("collision") if isinstance(manifest, dict) else None
    if not isinstance(collision, dict) or not isinstance(collision.get("items"), list):
        raise RuntimeError(f"{manifest_path}: manifest has no collision.items data")
    return str(collision.get("root", "Geometry/world")), [
        item for item in collision["items"] if isinstance(item, dict)
    ]


def link_source_paths(component: str, visual_root: str, items: list[dict]) -> dict[str, str]:
    result: dict[str, str] = {}
    for item in items:
        link = str(item.get("source_link", "")).strip()
        if not link or link in result:
            continue
        source_relative = str(item.get("prim", "")).strip("/")
        if source_relative:
            result[link] = f"/World/{component}/{visual_root}/{source_relative}"
        else:
            result[link] = f"/World/{component}/{visual_root}"
    return result


def _usd_matrix_to_numpy_column_major(matrix) -> np.ndarray:
    return np.array(matrix, dtype=float).T


def _rotation_angle_degrees(reference: np.ndarray, actual: np.ndarray) -> float:
    def normalized(rotation: np.ndarray) -> np.ndarray:
        result = rotation[:3, :3].copy()
        for index in range(3):
            norm = np.linalg.norm(result[:, index])
            if norm > 0.0:
                result[:, index] /= norm
        return result

    delta = normalized(reference).T @ normalized(actual)
    cosine = float((np.trace(delta) - 1.0) * 0.5)
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def _format_vector(values) -> str:
    return "[" + ", ".join(f"{float(value): .6f}" for value in values) + "]"


def _print_frame(label: str, matrix: np.ndarray) -> None:
    rotation = matrix[:3, :3]
    translation = matrix[:3, 3]
    print(f"  {label} origin:", _format_vector(translation), flush=True)
    print(f"  {label} x-axis:", _format_vector(rotation[:, 0]), flush=True)
    print(f"  {label} y-axis:", _format_vector(rotation[:, 1]), flush=True)
    print(f"  {label} z-axis:", _format_vector(rotation[:, 2]), flush=True)


def report_collision_item(
    stage,
    component: str,
    visual_root: str,
    item: dict,
    source_paths: dict[str, str],
    index: int,
) -> None:
    import dvrk_isaac_sim.usd_physics_links as usd_physics_links
    from dvrk_isaac_sim.usd_physics_links import _candidate_collision_prim, _collision_body_name
    from pxr import Usd, UsdGeom  # type: ignore[import-not-found]

    source_link = str(item.get("source_link", ""))
    link_prim_path = source_paths.get(source_link, f"/World/{component}/{visual_root}")
    source = stage.GetPrimAtPath(link_prim_path)
    if not source.IsValid():
        print(f"[{source_link}] MISSING link prim: {link_prim_path}", flush=True)
        return

    blocked_paths = tuple(
        other_source_path
        for other_link, other_source_path in source_paths.items()
        if other_link != source_link and other_source_path.startswith(link_prim_path + "/")
    )
    candidate = _candidate_collision_prim(source, Usd, UsdGeom, item, blocked_paths)
    world_link = UsdGeom.Xformable(source).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    flattened_collision_path = f"/World/{component}/PhysicsLinks/{_collision_body_name(item, index)}/Collision"
    flattened_collision = stage.GetPrimAtPath(flattened_collision_path)
    if not flattened_collision.IsValid():
        fallback_path = f"/World/{component}/PhysicsLinks/{source_link}/Collision"
        flattened_collision = stage.GetPrimAtPath(fallback_path)
    if not flattened_collision.IsValid():
        print(f"[{source_link}] no flattened collision prim yet", flush=True)
        return

    world_flat = UsdGeom.Xformable(flattened_collision).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    world_link_np = _usd_matrix_to_numpy_column_major(world_link)
    world_flat_np = _usd_matrix_to_numpy_column_major(world_flat)
    world_candidate_np = None
    if candidate is not None:
        world_candidate = UsdGeom.Xformable(candidate).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        world_candidate_np = _usd_matrix_to_numpy_column_major(world_candidate)

    print(source_link, flattened_collision.GetPath(), flush=True)
    print("  module path:     ", usd_physics_links.__file__, flush=True)
    print("  manifest name:   ", item.get("name"), flush=True)
    print("  source prim:     ", link_prim_path, flush=True)
    print("  blocked paths:   ", blocked_paths if blocked_paths else "<none>", flush=True)
    print("  nested candidate:", candidate.GetPath() if candidate is not None else "<none>", flush=True)
    print("  URDF origin used:", item.get("origin_xyz"), flush=True)
    _print_frame("link frame     ", world_link_np)
    if world_candidate_np is not None:
        _print_frame("nested frame   ", world_candidate_np)
    _print_frame("flattened frame", world_flat_np)
    if world_candidate_np is not None:
        position_error = np.linalg.norm(world_candidate_np[:3, 3] - world_flat_np[:3, 3])
        angle_error = _rotation_angle_degrees(world_candidate_np, world_flat_np)
        print(f"  nested-vs-flat position error: {position_error:.9f} m", flush=True)
        print(f"  nested-vs-flat rotation error: {angle_error:.9f} deg", flush=True)
        link_to_collision = np.linalg.inv(world_link_np) @ world_candidate_np
        _print_frame("link->collision", link_to_collision)


def report_collision_frames(stage, component: str, manifest_path: Path) -> None:
    visual_root, items = load_collision_items(manifest_path)
    source_paths = link_source_paths(component, visual_root, items)
    for index, item in enumerate(items):
        report_collision_item(stage, component, visual_root, item, source_paths, index)
