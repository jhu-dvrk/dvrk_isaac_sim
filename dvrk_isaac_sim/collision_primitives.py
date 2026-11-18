"""Collision recipes fitted from URDF visual meshes."""

from __future__ import annotations

import json
import math
import re
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from .transforms import transform_from_rpy as _transform_from_rpy
from .urdf_utils import numbers as _numbers
from .urdf_utils import package_mesh_path as _package_mesh_path


def _origin(element: ET.Element) -> tuple[list[float], list[float]]:
    origin = element.find("origin")
    return (
        _numbers(origin.attrib.get("xyz") if origin is not None else None, 3),
        _numbers(origin.attrib.get("rpy") if origin is not None else None, 3),
    )


def _rpy_from_matrix(rotation: np.ndarray) -> list[float]:
    sy = -float(rotation[2, 0])
    if abs(sy) < 1.0 - 1e-12:
        pitch = math.asin(sy)
        roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
        yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    else:
        pitch = math.copysign(math.pi / 2.0, sy)
        roll = 0.0
        yaw = math.atan2(-float(rotation[0, 1]), float(rotation[1, 1]))
    return [roll, pitch, yaw]


def _matrix_origin(transform: np.ndarray) -> tuple[list[float], list[float]]:
    return (
        [float(value) for value in transform[:3, 3]],
        _rpy_from_matrix(transform[:3, :3]),
    )


def _read_obj_vertices(path: Path) -> np.ndarray:
    vertices: list[list[float]] = []
    with path.open("r", encoding="utf-8", errors="ignore") as stream:
        for line in stream:
            if not line.startswith("v "):
                continue
            fields = line.split()
            if len(fields) < 4:
                continue
            vertices.append([float(fields[1]), float(fields[2]), float(fields[3])])
    return np.asarray(vertices, dtype=float)


def _link_visuals(urdf_path: Path, model_root: Path) -> dict[str, list[dict[str, Any]]]:
    root = ET.parse(urdf_path).getroot()
    visuals: dict[str, list[dict[str, Any]]] = {}
    for link in root.findall("link"):
        link_name = link.attrib.get("name", "")
        if not link_name:
            continue
        for index, visual in enumerate(link.findall("visual")):
            geometry = visual.find("geometry")
            mesh = geometry.find("mesh") if geometry is not None else None
            if mesh is None:
                continue
            filename = mesh.attrib.get("filename", "")
            mesh_path = _package_mesh_path(model_root, filename)
            if mesh_path is None or mesh_path.suffix.lower() != ".obj" or not mesh_path.is_file():
                continue
            xyz, rpy = _origin(visual)
            visuals.setdefault(link_name, []).append({
                "index": index,
                "filename": filename,
                "path": mesh_path,
                "scale": _numbers(mesh.attrib.get("scale"), 3, default=1.0),
                "origin_xyz": xyz,
                "origin_rpy": rpy,
            })
    return visuals


def _regex_matches(pattern: str | None, value: str) -> bool:
    return bool(pattern is None or re.search(pattern, value))


def _rule_matches(rule: dict[str, Any], items: list[dict], visuals: list[dict[str, Any]]) -> bool:
    match = rule.get("match") or {}
    if not isinstance(match, dict):
        return False
    source_link = str(items[0].get("source_link", "")) if items else ""
    if "source_link" in match and source_link != str(match["source_link"]):
        return False
    if "source_link_suffix" in match and not source_link.endswith(str(match["source_link_suffix"])):
        return False
    if not _regex_matches(match.get("source_link_regex"), source_link):
        return False
    collision_names = " ".join(str(item.get("name", "")) for item in items)
    if not _regex_matches(match.get("collision_name_regex"), collision_names):
        return False
    collision_files = " ".join(
        str((item.get("geometry") or {}).get("filename", ""))
        for item in items
        if isinstance(item.get("geometry"), dict)
    )
    if not _regex_matches(match.get("collision_filename_regex"), collision_files):
        return False
    visual_files = " ".join(str(visual.get("filename", "")) for visual in visuals)
    return _regex_matches(match.get("visual_filename_regex"), visual_files)


def _selected_rule(rules: list[dict[str, Any]], items: list[dict], visuals: list[dict[str, Any]]) -> dict[str, Any] | None:
    for rule in rules:
        if not isinstance(rule, dict) or not bool(rule.get("enabled", True)):
            continue
        if _rule_matches(rule, items, visuals):
            return rule
    return None


def has_instrument_collisions(manifest: dict[str, Any]) -> bool:
    """Validate coverage by generated primitives or bounded convex hulls."""
    return _has_instrument_collisions(manifest)


def _has_instrument_collisions(manifest):
    collision = manifest.get("collision")
    if not isinstance(collision, dict) or not isinstance(collision.get("items"), list):
        return False
    items = [item for item in collision["items"] if isinstance(item, dict)]
    required_suffixes = (
        "roll_link",
        "wrist_pitch_link",
        "wrist_yaw_link",
        "jaw_1_link",
        "jaw_2_link",
    )
    for suffix in required_suffixes:
        link_items = [
            item for item in items
            if str(item.get("source_link", "")).endswith(suffix)
        ]
        if not link_items:
            return False
        if any(
            not isinstance(item.get("geometry"), dict)
            or not (
                item["geometry"].get("type") == "cylinder" and item.get("primitive_profile")
                or item["geometry"].get("type") == "convex_mesh"
                and item.get("collision_profile") and item["geometry"].get("points")
                and item["geometry"].get("faces")
            )
            for item in link_items
        ):
            return False
    return True


def _fit_axis_index(axis: str, points: np.ndarray) -> int:
    if axis.lower() == "auto":
        extents = points.max(axis=0) - points.min(axis=0)
        return int(np.argmax(extents))
    try:
        return "xyz".index(axis.lower())
    except ValueError as exc:
        raise ValueError(f"unsupported primitive fit axis {axis!r}; expected x, y, z, or auto") from exc


def _cylinder_alignment(axis_index: int) -> np.ndarray:
    if axis_index == 0:
        return np.array([[0.0, 0.0, 1.0],
                         [0.0, 1.0, 0.0],
                         [-1.0, 0.0, 0.0]])
    if axis_index == 1:
        return np.array([[1.0, 0.0, 0.0],
                         [0.0, 0.0, 1.0],
                         [0.0, -1.0, 0.0]])
    return np.eye(3)


def _slice_bounds(points: np.ndarray, axis_index: int, slices: int) -> list[np.ndarray]:
    minimum = float(points[:, axis_index].min())
    maximum = float(points[:, axis_index].max())
    if slices <= 1 or math.isclose(minimum, maximum):
        return [points]
    edges = np.linspace(minimum, maximum, slices + 1)
    result: list[np.ndarray] = []
    for index in range(slices):
        low = edges[index]
        high = edges[index + 1]
        if index == slices - 1:
            mask = (points[:, axis_index] >= low) & (points[:, axis_index] <= high)
        else:
            mask = (points[:, axis_index] >= low) & (points[:, axis_index] < high)
        subset = points[mask]
        if len(subset):
            result.append(subset)
    return result


def _primitive_geometry(
    fit: str,
    points: np.ndarray,
    axis_index: int,
    margin: float,
    min_size: float,
) -> dict[str, Any]:
    lower = points.min(axis=0) - margin
    upper = points.max(axis=0) + margin
    size = np.maximum(upper - lower, min_size)
    if fit != "cylinder":
        raise ValueError(f"unsupported primitive fit {fit!r}; expected cylinder or vhacd")
    radial = [index for index in range(3) if index != axis_index]
    radius = float(max(size[radial[0]], size[radial[1]]) * 0.5)
    return {"type": "cylinder", "radius": radius, "length": float(size[axis_index])}


def _fit_visual_primitives(
    item_template: dict,
    visual: dict[str, Any],
    rule: dict[str, Any],
) -> list[dict]:
    points = _read_obj_vertices(visual["path"])
    if points.size == 0:
        return []
    points = points * np.asarray(visual["scale"], dtype=float)
    axis_index = _fit_axis_index(str(rule.get("axis", "auto")), points)
    fit = str(rule.get("fit", "cylinder")).lower()
    slices = max(1, int(rule.get("slices", 1)))
    margin = float(rule.get("margin", 0.0))
    min_size = float(rule.get("min_size", 1e-5))
    visual_transform = _transform_from_rpy(visual["origin_xyz"], visual["origin_rpy"])
    rotation = _cylinder_alignment(axis_index)

    fitted: list[dict] = []
    base_name = re.sub(r"[^A-Za-z0-9_]+", "_", str(rule.get("name", item_template.get("source_link", "primitive"))))
    for index, subset in enumerate(_slice_bounds(points, axis_index, slices)):
        lower = subset.min(axis=0)
        upper = subset.max(axis=0)
        center = (lower + upper) * 0.5
        primitive_transform = np.eye(4)
        primitive_transform[:3, :3] = rotation
        primitive_transform[:3, 3] = center
        origin_xyz, origin_rpy = _matrix_origin(visual_transform @ primitive_transform)
        fitted.append({
            "name": f"{base_name}_{index:02d}",
            "source_link": item_template["source_link"],
            "prim": item_template.get("prim", ""),
            "origin_xyz": origin_xyz,
            "origin_rpy": origin_rpy,
            "geometry": _primitive_geometry(fit, subset, axis_index, margin, min_size),
            "primitive_profile": str(rule.get("name", "")),
            "visual_filename": visual["filename"],
        })
    return fitted


def _items_by_source_link(items: list[dict]) -> list[tuple[str, list[dict]]]:
    groups: list[tuple[str, list[dict]]] = []
    by_link: dict[str, list[dict]] = {}
    for item in items:
        link = str(item.get("source_link", ""))
        if link not in by_link:
            by_link[link] = []
            groups.append((link, by_link[link]))
        by_link[link].append(item)
    return groups


def apply_collision_profile(
    manifest_path: str | Path,
    urdf_path: str | Path,
    model_root: str | Path,
    profile_path: str | Path,
) -> int:
    """Replace manifest collision mesh entries using collision profile rules."""

    profile_source = Path(profile_path).expanduser().resolve()
    profile = yaml.safe_load(profile_source.read_text(encoding="utf-8")) or {}
    collision_primitives = profile.get("collision_primitives", profile)
    if not isinstance(collision_primitives, dict) or not bool(collision_primitives.get("enabled", True)):
        return 0
    rules = collision_primitives.get("rules", [])
    if not isinstance(rules, list):
        raise ValueError(f"{profile_source}: collision_primitives.rules must be a list")

    manifest_source = Path(manifest_path)
    manifest = json.loads(manifest_source.read_text(encoding="utf-8"))
    collision = manifest.get("collision")
    if not isinstance(collision, dict) or not isinstance(collision.get("items"), list):
        raise ValueError(f"{manifest_source}: manifest has no collision.items data")

    visuals = _link_visuals(Path(urdf_path), Path(model_root))
    link_prims = {manifest.get("root_link", ""): ""}
    pending = list(manifest.get("all_joints", manifest.get("joints", [])))
    while pending:
        unresolved = []
        for joint in pending:
            if joint["parent"] not in link_prims:
                unresolved.append(joint)
                continue
            parent_prim = link_prims[joint["parent"]]
            link_prims[joint["child"]] = (
                parent_prim if joint["type"] == "fixed"
                else "/".join(part for part in (parent_prim, joint["child"]) if part)
            )
        if len(unresolved) == len(pending):
            break
        pending = unresolved
    groups = _items_by_source_link([item for item in collision["items"] if isinstance(item, dict)])
    collision_links = {link for link, _ in groups}
    groups.extend((link, []) for link in visuals if link not in collision_links and link in link_prims)
    updated_items: list[dict] = []
    replacements = 0
    for link, items in groups:
        link_visuals = visuals.get(link, [])
        template = items[0] if items else {"source_link": link, "prim": link_prims[link]}
        rule = _selected_rule(rules, items or [template], link_visuals)
        if rule is None:
            updated_items.extend(items)
            continue
        action = str(rule.get("action", "replace")).lower()
        if action != "replace":
            raise ValueError(f"{profile_source}: unsupported rule action {action!r}; expected replace")
        if not link_visuals:
            updated_items.extend(items)
            continue
        fitted: list[dict] = []
        for visual in link_visuals:
            if str(rule.get("fit", "cylinder")).lower() == "vhacd":
                from .collision_decomposition import fit_visual_hulls

                fitted.extend(fit_visual_hulls(template, visual, rule))
            else:
                fitted.extend(_fit_visual_primitives(template, visual, rule))
        if fitted:
            updated_items.extend(fitted)
            replacements += len(items)
        else:
            updated_items.extend(items)

    collision["items"] = updated_items
    collision["primitive_profile"] = str(profile_source)
    from .collision_decomposition import GENERATOR_VERSION, generation_inputs

    input_paths = [profile_source, *Path(model_root).rglob("*.xacro")]
    input_paths.extend(visual["path"] for link_visuals in visuals.values() for visual in link_visuals)
    collision["generation"] = {
        "generator_version": GENERATOR_VERSION,
        "inputs": generation_inputs(input_paths),
        "profile": profile,
    }
    hull_directory = manifest_source.parent / "collision_hulls"
    for item in updated_items:
        geometry = item.get("geometry", {})
        if geometry.get("type") != "convex_mesh":
            continue
        hull_directory.mkdir(exist_ok=True)
        hull_path = hull_directory / f"{item['name']}.obj"
        vertices = ["v " + " ".join(str(value) for value in point) for point in geometry["points"]]
        faces = ["f " + " ".join(str(index + 1) for index in face) for face in geometry["faces"]]
        hull_path.write_text("\n".join(vertices + faces) + "\n", encoding="utf-8")
        geometry["filename"] = hull_path.relative_to(manifest_source.parent).as_posix()
    manifest["format"] = max(int(manifest.get("format", 0)), 7)
    manifest_source.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return replacements

