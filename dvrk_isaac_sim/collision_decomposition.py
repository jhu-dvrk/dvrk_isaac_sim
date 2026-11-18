"""Offline V-HACD generation through an isolated, pinned Python binding."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np


BACKEND_VERSION = "0.0.10"
GENERATOR_VERSION = 1


def file_digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def generation_inputs(paths):
    return {str(Path(path).resolve()): file_digest(path) for path in sorted(set(paths))}


def collision_cache_valid(manifest, profile_path):
    generation = manifest.get("collision", {}).get("generation", {})
    if generation.get("generator_version") != GENERATOR_VERSION:
        return False
    inputs = generation.get("inputs", {})
    profile = str(Path(profile_path).resolve())
    if not inputs or profile not in inputs:
        return False
    try:
        return all(file_digest(path) == digest for path, digest in inputs.items())
    except OSError:
        return False


def validate_hull(points, faces, max_vertices):
    points = np.asarray(points, dtype=float)
    faces = np.asarray(faces, dtype=int)
    if points.ndim != 2 or points.shape[1] != 3 or not 4 <= len(points) <= max_vertices:
        raise ValueError("V-HACD returned an invalid vertex count")
    if faces.ndim != 2 or faces.shape[1] != 3 or len(faces) < 4:
        raise ValueError("V-HACD returned invalid triangle faces")
    if not np.isfinite(points).all() or faces.min() < 0 or faces.max() >= len(points):
        raise ValueError("V-HACD returned nonfinite vertices or out-of-range indices")
    centered = points - points.mean(axis=0)
    triangles = centered[faces]
    volume = abs(float(np.einsum("ij,ij->i", triangles[:, 0], np.cross(triangles[:, 1], triangles[:, 2])).sum())) / 6.0
    if not volume > 1e-18:
        raise ValueError("V-HACD returned a zero-volume hull")
    return points, faces


def fit_visual_hulls(template, visual, rule):
    from .newton_runtime import resolve_newton_python

    settings = {
        "maxConvexHulls": int(rule.get("max_hulls", 4)),
        "maxNumVerticesPerCH": int(rule.get("max_vertices", 32)),
        "resolution": int(rule.get("resolution", 100000)),
        "minimumVolumePercentErrorAllowed": float(rule.get("volume_error_percent", 1.0)),
        "asyncACD": False,
    }
    if not 1 <= settings["maxConvexHulls"] <= 32 or not 4 <= settings["maxNumVerticesPerCH"] <= 128:
        raise ValueError("V-HACD budgets must be 1..32 hulls and 4..128 vertices")
    python = os.environ.get("DVRK_COLLISION_PYTHON") or resolve_newton_python()
    environment = dict(os.environ)
    environment.pop("LD_PRELOAD", None)
    environment.pop("LD_LIBRARY_PATH", None)
    environment["PYTHONPATH"] = str(Path(__file__).resolve().parents[1])
    with tempfile.TemporaryDirectory(prefix="dvrk-vhacd-") as temporary:
        output = Path(temporary) / "hulls.json"
        subprocess.run(
            [str(python), "-m", __name__, str(visual["path"]), str(output),
             json.dumps(settings), json.dumps(visual["scale"])],
            check=True, env=environment, timeout=300,
        )
        hulls = json.loads(output.read_text(encoding="utf-8"))
    if not 1 <= len(hulls) <= settings["maxConvexHulls"]:
        raise ValueError("V-HACD returned an invalid hull count")
    result = []
    for index, hull in enumerate(hulls):
        points, faces = validate_hull(hull["points"], hull["faces"], settings["maxNumVerticesPerCH"])
        result.append({
            "name": f"{template['source_link']}_{visual['index']}_hull_{index:02d}",
            "source_link": template["source_link"], "prim": template.get("prim", ""),
            "origin_xyz": visual["origin_xyz"], "origin_rpy": visual["origin_rpy"],
            "geometry": {"type": "convex_mesh", "points": points.tolist(), "faces": faces.tolist()},
            "collision_profile": str(rule.get("name", "")),
            "visual_filename": visual["filename"],
            "generator": {"backend": "vhacdx", "version": BACKEND_VERSION, "settings": settings},
        })
    return result


def main():
    import importlib.metadata
    import trimesh
    import vhacdx

    if importlib.metadata.version("vhacdx") != BACKEND_VERSION:
        raise RuntimeError(f"Collision generation requires vhacdx=={BACKEND_VERSION}")
    source, output, settings, scale = sys.argv[1:]
    mesh = trimesh.load_mesh(source, process=True)
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"Expected a triangle mesh: {source}")
    mesh.apply_scale(json.loads(scale))
    if not np.isfinite(mesh.vertices).all() or not len(mesh.faces):
        raise ValueError(f"Invalid source mesh: {source}")
    settings = json.loads(settings)
    hulls = vhacdx.compute_vhacd(
        np.asarray(mesh.vertices, dtype=np.float64),
        np.asarray(mesh.faces, dtype=np.uint32).reshape(-1), **settings,
    )
    result = []
    for points, faces in hulls:
        points, faces = validate_hull(points, np.asarray(faces).reshape(-1, 3), settings["maxNumVerticesPerCH"])
        result.append({"points": points.tolist(), "faces": faces.tolist()})
    Path(output).write_text(json.dumps(result), encoding="utf-8")


if __name__ == "__main__":
    main()