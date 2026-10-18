import json
from pathlib import Path

import numpy as np
import pytest

from dvrk_isaac_sim.collision_decomposition import (
    GENERATOR_VERSION, collision_cache_valid, generation_inputs, validate_hull,
)


def test_collision_cache_detects_changed_inputs(tmp_path):
    profile = tmp_path / "profile.yaml"
    profile.write_text("rules: []\n")
    manifest = {"collision": {"generation": {
        "generator_version": GENERATOR_VERSION, "inputs": generation_inputs([profile]),
    }}}
    assert collision_cache_valid(manifest, profile)
    profile.write_text("rules: [changed]\n")
    assert not collision_cache_valid(manifest, profile)
    profile.unlink()
    assert not collision_cache_valid(manifest, profile)


def test_hull_validation_rejects_degenerate_and_invalid_data():
    points = [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]]
    faces = [[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]]
    validate_hull(points, faces, 32)
    with pytest.raises(ValueError):
        validate_hull(np.zeros((4, 3)), faces, 32)
    with pytest.raises(ValueError):
        validate_hull(points, [[0, 1, 9]] * 4, 32)


def test_real_vhacd_worker_generates_bounded_meshes(tmp_path):
    pytest.importorskip("vhacdx")
    trimesh = pytest.importorskip("trimesh")
    import subprocess
    import sys

    mesh = tmp_path / "box.obj"
    trimesh.creation.box().export(mesh)
    output = tmp_path / "hulls.json"
    subprocess.run([
        sys.executable, "-m", "dvrk_isaac_sim.collision_decomposition", str(mesh), str(output),
        json.dumps({"maxConvexHulls": 2, "maxNumVerticesPerCH": 16, "resolution": 10000, "asyncACD": False}),
        "[0.01, 0.02, 0.03]",
    ], check=True, cwd=Path(__file__).resolve().parents[1], timeout=60)
    hulls = json.loads(output.read_text())
    assert 1 <= len(hulls) <= 2
    all_points = []
    for hull in hulls:
        points, faces = validate_hull(hull["points"], hull["faces"], 16)
        all_points.extend(points)
    np.testing.assert_allclose(np.ptp(all_points, axis=0), [0.01, 0.02, 0.03], rtol=0.05)


def test_convex_mesh_authors_geometry_without_visual_lookup():
    pytest.importorskip("pxr")
    from pxr import Usd, UsdGeom, UsdPhysics
    from dvrk_isaac_sim.usd_physics_links import PhysicsLinkSync

    stage = Usd.Stage.CreateInMemory()
    sync = PhysicsLinkSync.__new__(PhysicsLinkSync)
    sync._stage = stage
    sync._component_name = "PSM1"
    sync._visual_root = "Geometry/world"
    item = {
        "origin_xyz": [0.01, 0.02, 0.03], "origin_rpy": [0, 0, 0],
        "geometry": {"type": "convex_mesh", "points": [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]],
                     "faces": [[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]]},
    }
    _, _, offset = sync._create_flattened_link("/World/PSM1/PhysicsLinks/jaw", item, UsdGeom, UsdPhysics)
    np.testing.assert_allclose(offset[:3, 3], item["origin_xyz"])
    mesh = UsdGeom.Mesh(stage.GetPrimAtPath("/World/PSM1/PhysicsLinks/jaw/Collision"))
    assert UsdPhysics.MeshCollisionAPI(mesh).GetApproximationAttr().Get() == "convexHull"
    np.testing.assert_allclose(mesh.GetPointsAttr().Get(), item["geometry"]["points"])