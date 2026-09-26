import json
from pathlib import Path

import numpy as np

from dvrk_isaac_sim.kinematic_contact_guard import KinematicContactGuard, _geometry_points
from dvrk_isaac_sim.scene import SceneProp


def test_geometry_points_samples_cylinder_surface():
    points = _geometry_points({"type": "cylinder", "radius": 0.004, "length": 0.12})

    assert points is not None
    np.testing.assert_allclose(points[:, 2].min(), -0.06)
    np.testing.assert_allclose(points[:, 2].max(), 0.06)
    assert np.isclose(np.linalg.norm(points[:, :2], axis=1).max(), 0.004)


def test_contact_guard_loads_primitive_collision_items(tmp_path: Path):
    manifest = {
        "collision": {
            "items": [
                {
                    "source_link": "PSM1_roll_link",
                    "origin_xyz": [1.0, 2.0, 3.0],
                    "origin_rpy": [0.0, 0.0, 0.0],
                    "geometry": {"type": "cylinder", "radius": 0.004, "length": 0.12},
                }
            ]
        }
    }
    manifest_path = tmp_path / "kinematics.json"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    prop = SceneProp(
        name="test_cube",
        kind="cube",
        position=(0.0, 0.0, 0.0),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        size=(0.1, 0.1, 0.1),
        static=True,
        dynamic=False,
    )

    guard = KinematicContactGuard(manifest_path, object(), [prop])

    assert guard.enabled
    assert len(guard._clouds) == 1
    cloud = guard._clouds[0]
    assert cloud.source_link == "PSM1_roll_link"
    np.testing.assert_allclose(cloud.local_points[:, 0].mean(), 1.0, atol=1e-4)
    np.testing.assert_allclose(cloud.local_points[:, 2].min(), 2.94)
    np.testing.assert_allclose(cloud.local_points[:, 2].max(), 3.06)
