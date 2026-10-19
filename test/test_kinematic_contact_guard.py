import json
from pathlib import Path

import numpy as np
import pytest

from dvrk_isaac_sim.kinematic_contact_guard import KinematicContactGuard, _geometry_points
from dvrk_isaac_sim.scene import SceneProp


class _TranslationChain:
    def forward_all_links(self, position, joint_names):
        transform = np.eye(4)
        transform[:3, 3] = position
        return {"tool": transform}


def _cone_guard(tmp_path, geometry=None):
    path = tmp_path / "cone_manifest.json"
    path.write_text(json.dumps({"collision": {"items": [{
        "source_link": "tool", "geometry": geometry or {"type": "box", "size": [0.004] * 3}
    }]}}), encoding="ascii")
    cone = SceneProp("cone", "cone_bowl", (0, 0, -0.02), (0, 0, 0, 1),
                     (0.20, 0.20, 0.06), True, False, guard_kinematic_tools=True)
    return KinematicContactGuard(path, _TranslationChain(), [cone], static_only=True)


def test_cone_guard_allows_entry_and_motion_inside(tmp_path):
    guard = _cone_guard(tmp_path)
    assert guard.enabled
    for start, end in [([0, 0, 0.08], [0, 0, 0.01]),
                       ([0, 0, 0.01], [0.02, 0.01, 0.01]),
                       ([0, 0, 0.01], [0, 0, 0.08])]:
        np.testing.assert_allclose(guard.clamp(("x", "y", "z"), np.array(start), np.array(end)), end)


@pytest.mark.parametrize("target", [[0.08, 0, 0.01], [-0.08, 0, 0.01],
                                   [0, 0.08, 0.01], [0, 0, -0.05]])
def test_cone_guard_stops_wall_and_tip_crossings(tmp_path, target):
    guard = _cone_guard(tmp_path)
    start = np.array([0, 0, 0.01])
    target = np.asarray(target)
    stopped = guard.clamp(("x", "y", "z"), start, target)
    assert np.linalg.norm(stopped - start) > 0.001
    assert np.linalg.norm(stopped - start) < np.linalg.norm(target - start)
    np.testing.assert_allclose(guard.clamp(("x", "y", "z"), stopped, start), start)


def test_cone_guard_loads_embedded_wrist_and_jaw_hulls(tmp_path):
    geometry = {"type": "convex_mesh", "points": [[-0.002, 0, 0], [0.002, 0, 0],
                                                       [0, -0.002, 0], [0, 0, 0.002]]}
    guard = _cone_guard(tmp_path, geometry)
    assert guard.enabled
    assert len(guard._clouds[0].local_points) == 4
    start = np.array([0, 0, 0.01])
    target = np.array([0.08, 0, 0.01])
    assert guard.clamp(("x", "y", "z"), start, target)[0] < 0.05


def test_cone_guard_applies_robot_base_transform(tmp_path):
    guard = _cone_guard(tmp_path)
    guard._base_transform[:3, 3] = [0.2, 0.1, 0.3]
    start = np.array([-0.2, -0.1, -0.29])
    target = start + [0.08, 0, 0]
    stopped = guard.clamp(("x", "y", "z"), start, target)
    assert start[0] < stopped[0] < target[0]


def test_cone_guard_blocks_jaw_opening_at_fixed_arm_pose(tmp_path):
    class JawChain:
        def forward_all_links(self, position, joint_names):
            assert joint_names[-1] == "jaw"
            transform = np.eye(4)
            transform[:3, 3] = [position[-1], 0, 0.01]
            return {"tool": transform}

    guard = _cone_guard(tmp_path)
    guard._chain = JawChain()
    stopped = guard.clamp(("arm", "jaw"), np.array([0, 0.0]), np.array([0, 0.08]))
    assert stopped[0] == 0
    assert 0 < stopped[1] < 0.05


def test_cone_guard_stops_long_shaft_even_when_link_origin_is_above_rim(tmp_path):
    guard = _cone_guard(tmp_path, {"type": "cylinder", "radius": 0.004, "length": 0.12})
    start = np.array([0, 0, 0.06])
    target = np.array([0.08, 0, 0.06])
    stopped = guard.clamp(("x", "y", "z"), start, target)
    assert 0.01 < stopped[0] < 0.04
    np.testing.assert_allclose(guard.clamp(("x", "y", "z"), stopped, start), start)


def test_geometry_points_samples_cylinder_surface():
    points = _geometry_points({"type": "cylinder", "radius": 0.004, "length": 0.12})

    assert points is not None
    np.testing.assert_allclose(points[:, 2].min(), -0.06)
    np.testing.assert_allclose(points[:, 2].max(), 0.06)
    assert np.isclose(np.linalg.norm(points[:, :2], axis=1).max(), 0.004)


def test_geometry_points_samples_capsule_surface():
    points = _geometry_points({"type": "capsule", "radius": 0.004, "length": 0.12})

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


def test_contact_guard_can_leave_dynamic_props_for_physics(tmp_path: Path):
    static_base = SceneProp(
        name="tray_base",
        kind="table",
        position=(0.0, 0.0, 0.0),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        size=(0.3, 0.3, 0.01),
        static=True,
        dynamic=False,
    )
    dynamic_ball = SceneProp(
        name="ball",
        kind="sphere",
        position=(0.0, 0.0, 0.02),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        size=(0.03, 0.03, 0.03),
        static=False,
        dynamic=True,
    )
    manifest_path = tmp_path / "empty_collision_manifest.json"
    manifest_path.write_text('{"collision": {"items": []}}', encoding="utf-8")

    guard = KinematicContactGuard(
        manifest_path, object(), [static_base, dynamic_ball], static_only=True
    )

    assert [obstacle.name for obstacle in guard._obstacles] == ["tray_base"]
