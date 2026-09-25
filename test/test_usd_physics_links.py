import numpy as np

import pytest

from dvrk_isaac_sim.urdf_kinematics import _rotation, _transform
from dvrk_isaac_sim.usd_physics_links import (
    _candidate_collision_prim,
    _collision_body_name,
    _relative_offset_from_stage_transforms,
    _require_collision_candidate,
)


class _FakeAttr:
    def __init__(self, value=None, valid=False, authored=False):
        self._value = value
        self._valid = valid
        self._authored = authored

    def IsValid(self):
        return self._valid

    def HasAuthoredValueOpinion(self):
        return self._authored

    def Get(self):
        return self._value


class _FakePrim:
    def __init__(self, name: str, path: str, is_gprim: bool = True):
        self._name = name
        self._path = path
        self._is_gprim = is_gprim

    def IsActive(self):
        return True

    def IsA(self, schema):
        return self._is_gprim and schema is _FakeUsdGeom.Gprim

    def GetName(self):
        return self._name

    def GetPath(self):
        return self._path

    def GetAttribute(self, name: str):
        return _FakeAttr(valid=False, authored=False)


class _FakeRoot:
    def __init__(self, path: str, children):
        self._path = path
        self._children = list(children)

    def GetPath(self):
        return self._path


class _FakeUsd:
    @staticmethod
    def PrimRange(prim):
        return prim._children


class _FakeUsdGeom:
    class Gprim:
        pass


def test_relative_offset_from_stage_transforms_matches_candidate_world_transform():
    link_world = _transform(_rotation(np.array([0.0, 0.0, 1.0]), np.pi / 2.0), [0.4, -0.2, 0.1])
    expected_offset = _transform(_rotation(np.array([1.0, 0.0, 0.0]), 0.3), [0.02, 0.03, -0.04])
    candidate_world = link_world @ expected_offset

    offset = _relative_offset_from_stage_transforms(link_world.T, candidate_world.T)

    np.testing.assert_allclose(offset, expected_offset, atol=1e-12)
    np.testing.assert_allclose(link_world @ offset, candidate_world, atol=1e-12)


def test_candidate_collision_prim_warns_on_ambiguous_named_matches(capsys):
    prims = [
        _FakePrim("wrist_yaw_shaft_collision", "/World/shaft"),
        _FakePrim("wrist_yaw_shaft_tip_collision", "/World/shaft_tip"),
    ]
    root = _FakeRoot("/World", prims)
    item = {"name": "wrist_yaw_shaft", "source_link": "wrist_yaw_shaft"}

    result = _candidate_collision_prim(root, _FakeUsd, _FakeUsdGeom, item)

    assert result is prims[0]
    assert "ambiguous collision match" in capsys.readouterr().out


def test_candidate_collision_prim_prefers_geometry_filename_hint_over_descendant_tip():
    prims = [
        _FakePrim("tip_006_1_collision", "/World/roll/wrist/tip_006_1_collision"),
        _FakePrim("roll_4670_collision", "/World/roll/roll_4670_collision"),
    ]
    root = _FakeRoot("/World/roll", prims)
    item = {
        "name": "PSM1_roll_link_collision_0",
        "source_link": "PSM1_roll_link",
        "geometry": {"filename": "package://dvrk_model/meshes/instruments/roll/4670/roll_4670_collision.obj"},
    }

    result = _candidate_collision_prim(root, _FakeUsd, _FakeUsdGeom, item)

    assert result is prims[1]


def test_candidate_collision_prim_skips_blocked_descendant_link_subtrees():
    prims = [
        _FakePrim("tip_006_1_collision", "/World/wrist_yaw/PSM1_jaw_1_link/tip_006_1_collision"),
        _FakePrim("wrist_yaw_collision", "/World/wrist_yaw/wrist_yaw_collision"),
    ]
    root = _FakeRoot("/World/wrist_yaw", prims)
    item = {
        "name": "PSM1_wrist_yaw_link_collision_0",
        "source_link": "PSM1_wrist_yaw_link",
    }

    result = _candidate_collision_prim(
        root,
        _FakeUsd,
        _FakeUsdGeom,
        item,
        blocked_paths=("/World/wrist_yaw/PSM1_jaw_1_link",),
    )

    assert result is prims[1]


def test_candidate_collision_prim_matches_imported_collision_xform_piece():
    prims = [
        _FakePrim("tip_006_1", "/World/jaw_1/tip_006_1", is_gprim=False),
        _FakePrim("jaw_1_collision_00", "/World/jaw_1/jaw_1_collision_00", is_gprim=False),
        _FakePrim("jaw_1_collision_01", "/World/jaw_1/jaw_1_collision_01", is_gprim=False),
    ]
    root = _FakeRoot("/World/jaw_1", prims)
    item = {
        "name": "jaw_1_collision_01",
        "source_link": "PSM1_jaw_1_link",
        "geometry": {"filename": "package://dvrk_model/meshes/instruments/tip/006/collision_hulls/tip_006_1_convex_hull_01.obj"},
    }

    result = _candidate_collision_prim(root, _FakeUsd, _FakeUsdGeom, item)

    assert result is prims[2]


def test_candidate_collision_prim_matches_imported_filename_stem_xform():
    prims = [
        _FakePrim("roll_4670", "/World/roll/roll_4670", is_gprim=False),
        _FakePrim(
            "roll_4670_collision_convex_cylinder",
            "/World/roll/roll_4670_collision_convex_cylinder",
            is_gprim=False,
        ),
    ]
    root = _FakeRoot("/World/roll", prims)
    item = {
        "name": "PSM1_roll_link_collision_0",
        "source_link": "PSM1_roll_link",
        "geometry": {
            "filename": "package://dvrk_model/meshes/instruments/roll/4670/roll_4670_collision_convex_cylinder.obj"
        },
    }

    result = _candidate_collision_prim(root, _FakeUsd, _FakeUsdGeom, item)

    assert result is prims[1]


def test_require_collision_candidate_rejects_unmapped_psm_link():
    with pytest.raises(RuntimeError, match="PSM1_pitch_link") as error:
        _require_collision_candidate(
            None,
            {"source_link": "PSM1_pitch_link"},
            "/World/PSM1/Geometry/world/PSM1_pitch_link",
        )

    assert "cannot create flattened collision body" in str(error.value)


def test_collision_body_name_preserves_collision_item_identity():
    assert (
        _collision_body_name({"name": "jaw_1_collision_3", "source_link": "PSM1_jaw_1_link"}, 7)
        == "jaw_1_collision_3"
    )
    assert (
        _collision_body_name({"name": "jaw 1 collision/3", "source_link": "PSM1_jaw_1_link"}, 7)
        == "jaw_1_collision_3"
    )
    assert _collision_body_name({"source_link": "PSM1_jaw_1_link"}, 7) == "PSM1_jaw_1_link_collision_7"
