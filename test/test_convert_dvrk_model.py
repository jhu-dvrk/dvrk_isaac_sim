import importlib.util
import json
from pathlib import Path
import sys
import pytest
import xml.etree.ElementTree as ET


_SCRIPT = Path(__file__).parents[1] / "scripts" / "convert_dvrk_model.py"
sys.path.insert(0, str(_SCRIPT.parent))
_SPEC = importlib.util.spec_from_file_location("convert_dvrk_model", _SCRIPT)
assert _SPEC is not None and _SPEC.loader is not None
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)


def test_failed_asset_build_keeps_previous_asset(tmp_path):
  destination = tmp_path / "PSM1_420006"
  destination.mkdir()
  (destination / "old.usda").write_text("previous")
  with pytest.raises(RuntimeError, match="conversion failed"):
    with _MODULE._asset_build_directory(tmp_path, destination.name, True) as staging:
      (staging / "partial.usda").write_text("partial")
      raise RuntimeError("conversion failed")
  assert (destination / "old.usda").read_text() == "previous"
  assert not (destination / "partial.usda").exists()


def test_successful_asset_build_publishes_only_complete_asset(tmp_path):
  destination = tmp_path / "PSM1_420006"
  destination.mkdir()
  (destination / "old.usda").write_text("previous")
  with _MODULE._asset_build_directory(tmp_path, destination.name, True) as staging:
    (staging / "new.usda").write_text("complete")
    assert (destination / "old.usda").exists()
  assert (destination / "new.usda").read_text() == "complete"
  assert not (destination / "old.usda").exists()


def test_strip_physics_handles_binary_usd_layers(tmp_path):
  pytest.importorskip("pxr")
  from pxr import Usd, UsdGeom, UsdPhysics

  path = tmp_path / "geometry.usd"
  stage = Usd.Stage.CreateNew(str(path))
  shape = UsdGeom.Mesh.Define(stage, "/Shape")
  UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
  UsdPhysics.MeshCollisionAPI.Apply(shape.GetPrim()).CreateApproximationAttr().Set("convexHull")
  stage.GetRootLayer().Save()
  assert path.read_bytes().startswith(b"PXR-USDC")

  _MODULE._strip_physics(str(path))

  assert not shape.GetPrim().HasAPI(UsdPhysics.CollisionAPI)
  assert shape.GetPrim().HasAPI(UsdPhysics.MeshCollisionAPI)
  assert path.read_bytes().startswith(b"PXR-USDC")


def test_strip_physics_api_schemas_retains_mesh_collision_api():
    line = 'prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMeshCollisionAPI"]\n'

    assert _MODULE._strip_physics_api_schemas(line) == 'prepend apiSchemas = ["PhysicsMeshCollisionAPI"]\n'


def test_strip_physics_api_schemas_removes_physics_only_entry():
    assert _MODULE._strip_physics_api_schemas('prepend apiSchemas = ["PhysicsRigidBodyAPI"]\n') is None


def test_normalize_geometric_collision_meshes_uses_visual_frame(tmp_path: Path):
    model_root = tmp_path / "dvrk_model"
    mesh_dir = model_root / "meshes" / "instruments" / "roll" / "4670"
    mesh_dir.mkdir(parents=True)
    (mesh_dir / "roll_4670_collision_geometric.obj").write_text("# test\n", encoding="utf-8")
    urdf = tmp_path / "PSM1.urdf"
    urdf.write_text(
        """<robot name="test">
  <link name="PSM1_roll_link">
    <visual>
      <origin xyz="0 0 0.02975" rpy="-1.57079632679 0 1.57079632679"/>
      <geometry>
        <mesh filename="package://dvrk_model/meshes/instruments/roll/4670/roll_4670.obj" scale="0.1 0.1 0.1"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.02975" rpy="-1.57079632679 0 0"/>
      <geometry>
        <mesh filename="package://dvrk_model/meshes/instruments/roll/4670/roll_4670_collision.obj" scale="1 1 1"/>
      </geometry>
    </collision>
  </link>
</robot>
""",
        encoding="utf-8",
    )

    assert _MODULE._normalize_geometric_collision_meshes(urdf, model_root) == 2

    collision = ET.parse(urdf).getroot().find("link/collision")
    assert collision is not None
    origin = collision.find("origin")
    assert origin is not None
    assert origin.attrib["xyz"] == "0 0 0.02975"
    assert origin.attrib["rpy"] == "-1.57079632679 0 1.57079632679"
    mesh = collision.find("geometry/mesh")
    assert mesh is not None
    assert mesh.attrib["filename"].endswith("/roll_4670_collision_geometric.obj")
    assert mesh.attrib["scale"] == "0.1 0.1 0.1"


def test_normalize_geometric_collision_meshes_handles_stl_counterpart(tmp_path: Path):
    model_root = tmp_path / "dvrk_model"
    mesh_dir = model_root / "meshes" / "instruments" / "wrist_pitch" / "0091"
    mesh_dir.mkdir(parents=True)
    (mesh_dir / "wrist_pitch_0091_collision_geometric.STL").write_text("solid test\nendsolid\n", encoding="utf-8")
    urdf = tmp_path / "PSM1.urdf"
    urdf.write_text(
        """<robot name="test">
  <link name="PSM1_wrist_pitch_link">
    <visual>
      <origin xyz="0.0295 0 0" rpy="0 1.57079632679 1.57079632679"/>
      <geometry>
        <mesh filename="package://dvrk_model/meshes/instruments/wrist_pitch/0091/wrist_pitch_0091.obj" scale="0.1 0.1 0.1"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0.0295 0 0" rpy="0 1.57079632679 1.57079632679"/>
      <geometry>
        <mesh filename="package://dvrk_model/meshes/instruments/wrist_pitch/0091/wrist_pitch_0091_collision.obj" scale="0.1 0.1 0.1"/>
      </geometry>
    </collision>
  </link>
</robot>
""",
        encoding="utf-8",
    )

    assert _MODULE._normalize_geometric_collision_meshes(urdf, model_root) == 1

    mesh = ET.parse(urdf).getroot().find("link/collision/geometry/mesh")
    assert mesh is not None
    assert mesh.attrib["filename"].endswith("/wrist_pitch_0091_collision_geometric.STL")


def test_collision_mesh_source_links_only_reports_mesh_entries(tmp_path: Path):
    manifest = tmp_path / "kinematics.json"
    manifest.write_text(
        json.dumps({
            "collision": {
                "items": [
                    {"source_link": "PSM1_roll_link", "geometry": {"type": "mesh"}},
                    {"source_link": "PSM1_jaw_1_link", "geometry": {"type": "box"}},
                    {"source_link": "", "geometry": {"type": "mesh"}},
                ]
            }
        }),
        encoding="utf-8",
    )

    assert _MODULE._collision_mesh_source_links(manifest) == {"PSM1_roll_link"}


def test_strip_collision_elements_removes_only_selected_links(tmp_path: Path):
    urdf = tmp_path / "PSM1.urdf"
    urdf.write_text(
        """<robot name="test">
  <link name="PSM1_roll_link">
    <visual/>
    <collision name="remove_a"/>
    <collision name="remove_b"/>
  </link>
  <link name="PSM1_jaw_1_link">
    <visual/>
    <collision name="keep"/>
  </link>
</robot>
""",
        encoding="utf-8",
    )

    assert _MODULE._strip_collision_elements(urdf, {"PSM1_roll_link"}) == 2

    root = ET.parse(urdf).getroot()
    roll = root.find("link[@name='PSM1_roll_link']")
    jaw = root.find("link[@name='PSM1_jaw_1_link']")
    assert roll is not None and roll.findall("collision") == []
    assert jaw is not None
    assert [collision.attrib["name"] for collision in jaw.findall("collision")] == ["keep"]
