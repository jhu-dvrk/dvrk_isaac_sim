import importlib.util
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


_SCRIPT = Path(__file__).parents[1] / "scripts" / "convert_dvrk_model.py"
sys.path.insert(0, str(_SCRIPT.parent))
_SPEC = importlib.util.spec_from_file_location("convert_dvrk_model", _SCRIPT)
assert _SPEC is not None and _SPEC.loader is not None
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)


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
