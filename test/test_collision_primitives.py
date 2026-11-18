from pathlib import Path
import json

from dvrk_isaac_sim.collision_primitives import (
  apply_collision_profile,
  has_instrument_collisions,
)
from dvrk_isaac_sim.urdf_kinematics import write_kinematics_manifest


def _write_box_obj(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    vertices = [
        (-1.0, -0.2, -0.1),
        (-1.0, -0.2, 0.1),
        (-1.0, 0.2, -0.1),
        (-1.0, 0.2, 0.1),
        (1.0, -0.2, -0.1),
        (1.0, -0.2, 0.1),
        (1.0, 0.2, -0.1),
        (1.0, 0.2, 0.1),
    ]
    path.write_text(
        "\n".join(f"v {x} {y} {z}" for x, y, z in vertices) + "\n",
        encoding="utf-8",
    )


def _write_urdf(path: Path) -> None:
    path.write_text(
        """
<robot name="psm1_test">
  <link name="world"/>
  <link name="PSM1_roll_link">
    <visual>
      <origin xyz="0.1 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://dvrk_model/meshes/instruments/roll/test_roll.obj" scale="0.01 0.01 0.01"/>
      </geometry>
    </visual>
    <collision name="roll_collision_0">
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://dvrk_model/meshes/instruments/roll/hull_0.obj"/>
      </geometry>
    </collision>
    <collision name="roll_collision_1">
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://dvrk_model/meshes/instruments/roll/hull_1.obj"/>
      </geometry>
    </collision>
  </link>
  <link name="PSM1_tool_tip_link"/>

  <joint name="roll" type="revolute">
    <parent link="world"/>
    <child link="PSM1_roll_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" velocity="1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  <joint name="tool_tip" type="fixed">
    <parent link="PSM1_roll_link"/>
    <child link="PSM1_tool_tip_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
</robot>
""".strip()
        + "\n",
        encoding="utf-8",
    )


def test_collision_profile_replaces_link_mesh_hulls_with_visual_primitive_slices(tmp_path: Path):
    model_root = tmp_path / "dvrk_model"
    _write_box_obj(model_root / "meshes/instruments/roll/test_roll.obj")
    urdf = tmp_path / "PSM1.urdf"
    _write_urdf(urdf)
    manifest_path = write_kinematics_manifest(urdf, tmp_path / "kinematics.json", "PSM1")
    profile = tmp_path / "profile.yaml"
    profile.write_text(
        """
collision_primitives:
  enabled: true
  rules:
    - name: roll_test
      action: replace
      match:
        source_link_suffix: roll_link
      fit: cylinder
      axis: x
      slices: 2
      margin: 0.001
""".strip()
        + "\n",
        encoding="utf-8",
    )

    replaced = apply_collision_profile(manifest_path, urdf, model_root, profile)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    items = manifest["collision"]["items"]

    assert replaced == 2
    assert manifest["format"] == 7
    assert len(items) == 2
    assert {item["geometry"]["type"] for item in items} == {"cylinder"}
    assert all("filename" not in item["geometry"] for item in items)
    assert [item["name"] for item in items] == ["roll_test_00", "roll_test_01"]
    assert all(item["source_link"] == "PSM1_roll_link" for item in items)
    assert all(item["primitive_profile"] == "roll_test" for item in items)


def test_instrument_collision_check_requires_profiled_links():
    manifest = {
        "collision": {
            "items": [
                {
                    "source_link": "PSM1_roll_link",
                    "geometry": {"type": "cylinder"},
                    "primitive_profile": "roll_shaft",
                },
                *[
                    {
                        "source_link": f"PSM1_{suffix}",
                        "geometry": {
                            "type": "convex_mesh",
                            "points": [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]],
                            "faces": [[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]],
                        },
                        "collision_profile": suffix,
                    }
                    for suffix in (
                        "wrist_pitch_link",
                        "wrist_yaw_link",
                        "jaw_1_link",
                        "jaw_2_link",
                    )
                ],
            ]
        }
    }

    assert has_instrument_collisions(manifest)
    manifest["collision"]["items"].pop()
    assert not has_instrument_collisions(manifest)
