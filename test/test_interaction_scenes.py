from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import pytest

from dvrk_simulator_base.scene import SceneResolver, load_scene_config


@pytest.mark.parametrize(
    ("name", "expected_objects"),
    [
        ("test_cube", {"test_table", "test_cube"}),
        ("ball_tray", {"cone_bowl", "ball_01", "ball_02_h"}),
        ("match_row", {"match_table", "match_left", "match_center", "match_right"}),
        ("peg_transfer", {"peg_transfer_board", "triangle_orange_1", "triangle_green_3"}),
    ],
)
def test_interaction_scene_loads_objects(name, expected_objects):
    scene_root = Path(__file__).resolve().parents[1] / "share" / "scenes"
    arms_root = Path(get_package_share_directory("dvrk_arm_description")) / "arms"
    resolver = SceneResolver((scene_root,))

    scene = load_scene_config(
        scene_root / f"{name}.yaml",
        robot_config_root=arms_root,
        resolver=resolver,
    )

    assert scene.name.endswith(name)
    assert {item.name for item in scene.objects} >= expected_objects
    assert all(robot.name in {"PSM1", "ECM"} for robot in scene.robots)
    assert scene.camera.mode == "off"
    for item in scene.objects:
        if item.name in expected_objects:
            assert item.asset if name == "peg_transfer" else item.shape


@pytest.mark.parametrize("name", ["test_cube", "ball_tray", "match_row", "peg_transfer"])
def test_full_cart_with_interaction_objects_preserves_robots_and_camera(name):
    scene_root = Path(__file__).resolve().parents[1] / "share" / "scenes"
    arms_root = Path(get_package_share_directory("dvrk_arm_description")) / "arms"
    resolver = SceneResolver((scene_root,))
    cart_path = scene_root / "ECM_PSM1_PSM2_PSM3_stereo_rtsp.yaml"
    cart = load_scene_config(cart_path, robot_config_root=arms_root, resolver=resolver)
    scene = load_scene_config(
        [cart_path, scene_root / f"{name}_objects.yaml"],
        robot_config_root=arms_root,
        resolver=resolver,
    )
    standalone = load_scene_config(
        scene_root / f"{name}.yaml", robot_config_root=arms_root, resolver=resolver,
    )

    assert [robot.name for robot in scene.robots] == [robot.name for robot in cart.robots]
    assert scene.camera == cart.camera
    assert scene.objects == standalone.objects
