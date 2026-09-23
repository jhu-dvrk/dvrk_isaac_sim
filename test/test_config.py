from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory

from dvrk_arm_description import load_robot_config
from dvrk_isaac_sim.scene import (
    available_scene_names, available_scene_paths, load_scene,
    load_simulator_config, resolve_scene_path,
)


ROOT = Path(__file__).parents[1]
BASE_ARMS = Path(get_package_share_directory("dvrk_simulator_base")) / "share" / "arms"


def test_psm_config_loads():
    config = load_robot_config(BASE_ARMS / "PSM1.yaml")
    assert config.name == "PSM1"
    assert [joint.name for joint in config.joints] == ["yaw", "pitch", "insertion", "roll", "wrist_pitch", "wrist_yaw"]
    np.testing.assert_allclose(config.home_position, [0.0, 0.0, 0.12, 0.0, 0.0, 0.0])


def test_ecm_config_loads():
    config = load_robot_config(BASE_ARMS / "ECM.yaml")
    assert config.name == "ECM"
    assert [joint.name for joint in config.joints] == ["yaw", "pitch", "insertion", "roll"]
    np.testing.assert_allclose(config.home_position, [0.0, 0.0, 0.02, 0.0])


def test_psm_instances_include_shared_defaults():
    for name in ("PSM1", "PSM2", "PSM3"):
        config = load_robot_config(BASE_ARMS / f"{name}.yaml")
        assert config.name == name
        assert len(config.joints) == 6
        expected_velocity = {
            "yaw": 5.0,
            "pitch": 5.0,
            "insertion": 0.4,
            "roll": 50.0,
            "wrist_pitch": 50.0,
            "wrist_yaw": 50.0,
        }
        assert all(joint.velocity == expected_velocity[joint.name]
                   for joint in config.joints)


def test_scene_resolution_and_scene_owned_variants():
    config_path = ROOT / "share" / "isaac_sim.yaml.example"
    assert "PSM2_420093_mono.yaml" in available_scene_names(config_path)
    scene_path = resolve_scene_path(config_path, "PSM2_420093_mono.yaml")
    scene = load_scene(scene_path)
    assert scene.camera.mode == "mono"
    assert scene.camera.as_dict().get("transports") == ["rtsp"]
    assert scene.camera.as_dict()["rtsp"]["encoding"] == "raw"
    assert [(robot.name, robot.instrument, robot.endoscope) for robot in scene.robots] == [
        ("PSM2", "420093", None),
        ("ECM", None, "Si_straight"),
    ]
    assert resolve_scene_path(config_path, scene_path) == scene_path


def test_simulator_config_is_typed_and_scene_free_by_default():
    config = load_simulator_config(ROOT / "share" / "isaac_sim.yaml.example")
    assert config.renderer == "RaytracedLighting"
    assert config.simulation_rate_hz == 120.0
    assert config.render_rate_hz == 30.0
    assert config.headless is False
    assert config.scene is None
    assert config.generated_dir.is_absolute()


def test_minimal_renderer_is_supported(tmp_path):
    config_path = tmp_path / "isaac_sim.yaml"
    config_path.write_text("renderer: MinimalRendering\n", encoding="utf-8")

    assert load_simulator_config(config_path).renderer == "MinimalRendering"


def test_shipped_scenes_use_expected_camera_outputs_with_close_near_clip():
    config_path = ROOT / "share" / "isaac_sim.yaml.example"

    for scene_path in available_scene_paths(config_path):
        camera = load_scene(scene_path).camera.as_dict()

        assert camera["transports"] == ["rtsp"]
        assert camera["rtsp"]["encoding"] == "raw"
        assert camera["rtsp"]["mount_path"] == "/ECM"
        assert camera["near_clip_m"] == 0.005
