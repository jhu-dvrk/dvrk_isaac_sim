from pathlib import Path

import numpy as np

from dvrk_isaac_sim.config import load_robot_config
from dvrk_isaac_sim.scene import (
    available_environment_names,
    available_scene_names,
    available_scene_paths,
    load_scene,
    load_simulator_config,
    merge_scene_environment,
    resolve_environment_path,
    resolve_scene_path,
)


ROOT = Path(__file__).parents[1]


def test_psm_config_loads():
    config = load_robot_config(ROOT / "share/arms/PSM1.yaml")
    assert config.name == "PSM1"
    assert [joint.name for joint in config.joints] == ["yaw", "pitch", "insertion", "roll", "wrist_pitch", "wrist_yaw"]
    np.testing.assert_allclose(config.home_position, [0.0, 0.0, 0.12, 0.0, 0.0, 0.0])


def test_ecm_config_loads():
    config = load_robot_config(ROOT / "share/arms/ECM.yaml")
    assert config.name == "ECM"
    assert [joint.name for joint in config.joints] == ["yaw", "pitch", "insertion", "roll"]
    np.testing.assert_allclose(config.home_position, [0.0, 0.0, 0.02, 0.0])


def test_psm_instances_include_shared_defaults():
    for name in ("PSM1", "PSM2", "PSM3"):
        config = load_robot_config(ROOT / "share" / "arms" / f"{name}.yaml")
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


def test_environment_resolution_and_prop_only_environment_loads():
    config_path = ROOT / "share" / "isaac_sim.yaml.example"
    assert "test_cube.yaml" in available_environment_names(config_path)
    environment_path = resolve_environment_path(config_path, "test_cube")
    environment = load_scene(environment_path)
    assert environment.name == "test_cube"
    assert environment.robots == ()
    assert [(prop.name, prop.kind) for prop in environment.props] == [
        ("test_table", "table"),
        ("test_cube", "cube"),
    ]
    assert resolve_environment_path(config_path, environment_path) == environment_path


def test_ball_tray_environment_has_four_walls_and_dynamic_spheres():
    config_path = ROOT / "share" / "isaac_sim.yaml.example"
    environment_path = resolve_environment_path(config_path, "ball_tray")
    environment = load_scene(environment_path)

    assert environment.name == "ball_tray"
    assert len(environment.props) == 14
    walls = [prop for prop in environment.props if prop.name.startswith("wall_")]
    balls = [prop for prop in environment.props if prop.kind == "sphere"]
    assert len(walls) == 4
    assert all(prop.static for prop in walls)
    assert all(prop.guard_kinematic_tools for prop in walls)
    assert len(balls) == 9
    assert all(prop.dynamic for prop in balls)
    assert not any(prop.guard_kinematic_tools for prop in balls)
    assert {round(prop.size[0] * 100, 1) for prop in balls} == {
        1.0, 1.5, 2.0, 2.5, 3.0
    }


def test_environment_can_overlay_a_scene():
    config_path = ROOT / "share" / "isaac_sim.yaml.example"
    scene = load_scene(resolve_scene_path(config_path, "ECM_PSM1_PSM2_PSM3_mono.yaml"))
    environment = load_scene(resolve_environment_path(config_path, "test_cube"))
    combined = merge_scene_environment(scene, environment)
    assert combined.name == "ECM_PSM1_PSM2_PSM3_mono+test_cube"
    assert [robot.name for robot in combined.robots] == ["PSM1", "PSM2", "PSM3", "ECM"]
    assert [prop.name for prop in combined.props] == ["test_table", "test_cube"]
    assert combined.camera.mode == "mono"


def test_shipped_scenes_enable_all_camera_outputs_with_close_near_clip():
    config_path = ROOT / "share" / "isaac_sim.yaml.example"

    for scene_path in available_scene_paths(config_path):
        camera = load_scene(scene_path).camera.as_dict()

        assert camera["transports"] == ["rtsp"]
        assert camera["rtsp"]["encoding"] == "raw"
        assert camera["rtsp"]["mount_path"] == "/ECM"
        assert camera["near_clip_m"] == 0.005
