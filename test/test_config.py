from pathlib import Path
from dvrk_isaac_sim.scene import (
    available_scene_names, available_scene_paths, load_scene,
    load_simulator_config, resolve_scene_path,
)


ROOT = Path(__file__).parents[1]


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
