from pathlib import Path
import pytest

from dvrk_isaac_sim.configuration import (
    load_installed_scene_config,
    load_simulator_config,
    resolve_scene_path,
    scene_search_paths,
)


def test_load_simulator_config_defaults(tmp_path):
    path = tmp_path / "isaac_sim.yaml"
    path.write_text("{}\n", encoding="utf-8")
    config = load_simulator_config(path)
    assert config.renderer == "RaytracedLighting"
    assert config.headless is False
    assert config.simulation_rate_hz == 120.0
    assert config.render_rate_hz == 30.0
    assert config.state_publish_rate_hz == 100.0
    assert config.command_queue_capacity == 32
    assert config.scene is None


def test_load_simulator_config_custom(tmp_path):
    path = tmp_path / "isaac_sim.yaml"
    path.write_text(
        "renderer: MinimalRendering\n"
        "headless: true\n"
        "simulation_rate_hz: 60.0\n"
        "render_rate_hz: 15.0\n"
        "state_publish_rate_hz: 50.0\n"
        "command_queue_capacity: 16\n"
        "scene: ECM_PSM1_PSM2.yaml\n",
        encoding="utf-8",
    )
    config = load_simulator_config(path)
    assert config.renderer == "MinimalRendering"
    assert config.headless is True
    assert config.simulation_rate_hz == 60.0
    assert config.render_rate_hz == 15.0
    assert config.state_publish_rate_hz == 50.0
    assert config.command_queue_capacity == 16
    assert config.scene == "ECM_PSM1_PSM2.yaml"


def test_invalid_renderer(tmp_path):
    path = tmp_path / "invalid.yaml"
    path.write_text("renderer: UnsupportedRenderer\n", encoding="utf-8")
    with pytest.raises(ValueError, match="unsupported renderer"):
        load_simulator_config(path)


def test_invalid_rates(tmp_path):
    path = tmp_path / "invalid_rate.yaml"
    path.write_text("simulation_rate_hz: -1.0\n", encoding="utf-8")
    with pytest.raises(ValueError, match="rates must be positive"):
        load_simulator_config(path)


def test_scene_search_paths_includes_scenes():
    paths = scene_search_paths(Path("/tmp/test_config.yaml"))
    path_strings = [str(p) for p in paths]
    assert any("scenes" in s for s in path_strings)
