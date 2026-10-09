"""Runtime options for the isaac_sim backend."""

from dataclasses import dataclass
from dvrk_simulator_base.configuration import RuntimeConfig, load_runtime_config
from dvrk_simulator_base import configuration as _shared

VALID_RENDERERS = {"MinimalRendering", "RaytracedLighting", "RealTimePathTracing", "PathTracing"}


@dataclass(frozen=True)
class SimulatorConfig(RuntimeConfig):
    renderer: str = "RaytracedLighting"
    render_rate_hz: float = 30.0


def load_simulator_config(path):
    return load_runtime_config(path, SimulatorConfig, renderers=VALID_RENDERERS)


def scene_search_paths(config_path):
    return _shared.scene_search_paths("dvrk_isaac_sim", config_path)


def resolve_scene_path(config_path, selection):
    return _shared.resolve_scene_path("dvrk_isaac_sim", config_path, selection)


def load_installed_scene_config(path, *, search_paths=None):
    return _shared.load_installed_scene_config("dvrk_isaac_sim", path, search_paths=search_paths)
