"""Runtime options for the isaac_sim backend."""

from dataclasses import dataclass, replace
from pathlib import Path
from dvrk_simulator_base.configuration import RuntimeConfig, load_runtime_config
from dvrk_simulator_base import configuration as _shared

VALID_RENDERERS = {"MinimalRendering", "RaytracedLighting", "RealTimePathTracing", "PathTracing"}


@dataclass(frozen=True)
class SimulatorConfig(RuntimeConfig):
    renderer: str = "RaytracedLighting"
    render_rate_hz: float = 30.0
    collision_backend: str = "newton"
    newton_python: str | None = None
    newton_device: str = "auto"


def load_simulator_config(path):
    config = load_runtime_config(path, SimulatorConfig, renderers=VALID_RENDERERS)
    if config.collision_backend not in {"newton", "none"}:
        raise ValueError(
            f"{path}: collision_backend must be 'newton' or 'none', "
            f"got {config.collision_backend!r}"
        )
    if config.newton_python is not None and (
        not isinstance(config.newton_python, str) or not config.newton_python.strip()
    ):
        raise ValueError(f"{path}: newton_python must be a non-empty path or null")
    if not isinstance(config.newton_device, str) or not config.newton_device.strip():
        raise ValueError(f"{path}: newton_device must be a non-empty string")
    if config.newton_python is not None:
        executable = Path(config.newton_python).expanduser()
        if not executable.is_absolute():
            executable = Path(path).expanduser().resolve().parent / executable
        config = replace(config, newton_python=str(executable.resolve()))
    return config


def scene_search_paths(config_path):
    return _shared.scene_search_paths("dvrk_isaac_sim", config_path)


def resolve_scene_path(config_path, selection):
    return _shared.resolve_scene_path("dvrk_isaac_sim", config_path, selection)


def load_installed_scene_config(path, *, search_paths=None):
    return _shared.load_installed_scene_config("dvrk_isaac_sim", path, search_paths=search_paths)
