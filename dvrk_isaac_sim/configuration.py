"""Resolve robot and simulator configuration for the NVIDIA Isaac Sim backend."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Sequence

import yaml

from ament_index_python.packages import get_package_share_directory

from dvrk_simulator_base.scene import SceneConfig, SceneResolver, load_scene_config
from .python_runtime import default_generated_root


VALID_RENDERERS = {
    "MinimalRendering",
    "RaytracedLighting",
    "RealTimePathTracing",
    "PathTracing",
}


@dataclass(frozen=True)
class SimulatorConfig:
    renderer: str = "RaytracedLighting"
    headless: bool = False
    simulation_rate_hz: float = 120.0
    render_rate_hz: float = 30.0
    state_publish_rate_hz: float = 100.0
    generated_root: Path | None = None
    command_queue_capacity: int = 32
    scene: str | None = None


def _boolean(value, *, source: Path, field: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{source}: {field} must be true or false")
    return value


def load_simulator_config(path: str | Path) -> SimulatorConfig:
    source = Path(path).expanduser().resolve()
    with source.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    if not isinstance(document, dict):
        raise ValueError(f"{source}: simulator configuration must be a mapping")

    renderer = str(document.get("renderer", "RaytracedLighting")).strip()
    if renderer not in VALID_RENDERERS:
        raise ValueError(f"{source}: unsupported renderer {renderer}")

    simulation_rate = float(document.get("simulation_rate_hz", 120.0))
    render_rate = float(document.get("render_rate_hz", 30.0))
    state_rate = float(document.get("state_publish_rate_hz", 100.0))
    if any(not math.isfinite(rate) or rate <= 0.0
           for rate in (simulation_rate, render_rate, state_rate)):
        raise ValueError(f"{source}: simulation, render, and state publish rates must be positive")

    capacity = int(document.get("command_queue_capacity", 32))
    if capacity <= 0:
        raise ValueError(f"{source}: command_queue_capacity must be positive")

    generated = document.get("generated_root") or document.get("generated_dir")
    generated_root = None
    if generated not in (None, ""):
        generated_root = Path(str(generated)).expanduser()
        if not generated_root.is_absolute():
            generated_root = (source.parent / generated_root).resolve()

    scene = document.get("scene")
    if scene in (None, ""):
        scene = None
    else:
        scene = str(scene)

    headless = _boolean(document.get("headless", False), source=source, field="headless")

    return SimulatorConfig(
        renderer=renderer,
        headless=headless,
        simulation_rate_hz=simulation_rate,
        render_rate_hz=render_rate,
        state_publish_rate_hz=state_rate,
        generated_root=generated_root,
        command_queue_capacity=capacity,
        scene=scene,
    )


def scene_search_paths(config_path: str | Path) -> tuple[Path, ...]:
    """Return the ordered directories used for a bare scene selection."""
    config = Path(config_path).expanduser().resolve()
    candidates = [config.parent / "scenes"]
    try:
        package_share = Path(get_package_share_directory("dvrk_isaac_sim"))
        candidates.append(package_share / "share" / "scenes")
    except Exception:
        pass
    try:
        simulator_base_share = Path(get_package_share_directory("dvrk_simulator_base"))
        candidates.append(simulator_base_share / "share" / "scenes")
        candidates.append(simulator_base_share / "share" / "exercises")
    except Exception:
        pass
    paths = []
    for path in candidates:
        path = path.resolve()
        if path.is_dir() and path not in paths:
            paths.append(path)
    return tuple(paths)


def resolve_scene_path(
    config_path: str | Path,
    selection: str | Path | Sequence[str | Path],
) -> Path | tuple[Path, ...]:
    """Resolve an absolute path or scene name(s) found in the search paths."""
    config = Path(config_path).expanduser().resolve()
    resolver = SceneResolver(scene_search_paths(config), relative_root=config.parent)
    if isinstance(selection, (list, tuple)):
        return resolver.resolve_all(selection)
    return resolver.resolve(selection)


def load_installed_scene_config(
    path: str | Path | Sequence[str | Path],
    *,
    search_paths: Sequence[Path] | None = None,
) -> SceneConfig:
    share = Path(get_package_share_directory("dvrk_simulator_base"))
    arm_description_share = Path(get_package_share_directory("dvrk_arm_description"))
    try:
        isaac_share = Path(get_package_share_directory("dvrk_isaac_sim"))
        default_search = (
            isaac_share / "share" / "scenes",
            share / "share" / "scenes",
            share / "share" / "exercises",
        )
    except Exception:
        default_search = (
            share / "share" / "scenes",
            share / "share" / "exercises",
        )
    resolver = SceneResolver(tuple(search_paths or default_search))
    return load_scene_config(
        path,
        robot_config_root=arm_description_share / "arms",
        resolver=resolver,
    )
