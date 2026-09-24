"""Scene and simulator-configuration loading independent of Isaac Sim."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
from typing import Any

import yaml

from dvrk_arm_description import load_robot_document


@dataclass(frozen=True)
class SimulatorConfig:
    path: Path
    isaac_sim_dir: Path | None
    generated_dir: Path
    renderer: str
    headless: bool
    duration: float
    simulation_rate_hz: float
    render_rate_hz: float
    ros_distro: str
    rmw_implementation: str
    scene: str | None


@dataclass(frozen=True)
class SceneCamera:
    mode: str = "mono"
    owner: str = "ECM"
    settings: dict[str, Any] | None = None

    def as_dict(self) -> dict[str, Any]:
        return dict(self.settings or {})


@dataclass(frozen=True)
class SceneRobot:
    name: str
    type: str
    config_path: Path
    frame: dict[str, Any]
    instrument: str | None = None
    endoscope: str | None = None


@dataclass(frozen=True)
class SceneConfig:
    path: Path
    name: str
    base_frame_provider: str
    frames: dict[str, dict[str, Any]]
    robots: tuple[SceneRobot, ...]
    camera: SceneCamera


def load_yaml_mapping(path: str | Path) -> dict[str, Any]:
    source = Path(path).expanduser().resolve()
    with source.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    if not isinstance(document, dict):
        raise ValueError(f"{source}: expected a YAML mapping")
    return document


def _default_generated_dir(source: Path) -> Path:
    """Return the user cache directory for Isaac Sim assets."""
    cache_root = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache"))
    return (cache_root / "dvrk_isaac_sim").resolve()



def load_simulator_config(path: str | Path) -> SimulatorConfig:
    """Load and validate simulator-level settings from a config YAML."""
    source = Path(path).expanduser().resolve()
    document = load_yaml_mapping(source)

    def value(name: str, default=None):
        return document.get(name, default)

    def path_value(name: str, default=None) -> Path | None:
        raw = value(name, default)
        if raw in (None, ""):
            return None
        selected = Path(str(raw)).expanduser()
        return selected if selected.is_absolute() else (source.parent / selected).resolve()

    generated_dir = path_value("generated_dir", _default_generated_dir(source))
    if generated_dir is None:
        raise ValueError(f"{source}: generated_dir is required")
    renderer = str(value("renderer", "RaytracedLighting"))
    if renderer not in {
        "MinimalRendering", "RaytracedLighting", "RealTimePathTracing", "PathTracing"
    }:
        raise ValueError(f"{source}: unsupported renderer {renderer}")
    duration = float(value("duration", 0.0))
    if duration < 0.0:
        raise ValueError(f"{source}: duration cannot be negative")
    simulation_rate_hz = float(value("simulation_rate_hz", 120.0))
    if simulation_rate_hz <= 0.0:
        raise ValueError(f"{source}: simulation_rate_hz must be positive")
    render_rate_hz = float(value("render_rate_hz", 30.0))
    if render_rate_hz <= 0.0:
        raise ValueError(f"{source}: render_rate_hz must be positive")
    return SimulatorConfig(
        path=source,
        isaac_sim_dir=path_value("isaac_sim_dir"),
        generated_dir=generated_dir,
        renderer=renderer,
        headless=bool(value("headless", False)),
        duration=duration,
        simulation_rate_hz=simulation_rate_hz,
        render_rate_hz=render_rate_hz,
        ros_distro=str(value("ros_distro", "jazzy")),
        rmw_implementation=str(value("rmw_implementation", "rmw_fastrtps_cpp")),
        scene=str(value("scene")) if value("scene") not in (None, "") else None,
    )


def _base_scenes_directory() -> Path | None:
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory("dvrk_simulator_base")) / "share" / "scenes"
        if share.is_dir():
            return share
    except Exception:
        pass
    return None


def available_scene_paths(config_path: str | Path) -> tuple[Path, ...]:
    """Return scene YAMLs next to a simulator config file and in dvrk_simulator_base."""
    paths = []
    directory = Path(config_path).expanduser().resolve().parent / "scenes"
    if directory.is_dir():
        paths.extend(directory.glob("*.yaml"))
    base_dir = _base_scenes_directory()
    if base_dir is not None:
        seen = {p.name for p in paths}
        for p in base_dir.glob("*.yaml"):
            if p.name not in seen:
                paths.append(p)
    return tuple(sorted(paths, key=lambda p: p.name))


def available_scene_names(config_path: str | Path) -> tuple[str, ...]:
    return tuple(path.name for path in available_scene_paths(config_path))


def resolve_scene_path(config_path: str | Path, selection: str | Path | None) -> Path:
    """Resolve a scene filename, relative path, or absolute path.

    Bare filenames are searched below the config file's ``scenes`` directory
    or dvrk_simulator_base shared scenes.
    """
    config = Path(config_path).expanduser().resolve()
    if selection in (None, ""):
        names = "\n  ".join(available_scene_names(config)) or "(none)"
        raise ValueError(
            f"No scene selected. Choose --scene or set scene in {config}.\n"
            f"Available scenes:\n  {names}"
        )
    selected = Path(str(selection)).expanduser()
    if selected.is_absolute():
        resolved = selected.resolve()
    else:
        resolved = (config.parent / selected).resolve()
        if not resolved.is_file() and selected.parent == Path("."):
            resolved = (config.parent / "scenes" / selected).resolve()
        if not resolved.suffix:
            resolved = resolved.with_suffix(".yaml")
        if not resolved.is_file() and selected.parent == Path("."):
            base_dir = _base_scenes_directory()
            if base_dir is not None:
                candidate = (base_dir / selected).resolve()
                if not candidate.suffix:
                    candidate = candidate.with_suffix(".yaml")
                if candidate.is_file():
                    resolved = candidate
    if not resolved.is_file():
        names = "\n  ".join(available_scene_names(config)) or "(none)"
        raise FileNotFoundError(
            f"Scene configuration not found: {resolved}\nAvailable scenes:\n  {names}"
        )
    return resolved


def _robot_config_path(scene_path: Path, configured: Any) -> Path:
    if isinstance(configured, dict):
        configured = configured.get("config")
    if not configured:
        raise ValueError(f"{scene_path}: scene robot is missing config")
    configured_text = str(configured)
    if configured_text.startswith("package://"):
        package, separator, relative = configured_text[len("package://"):].partition("/")
        if not package or not separator or not relative:
            raise ValueError(f"{scene_path}: invalid package asset URI {configured_text!r}")
        try:
            from ament_index_python.packages import get_package_share_directory

            resolved = (Path(get_package_share_directory(package)) / relative).resolve()
        except Exception as error:
            raise FileNotFoundError(
                f"{scene_path}: could not resolve robot configuration {configured_text!r}"
            ) from error
        if not resolved.is_file():
            raise FileNotFoundError(
                f"{scene_path}: robot configuration does not exist: {resolved}"
            )
        return resolved

    path = Path(configured_text).expanduser()
    if path.is_absolute():
        return path.resolve()

    try:
        from ament_index_python.packages import get_package_share_directory

        arm_desc = (Path(get_package_share_directory("dvrk_arm_description")) / "arms" / path).resolve()
        if arm_desc.is_file():
            return arm_desc
    except Exception:
        pass

    # A standalone scene may provide a custom robot definition next to its
    # package root.  Package-owned robot definitions use package:// URIs.
    scene_relative = (scene_path.parent.parent.parent / path).resolve()
    if scene_relative.is_file():
        return scene_relative

    # Keep scene/configuration tooling usable outside a sourced ROS
    # environment.  Ament is only needed for an installed package whose scene
    # refers to the package-owned arm definitions.
    try:
        from ament_index_python.packages import get_package_share_directory

        package_relative = (Path(get_package_share_directory("dvrk_isaac_sim")) /
                            path).resolve()
        if package_relative.is_file():
            return package_relative
    except ModuleNotFoundError:
        pass

    return scene_relative


def load_scene(scene_path: str | Path) -> SceneConfig:
    """Load, resolve, and validate one scene document."""
    source = Path(scene_path).expanduser().resolve()
    document = load_yaml_mapping(source)
    scene = document.get("scene")
    if not isinstance(scene, dict):
        raise ValueError(f"{source}: expected a top-level scene mapping")

    frames = scene.get("frames", {}) or {}
    if not isinstance(frames, dict):
        raise ValueError(f"{source}: scene.frames must be a mapping")

    has_camera = "camera" in scene
    camera_document = scene.get("camera", {}) or {}
    if not isinstance(camera_document, dict):
        raise ValueError(f"{source}: scene.camera must be a mapping")
    camera = SceneCamera(
        mode=str(camera_document.get("mode", "mono" if has_camera else "off")),
        owner=str(camera_document.get("owner", "ECM")),
        settings=dict(camera_document),
    )
    if camera.mode not in {"off", "mono", "stereo"}:
        raise ValueError(f"{source}: camera.mode must be off, mono, or stereo")
    if camera.mode != "off":
        if "transport" in camera_document:
            raise ValueError(f"{source}: camera.transport was replaced by camera.transports")
        camera_transports = camera_document.get("transports", ["ros_raw"])
        if (not isinstance(camera_transports, list)
                or not all(isinstance(item, str) for item in camera_transports)):
            raise ValueError(f"{source}: camera.transports must be a list of strings")
        if len(set(camera_transports)) != len(camera_transports):
            raise ValueError(f"{source}: camera.transports must not contain duplicates")
        unsupported_transports = set(camera_transports) - {"ros_raw", "ros_compressed", "rtsp", "unixfd"}
        if unsupported_transports:
            raise ValueError(
                f"{source}: unsupported camera transport(s): {', '.join(sorted(unsupported_transports))}"
            )
        if "ros_compressed" in camera_transports:
            compressed_document = camera_document.get("ros_compressed", {}) or {}
            if not isinstance(compressed_document, dict):
                raise ValueError(f"{source}: camera.ros_compressed must be a mapping")
            quality = int(compressed_document.get("quality", 85))
            if not 1 <= quality <= 100:
                raise ValueError(f"{source}: camera.ros_compressed.quality must be between 1 and 100")
        if "rtsp" in camera_transports:
            rtsp_document = camera_document.get("rtsp", {}) or {}
            if not isinstance(rtsp_document, dict):
                raise ValueError(f"{source}: camera.rtsp must be a mapping")
            port = int(rtsp_document.get("port", 8554))
            mount_path = str(rtsp_document.get("mount_path", "/ECM"))
            encoding = str(rtsp_document.get("encoding", "h264")).lower()
            if not 1 <= port <= 65535:
                raise ValueError(f"{source}: camera.rtsp.port must be between 1 and 65535")
            if not mount_path.startswith("/"):
                raise ValueError(f"{source}: camera.rtsp.mount_path must start with '/'")
            if encoding not in {"h264", "raw"}:
                raise ValueError(f"{source}: camera.rtsp.encoding must be h264 or raw")
        if camera.owner != "ECM":
            raise ValueError(f"{source}: only ECM is supported as the camera owner")

    entries = []
    names = set()
    for configured in scene.get("robots", []) or []:
        if not isinstance(configured, dict):
            raise ValueError(f"{source}: each scene robot must be a mapping")
        options = configured
        config_path = _robot_config_path(source, configured)
        robot_document = load_robot_document(config_path)
        robot = robot_document.get("robot", {})
        name = str(robot.get("name", ""))
        robot_type = str(robot.get("type", "")).upper()
        if not name or robot_type not in {"PSM", "ECM"}:
            raise ValueError(f"{source}: invalid robot configuration {config_path}")
        if name in names:
            raise ValueError(f"{source}: duplicate robot name {name}")
        names.add(name)
        frame = frames.get(name, {}) or {}
        if not isinstance(frame, dict):
            raise ValueError(f"{source}: frame for {name} must be a mapping")
        entries.append(SceneRobot(
            name=name,
            type=robot_type,
            config_path=config_path,
            frame=dict(frame),
            instrument=str(options["instrument"]) if "instrument" in options else None,
            endoscope=str(options["endoscope"]) if "endoscope" in options else None,
        ))
    if not entries:
        raise ValueError(f"{source}: scene has no robots")

    return SceneConfig(
        path=source,
        name=str(scene.get("name", source.stem)),
        base_frame_provider=str(scene.get("base_frame_provider", "yaml")),
        frames={str(name): dict(frame) for name, frame in frames.items()},
        robots=tuple(entries),
        camera=camera,
    )
