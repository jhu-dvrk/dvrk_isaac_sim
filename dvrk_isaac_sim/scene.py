"""Scene and simulator-configuration loading independent of Isaac Sim."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .config import load_robot_document


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
class SceneProp:
    name: str
    kind: str
    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    size: tuple[float, float, float]
    static: bool
    dynamic: bool
    mass: float | None = None
    color: tuple[float, float, float, float] | None = None
    guard_kinematic_tools: bool = False


@dataclass(frozen=True)
class SceneConfig:
    path: Path
    name: str
    base_frame_provider: str
    frames: dict[str, dict[str, Any]]
    robots: tuple[SceneRobot, ...]
    props: tuple[SceneProp, ...]
    camera: SceneCamera


def merge_scene_environment(scene: SceneConfig, environment: SceneConfig) -> SceneConfig:
    """Overlay environment props onto a base scene.

    The base scene owns robot selection, frames, and camera settings. The
    environment contributes additional props and may add robots or frames when
    they do not conflict with the base scene.
    """
    combined_frames = {name: dict(frame) for name, frame in scene.frames.items()}
    for name, frame in environment.frames.items():
        if name in combined_frames:
            raise ValueError(
                f"Cannot overlay environment {environment.path} on {scene.path}: "
                f"duplicate frame entry {name}"
            )
        combined_frames[name] = dict(frame)

    combined_robots = list(scene.robots)
    robot_names = {robot.name for robot in scene.robots}
    for robot in environment.robots:
        if robot.name in robot_names:
            raise ValueError(
                f"Cannot overlay environment {environment.path} on {scene.path}: "
                f"duplicate robot {robot.name}"
            )
        robot_names.add(robot.name)
        combined_robots.append(robot)

    prop_names = {prop.name for prop in scene.props}
    combined_props = list(scene.props)
    for prop in environment.props:
        if prop.name in prop_names or prop.name in robot_names:
            raise ValueError(
                f"Cannot overlay environment {environment.path} on {scene.path}: "
                f"duplicate prop {prop.name}"
            )
        prop_names.add(prop.name)
        combined_props.append(prop)

    combined_name = scene.name if environment.name == scene.name else f"{scene.name}+{environment.name}"
    return SceneConfig(
        path=scene.path,
        name=combined_name,
        base_frame_provider=scene.base_frame_provider,
        frames=combined_frames,
        robots=tuple(combined_robots),
        props=tuple(combined_props),
        camera=scene.camera,
    )


def load_yaml_mapping(path: str | Path) -> dict[str, Any]:
    source = Path(path).expanduser().resolve()
    with source.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    if not isinstance(document, dict):
        raise ValueError(f"{source}: expected a YAML mapping")
    return document


def _default_generated_dir(source: Path) -> Path:
    """Choose the workspace cache, without placing assets below ``src``."""
    for parent in source.parents:
        if parent.name == "src":
            return parent.parent / ".generated" / "isaacsim-6.0"
        if parent.name == "install":
            return parent.parent / ".generated" / "isaacsim-6.0"
    # A standalone config outside a colcon workspace gets a local sibling cache.
    return source.parent / ".generated" / "isaacsim-6.0"


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


def _available_yaml_paths(config_path: str | Path, directory_name: str) -> tuple[Path, ...]:
    directory = Path(config_path).expanduser().resolve().parent / directory_name
    return tuple(sorted(directory.glob("*.yaml")))


def available_scene_paths(config_path: str | Path) -> tuple[Path, ...]:
    """Return scene YAMLs next to a simulator config file."""
    return _available_yaml_paths(config_path, "scenes")


def available_scene_names(config_path: str | Path) -> tuple[str, ...]:
    return tuple(path.name for path in available_scene_paths(config_path))


def available_environment_paths(config_path: str | Path) -> tuple[Path, ...]:
    """Return environment YAMLs next to a simulator config file."""
    return _available_yaml_paths(config_path, "environments")


def available_environment_names(config_path: str | Path) -> tuple[str, ...]:
    return tuple(path.name for path in available_environment_paths(config_path))


def _resolve_config_path(
    config_path: str | Path,
    selection: str | Path | None,
    *,
    directory_name: str,
    description: str,
    available_names: tuple[str, ...],
) -> Path:
    """Resolve a config filename, relative path, or absolute path."""
    config = Path(config_path).expanduser().resolve()
    if selection in (None, ""):
        names = "\n  ".join(available_names) or "(none)"
        raise ValueError(
            f"No {description} selected. Choose --{description} or set scene in {config}.\n"
            f"Available {description}s:\n  {names}"
        )
    selected = Path(str(selection)).expanduser()
    if selected.is_absolute():
        resolved = selected.resolve()
    else:
        resolved = (config.parent / selected).resolve()
        if not resolved.is_file() and selected.parent == Path("."):
            resolved = (config.parent / directory_name / selected).resolve()
        if not resolved.suffix:
            resolved = resolved.with_suffix(".yaml")
    if not resolved.is_file():
        names = "\n  ".join(available_names) or "(none)"
        raise FileNotFoundError(
            f"{description.capitalize()} configuration not found: {resolved}\n"
            f"Available {description}s:\n  {names}"
        )
    return resolved


def resolve_scene_path(config_path: str | Path, selection: str | Path | None) -> Path:
    """Resolve a scene filename, relative path, or absolute path.

    Bare filenames are searched below the config file's ``scenes`` directory.
    """
    return _resolve_config_path(
        config_path,
        selection,
        directory_name="scenes",
        description="scene",
        available_names=available_scene_names(config_path),
    )


def resolve_environment_path(config_path: str | Path, selection: str | Path | None) -> Path:
    """Resolve an environment filename, relative path, or absolute path.

    Bare filenames are searched below the config file's ``environments`` directory.
    """
    return _resolve_config_path(
        config_path,
        selection,
        directory_name="environments",
        description="environment",
        available_names=available_environment_names(config_path),
    )


def _float_tuple(source: Path, value: Any, size: int, field_name: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f"{source}: {field_name} must contain {size} values")
    return tuple(float(item) for item in value)


def _robot_config_path(scene_path: Path, configured: Any) -> Path:
    if isinstance(configured, dict):
        configured = configured.get("config")
    if not configured:
        raise ValueError(f"{scene_path}: scene robot is missing config")
    path = Path(str(configured)).expanduser()
    if path.is_absolute():
        return path.resolve()

    # A standalone scene may provide a custom robot definition next to its
    # package root.  Otherwise use the canonical definitions installed with
    # dvrk_isaac_sim, so scene bundles need not duplicate ECM/PSM YAML files.
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

    camera_document = scene.get("camera", {}) or {}
    if not isinstance(camera_document, dict):
        raise ValueError(f"{source}: scene.camera must be a mapping")
    camera = SceneCamera(
        mode=str(camera_document.get("mode", "mono")),
        owner=str(camera_document.get("owner", "ECM")),
        settings=dict(camera_document),
    )
    if camera.mode not in {"off", "mono", "stereo"}:
        raise ValueError(f"{source}: camera.mode must be off, mono, or stereo")
    if "transport" in camera_document:
        raise ValueError(f"{source}: camera.transport was replaced by camera.transports")
    camera_transports = camera_document.get("transports", ["ros_raw"])
    if (not isinstance(camera_transports, list)
            or not all(isinstance(item, str) for item in camera_transports)):
        raise ValueError(f"{source}: camera.transports must be a list of strings")
    if len(set(camera_transports)) != len(camera_transports):
        raise ValueError(f"{source}: camera.transports must not contain duplicates")
    unsupported_transports = set(camera_transports) - {"ros_raw", "ros_compressed", "rtsp"}
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

    props = []
    for configured in scene.get("props", []) or []:
        if not isinstance(configured, dict):
            raise ValueError(f"{source}: each scene prop must be a mapping")
        name = str(configured.get("name", ""))
        kind = str(configured.get("kind", "")).lower()
        if not name:
            raise ValueError(f"{source}: scene prop is missing name")
        if name in names:
            raise ValueError(f"{source}: duplicate scene entry name {name}")
        if kind not in {"cube", "sphere", "table"}:
            raise ValueError(f"{source}: unsupported prop kind {kind!r} for {name}")
        position = _float_tuple(source, configured.get("position", [0.0, 0.0, 0.0]), 3,
                                f"scene.props[{name}].position")
        orientation_xyzw = _float_tuple(
            source, configured.get("orientation_xyzw", [0.0, 0.0, 0.0, 1.0]), 4,
            f"scene.props[{name}].orientation_xyzw"
        )
        if not any(abs(component) > 0.0 for component in orientation_xyzw):
            raise ValueError(f"{source}: scene.props[{name}].orientation_xyzw cannot be zero")
        size_value = configured.get("size", [1.0, 1.0, 1.0])
        size = _float_tuple(source, size_value, 3, f"scene.props[{name}].size")
        if any(component <= 0.0 for component in size):
            raise ValueError(f"{source}: scene.props[{name}].size values must be positive")
        if kind == "sphere" and max(size) - min(size) > 1e-9:
            raise ValueError(f"{source}: scene.props[{name}].size must be uniform for a sphere")
        color_value = configured.get("color")
        color = None
        if color_value is not None:
            color = _float_tuple(source, color_value, 4, f"scene.props[{name}].color")
        static = bool(configured.get("static", False))
        dynamic = bool(configured.get("dynamic", False))
        if static and dynamic:
            raise ValueError(f"{source}: scene.props[{name}] cannot be both static and dynamic")
        mass = None
        if "mass" in configured:
            mass = float(configured["mass"])
            if mass <= 0.0:
                raise ValueError(f"{source}: scene.props[{name}].mass must be positive")
        names.add(name)
        props.append(SceneProp(
            name=name,
            kind=kind,
            position=position,
            orientation_xyzw=orientation_xyzw,
            size=size,
            static=static,
            dynamic=dynamic,
            mass=mass,
            color=color,
            guard_kinematic_tools=bool(configured.get("guard_kinematic_tools", False)),
        ))

    if not entries and not props:
        raise ValueError(f"{source}: scene has no robots or props")

    return SceneConfig(
        path=source,
        name=str(scene.get("name", source.stem)),
        base_frame_provider=str(scene.get("base_frame_provider", "yaml")),
        frames={str(name): dict(frame) for name, frame in frames.items()},
        robots=tuple(entries),
        props=tuple(props),
        camera=camera,
    )
