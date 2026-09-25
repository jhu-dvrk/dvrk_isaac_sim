from pathlib import Path
import os
import re
import sys
import warnings

from setuptools import find_packages, setup


package_name = "dvrk_isaac_sim"
local_isaac_config = Path("share/isaac_sim.yaml")
example_isaac_config = Path("share/isaac_sim.yaml.example")
script_files = [
    "scripts/simulator.py",
    "scripts/tests",
    "scripts/convert_dvrk_model.py",
    "scripts/generate_cart_frames.py",
    "scripts/validate_config.py",
    "scripts/benchmark_interactive.py",
    "scripts/debug_collision_offsets.py",
    "scripts/_isaac_sim_build.py",
]


def _isaac_sim_dir_from_input(value: str) -> Path:
    """Resolve an Isaac Sim root or source/build root to its python.sh dir."""
    selected = Path(value).expanduser()
    candidates = (
        selected / "python.sh",
        selected / "_build/linux-x86_64/release/python.sh",
    )
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate.parent.resolve()
    locations = "\n".join(f"  {candidate}" for candidate in candidates)
    raise RuntimeError(
        "ISAAC_SIM_DIR does not point to a usable Isaac Sim installation. "
        "Expected an executable python.sh at one of:\n" + locations
    )


def _saved_isaac_sim_dir(path: Path) -> Path | None:
    if not path.is_file():
        return None
    match = re.search(r'^isaac_sim_dir:\s*["\']?([^"\'\s]+)',
                      path.read_text(encoding="utf-8"), re.MULTILINE)
    if not match or match.group(1) in {"null", "None"}:
        return None
    return Path(match.group(1)).expanduser().resolve()


def _workspace_root() -> Path | None:
    for parent in Path(__file__).resolve().parents:
        if (parent / "src").is_dir():
            return parent
    return None


def _check_installed_configs(saved: Path) -> None:
    workspace = _workspace_root()
    if workspace is None:
        return
    for tree_name in ("build", "install"):
        tree = workspace / tree_name
        if not tree.is_dir():
            continue
        for config in tree.rglob("isaac_sim.yaml"):
            installed = _saved_isaac_sim_dir(config)
            if installed is not None and installed != saved:
                warnings.warn(
                    f"{config} contains Isaac Sim path {installed}, but the "
                    f"source configuration contains {saved}. Rebuild the "
                    "package to refresh that tree.",
                    RuntimeWarning,
                )


def _configure_isaac_sim() -> bool:
    """Configure Isaac Sim and return whether its scripts may be installed."""
    environment_value = os.environ.get("ISAAC_SIM_DIR", "").strip()
    try:
        environment_dir = (_isaac_sim_dir_from_input(environment_value)
                           if environment_value else None)
    except RuntimeError as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2)
    saved_dir = _saved_isaac_sim_dir(local_isaac_config)

    if environment_dir is not None:
        if saved_dir is not None and environment_dir != saved_dir:
            warnings.warn(
                f"ISAAC_SIM_DIR={environment_dir} differs from the saved "
                f"Isaac Sim path {saved_dir}; updating the saved path.",
                RuntimeWarning,
            )
        local_isaac_config.write_text(
            f'isaac_sim_dir: "{environment_dir}"\n', encoding="utf-8")
        saved_dir = environment_dir
    elif saved_dir is None:
        if not local_isaac_config.exists():
            print(
                "warning: Isaac Sim is not configured. Set ISAAC_SIM_DIR to the Isaac Sim "
                "root directory and rebuild the workspace. Installed scripts will "
                "report this configuration error until then. The directory may "
                "contain python.sh directly or under _build/linux-x86_64/release/.",
                file=sys.stderr,
            )
        local_isaac_config.parent.mkdir(parents=True, exist_ok=True)
        local_isaac_config.write_text("isaac_sim_dir: null\n", encoding="utf-8")
        return False

    _check_installed_configs(saved_dir)
    return True


def _remove_installed_script_links() -> None:
    """Allow symlink-install to refresh this package's script data files."""
    workspace = _workspace_root()
    if workspace is None:
        return
    directory = workspace / "install" / package_name / "share" / package_name / "scripts"
    for script in script_files:
        installed = directory / Path(script).name
        if installed.is_symlink():
            installed.unlink()


# colcon runs setup.py with metadata/help commands before the actual build.
# Do not require local machine configuration during those inspections; the
# real build invocation below performs the validation and reports the useful
# Isaac Sim-specific message.  ``--help develop`` is one such probe.
isaac_sim_configured = True
if "--dry-run" not in sys.argv and not any(
        argument.startswith("--help") for argument in sys.argv):
    isaac_sim_configured = _configure_isaac_sim()
    _remove_installed_script_links()
config_files = [
    "share/arms/PSM.yaml",
    "share/arms/PSM1.yaml",
    "share/arms/PSM2.yaml",
    "share/arms/PSM3.yaml",
    "share/arms/ECM.yaml",
]
data_files = [
    ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml"]),
    (f"share/{package_name}/share/arms", [
        *config_files,
    ]),
    (f"share/{package_name}/share/scenes", [
        str(path) for path in sorted(Path("share/scenes").glob("*.yaml"))
    ]),
    (f"share/{package_name}/share/environments", [
        str(path) for path in sorted(Path("share/environments").glob("*.yaml"))
    ]),
    (f"share/{package_name}/share/dvrk_systems", [
        str(path) for path in sorted(Path("share/dvrk_systems").glob("*.json"))
    ]),
]
if local_isaac_config.exists():
    data_files.append((f"share/{package_name}/share", [str(local_isaac_config)]))
data_files.append((f"share/{package_name}/share", [str(example_isaac_config)]))

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        *data_files,
        (f"share/{package_name}/launch", ["launch/simulator.launch.py"]),
        (f"share/{package_name}/scripts", script_files),
    ],
    install_requires=["setuptools", "numpy", "PyYAML"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "clean_cache = dvrk_isaac_sim.clean_cache:main",
            "dvrk_isaac_sim_kinematics = dvrk_isaac_sim.kinematics:main",
            "dvrk_isaac_sim_ros = dvrk_isaac_sim.ros_node:main",
        ],
    },
)
