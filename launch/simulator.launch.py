"""Start one configured dVRK Isaac Sim scene using the configured Python interpreter."""

from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from dvrk_isaac_sim.configuration import (
    load_installed_scene_config,
    load_simulator_config,
    resolve_scene_path,
)
from dvrk_isaac_sim.python_runtime import default_generated_root
from dvrk_simulator_base.rqt_perspective import (
    existing_ament_prefix_path,
    write_monitor_perspective,
)


def _start_sim(context):
    package_share = Path(get_package_share_directory("dvrk_isaac_sim"))
    config_path = Path(LaunchConfiguration("config").perform(context)).expanduser().resolve()
    simulator_config = load_simulator_config(config_path)

    scene = LaunchConfiguration("scene").perform(context) or simulator_config.scene
    if not scene:
        raise ValueError("a scene is required")
    script = package_share / "scripts" / "simulator.py"

    headless_arg = LaunchConfiguration("headless").perform(context)
    if headless_arg:
        headless = "true" if headless_arg.lower() == "true" else "false"
    else:
        headless = "true" if simulator_config.headless else "false"

    cmd = [
        sys.executable,
        str(script),
        "--config", str(config_path),
        "--headless", str(headless),
        "--scene", str(scene),
    ]
    actions = [
        ExecuteProcess(cmd=cmd, output="screen"),
    ]
    if LaunchConfiguration("rqt").perform(context).lower() == "true":
        scene_config = resolve_scene_path(config_path, scene)
        scene_description = load_installed_scene_config(scene_config)
        arms = [robot.name for robot in scene_description.robots]
        perspective = write_monitor_perspective(
            (simulator_config.generated_root or default_generated_root()) / "rqt" / "monitor.perspective",
            arms,
            include_console=LaunchConfiguration("rqt_console").perform(context).lower() == "true",
        )
        rqt_environment = {"DVRK_RQT_ARMS": ",".join(arms)}
        if prefix_path := existing_ament_prefix_path():
            rqt_environment["AMENT_PREFIX_PATH"] = prefix_path
        actions.append(ExecuteProcess(
            cmd=["rqt", "--perspective-file", str(perspective)], output="screen",
            additional_env=rqt_environment,
        ))
    return actions


def generate_launch_description():
    package_share = Path(get_package_share_directory("dvrk_isaac_sim"))
    default_config = package_share / "share" / "isaac_sim.yaml"
    if not default_config.is_file():
        default_config = package_share / "share" / "isaac_sim.yaml.example"

    return LaunchDescription([
        DeclareLaunchArgument(
            "config", default_value=str(default_config),
            description="Backend runtime configuration YAML",
        ),
        DeclareLaunchArgument(
            "scene", default_value="",
            description="Scene YAML path or installed scene filename",
        ),
        DeclareLaunchArgument(
            "headless", default_value="",
            description="Run without GUI window (true/false, default: runtime YAML)",
        ),
        DeclareLaunchArgument(
            "rqt", default_value="false",
            description="start a dockable dVRK rqt monitor",
        ),
        DeclareLaunchArgument(
            "rqt_console", default_value="false",
            description="include the dVRK Console widget in the rqt monitor",
        ),
        OpaqueFunction(function=_start_sim),
    ])
