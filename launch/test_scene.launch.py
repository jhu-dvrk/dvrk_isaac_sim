"""Run a bounded headless Isaac Sim scene smoke test."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("dvrk_isaac_sim"))
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=str(share / "share" / "isaac_sim.yaml")),
        DeclareLaunchArgument("scene", description="Scene YAML path or filename under share/scenes"),
        DeclareLaunchArgument("timeout", default_value="1.0", description="Maximum test run duration in seconds"),
        SetEnvironmentVariable("DVRK_SIMULATOR_TEST_TIMEOUT", LaunchConfiguration("timeout")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / "simulator.launch.py")),
            launch_arguments={"config": LaunchConfiguration("config"), "scene": LaunchConfiguration("scene")}.items(),
        ),
    ])