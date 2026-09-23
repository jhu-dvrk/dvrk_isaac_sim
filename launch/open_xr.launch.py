"""Start the Isaac Sim patient cart with the OpenXR dVRK console."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("dvrk_isaac_sim"))
    open_xr_directory = share / "share" / "open-xr"
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / "simulator.launch.py")),
        launch_arguments={
            "config": LaunchConfiguration("config"),
            "scene": "ECM_PSM1_PSM2_PSM3_stereo_rtsp.yaml",
        }.items(),
    )
    dvrk_system = Node(
        package="dvrk_robot", executable="dvrk_system", name="dvrk_system",
        output="screen", cwd=str(open_xr_directory),
        arguments=["--json-config", str(open_xr_directory / "system-MTML-MTMR-OpenXR-patient-cart-ROS.json")],
    )
    start_system = Node(
        package="dvrk_simulator_base", executable="start_dvrk_system", output="screen",
        arguments=["--console", LaunchConfiguration("console")],
    )
    stop_with_console = RegisterEventHandler(
        OnProcessExit(
            target_action=dvrk_system,
            on_exit=[EmitEvent(event=Shutdown(reason="dvrk_system exited"))],
        )
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "config", default_value=str(share / "share" / "isaac_sim.yaml"),
            description="Isaac Sim runtime configuration YAML",
        ),
        DeclareLaunchArgument(
            "console", default_value="console", description="dVRK console ROS namespace",
        ),
        SetEnvironmentVariable("DVRK_SIMULATOR_FORCE_HEADLESS", "true"),
        simulator,
        dvrk_system,
        start_system,
        stop_with_console,
    ])