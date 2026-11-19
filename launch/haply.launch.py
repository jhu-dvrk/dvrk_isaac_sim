"""Start the Isaac Sim Haply session with the cisst dVRK system GUI."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from dvrk_simulator_base.launch import SystemProfile, start_session
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    haply = Path(get_package_share_directory("haply_config")) / "dvrk_simulation"
    profile = SystemProfile(
        config=haply / "isaac_sim.yaml",
        cart_scene=haply / "ECM_PSM1_PSM2_PSM3_mono_rtsp.yaml",
        system_config=haply / "system-Haply.json",
        cwd=haply,
    )
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=str(profile.config),
                              description="Backend runtime YAML"),
        DeclareLaunchArgument("exercise", default_value="test_cube_objects.yaml",
                              description="Exercise overlay YAML path or installed filename"),
        DeclareLaunchArgument("scene", default_value=LaunchConfiguration("exercise")),
        DeclareLaunchArgument("headless", default_value="false",
                              description="Desktop viewer override; empty uses runtime YAML"),
        DeclareLaunchArgument("rqt", default_value="false",
                              description="Start the dVRK monitor"),
        DeclareLaunchArgument("rqt_console", default_value="false",
                              description="Include console monitoring"),
        DeclareLaunchArgument("console", default_value="console",
                              description="Console namespace for clients"),
        OpaqueFunction(function=start_session, args=["dvrk_isaac_sim", profile]),
    ])
