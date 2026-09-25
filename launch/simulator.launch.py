"""Start a configured dVRK Isaac Sim scene through Isaac Sim Python."""

from pathlib import Path
import json
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from dvrk_isaac_sim.scene import (
    load_scene,
    merge_scene_environment,
    load_simulator_config,
    resolve_environment_path,
    resolve_scene_path,
)



def _start_sim(context):
    package_share = Path(get_package_share_directory("dvrk_isaac_sim"))
    config_path = Path(LaunchConfiguration("config").perform(context)).expanduser().resolve()
    simulator_config = load_simulator_config(config_path)

    isaac_dir_arg = LaunchConfiguration("isaac_sim_dir").perform(context)
    isaac_dir = (Path(isaac_dir_arg).expanduser() if isaac_dir_arg
                 else simulator_config.isaac_sim_dir)
    if isaac_dir is None:
        raise RuntimeError(
            "Isaac Sim path is not configured. Build the workspace with "
            "ISAAC_SIM_DIR set to the Isaac Sim root directory."
        )
    isaac_dir = isaac_dir.resolve()
    isaac_python = isaac_dir / "python.sh"
    if not isaac_python.is_file():
        raise RuntimeError(f"Isaac Sim python.sh not found: {isaac_python}")

    scene_selection = LaunchConfiguration("scene").perform(context)
    environment_selection = LaunchConfiguration("env").perform(context)
    base_scene_selection = scene_selection or simulator_config.scene
    scene_config = (resolve_scene_path(config_path, base_scene_selection)
                    if base_scene_selection else None)
    environment_config = (resolve_environment_path(config_path, environment_selection)
                          if environment_selection else None)
    if scene_config is None and environment_config is None:
        scene_config = resolve_scene_path(config_path, simulator_config.scene)
    if scene_config is not None:
        scene = load_scene(scene_config)
        if environment_config is not None:
            scene = merge_scene_environment(scene, load_scene(environment_config))
    else:
        scene = load_scene(environment_config)
    generated_dir = simulator_config.generated_dir

    conversion_commands = []
    converter = package_share / "scripts" / "convert_dvrk_model.py"

    def ensure_asset(robot):
        variant = (robot.instrument or "420006" if robot.type == "PSM"
                   else robot.endoscope or "Si_straight")
        asset_name = f"{robot.name}_{variant}"
        asset_path = generated_dir / asset_name / robot.name / f"{robot.name}.usda"
        manifest_path = generated_dir / asset_name / "kinematics.json"
        if asset_path.is_file() and manifest_path.is_file():
            try:
                manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                manifest = None
            if isinstance(manifest, dict) and manifest.get("format", 0) >= 5 and isinstance(manifest.get("visual"), dict):
                if robot.type != "PSM":
                    return
                collision = manifest.get("collision")
                if isinstance(collision, dict) and len(collision.get("items", [])) >= 30:
                    return
            # Older manifests lack the USD transform-order metadata required
            # to preserve joints whose visual offsets were folded by Isaac.
        command = [str(isaac_python), str(converter), "--model", robot.name,
                   "--output-dir", str(generated_dir), "--asset-name", asset_name, "--force"]
        command.extend(["--instrument", variant] if robot.type == "PSM"
                       else ["--endoscope", variant])
        conversion_commands.append(command)

    for robot in scene.robots:
        ensure_asset(robot)

    command = [str(isaac_python), str(package_share / "scripts" / "simulator.py"),
               "--config", str(config_path)]
    if scene_config is not None:
        command.extend(["--scene", str(scene_config)])
    if environment_config is not None:
        command.extend(["--env", str(environment_config)])
    renderer = LaunchConfiguration("renderer").perform(context)
    if renderer:
        command.extend(["--renderer", renderer])
    if LaunchConfiguration("headless").perform(context).lower() in {"true", "1", "yes"}:
        command.append("--headless")
    duration = LaunchConfiguration("duration").perform(context)
    if duration and float(duration) > 0.0:
        command.extend(["--duration", duration])
    if LaunchConfiguration("run_crtk_integration_test").perform(context).lower() in {"true", "1", "yes"}:
        command.append("--run-crtk-integration-test")
    if LaunchConfiguration("report_collision_frames").perform(context).lower() in {"true", "1", "yes"}:
        command.append("--report-collision-frames")
    if LaunchConfiguration("kinematic_contact_guard").perform(context).lower() in {"true", "1", "yes"}:
        command.append("--kinematic-contact-guard")

    ros_distro = LaunchConfiguration("ros_distro").perform(context) or simulator_config.ros_distro
    rmw_implementation = LaunchConfiguration("rmw_implementation").perform(context) or simulator_config.rmw_implementation
    environment = {
        "ROS_DISTRO": ros_distro,
        "RMW_IMPLEMENTATION": rmw_implementation,
        "PYTHONUNBUFFERED": "1",
    }
    if conversion_commands:
        shell_command = " && ".join([shlex.join(item) for item in conversion_commands] + [shlex.join(command)])
        return [
            LogInfo(msg=f"Generating {len(conversion_commands)} missing USD/URDF asset set(s) from dvrk_model"),
            ExecuteProcess(cmd=["bash", "-c", shell_command], cwd=str(isaac_dir),
                           additional_env=environment, output="screen"),
        ]
    return [
        LogInfo(msg=f"Starting Isaac Sim scene {scene.name}"),
        ExecuteProcess(cmd=command, cwd=str(isaac_dir), additional_env=environment, output="screen"),
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory("dvrk_isaac_sim"))
    config_path = package_share / "share" / "isaac_sim.yaml"
    if not config_path.is_file():
        config_path = package_share / "share" / "isaac_sim.yaml.example"
    default_config = str(config_path)
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config,
                              description="Saved simulator config YAML"),
        DeclareLaunchArgument("isaac_sim_dir", default_value="",
                              description="Optional Isaac Sim path override"),
        DeclareLaunchArgument("scene", default_value="",
                              description="Scene YAML path or filename under share/scenes"),
        DeclareLaunchArgument("renderer", default_value="",
                              description="Optional renderer override"),
        DeclareLaunchArgument("env", default_value="",
                      description="Environment YAML path or filename under share/environments"),
        DeclareLaunchArgument("headless", default_value="",
                              description="Optional headless override; otherwise use config"),
        DeclareLaunchArgument("duration", default_value="",
                              description="Optional duration override in seconds"),
        DeclareLaunchArgument("run_crtk_integration_test", default_value="false",
                              description="Run the test-only CRTK integration test"),
        DeclareLaunchArgument("report_collision_frames", default_value="false",
                              description="Print collision mesh coordinate-frame diagnostics at startup"),
        DeclareLaunchArgument("kinematic_contact_guard", default_value="false",
                              description="Clamp fast kinematic PSM motion against scene prop AABBs"),
        DeclareLaunchArgument("ros_distro", default_value=""),
        DeclareLaunchArgument("rmw_implementation", default_value=""),
        OpaqueFunction(function=_start_sim),
    ])
