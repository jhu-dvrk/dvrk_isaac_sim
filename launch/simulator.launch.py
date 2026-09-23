"""Start a configured dVRK Isaac Sim scene through Isaac Sim Python."""

from pathlib import Path
import json
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from dvrk_isaac_sim.scene import load_scene, load_simulator_config, resolve_scene_path



def _start_sim(context):
    package_share = Path(get_package_share_directory("dvrk_isaac_sim"))
    config_path = Path(LaunchConfiguration("config").perform(context)).expanduser().resolve()
    simulator_config = load_simulator_config(config_path)

    isaac_dir = simulator_config.isaac_sim_dir
    if isaac_dir is None:
        raise RuntimeError(
            "Isaac Sim path is not configured. Build the workspace with "
            "ISAAC_SIM_DIR set to the Isaac Sim root directory."
        )
    isaac_dir = isaac_dir.resolve()
    isaac_python = isaac_dir / "python.sh"
    if not isaac_python.is_file():
        raise RuntimeError(f"Isaac Sim python.sh not found: {isaac_python}")

    scene_selection = LaunchConfiguration("scene").perform(context) or simulator_config.scene
    scene_config = resolve_scene_path(config_path, scene_selection)
    scene = load_scene(scene_config)
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
            if (isinstance(manifest, dict) and manifest.get("format", 0) >= 3
                    and isinstance(manifest.get("visual"), dict)):
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
               "--config", str(config_path), "--scene", str(scene_config)]
    environment = {
        "ROS_DISTRO": simulator_config.ros_distro,
        "RMW_IMPLEMENTATION": simulator_config.rmw_implementation,
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
        DeclareLaunchArgument("scene", description="Scene YAML path or filename under share/scenes"),
        OpaqueFunction(function=_start_sim),
    ])
