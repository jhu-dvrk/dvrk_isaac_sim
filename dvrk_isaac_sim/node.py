"""Scene-based ROS frontend for a separate Isaac Sim simulation process."""

import argparse
import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from rclpy.utilities import remove_ros_args


from dvrk_simulator_base.ros_node import SimulatorRosNode, run_frontend
from .configuration import load_installed_scene_config, load_simulator_config, resolve_scene_path
from .python_runtime import resolve_isaac_sim_python


class DvrkIsaacSimNode(SimulatorRosNode):
    def __init__(self, *, scene_path, state_publish_rate_hz=100.0, command_queue_capacity=32):
        scene = load_installed_scene_config(scene_path)
        super().__init__("dvrk_isaac_sim", scene.robots, state_publish_rate_hz=state_publish_rate_hz,
                         command_queue_capacity=command_queue_capacity)


def _parse_command_line(args):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, help="Isaac Sim runtime YAML")
    parser.add_argument("--scene", type=Path, action="append", help="scene YAML; may be repeated")
    parser.add_argument("--headless", choices=("true", "false"), help="override desktop viewer setting")
    return parser.parse_args(remove_ros_args(args))


def main(args=None):
    raw_args = list(sys.argv[1:] if args is None else args)
    options = _parse_command_line(raw_args)
    share = Path(get_package_share_directory("dvrk_isaac_sim"))
    path = options.config or share / "share/isaac_sim.yaml"
    if options.config is None and not path.is_file():
        path = share / "share/isaac_sim.yaml.example"
    try:
        config = load_simulator_config(path)
        selection = options.scene or config.scene
        if not selection:
            raise ValueError("a scene is required: specify --scene or scene in the runtime YAML")
        scenes = resolve_scene_path(path, selection)
        python = resolve_isaac_sim_python(config.generated_root).path
    except (RuntimeError, OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    headless = config.headless if options.headless is None else options.headless == "true"
    start = dict(
        config=str(path.expanduser().resolve()),
        scene=[str(item) for item in scenes] if isinstance(scenes, (tuple, list)) else str(scenes),
        headless=headless or "DVRK_SIMULATOR_TEST_TIMEOUT" in os.environ,
    )
    return run_frontend(
        lambda: DvrkIsaacSimNode(scene_path=scenes, state_publish_rate_hz=config.state_publish_rate_hz,
                                command_queue_capacity=config.command_queue_capacity),
        python, "dvrk_isaac_sim.simulation_worker", start, raw_args,
    )


if __name__ == "__main__":
    raise SystemExit(main())
