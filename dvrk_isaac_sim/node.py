"""Scene-based ROS frontend for a separate Isaac Sim simulation process."""

from dvrk_simulator_base.frontend import parse_command_line, simulator_main
from dvrk_simulator_base.ros_node import SceneBasedSimulatorNode, run_frontend
from .configuration import load_installed_scene_config, load_simulator_config, resolve_scene_path
from .python_runtime import resolve_isaac_sim_python


class DvrkIsaacSimNode(SceneBasedSimulatorNode):
    def __init__(self, *, scene_path, state_publish_rate_hz=100.0, command_queue_capacity=32):
        super().__init__(
            "dvrk_isaac_sim",
            scene_path=scene_path,
            load_scene_fn=load_installed_scene_config,
            state_publish_rate_hz=state_publish_rate_hz,
            command_queue_capacity=command_queue_capacity,
        )


def _parse_command_line(args):
    return parse_command_line(args, __doc__)


def main(args=None):
    return simulator_main(
        "dvrk_isaac_sim", args, _parse_command_line, load_simulator_config,
        resolve_scene_path, resolve_isaac_sim_python, DvrkIsaacSimNode, run_frontend,
    )


if __name__ == "__main__":
    raise SystemExit(main())
