from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory

from dvrk_isaac_sim.kinematics import CRTKPSM
from dvrk_isaac_sim.ros_interface import CRTKROSComponent
from dvrk_arm_description import load_robot_config


class _Publisher:
    def publish(self, _message):
        pass


class _Logger:
    def debug(self, _message):
        pass

    def info(self, _message):
        pass

    def warning(self, _message):
        pass


class _Stamp:
    sec = 1
    nanosec = 0


class _Clock:
    def now(self):
        return self

    def to_msg(self):
        return _Stamp()


class _Node:
    def create_publisher(self, *_args):
        return _Publisher()

    def create_subscription(self, *_args):
        return object()

    def get_clock(self):
        return _Clock()

    def get_logger(self):
        return _Logger()


class _JointMessage:
    def __init__(self, position):
        self.name = [str(index) for index in range(len(position))]
        self.position = list(position)


def _component():
    arms = Path(get_package_share_directory("dvrk_arm_description")) / "arms"
    model = CRTKPSM(load_robot_config(arms / "PSM1.yaml"))
    return CRTKROSComponent(_Node(), model.config, model, _Stamp())


def test_ros_callbacks_queue_servo_commands_until_isaac_processes_them():
    component = _component()
    first = np.array([0.05, 0.02, 0.1, 0.1, 0.1, 0.1])
    newest = np.array([0.1, -0.1, 0.11, 0.2, -0.1, 0.1])

    component._servo_jp_callback(_JointMessage(first))
    component._servo_jp_callback(_JointMessage(newest))

    np.testing.assert_allclose(
        component.model.goal_js().position, component.config.home_position
    )
    assert component.command_metrics()[2] == 1

    component.process_pending_commands()

    np.testing.assert_allclose(component.model.goal_js().position, newest)


def test_ros_publication_uses_an_immutable_base_snapshot():
    component = _component()

    snapshot = component.publish(_Stamp())

    assert component.latest_snapshot is snapshot
    assert snapshot.sequence == 0
    assert snapshot.simulation_time == 1.0
    assert snapshot.measured_js.names == tuple(
        joint.name for joint in component.config.joints
    )
    assert not snapshot.measured_js.position.flags.writeable
