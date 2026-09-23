"""Standalone ROS 2 node lifecycle for one configured CRTK component."""

from __future__ import annotations

from pathlib import Path

from dvrk_arm_description import load_robot_config
from .kinematics import CRTKECM, CRTKPSM
from .ros_interface import CRTKROSComponent


class CRTKROSNode:
    """ROS 2 node for one configured PSM or ECM."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node

        class _Node(Node):
            pass

        self._node = _Node("dvrk_isaac_sim")
        self._node.declare_parameter("robot_config", "")
        self._node.declare_parameter("update_rate_hz", 120.0)
        config_path = self._node.get_parameter("robot_config").get_parameter_value().string_value
        if not config_path:
            raise ValueError("robot_config ROS parameter is required")
        config = load_robot_config(Path(config_path))
        model = CRTKPSM(config) if config.type == "PSM" else CRTKECM(config)
        self.component = CRTKROSComponent(self._node, config, model)
        self.component.publish_tool_type()
        rate = self._node.get_parameter("update_rate_hz").value
        self._node.create_timer(1.0 / float(rate), self._update)
        self._last_time = self._node.get_clock().now()

    def _update(self) -> None:
        now = self._node.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now
        self.component.model.step(max(0.0, dt))
        self.component.publish(now.to_msg())

    def spin(self) -> None:
        import rclpy
        rclpy.spin(self._node)

    def destroy(self) -> None:
        self._node.destroy_node()


def main() -> None:
    import rclpy

    rclpy.init()
    node = None
    try:
        node = CRTKROSNode()
        node.spin()
    finally:
        if node is not None:
            node.destroy()
        rclpy.shutdown()
