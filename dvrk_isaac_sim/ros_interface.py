"""ROS 2 CRTK interface for one backend-independent PSM or ECM model."""

from __future__ import annotations

import time
import threading
from typing import Iterable

import numpy as np

from dvrk_simulator_base.command_validation import (
    jaw_position_from_message,
    joint_positions_from_message,
    pose_from_message as _pose_from_ros,
)
from dvrk_simulator_base.command_mailbox import CommandEnvelope, CommandMailboxes
from dvrk_simulator_base.cartesian_frames import (
    _VIEW_TO_OPTICAL_ROTATION,
    compose_pose as _compose_pose,
    inverse_pose as _inverse_pose,
    relative_pose as _relative_pose,
    relative_twist as _relative_twist,
    view_pose_from_optical as _view_pose_from_optical,
)
from dvrk_arm_description import RobotConfig
from dvrk_simulator_base.operating_state import CRTKOperatingState
from dvrk_simulator_base.ros_qos import (
    transient_local_event_qos,
    transient_local_latched_qos,
)
from dvrk_simulator_base.rotations import (
    quaternion_matrix_xyzw as _quaternion_matrix_xyzw,
    rotation_to_quaternion_xyzw as _quaternion_xyzw,
)
from dvrk_simulator_base.snapshots import ArmSnapshot, OperatingStateSnapshot
from dvrk_simulator_base.types import JointState as SnapshotJointState
from dvrk_simulator_base.types import Pose, Pose as SnapshotPose
from dvrk_simulator_base.types import Twist as SnapshotTwist
from .kinematics import CRTKECM, CRTKPSM, CRTKComponent




class CRTKROSComponent:
    """ROS 2 adapter exposing CRTK topics for one kinematic component."""

    def __init__(self, node, config: RobotConfig, model: CRTKComponent,
                 simulation_stamp=None,
                 component_lock: threading.RLock | None = None):
        from crtk_msgs.msg import OperatingState, StringStamped
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String

        self.node = node
        self._component_lock = component_lock or threading.RLock()
        self.config = config
        self.model = model
        self._JointState = JointState
        self._PoseStamped = PoseStamped
        self._TwistStamped = TwistStamped
        self._OperatingState = OperatingState
        self._StringStamped = StringStamped
        initial_state = CRTKOperatingState.ENABLED
        self._operating_state = CRTKOperatingState(initial_state)
        self._has_jaw = config.type == "PSM"
        jaw_config = config.raw.get("robot", {}).get("jaw", {})
        self._jaw_lower = float(jaw_config.get("lower", -0.349066))
        self._jaw_upper = float(jaw_config.get("upper", 1.39626))
        self._jaw_position = 0.0
        self._jaw_velocity = 0.0
        self._frame_id = str(config.raw.get("robot", {}).get("cartesian", {}).get("reference_frame", config.parent_frame))
        self._cartesian_reference: CRTKComponent | None = None
        self._simulation_stamp = simulation_stamp
        asset = config.raw.get("robot", {}).get("asset", {})
        instrument = asset.get("instrument") if isinstance(asset, dict) else None
        self._instrument_name = str(instrument) if instrument not in (None, "") else None
        self._cartesian_reference_frame = self._frame_id
        # ROS callbacks only validate and enqueue.  Isaac owns model mutation
        # when its control loop drains these bounded mailboxes.
        self.commands = CommandMailboxes()
        self._commands_applied = 0
        self._commands_rejected = 0
        self._last_command_age_ms: float | None = None
        self._snapshot_sequence = 0
        self._latest_snapshot: ArmSnapshot | None = None
        self._operating_state_event_pending = True
        self._last_published_busy = None
        self._motion_busy = False
        self._motion_start_stamp: tuple[int, int] | None = None
        self._motion_failure_pending = False
        self._last_ik_warning = None
        self._last_ik_warning_time = 0.0
        self._ik_warning_count = 0
        # Static world pose of this PSM base frame. The ECM view pose is
        # dynamic and is evaluated from ECM FK for every conversion.
        self._base_pose = Pose(
            config.base_position.copy(),
            _quaternion_matrix_xyzw(config.base_orientation_xyzw),
        )
        state_qos = transient_local_event_qos()

        self.measured_js_publisher = node.create_publisher(JointState, "measured_js", 10)
        self.measured_cp_publisher = node.create_publisher(PoseStamped, "measured_cp", 10)
        self.setpoint_cp_publisher = node.create_publisher(PoseStamped, "setpoint_cp", 10)
        self.measured_cv_publisher = node.create_publisher(TwistStamped, "measured_cv", 10)
        if self._has_jaw:
            self.jaw_measured_js_publisher = node.create_publisher(JointState, "jaw/measured_js", 10)
            self.jaw_setpoint_js_publisher = node.create_publisher(JointState, "jaw/setpoint_js", 10)
        self.setpoint_js_publisher = node.create_publisher(JointState, "setpoint_js", 10)
        self.operating_state_publisher = node.create_publisher(OperatingState, "operating_state", state_qos)
        self.state_publisher = node.create_publisher(StringStamped, "state", state_qos)
        self.info_publisher = node.create_publisher(StringStamped, "info", 10)
        self.warning_publisher = node.create_publisher(StringStamped, "warning", 10)
        self.error_publisher = node.create_publisher(StringStamped, "error", 10)
        self.tool_type_publisher = (
            node.create_publisher(String, "tool_type", transient_local_latched_qos())
            if config.type == "PSM" else None
        )

        def locked(callback):
            def invoke(message):
                with self._component_lock:
                    callback(message)
            return invoke

        node.create_subscription(
            JointState, "move_jp", locked(self._move_jp_callback), 10
        )
        # Servo inputs are superseding setpoints. A depth-one queue prevents
        # old controller samples from being replayed after a brief CPU/render
        # stall, which would add visible arm and jaw lag.
        node.create_subscription(
            JointState, "servo_jp", locked(self._servo_jp_callback), 1
        )
        node.create_subscription(
            PoseStamped, "move_cp", locked(self._move_cp_callback), 10
        )
        node.create_subscription(
            PoseStamped, "servo_cp", locked(self._servo_cp_callback), 1
        )
        if self._has_jaw:
            self._jaw_move_subscription = node.create_subscription(
                JointState, "jaw/move_jp", locked(self._jaw_servo_jp_callback), 10
            )
            self._jaw_servo_subscription = node.create_subscription(
                JointState, "jaw/servo_jp", locked(self._jaw_servo_jp_callback), 1
            )
        self._state_command_subscription = node.create_subscription(
            StringStamped, "state_command", locked(self._state_command_callback), 10
        )
        self._publish_operating_state(self._event_stamp())
        self._publish_info("initialized; state is ENABLED")

    def publish_tool_type(self) -> None:
        """Publish the configured six-digit PSM tool type once.

        The publisher is transient-local, so subscribers that join after the
        simulation starts still receive the retained identifier.  It is not
        published during component construction because that precedes the
        Isaac timeline start.
        """
        if self.config.type != "PSM" or self._instrument_name is None:
            return
        from std_msgs.msg import String

        message = String()
        message.data = self._instrument_name
        if self.tool_type_publisher is not None:
            self.tool_type_publisher.publish(message)

    def set_simulation_stamp(self, stamp) -> None:
        """Set the simulation timestamp used by event messages.

        The Isaac runner updates this before processing each simulation step so
        state, diagnostics, and periodic CRTK messages share one time source.
        Standalone ROS-node use retains the node clock until this is called.
        """
        self._simulation_stamp = stamp

    def _event_stamp(self):
        if self._simulation_stamp is not None:
            return self._simulation_stamp
        return self.node.get_clock().now().to_msg()

    def _publish_message(self, publisher, message: str) -> None:
        event = self._StringStamped()
        event.header.stamp = self._event_stamp()
        event.header.frame_id = self._frame_id
        event.string = str(message)
        publisher.publish(event)

    def _publish_info(self, message: str) -> None:
        self._publish_message(self.info_publisher, message)

    def _publish_warning(self, message: str) -> None:
        self._publish_message(self.warning_publisher, message)

    def _publish_error(self, message: str) -> None:
        self._publish_message(self.error_publisher, message)

    def _ik_failure(self, command: str, message: str) -> None:
        text = f"{command} IK failed: {message}"
        now = time.monotonic()
        if text != self._last_ik_warning:
            self._last_ik_warning = text
            self._last_ik_warning_time = now
            self._ik_warning_count = 1
            self._publish_warning(text)
            self.node.get_logger().warning(f"{self.config.name} {text}")
            return

        self._ik_warning_count += 1
        if now - self._last_ik_warning_time >= 1.0:
            self._last_ik_warning_time = now
            self._publish_warning(
                f"{text} (repeated {self._ik_warning_count} times)"
            )
            self.node.get_logger().warning(
                f"{self.config.name} {text} "
                f"(repeated {self._ik_warning_count} times)"
            )

    def _clear_ik_warning(self) -> None:
        self._last_ik_warning = None
        self._last_ik_warning_time = 0.0
        self._ik_warning_count = 0

    def set_cartesian_reference(self, reference: CRTKComponent, frame_id: str) -> None:
        """Use a moving ECM pose as the PSM Cartesian reference frame.

        PSM Cartesian ROS topics are expressed in the current dVRK view
        frame derived from ECM optical FK. Joint topics remain local to the
        PSM and are unaffected.
        """
        if self.config.type != "PSM":
            return
        self._cartesian_reference = reference
        self._cartesian_reference_frame = str(frame_id)
        self._frame_id = self._cartesian_reference_frame

    def _view_to_base(self) -> Pose:
        """Return the current transform from ECM view coordinates to PSM base."""
        if self._cartesian_reference is None:
            return _inverse_pose(self._base_pose)
        # T_BV = inverse(T_WB) * T_WV. ECM FK supplies T_WC; the fixed
        # optical-to-view rotation supplies T_CV. Both are needed because
        # Cartesian teleoperation is defined in dVRK view axes, not optical axes.
        view_pose = _view_pose_from_optical(self._cartesian_reference.measured_cp())
        return _relative_pose(view_pose, self._base_pose)

    def _world_to_base(self, pose: Pose) -> Pose:
        return _relative_pose(pose, self._base_pose)

    def _base_to_world(self, pose: Pose) -> Pose:
        return _compose_pose(self._base_pose, pose)

    def _pose_for_ros(self, pose: Pose) -> Pose:
        if self._cartesian_reference is None:
            return pose
        # FK pipeline: world -> PSM base -> current ECM view.
        base_tool = self._world_to_base(pose)
        return _relative_pose(base_tool, self._view_to_base())

    def _pose_from_ros(self, pose: Pose, frame_id: str) -> Pose:
        if self._cartesian_reference is None:
            return pose
        # An explicitly world-referenced command remains useful for diagnostics
        # and preserves the single-PSM/no-ECM mode. All normal dVRK commands
        # use ECM_optical (or leave frame_id empty).
        if frame_id and frame_id == self.config.parent_frame:
            return pose
        # Command pipeline: current ECM view -> PSM base -> world, then IK.
        base_target = _compose_pose(self._view_to_base(), pose)
        return self._base_to_world(base_target)

    def _twist_for_ros(self, pose: Pose, twist):
        if self._cartesian_reference is None:
            return twist.linear, twist.angular
        relative = _relative_twist(
            pose, twist, self._cartesian_reference.measured_cp(),
            self._cartesian_reference.measured_cv(),
        )
        # _relative_twist is in ECM optical axes; publish dVRK view axes.
        return (
            _VIEW_TO_OPTICAL_ROTATION.T @ relative.linear,
            _VIEW_TO_OPTICAL_ROTATION.T @ relative.angular,
        )

    @property
    def jaw_position(self) -> float | None:
        """Current logical PSM jaw position in radians."""
        return self._jaw_position if self._has_jaw else None

    def command_metrics(self) -> tuple[int, int, int, int, float | None]:
        """Return cumulative receive/apply counters for latency diagnostics."""
        counters = self.commands.counters
        return (
            counters.received,
            self._commands_applied,
            counters.coalesced,
            counters.rejected + self._commands_rejected,
            self._last_command_age_ms,
        )

    def _command_applied(self, command: CommandEnvelope) -> None:
        self._commands_applied += 1
        self._last_command_age_ms = (
            time.monotonic_ns() - command.received_at_ns
        ) * 1.0e-6

    def _command_rejected(self) -> None:
        self._commands_rejected += 1

    def _submit(self, channel: str, payload, *, servo: bool) -> bool:
        """Queue a command from ROS or the external rqt client.

        This method is safe to call from a ROS executor thread.  It does not
        inspect or mutate the Isaac-owned model.
        """
        if servo:
            self.commands.submit_servo(channel, payload)
            return True
        if self.commands.submit_discrete(channel, payload) is not None:
            return True
        self._command_rejected()
        self._publish_warning(f"rejected {channel}: command queue is full")
        return False

    def command_jaw_position(self, position: float) -> bool:
        """Queue a jaw command from a GUI/ROS-equivalent caller."""
        if not self._has_jaw:
            return False
        try:
            value = float(position)
        except (TypeError, ValueError):
            return False
        if not np.isfinite(value) or not self._jaw_lower <= value <= self._jaw_upper:
            self._publish_warning(
                f"rejected jaw position {value}: "
                f"expected [{self._jaw_lower}, {self._jaw_upper}] radians"
            )
            return False
        return self._submit("jaw/servo_jp", value, servo=True)

    @property
    def operating_state(self) -> str:
        """Current CRTK operating-state name for GUI and diagnostics."""
        return self._operating_state.state

    @property
    def is_homed(self) -> bool:
        return self._operating_state.is_homed

    def command_state(self, command: str) -> bool:
        """Queue a state command through the same path as ROS state_command."""
        return self._submit("state_command", str(command), servo=False)

    def command_joint_position(self, position: Iterable[float]) -> bool:
        """Queue a GUI joint target for the Isaac control loop."""
        try:
            values = np.asarray(list(position), dtype=float)
            if values.shape != (len(self.config.joints),) or not np.all(np.isfinite(values)):
                raise ValueError("joint position has the wrong size or contains non-finite values")
        except (TypeError, ValueError) as error:
            self._publish_warning(f"rejected GUI joint command: {error}")
            return False
        return self._submit("move_jp", values, servo=False)

    def command_cartesian_position(self, pose: Pose) -> bool:
        """Queue a world-frame GUI Cartesian target for the Isaac control loop."""
        return self._submit("move_cp_world", pose, servo=False)

    def _apply_state_command(self, command: CommandEnvelope) -> None:
        success, error = self._operating_state.command(command.payload)
        if not success:
            self._command_rejected()
            self._publish_warning(f"rejected state_command {command.payload!r}: {error}")
            self.node.get_logger().warning(
                f"{self.config.name} rejected state_command {command.payload!r}: {error}"
            )
            return
        if self._operating_state.state in (CRTKOperatingState.DISABLED, CRTKOperatingState.FAULT):
            self.model.move_jp(self.model.measured_js().position)
        self._publish_operating_state(self._event_stamp())
        self._publish_info(f"state is now {self._operating_state.state}")
        self._operating_state_event_pending = True
        self._command_applied(command)

    def _state_command_callback(self, message) -> None:
        self._submit("state_command", str(message.string), servo=False)

    def _jaw_servo_jp_callback(self, message) -> None:
        try:
            position = jaw_position_from_message(message)
        except ValueError as error:
            self._command_rejected()
            self.node.get_logger().warning(f"{self.config.name} rejected jaw/servo_jp: {error}")
            return
        self._submit("jaw/servo_jp", position, servo=True)

    def _motion_allowed(self, command: str) -> bool:
        if self._operating_state.accepts_motion:
            return True
        self.node.get_logger().debug(
            f"{self.config.name} ignored {command}: state is {self._operating_state.state}"
        )
        return False

    def _positions_from_message(self, message) -> np.ndarray:
        return joint_positions_from_message(
            message, (joint.name for joint in self.config.joints)
        )

    def _move_jp_callback(self, message) -> None:
        try:
            target = self._positions_from_message(message)
        except ValueError as error:
            self._command_rejected()
            self.node.get_logger().warning(f"{self.config.name} rejected move_jp: {error}")
            return
        self._submit("move_jp", target, servo=False)

    def _servo_jp_callback(self, message) -> None:
        try:
            target = self._positions_from_message(message)
        except ValueError as error:
            self._command_rejected()
            self.node.get_logger().warning(f"{self.config.name} rejected servo_jp: {error}")
            return
        self._submit("servo_jp", target, servo=True)

    def _move_cp_callback(self, message) -> None:
        try:
            payload = (_pose_from_ros(message), str(message.header.frame_id))
        except ValueError as error:
            self._command_rejected()
            self._publish_warning(f"rejected move_cp: {error}")
            self.node.get_logger().warning(f"{self.config.name} rejected move_cp: {error}")
            return
        self._submit("move_cp", payload, servo=False)

    def _servo_cp_callback(self, message) -> None:
        try:
            payload = (_pose_from_ros(message), str(message.header.frame_id))
        except ValueError as error:
            self._command_rejected()
            self._publish_warning(f"rejected servo_cp: {error}")
            self.node.get_logger().warning(f"{self.config.name} rejected servo_cp: {error}")
            return
        self._submit("servo_cp", payload, servo=True)

    def process_pending_commands(self) -> None:
        """Apply all queued commands from the Isaac-owned control loop."""
        for command in self.commands.drain():
            if command.channel == "state_command":
                self._apply_state_command(command)
                continue
            if not self._motion_allowed(command.channel):
                self._command_rejected()
                if command.channel.startswith("move_"):
                    self._publish_motion_failure()
                continue
            try:
                if command.channel == "jaw/servo_jp":
                    position = float(command.payload)
                    if not self._jaw_lower <= position <= self._jaw_upper:
                        raise ValueError("jaw position exceeds configured limits")
                    self._jaw_position = position
                    self._jaw_velocity = 0.0
                elif command.channel == "move_jp":
                    self.model.move_jp(command.payload)
                    self._publish_motion_edges()
                elif command.channel == "servo_jp":
                    self.model.servo_jp(command.payload)
                elif command.channel in {"move_cp", "servo_cp", "move_cp_world"}:
                    if command.channel == "move_cp_world":
                        target = command.payload
                    else:
                        pose, frame_id = command.payload
                        target = self._pose_from_ros(pose, frame_id)
                    result = self.model.move_cp(target)
                    if not result.success:
                        self._ik_failure(command.channel, result.message)
                        if command.channel.startswith("move_"):
                            self._publish_motion_failure()
                        self._command_rejected()
                        continue
                    self._clear_ik_warning()
                    if command.channel.startswith("move_"):
                        self._publish_motion_edges()
                else:
                    raise ValueError(f"unsupported command channel {command.channel!r}")
            except ValueError as error:
                self._command_rejected()
                self._publish_warning(f"rejected {command.channel}: {error}")
                self.node.get_logger().warning(
                    f"{self.config.name} rejected {command.channel}: {error}"
                )
                continue
            self._command_applied(command)

    def _publish_jaw_state(self, stamp) -> None:
        if not self._has_jaw:
            return
        for publisher in (self.jaw_measured_js_publisher, self.jaw_setpoint_js_publisher):
            jaw = self._JointState()
            jaw.header.stamp = stamp
            jaw.header.frame_id = self._frame_id
            jaw.name = ["jaw"]
            jaw.position = [self._jaw_position]
            jaw.velocity = [self._jaw_velocity]
            publisher.publish(jaw)

    def _publish_operating_state(self, stamp, only_on_change: bool = False,
                                 busy_override: bool | None = None) -> None:
        """Publish state, optionally only when the motion busy flag changes."""
        busy = (
            busy_override
            if busy_override is not None
            else self._operating_state.accepts_motion and self._motion_busy
        )
        if only_on_change and busy == self._last_published_busy:
            return
        self._last_published_busy = busy
        operating_state = self._OperatingState()
        operating_state.header.stamp = stamp
        operating_state.header.frame_id = self._frame_id
        operating_state.state = self._operating_state.state
        operating_state.is_homed = self._operating_state.is_homed
        operating_state.is_busy = busy
        self.operating_state_publisher.publish(operating_state)

        state = self._StringStamped()
        state.header.stamp = stamp
        state.header.frame_id = self._frame_id
        state.string = self._operating_state.state
        self.state_publisher.publish(state)

    def _publish_motion_edges(self) -> None:
        """Publish a move-start edge and defer completion to a later tick."""
        stamp = self._event_stamp()
        self._motion_busy = True
        self._motion_start_stamp = (int(stamp.sec), int(stamp.nanosec))
        self._motion_failure_pending = False
        self._publish_operating_state(stamp, busy_override=True)

    def _publish_motion_failure(self) -> None:
        """Complete a rejected move handle without leaving it blocked."""
        stamp = self._event_stamp()
        self._motion_busy = True
        self._motion_start_stamp = (int(stamp.sec), int(stamp.nanosec))
        self._motion_failure_pending = True
        self._publish_operating_state(stamp, busy_override=True)

    @property
    def latest_snapshot(self) -> ArmSnapshot | None:
        """Most recent immutable simulator-owned state for ROS/GUI consumers."""
        return self._latest_snapshot

    def _capture_snapshot(self, stamp, valid: bool) -> ArmSnapshot:
        """Capture backend state after command application and kinematic stepping."""
        measured_js = self.model.measured_js()
        setpoint_js = self.model.goal_js()
        measured_pose = self.model.measured_cp()
        measured_twist = self.model.measured_cv()
        simulation_time = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
        snapshot = ArmSnapshot(
            sequence=self._snapshot_sequence,
            simulation_time=simulation_time,
            valid=bool(valid),
            measured_js=SnapshotJointState(
                measured_js.names, measured_js.position, measured_js.velocity
            ),
            setpoint_js=SnapshotJointState(
                setpoint_js.names, setpoint_js.position, setpoint_js.velocity
            ),
            measured_cp_world=SnapshotPose(
                measured_pose.position, measured_pose.orientation
            ),
            # The current kinematic implementation has no independently
            # measured Cartesian controller state.
            setpoint_cp_world=SnapshotPose(
                measured_pose.position, measured_pose.orientation
            ),
            measured_cv_world=SnapshotTwist(
                measured_twist.linear, measured_twist.angular
            ),
            jaw_measured=self._jaw_position if self._has_jaw else None,
            jaw_setpoint=self._jaw_position if self._has_jaw else None,
            operating_state=OperatingStateSnapshot(
                self._operating_state.state,
                self._operating_state.is_homed,
                self._operating_state.accepts_motion and self._motion_busy,
            ),
            operating_state_event=self._operating_state_event_pending,
        )
        self._snapshot_sequence += 1
        self._latest_snapshot = snapshot
        self._operating_state_event_pending = False
        return snapshot

    def publish(self, stamp, valid: bool = True) -> ArmSnapshot:
        """Publish the newest simulator-owned immutable state snapshot."""
        if not valid:
            stamp = type(stamp)()
        snapshot = self._capture_snapshot(stamp, valid)
        joint_state = snapshot.measured_js
        world_pose = snapshot.measured_cp_world
        pose = self._pose_for_ros(world_pose)
        linear, angular = self._twist_for_ros(world_pose, snapshot.measured_cv_world)

        measured_js = self._JointState()
        measured_js.header.stamp = stamp
        measured_js.header.frame_id = self._frame_id
        measured_js.name = list(joint_state.names)
        measured_js.position = joint_state.position.tolist()
        measured_js.velocity = joint_state.velocity.tolist()
        self.measured_js_publisher.publish(measured_js)

        measured_cp = self._PoseStamped()
        measured_cp.header.stamp = stamp
        measured_cp.header.frame_id = self._frame_id
        measured_cp.pose.position.x, measured_cp.pose.position.y, measured_cp.pose.position.z = pose.position
        measured_cp.pose.orientation.x, measured_cp.pose.orientation.y, measured_cp.pose.orientation.z, measured_cp.pose.orientation.w = _quaternion_xyzw(pose.orientation)
        self.measured_cp_publisher.publish(measured_cp)
        # CRTK teleoperation commonly consumes setpoint_cp from the puppet.
        # This simulator has no separate controller, so it is identical to measured_cp.
        self.setpoint_cp_publisher.publish(measured_cp)

        measured_cv = self._TwistStamped()
        measured_cv.header.stamp = stamp
        measured_cv.header.frame_id = self._frame_id
        measured_cv.twist.linear.x, measured_cv.twist.linear.y, measured_cv.twist.linear.z = linear
        measured_cv.twist.angular.x, measured_cv.twist.angular.y, measured_cv.twist.angular.z = angular
        self.measured_cv_publisher.publish(measured_cv)

        setpoint = self._JointState()
        setpoint.header.stamp = stamp
        setpoint.header.frame_id = self._frame_id
        setpoint.name = list(snapshot.setpoint_js.names)
        setpoint.position = snapshot.setpoint_js.position.tolist()
        self.setpoint_js_publisher.publish(setpoint)
        self._publish_jaw_state(stamp)
        # Servo commands do not affect busy. Complete only a move operation
        # that previously emitted its busy=true edge.
        stamp_key = (int(stamp.sec), int(stamp.nanosec))
        if (
            self._motion_busy
            and (self._motion_failure_pending or not self.model.is_busy())
            and self._motion_start_stamp is not None
            and stamp_key > self._motion_start_stamp
        ):
            self._motion_busy = False
            self._motion_start_stamp = None
            self._motion_failure_pending = False
            self._publish_operating_state(stamp, busy_override=False)
        return snapshot
