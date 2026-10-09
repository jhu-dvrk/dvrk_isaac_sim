from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest

from dvrk_arm_description import load_robot_config
from dvrk_isaac_sim.runtime import IsaacSimArm, RuntimeOptions
from dvrk_simulator_base.command_mailbox import CommandMailboxes
from dvrk_simulator_base.operating_state import CRTKOperatingState
from dvrk_simulator_base.types import Pose


def _psm1_robot():
    share = Path(get_package_share_directory("dvrk_arm_description"))
    config_path = share / "arms/PSM1.yaml"
    return load_robot_config(config_path, instrument="420006")


def test_runtime_options_defaults():
    options = RuntimeOptions()
    assert options.headless is False
    assert options.renderer == "RaytracedLighting"
    assert options.simulation_rate_hz == 120.0
    assert options.render_rate_hz == 30.0


def test_arm_initial_snapshot_and_stepping():
    robot_config = _psm1_robot()
    commands = CommandMailboxes()
    arm = IsaacSimArm(robot_config, commands, manifest_path=None)

    snapshot = arm.snapshot(simulation_time=0.0, sequence=0)
    assert snapshot.valid is True
    assert snapshot.sequence == 0
    assert snapshot.simulation_time == 0.0
    np.testing.assert_allclose(
        snapshot.measured_js.position, [0.0, 0.0, 0.12, 0.0, 0.0, 0.0]
    )
    assert isinstance(snapshot.measured_cp_world, Pose)
    assert snapshot.operating_state.state == CRTKOperatingState.ENABLED
    assert snapshot.operating_state.is_busy is False


def test_arm_command_mailbox_servos():
    robot_config = _psm1_robot()
    commands = CommandMailboxes()
    arm = IsaacSimArm(robot_config, commands, manifest_path=None)

    target_q = np.array([0.1, 0.1, 0.15, 0.2, -0.1, 0.1])
    arm.commands.submit_servo("servo_jp", target_q)
    arm.commands.submit_servo("jaw/servo_jp", 0.5)

    arm.prepare_step(dt=0.01, now=1.0)
    snapshot = arm.snapshot(simulation_time=0.01, sequence=1)

    np.testing.assert_allclose(snapshot.setpoint_js.position, target_q)
    assert snapshot.jaw_setpoint == pytest.approx(0.5)
    assert snapshot.jaw_measured == pytest.approx(0.5)


def test_arm_operating_state_disable_rejects_motion():
    robot_config = _psm1_robot()
    commands = CommandMailboxes()
    arm = IsaacSimArm(robot_config, commands, manifest_path=None)

    arm.commands.submit_discrete("state_command", "disable")
    arm.prepare_step(dt=0.01, now=1.0)
    snapshot = arm.snapshot(simulation_time=0.01, sequence=1)
    assert snapshot.operating_state.state == CRTKOperatingState.DISABLED

    # Submit servo while disabled
    target_q = np.array([0.1, 0.1, 0.15, 0.2, -0.1, 0.1])
    arm.commands.submit_servo("servo_jp", target_q)
    arm.prepare_step(dt=0.01, now=1.01)
    snapshot2 = arm.snapshot(simulation_time=0.02, sequence=2)

    # Should remain at home position
    np.testing.assert_allclose(
        snapshot2.setpoint_js.position, [0.0, 0.0, 0.12, 0.0, 0.0, 0.0]
    )


def test_completed_jaw_move_keeps_requested_setpoint():
    arm = IsaacSimArm(_psm1_robot(), CommandMailboxes(), manifest_path=None)
    arm.commands.submit_discrete("jaw/move_jp", 0.5)
    arm.prepare_step(0.01, 1.0)
    arm.prepare_step(0.01, 4.0)
    state = arm.snapshot(0.02, 2)
    assert state.jaw_measured == pytest.approx(0.5)
    assert state.jaw_setpoint == pytest.approx(0.5)
    assert not state.operating_state.is_busy


def test_cartesian_snapshot_tracks_setpoint_and_measured_velocity():
    arm = IsaacSimArm(_psm1_robot(), CommandMailboxes(), manifest_path=None)
    target = arm.config.home_position.copy()
    target[0] += 0.1
    arm.commands.submit_servo("servo_jp", target)
    arm.prepare_step(0.01, 1.0)
    state = arm.snapshot(0.01, 1)
    np.testing.assert_allclose(state.setpoint_cp_world.position, arm.model.compute_fk(target).position)
    assert not np.allclose(state.setpoint_cp_world.position, state.measured_cp_world.position)
    velocity = arm.model.measured_cv()
    assert np.linalg.norm(velocity.angular) > 0
    np.testing.assert_allclose(state.measured_cv_world.angular, velocity.angular)
    np.testing.assert_allclose(state.measured_cv_world.linear, velocity.linear)


def test_render_rate_and_camera_pose_order(monkeypatch):
    import dvrk_isaac_sim.runtime as module
    from dvrk_isaac_sim.runtime import IsaacSimRuntime

    clock = [0.0]
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])
    robot = load_robot_config(Path(get_package_share_directory("dvrk_arm_description")) / "arms/ECM.yaml")
    commands = {robot.name: CommandMailboxes()}
    runtime = IsaacSimRuntime([robot], RuntimeOptions(simulation_rate_hz=120, render_rate_hz=30), commands)
    runtime.arms[robot.name] = IsaacSimArm(robot, commands[robot.name], None)
    events = []

    class App:
        def update(self): events.append("render")

    class Camera:
        def set_pose(self, pose): events.append("pose")
        def capture(self, stamp): events.append("capture")

    runtime.simulation_app = App()
    runtime.camera = Camera()
    for index in range(12):
        clock[0] = index / 120
        runtime.step()
    assert events == ["pose", "render", "capture"] * 3
    assert runtime._simulation_time == pytest.approx(0.1)
