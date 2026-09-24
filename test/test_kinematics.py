from pathlib import Path

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from dvrk_arm_description import load_robot_config
from dvrk_isaac_sim.kinematics import CRTKECM, CRTKPSM, Pose


ROOT = Path(__file__).parents[1]
BASE_ARMS = Path(get_package_share_directory("dvrk_arm_description")) / "arms"


def test_psm_home_pose_and_jacobian():
    robot = CRTKPSM(load_robot_config(BASE_ARMS / "PSM1.yaml"))
    pose = robot.measured_cp()
    assert pose.position.shape == (3,)
    assert pose.orientation.shape == (3, 3)
    np.testing.assert_allclose(pose.orientation @ pose.orientation.T, np.eye(3), atol=1e-12)
    assert robot.compute_jacobian().shape == (6, 6)


def test_psm_move_jp_respects_velocity_limit():
    robot = CRTKPSM(load_robot_config(BASE_ARMS / "PSM1.yaml"))
    robot.move_jp([0.5, 0.2, 0.2, 0.1, 0.1, 0.1])
    robot.step(0.1)
    np.testing.assert_allclose(robot.measured_js().position, [0.5, 0.2, 0.16, 0.1, 0.1, 0.1])
    np.testing.assert_allclose(robot.measured_js().velocity, [5.0, 2.0, 0.4, 1.0, 1.0, 1.0])


def test_ecm_has_four_joints():
    robot = CRTKECM(load_robot_config(BASE_ARMS / "ECM.yaml"))
    assert robot.measured_js().names == ("yaw", "pitch", "insertion", "roll")
    assert robot.compute_jacobian().shape == (6, 4)


def test_joint_limits_are_rejected():
    robot = CRTKPSM(load_robot_config(BASE_ARMS / "PSM1.yaml"))
    with pytest.raises(ValueError):
        robot.move_jp([2.0, 0.0, 0.0])


def test_position_ik_reaches_a_nearby_target():
    robot = CRTKPSM(load_robot_config(BASE_ARMS / "PSM1.yaml"))
    target_q = np.array([0.2, -0.1, 0.08, 0.1, -0.1, 0.1])
    target = robot.compute_fk(target_q)
    result = robot.compute_ik(Pose(target.position, target.orientation), seed=[0.0, 0.0, 0.08, 0.0, 0.0, 0.0])
    assert result.success
    np.testing.assert_allclose(robot.compute_fk(result.position).position, target.position, atol=1e-4)


def test_psm_pose_ik_reaches_orientation():
    robot = CRTKPSM(load_robot_config(BASE_ARMS / "PSM1.yaml"))
    target = robot.compute_fk([0.15, -0.1, 0.1, 0.2, -0.15, 0.1])
    result = robot.compute_ik(target, seed=[0.0, 0.0, 0.08, 0.0, 0.0, 0.0])
    assert result.success
    solved = robot.compute_fk(result.position)
    np.testing.assert_allclose(solved.position, target.position, atol=1e-4)
    np.testing.assert_allclose(solved.orientation, target.orientation, atol=1e-4)
