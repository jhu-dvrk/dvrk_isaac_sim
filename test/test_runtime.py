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
    assert options.collision_backend == "newton"
    assert options.newton_device == "auto"


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


def test_arm_servo_preserves_independent_jaw_move():
    arm = IsaacSimArm(_psm1_robot(), CommandMailboxes(), manifest_path=None)
    arm.commands.submit_discrete("jaw/move_jp", 0.5)
    arm.prepare_step(1.0 / 120.0, 1.0)
    assert arm.jaw_trajectory is not None
    arm.commands.submit_servo("servo_jp", arm.config.home_position)
    arm.prepare_step(1.0 / 120.0, 1.01)
    assert arm.jaw_trajectory is not None


def test_scene_shape_objects_author_rotation_and_unscaled_placement(monkeypatch):
    pytest.importorskip("pxr")
    import math
    import sys
    import types

    from pxr import Gf, Usd, UsdGeom

    from dvrk_isaac_sim.runtime import IsaacSimRuntime
    from dvrk_simulator_base.scene import SceneConfig, SceneObject

    stage = Usd.Stage.CreateInMemory()

    omni = types.ModuleType("omni")
    omni_usd = types.ModuleType("omni.usd")
    context = types.SimpleNamespace(get_stage=lambda: stage)
    omni_usd.get_context = lambda: context
    omni.usd = omni_usd
    importer = types.ModuleType("isaacsim.asset.importer.urdf.impl")
    importer.URDFImporter = object
    importer.URDFImporterConfig = object
    for name, module in {
        "omni": omni,
        "omni.usd": omni_usd,
        "isaacsim": types.ModuleType("isaacsim"),
        "isaacsim.asset": types.ModuleType("isaacsim.asset"),
        "isaacsim.asset.importer": types.ModuleType("isaacsim.asset.importer"),
        "isaacsim.asset.importer.urdf": types.ModuleType("isaacsim.asset.importer.urdf"),
        "isaacsim.asset.importer.urdf.impl": importer,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)

    half_sqrt2 = math.sqrt(0.5)
    runtime = IsaacSimRuntime.__new__(IsaacSimRuntime)
    runtime.scene_config = SceneConfig(
        name="test",
        robots=(),
        objects=(
            SceneObject(
                name="table",
                asset=None,
                fixed=True,
                position=(0.0, 0.0, -0.025),
                orientation_xyzw=(0.0, 0.0, half_sqrt2, half_sqrt2),
                settings={"size": (0.8, 0.8, 0.08)},
                shape="box",
            ),
        ),
    )
    runtime._load_scene_objects()

    prim = stage.GetPrimAtPath("/World/Environment/table")
    xform = UsdGeom.Xformable(prim)
    assert [op.GetOpName() for op in xform.GetOrderedXformOps()] == [
        "xformOp:translate", "xformOp:orient", "xformOp:scale",
    ]
    orient = prim.GetAttribute("xformOp:orient").Get()
    assert orient is not None
    assert orient.GetReal() == pytest.approx(half_sqrt2)
    assert tuple(orient.GetImaginary()) == pytest.approx((0.0, 0.0, half_sqrt2))
    matrix = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    transform = Gf.Transform(matrix)
    # placement must not be multiplied by the box scale
    assert tuple(transform.GetTranslation()) == pytest.approx((0.0, 0.0, -0.025))
    assert tuple(transform.GetScale()) == pytest.approx((0.8, 0.8, 0.08))
