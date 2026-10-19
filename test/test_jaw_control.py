import math

from dvrk_isaac_sim.jaw_control import JawPIDConfig, JawPIDController


def test_jaw_pid_tracks_command_without_teleporting():
    controller = JawPIDController(JawPIDConfig(
        lower=-0.5,
        upper=1.5,
        kp=40.0,
        kd=5.0,
        max_effort=10.0,
        max_velocity=0.5,
        target_rate=0.25,
    ))

    controller.set_command(1.0)
    first = controller.step(0.1)
    assert 0.0 < first.target <= 0.025 + 1e-12
    assert 0.0 < first.position < 1.0
    assert first.velocity <= 0.5

    previous = first.position
    for _ in range(100):
        state = controller.step(0.1)
        assert state.position >= previous - 1e-12
        assert abs(state.velocity) <= 0.5 + 1e-12
        previous = state.position
    assert math.isclose(state.commanded_target, 1.0)


def test_jaw_pid_saturates_effort_and_limits_integral():
    controller = JawPIDController(JawPIDConfig(
        lower=-0.1,
        upper=0.1,
        kp=1000.0,
        ki=10.0,
        kd=0.0,
        max_effort=0.2,
        max_velocity=10.0,
        integral_limit=0.01,
        target_rate=0.0,
    ))

    controller.set_command(0.1)
    state = controller.step(0.1)
    assert state.saturated
    assert state.effort == 0.2
    assert abs(state.integral) <= 0.01


def test_jaw_pid_hold_clears_target_and_effort():
    controller = JawPIDController(JawPIDConfig(lower=-0.5, upper=1.0))
    controller.set_command(1.0)
    controller.step(0.05)

    controller.hold()
    state = controller.state
    assert state.commanded_target == state.position
    assert state.target == state.position
    assert state.effort == 0.0
    assert state.integral == 0.0


def test_jaw_pid_guard_stops_at_wall_and_allows_retreat():
    controller = JawPIDController(JawPIDConfig(lower=-0.5, upper=1.0))
    controller.set_motion_guard(lambda previous, proposed: min(proposed, 0.05))
    controller.set_command(1.0)
    for _ in range(100):
        state = controller.step(0.01)
        assert state.position <= 0.05
    assert state.position == 0.05
    assert state.velocity == 0.0
    assert state.integral == 0.0
    controller.set_command(-0.1)
    for _ in range(150):
        state = controller.step(0.01)
    assert state.position < 0.0
