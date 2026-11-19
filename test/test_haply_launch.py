import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.utilities import perform_substitutions
import pytest


@pytest.mark.parametrize(
    ("exercise", "expected"),
    [
        (None, "test_cube_objects.yaml"),
        ("test_cube.yaml", "test_cube_objects.yaml"),
        ("ball_tray_objects.yaml", "ball_tray_objects.yaml"),
    ],
)
def test_haply_launch_uses_isaac_cart_and_exercise(exercise, expected):
    path = Path(__file__).resolve().parents[1] / "launch" / "haply.launch.py"
    spec = importlib.util.spec_from_file_location("isaac_haply_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations["headless"] = "true"
    if exercise is not None:
        context.launch_configurations["exercise"] = exercise

    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    session = next(action for action in description.entities if isinstance(action, OpaqueFunction))
    actions = session.execute(context)
    commands = [
        [perform_substitutions(context, part) for part in action.cmd]
        for action in actions if isinstance(action, ExecuteProcess)
    ]
    simulator = next(command for command in commands if "--scene" in command)
    assert [Path(simulator[index + 1]).name for index, value in enumerate(simulator) if value == "--scene"] == [
        "ECM_PSM1_PSM2_PSM3_mono_rtsp.yaml", expected,
    ]
    assert simulator[simulator.index("--headless") + 1] == "true"
    system = next(command for command in commands if "--json-config" in command)
    assert Path(system[system.index("--json-config") + 1]).name == "system-Haply.json"
    # No GStreamer camera viewer: only the simulator and the dVRK system run.
    assert len(commands) == 2
