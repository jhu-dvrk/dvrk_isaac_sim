from pathlib import Path

import dvrk_isaac_sim.node as node_module


def test_scene_command_line_argument_is_parsed_without_ros_arguments():
    args = node_module._parse_command_line([
        "--scene", "ECM_PSM1_PSM2.yaml",
        "--ros-args", "-r", "__ns:=/simulation",
    ])
    assert args.scene == [Path("ECM_PSM1_PSM2.yaml")]


def test_headless_command_line_argument_parsed():
    args = node_module._parse_command_line([
        "--headless", "true",
    ])
    assert args.headless == "true"

    args_false = node_module._parse_command_line([
        "--headless", "false",
    ])
    assert args_false.headless == "false"


def test_unknown_scene_is_reported_without_a_traceback(capsys):
    assert node_module.main(["--scene", "does-not-exist"]) == 2
    error = capsys.readouterr().err
    assert "Scene configuration not found: does-not-exist" in error
    assert "Searched scene paths:" in error
    assert "Traceback" not in error
