"""Run a bounded headless isaac_sim scene smoke test."""

from dvrk_simulator_base.launch import scene_test_launch


def generate_launch_description():
    return scene_test_launch("dvrk_isaac_sim")
