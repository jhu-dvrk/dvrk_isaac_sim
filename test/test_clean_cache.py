from dvrk_isaac_sim.clean_cache import main


def test_configured_cache_removes_assets_and_preserves_other_files(tmp_path):
    cache = tmp_path / "assets"
    robot = cache / "PSM1_420006"
    robot.mkdir(parents=True)
    (robot / "kinematics.json").write_text("{}")
    (robot / "PSM1.usda").write_text("#usda 1.0")
    unrelated = cache / "unrelated"
    unrelated.mkdir()
    runtime = cache / "python-runtime.json"
    runtime.write_text("{}")
    config = tmp_path / "runtime.yaml"
    config.write_text("generated_root: assets\n")
    assert main(["--config", str(config)]) == 0
    assert not robot.exists()
    assert unrelated.is_dir()
    assert runtime.is_file()
