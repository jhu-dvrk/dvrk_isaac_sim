from dvrk_isaac_sim import python_runtime


def test_explicit_interpreter_is_saved_and_reused(tmp_path, monkeypatch):
    interpreter = (tmp_path / "venv" / "bin" / "python").absolute()
    monkeypatch.setenv("DVRK_ISAAC_SIM_PYTHON", str(interpreter))
    monkeypatch.setattr(
        python_runtime, "_imports_isaac_sim", lambda candidate: candidate == interpreter
    )

    explicit = python_runtime.resolve_isaac_sim_python(tmp_path)
    assert explicit.path == interpreter
    assert explicit.source == "DVRK_ISAAC_SIM_PYTHON"
    assert (tmp_path / "python-runtime.json").is_file()

    monkeypatch.delenv("DVRK_ISAAC_SIM_PYTHON")
    cached = python_runtime.resolve_isaac_sim_python(tmp_path)
    assert cached.path == interpreter
    assert cached.source == f"saved selection in {tmp_path / 'python-runtime.json'}"
