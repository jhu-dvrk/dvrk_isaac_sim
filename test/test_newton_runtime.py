import pytest

from dvrk_isaac_sim.newton_runtime import resolve_newton_python


def test_explicit_newton_python_preserves_venv_symlink(tmp_path):
    target = tmp_path / "runtime"
    target.write_text("", encoding="ascii")
    target.chmod(0o755)
    executable = tmp_path / "python"
    executable.symlink_to(target)
    assert resolve_newton_python(executable) == executable


def test_environment_selects_newton_python(tmp_path, monkeypatch):
    executable = tmp_path / "python"
    executable.write_text("", encoding="ascii")
    executable.chmod(0o755)
    monkeypatch.setenv("DVRK_NEWTON_PYTHON", str(executable))
    assert resolve_newton_python() == executable


def test_invalid_explicit_python_does_not_silently_fall_back(tmp_path):
    with pytest.raises(ValueError, match="No isolated Newton Python"):
        resolve_newton_python(tmp_path / "missing")