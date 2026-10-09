"""Runtime selection of the Python interpreter used for NVIDIA Isaac Sim."""

from __future__ import annotations

import os
from pathlib import Path

from dvrk_simulator_base.python_runtime import (
    CACHE_FILE_NAME,
    SimulatorPython,
    check_python_imports,
    read_cached_python,
    resolve_simulator_python,
    save_cached_python,
)


class IsaacSimPython(SimulatorPython):
    pass


def default_generated_root() -> Path:
    """Return the user cache directory for Isaac Sim artifacts."""
    cache_root = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache"))
    return (cache_root / "dvrk_isaac_sim").resolve()


def _imports_isaac_sim(python: Path) -> bool:
    return check_python_imports(python, "isaacsim")


def _find_candidate_isaac_python() -> tuple[Path, str] | None:
    """Check standard Isaac Sim installation directories."""
    candidates: list[tuple[Path, str]] = []
    isaac_sim_dir = os.environ.get("ISAAC_SIM_DIR", "").strip()
    if isaac_sim_dir:
        dir_path = Path(isaac_sim_dir).expanduser()
        candidates.append((dir_path / "python.sh", f"ISAAC_SIM_DIR ({dir_path})"))
        candidates.append((dir_path / "_build/linux-x86_64/release/python.sh", f"ISAAC_SIM_DIR ({dir_path})"))
    candidates.append((Path.home() / "devel/isaac/python.sh", "~/devel/isaac/python.sh"))
    candidates.append((Path.home() / "isaac-sim/python.sh", "~/isaac-sim/python.sh"))
    pkg_dir = Path.home() / ".local/share/ov/pkg"
    if pkg_dir.is_dir():
        for path in sorted(pkg_dir.glob("isaac*sim*/python.sh"), reverse=True):
            candidates.append((path, f"Omniverse installation at {path}"))

    for candidate, desc in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK) and _imports_isaac_sim(candidate):
            return candidate.resolve(), desc
    return None


def resolve_isaac_sim_python(
    generated_root: str | Path | None = None,
) -> IsaacSimPython:
    """Select an Isaac Sim interpreter and record a portable workspace cache.

    Precedence:
    1. Explicit ``DVRK_ISAAC_SIM_PYTHON`` environment variable override.
    2. Cached interpreter from a previous run.
    3. Candidate Isaac Sim installation (e.g. ``ISAAC_SIM_DIR`` or ``~/devel/isaac/python.sh``).
    4. Workspace virtual environment.
    5. Current running Python interpreter.
    """
    root = Path(
        generated_root or default_generated_root()
    ).expanduser().resolve()
    cache_path = root / CACHE_FILE_NAME

    if not os.environ.get("DVRK_ISAAC_SIM_PYTHON", "").strip():
        cached = read_cached_python(cache_path)
        if cached is None or not _imports_isaac_sim(cached):
            candidate_info = _find_candidate_isaac_python()
            if candidate_info is not None:
                candidate_path, _ = candidate_info
                save_cached_python(cache_path, candidate_path)

    return resolve_simulator_python(
        simulator_name="Isaac Sim",
        env_var="DVRK_ISAAC_SIM_PYTHON",
        generated_root=root,
        default_generated_root=default_generated_root(),
        check_import_fn=_imports_isaac_sim,
        workspace_venv_names=(".venv-isaac", ".venv"),
        result_factory=IsaacSimPython,
        source_file=__file__,
    )
