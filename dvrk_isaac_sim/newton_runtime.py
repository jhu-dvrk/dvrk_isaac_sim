"""Locate isolated Newton dependencies without importing them into Isaac."""

import os
from pathlib import Path


def resolve_newton_python(selected=None):
    configured = selected or os.environ.get("DVRK_NEWTON_PYTHON")
    if configured:
        candidates = [Path(configured).expanduser()]
    else:
        package = Path(__file__).resolve().parent.parent
        candidates = [package / ".venv-1/bin/python", package / ".venv-newton/bin/python"]
        for parent in package.parents:
            if (parent / "src").is_dir():
                candidates.append(parent / ".venv-newton/bin/python")
                candidates.append(parent / "src/dvrk/dvrk_isaac_sim/.venv-1/bin/python")
                break
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate.absolute()
    raise ValueError(
        "No isolated Newton Python found. Set DVRK_NEWTON_PYTHON or --newton-python "
        "to an environment with newton[sim,importers]==1.6.0 and usd-core."
    )