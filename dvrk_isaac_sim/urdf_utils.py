"""Shared helpers for URDF-derived paths and numeric attributes."""

from __future__ import annotations

from pathlib import Path


def numbers(value: str | None, count: int, default: float = 0.0) -> list[float]:
    """Parse a fixed-length numeric URDF attribute, falling back on defaults."""
    if value is None:
        return [default] * count
    values = [float(item) for item in value.split()]
    if len(values) != count:
        return [default] * count
    return values


def package_mesh_path(model_root: Path, filename: str, package: str = "dvrk_model") -> Path | None:
    """Resolve a ``package://`` mesh path inside a known package root."""
    prefix = f"package://{package}/"
    if not filename.startswith(prefix):
        return None
    return model_root / filename[len(prefix):]
