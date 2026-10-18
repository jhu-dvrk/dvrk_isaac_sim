"""Collision backend adapters for the Isaac simulator runtime."""

from __future__ import annotations

from pathlib import Path
from typing import Any


class NoCollisionBackend:
    """No-op backend used when contacts are disabled or guard-only."""

    display = None
    contact_count = 0

    def advance(self, current_time: float, *, playing: bool, render_period: float) -> None:
        return

    def render(self) -> None:
        return

    def close(self) -> None:
        return


class NewtonCollisionBackend:
    """Newton contact backend running through the isolated worker process."""

    def __init__(
        self,
        stage,
        roots: list[str],
        *,
        python: Path,
        device: str,
        initial_time: float,
    ) -> None:
        from .newton_contacts import NewtonContactProcess

        self._world = NewtonContactProcess(
            stage, roots, python=python.expanduser(), device=device
        )
        self._time = float(initial_time)
        self.display = self._world.display

    @property
    def contact_count(self) -> int:
        return int(self._world.contact_count)

    def advance(self, current_time: float, *, playing: bool, render_period: float) -> None:
        if playing and current_time > self._time:
            self._world.advance(min(current_time - self._time, render_period))
        self._time = current_time

    def render(self) -> None:
        self._world.render()

    def close(self) -> None:
        self._world.close()


def make_collision_backend(
    kind: str,
    *,
    stage,
    physics_links: dict[str, Any],
    has_environment: bool,
    newton_python: Path | None,
    newton_device: str,
    initial_time: float,
):
    """Create the selected backend.

    ``kinematic_guard`` is handled before this layer by installing motion
    guards on kinematic models; it intentionally has no live contact backend.
    """

    if kind in {"none", "kinematic_guard"}:
        return NoCollisionBackend()
    if kind != "newton":
        raise ValueError(f"unsupported collision backend: {kind}")
    roots = [f"/World/{name}/PhysicsLinks" for name in physics_links]
    if has_environment:
        roots.append("/World/Environment")
    if not roots:
        return NoCollisionBackend()
    if newton_python is None:
        raise ValueError("newton_python is required when collision_backend is newton")
    backend = NewtonCollisionBackend(
        stage,
        roots,
        python=newton_python,
        device=newton_device,
        initial_time=initial_time,
    )
    print(
        "Newton collision backend: red colliders turn green on contact; "
        "instrument and jaws remain kinematic",
        flush=True,
    )
    backend.render()
    return backend
