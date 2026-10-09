"""Exception types for the Isaac Sim simulator backend."""

from __future__ import annotations


class IsaacSimBackendError(RuntimeError):
    """Raised when an Isaac Sim call or scene setup fails."""
