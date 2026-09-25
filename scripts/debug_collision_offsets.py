#!/usr/bin/env python3
"""Print flattened collision/link offsets from the live Isaac Sim stage."""

from __future__ import annotations

import argparse
from pathlib import Path

from dvrk_isaac_sim.collision_diagnostics import report_collision_frames


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("component", help="Component name under /World, for example PSM1")
    parser.add_argument("manifest", type=Path, help="Kinematics manifest JSON used by PhysicsLinkSync")
    args = parser.parse_args()

    import omni.usd  # type: ignore[import-not-found]

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Isaac Sim stage is not available")

    report_collision_frames(stage, args.component, args.manifest)


if __name__ == "__main__":
    main()
