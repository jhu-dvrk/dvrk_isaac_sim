#!/usr/bin/env python3
"""Validate simulator config and all scene YAML files without Isaac Sim."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

# Permit running the source script directly.
_package_root = Path(__file__).resolve().parents[1]
if str(_package_root) not in sys.path:
    sys.path.insert(0, str(_package_root))

from dvrk_isaac_sim.configuration import load_installed_scene_config, load_simulator_config



def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=root / "share" / "isaac_sim.yaml.example")
    args = parser.parse_args()
    config_path = args.config.expanduser().resolve()
    simulator = load_simulator_config(config_path)
    scenes = sorted((root / "share" / "scenes").glob("*.yaml"))
    for scene_path in scenes:
        scene = load_installed_scene_config(scene_path)
        print(f"OK {scene_path.name}: {len(scene.robots)} robots, camera={scene.camera.mode}")
    print(f"OK {config_path.name}: renderer={simulator.renderer}, scenes={len(scenes)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
