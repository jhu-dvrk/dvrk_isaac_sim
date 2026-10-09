#!/usr/bin/env python3
"""Convert a dVRK robot to a cached Isaac USD asset."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dvrk_isaac_sim.assets import main

if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"Conversion failed: {error}", file=sys.stderr)
        raise SystemExit(1)
