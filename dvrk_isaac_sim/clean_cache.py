"""Remove Isaac Sim generated assets while retaining the interpreter selection."""

import argparse
from pathlib import Path
import shutil

from .python_runtime import default_generated_root


def main(arguments=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--config", type=Path, help="runtime YAML containing generated_root")
    selection.add_argument("--generated-root", type=Path, help="explicit generated asset directory")
    options = parser.parse_args(arguments)
    if options.config:
        from .configuration import load_simulator_config
        cache = load_simulator_config(options.config).generated_root or default_generated_root()
    else:
        cache = options.generated_root or default_generated_root()
    cache = cache.expanduser().absolute()
    if cache.is_symlink():
        parser.error(f"refusing to remove assets through symlink: {cache}")
    if not cache.exists():
        print(f"Cache does not exist: {cache}")
        return 0
    # A configured root may contain unrelated files. Delete only recognized
    # converter outputs, never the root itself or its interpreter cache.
    for asset in sorted(cache.iterdir()):
        if asset.is_symlink() or not asset.is_dir():
            continue
        robot_asset = (asset / "kinematics.json").is_file() and any(asset.rglob("*.usd*"))
        object_assets = asset.name == "objects" and any(asset.rglob("*.usd*"))
        if robot_asset or object_assets:
            shutil.rmtree(asset)
            print(f"Removed generated Isaac Sim assets: {asset}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
