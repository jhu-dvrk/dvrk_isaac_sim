#!/usr/bin/env python3
"""Convert a dvrk_model virtual PSM or ECM Xacro to a cached Isaac USD asset."""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from _isaac_sim_build import require_isaac_sim_build
from dvrk_isaac_sim.urdf_kinematics import write_kinematics_manifest

require_isaac_sim_build(__file__)

def _default_output() -> Path:
    cache_root = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache"))
    return (cache_root / "dvrk_isaac_sim").resolve()



DEFAULT_OUTPUT = _default_output()


def _model_root() -> Path:
    configured = os.environ.get("DVRK_MODEL_PATH")
    if configured:
        root = Path(configured).expanduser().resolve()
        if (root / "urdf").is_dir():
            return root
        raise RuntimeError(f"DVRK_MODEL_PATH does not contain an urdf directory: {root}")
    try:
        prefix = subprocess.check_output(
            ["ros2", "pkg", "prefix", "dvrk_model"], text=True, stderr=subprocess.STDOUT
        ).strip()
    except (OSError, subprocess.CalledProcessError) as exc:
        raise RuntimeError("dvrk_model was not found; source the workspace or set DVRK_MODEL_PATH") from exc
    prefix_path = Path(prefix)
    for candidate in (prefix_path / "share" / "dvrk_model", prefix_path):
        if (candidate / "urdf").is_dir():
            return candidate
    raise RuntimeError(f"Could not locate dvrk_model/urdf below ROS prefix: {prefix_path}")


def _expand_xacro(args: argparse.Namespace, xacro_path: Path, output: Path) -> None:
    command = [args.xacro, str(xacro_path)]
    if args.model.startswith("PSM"):
        command.append(f"instrument:={args.instrument}")
    else:
        command.append(f"endoscope:={args.endoscope}")
    command.append(f"parent_link_:={args.parent_link}")
    command.extend(args.xacro_arg)
    try:
        with output.open("w", encoding="utf-8") as stream:
            subprocess.run(command, check=True, stdout=stream)
    except FileNotFoundError as exc:
        raise RuntimeError(f"Xacro executable not found: {args.xacro}") from exc
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(f"Xacro expansion failed with exit code {exc.returncode}") from exc


def _strip_physics(usd_path: str) -> None:
    """Remove authored Physics schemas from a visual-only kinematic asset."""
    removed = 0
    asset_root = Path(usd_path).parent
    for layer_path in asset_root.rglob("*.usd*"):
        if layer_path.suffix not in {".usd", ".usda"}:
            continue
        source = layer_path.read_text(encoding="utf-8")
        original = source
        if layer_path == Path(usd_path):
            source = source.replace('string Physics = "physx"', 'string Physics = "none"')
        lines = source.splitlines(keepends=True)
        filtered = []
        for line in lines:
            if "apiSchemas" in line and re.search(r'"(?:Physics|Physx)', line):
                removed += 1
                continue
            filtered.append(line)
        if len(filtered) != len(lines) or source != original:
            layer_path.write_text("".join(filtered), encoding="utf-8")
    print(f"Removed {removed} Physics schemas for kinematic mode", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", choices=("PSM1", "PSM2", "PSM3", "ECM"), required=True)
    parser.add_argument("--xacro", default="xacro")
    parser.add_argument("--xacro-file", type=Path)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--asset-name", help="cache directory name; defaults to model plus variant")
    parser.add_argument("--instrument", default="420006")
    parser.add_argument("--endoscope", default="Si_straight")
    parser.add_argument("--parent-link", default="world")
    parser.add_argument("--xacro-arg", action="append", default=[])
    parser.add_argument("--merge-fixed-joints", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--fix-base", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--collision-from-visuals", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--force", action="store_true", help="replace an existing generated asset for this model")
    parser.add_argument(
        "--keep-physics",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="retain importer-authored PhysX schemas instead of producing a visual-only asset",
    )
    args = parser.parse_args()

    asset_name = args.asset_name or (
        f"{args.model}_{args.instrument}" if args.model.startswith("PSM")
        else f"{args.model}_{args.endoscope}"
    )

    root = _model_root()
    xacro_path = (args.xacro_file or root / "urdf" / "Virtual" / f"{args.model}.urdf.xacro").expanduser().resolve()
    if not xacro_path.is_file():
        raise RuntimeError(f"Xacro file not found: {xacro_path}")
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    asset_dir = output_dir / asset_name
    if asset_dir.exists():
        if not args.force:
            raise RuntimeError(f"Generated asset already exists; use --force to replace it: {asset_dir}")
        shutil.rmtree(asset_dir)

    with tempfile.TemporaryDirectory(prefix="dvrk_isaac_sim_") as temporary:
        urdf_path = Path(temporary) / f"{args.model}.urdf"
        _expand_xacro(args, xacro_path, urdf_path)
        manifest_path = write_kinematics_manifest(urdf_path, asset_dir / "kinematics.json", args.model)
        print(f"Generated kinematics manifest {manifest_path}", flush=True)
        from isaacsim import SimulationApp

        simulation_app = SimulationApp({"headless": True})
        try:
            from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig

            config = URDFImporterConfig()
            config.urdf_path = str(urdf_path)
            config.usd_path = str(asset_dir)
            config.merge_fixed_joints = args.merge_fixed_joints
            config.fix_base = args.fix_base
            config.collision_from_visuals = args.collision_from_visuals
            config.merge_mesh = True
            config.ros_package_paths = [{"name": "dvrk_model", "path": str(root)}]
            output_usd = URDFImporter(config).import_urdf()
            if not output_usd:
                raise RuntimeError("Isaac Sim URDF importer returned no USD path")
            if not args.keep_physics:
                _strip_physics(output_usd)
            print(f"Generated {output_usd}", flush=True)
        finally:
            simulation_app.close()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"Conversion failed: {exc}", file=sys.stderr)
        sys.exit(1)
