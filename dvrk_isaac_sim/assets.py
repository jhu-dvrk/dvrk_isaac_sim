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

from dvrk_isaac_sim.python_runtime import default_generated_root
from dvrk_isaac_sim.urdf_kinematics import write_kinematics_manifest


def _default_output() -> Path:
    return default_generated_root()


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


def _expand_xacro(model, instrument, endoscope, xacro, parent_link, xacro_arg, xacro_path, output) -> None:
    command = [xacro, str(xacro_path)]
    if model.startswith("PSM"):
        command.append(f"instrument:={instrument}")
    else:
        command.append(f"endoscope:={endoscope}")
    command.append(f"parent_link_:={parent_link}")
    command.extend(xacro_arg)
    try:
        with output.open("w", encoding="utf-8") as stream:
            subprocess.run(command, check=True, stdout=stream)
    except FileNotFoundError as exc:
        raise RuntimeError(f"Xacro executable not found: {xacro}") from exc
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(f"Xacro expansion failed with exit code {exc.returncode}") from exc


def _strip_physics(usd_path: str) -> None:
    """Remove authored Physics schemas from a visual-only kinematic asset."""
    from pxr import Sdf

    removed = 0
    asset_root = Path(usd_path).parent
    for layer_path in asset_root.rglob("*.usd*"):
        if layer_path.suffix not in {".usd", ".usda", ".usdc"}:
            continue
        # .usd can contain either ASCII or binary data. Let USD read and save
        # the layer rather than assuming all importer output is UTF-8 text.
        layer = Sdf.Layer.FindOrOpen(str(layer_path))
        if layer is None:
            raise RuntimeError(f"could not open USD layer {layer_path}")
        source = layer.ExportToString()
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
            layer.ImportFromString("".join(filtered))
            layer.Save()
    print(f"Removed {removed} Physics schemas for kinematic mode", flush=True)


def convert_model(*, model, output_dir, instrument="420006", endoscope="Si_straight",
                  asset_name=None, xacro="xacro", xacro_file=None, parent_link="world",
                  xacro_arg=(), merge_fixed_joints=True, fix_base=True,
                  collision_from_visuals=False, force=False, keep_physics=False):
    """Convert one robot using the current Isaac application."""
    asset_name = asset_name or (
        f"{model}_{instrument}" if model.startswith("PSM")
        else f"{model}_{endoscope}"
    )

    root = _model_root()
    xacro_path = (xacro_file or root / "urdf" / "Virtual" / f"{model}.urdf.xacro").expanduser().resolve()
    if not xacro_path.is_file():
        raise RuntimeError(f"Xacro file not found: {xacro_path}")
    output_dir = Path(output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    asset_dir = output_dir / asset_name
    if asset_dir.exists():
        if not force:
            raise RuntimeError(f"Generated asset already exists; use --force to replace it: {asset_dir}")
        shutil.rmtree(asset_dir)

    try:
        with tempfile.TemporaryDirectory(prefix="dvrk_isaac_sim_") as temporary:
            urdf_path = Path(temporary) / f"{model}.urdf"
            _expand_xacro(model, instrument, endoscope, xacro, parent_link, xacro_arg, xacro_path, urdf_path)
            manifest_path = write_kinematics_manifest(urdf_path, asset_dir / "kinematics.json", model)
            print(f"Generated kinematics manifest {manifest_path}", flush=True)
            from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig

            config = URDFImporterConfig()
            config.urdf_path = str(urdf_path)
            config.usd_path = str(asset_dir)
            config.merge_fixed_joints = merge_fixed_joints
            config.fix_base = fix_base
            config.collision_from_visuals = collision_from_visuals
            config.merge_mesh = True
            config.ros_package_paths = [{"name": "dvrk_model", "path": str(root)}]
            output_usd = URDFImporter(config).import_urdf()
            if not output_usd:
                raise RuntimeError("Isaac Sim URDF importer returned no USD path")
            if not keep_physics:
                _strip_physics(output_usd)
            print(f"Generated {output_usd}", flush=True)
    except BaseException:
        # A partial asset plus manifest must not look like a valid cache entry.
        shutil.rmtree(asset_dir, ignore_errors=True)
        raise
    return Path(output_usd)


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

    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": True})
    try:
        convert_model(**vars(args))
    finally:
        simulation_app.close()

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"Conversion failed: {exc}", file=sys.stderr)
        sys.exit(1)
