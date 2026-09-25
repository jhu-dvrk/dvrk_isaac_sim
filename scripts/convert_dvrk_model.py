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
import xml.etree.ElementTree as ET

from _isaac_sim_build import require_isaac_sim_build
from dvrk_isaac_sim.urdf_kinematics import write_kinematics_manifest

require_isaac_sim_build(__file__)

def _default_output() -> Path:
    for parent in Path(__file__).resolve().parents:
        if parent.name == "src":
            return parent.parent / ".generated" / "isaacsim-6.0"
        if parent.name == "install":
            return parent.parent / ".generated" / "isaacsim-6.0"
    return Path.cwd() / ".generated" / "isaacsim-6.0"


DEFAULT_OUTPUT = _default_output()


def _strip_physics_api_schemas(line: str) -> str | None:
    """Drop importer physics APIs while retaining mesh collision configuration."""
    if "apiSchemas" not in line:
        return line
    match = re.search(r"\[([^]]*)\]", line)
    if match is None:
        return line
    schemas = re.findall(r'"([^"]+)"', match.group(1))
    removable = [schema for schema in schemas if schema.startswith(("Physics", "Physx"))]
    if not removable:
        return line
    retained = [schema for schema in schemas if schema not in removable or schema == "PhysicsMeshCollisionAPI"]
    if not retained:
        return None
    replacement = "[" + ", ".join(f'"{schema}"' for schema in retained) + "]"
    return line[:match.start()] + replacement + line[match.end():]


def _model_root() -> Path:
    configured = os.environ.get("DVRK_MODEL_PATH")
    if configured:
        root = Path(configured).expanduser().resolve()
        if (root / "urdf").is_dir():
            return root
        raise RuntimeError(f"DVRK_MODEL_PATH does not contain an urdf directory: {root}")
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "dvrk_model"
        if (candidate / "urdf").is_dir():
            return candidate
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


def _package_mesh_path(model_root: Path, filename: str) -> Path | None:
    prefix = "package://dvrk_model/"
    if not filename.startswith(prefix):
        return None
    return model_root / filename[len(prefix):]


def _package_mesh_name(model_root: Path, mesh_path: Path) -> str:
    return "package://dvrk_model/" + mesh_path.resolve().relative_to(model_root.resolve()).as_posix()


def _geometric_collision_mesh(model_root: Path, filename: str) -> str | None:
    mesh_path = _package_mesh_path(model_root, filename)
    if mesh_path is None:
        return None
    if "_collision_geometric" in mesh_path.stem:
        return filename if mesh_path.is_file() else None
    if "_collision" not in mesh_path.stem:
        return None

    geometric_stem = mesh_path.stem.replace("_collision", "_collision_geometric", 1)
    candidates = [
        mesh_path.with_name(geometric_stem + mesh_path.suffix),
        mesh_path.with_name(geometric_stem + ".obj"),
        mesh_path.with_name(geometric_stem + ".stl"),
        mesh_path.with_name(geometric_stem + ".STL"),
    ]
    for candidate in dict.fromkeys(candidates):
        if candidate.is_file():
            return _package_mesh_name(model_root, candidate)
    return None


def _origin(element: ET.Element) -> ET.Element:
    origin = element.find("origin")
    if origin is None:
        origin = ET.Element("origin")
        element.insert(0, origin)
    return origin


def _mesh(element: ET.Element) -> ET.Element | None:
    geometry = element.find("geometry")
    if geometry is None:
        return None
    return geometry.find("mesh")


def _first_visual_mesh(link: ET.Element) -> tuple[ET.Element, ET.Element] | None:
    for visual in link.findall("visual"):
        mesh = _mesh(visual)
        if mesh is not None and mesh.attrib.get("filename"):
            return visual, mesh
    return None


def _copy_visual_mesh_frame(
    visual: ET.Element,
    visual_mesh: ET.Element,
    collision: ET.Element,
    collision_mesh: ET.Element,
) -> None:
    visual_origin = visual.find("origin")
    collision_origin = _origin(collision)
    for attribute in ("xyz", "rpy"):
        if visual_origin is not None and attribute in visual_origin.attrib:
            collision_origin.set(attribute, visual_origin.attrib[attribute])
        elif attribute in collision_origin.attrib:
            del collision_origin.attrib[attribute]
    if "scale" in visual_mesh.attrib:
        collision_mesh.set("scale", visual_mesh.attrib["scale"])
    elif "scale" in collision_mesh.attrib:
        del collision_mesh.attrib["scale"]


def _normalize_geometric_collision_meshes(urdf_path: Path, model_root: Path) -> int:
    """Use geometric collision meshes in the same frame as their visual mesh."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    updated = 0
    for link in root.findall("link"):
        visual = _first_visual_mesh(link)
        if visual is None:
            continue
        visual_element, visual_mesh = visual
        for collision in link.findall("collision"):
            collision_mesh = _mesh(collision)
            if collision_mesh is None:
                continue
            filename = collision_mesh.attrib.get("filename", "")
            geometric = _geometric_collision_mesh(model_root, filename)
            if geometric is None:
                continue
            if geometric != filename:
                collision_mesh.set("filename", geometric)
                updated += 1
            before = (
                collision.find("origin").attrib.copy() if collision.find("origin") is not None else {},
                collision_mesh.attrib.copy(),
            )
            _copy_visual_mesh_frame(visual_element, visual_mesh, collision, collision_mesh)
            after = (
                collision.find("origin").attrib.copy() if collision.find("origin") is not None else {},
                collision_mesh.attrib.copy(),
            )
            updated += int(after != before)
    if updated:
        tree.write(urdf_path, encoding="utf-8", xml_declaration=True)
    return updated


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
            stripped_line = _strip_physics_api_schemas(line)
            if stripped_line is None:
                removed += 1
                continue
            filtered.append(stripped_line)
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
        normalized = _normalize_geometric_collision_meshes(urdf_path, root)
        if normalized:
            print(f"Normalized {normalized} geometric collision mesh references/frames", flush=True)
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
