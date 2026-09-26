#!/usr/bin/env python3
"""Make 420006 instrument collisions uniformly convex-hull mesh references."""

from __future__ import annotations

import argparse
from pathlib import Path


RED_MTL = """# Collision debug material
newmtl collision_red
Ka 1.000000 0.000000 0.000000
Kd 1.000000 0.000000 0.000000
Ks 0.100000 0.000000 0.000000
Ke 0.000000 0.000000 0.000000
Ni 1.500000
d 1.000000
illum 2
"""


def _workspace_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if parent.name == "src":
            return parent.parent
    return Path("/home/yzhan874/dvrk_ws")


def _default_roots() -> tuple[Path, ...]:
    workspace = _workspace_root()
    return (
        workspace / "src" / "dvrk" / "dvrk_model",
        workspace / "install" / "dvrk_model" / "share" / "dvrk_model",
    )


def _copy_hull(source_obj: Path, destination_obj: Path, object_name: str) -> None:
    destination_obj.parent.mkdir(parents=True, exist_ok=True)
    source = source_obj.read_text(encoding="utf-8")
    lines = []
    for line in source.splitlines():
        if line.startswith("mtllib "):
            lines.append(f"mtllib {destination_obj.with_suffix('.mtl').name}")
        elif line.startswith("o "):
            lines.append(f"o {object_name}")
        else:
            lines.append(line)
    destination_obj.write_text("\n".join(lines) + "\n", encoding="utf-8")
    destination_obj.with_suffix(".mtl").write_text(RED_MTL, encoding="utf-8")


def _replace_once(path: Path, old: str, new: str) -> bool:
    source = path.read_text(encoding="utf-8")
    if old not in source:
        return False
    path.write_text(source.replace(old, new, 1), encoding="utf-8")
    return True


def _patch_roll(root: Path) -> None:
    mesh_root = root / "meshes" / "instruments" / "roll" / "4670"
    _copy_hull(
        mesh_root / "roll_4670_collision_convex_cylinder.obj",
        mesh_root / "collision_hulls" / "roll_4670_convex_hull_00.obj",
        "roll_4670_convex_hull_00",
    )
    xacro = root / "urdf" / "common" / "instruments" / "roll" / "roll_4670.urdf.xacro"
    old = (
        '<mesh filename="package://dvrk_model/meshes/instruments/roll/4670/'
        'roll_4670_collision_convex_cylinder.obj" scale="0.1 0.1 0.1"/>'
    )
    new = (
        '<mesh filename="package://dvrk_model/meshes/instruments/roll/4670/'
        'collision_hulls/roll_4670_convex_hull_00.obj" scale="0.1 0.1 0.1"/>'
    )
    if not _replace_once(xacro, old, new):
        raise RuntimeError(f"{xacro}: roll collision mesh reference was not found")


def _patch_wrist_yaw(root: Path) -> None:
    mesh_root = root / "meshes" / "instruments" / "wrist_yaw" / "0091"
    _copy_hull(
        mesh_root / "wrist_yaw_0091_collision_geometric.obj",
        mesh_root / "collision_hulls" / "wrist_yaw_0091_convex_hull_00.obj",
        "wrist_yaw_0091_convex_hull_00",
    )
    xacro = root / "urdf" / "common" / "instruments" / "wrist_yaw" / "wrist_yaw_0091.urdf.xacro"
    old = (
        "<collision>\n"
        '        <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 -0.000236"/>\n'
        "        <geometry>\n"
        '          <mesh filename="package://dvrk_model/meshes/instruments/wrist_yaw/0091/'
        'wrist_yaw_0091_collision_geometric.obj"/>\n'
        "        </geometry>"
    )
    new = (
        '<collision name="wrist_yaw_collision_00">\n'
        '        <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 -0.000236"/>\n'
        "        <geometry>\n"
        '          <mesh filename="package://dvrk_model/meshes/instruments/wrist_yaw/0091/'
        'collision_hulls/wrist_yaw_0091_convex_hull_00.obj"/>\n'
        "        </geometry>"
    )
    if not _replace_once(xacro, old, new):
        raise RuntimeError(f"{xacro}: wrist yaw collision mesh reference was not found")


def apply(root: Path) -> None:
    if not (root / "urdf").is_dir() or not (root / "meshes").is_dir():
        raise RuntimeError(f"{root}: expected dvrk_model root with urdf/ and meshes/")
    _patch_roll(root)
    _patch_wrist_yaw(root)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", action="append", type=Path, help="dvrk_model root to patch; repeatable")
    args = parser.parse_args()

    roots = tuple(args.root) if args.root else _default_roots()
    for root in roots:
        apply(root.expanduser().resolve())
        print(f"Updated uniform convex-hull collisions in {root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
