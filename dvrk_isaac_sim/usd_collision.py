"""Apply collision APIs to imported URDF collision meshes in kinematic mode."""

from __future__ import annotations

import json
from pathlib import Path


_PSM_COLLISION_APPROXIMATIONS = {
    "insertion_link": "convexDecomposition",
    "roll_link": "convexDecomposition",
    "wrist_yaw_link": "convexDecomposition",
    "wrist_pitch_link": "sdf",
    "jaw_1_link": "sdf",
    "jaw_2_link": "sdf",
}


def _collision_approximation(prim) -> str | None:
    """Return the collision approximation tier for a PSM link's geometry."""
    current = prim
    while current.IsValid():
        name = current.GetName().lower()
        for link_name, approximation in _PSM_COLLISION_APPROXIMATIONS.items():
            if name == link_name or name.endswith(f"_{link_name}"):
                return approximation
        current = current.GetParent()
    return None


def _visual_root(manifest_path: str | Path | None) -> str:
    if manifest_path is None:
        return "Geometry/world"
    try:
        manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return "Geometry/world"
    collision = manifest.get("collision")
    if isinstance(collision, dict) and isinstance(collision.get("root"), str):
        return str(collision["root"])
    visual = manifest.get("visual")
    if isinstance(visual, dict) and isinstance(visual.get("root"), str):
        return str(visual["root"])
    return "Geometry/world"


def _is_collision_candidate(prim) -> bool:
    name = prim.GetName().lower()
    if name.endswith("_collision"):
        return True
    purpose = prim.GetAttribute("purpose")
    if purpose.IsValid() and purpose.HasAuthoredValueOpinion():
        if str(purpose.Get()).lower() == "guide":
            return True
    approximation = prim.GetAttribute("physics:approximation")
    return approximation.IsValid() and approximation.HasAuthoredValueOpinion()


def _apply_collision_debug_color(prim, UsdGeom) -> None:
    if UsdGeom is None:
        return
    gprim = UsdGeom.Gprim(prim)
    gprim.CreateDisplayColorAttr().Set([(1.0, 0.0, 0.0)])
    gprim.CreateDisplayOpacityAttr().Set([1.0])


def _is_mesh_geometry(prim, UsdGeom) -> bool:
    if UsdGeom is None or not hasattr(UsdGeom, "Mesh"):
        return True
    return prim.IsA(UsdGeom.Mesh)


def _apply_collision_api(prim, UsdPhysics, UsdGeom=None) -> bool:
    applied = False
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(prim)
        applied = True

    _apply_collision_debug_color(prim, UsdGeom)
    if _is_mesh_geometry(prim, UsdGeom):
        mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(prim)
        approximation = _collision_approximation(prim)
        if approximation is not None:
            mesh_collision.CreateApproximationAttr().Set(approximation)
    return applied


def _collision_targets(prim, Usd, UsdGeom):
    targets = [item for item in Usd.PrimRange(prim) if item.IsA(UsdGeom.Gprim)]
    return targets or [prim]


def apply_collision_meshes(component_name: str, manifest_path: str | Path | None = None) -> int:
    """Apply collision APIs to guide/collision prims authored by the URDF import.

    The robot remains kinematic: this only restores collision participation for
    the existing moving link hierarchy.
    """
    import omni.usd
    from pxr import Usd, UsdGeom, UsdPhysics

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Isaac Sim stage is not available")

    visual_root = _visual_root(manifest_path)
    root_path = f"/World/{component_name}/{visual_root}"
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        raise RuntimeError(f"USD collision root not found: {root_path}")

    applied = 0
    for prim in Usd.PrimRange(root_prim):
        if not prim.IsActive() or not _is_collision_candidate(prim):
            continue

        for target in _collision_targets(prim, Usd, UsdGeom):
            applied += int(_apply_collision_api(target, UsdPhysics, UsdGeom))
    return applied
