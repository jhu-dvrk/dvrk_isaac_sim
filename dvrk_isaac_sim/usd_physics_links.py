"""Flattened, non-nested kinematic rigid bodies for instrument collision."""

from __future__ import annotations

import json
import re
from pathlib import Path

import numpy as np

from .rotations import rotation_to_quaternion_xyzw
from .transforms import transform_from_rpy as _transform_from_rpy


def _usd_matrix_to_numpy_column_major(matrix) -> np.ndarray:
    return np.array(matrix, dtype=float).T


def _relative_offset_from_stage_transforms(world_link, world_candidate) -> np.ndarray:
    link_column_major = _usd_matrix_to_numpy_column_major(world_link)
    candidate_column_major = _usd_matrix_to_numpy_column_major(world_candidate)
    return np.linalg.inv(link_column_major) @ candidate_column_major


def _collision_name_hints(item: dict) -> tuple[str, ...]:
    hints: list[str] = []
    item_name = str(item.get("name", "")).strip().lower()
    if item_name:
        hints.append(item_name)
    geometry = item.get("geometry")
    if isinstance(geometry, dict):
        filename = str(geometry.get("filename", "")).strip().lower()
        if filename:
            hints.append(Path(filename).stem)
    return tuple(dict.fromkeys(hint for hint in hints if hint))


def _collision_body_name(item: dict, index: int) -> str:
    link = str(item.get("source_link", "")).strip()
    name = str(item.get("name", "")).strip()
    base = name or f"{link}_collision_{index}"
    base = re.sub(r"[^A-Za-z0-9_]+", "_", base).strip("_")
    return base or f"collision_{index}"


def _matches_collision_hint(prim, hints: tuple[str, ...]) -> bool:
    if not hints:
        return False
    name = prim.GetName().lower()
    path = str(prim.GetPath()).lower()
    return any(hint in name or hint in path for hint in hints)


def _has_collision_identity(prim) -> bool:
    name = prim.GetName().lower()
    path = str(prim.GetPath()).lower()
    if "collision" in name or "collision" in path:
        return True
    purpose = prim.GetAttribute("purpose")
    if purpose.IsValid() and purpose.HasAuthoredValueOpinion() and str(purpose.Get()).lower() == "guide":
        return True
    approximation = prim.GetAttribute("physics:approximation")
    return approximation.IsValid() and approximation.HasAuthoredValueOpinion()


def _apply_collision_debug_color(prim, UsdGeom) -> None:
    if not prim.IsA(UsdGeom.Gprim):
        return
    gprim = UsdGeom.Gprim(prim)
    gprim.CreateDisplayColorAttr().Set([(1.0, 0.0, 0.0)])
    gprim.CreateDisplayOpacityAttr().Set([1.0])


def _primitive_geometry(item: dict) -> dict | None:
    geometry = item.get("geometry")
    if not isinstance(geometry, dict):
        return None
    if geometry.get("type") in {"box", "capsule", "cylinder", "sphere"}:
        return geometry
    return None


def _candidate_collision_prim(prim, Usd, UsdGeom, item: dict, blocked_paths: tuple[str, ...] = ()):
    hints = _collision_name_hints(item)
    source_path = str(prim.GetPath())
    named = []
    generic = []
    for child in Usd.PrimRange(prim):
        child_path = str(child.GetPath())
        if any(child_path == blocked or child_path.startswith(blocked + "/") for blocked in blocked_paths):
            continue
        if not child.IsActive():
            continue
        depth = child_path.count("/") - source_path.count("/")
        hint_matched = _matches_collision_hint(child, hints)
        collision_like = _has_collision_identity(child)
        if collision_like:
            generic.append((depth, child_path, child))
            if hint_matched:
                named.append((depth, child_path, child))
                continue
        if child.IsA(UsdGeom.Gprim) and not collision_like:
            generic.append((depth, child_path, child))
            if hint_matched:
                named.append((depth, child_path, child))
    if len(named) > 1:
        named.sort(key=lambda entry: (entry[0], entry[1]))
        print(
            f"[usd_physics_links] ambiguous collision match for {str(item.get('name', ''))!r}: "
            f"{[path for _, path, _ in named]} - using {named[0][2].GetPath()}"
        )
    if named:
        named.sort(key=lambda entry: (entry[0], entry[1]))
        return named[0][2]
    if generic:
        generic.sort(key=lambda entry: (entry[0], entry[1]))
        return generic[0][2]
    return None


class MissingCollisionCandidate(RuntimeError):
    """Raised when an imported USD asset lacks an expected collision prim."""


def _require_collision_candidate(candidate, item: dict, source_path: str):
    if candidate is not None:
        return candidate
    link_name = str(item.get("source_link", "")).strip() or str(item.get("name", "")).strip()
    raise MissingCollisionCandidate(
        f"No collision prim matched link {link_name!r} below {source_path}; "
        "cannot create flattened collision body"
    )


class PhysicsLinkSync:
    """Synchronize flattened kinematic collision rigid bodies from URDF FK."""

    def __init__(self, component_name: str, manifest_path: str | Path, kinematic_chain):
        import omni.usd
        from pxr import UsdGeom, UsdPhysics

        self._component_name = component_name
        self._stage = omni.usd.get_context().get_stage()
        if self._stage is None:
            raise RuntimeError("Isaac Sim stage is not available")
        self._UsdGeom = UsdGeom
        self._UsdPhysics = UsdPhysics
        self._chain = kinematic_chain
        self._collision_ops: list[tuple[str, str, np.ndarray, object, object]] = []
        self._link_source_paths: dict[str, str] = {}
        self._visual_root = "Geometry/world"

        manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
        collision = manifest.get("collision") if isinstance(manifest, dict) else None
        if not isinstance(collision, dict) or not isinstance(collision.get("items"), list):
            raise RuntimeError(f"{manifest_path}: manifest has no collision.items data")
        if isinstance(collision.get("root"), str):
            self._visual_root = str(collision["root"])
        items = [item for item in collision["items"] if isinstance(item, dict)]
        if not items:
            return

        root_path = f"/World/{component_name}/PhysicsLinks"
        UsdGeom.Xform.Define(self._stage, root_path)

        for item in items:
            link = str(item.get("source_link", "")).strip()
            if not link or link in self._link_source_paths:
                continue
            source_relative = str(item.get("prim", "")).strip("/")
            if source_relative:
                self._link_source_paths[link] = f"/World/{component_name}/{self._visual_root}/{source_relative}"
            else:
                self._link_source_paths[link] = f"/World/{component_name}/{self._visual_root}"

        used_names: set[str] = set()
        for index, item in enumerate(items):
            link = str(item.get("source_link", "")).strip()
            if not link:
                continue
            body_name = _collision_body_name(item, index)
            if body_name in used_names:
                body_name = f"{body_name}_{index}"
            used_names.add(body_name)
            prim_path = f"{root_path}/{body_name}"
            try:
                translate_op, orient_op, offset = self._create_flattened_link(
                    prim_path, item, UsdGeom, UsdPhysics
                )
            except MissingCollisionCandidate as exc:
                print(f"[usd_physics_links] warning: {exc}", flush=True)
                continue
            self._collision_ops.append((link, prim_path, offset, translate_op, orient_op))

    def _create_flattened_link(self, prim_path, item, UsdGeom, UsdPhysics):
        from pxr import Usd

        link_xform = UsdGeom.Xform.Define(self._stage, prim_path)
        link_prim = link_xform.GetPrim()
        link_prim.SetCustomDataByKey("dvrk:source_link", str(item.get("source_link", "")))

        rigid = UsdPhysics.RigidBodyAPI.Apply(link_prim)
        rigid.CreateKinematicEnabledAttr().Set(True)

        xformable = UsdGeom.Xformable(link_prim)
        translate_op = xformable.AddTranslateOp(
            precision=UsdGeom.XformOp.PrecisionDouble, opSuffix="physics"
        )
        orient_op = xformable.AddOrientOp(
            precision=UsdGeom.XformOp.PrecisionDouble, opSuffix="physics"
        )

        source_relative = str(item.get("prim", "")).strip("/")
        if source_relative:
            source_path = f"/World/{self._component_name}/{self._visual_root}/{source_relative}"
        else:
            source_path = f"/World/{self._component_name}/{self._visual_root}"
        source_prim = self._stage.GetPrimAtPath(source_path)

        primitive = _primitive_geometry(item)
        geometry = item.get("geometry", {})
        if primitive is not None or geometry.get("type") == "convex_mesh":
            offset = _transform_from_rpy(
                item.get("origin_xyz", [0.0, 0.0, 0.0]),
                item.get("origin_rpy", [0.0, 0.0, 0.0]),
            )
            if primitive is not None:
                self._create_primitive_collision(f"{prim_path}/Collision", primitive, UsdGeom, UsdPhysics)
            else:
                self._create_convex_collision(f"{prim_path}/Collision", geometry, UsdGeom, UsdPhysics)
            return translate_op, orient_op, offset

        candidate = None
        if source_prim.IsValid():
            blocked_paths = tuple(
                other_source_path
                for other_link, other_source_path in self._link_source_paths.items()
                if other_link != str(item.get("source_link", "")).strip()
                and other_source_path.startswith(source_path + "/")
            )
            candidate = _candidate_collision_prim(
                source_prim,
                Usd,
                UsdGeom,
                item,
                blocked_paths,
            )
        candidate = _require_collision_candidate(candidate, item, source_path)

        offset = _transform_from_rpy(
            item.get("origin_xyz", [0.0, 0.0, 0.0]),
            item.get("origin_rpy", [0.0, 0.0, 0.0]),
        )

        world_link = UsdGeom.Xformable(source_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        candidate_parent = candidate.GetParent()
        if candidate_parent.IsValid():
            world_target_parent = UsdGeom.Xformable(candidate_parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        else:
            world_target_parent = world_link
        offset = _relative_offset_from_stage_transforms(world_link, world_target_parent)

        collision_prim_path = f"{prim_path}/Collision"
        collision_prim = self._stage.DefinePrim(collision_prim_path, candidate.GetTypeName() or "Xform")
        collision_prim.GetReferences().AddInternalReference(str(candidate.GetPath()))
        if collision_prim.IsA(UsdGeom.Gprim) and not collision_prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(collision_prim)
        geometry_targets = [child for child in Usd.PrimRange(collision_prim) if child.IsA(UsdGeom.Gprim)]
        for geometry in geometry_targets:
            if not geometry.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(geometry)
            _apply_collision_debug_color(geometry, UsdGeom)

        return translate_op, orient_op, offset

    def _create_convex_collision(self, prim_path, geometry, UsdGeom, UsdPhysics):
        from .collision_decomposition import validate_hull

        points, faces = validate_hull(geometry["points"], geometry["faces"], 128)
        collision = UsdGeom.Mesh.Define(self._stage, prim_path)
        collision.CreatePointsAttr().Set(points.tolist())
        collision.CreateFaceVertexCountsAttr().Set([3] * len(faces))
        collision.CreateFaceVertexIndicesAttr().Set(faces.reshape(-1).tolist())
        collision.CreateSubdivisionSchemeAttr().Set("none")
        UsdPhysics.CollisionAPI.Apply(collision.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(collision.GetPrim()).CreateApproximationAttr().Set("convexHull")
        _apply_collision_debug_color(collision.GetPrim(), UsdGeom)

    def _create_primitive_collision(self, prim_path: str, geometry: dict, UsdGeom, UsdPhysics) -> None:
        geometry_type = geometry.get("type")
        if geometry_type == "capsule":
            collision = UsdGeom.Capsule.Define(self._stage, prim_path)
            collision.CreateRadiusAttr().Set(float(geometry.get("radius", 0.0)))
            collision.CreateHeightAttr().Set(float(geometry.get("length", 0.0)))
        elif geometry_type == "cylinder":
            collision = UsdGeom.Cylinder.Define(self._stage, prim_path)
            collision.CreateRadiusAttr().Set(float(geometry.get("radius", 0.0)))
            collision.CreateHeightAttr().Set(float(geometry.get("length", 0.0)))
        elif geometry_type == "sphere":
            collision = UsdGeom.Sphere.Define(self._stage, prim_path)
            collision.CreateRadiusAttr().Set(float(geometry.get("radius", 0.0)))
        elif geometry_type == "box":
            collision = UsdGeom.Cube.Define(self._stage, prim_path)
            collision.CreateSizeAttr().Set(1.0)
            size = geometry.get("size", [1.0, 1.0, 1.0])
            scale_op = UsdGeom.Xformable(collision.GetPrim()).AddScaleOp(
                precision=UsdGeom.XformOp.PrecisionDouble, opSuffix="collision"
            )
            from pxr import Gf

            scale_op.Set(Gf.Vec3d(float(size[0]), float(size[1]), float(size[2])))
        else:
            raise RuntimeError(f"Unsupported primitive collision geometry: {geometry_type}")

        collision_prim = collision.GetPrim()
        if not collision_prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(collision_prim)
        _apply_collision_debug_color(collision_prim, UsdGeom)

    def update(self, joint_names, joint_position, jaw_position=None):
        q = np.asarray(joint_position, dtype=float)
        names = tuple(joint_names)
        if jaw_position is not None:
            names = names + ("jaw",)
            q = np.concatenate([q, np.asarray([float(jaw_position)], dtype=float)])
        poses = self._chain.forward_all_links(q, names)
        for link, _, local_offset, translate_op, orient_op in self._collision_ops:
            if link not in poses:
                continue
            world = poses[link] @ local_offset
            self._set_kinematic_target(translate_op, orient_op, world)

    def _set_kinematic_target(self, translate_op, orient_op, world_4x4):
        from pxr import Gf

        translation = world_4x4[:3, 3]
        rotation = world_4x4[:3, :3]
        x, y, z, w = rotation_to_quaternion_xyzw(rotation)
        translate_op.Set(Gf.Vec3d(float(translation[0]), float(translation[1]), float(translation[2])))
        orient_op.Set(Gf.Quatd(float(w), Gf.Vec3d(float(x), float(y), float(z))))
