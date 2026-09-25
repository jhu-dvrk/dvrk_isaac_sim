"""Dependency-free URDF kinematics manifest parsing."""

from __future__ import annotations

import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def _numbers(value: str | None, count: int) -> list[float]:
    values = [float(item) for item in (value or "").split()]
    if len(values) != count:
        return [0.0] * count
    return values


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _transform(rotation: np.ndarray, translation: list[float]) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def _rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c + x*x*(1-c), x*y*(1-c)-z*s, x*z*(1-c)+y*s],
                     [y*x*(1-c)+z*s, c + y*y*(1-c), y*z*(1-c)-x*s],
                     [z*x*(1-c)-y*s, z*y*(1-c)+x*s, c + z*z*(1-c)]])


def _joint_dict(element: ET.Element) -> dict:
    origin = element.find("origin")
    axis = element.find("axis")
    limit = element.find("limit")
    mimic = element.find("mimic")
    return {
        "name": element.attrib["name"],
        "type": element.attrib.get("type", "fixed"),
        "parent": element.find("parent").attrib.get("link") if element.find("parent") is not None else "",
        "child": element.find("child").attrib.get("link") if element.find("child") is not None else "",
        "origin_xyz": _numbers(origin.attrib.get("xyz") if origin is not None else None, 3),
        "origin_rpy": _numbers(origin.attrib.get("rpy") if origin is not None else None, 3),
        "axis": _numbers(axis.attrib.get("xyz") if axis is not None else None, 3),
        "lower": float(limit.attrib["lower"]) if limit is not None and "lower" in limit.attrib else None,
        "upper": float(limit.attrib["upper"]) if limit is not None and "upper" in limit.attrib else None,
        "velocity": float(limit.attrib["velocity"]) if limit is not None and "velocity" in limit.attrib else None,
        "mimic": ({
            "joint": mimic.attrib.get("joint", ""),
            "multiplier": float(mimic.attrib.get("multiplier", "1.0")),
            "offset": float(mimic.attrib.get("offset", "0.0")),
        } if mimic is not None else None),
    }


def _geometry_dict(collision: ET.Element) -> dict:
    geometry = collision.find("geometry")
    if geometry is None:
        return {"type": "unknown"}

    mesh = geometry.find("mesh")
    if mesh is not None:
        return {
            "type": "mesh",
            "filename": mesh.attrib.get("filename", ""),
            "scale": _numbers(mesh.attrib.get("scale"), 3),
        }

    box = geometry.find("box")
    if box is not None:
        return {
            "type": "box",
            "size": _numbers(box.attrib.get("size"), 3),
        }

    sphere = geometry.find("sphere")
    if sphere is not None:
        radius = sphere.attrib.get("radius")
        return {
            "type": "sphere",
            "radius": float(radius) if radius is not None else 0.0,
        }

    cylinder = geometry.find("cylinder")
    if cylinder is not None:
        radius = cylinder.attrib.get("radius")
        length = cylinder.attrib.get("length")
        return {
            "type": "cylinder",
            "radius": float(radius) if radius is not None else 0.0,
            "length": float(length) if length is not None else 0.0,
        }

    return {"type": "unknown"}


def _relative_visual_path(link_path: str, visual_root: str) -> str:
    if link_path == visual_root:
        return ""
    if link_path.startswith(visual_root + "/"):
        return link_path[len(visual_root) + 1:]
    return link_path


def _collision_items(root: ET.Element, link_paths: dict[str, str], visual_root: str) -> list[dict]:
    result = []
    for link in root.findall("link"):
        link_name = link.attrib.get("name", "")
        if not link_name or link_name not in link_paths:
            continue
        relative_prim = _relative_visual_path(link_paths[link_name], visual_root)
        collisions = list(link.findall("collision"))
        for index, collision in enumerate(collisions):
            origin = collision.find("origin")
            result.append({
                "name": collision.attrib.get("name", f"{link_name}_collision_{index}"),
                "source_link": link_name,
                "prim": relative_prim,
                "origin_xyz": _numbers(origin.attrib.get("xyz") if origin is not None else None, 3),
                "origin_rpy": _numbers(origin.attrib.get("rpy") if origin is not None else None, 3),
                "geometry": _geometry_dict(collision),
            })
    return result


def write_kinematics_manifest(urdf_path: str | Path, output_path: str | Path, model: str) -> Path:
    """Extract the selected root-to-tool chain from an expanded URDF."""
    root = ET.parse(urdf_path).getroot()
    joints = [_joint_dict(element) for element in root.findall("joint")]
    by_child = {joint["child"]: joint for joint in joints}
    prefix = f"{model}_"
    candidates = ([f"{prefix}tool_tip_link", f"{prefix}tip_link", f"{prefix}wrist_yaw_link", f"{prefix}adaptor_link"]
                  if model.startswith("PSM") else
                  [f"{prefix}tip_link", f"{prefix}endoscope_frame_link", f"{prefix}adaptor_link"] )
    links = {joint["child"] for joint in joints} | {joint["parent"] for joint in joints}
    tip = next((candidate for candidate in candidates if candidate in links), None)
    if tip is None:
        raise ValueError(f"could not find a tool tip link for {model} in {urdf_path}")

    chain_reversed = []
    current = tip
    while current in by_child:
        joint = by_child[current]
        chain_reversed.append(joint)
        current = joint["parent"]
    chain = list(reversed(chain_reversed))
    active = [joint["name"] for joint in chain if joint["type"] in ("revolute", "continuous", "prismatic") and joint["mimic"] is None]

    # The USD importer omits fixed-link nodes when composing the visual
    # hierarchy. Build paths from the URDF chain and retain the visual mapping
    # in the manifest so the Isaac backend does not need robot-specific paths.
    link_paths = {current: "Geometry/world"}
    pending = list(joints)
    while pending:
        next_pending = []
        progressed = False
        for joint in pending:
            if joint["parent"] not in link_paths:
                next_pending.append(joint)
                continue
            parent_path = link_paths[joint["parent"]]
            link_paths[joint["child"]] = (
                parent_path if joint["type"] == "fixed"
                else f"{parent_path}/{joint['child']}"
            )
            progressed = True
        if not progressed:
            break
        pending = next_pending

    visual_root = "Geometry/world"
    visual_joints = {}
    for joint in joints:
        if joint["type"] not in ("revolute", "continuous", "prismatic"):
            continue
        axis = np.asarray(joint["axis"], dtype=float)
        if np.linalg.norm(axis) == 0.0 or joint["child"] not in link_paths:
            continue
        axis_index = int(np.argmax(np.abs(axis)))
        if np.count_nonzero(np.abs(axis) > 1e-8) != 1:
            continue
        visual_path = link_paths[joint["child"]]
        relative_visual_path = _relative_visual_path(visual_path, visual_root)
        visual_joints[joint["name"]] = {
            "prim": relative_visual_path,
            "operation": "translate" if joint["type"] == "prismatic" else "rotate",
            "axis": "XYZ"[axis_index],
            "scale": float(axis[axis_index]),
            "mimic": joint["mimic"],
            "source_link": joint["child"],
            # URDF applies joint motion after the joint origin but before a
            # child link's visual offset.  Isaac's importer can fold that
            # visual offset into the child Xform; in that case the motion op
            # must be first in the USD stack to preserve the joint pivot.
            "motion_before_static_transform": bool(
                np.allclose(joint["origin_xyz"], 0.0)
                and np.allclose(joint["origin_rpy"], 0.0)
            ),
        }

    collision_items = _collision_items(root, link_paths, visual_root)

    manifest = {
        "format": 5,
        "model": model,
        "tip_link": tip,
        "root_link": current,
        "joints": chain,
        "all_joints": joints,
        "active_joints": active,
        "visual": {"root": visual_root, "joints": visual_joints},
        "collision": {
            "root": visual_root,
            "items": collision_items,
        },
    }
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return output


class UrdfKinematicChain:
    """FK/Jacobian evaluator for an independent URDF joint chain."""

    def __init__(self, manifest_path: str | Path):
        self.manifest_path = Path(manifest_path)
        manifest = json.loads(self.manifest_path.read_text(encoding="utf-8"))
        self.joints = tuple(manifest["joints"])
        self.all_joints = tuple(manifest.get("all_joints", self.joints))
        self.active_joints = tuple(manifest["active_joints"])
        self.tip_link = str(manifest.get("tip_link", ""))
        self.root_link = str(manifest.get("root_link", ""))

    def forward(self, q: np.ndarray, joint_names: tuple[str, ...]) -> tuple[np.ndarray, np.ndarray]:
        if q.shape != (len(joint_names),):
            raise ValueError("joint position has the wrong size")
        values = dict(zip(joint_names, q))
        transform = np.eye(4)
        axes_world, origins_world, joint_types = [], [], []
        for joint in self.joints:
            transform = transform @ _transform(_rpy_matrix(*joint["origin_rpy"]), joint["origin_xyz"])
            joint_type = joint["type"]
            if joint_type in ("revolute", "continuous", "prismatic"):
                axis = np.asarray(joint["axis"], dtype=float)
                if np.linalg.norm(axis) == 0.0:
                    axis = np.array([0.0, 0.0, 1.0])
                mimic = joint["mimic"]
                if mimic is not None:
                    angle = values.get(mimic["joint"], 0.0) * mimic["multiplier"] + mimic["offset"]
                else:
                    angle = values.get(joint["name"], 0.0)
                    axes_world.append(transform[:3, :3] @ axis)
                    origins_world.append(transform[:3, 3].copy())
                    joint_types.append(joint_type)
                if joint_type in ("revolute", "continuous"):
                    transform = transform @ _transform(_rotation(axis, angle), [0.0, 0.0, 0.0])
                else:
                    transform = transform @ _transform(np.eye(3), (axis * angle).tolist())
        position = transform[:3, 3]
        jacobian = np.zeros((6, len(axes_world)))
        for index, (axis, origin, joint_type) in enumerate(zip(axes_world, origins_world, joint_types)):
            if joint_type in ("revolute", "continuous"):
                jacobian[:3, index] = np.cross(axis, position - origin)
                jacobian[3:, index] = axis
            else:
                jacobian[:3, index] = axis
        return transform, jacobian

    def forward_all_links(self, q: np.ndarray, joint_names: tuple[str, ...]) -> dict[str, np.ndarray]:
        """Return the world transform for each joint child link in the chain."""
        if q.shape != (len(joint_names),):
            raise ValueError("joint position has the wrong size")
        values = dict(zip(joint_names, q))
        transforms = {self.root_link: np.eye(4)}
        poses: dict[str, np.ndarray] = {}
        pending = list(self.all_joints)
        while pending:
            next_pending = []
            progressed = False
            for joint in pending:
                parent = joint["parent"]
                if parent not in transforms:
                    next_pending.append(joint)
                    continue
                transform = transforms[parent] @ _transform(_rpy_matrix(*joint["origin_rpy"]), joint["origin_xyz"])
                joint_type = joint["type"]
                if joint_type in ("revolute", "continuous", "prismatic"):
                    axis = np.asarray(joint["axis"], dtype=float)
                    if np.linalg.norm(axis) == 0.0:
                        axis = np.array([0.0, 0.0, 1.0])
                    mimic = joint["mimic"]
                    if mimic is not None:
                        angle = values.get(mimic["joint"], 0.0) * mimic["multiplier"] + mimic["offset"]
                    else:
                        angle = values.get(joint["name"], 0.0)
                    if joint_type in ("revolute", "continuous"):
                        transform = transform @ _transform(_rotation(axis, angle), [0.0, 0.0, 0.0])
                    else:
                        transform = transform @ _transform(np.eye(3), (axis * angle).tolist())
                poses[joint["child"]] = transform.copy()
                transforms[joint["child"]] = transform
                progressed = True
            if not progressed:
                break
            pending = next_pending
        return poses

    def forward_all_chain_links(self, q: np.ndarray, joint_names: tuple[str, ...]) -> dict[str, np.ndarray]:
        """Return the world transform for each joint child link in the tip chain."""
        if q.shape != (len(joint_names),):
            raise ValueError("joint position has the wrong size")
        values = dict(zip(joint_names, q))
        transform = np.eye(4)
        poses: dict[str, np.ndarray] = {}
        for joint in self.joints:
            transform = transform @ _transform(_rpy_matrix(*joint["origin_rpy"]), joint["origin_xyz"])
            joint_type = joint["type"]
            if joint_type in ("revolute", "continuous", "prismatic"):
                axis = np.asarray(joint["axis"], dtype=float)
                if np.linalg.norm(axis) == 0.0:
                    axis = np.array([0.0, 0.0, 1.0])
                mimic = joint["mimic"]
                if mimic is not None:
                    angle = values.get(mimic["joint"], 0.0) * mimic["multiplier"] + mimic["offset"]
                else:
                    angle = values.get(joint["name"], 0.0)
                if joint_type in ("revolute", "continuous"):
                    transform = transform @ _transform(_rotation(axis, angle), [0.0, 0.0, 0.0])
                else:
                    transform = transform @ _transform(np.eye(3), (axis * angle).tolist())
            poses[joint["child"]] = transform.copy()
        return poses
