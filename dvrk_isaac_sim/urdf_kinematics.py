"""Dependency-free URDF kinematics manifest parsing."""

from __future__ import annotations

import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from dvrk_simulator_base.urdf_chain import UrdfChain
from .transforms import rotation_matrix as _rotation
from .transforms import rpy_matrix as _rpy_matrix
from .transforms import transform_matrix as _transform

# Kinematics manifest schema version; conversions older than this are stale.
MANIFEST_FORMAT = 5


def _numbers(value: str | None, count: int, default: float = 0.0) -> list[float]:
    values = [float(item) for item in (value or "").split()]
    if len(values) != count:
        return [default] * count
    return values


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
        "axis": _numbers(axis.attrib.get("xyz", "1 0 0") if axis is not None else "1 0 0", 3),
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
    for kind in ("mesh", "box", "sphere", "cylinder"):
        element = geometry.find(kind)
        if element is None:
            continue
        if kind == "mesh":
            return {
                "type": "mesh",
                "filename": element.attrib.get("filename", ""),
                "scale": _numbers(element.attrib.get("scale"), 3, default=1.0),
            }
        if kind == "box":
            return {"type": "box", "size": _numbers(element.attrib.get("size"), 3)}
        if kind == "sphere":
            return {"type": "sphere", "radius": float(element.attrib.get("radius", 0.0))}
        return {
            "type": "cylinder",
            "radius": float(element.attrib.get("radius", 0.0)),
            "length": float(element.attrib.get("length", 0.0)),
        }
    return {"type": "unknown"}


def _relative_visual_path(link_path: str, visual_root: str) -> str:
    if link_path == visual_root:
        return ""
    if link_path.startswith(visual_root + "/"):
        return link_path[len(visual_root) + 1:]
    return link_path


def _collision_items(
    root: ET.Element, link_paths: dict[str, str], visual_root: str,
) -> list[dict]:
    result = []
    for link in root.findall("link"):
        link_name = link.attrib.get("name", "")
        if not link_name or link_name not in link_paths:
            continue
        relative_prim = _relative_visual_path(link_paths[link_name], visual_root)
        for index, collision in enumerate(link.findall("collision")):
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
                  [f"{prefix}tip_link"] )
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
        "format": MANIFEST_FORMAT,
        "model": model,
        "tip_link": tip,
        "camera_frame": f"{prefix}optical" if model == "ECM" else None,
        "root_link": current,
        "joints": chain,
        "all_joints": joints,
        "active_joints": active,
        "visual": {"root": visual_root, "joints": visual_joints},
        "collision": {"root": visual_root, "items": collision_items},
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
        self.root_link = str(manifest.get("root_link", ""))
        self.tip_link = str(manifest.get("tip_link", ""))

        self._chains = {}

    def forward(self, q: np.ndarray, joint_names: tuple[str, ...]) -> tuple[np.ndarray, np.ndarray]:
        names = tuple(joint_names)
        if names not in self._chains:
            self._chains[names] = UrdfChain.from_manifest(self.joints, names)
        position, rotation, jacobian = self._chains[names].forward(q)
        transform = np.eye(4)
        transform[:3, :3] = rotation
        transform[:3, 3] = position
        return transform, jacobian

    def forward_all_links(
        self, q: np.ndarray, joint_names: tuple[str, ...],
    ) -> dict[str, np.ndarray]:
        """Return each URDF link transform, including branches such as jaws."""
        if q.shape != (len(joint_names),):
            raise ValueError("joint position has the wrong size")
        values = dict(zip(joint_names, q))
        transforms = {self.root_link: np.eye(4)}
        poses: dict[str, np.ndarray] = {}
        pending = list(self.all_joints)
        while pending:
            unresolved = []
            progressed = False
            for joint in pending:
                parent = joint["parent"]
                if parent not in transforms:
                    unresolved.append(joint)
                    continue
                transform = transforms[parent] @ _transform(
                    _rpy_matrix(*joint["origin_rpy"]), joint["origin_xyz"]
                )
                joint_type = joint["type"]
                if joint_type in ("revolute", "continuous", "prismatic"):
                    mimic = joint["mimic"]
                    if mimic is None:
                        position = values.get(joint["name"], 0.0)
                    else:
                        position = (
                            values.get(mimic["joint"], 0.0) * mimic["multiplier"]
                            + mimic["offset"]
                        )
                    axis = np.asarray(joint["axis"], dtype=float)
                    if not np.linalg.norm(axis):
                        axis = np.array([0.0, 0.0, 1.0])
                    if joint_type in ("revolute", "continuous"):
                        transform = transform @ _transform(_rotation(axis, position), (0.0, 0.0, 0.0))
                    else:
                        transform = transform @ _transform(np.eye(3), axis * position)
                poses[joint["child"]] = transform.copy()
                transforms[joint["child"]] = transform
                progressed = True
            if not progressed:
                break
            pending = unresolved
        return poses
