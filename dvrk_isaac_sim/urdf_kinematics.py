"""Dependency-free URDF kinematics manifest parsing."""

from __future__ import annotations

import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from dvrk_simulator_base.urdf_chain import UrdfChain


def _numbers(value: str | None, count: int) -> list[float]:
    values = [float(item) for item in (value or "").split()]
    if len(values) != count:
        return [0.0] * count
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
        visual_root = "Geometry/world"
        if visual_path == visual_root:
            relative_visual_path = ""
        elif visual_path.startswith(visual_root + "/"):
            relative_visual_path = visual_path[len(visual_root) + 1:]
        else:
            relative_visual_path = visual_path
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

    manifest = {
        "format": 3,
        "model": model,
        "tip_link": tip,
        "root_link": current,
        "joints": chain,
        "active_joints": active,
        "visual": {"root": "Geometry/world", "joints": visual_joints},
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
        self.active_joints = tuple(manifest["active_joints"])

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
