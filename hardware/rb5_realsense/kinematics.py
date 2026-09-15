"""Position-only URDF forward kinematics; no meshes, ROS, simulator or robot motion.

URDF joint transforms are T_parent_joint(origin) @ T_joint_child(q).
The selected base/tip are model frames, not the controller's configured TCP.
"""
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]


def vector(text, default):
    values = np.asarray(default if text is None else [float(x) for x in text.split()], dtype=float)
    if values.shape != (3,) or not np.isfinite(values).all():
        raise ValueError("URDF xyz/rpy/axis must have three finite numbers")
    return values


class URDFChain:
    def __init__(self, path, base_link, tip_link):
        self.path = Path(path).resolve()
        data = self.path.read_bytes()
        self.model_bytes = data
        root = ET.fromstring(data)
        if root.tag != "robot" or any("xacro" in node.tag for node in root.iter()):
            raise ValueError("Provide an expanded URDF <robot>, not xacro or MJCF")
        links = [node.attrib["name"] for node in root.findall("link")]
        if len(set(links)) != len(links) or base_link not in links or tip_link not in links:
            raise ValueError("Unknown or duplicate URDF base/tip link")
        by_child, names = {}, set()
        for node in root.findall("joint"):
            name = node.attrib["name"]
            parent, child = node.find("parent"), node.find("child")
            if parent is None or child is None:
                raise ValueError(f"URDF joint {name} lacks parent/child")
            parent, child = parent.attrib["link"], child.attrib["link"]
            if name in names or child in by_child or parent not in links or child not in links:
                raise ValueError("URDF contains duplicate joints, multiple parents or unknown links")
            by_child[child] = (parent, node)
            names.add(name)
        path_nodes, seen, link = [], set(), tip_link
        while link != base_link:
            if link in seen or link not in by_child:
                raise ValueError("Tip is not connected below the selected base, or chain is cyclic")
            seen.add(link)
            link, node = by_child[link]
            path_nodes.append(node)
        self.joints = []
        for node in reversed(path_nodes):
            kind = node.attrib["type"]
            if kind not in ("fixed", "revolute", "continuous", "prismatic") or node.find("mimic") is not None:
                raise ValueError("Unsupported joint on chain: floating/planar/mimic joints need an explicit model")
            origin_node = node.find("origin")
            origin_attrs = {} if origin_node is None else origin_node.attrib
            T = np.eye(4)
            T[:3, 3] = vector(origin_attrs.get("xyz"), [0, 0, 0])
            T[:3, :3] = Rotation.from_euler("xyz", vector(origin_attrs.get("rpy"), [0, 0, 0])).as_matrix()
            axis_node = node.find("axis")
            axis = vector(None if axis_node is None else axis_node.get("xyz"), [1, 0, 0])
            if kind != "fixed":
                if np.linalg.norm(axis) < 1e-12:
                    raise ValueError("Zero URDF joint axis")
                axis /= np.linalg.norm(axis)
            self.joints.append((node.attrib["name"], kind, T, axis))
        self.movable = [name for name, kind, _, _ in self.joints if kind != "fixed"]
        self.metadata = {"model_name": root.get("name", ""), "urdf_path": str(self.path),
                         "urdf_sha256": hashlib.sha256(data).hexdigest(),
                         "base_link": base_link, "tip_link": tip_link,
                         "chain_joint_names": self.movable}

    def forward(self, positions):
        """positions: exact mapping of movable joint names to radians (or metres)."""
        if set(positions) != set(self.movable):
            raise ValueError("Joint names must match all movable joints on the selected chain")
        if not all(np.isscalar(q) and np.isfinite(q) for q in positions.values()):
            raise ValueError("Joint positions must be finite scalars")
        T = np.eye(4)
        for name, kind, origin, axis in self.joints:
            motion = np.eye(4)
            if kind in ("revolute", "continuous"):
                motion[:3, :3] = Rotation.from_rotvec(axis * positions[name]).as_matrix()
            elif kind == "prismatic":
                motion[:3, 3] = axis * positions[name]
            T = T @ origin @ motion
        return T


class RB5Kinematics:
    def __init__(self, config):
        path = Path(config["urdf_path"])
        if not path.is_absolute():
            path = ROOT / path
        self.chain = URDFChain(path, config["base_link"], config["tip_link"])
        self.names = config["joint_names"]
        if len(self.names) != 6 or len(set(self.names)) != 6 or set(self.names) != set(self.chain.movable):
            raise ValueError("Map six encoder entries to the six URDF joint names explicitly")
        if any(kind == "prismatic" for _, kind, _, _ in self.chain.joints):
            raise ValueError("RB5 encoders provide six rotational joint angles")
        self.signs = np.asarray(config.get("joint_signs", [1] * 6), dtype=float)
        self.offsets = np.asarray(config.get("joint_offsets_deg", [0] * 6), dtype=float)
        if (self.signs.shape != (6,) or not np.isin(self.signs, [-1, 1]).all()
                or self.offsets.shape != (6,) or not np.isfinite(self.offsets).all()):
            raise ValueError("Joint signs must be +/-1; offsets must be six finite degree values")
        self.metadata = {**self.chain.metadata, "encoder_joint_names": list(self.names),
                         "joint_signs": self.signs.tolist(), "joint_offsets_deg": self.offsets.tolist(),
                         "pose_source": "urdf_fk", "joint_source": "rbpodo.sdata.jnt_ang",
                         "model_matches_robot": config.get("model_matches_robot", False),
                         "hand_frame": "URDF tip_link; not the configured controller TCP"}

    def forward_degrees(self, joints):
        joints = np.asarray(joints, dtype=float)
        if joints.shape != (6,) or not np.isfinite(joints).all():
            raise ValueError("RB5 joint input must be six finite measured angles in degrees")
        q = np.deg2rad(self.signs * joints + self.offsets)
        return self.chain.forward(dict(zip(self.names, q)))
