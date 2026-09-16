"""Atomic per-sample records are the source of truth; CSVs are derived exports."""
import json
import os
from pathlib import Path
import numpy as np
from .geometry import measurement_pair, to_row
from .weights import clamp_weights, sample_weights


def atomic_json(path, value):
    path = Path(path)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(tmp, path)


def export_csv(session, output=None):
    session = Path(session)
    metadata = json.loads((session / "session.json").read_text())
    records = sorted((session / "samples").glob("*.json"))
    if not records:
        raise ValueError("No accepted samples in this session")
    reference_type = metadata["config"].get("reference", {"type": "none"})["type"]
    if reference_type not in ("none", "charuco"):
        raise ValueError("Invalid reference type")
    target_type = metadata["config"].get("target", {"type": "charuco"})["type"]
    if target_type not in ("aruco", "charuco"):
        raise ValueError("Invalid target type")
    if reference_type == "charuco" and target_type != "aruco":
        raise ValueError("Fixed ChArUco reference requires an ArUco hand marker")
    A, B, references, weights = [], [], [], []
    for path in records:
        sample = json.loads(path.read_text())
        camera_target = sample.get("T_camera_target", sample.get("T_camera_board"))
        if reference_type == "charuco" and "T_camera_reference" not in sample:
            raise ValueError(f"{path.name}: missing same-frame ChArUco reference observation")
        camera_reference = sample["T_camera_reference"] if reference_type == "charuco" else None
        robot_pose = sample.get("T_base_hand", sample.get("T_base_tcp"))
        a, b = measurement_pair(robot_pose, camera_target,
                                metadata["config"]["mount"], camera_reference)
        if camera_reference is not None:
            references.append(to_row(camera_reference))
        A.append(to_row(a))
        B.append(to_row(b))
        weights.append(sample_weights(sample, camera_target))
    output = Path(output) if output else session / "combined"
    # Never mix old exports with a new acquisition: each export gets a new directory.
    output.mkdir(parents=True, exist_ok=False)
    np.savetxt(output / "tag_0_cam_0_A.csv", A, delimiter=",", fmt="%.17g")
    np.savetxt(output / "tag_0_cam_0_B.csv", B, delimiter=",", fmt="%.17g")
    if references:
        np.savetxt(output / "T_camera_charuco.csv", references, delimiter=",", fmt="%.17g")
    # Per-sample precisions for the MLE. Written only when every sample carries a
    # PnP covariance, since a partial set would silently mix two noise models.
    if all(w is not None for w in weights):
        np.savetxt(output / "weights.csv", clamp_weights(weights), delimiter=",",
                   fmt="%.17g", header="kappa,tau", comments="")
    atomic_json(output / "metadata.json", {**metadata, "sample_count": len(A)})
    (output / "convention.toml").write_text(
        f'mount = "{metadata["config"]["mount"]}"\nunits = "metres"\n'
        f'target_type = "{target_type}"\n'
        f'reference_type = "{reference_type}"\n'
        f'hand_frame = "{"urdf_tip" if metadata["config"].get("robot", {}).get("pose_source") == "urdf_fk" else "controller_tcp"}"\n'
        'columns = "qw,qx,qy,qz,tx,ty,tz"\n')
    return output
