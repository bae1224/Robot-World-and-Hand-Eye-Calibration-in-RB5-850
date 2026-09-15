"""Atomic per-sample records are the source of truth; CSVs are derived exports."""
import json
import os
from pathlib import Path
import numpy as np
from .geometry import measurement_pair, to_row


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
    A, B = [], []
    for path in records:
        sample = json.loads(path.read_text())
        a, b = measurement_pair(sample["T_base_tcp"], sample["T_camera_board"],
                                metadata["config"]["mount"])
        A.append(to_row(a))
        B.append(to_row(b))
    output = Path(output) if output else session / "combined"
    # Never mix old exports with a new acquisition: each export gets a new directory.
    output.mkdir(parents=True, exist_ok=False)
    np.savetxt(output / "tag_0_cam_0_A.csv", A, delimiter=",", fmt="%.17g")
    np.savetxt(output / "tag_0_cam_0_B.csv", B, delimiter=",", fmt="%.17g")
    atomic_json(output / "metadata.json", {**metadata, "sample_count": len(A)})
    (output / "convention.toml").write_text(
        f'mount = "{metadata["config"]["mount"]}"\nunits = "metres"\n'
        'columns = "qw,qx,qy,qz,tx,ty,tz"\n')
    return output
