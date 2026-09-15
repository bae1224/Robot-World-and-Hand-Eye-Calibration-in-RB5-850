"""T_parent_child maps a point from child coordinates into parent coordinates."""
import numpy as np
from scipy.spatial.transform import Rotation


def validate_transform(value):
    T = np.asarray(value, dtype=float)
    if T.shape != (4, 4) or not np.isfinite(T).all():
        raise ValueError("Expected a finite 4x4 transform")
    R = T[:3, :3]
    if (not np.allclose(T[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(R.T @ R, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(R), 1, atol=1e-6)):
        raise ValueError("Transform is not in SE(3)")
    return T


def rb_pose_to_transform(pose):
    """RB [x,y,z,Rx,Ry,Rz]: mm, degrees; R = Rz @ Ry @ Rx.

    Uses the controller's active TCP, which is not necessarily the flange.
    Official RB5-850 analytic IK documentation specifies this Euler convention.
    """
    pose = np.asarray(pose, dtype=float).reshape(-1)
    if pose.shape != (6,) or not np.isfinite(pose).all():
        raise ValueError("RB pose must contain six finite values in mm/degrees")
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("xyz", pose[3:], degrees=True).as_matrix()
    T[:3, 3] = pose[:3] / 1000.0
    return T


def inverse(T):
    T = validate_transform(T)
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ T[:3, 3]
    return out


def measurement_pair(T_base_tcp, T_camera_board, mount):
    A = validate_transform(T_base_tcp)
    C = validate_transform(T_camera_board)
    if mount == "eye_to_hand":
        # A X = Y B: X=T_tcp_board, Y=T_base_camera.
        return A, C
    if mount == "eye_in_hand":
        # A X = Y B: X=T_tcp_camera, Y=T_base_board (paper Eq. 6).
        return A, inverse(C)
    raise ValueError("mount must be eye_to_hand or eye_in_hand")


def to_row(T):
    T = validate_transform(T)
    q = Rotation.from_matrix(T[:3, :3]).as_quat()  # xyzw -> wxyz
    return np.r_[q[3], q[:3], T[:3, 3]]


def from_row(row):
    row = np.asarray(row, dtype=float)
    if row.shape != (7,) or not np.isfinite(row).all():
        raise ValueError("Pose row must be [qw,qx,qy,qz,tx,ty,tz]")
    if not np.isclose(np.linalg.norm(row[:4]), 1, atol=1e-5):
        raise ValueError("Quaternion must have unit norm")
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(row[[1, 2, 3, 0]]).as_matrix()
    T[:3, 3] = row[4:]
    return T


def pose_distance(T1, T2):
    delta = inverse(T1) @ validate_transform(T2)
    return (float(np.linalg.norm(delta[:3, 3])),
            float(np.degrees(Rotation.from_matrix(delta[:3, :3]).magnitude())))
