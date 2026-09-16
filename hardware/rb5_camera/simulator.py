"""A virtual RB5 + wide-angle camera, so the whole pipeline runs with no hardware.

The existing `synthetic` command fabricates B directly from a known X and Y. That
checks the algebra but skips everything that actually breaks on a real rig: the
lens model, ChArUco detection, subpixel corner noise, and PnP. This module
renders real images through a real distortion model and feeds them to the real
detector, so `simulate` exercises the same code path that `collect` will.

It also separates the *true* camera from the *assumed* camera. Calibrating a
GoPro through a capture box is the one step with no ground truth, so being able
to run the pipeline with a deliberately wrong intrinsic file is what makes the
resulting extrinsic error measurable rather than hypothetical.
"""
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from .board import make_board
from .geometry import inverse
from .optics import CameraModel

TABLE_GREY = 118
PIXELS_PER_METRE = 4000
PAPER_MARGIN_M = 0.02


class BoardRenderer:
    """Renders a printed ChArUco sheet seen by an arbitrary camera model.

    Works by inverse warping: every output pixel is turned into a bearing, the
    bearing is intersected with the board plane, and the hit point is sampled from
    a high-resolution bitmap of the print. The bearing grid depends only on the
    camera, so it is computed once and reused for every view.
    """

    def __init__(self, board_config, camera, rng=None):
        self.camera = camera
        self.rng = rng or np.random.default_rng(0)
        board = make_board(board_config)
        nx, ny = board_config["squares_x"], board_config["squares_y"]
        square = board_config["square_length_m"]
        self.extent = np.array([nx * square, ny * square])
        bitmap = board.generateImage(
            (int(round(self.extent[0] * PIXELS_PER_METRE)),
             int(round(self.extent[1] * PIXELS_PER_METRE))), marginSize=0, borderBits=1)
        margin = int(round(PAPER_MARGIN_M * PIXELS_PER_METRE))
        canvas = np.full((bitmap.shape[0] + 2 * margin, bitmap.shape[1] + 2 * margin),
                         255, dtype=np.uint8)
        canvas[margin:margin + bitmap.shape[0], margin:margin + bitmap.shape[1]] = bitmap
        self.canvas = canvas
        self.margin_m = PAPER_MARGIN_M
        # Bearings for every output pixel, on the z=1 plane of the camera.
        self.rays = np.dstack([camera.unproject_grid(), np.ones((camera.height, camera.width))])

    def render(self, T_camera_board, blur_sigma=0.7, noise_sigma=1.5):
        R = np.asarray(T_camera_board, dtype=float)[:3, :3]
        t = np.asarray(T_camera_board, dtype=float)[:3, 3]
        normal = R[:, 2]                       # board plane normal, in camera frame
        offset = float(normal @ t)
        denominator = self.rays @ normal
        with np.errstate(divide="ignore", invalid="ignore"):
            depth = offset / denominator
        points = self.rays * depth[..., None]                    # hit point, camera frame
        local = (points - t) @ R                                 # board frame (R is orthonormal)
        columns = (local[..., 0] + self.margin_m) * PIXELS_PER_METRE
        rows = (local[..., 1] + self.margin_m) * PIXELS_PER_METRE
        behind = ~np.isfinite(depth) | (depth <= 0)
        columns = np.where(behind, -1.0, columns)
        rows = np.where(behind, -1.0, rows)
        image = cv2.remap(self.canvas, columns.astype(np.float32), rows.astype(np.float32),
                          cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                          borderValue=TABLE_GREY)
        if blur_sigma > 0:
            image = cv2.GaussianBlur(image, (0, 0), blur_sigma)
        if noise_sigma > 0:
            image = np.clip(image.astype(np.float32)
                            + self.rng.normal(0, noise_sigma, image.shape), 0, 255)
        image = image.astype(np.uint8)
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


def perturb_camera(camera, focal_relative=0.0, principal_px=0.0, distortion_relative=0.0,
                   rng=None):
    """A deliberately imperfect copy of a camera, to stand in for calibration error."""
    rng = rng or np.random.default_rng(0)
    K = camera.K.copy()
    K[0, 0] *= 1 + focal_relative
    K[1, 1] *= 1 + focal_relative
    if principal_px:
        K[0, 2] += rng.normal(0, principal_px)
        K[1, 2] += rng.normal(0, principal_px)
    distortion = camera.distortion * (1 + distortion_relative)
    return CameraModel(K, distortion, camera.width, camera.height, camera.model,
                       {**camera.metadata, "perturbed_from_truth": True,
                        "focal_relative_error": focal_relative,
                        "principal_point_sigma_px": principal_px,
                        "distortion_relative_error": distortion_relative})


def default_true_camera(width=1920, height=1080, model="fisheye", fov_deg=120.0):
    """Stand-in for a HERO13 in a wide FOV mode, behind a 1080p capture box."""
    if model == "fisheye":
        # Equidistant: r = f * theta, so f follows directly from the field of view.
        focal = (width / 2) / np.deg2rad(fov_deg / 2)
        distortion = np.array([0.02, -0.006, 0.0018, -0.0004])
    else:
        focal = (width / 2) / np.tan(np.deg2rad(fov_deg / 2))
        distortion = np.array([-0.21, 0.045, 0.0004, -0.0003, -0.004])
    K = np.array([[focal, 0, width / 2 - 3.5], [0, focal, height / 2 + 2.5], [0, 0, 1.0]])
    return CameraModel(K, distortion, width, height, model,
                       {"backend": "simulated", "intrinsics_source": "ground_truth",
                        "description": f"virtual HERO13-like {model} {fov_deg:g} deg FOV"})


def viewing_poses(kinematics, X, count, rng, nominal_deg=(0, -35, 75, 0, 70, 0),
                  spread_deg=14.0):
    """Joint poses near a nominal 'looking at the table' stance.

    Using forward kinematics rather than arbitrary SE(3) keeps A on the manifold
    the real robot can actually produce, which is what makes the translation
    observability check in the solver meaningful.
    """
    nominal = np.asarray(nominal_deg, dtype=float)
    poses = []
    for _ in range(count):
        joints = nominal + rng.uniform(-spread_deg, spread_deg, 6)
        poses.append((joints, kinematics.forward_degrees(joints)))
    return poses


def board_pose_for_stance(A_nominal, X, extent, distance=0.55, tilt_deg=18.0):
    """Place the board where the nominal stance is already looking at it.

    ``extent`` is the printed board size in metres. The ChArUco origin sits at a
    corner, so the board is stepped back by half of itself to put its centre on
    the optical axis.
    """
    tilt = Rotation.from_euler("xyz", [tilt_deg, -tilt_deg / 2, 0], degrees=True).as_matrix()
    T_camera_board = np.eye(4)
    T_camera_board[:3, :3] = tilt
    centre = np.array([-extent[0] / 2, -extent[1] / 2, 0.0])
    T_camera_board[:3, 3] = tilt @ centre + np.array([0, 0, distance])
    return A_nominal @ X @ T_camera_board


def board_extent(board_config):
    return np.array([board_config["squares_x"], board_config["squares_y"]],
                    dtype=float) * board_config["square_length_m"]
