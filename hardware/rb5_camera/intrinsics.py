"""Intrinsic calibration from ChArUco images, with a covariance for the result.

A RealSense ships factory intrinsics; a GoPro behind a capture box does not, so
for that path this file produces the artefact every pose downstream depends on.

The covariance is not decoration. Section 7 of the paper establishes tightness of
the SDP relaxation on a *neighbourhood* of the measurement parameter
theta = vec({A_i, B_i}). Since B_i is a smooth function of the intrinsics, a
bound on the intrinsic error is what lets that neighbourhood be stated in terms
of the camera calibration rather than in terms of unobservable pose noise. See
docs/CERTIFIED_INTRINSICS.md.
"""
from datetime import datetime, timezone
import json
from pathlib import Path
import cv2
import numpy as np
from .board import make_board
from .optics import MODELS, CameraModel

# OpenCV parameter order shared by the pinhole and rational models.
BROWN_NAMES = ["k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6"]
FISHEYE_NAMES = ["k1", "k2", "k3", "k4"]


def detect_charuco(image, board, min_corners):
    """Plain 2D ChArUco detection: no camera model, because we are solving for it."""
    detector = cv2.aruco.CharucoDetector(board, cv2.aruco.CharucoParameters())
    corners, ids, _, _ = detector.detectBoard(image)
    if ids is None or len(ids) < min_corners:
        return None
    if board.checkCharucoCornersCollinear(ids):
        return None
    objects, pixels = board.matchImagePoints(corners, ids)
    if objects is None or len(objects) < min_corners:
        return None
    return (np.asarray(objects, dtype=np.float64).reshape(-1, 3),
            np.asarray(pixels, dtype=np.float64).reshape(-1, 2))


def _residuals(model, views, parameters):
    """Stacked pixel residuals for [intrinsics; per-view rvec,tvec]."""
    count = MODELS[model.model]
    probe = CameraModel(np.array([[parameters[0], 0, parameters[2]],
                                  [0, parameters[1], parameters[3]],
                                  [0, 0, 1.0]]),
                        parameters[4:4 + count], model.width, model.height, model.model)
    out = []
    for index, (objects, pixels) in enumerate(views):
        base = 4 + count + 6 * index
        out.append((probe.project(objects, parameters[base:base + 3],
                                  parameters[base + 3:base + 6]) - pixels).ravel())
    return np.concatenate(out)


def intrinsic_covariance(model, views, rvecs, tvecs):
    """Covariance of [fx, fy, cx, cy, distortion...] with the extrinsics marginalised out.

    Built from a numerical Jacobian of the same pixel residual the calibration
    minimised, so it is the standard first-order estimate: sigma^2 (J^T J)^-1,
    restricted to the intrinsic block. Marginalising rather than fixing the
    extrinsics matters, because focal length and board distance are strongly
    correlated and holding the poses fixed would understate the focal uncertainty
    by a large factor.
    """
    count = MODELS[model.model]
    intrinsic_count = 4 + count
    parameters = np.concatenate([[model.fx, model.fy, model.cx, model.cy],
                                 model.distortion,
                                 np.concatenate([np.r_[np.asarray(r).reshape(3),
                                                       np.asarray(t).reshape(3)]
                                                 for r, t in zip(rvecs, tvecs)])])
    base = _residuals(model, views, parameters)
    dof = base.size - parameters.size
    if dof <= 0:
        return None, None
    jacobian = np.empty((base.size, parameters.size))
    for i in range(parameters.size):
        step = 1e-6 * max(abs(parameters[i]), 1e-3)
        shifted = parameters.copy()
        shifted[i] += step
        jacobian[:, i] = (_residuals(model, views, shifted) - base) / step
    variance = float(base @ base) / dof
    normal = jacobian.T @ jacobian
    try:
        covariance = variance * np.linalg.inv(normal)
    except np.linalg.LinAlgError:
        return None, None
    block = covariance[:intrinsic_count, :intrinsic_count]
    if not np.isfinite(block).all():
        return None, None
    return block, float(np.sqrt(variance / 2))


def calibrate(image_paths, board_config, model="pinhole", min_corners=12):
    if model not in MODELS:
        raise ValueError(f"model must be one of {sorted(MODELS)}")
    board = make_board(board_config)
    views, used, size = [], [], None
    for path in sorted(Path(p) for p in image_paths):
        image = cv2.imread(str(path))
        if image is None:
            raise ValueError(f"{path}: not a readable image")
        height, width = image.shape[:2]
        if size is None:
            size = (width, height)
        elif size != (width, height):
            raise ValueError("All calibration images must share one resolution and FOV mode")
        found = detect_charuco(image, board, min_corners)
        if found is None:
            continue
        views.append(found)
        used.append(str(path))
    if len(views) < 8:
        raise ValueError(f"Only {len(views)} usable views; collect at least 8 (15-30 is better) "
                         "covering the corners of the frame and several board tilts")

    objects = [v[0].reshape(-1, 1, 3) for v in views]
    pixels = [v[1].reshape(-1, 1, 2) for v in views]
    if model == "fisheye":
        K = np.eye(3)
        distortion = np.zeros((4, 1))
        flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
        rms, K, distortion, rvecs, tvecs = cv2.fisheye.calibrate(
            objects, pixels, size, K, distortion, flags=flags,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-10))
        coefficients = np.asarray(distortion).reshape(-1)[:4]
    else:
        flags = cv2.CALIB_RATIONAL_MODEL if model == "rational" else 0
        rms, K, distortion, rvecs, tvecs = cv2.calibrateCamera(
            objects, pixels, size, None, None, flags=flags,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-10))
        coefficients = np.asarray(distortion).reshape(-1)[:MODELS[model]]
        if coefficients.size < MODELS[model]:
            coefficients = np.r_[coefficients, np.zeros(MODELS[model] - coefficients.size)]

    camera = CameraModel(K, coefficients, size[0], size[1], model)
    covariance, pixel_sigma = intrinsic_covariance(camera, views, rvecs, tvecs)
    per_view = []
    for (objects_i, pixels_i), rvec, tvec in zip(views, rvecs, tvecs):
        residual = camera.project(objects_i, rvec, tvec) - pixels_i
        per_view.append(float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1)))))

    names = ["fx", "fy", "cx", "cy"] + (
        FISHEYE_NAMES if model == "fisheye" else BROWN_NAMES[:MODELS[model]])
    record = {
        "model": model, "width": size[0], "height": size[1],
        "K": np.asarray(K, dtype=float).tolist(),
        "distortion": coefficients.tolist(),
        "parameter_names": names,
        "reprojection_rms_px": float(rms),
        "worst_view_rms_px": float(max(per_view)),
        "per_view_rms_px": per_view,
        "image_count": len(views),
        "images": used,
        "board": dict(board_config),
        "horizontal_fov_deg": camera.horizontal_fov_deg(),
        "calibrated_utc": datetime.now(timezone.utc).isoformat(),
        "note": ("Valid only for this exact resolution and camera FOV/lens mode. "
                 "Re-run after any change to the capture chain."),
    }
    if covariance is not None:
        sigmas = np.sqrt(np.clip(np.diag(covariance), 0, None))
        record["intrinsic_covariance"] = covariance.tolist()
        record["intrinsic_sigma"] = dict(zip(names, sigmas.tolist()))
        record["focal_relative_sigma"] = float(
            max(sigmas[0] / camera.fx, sigmas[1] / camera.fy))
        record["pixel_sigma_px"] = pixel_sigma
    return camera, record


def write_calibration(record, output):
    output = Path(output)
    if output.exists():
        raise ValueError(f"{output} already exists; intrinsics are versioned, choose a new file")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    return output
