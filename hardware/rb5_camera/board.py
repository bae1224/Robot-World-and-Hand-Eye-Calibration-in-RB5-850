"""ChArUco and ArUco metric RGB pose estimation (no depth required).

Every pose is solved from *bearings* produced by ``optics.CameraModel``, so one
code path serves a Brown-Conrady RealSense and an equidistant fisheye GoPro.
Reprojection error is always reported in pixels of the original image.
"""
import base64
from pathlib import Path
import cv2
import numpy as np
from scipy.optimize import least_squares


def get_dictionary(name):
    if not isinstance(name, str) or not name.startswith("DICT_") or not hasattr(cv2.aruco, name):
        raise ValueError(f"Unknown OpenCV marker dictionary: {name}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def make_board(config):
    if not hasattr(cv2.aruco, "CharucoDetector"):
        raise RuntimeError("Install hardware/requirements-rb5.txt (OpenCV 4.11 required)")
    dictionary = get_dictionary(config["dictionary"])
    nx, ny = config["squares_x"], config["squares_y"]
    square, marker = config["square_length_m"], config["marker_length_m"]
    if not (isinstance(nx, int) and isinstance(ny, int) and nx >= 3 and ny >= 3
            and nx * ny // 2 <= len(dictionary.bytesList) and np.isfinite([square, marker]).all()
            and 0 < marker < square):
        raise ValueError("Invalid board dimensions or marker dictionary capacity")
    ids = config.get("marker_ids")
    if ids is not None:
        if (len(ids) != nx * ny // 2 or any(type(i) is not int for i in ids)
                or len(set(ids)) != len(ids) or min(ids) < 0 or max(ids) >= len(dictionary.bytesList)):
            raise ValueError("ChArUco marker_ids must list one unique valid ID per marker square")
        board = cv2.aruco.CharucoBoard((nx, ny), square, marker, dictionary, np.array(ids, dtype=np.int32))
    else:
        board = cv2.aruco.CharucoBoard((nx, ny), square, marker, dictionary)
    board.setLegacyPattern(config.get("legacy_pattern", False))
    return board


def write_board(config, output):
    board = make_board(config)
    nx, ny = config["squares_x"], config["squares_y"]
    bitmap = board.generateImage((nx * 240, ny * 240), marginSize=0, borderBits=1)
    ok, png = cv2.imencode(".png", bitmap)
    if not ok:
        raise RuntimeError("Board PNG encoding failed")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    (output / "board.png").write_bytes(png.tobytes())
    w, h = np.array([nx, ny]) * config["square_length_m"] * 1000
    encoded = base64.b64encode(png).decode("ascii")
    svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{w+20}mm" '
           f'height="{h+20}mm" viewBox="0 0 {w+20} {h+20}">'
           '<rect width="100%" height="100%" fill="white"/>'
           f'<image x="10" y="10" width="{w}" height="{h}" '
           f'href="data:image/png;base64,{encoded}"/></svg>')
    (output / "board-print.svg").write_text(svg)
    print(f"Print SVG at 100%: board {w:g} x {h:g} mm; measure each square afterwards.")


def estimate_board(image, board, camera, capture):
    params = cv2.aruco.CharucoParameters()
    if camera.supports_opencv_charuco_refinement():
        # CharucoDetector's camera-aware corner refinement assumes Brown-Conrady.
        # Handing it equidistant coefficients would silently degrade the corners,
        # so a fisheye detects without them and relies on subpixel refinement.
        params.cameraMatrix = camera.K
        params.distCoeffs = camera.distortion
    detector = cv2.aruco.CharucoDetector(board, params)
    corners, ids, _, _ = detector.detectBoard(image)
    if ids is None or len(ids) < capture["min_corners"]:
        raise ValueError("Too few ChArUco corners")
    if board.checkCharucoCornersCollinear(ids):
        raise ValueError("ChArUco corners are collinear")
    objects, pixels = board.matchImagePoints(corners, ids)
    T, quality = solve_planar_pose(objects, pixels, camera, capture)
    return T, {**quality, "target_type": "charuco", "corner_count": len(ids),
               "corner_ids": ids.reshape(-1).tolist(),
               "corners_px": corners.reshape(-1, 2).tolist()}


def pose_residuals(objects, pixels, camera, rvec, tvec):
    return (camera.project(objects, rvec, tvec) - pixels.reshape(-1, 2)).ravel()


def refine_pose_pixels(objects, pixels, camera, rvec, tvec):
    """Levenberg-Marquardt on the *pixel* residual, for any distortion model.

    OpenCV's solvePnPRefineLM only understands Brown-Conrady, and refining in
    normalised coordinates instead would over-weight peripheral corners on a wide
    lens - exactly where a GoPro puts most of the board. Detection noise lives in
    pixels, so the refinement does too.

    Also returns the Gauss-Newton covariance of [rotation vector; translation],
    scaled by the observed residual variance. Downstream this becomes the
    per-sample precision of the maximum likelihood objective.
    """
    objects = np.asarray(objects, dtype=np.float64).reshape(-1, 3)
    pixels = np.asarray(pixels, dtype=np.float64).reshape(-1, 2)
    x0 = np.r_[np.asarray(rvec, dtype=float).reshape(3),
               np.asarray(tvec, dtype=float).reshape(3)]
    result = least_squares(lambda x: pose_residuals(objects, pixels, camera, x[:3], x[3:]),
                           x0, method="lm", xtol=1e-12, ftol=1e-12)
    residual = result.fun
    dof = max(residual.size - 6, 1)
    variance = float(residual @ residual) / dof
    try:
        covariance = variance * np.linalg.inv(result.jac.T @ result.jac)
    except np.linalg.LinAlgError:
        covariance = None
    return result.x[:3], result.x[3:], covariance


def solve_planar_pose(objects, pixels, camera, capture, method=cv2.SOLVEPNP_IPPE):
    """Planar PnP through undistorted bearings; quality always in image pixels.

    IPPE supplies both planar hypotheses; choose the positive-depth solution with
    lowest reprojection error, then refine using all matched corners.
    """
    objects = np.asarray(objects, dtype=np.float64).reshape(-1, 3)
    pixels = np.asarray(pixels, dtype=np.float64).reshape(-1, 2)
    bearings = camera.normalize(pixels)
    unit, undistorted = np.eye(3), np.zeros(5)

    def error(rvec, tvec):
        return float(np.sqrt(np.mean(np.sum(
            pose_residuals(objects, pixels, camera, rvec, tvec).reshape(-1, 2) ** 2, axis=1))))

    def in_front(rvec, tvec):
        R = cv2.Rodrigues(np.asarray(rvec, dtype=float).reshape(3, 1))[0]
        return bool(np.all((objects @ R.T + np.asarray(tvec, dtype=float).reshape(3))[:, 2] > 0))

    found, rotations, translations, _ = cv2.solvePnPGeneric(
        objects, bearings, unit, undistorted, flags=method)
    candidates = []
    if found:
        for rvec, tvec in zip(rotations, translations):
            if in_front(rvec, tvec):
                candidates.append((error(rvec, tvec), rvec, tvec))
    # IPPE can be numerically singular at exactly frontal square orientations.
    # A planar iterative initialization supplies a fallback, subject to the same
    # positive-depth, ambiguity, and reprojection checks (never accept a bad fit).
    if not candidates or min(item[0] for item in candidates) > capture["max_reprojection_px"]:
        ok, rvec, tvec = cv2.solvePnP(objects, bearings, unit, undistorted,
                                      flags=cv2.SOLVEPNP_ITERATIVE)
        if ok and in_front(rvec, tvec):
            candidates.append((error(rvec, tvec), rvec, tvec))
    if not candidates:
        raise ValueError("No positive-depth board pose")
    candidates.sort(key=lambda item: item[0])
    if len(candidates) > 1:
        first_R = cv2.Rodrigues(np.asarray(candidates[0][1], dtype=float).reshape(3, 1))[0]
        second_R = cv2.Rodrigues(np.asarray(candidates[1][1], dtype=float).reshape(3, 1))[0]
        separation_deg = np.degrees(np.linalg.norm(cv2.Rodrigues(first_R.T @ second_R)[0]))
        if candidates[1][0] - candidates[0][0] < 0.1 and separation_deg > 5:
            raise ValueError("Ambiguous planar pose; tilt the board or improve image quality")
    _, rvec, tvec = candidates[0]
    rvec, tvec, covariance = refine_pose_pixels(objects, pixels, camera, rvec, tvec)
    error_px = error(rvec, tvec)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = cv2.Rodrigues(rvec.reshape(3, 1))[0], tvec.reshape(3)
    if (not np.isfinite(T).all() or not np.isfinite(error_px)
            or error_px > capture["max_reprojection_px"]
            or np.any((objects @ T[:3, :3].T + T[:3, 3])[:, 2] <= 0)):
        raise ValueError(f"Board reprojection/cheirality check failed ({error_px:.3f} px)")
    quality = {"reprojection_rms_px": error_px,
               "planar_hypothesis_errors_px": [float(item[0]) for item in candidates],
               "point_count": int(objects.shape[0])}
    if covariance is not None and np.isfinite(covariance).all():
        quality["translation_sigma_m"] = float(np.sqrt(max(np.trace(covariance[3:, 3:]) / 3, 0)))
        quality["rotation_sigma_deg"] = float(np.degrees(
            np.sqrt(max(np.trace(covariance[:3, :3]) / 3, 0))))
        quality["pose_covariance"] = covariance.tolist()
    return T, quality


class ArucoTarget:
    def __init__(self, config):
        self.dictionary = get_dictionary(config["dictionary"])
        self.marker_id = config["id"]
        self.length = config["marker_length_m"]
        self.min_edge_px = config.get("min_edge_px", 40)
        if (type(self.marker_id) is not int or not 0 <= self.marker_id < len(self.dictionary.bytesList)
                or not np.isfinite([self.length, self.min_edge_px]).all()
                or min(self.length, self.min_edge_px) <= 0):
            raise ValueError("Invalid ArUco ID, measured side length, or minimum pixel size")
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, params)
        h = self.length / 2
        # IPPE_SQUARE required order: TL, TR, BR, BL. Marker origin at centre,
        # x right, y up, z out of printed face (different from ChArUco frame).
        self.objects = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)

    def estimate(self, image, camera, capture):
        corners, ids, _ = self.detector.detectMarkers(image)
        selected = [] if ids is None else np.flatnonzero(ids.reshape(-1) == self.marker_id)
        if len(selected) != 1:
            raise ValueError(f"Expected exactly one ArUco ID {self.marker_id}; found {len(selected)}. "
                             "Remove duplicate IDs, including any on the separate ChArUco board.")
        pixels = np.asarray(corners[selected[0]], dtype=np.float64).reshape(4, 2)
        min_edge = float(np.min(np.linalg.norm(pixels - np.roll(pixels, 1, axis=0), axis=1)))
        if min_edge < self.min_edge_px:
            raise ValueError(f"ArUco marker too small in image: {min_edge:.1f} px")
        T, quality = solve_planar_pose(self.objects, pixels, camera, capture,
                                       cv2.SOLVEPNP_IPPE_SQUARE)
        return T, {**quality, "target_type": "aruco", "marker_id": self.marker_id,
                   "corner_count": 4, "corners_px": pixels.tolist(), "min_edge_px": min_edge}


def target_config(config):
    # Existing ChArUco sessions/configurations retain their original interpretation.
    return config.get("target", {"type": "charuco"})


def make_target(config):
    target = target_config(config)
    if target["type"] == "aruco":
        return ArucoTarget(target)
    if target["type"] == "charuco":
        return make_board(config["board"])
    raise ValueError("target.type must be aruco or charuco")


def make_reference(config):
    reference = config.get("reference", {"type": "none"})["type"]
    if reference == "none":
        return None
    if (reference != "charuco" or config["mount"] != "eye_to_hand"
            or target_config(config)["type"] != "aruco"):
        raise ValueError("Fixed ChArUco reference requires eye_to_hand with an ArUco hand marker")
    board = make_board(config["board"])
    marker = ArucoTarget(config["target"])
    dictionary = board.getDictionary()
    # Dictionaries such as 4X4_50 and 4X4_100 can share the same codes.
    if dictionary.markerSize == marker.dictionary.markerSize:
        for marker_id in board.getIds().reshape(-1):
            if np.array_equal(dictionary.bytesList[marker_id], marker.dictionary.bytesList[marker.marker_id]):
                raise ValueError("Hand marker duplicates an ID/code on the ChArUco board; "
                                 "choose a different code or marker family")
    return board


def estimate_observations(image, target, reference, camera, capture):
    T_camera_target, target_quality = estimate_target(image, target, camera, capture)
    if reference is None:
        return T_camera_target, None, {"target": target_quality}
    T_camera_reference, reference_quality = estimate_board(image, reference, camera, capture)
    return T_camera_target, T_camera_reference, {
        "target": target_quality, "reference": reference_quality}


def estimate_target(image, target, camera, capture):
    if isinstance(target, ArucoTarget):
        return target.estimate(image, camera, capture)
    return estimate_board(image, target, camera, capture)


def write_marker(config, output):
    marker = ArucoTarget(config)
    pixels = (marker.dictionary.markerSize + 2) * 100
    bitmap = cv2.aruco.generateImageMarker(marker.dictionary, marker.marker_id, pixels)
    ok, png = cv2.imencode(".png", bitmap)
    if not ok:
        raise RuntimeError("Marker PNG encoding failed")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    (output / "marker.png").write_bytes(png.tobytes())
    side = marker.length * 1000
    margin = max(5.0, side * .2)
    total = side + margin * 2
    encoded = base64.b64encode(png).decode("ascii")
    (output / "marker-print.svg").write_text(
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{total}mm" height="{total}mm" '
        f'viewBox="0 0 {total} {total}"><rect width="100%" height="100%" fill="white"/>'
        f'<image x="{margin}" y="{margin}" width="{side}" height="{side}" '
        f'href="data:image/png;base64,{encoded}"/></svg>')
    print(f"ArUco {config['dictionary']} ID {marker.marker_id}: black outer edge {side:g} mm. "
          "Print SVG at 100%, retain white margin, then measure the black square.")
