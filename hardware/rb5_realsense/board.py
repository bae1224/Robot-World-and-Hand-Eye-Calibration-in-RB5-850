"""ChArUco board definition and metric RGB pose estimation (no depth required)."""
import base64
from pathlib import Path
import cv2
import numpy as np


def make_board(config):
    if not hasattr(cv2.aruco, "CharucoDetector"):
        raise RuntimeError("Install hardware/requirements-rb5.txt (OpenCV 4.11 required)")
    if config["dictionary"] != "DICT_4X4_50":
        raise ValueError("This integration uses DICT_4X4_50")
    nx, ny = config["squares_x"], config["squares_y"]
    square, marker = config["square_length_m"], config["marker_length_m"]
    if not (isinstance(nx, int) and isinstance(ny, int) and nx >= 3 and ny >= 3
            and nx * ny // 2 <= 50 and np.isfinite([square, marker]).all()
            and 0 < marker < square):
        raise ValueError("Invalid board dimensions or marker dictionary capacity")
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.CharucoBoard((nx, ny), square, marker, dictionary)
    board.setLegacyPattern(False)
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


def estimate_board(image, board, K, distortion, capture):
    params = cv2.aruco.CharucoParameters()
    params.cameraMatrix = K
    params.distCoeffs = distortion
    detector = cv2.aruco.CharucoDetector(board, params)
    corners, ids, _, _ = detector.detectBoard(image)
    if ids is None or len(ids) < capture["min_corners"]:
        raise ValueError("Too few ChArUco corners")
    if board.checkCharucoCornersCollinear(ids):
        raise ValueError("ChArUco corners are collinear")
    objects, pixels = board.matchImagePoints(corners, ids)
    # IPPE supplies both planar hypotheses; choose the positive-depth solution
    # with lowest reprojection error, then refine using all matched corners.
    found, rotations, translations, _ = cv2.solvePnPGeneric(
        objects, pixels, K, distortion, flags=cv2.SOLVEPNP_IPPE)
    candidates = []
    if found:
        for rvec, tvec in zip(rotations, translations):
            R = cv2.Rodrigues(rvec)[0]
            if np.all((objects.reshape(-1, 3) @ R.T + tvec.reshape(3))[:, 2] > 0):
                projected = cv2.projectPoints(objects, rvec, tvec, K, distortion)[0]
                error = float(np.sqrt(np.mean(np.sum(
                    (projected.reshape(-1, 2) - pixels.reshape(-1, 2)) ** 2, axis=1))))
                candidates.append((error, rvec, tvec))
    if not candidates:
        raise ValueError("No positive-depth board pose")
    candidates.sort(key=lambda item: item[0])
    if len(candidates) > 1:
        first_R = cv2.Rodrigues(candidates[0][1])[0]
        second_R = cv2.Rodrigues(candidates[1][1])[0]
        separation_deg = np.degrees(np.linalg.norm(cv2.Rodrigues(first_R.T @ second_R)[0]))
        if candidates[1][0] - candidates[0][0] < 0.1 and separation_deg > 5:
            raise ValueError("Ambiguous planar pose; tilt the board or improve image quality")
    _, rvec, tvec = candidates[0]
    rvec, tvec = cv2.solvePnPRefineLM(objects, pixels, K, distortion, rvec, tvec)
    projected = cv2.projectPoints(objects, rvec, tvec, K, distortion)[0]
    error = float(np.sqrt(np.mean(np.sum(
        (projected.reshape(-1, 2) - pixels.reshape(-1, 2)) ** 2, axis=1))))
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = cv2.Rodrigues(rvec)[0], tvec.reshape(3)
    if (not np.isfinite(T).all() or not np.isfinite(error)
            or error > capture["max_reprojection_px"]
            or np.any((objects.reshape(-1, 3) @ T[:3, :3].T + T[:3, 3])[:, 2] <= 0)):
        raise ValueError(f"Board reprojection/cheirality check failed ({error:.3f} px)")
    return T, {"corner_count": len(ids), "reprojection_rms_px": error,
               "corner_ids": ids.reshape(-1).tolist(),
               "corners_px": corners.reshape(-1, 2).tolist(),
               "planar_hypothesis_errors_px": [item[0] for item in candidates]}
