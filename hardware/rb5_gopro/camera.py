"""Use the calibrated encoded pixel frame and original-pixel fisheye residuals."""
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from hardware.rb5_realsense.kinematics import ROOT


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def repo_path(value):
    path = Path(value)
    return path if path.is_absolute() else ROOT / path


class GoProCamera:
    def __init__(self, config):
        self.path = repo_path(config['intrinsics_path'])
        self.validation_path = repo_path(config['validation_path'])
        self.calibration = json.loads(self.path.read_text())
        validation = json.loads(self.validation_path.read_text())
        c = self.calibration
        if c.get('schema') != 'gopro-intrinsics/v1' or c.get('model') != 'fisheye':
            raise ValueError('GoPro importer requires a gopro-intrinsics/v1 fisheye calibration')
        if c.get('pixel_orientation') != 'encoded_pixels_ignore_exif':
            raise ValueError('Unsupported calibrated pixel orientation')
        self.K, self.D = np.asarray(c['K'], dtype=float), np.asarray(c['D'], dtype=float)
        if (self.K.shape != (3, 3) or self.D.shape != (4,)
                or not np.isfinite(self.K).all() or not np.isfinite(self.D).all()
                or min(self.K[0, 0], self.K[1, 1]) <= 0
                or not np.allclose(self.K[2], [0, 0, 1])
                or self.K[0, 1] != 0 or self.K[1, 0] != 0):
            raise ValueError('Invalid fisheye K/D (zero skew required)')
        self.size = c['image_size']
        if len(self.size) != 2 or any(type(v) is not int or v <= 0 for v in self.size):
            raise ValueError('Invalid calibration image_size')
        digest = sha256(self.path)
        if validation.get('calibration_sha256') != digest or validation.get('quality_status') != 'PASS':
            raise ValueError('Intrinsic validation must PASS and match the calibration SHA256')
        self.metadata = {'name': 'GoPro HERO13 Black', 'model': 'fisheye',
                         'K': self.K.tolist(), 'D': self.D.tolist(), 'image_size': self.size,
                         'pixel_orientation': c['pixel_orientation'],
                         'camera_profile': c['camera_profile'], 'intrinsics_sha256': digest,
                         'validation_sha256': sha256(self.validation_path),
                         'validation_rms_px': validation['overall']['rms_px']}

    def read(self, path):
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE | cv2.IMREAD_IGNORE_ORIENTATION)
        if image is None:
            raise ValueError(f'Cannot decode image: {path}')
        if list(image.shape[::-1]) != self.size:
            raise ValueError(f'{path}: expected original {self.size} pixels; no resize/crop/rotation')
        return image

    def project(self, objects, parameters):
        return cv2.fisheye.projectPoints(objects, parameters[:3], parameters[3:], self.K, self.D)[0]

    def solve(self, objects, pixels, capture):
        objects = np.asarray(objects, dtype=np.float64).reshape(-1, 1, 3)
        pixels = np.asarray(pixels, dtype=np.float64).reshape(-1, 1, 2)
        normalized = cv2.fisheye.undistortPoints(pixels, self.K, self.D)
        if not np.isfinite(normalized).all() or np.max(np.abs(normalized)) > 1e5:
            raise ValueError('Fisheye undistortion failed')
        found, rotations, translations, _ = cv2.solvePnPGeneric(
            objects, normalized, np.eye(3), None, flags=cv2.SOLVEPNP_IPPE)
        candidates = []
        if found:
            for rvec, tvec in zip(rotations, translations):
                initial = np.r_[rvec.ravel(), tvec.ravel()]
                if not np.isfinite(initial).all():
                    continue
                fit = least_squares(lambda p: (self.project(objects, p) - pixels).ravel(),
                                    initial, max_nfev=200, ftol=1e-10, xtol=1e-10, gtol=1e-10)
                if not fit.success or not np.isfinite(fit.x).all():
                    continue
                T = np.eye(4)
                T[:3, :3] = Rotation.from_rotvec(fit.x[:3]).as_matrix()
                T[:3, 3] = fit.x[3:]
                if np.min((objects.reshape(-1, 3) @ T[:3, :3].T + T[:3, 3])[:, 2]) <= 0:
                    continue
                rms = float(np.sqrt(np.mean(np.sum(fit.fun.reshape(-1, 2)**2, axis=1))))
                candidates.append((rms, T))
        if not candidates:
            raise ValueError('No converged positive-depth fisheye board pose')
        candidates.sort(key=lambda item: item[0])
        error, T = candidates[0]
        if error > capture['max_reprojection_px']:
            raise ValueError(f'Board reprojection RMS {error:.3f} px exceeds threshold')
        if len(candidates) > 1:
            separation = np.degrees(Rotation.from_matrix(T[:3, :3].T @ candidates[1][1][:3, :3]).magnitude())
            if candidates[1][0] - error < .1 and separation > 5:
                raise ValueError('Ambiguous planar pose; use a more tilted view')
        return T, {'reprojection_rms_px': error, 'distortion_model': 'fisheye',
                   'residual_pixel_frame': 'original_encoded_pixels',
                   'refined_hypothesis_errors_px': [item[0] for item in candidates]}

    def estimate(self, image, board, capture):
        # ChArUco's cameraMatrix/distCoeffs interface is Brown, not fisheye.
        # Match intrinsic calibration's local-homography detection on original pixels.
        corners, ids, _, _ = cv2.aruco.CharucoDetector(board).detectBoard(image)
        if ids is None or len(ids) < capture['min_corners']:
            raise ValueError('Too few ChArUco corners')
        if board.checkCharucoCornersCollinear(ids):
            raise ValueError('ChArUco corners are collinear')
        objects, pixels = board.matchImagePoints(corners, ids)
        T, quality = self.solve(objects, pixels, capture)
        return T, {'target': {**quality, 'target_type': 'charuco', 'corner_count': len(ids),
                             'corner_ids': ids.ravel().tolist(),
                             'corners_px': corners.reshape(-1, 2).tolist()}}
