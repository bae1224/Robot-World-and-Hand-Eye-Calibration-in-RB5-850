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
        settings = c.get('detection_settings', {})
        self.detection_settings = {
            'min_marker_perimeter_rate': settings.get('min_marker_perimeter_rate', .03),
            'adaptive_thresh_win_size_max': settings.get('adaptive_thresh_win_size_max', 23),
            'fallback_default_detector': settings.get('fallback_default_detector', False),
            'min_corners': settings.get('min_corners', 20),
            'min_area_ratio': settings.get('min_area_ratio', .01),
        }
        rate = self.detection_settings['min_marker_perimeter_rate']
        window = self.detection_settings['adaptive_thresh_win_size_max']
        if (not np.isfinite(rate) or not 0 < rate < 1
                or type(window) is not int or window < 3 or window % 2 == 0
                or type(self.detection_settings['fallback_default_detector']) is not bool
                or type(self.detection_settings['min_corners']) is not int
                or self.detection_settings['min_corners'] < 4
                or not 0 <= self.detection_settings['min_area_ratio'] < 1):
            raise ValueError('Invalid calibrated ChArUco detection settings')
        # Use the same sliced dictionary as intrinsic detection, even when the
        # application board expresses the same pattern with family IDs 50..112.
        spec = c['board']
        family = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, spec['dictionary']))
        dictionary = cv2.aruco.Dictionary(family.bytesList[spec['dictionary_offset']:].copy(),
                                          family.markerSize, family.maxCorrectionBits)
        self.board = cv2.aruco.CharucoBoard((spec['squares_x'], spec['squares_y']),
                                           spec['square_length_m'], spec['marker_length_m'], dictionary)
        self.board.setLegacyPattern(spec['legacy_pattern'])
        params = cv2.aruco.DetectorParameters()
        params.minMarkerPerimeterRate = rate
        params.adaptiveThreshWinSizeMax = window
        self.detector = cv2.aruco.CharucoDetector(self.board, detectorParams=params)
        self.fallback = (cv2.aruco.CharucoDetector(self.board)
                         if self.detection_settings['fallback_default_detector'] else None)
        self.max_ray_angle_deg = config.get('max_ray_angle_deg')
        self.max_distorted_radius = None
        if self.max_ray_angle_deg is not None:
            if not np.isfinite(self.max_ray_angle_deg) or not 0 < self.max_ray_angle_deg < 90:
                raise ValueError('max_ray_angle_deg must be finite and between 0 and 90')
            theta = np.deg2rad(self.max_ray_angle_deg)
            # d(theta_d)/d(theta) is a polynomial in theta squared. Any real
            # root in the allowed interval destroys strict invertibility there.
            derivative = np.r_[1., self.D * np.array([3, 5, 7, 9])]
            roots = np.polynomial.polynomial.polyroots(derivative)
            if any(abs(z.imag) < 1e-9 and 0 < z.real <= theta**2 for z in roots):
                raise ValueError('Fisheye model is not monotonic inside max_ray_angle_deg')
            self.max_distorted_radius = theta * (1 + sum(
                k * theta**(2*i+2) for i, k in enumerate(self.D)))
        self.metadata = {'name': 'GoPro HERO13 Black', 'model': 'fisheye',
                         'K': self.K.tolist(), 'D': self.D.tolist(), 'image_size': self.size,
                         'pixel_orientation': c['pixel_orientation'],
                         'camera_profile': c['camera_profile'], 'intrinsics_sha256': digest,
                         'validation_sha256': sha256(self.validation_path),
                         'validation_rms_px': validation['overall']['rms_px'],
                         'detection_settings': self.detection_settings,
                         'max_ray_angle_deg': self.max_ray_angle_deg}

    def check_board(self, board):
        def same(a, b):
            a, b = np.asarray(a), np.asarray(b)
            return a.shape == b.shape and np.allclose(a, b, rtol=0, atol=1e-8)
        codes = board.getDictionary().bytesList[board.getIds().ravel()]
        expected_codes = self.board.getDictionary().bytesList[self.board.getIds().ravel()]
        if (not same(board.getChessboardCorners(), self.board.getChessboardCorners())
                or not same(board.getObjPoints(), self.board.getObjPoints())
                or not np.array_equal(codes, expected_codes)):
            raise ValueError('Experiment board does not match intrinsic calibration geometry/pattern')

    def detect(self, image, board):
        self.check_board(board)
        if list(image.shape[1::-1]) != self.size:
            raise ValueError(f'Expected original {self.size} pixels for ChArUco detection')
        corners, ids, _, _ = self.detector.detectBoard(image)
        count = 0 if ids is None else len(ids)
        selected = 'configured'
        if self.fallback is not None and count < len(self.board.getChessboardCorners()):
            other_corners, other_ids, _, _ = self.fallback.detectBoard(image)
            if other_ids is not None and len(other_ids) > count:
                corners, ids = other_corners, other_ids
                selected = 'opencv_default_fallback'
        return corners, ids, selected

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
        if not np.isfinite(pixels).all() or not np.isfinite(objects).all():
            raise ValueError('Non-finite board observations')
        # Limit use to the monotonic calibrated radial interval before inversion.
        radii = np.linalg.norm((pixels.reshape(-1, 2) - self.K[:2, 2])
                               / np.diag(self.K)[:2], axis=1)
        if self.max_distorted_radius is not None and np.any(radii > self.max_distorted_radius):
            raise ValueError(f'Board corners exceed calibrated ray limit {self.max_ray_angle_deg:g} deg')
        normalized = cv2.fisheye.undistortPoints(pixels, self.K, self.D)
        if not np.isfinite(normalized).all() or np.max(np.abs(normalized)) > 1e5:
            raise ValueError('Fisheye undistortion failed')
        roundtrip = cv2.fisheye.distortPoints(normalized, self.K, self.D)
        if np.max(np.linalg.norm(roundtrip - pixels, axis=2)) > 1e-3:
            raise ValueError('Fisheye inverse does not reproduce the observed pixels')
        max_ray = float(np.degrees(np.arctan(np.linalg.norm(
            normalized.reshape(-1, 2), axis=1))).max())
        if self.max_ray_angle_deg is not None and max_ray > self.max_ray_angle_deg + 1e-7:
            raise ValueError(f'Board corners exceed calibrated ray limit {self.max_ray_angle_deg:g} deg')
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
                   'max_observed_ray_angle_deg': max_ray,
                   'refined_hypothesis_errors_px': [item[0] for item in candidates]}

    def estimate(self, image, board, capture):
        # ChArUco's cameraMatrix/distCoeffs interface is Brown, not fisheye.
        # Match intrinsic calibration's local-homography detection on original pixels.
        corners, ids, selected = self.detect(image, board)
        if ids is None or len(ids) < max(capture['min_corners'], self.detection_settings['min_corners']):
            raise ValueError('Too few ChArUco corners')
        if board.checkCharucoCornersCollinear(ids):
            raise ValueError('ChArUco corners are collinear')
        area = cv2.contourArea(cv2.convexHull(corners.reshape(-1, 2))) / (image.shape[0] * image.shape[1])
        if area < self.detection_settings['min_area_ratio']:
            raise ValueError('Board image area is below intrinsic detection minimum')
        objects, pixels = board.matchImagePoints(corners, ids)
        T, quality = self.solve(objects, pixels, capture)
        return T, {'target': {**quality, 'target_type': 'charuco', 'corner_count': len(ids),
                             'selected_detector': selected, 'image_area_ratio': area,
                             'corner_ids': ids.ravel().tolist(),
                             'corners_px': corners.reshape(-1, 2).tolist()}}
