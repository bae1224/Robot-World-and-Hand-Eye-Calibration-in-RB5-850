"""Tests for the GoPro-through-a-capture-box path: optics, intrinsics, simulation."""
import json
import shutil
import sys
import tempfile
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hardware.rb5_camera.cameras import require_gopro_acknowledgements
from hardware.rb5_camera.intrinsics import calibrate
from hardware.rb5_camera.optics import CameraModel, load_intrinsics
from hardware.rb5_camera.simulator import BoardRenderer, default_true_camera, perturb_camera
from hardware.rb5_camera.weights import clamp_weights, sample_weights

BOARD = {"dictionary": "DICT_5X5_100", "squares_x": 8, "squares_y": 11,
         "square_length_m": 0.04, "marker_length_m": 0.03,
         "dimensions_measured": True, "legacy_pattern": False}
CAPTURE = {"min_corners": 20, "max_reprojection_px": 1.0}


def board_view(depth, rx=0.30, ry=-0.18, rz=0.05):
    import cv2
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(np.array([rx, ry, rz]))[0]
    T[:3, 3] = T[:3, :3] @ np.array([-0.16, -0.22, 0.0]) + np.array([0, 0, depth])
    return T


class OpticsTests(unittest.TestCase):
    def test_round_trip_for_every_model(self):
        points = np.array([[0.0, 0.0, 1.0], [0.12, -0.07, 0.9], [-0.2, 0.15, 1.3]])
        for model, count, fov in (("pinhole", 5, 85.0), ("rational", 8, 85.0),
                                  ("fisheye", 4, 130.0)):
            with self.subTest(model=model):
                camera = default_true_camera(1280, 720, "fisheye" if model == "fisheye"
                                             else "pinhole", fov)
                camera = CameraModel(camera.K, np.zeros(count), 1280, 720, model)
                pixels = camera.project(points, np.zeros(3), np.zeros(3))
                bearings = camera.normalize(pixels)
                np.testing.assert_allclose(bearings, points[:, :2] / points[:, 2:], atol=1e-9)

    def test_rejects_inconsistent_intrinsics(self):
        K = np.array([[900., 0, 640], [0, 900, 360], [0, 0, 1]])
        with self.assertRaises(ValueError):          # wrong coefficient count
            CameraModel(K, np.zeros(5), 1280, 720, "fisheye")
        with self.assertRaises(ValueError):          # non-positive focal length
            CameraModel(np.array([[-900., 0, 640], [0, 900, 360], [0, 0, 1]]),
                        np.zeros(5), 1280, 720)
        with self.assertRaises(ValueError):          # principal point off-sensor
            CameraModel(np.array([[900., 0, 4000], [0, 900, 360], [0, 0, 1]]),
                        np.zeros(5), 1280, 720)

    def test_intrinsics_file_is_bound_to_its_resolution(self):
        folder = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, folder, True)
        path = folder / "intr.json"
        path.write_text(json.dumps({"model": "fisheye", "width": 1920, "height": 1080,
                                    "K": [[900, 0, 960], [0, 900, 540], [0, 0, 1]],
                                    "distortion": [0.01, 0, 0, 0]}))
        model = load_intrinsics(path, (1920, 1080))
        self.assertEqual(len(model.metadata["intrinsics_sha256"]), 64)
        # Calibrating at 1080p and then streaming 720p is the single easiest way
        # to be silently wrong, so it has to be refused rather than rescaled.
        with self.assertRaises(ValueError):
            load_intrinsics(path, (1280, 720))


class RenderingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.camera = default_true_camera(1280, 720, "fisheye", 120.0)
        cls.renderer = BoardRenderer(BOARD, cls.camera, np.random.default_rng(3))

    def test_fisheye_render_detects_and_recovers_the_pose(self):
        from hardware.rb5_camera.board import estimate_board, make_board
        T = board_view(0.55)
        image = self.renderer.render(T)
        estimated, quality = estimate_board(image, make_board(BOARD), self.camera, CAPTURE)
        self.assertGreaterEqual(quality["corner_count"], 40)
        self.assertLess(quality["reprojection_rms_px"], 0.5)
        np.testing.assert_allclose(estimated[:3, 3], T[:3, 3], atol=2e-3)
        self.assertIn("translation_sigma_m", quality)
        self.assertLess(quality["translation_sigma_m"], 5e-3)

    def test_wrong_focal_length_biases_depth_and_passes_every_quality_gate(self):
        """The failure this whole branch exists to make visible.

        A focal length that is wrong by a relative amount e scales the recovered
        depth by about (1+e) while leaving the reprojection residual almost
        untouched, so no downstream check - not the pixel gate, not the AX=YB
        residual, not the optimality certificate - can see it.
        """
        from hardware.rb5_camera.board import estimate_board, make_board
        board, T = make_board(BOARD), board_view(0.55)
        image = self.renderer.render(T)
        truth, honest = estimate_board(image, board, self.camera, CAPTURE)
        wrong = perturb_camera(self.camera, focal_relative=0.02)
        biased, quality = estimate_board(image, board, wrong, CAPTURE)
        self.assertAlmostEqual(biased[2, 3] / truth[2, 3], 1.02, delta=0.004)
        self.assertLess(quality["reprojection_rms_px"],
                        honest["reprojection_rms_px"] + 0.2)
        self.assertLess(quality["reprojection_rms_px"], CAPTURE["max_reprojection_px"])

    def test_calibration_recovers_the_true_intrinsics(self):
        folder = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, folder, True)
        import cv2
        rng = np.random.default_rng(11)
        paths = []
        for index in range(12):
            T = board_view(rng.uniform(0.42, 0.75),
                           *rng.uniform(-0.45, 0.45, 3))
            path = folder / f"{index:02d}.png"
            cv2.imwrite(str(path), self.renderer.render(T))
            paths.append(path)
        camera, record = calibrate(paths, BOARD, "fisheye", min_corners=20)
        self.assertGreaterEqual(record["image_count"], 10)
        self.assertLess(record["reprojection_rms_px"], 0.5)
        self.assertLess(abs(camera.fx / self.camera.fx - 1), 0.01)
        self.assertLess(abs(camera.cx - self.camera.cx), 12)
        self.assertIn("focal_relative_sigma", record)
        self.assertLess(record["focal_relative_sigma"], 0.01)


class AcknowledgementTests(unittest.TestCase):
    def test_gopro_requires_stabilisation_to_be_confirmed_off(self):
        config = {"is_gopro": True, "acknowledgements": []}
        with self.assertRaises(ValueError) as caught:
            require_gopro_acknowledgements(config)
        self.assertIn("stabilization_off", str(caught.exception))
        config["acknowledgements"] = ["stabilization_off"]
        with self.assertRaises(ValueError):
            require_gopro_acknowledgements(config)
        config["acknowledgements"] = ["stabilization_off", "horizon_lock_off"]
        require_gopro_acknowledgements(config)
        require_gopro_acknowledgements({"is_gopro": False})

    def test_uvc_backend_demands_an_intrinsics_file(self):
        from hardware.rb5_camera.__main__ import load_config
        root = Path(__file__).resolve().parents[1]
        config = json.loads((root / "hardware" / "rb5.gopro.json").read_text())
        self.assertEqual(config["camera"]["backend"], "uvc")
        folder = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, folder, True)
        config["camera"]["intrinsics"] = ""
        path = folder / "broken.json"
        path.write_text(json.dumps(config))
        with self.assertRaises(ValueError):
            load_config(path)


class WeightTests(unittest.TestCase):
    def test_sigma_maps_to_the_cost_functions_weights(self):
        sigma_rotation_deg, sigma_translation = 0.10, 0.0004
        T = np.eye(4)
        T[:3, 3] = [0, 0, 0.5]
        sample = {"quality": {"target": {"rotation_sigma_deg": sigma_rotation_deg,
                                         "translation_sigma_m": sigma_translation}}}
        kappa, tau = sample_weights(sample, T)
        sigma_rotation = np.deg2rad(sigma_rotation_deg)
        self.assertAlmostEqual(kappa, 1 / (4 * sigma_rotation ** 2), delta=1e-6 * kappa)
        # Inverting the pose adds the lever-arm term ||t|| * sigma_R.
        expected = np.hypot(sigma_translation, 0.5 * sigma_rotation)
        self.assertAlmostEqual(tau, 1 / (2 * expected ** 2), delta=1e-6 * tau)
        self.assertIsNone(sample_weights({"quality": {"target": {}}}, T))

    def test_one_confident_view_cannot_dominate(self):
        rows = clamp_weights([[1.0, 1.0], [1e9, 1e9], [10.0, 10.0]])
        self.assertAlmostEqual(rows[1, 0], 1e4)
        self.assertAlmostEqual(rows[1, 1], 1e4)
        self.assertAlmostEqual(rows[0, 0], 1.0)


if __name__ == "__main__":
    unittest.main()
