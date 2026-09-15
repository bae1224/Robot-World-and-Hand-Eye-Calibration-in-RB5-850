import json
import copy
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from hardware.rb5_realsense.geometry import (
    rb_pose_to_transform, inverse, measurement_pair, to_row, from_row)
from hardware.rb5_realsense.dataset import atomic_json, export_csv
from hardware.rb5_realsense.devices import RobotReader, require_stationary

CONFIG = json.loads(Path("tests/fixtures/aruco_charuco_config.json").read_text())


class GeometryTests(unittest.TestCase):
    def test_rb_units_and_rotation_order(self):
        T = rb_pose_to_transform([1000, -200, 500, 90, 0, 90])
        expected = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
        np.testing.assert_allclose(T[:3, :3], expected, atol=1e-14)
        np.testing.assert_allclose(T[:3, 3], [1, -.2, .5])

    def test_both_mounts_and_csv_convention(self):
        X = rb_pose_to_transform([20, 30, 40, 20, -30, 40])
        Y = rb_pose_to_transform([500, 600, 800, -10, 50, -20])
        for mount in ("eye_to_hand", "eye_in_hand"):
            for i in range(10):
                A = rb_pose_to_transform([100+i, 200, 400, i*10, 40, 30])
                C = inverse(Y) @ A @ X
                if mount == "eye_in_hand":
                    C = inverse(C)
                a, b = measurement_pair(A, C, mount)
                np.testing.assert_allclose(a @ X, Y @ b, atol=1e-12)
                np.testing.assert_allclose(from_row(to_row(b)), b, atol=1e-12)
        np.testing.assert_allclose(to_row(np.eye(4)), [1, 0, 0, 0, 0, 0, 0])

    def test_invalid_poses(self):
        for pose in ([0]*5, [0]*5+[float("nan")]):
            with self.assertRaises(ValueError):
                rb_pose_to_transform(pose)
        with self.assertRaises(ValueError):
            from_row([2, 0, 0, 0, 0, 0, 0])
        with self.assertRaises(ValueError):
            inverse(np.diag([1, 1, -1, 1]))

    def test_session_export_pairing_and_overwrite(self):
        with tempfile.TemporaryDirectory() as folder:
            session = Path(folder)
            (session / "samples").mkdir()
            legacy_config = copy.deepcopy(CONFIG)
            legacy_config.pop("reference", None)
            legacy_config.pop("target", None)
            atomic_json(session / "session.json", {"config": legacy_config})
            for i in range(3):
                A = rb_pose_to_transform([i*100, 0, 0, 0, 0, 0])
                atomic_json(session / "samples" / f"{i:04d}.json",
                            {"T_base_tcp": A.tolist(), "T_camera_board": inverse(A).tolist()})
            output = export_csv(session)
            a = np.loadtxt(output / "tag_0_cam_0_A.csv", delimiter=",")
            b = np.loadtxt(output / "tag_0_cam_0_B.csv", delimiter=",")
            np.testing.assert_allclose(a[:, 4], [0, .1, .2])
            np.testing.assert_allclose(b[:, 4], [0, -.1, -.2])
            self.assertIn('mount = "eye_to_hand"', (output / "convention.toml").read_text())
            with self.assertRaises(FileExistsError):
                export_csv(session)

    def test_fixed_charuco_reference_chain_and_export(self):
        X = rb_pose_to_transform([10, 20, 50, 5, 10, 20])
        Y = rb_pose_to_transform([400, -100, 200, 15, 20, 30])
        C_ref = rb_pose_to_transform([100, 200, 600, 150, 10, 5])
        with tempfile.TemporaryDirectory() as folder:
            session = Path(folder)
            (session / "samples").mkdir()
            atomic_json(session / "session.json", {"schema_version": 2, "config": CONFIG})
            for i in range(8):
                A = rb_pose_to_transform([200+i*10, -200, 500, i*10, 30, 45])
                B = inverse(Y) @ A @ X
                C_marker = C_ref @ B
                a, b = measurement_pair(A, C_marker, "eye_to_hand", C_ref)
                np.testing.assert_allclose(a @ X, Y @ b, atol=1e-12)
                atomic_json(session / "samples" / f"{i:04d}.json", {
                    "T_base_tcp": A.tolist(), "T_camera_target": C_marker.tolist(),
                    "T_camera_reference": C_ref.tolist()})
            output = export_csv(session)
            rows = np.loadtxt(output / "tag_0_cam_0_B.csv", delimiter=",")
            np.testing.assert_allclose(from_row(rows[-1]), B, atol=1e-12)
            ref_rows = np.loadtxt(output / "T_camera_charuco.csv", delimiter=",")
            self.assertEqual(ref_rows.shape, (8, 7))
            self.assertIn('reference_type = "charuco"', (output / "convention.toml").read_text())
        with self.assertRaises(ValueError):
            measurement_pair(A, C_marker, "eye_in_hand", C_ref)


class CaptureTests(unittest.TestCase):
    def reader(self, **changes):
        state = SimpleNamespace(time=1., robot_state=1, real_vs_simulation_mode=0,
                                tcp_pos=np.zeros(6), jnt_ang=np.zeros(6))
        for key, value in changes.items():
            setattr(state, key, value)
        reader = RobotReader.__new__(RobotReader)
        reader.timeout = .1
        reader.fk = None
        reader.last_controller_time = None
        reader.channel = SimpleNamespace(request_data=lambda timeout: SimpleNamespace(sdata=state))
        return reader

    def test_stale_moving_simulation_timeout(self):
        reader = self.reader()
        reader.read()
        with self.assertRaises(ValueError):
            reader.read()
        for params in ({"robot_state": 3}, {"real_vs_simulation_mode": 1},
                       {"tcp_pos": np.full(6, np.nan)}):
            with self.assertRaises(ValueError):
                self.reader(**params).read()
        reader.channel.request_data = lambda timeout: None
        with self.assertRaises(TimeoutError):
            reader.read()

    def test_joint_motion_rejected_even_with_constant_reference_tcp(self):
        first = self.reader().read()
        second = self.reader(jnt_ang=[0, 1, 0, 0, 0, 0]).read()
        with self.assertRaises(ValueError):
            require_stationary(first, second, CONFIG["capture"])


class BoardTests(unittest.TestCase):
    def test_existing_board_custom_dictionary_and_ids(self):
        from hardware.rb5_realsense.board import make_board, make_reference
        config = copy.deepcopy(CONFIG)
        config["board"].update(dictionary="DICT_6X6_250", marker_ids=list(range(20, 37)))
        board = make_reference(config)
        np.testing.assert_array_equal(board.getIds().reshape(-1), np.arange(20, 37))
        config["target"]["id"] = 22
        with self.assertRaises(ValueError):
            make_reference(config)
        config["board"]["marker_ids"] = [0] * 17
        with self.assertRaises(ValueError):
            make_board(config["board"])

    def test_render_detect_metric_pose(self):
        import cv2
        if not hasattr(cv2.aruco, "CharucoDetector"):
            self.skipTest("Requires the specified OpenCV 4.11 installation")
        from hardware.rb5_realsense.board import make_board, estimate_board
        board = make_board(CONFIG["board"])
        image = np.full((900, 1100), 255, dtype=np.uint8)
        image[100:800, 300:800] = board.generateImage((500, 700))
        K = np.array([[1000., 0, 550], [0, 1000, 450], [0, 0, 1]])
        T, quality = estimate_board(image, board, K, np.zeros(5), CONFIG["capture"])
        # 150mm spans 500px at focal length 1000px => depth 300mm.
        self.assertAlmostEqual(T[2, 3], .3, delta=.002)
        self.assertLess(quality["reprojection_rms_px"], .3)
        self.assertGreater(quality["corner_count"], 15)
        with self.assertRaises(ValueError):
            estimate_board(np.full_like(image, 255), board, K, np.zeros(5), CONFIG["capture"])

    def test_single_aruco_pose_and_wrong_id(self):
        import cv2
        from hardware.rb5_realsense.board import ArucoTarget
        target = ArucoTarget(CONFIG["target"])
        K = np.array([[1000., 0, 550], [0, 1000, 450], [0, 0, 1]])
        R = Rotation.from_euler("xyz", [160, 10, 5], degrees=True).as_matrix()
        t = np.array([-.08, -.03, .4])
        points = cv2.projectPoints(target.objects, cv2.Rodrigues(R)[0], t, K, np.zeros(5))[0]
        bitmap = cv2.aruco.generateImageMarker(target.dictionary, target.marker_id, 800)
        homography = cv2.getPerspectiveTransform(
            np.float32([[0, 0], [799, 0], [799, 799], [0, 799]]), points.reshape(4, 2).astype(np.float32))
        image = cv2.warpPerspective(bitmap, homography, (1100, 900), borderValue=255)
        T, quality = target.estimate(image, K, np.zeros(5), CONFIG["capture"])
        self.assertEqual(quality["corner_count"], 4)  # min_corners=8 applies only to ChArUco
        np.testing.assert_allclose(T[:3, 3], t, atol=.003)
        self.assertLess(np.degrees(Rotation.from_matrix(R.T @ T[:3, :3]).magnitude()), 3)
        wrong = ArucoTarget({**CONFIG["target"], "id": target.marker_id + 1})
        with self.assertRaises(ValueError):
            wrong.estimate(image, K, np.zeros(5), CONFIG["capture"])

    def test_same_frame_reference_and_duplicate_markers(self):
        import cv2
        from hardware.rb5_realsense.board import make_target, make_reference, estimate_observations
        target = make_target(CONFIG)
        reference = make_reference(CONFIG)
        K = np.array([[1000., 0, 550], [0, 1000, 450], [0, 0, 1]])
        image = np.full((900, 1100), 255, dtype=np.uint8)
        image[100:800, 300:800] = reference.generateImage((500, 700))
        image[350:510, 100:260] = cv2.aruco.generateImageMarker(target.dictionary, target.marker_id, 160)
        marker_T, board_T, quality = estimate_observations(
            image, target, reference, K, np.zeros(5), CONFIG["capture"])
        self.assertAlmostEqual(marker_T[2, 3], .25, delta=.003)
        self.assertAlmostEqual(board_T[2, 3], .3, delta=.003)
        self.assertGreater(quality["reference"]["corner_count"], 15)
        image[100:800, 300:800] = 255
        with self.assertRaises(ValueError):
            estimate_observations(image, target, reference, K, np.zeros(5), CONFIG["capture"])
        image[350:510, 850:1010] = cv2.aruco.generateImageMarker(target.dictionary, target.marker_id, 160)
        with self.assertRaises(ValueError):
            target.estimate(image, K, np.zeros(5), CONFIG["capture"])
        duplicate_config = copy.deepcopy(CONFIG)
        duplicate_config["target"]["dictionary"] = "DICT_4X4_100"
        with self.assertRaises(ValueError):
            make_reference(duplicate_config)


if __name__ == "__main__":
    unittest.main()
