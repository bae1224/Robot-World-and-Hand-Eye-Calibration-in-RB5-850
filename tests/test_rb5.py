import json
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

CONFIG = json.loads(Path("hardware/rb5.example.json").read_text())


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
            atomic_json(session / "session.json", {"config": CONFIG})
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


class CaptureTests(unittest.TestCase):
    def reader(self, **changes):
        state = SimpleNamespace(time=1., robot_state=1, real_vs_simulation_mode=0,
                                tcp_pos=np.zeros(6), jnt_ang=np.zeros(6))
        for key, value in changes.items():
            setattr(state, key, value)
        reader = RobotReader.__new__(RobotReader)
        reader.timeout = .1
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


if __name__ == "__main__":
    unittest.main()
