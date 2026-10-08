import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from hardware.rb5_gopro.camera import GoProCamera, sha256
from hardware.rb5_gopro.workflow import (
    checked_config, import_images, load_manifest, load_robot_record, record_pose, compact_robot_record)
from hardware.rb5_realsense.board import make_board
from hardware.rb5_realsense.geometry import inverse, from_row
from hardware.rb5_realsense.kinematics import RB5Kinematics

CONFIG = checked_config('hardware/rb5.gopro.json')


def robot_record(config, sample_id, joints=None, T=None):
    fk = RB5Kinematics(config['robot']['kinematics']) if config['robot']['pose_source'] == 'urdf_fk' else None
    joints = [0, 20, 30, 10, 50, 0] if joints is None else joints
    T = fk.forward_degrees(joints) if fk else T
    tcp = np.r_[T[:3, 3]*1000, Rotation.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)].tolist()
    readings = [{'T': T.tolist(), 'joints_deg': list(joints), 'tcp_mm_deg': tcp,
                 'pose_source': config['robot']['pose_source'], 'controller_time_s': float(t)} for t in (1, 2)]
    return {'schema': 'rb5-gopro-robot/v1', 'sample_id': sample_id, 'robot_config': config['robot'],
            'robot_pose': fk.metadata if fk else {'pose_source': 'controller_tcp'},
            'T_base_hand': T.tolist(), 'readings': readings}


def write_json(path, data):
    Path(path).write_text(json.dumps(data))


def render_fisheye(board, camera, T):
    width, height = camera.size
    grid = np.indices((height, width), dtype=np.float64)[::-1].transpose(1, 2, 0).reshape(-1, 1, 2)
    normalized = cv2.fisheye.undistortPoints(grid, camera.K, camera.D).reshape(-1, 2)
    rays = np.c_[normalized, np.ones(len(normalized))]
    normal = T[:3, 2]
    depth = np.dot(normal, T[:3, 3]) / (rays @ normal)
    points = (depth[:, None]*rays - T[:3, 3]) @ T[:3, :3]
    bitmap = board.generateImage((1400, 900))
    maps = (points[:, :2] / [0.56, 0.36] * [1400, 900]).astype(np.float32)
    return cv2.remap(bitmap, maps[:, 0].reshape(height, width), maps[:, 1].reshape(height, width),
                     cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=255)


class GoProTests(unittest.TestCase):
    def test_calibration_hash_and_board_codes(self):
        camera = GoProCamera(CONFIG['camera'])
        self.assertEqual(camera.size, [5568, 4872])
        self.assertAlmostEqual(camera.metadata['validation_rms_px'], .8724809809368699)
        board = make_board(CONFIG['board'])
        spec = camera.calibration['board']
        family = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        sliced = cv2.aruco.Dictionary(family.bytesList[spec['dictionary_offset']:].copy(),
                                      family.markerSize, family.maxCorrectionBits)
        original = cv2.aruco.CharucoBoard((14, 9), .04, .024, sliced)
        original.setLegacyPattern(True)
        np.testing.assert_array_equal(board.generateImage((1400, 900)), original.generateImage((1400, 900)))

    def test_fisheye_metric_pose_at_center_and_edge(self):
        camera = GoProCamera(CONFIG['camera'])
        objects = make_board(CONFIG['board']).getChessboardCorners().astype(float).reshape(-1, 1, 3)
        for translation in ([-.25, -.15, .75], [.65, -.2, .65]):
            p = np.r_[[.35, -.25, .12], translation]
            pixels = camera.project(objects, p)
            T, quality = camera.solve(objects, pixels, CONFIG['capture'])
            np.testing.assert_allclose(T[:3, 3], p[3:], atol=1e-7)
            np.testing.assert_allclose(T[:3, :3], Rotation.from_rotvec(p[:3]).as_matrix(), atol=1e-7)
            self.assertLess(quality['reprojection_rms_px'], 1e-6)
        noisy = pixels.copy()
        noisy[0, 0] += 100
        with self.assertRaisesRegex(ValueError, 'reprojection'):
            camera.solve(objects, noisy, CONFIG['capture'])

    def test_size_and_calibration_mismatch(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            camera = GoProCamera(CONFIG['camera'])
            image = folder/'resized.png'
            cv2.imwrite(str(image), np.zeros((100, 200), dtype=np.uint8))
            with self.assertRaisesRegex(ValueError, 'original'):
                camera.read(image)
            c = copy.deepcopy(camera.calibration)
            c['K'][0][0] += 1
            path = folder/'changed.json'
            write_json(path, c)
            with self.assertRaisesRegex(ValueError, 'SHA256'):
                GoProCamera({**CONFIG['camera'], 'intrinsics_path': str(path)})
            c['model'] = 'pinhole'
            write_json(path, c)
            with self.assertRaisesRegex(ValueError, 'fisheye'):
                GoProCamera({**CONFIG['camera'], 'intrinsics_path': str(path)})

    def test_robot_record_integrity_and_stationarity(self):
        fk = RB5Kinematics(CONFIG['robot']['kinematics'])
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'robot.json'
            good = robot_record(CONFIG, 'pose01')
            write_json(path, good)
            T, _ = load_robot_record(path, 'pose01', CONFIG, fk)
            np.testing.assert_allclose(T, good['T_base_hand'])
            with self.assertRaisesRegex(ValueError, 'sample_id'):
                load_robot_record(path, 'pose02', CONFIG, fk)
            for change in ('stale', 'moving', 'model', 'source', 'transform'):
                bad = copy.deepcopy(good)
                if change == 'stale':
                    bad['readings'][1]['controller_time_s'] = 1
                elif change == 'moving':
                    bad['readings'][1]['joints_deg'][0] += 2
                    bad['readings'][1]['T'] = fk.forward_degrees(bad['readings'][1]['joints_deg']).tolist()
                elif change == 'model':
                    bad['robot_pose']['tip_link'] = 'wrong_tip'
                elif change == 'source':
                    bad['robot_pose']['pose_source'] = 'controller_tcp'
                else:
                    bad['T_base_hand'][0][3] += .01
                write_json(path, bad)
                with self.assertRaises(ValueError, msg=change):
                    load_robot_record(path, 'pose01', CONFIG, fk)

    def test_compact_record_retains_first_pose_and_rejects_invalid_summary(self):
        fk = RB5Kinematics(CONFIG['robot']['kinematics'])
        legacy = robot_record(CONFIG, 'p01')
        compact = compact_robot_record(legacy, CONFIG, fk)
        self.assertNotIn('readings', compact)
        self.assertEqual(compact['T_base_hand'], legacy['T_base_hand'])
        self.assertEqual(compact['representative']['joints_deg'], legacy['readings'][0]['joints_deg'])
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'robot.json'
            write_json(path, compact)
            T, _ = load_robot_record(path, 'p01', CONFIG, fk)
            np.testing.assert_allclose(T, legacy['T_base_hand'])
            for key, value in [('max_joint_delta_deg', 3), ('max_translation_m', float('nan')),
                               ('sample_count', 1)]:
                bad = copy.deepcopy(compact)
                bad['stationary_window'][key] = value
                write_json(path, bad)
                with self.assertRaises(ValueError):
                    load_robot_record(path, 'p01', CONFIG, fk)
            bad = copy.deepcopy(compact)
            bad['representative']['joints_deg'][0] += 2
            write_json(path, bad)
            with self.assertRaisesRegex(ValueError, 'transform'):
                load_robot_record(path, 'p01', CONFIG, fk)
        moving = copy.deepcopy(legacy)
        moving['readings'][1]['joints_deg'][0] += 3
        moving['readings'][1]['T'] = fk.forward_degrees(moving['readings'][1]['joints_deg']).tolist()
        with self.assertRaisesRegex(ValueError, 'moved'):
            compact_robot_record(moving, CONFIG, fk)

    def test_record_pose_reads_stationary_window_without_camera_sdk(self):
        config = copy.deepcopy(CONFIG)
        config['robot']['kinematics']['model_matches_robot'] = True
        fk = RB5Kinematics(config['robot']['kinematics'])
        record = robot_record(config, 'p01')
        class FakeRobot:
            metadata = fk.metadata
            count = 0
            def read(self):
                self.count += 1
                return {**copy.deepcopy(record['readings'][0]),
                        'T': np.asarray(record['T_base_hand']), 'controller_time_s': self.count}
        robot = FakeRobot()
        robot.fk = fk
        with tempfile.TemporaryDirectory() as folder, patch('hardware.rb5_gopro.workflow.make_robot_reader', return_value=robot):
            path = Path(folder)/'robot.json'
            config['capture']['settle_s'] = .001
            record_pose(config, 'p01', path, duration_s=.001)
            T, saved = load_robot_record(path, 'p01', config, fk)
            self.assertNotIn('readings', saved)
            self.assertEqual(saved['schema'], 'rb5-gopro-robot/v2')
            self.assertNotIn('T', saved['representative'])
            self.assertGreaterEqual(saved['stationary_window']['sample_count'], 2)
            np.testing.assert_allclose(T, record['T_base_hand'])
            with self.assertRaises(ValueError):
                record_pose(config, 'p01', path)

    def test_rendered_photo_to_paired_csv(self):
        config = copy.deepcopy(CONFIG)
        config['robot'].update(pose_source='controller_tcp', tcp_description='Test TCP, base reference')
        config['board']['dimensions_measured'] = True
        board = make_board(config['board'])
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            calibration = copy.deepcopy(GoProCamera(config['camera']).calibration)
            calibration.update(image_size=[800, 600], K=[[450., 0, 400], [0, 450., 300], [0, 0, 1]])
            intrinsic_path = folder/'camera.json'
            write_json(intrinsic_path, calibration)
            validation_path = folder/'validation.json'
            write_json(validation_path, {'calibration_sha256': sha256(intrinsic_path),
                                         'quality_status': 'PASS', 'overall': {'rms_px': 0}})
            config['camera'].update(intrinsics_path=str(intrinsic_path), validation_path=str(validation_path))
            camera = GoProCamera(config['camera'])
            entries, expected = [], []
            for i in range(3):
                C = np.eye(4)
                C[:3, :3] = Rotation.from_rotvec([.3+i*.07, -.2+i*.1, .05]).as_matrix()
                C[:3, 3] = [-.25+i*.015, -.16, .65+i*.02]
                photo = folder/f'photo{i}.png'
                cv2.imwrite(str(photo), render_fisheye(board, camera, C))
                A = inverse(C)  # Synthetic eye-in-hand X=Y=I => A=B=inverse(C).
                record = robot_record(config, f'p{i}', T=A)
                if i != 1:  # Mixed legacy/compact inputs must keep identical CSV pairing.
                    record = compact_robot_record(record, config, None)
                write_json(folder/f'robot{i}.json', record)
                entries.append({'sample_id': f'p{i}', 'image': photo.name, 'robot_record': f'robot{i}.json'})
                expected.append(A)
            manifest = folder/'manifest.json'
            write_json(manifest, {'samples': entries[::-1]})
            output = import_images(config, manifest, folder/'session')
            rows_a = np.loadtxt(output/'tag_0_cam_0_A.csv', delimiter=',')
            rows_b = np.loadtxt(output/'tag_0_cam_0_B.csv', delimiter=',')
            self.assertEqual(rows_a.shape, (3, 7))
            for a, b, expected_a in zip(rows_a, rows_b, expected[::-1]):
                np.testing.assert_allclose(from_row(a), expected_a, atol=1e-10)
                np.testing.assert_allclose(from_row(b), expected_a, atol=.006)
            self.assertEqual(json.loads((output/'sample_ids.json').read_text()), ['p2', 'p1', 'p0'])
            self.assertIn('mount = "eye_in_hand"', (output/'convention.toml').read_text())
            with self.assertRaisesRegex(ValueError, 'new session'):
                import_images(config, manifest, folder/'session')
            write_json(manifest, {'samples': [entries[0], entries[0]]})
            with self.assertRaisesRegex(ValueError, 'Duplicate'):
                load_manifest(manifest)
            write_json(manifest, {'samples': []})
            with self.assertRaisesRegex(ValueError, 'no samples'):
                load_manifest(manifest)


if __name__ == '__main__':
    unittest.main()
