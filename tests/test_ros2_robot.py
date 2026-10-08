import copy
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import time
import unittest
from unittest.mock import patch
import numpy as np
from hardware.rb5_gopro.feedback import validate_feedback
from hardware.rb5_gopro.ros2_robot import ROS2RobotReader, state_from_ros
from hardware.rb5_gopro.workflow import make_robot_reader, record_pose
from hardware.rb5_realsense.geometry import rb_pose_to_transform
from hardware.rb5_realsense.kinematics import RB5Kinematics

CONFIG = json.loads(Path('hardware/rb5.gopro.json').read_text())


def message(**changes):
    values = dict(time=1.0, jnt_ang=np.deg2rad([10, 20, 30, 40, 50, 60]),
                  jnt_ref=np.ones(6)*999, tcp_pos=[.1, -.2, .3, .2, -.4, .7],
                  tcp_ref=np.ones(6)*999, robot_state=1, real_vs_simulation_mode=False)
    values.update(changes)
    return SimpleNamespace(**values)


def offline_reader(msg, fk=True):
    reader = ROS2RobotReader.__new__(ROS2RobotReader)
    reader.fk = RB5Kinematics(CONFIG['robot']['kinematics']) if fk else None
    reader.timeout = .1
    reader.topic = '/test/measured_state'
    reader.last_controller_time = None
    reader._feedback = {'feedback_stream_id': 'test:1', 'feedback_sequence': 1,
                        'feedback_received_monotonic_ns': time.monotonic_ns()}
    reader.channel = SimpleNamespace(request_data=lambda timeout: SimpleNamespace(sdata=state_from_ros(msg)))
    return reader


class ROS2ConversionTests(unittest.TestCase):
    def test_actual_joints_rad_to_degrees_and_fk(self):
        reader = offline_reader(message())
        reading = reader.read()
        np.testing.assert_allclose(reading['joints_deg'], [10, 20, 30, 40, 50, 60])
        np.testing.assert_allclose(reading['T'], reader.fk.forward_degrees([10, 20, 30, 40, 50, 60]))
        self.assertEqual(reading['transport'], 'ros2')

    def test_measured_tcp_metres_radians(self):
        reading = offline_reader(message(), fk=False).read()
        expected = rb_pose_to_transform([100, -200, 300, *np.rad2deg([.2, -.4, .7])])
        np.testing.assert_allclose(reading['T'], expected)
        np.testing.assert_allclose(reading['tcp_mm_deg'][:3], [100, -200, 300])

    def test_reject_stale_reset_simulation_moving_and_nonfinite(self):
        msg = message()
        reader = offline_reader(msg)
        reader.read()
        # A constant or reset raw timer is permitted; envelope freshness is separate.
        reader.read()
        msg.time = 3.0729074024178914e-41
        self.assertEqual(reader.read()['controller_time_s'], msg.time)
        for changes in ({'robot_state': 3}, {'real_vs_simulation_mode': True},
                        {'jnt_ang': [float('nan')]*6}, {'time': float('nan')}):
            with self.assertRaises(ValueError):
                offline_reader(message(**changes)).read()
        with self.assertRaises(ValueError):
            state_from_ros(message(jnt_ang=[0]*5))
        with self.assertRaises(ValueError):
            offline_reader(message(tcp_pos=[float('nan')]*6), fk=False).read()

    def test_receive_freshness_rejects_repeat_delay_restart_and_wrong_host(self):
        first = {'feedback_stream_id': 'boot:instance', 'feedback_sequence': 4,
                 'feedback_received_monotonic_ns': 1_000_000_000}
        validate_feedback(first, None, 1_020_000_000, boot_id='boot')
        fresh = {**first, 'feedback_sequence': 5, 'feedback_received_monotonic_ns': 1_020_000_000}
        validate_feedback(fresh, first, 1_030_000_000, boot_id='boot')
        for changes, now, boot in [({}, 1_030_000_000, 'boot'),
                                   ({'feedback_sequence': 3}, 1_030_000_000, 'boot'),
                                   ({'feedback_stream_id': 'boot:restart'}, 1_030_000_000, 'boot'),
                                   ({'feedback_sequence': 5}, 2_000_000_000, 'boot'),
                                   ({'feedback_sequence': 5}, 900_000_000, 'boot'),
                                   ({'feedback_sequence': 5}, 1_030_000_000, 'other'),
                                   ({'feedback_sequence': True}, 1_030_000_000, 'boot')]:
            with self.subTest(changes=changes, now=now, boot=boot), self.assertRaises(ValueError):
                validate_feedback({**first, **changes}, first, now, boot_id=boot)

    def test_backend_selection_and_legacy_default(self):
        with patch('hardware.rb5_gopro.ros2_robot.ROS2RobotReader') as ros:
            make_robot_reader(CONFIG['robot'])
            ros.assert_called_once_with(CONFIG['robot'])
        config = copy.deepcopy(CONFIG['robot'])
        config.pop('transport')
        with patch('hardware.rb5_gopro.workflow.RobotReader') as sdk:
            make_robot_reader(config)
            sdk.assert_called_once_with(config)
        with self.assertRaises(ValueError):
            make_robot_reader({**config, 'transport': 'typo'})

    def test_recording_closes_ros_reader_on_failure(self):
        config = copy.deepcopy(CONFIG)
        config['robot']['kinematics']['model_matches_robot'] = True
        with tempfile.TemporaryDirectory() as folder, patch('hardware.rb5_gopro.workflow.make_robot_reader') as factory:
            factory.return_value.read.side_effect = TimeoutError('no feedback')
            path = Path(folder)/'missing.json'
            with self.assertRaises(TimeoutError):
                record_pose(config, 'p01', path)
            factory.return_value.close.assert_called_once()
            self.assertFalse(path.exists())


if __name__ == '__main__':
    unittest.main()
