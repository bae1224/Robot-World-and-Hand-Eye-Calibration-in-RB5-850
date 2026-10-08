"""Real ROS/DDS transport tests with synthetic feedback on unique localhost topics.

Run after sourcing Humble and the locally built rbpodo_msgs package. No robot connection.
"""
import copy
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
import uuid
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rbpodo_msgs.msg import SystemState, MeasuredFeedback
from hardware.rb5_gopro.ros2_robot import ROS2RobotReader
from hardware.rb5_gopro.workflow import record_pose, load_robot_record
from hardware.rb5_realsense.kinematics import RB5Kinematics

CONFIG = json.loads(Path('hardware/rb5.gopro.json').read_text())


class SyntheticPublisher:
    def __init__(self, topic):
        self.context = Context()
        rclpy.init(args=[], context=self.context)
        self.node = rclpy.create_node('rwhe_test_publisher', context=self.context)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.node.create_publisher(MeasuredFeedback, topic, qos)
        self.stop = threading.Event()
        self.simulation = False
        self.moving = False
        self.fixed_time = 3.0729074024178914e-41
        self.sequence = 0
        self.freeze_sequence = False
        self.delay_ns = 0
        self.stream_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip() + ':synthetic'

        self.thread = threading.Thread(target=self.publish_loop)
        self.thread.start()

    def publish_loop(self):
        started = time.monotonic()
        while not self.stop.is_set():
            msg = SystemState()
            msg.time = float(self.fixed_time if self.fixed_time is not None else 10 + time.monotonic()-started)
            msg.jnt_ang = np.deg2rad([0, 20, 30, 10, 50, 0]).astype(np.float32).tolist()
            msg.jnt_ref = [9.0]*6  # Deliberately different from real feedback.
            msg.tcp_pos = [.1, .2, .3, .1, .2, .3]
            msg.robot_state = 3 if self.moving else 1
            msg.real_vs_simulation_mode = self.simulation
            if not self.freeze_sequence:
                self.sequence += 1
            envelope = MeasuredFeedback()
            envelope.state = msg
            envelope.stream_id = self.stream_id
            envelope.sequence = self.sequence
            envelope.received_monotonic_ns = time.monotonic_ns() - self.delay_ns
            self.publisher.publish(envelope)
            self.stop.wait(.02)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)
        self.node.destroy_node()
        self.context.shutdown()


class FeedbackIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.config = copy.deepcopy(CONFIG)
        self.config['robot']['timeout_s'] = 3
        self.config['robot']['kinematics']['model_matches_robot'] = True
        self.config['robot']['ros2']['state_topic'] = '/rwhe_test_' + uuid.uuid4().hex
        self.publisher = SyntheticPublisher(self.config['robot']['ros2']['state_topic'])
        self.reader = ROS2RobotReader(self.config['robot'])

    def tearDown(self):
        self.reader.close()
        self.publisher.close()

    def test_actual_dds_measured_joints_and_timeout(self):
        data = self.reader.read()
        np.testing.assert_allclose(data['joints_deg'], [0, 20, 30, 10, 50, 0], atol=1e-5)
        later = self.reader.read()
        self.assertGreater(later['feedback_sequence'], data['feedback_sequence'])
        self.assertEqual(later['controller_time_s'], data['controller_time_s'])
        self.publisher.stop.set()
        self.publisher.thread.join(timeout=2)
        # There can be one queued sample. Drain it, then require a timeout.
        self.reader.timeout = .15
        try:
            self.reader.read()
        except TimeoutError:
            pass
        with self.assertRaises(TimeoutError):
            self.reader.read()

    def test_simulation_and_stale_feedback_rejected(self):
        self.publisher.simulation = True
        with self.assertRaisesRegex(ValueError, 'simulation'):
            self.reader.read()
        self.publisher.simulation = False
        # First consume a new real-mode response; simulation rejection also advances
        # the transport sequence, so freezing immediately could repeat that sample.
        for _ in range(10):
            try:
                self.reader.read()
                break
            except ValueError:
                continue
        else:
            self.fail('No real-mode feedback received')
        self.publisher.freeze_sequence = True
        for _ in range(10):
            try:
                self.reader.read()
            except ValueError as error:
                self.assertIn('Stale', str(error))
                break
        else:
            self.fail('Repeated response sequence was not rejected')

    def test_delayed_feedback_is_rejected_over_dds(self):
        self.publisher.delay_ns = 2_000_000_000
        for _ in range(10):
            try:
                self.reader.read()
            except ValueError as error:
                self.assertIn('Stale', str(error))
                break
        else:
            self.fail('Delayed response was accepted')

    def test_go_pro_record_and_offline_fk_revalidation(self):
        self.reader.close()  # record_pose owns its own reader/context.
        self.config['capture']['settle_s'] = .08
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'synthetic_test_only.json'
            record_pose(self.config, 'synthetic_test_only', path, duration_s=.12)
            fk = RB5Kinematics(self.config['robot']['kinematics'])
            T, record = load_robot_record(path, 'synthetic_test_only', self.config, fk)
            np.testing.assert_allclose(T, fk.forward_degrees([0, 20, 30, 10, 50, 0]), atol=1e-7)
            self.assertEqual(record['robot_pose']['transport'], 'ros2')
            self.assertEqual(record['robot_pose']['clock_source'], 'driver_receive_monotonic')
            self.assertNotIn('readings', record)
            self.assertEqual(record['schema'], 'rb5-gopro-robot/v2')
            self.assertEqual(record['representative']['controller_time_s'],
                             record['stationary_window']['last_observation']['controller_time_s'])
            tampered = copy.deepcopy(record)
            tampered['stationary_window']['last_observation']['feedback_sequence'] = record['representative']['feedback_sequence']
            path.write_text(json.dumps(tampered))
            with self.assertRaisesRegex(ValueError, 'Stale'):
                load_robot_record(path, 'synthetic_test_only', self.config, fk)
            self.assertIn('SystemState.jnt_ang', record['robot_pose']['joint_source'])
            self.assertGreaterEqual(record['stationary_window']['sample_count'], 3)


if __name__ == '__main__':
    unittest.main(verbosity=2)
