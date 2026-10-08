"""Subscribe to the patched rbpodo_ros2 measured-state topic; send no commands.

The ROS message uses rad and m, while the existing dataset uses deg and mm.
ROS imports stay inside the reader so offline image processing needs no ROS.
"""
import time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from hardware.rb5_realsense.devices import RobotReader
from .feedback import MESSAGE_TYPE, CLOCK_SOURCE, validate_feedback
from hardware.rb5_realsense.kinematics import RB5Kinematics


def state_from_ros(message):
    """Convert actual feedback, never jnt_ref/tcp_ref, to the existing SDK contract."""
    joints = np.asarray(message.jnt_ang, dtype=float)
    tcp = np.asarray(message.tcp_pos, dtype=float)
    if joints.shape != (6,) or tcp.shape != (6,):
        raise ValueError('ROS SystemState must contain six measured joints and six TCP values')
    return SimpleNamespace(
        time=float(message.time), jnt_ang=np.rad2deg(joints),
        tcp_pos=np.r_[tcp[:3] * 1000.0, np.rad2deg(tcp[3:])],
        robot_state=int(message.robot_state),
        real_vs_simulation_mode=int(message.real_vs_simulation_mode))


class ROS2RobotReader(RobotReader):
    """Use measured FK and idle/simulation checks plus driver receive freshness."""
    def __init__(self, config):
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
            from rbpodo_msgs.msg import MeasuredFeedback
        except ImportError as error:
            raise ImportError('Source scripts/ros2_env.sh after building the patched driver; '
                              'run with /usr/bin/python3') from error
        self.timeout = float(config['timeout_s'])
        if not np.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('Robot timeout must be positive and finite')
        self.topic = config.get('ros2', {}).get('state_topic', '/rbpodo_hardware/measured_feedback')
        if not isinstance(self.topic, str) or not self.topic.startswith('/'):
            raise ValueError('robot.ros2.state_topic must be an absolute ROS topic name')
        self.fk = None
        source = config.get('pose_source', 'urdf_fk')
        if source == 'urdf_fk':
            self.fk = RB5Kinematics(config['kinematics'])
        elif source != 'controller_tcp':
            raise ValueError('robot.pose_source must be urdf_fk or controller_tcp')
        self.metadata = {
            **(self.fk.metadata if self.fk else {'pose_source': 'controller_tcp'}),
            'transport': 'ros2', 'state_topic': self.topic,
            'message_type': MESSAGE_TYPE,
            'joint_source': 'rbpodo_msgs/msg/SystemState.jnt_ang',
            'source_units': 'metres/radians', 'record_units': 'millimetres/degrees',
            'clock_source': CLOCK_SOURCE,
            'controller_time_note': 'raw SystemState.time retained; not used for freshness',
            'freshness_note': 'same Linux boot; increasing response sequence and monotonic receipt; does not prove controller-internal sample age'}
        self.last_controller_time = None
        self._feedback = None
        self.boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self.max_feedback_age_s = float(config.get('ros2', {}).get('max_feedback_age_s', .2))
        if not np.isfinite(self.max_feedback_age_s) or not 0 < self.max_feedback_age_s <= self.timeout:
            raise ValueError('max_feedback_age_s must be positive and at most timeout_s')
        self._pending = None
        self.context = Context()
        self.node = None
        self.executor = None
        rclpy.init(args=[], context=self.context)
        try:
            self.node = rclpy.create_node('rwhe_gopro_robot_reader', context=self.context,
                                         use_global_arguments=False, start_parameter_services=False)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
            self.subscription = self.node.create_subscription(MeasuredFeedback, self.topic, self._receive, qos)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            # RobotReader.read calls request_data and applies its existing checks.
            self.channel = self
        except BaseException:
            self.close()
            raise

    def _receive(self, message):
        self._pending = message

    def request_data(self, timeout):
        self._pending = None
        deadline = time.monotonic() + timeout
        while self.context.ok():
            if self.node.count_publishers(self.topic) > 1:
                raise ValueError('Multiple publishers on measured-state topic; select exactly one robot')
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f'No fresh measured state on {self.topic}; check patched driver, ROS_DOMAIN_ID and source environment')
            self.executor.spin_once(timeout_sec=min(remaining, .1))
            if self._pending is not None:
                message = self._pending
                feedback = {
                    'feedback_stream_id': message.stream_id,
                    'feedback_sequence': int(message.sequence),
                    'feedback_received_monotonic_ns': int(message.received_monotonic_ns),
                }
                validate_feedback(feedback, self._feedback, time.monotonic_ns(),
                                  self.max_feedback_age_s, self.boot_id)
                self._feedback = feedback
                return SimpleNamespace(sdata=state_from_ros(message.state))
        raise RuntimeError('ROS context stopped while recording robot feedback')

    def validate_controller_time(self, state):
        # CB 25040302 [7.4.2] was observed to return a constant subnormal timer.
        # Freshness is checked using the envelope, never a synthetic controller time.
        stamp = float(state.time)
        if not np.isfinite(stamp):
            raise ValueError('Non-finite raw controller time')
        return stamp

    def read(self):
        reading = super().read()
        if self._feedback is None:
            raise ValueError('Missing driver feedback envelope')
        return {**reading, **self._feedback, 'transport': 'ros2', 'state_topic': self.topic}

    def close(self):
        if self.executor is not None:
            self.executor.shutdown(timeout_sec=1.0)
            self.executor = None
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if self.context.ok():
            self.context.shutdown()
