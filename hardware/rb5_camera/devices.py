"""Only the RB data channel (port 5001) is used. No motion commands are sent.

Camera acquisition moved to cameras.py when the RealSense SDK stopped being the
only way intrinsics could arrive; it is re-exported here for existing callers.
"""
import socket
import time
import numpy as np
from .cameras import RealSenseCamera, UVCCamera, make_camera  # noqa: F401
from .geometry import rb_pose_to_transform, pose_distance


class RobotReader:
    def __init__(self, config):
        import rbpodo
        self.fk = None
        source = config.get("pose_source", "controller_tcp")
        if source == "urdf_fk":
            from .kinematics import RB5Kinematics
            self.fk = RB5Kinematics(config["kinematics"])
        elif source != "controller_tcp":
            raise ValueError("robot.pose_source must be urdf_fk or controller_tcp")
        self.metadata = self.fk.metadata if self.fk else {"pose_source": "controller_tcp"}
        if not config["ip"]:
            raise ValueError("Set robot.ip in your local configuration")
        self.timeout = config["timeout_s"]
        # The SDK constructor's connect has no timeout argument.
        with socket.create_connection((config["ip"], config["port"]), self.timeout):
            pass
        self.channel = rbpodo.CobotData(config["ip"], config["port"])
        self.last_controller_time = None

    def read(self):
        started = time.monotonic()
        packet = self.channel.request_data(self.timeout)
        if packet is None:
            raise TimeoutError("RB data channel timed out")
        state = packet.sdata
        controller_time = float(state.time)
        if not np.isfinite(controller_time) or (self.last_controller_time is not None
                                                and controller_time <= self.last_controller_time):
            raise ValueError("Stale RB data or controller clock reset; reconnect")
        self.last_controller_time = controller_time
        if state.robot_state != 1:
            raise ValueError("Robot is not idle")
        if state.real_vs_simulation_mode != 0:
            raise ValueError("Controller is in simulation mode")
        pose = np.asarray(state.tcp_pos, dtype=float).reshape(6).copy()
        joints = np.asarray(state.jnt_ang, dtype=float).reshape(6).copy()
        if not np.isfinite(joints).all():
            raise ValueError("Non-finite joint feedback")
        T = self.fk.forward_degrees(joints) if self.fk else rb_pose_to_transform(pose)
        return {"T": T, "tcp_mm_deg": pose.tolist() if np.isfinite(pose).all() else None,
                "joints_deg": joints.tolist(),
                "pose_source": "urdf_fk" if self.fk else "controller_tcp",
                "controller_time_s": controller_time, "request_start_monotonic_s": started,
                "received_monotonic_s": time.monotonic()}


def require_stationary(reference, current, config):
    distance, angle = pose_distance(reference["T"], current["T"])
    joint_delta = np.max(np.abs(np.asarray(reference["joints_deg"])
                               - current["joints_deg"]))
    if (distance > config["max_motion_m"] or angle > config["max_motion_deg"]
            or joint_delta > config["max_joint_motion_deg"]):
        raise ValueError("Robot moved during the stationary capture window")
