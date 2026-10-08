"""Only the RB data channel (port 5001) is used. No motion commands are sent."""
import socket
import time
import numpy as np
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

    def validate_controller_time(self, state):
        controller_time = float(state.time)
        if not np.isfinite(controller_time) or (self.last_controller_time is not None
                                                and controller_time <= self.last_controller_time):
            raise ValueError("Stale RB data or controller clock reset; reconnect")
        self.last_controller_time = controller_time
        return controller_time

    def read(self):
        started = time.monotonic()
        packet = self.channel.request_data(self.timeout)
        if packet is None:
            raise TimeoutError("RB data channel timed out")
        state = packet.sdata
        controller_time = self.validate_controller_time(state)
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


class ColorCamera:
    def __init__(self, config):
        import pyrealsense2 as rs
        self.rs = rs
        self.pipeline = rs.pipeline()
        stream = rs.config()
        if config["serial"]:
            stream.enable_device(config["serial"])
        elif len(rs.context().query_devices()) != 1:
            raise ValueError("Connect one RealSense or select camera.serial explicitly")
        stream.enable_stream(rs.stream.color, config["width"], config["height"],
                             rs.format.bgr8, config["fps"])
        profile = self.pipeline.start(stream)
        try:
            intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            if intr.model not in (rs.distortion.none, rs.distortion.brown_conrady):
                raise ValueError(f"Color distortion {intr.model} needs an explicit OpenCV "
                                 "intrinsic calibration; not valid as Brown coefficients")
            self.K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1.]])
            self.distortion = np.array(intr.coeffs) if intr.model != rs.distortion.none else np.zeros(5)
            device = profile.get_device()
            self.metadata = {"name": device.get_info(rs.camera_info.name),
                             "serial": device.get_info(rs.camera_info.serial_number),
                             "firmware": device.get_info(rs.camera_info.firmware_version),
                             "stream": "color_optical_frame", "K": self.K.tolist(),
                             "distortion": self.distortion.tolist(),
                             "distortion_model": str(intr.model),
                             "width": intr.width, "height": intr.height}
            self.last_frame = -1
            for _ in range(30):
                self.frame()
        except BaseException:
            self.pipeline.stop()
            raise

    def frame(self):
        frame = self.pipeline.wait_for_frames(3000).get_color_frame()
        if not frame or frame.get_frame_number() <= self.last_frame:
            raise ValueError("Missing or stale RGB frame")
        self.last_frame = frame.get_frame_number()
        return np.asanyarray(frame.get_data()).copy(), {
            "frame_number": self.last_frame,
            "camera_timestamp_ms": frame.get_timestamp(),
            "timestamp_domain": str(frame.get_frame_timestamp_domain()),
            "received_monotonic_s": time.monotonic()}

    def close(self):
        self.pipeline.stop()
