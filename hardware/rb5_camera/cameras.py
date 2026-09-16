"""Camera acquisition backends.

``realsense``  Intel RealSense over pyrealsense2; intrinsics come from the SDK.
``uvc``        Any UVC video device, which is how a GoPro HERO13 appears behind
               an HDMI capture box. The box carries no intrinsics and no frame
               metadata, so intrinsics are loaded from a calibration file and
               staleness is handled by draining the driver queue.

Both expose the same surface: ``.model`` (an optics.CameraModel), ``.metadata``,
``.frame()`` and ``.close()``.
"""
import time
import cv2
import numpy as np
from .optics import CameraModel, load_intrinsics

# A GoPro applies these in-camera. Each one re-crops or rotates the image per
# frame, which makes T_hand_camera time-varying and silently breaks AX=YB while
# every reprojection and residual check still looks healthy.
GOPRO_REQUIRED_OFF = ("stabilization_off", "horizon_lock_off")


class RealSenseCamera:
    def __init__(self, config):
        import pyrealsense2 as rs
        self.rs = rs
        self.pipeline = rs.pipeline()
        stream = rs.config()
        if config.get("serial"):
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
            K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1.]])
            coefficients = np.array(intr.coeffs) if intr.model != rs.distortion.none else np.zeros(5)
            device = profile.get_device()
            self.model = CameraModel(K, coefficients, intr.width, intr.height, "pinhole", {
                "backend": "realsense", "intrinsics_source": "realsense_sdk",
                "name": device.get_info(rs.camera_info.name),
                "serial": device.get_info(rs.camera_info.serial_number),
                "firmware": device.get_info(rs.camera_info.firmware_version),
                "stream": "color_optical_frame",
                "distortion_model": str(intr.model)})
            self.metadata = self.model.to_dict()
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


class UVCCamera:
    """GoPro HERO13 (or any camera) through an HDMI capture box.

    The capture path gives us no frame numbers, no device timestamps and no
    intrinsics, so this backend compensates for each in turn:

    * intrinsics are loaded from ``camera.intrinsics`` and checked against the
      negotiated stream size, because a calibration is only valid at the
      resolution and FOV mode it was captured at;
    * the negotiated format/size is verified rather than assumed, since capture
      boxes silently fall back to another mode;
    * every capture drains ``drain_frames`` buffered frames before reading, which
      is the only way to know the image is not a stale one from before the robot
      arrived at the pose.
    """

    def __init__(self, config):
        source = config.get("device", 0)
        backend = {"any": cv2.CAP_ANY, "msmf": cv2.CAP_MSMF, "dshow": cv2.CAP_DSHOW,
                   "v4l2": cv2.CAP_V4L2, "avfoundation": cv2.CAP_AVFOUNDATION}[
                       config.get("api", "any")]
        self.capture = cv2.VideoCapture(source, backend)
        if not self.capture.isOpened():
            raise ValueError(f"Cannot open capture device {source!r}. Check the capture box is "
                             "enumerated and no other application holds it open.")
        try:
            fourcc = config.get("fourcc", "MJPG")
            if fourcc:
                self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, config["width"])
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config["height"])
            self.capture.set(cv2.CAP_PROP_FPS, config["fps"])
            # Shrink the driver queue where the backend honours it; drain_frames
            # below is what actually guarantees freshness.
            self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.drain = int(config.get("drain_frames", 5))
            if self.drain < 1:
                raise ValueError("camera.drain_frames must be at least 1")
            ok, probe = self.capture.read()
            if not ok or probe is None:
                raise ValueError("Capture device opened but delivered no frame")
            height, width = probe.shape[:2]
            if (width, height) != (config["width"], config["height"]):
                raise ValueError(f"Capture box negotiated {width}x{height}, not the configured "
                                 f"{config['width']}x{config['height']}. Intrinsics are tied to "
                                 "the exact stream size; fix the mode or recalibrate.")
            if not config.get("intrinsics"):
                raise ValueError("A capture box provides no intrinsics. Set camera.intrinsics to a "
                                 "calibration file (python -m hardware.rb5_camera intrinsics ...)")
            self.model = load_intrinsics(config["intrinsics"], (width, height))
            require_gopro_acknowledgements(config)
            self.model.metadata.update({
                "backend": "uvc", "intrinsics_source": "file", "device": str(source),
                "api": config.get("api", "any"), "fourcc": fourcc,
                "negotiated_fps": float(self.capture.get(cv2.CAP_PROP_FPS)),
                "drain_frames": self.drain,
                "camera_description": config.get("description", ""),
                "acknowledgements": sorted(config.get("acknowledgements", []))})
            self.metadata = self.model.to_dict()
            self.count = 0
        except BaseException:
            self.capture.release()
            raise

    def frame(self):
        # grab() decodes nothing, so this discards the queue cheaply; the final
        # read() is the only frame that is actually decoded and returned.
        for _ in range(self.drain):
            if not self.capture.grab():
                raise ValueError("Capture device stopped delivering frames")
        ok, image = self.capture.read()
        if not ok or image is None:
            raise ValueError("Capture device returned an empty frame")
        if image.shape[:2] != (self.model.height, self.model.width):
            raise ValueError("Capture frame size changed mid-session; intrinsics are invalid")
        self.count += 1
        return image.copy(), {"frame_number": self.count, "drained": self.drain,
                              "camera_timestamp_ms": float(self.capture.get(cv2.CAP_PROP_POS_MSEC)),
                              "timestamp_domain": "capture_box_uncalibrated",
                              "received_monotonic_s": time.monotonic()}

    def close(self):
        self.capture.release()


def require_gopro_acknowledgements(config):
    """Refuse to collect until the operator has confirmed the in-camera processing is off.

    This is not bureaucracy. HyperSmooth and Horizon Lock re-crop and rotate each
    frame independently, so the hand-to-camera transform stops being rigid. The
    residuals, the reprojection error and the optimality certificate all stay
    clean while the answer is wrong, which makes it the one failure this pipeline
    cannot detect after the fact.
    """
    if not config.get("is_gopro", False):
        return
    acknowledged = set(config.get("acknowledgements", []))
    missing = [name for name in GOPRO_REQUIRED_OFF if name not in acknowledged]
    if missing:
        raise ValueError(
            "Turn off in-camera image processing on the GoPro, then list each item in "
            f"camera.acknowledgements. Missing: {', '.join(missing)}. "
            "HyperSmooth/stabilisation and Horizon Lock crop and rotate per frame, which "
            "makes T_hand_camera time-varying; nothing downstream can detect it.")


def make_camera(config):
    backend = config.get("backend", "realsense")
    if backend == "realsense":
        return RealSenseCamera(config)
    if backend == "uvc":
        return UVCCamera(config)
    raise ValueError("camera.backend must be 'realsense' or 'uvc'")


def describe_camera_config(config):
    """Intrinsics without opening the device, for offline simulation and doctor."""
    backend = config.get("backend", "realsense")
    if backend != "uvc":
        raise ValueError("Only the uvc backend carries offline intrinsics")
    if not config.get("intrinsics"):
        raise ValueError("Set camera.intrinsics to a calibration file")
    return load_intrinsics(config["intrinsics"], (config["width"], config["height"]))
