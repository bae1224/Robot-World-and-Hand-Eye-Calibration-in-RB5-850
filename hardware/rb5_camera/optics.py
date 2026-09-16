"""Camera intrinsic models.

A RealSense hands us factory intrinsics over its SDK. A GoPro behind an HDMI
capture box hands us nothing at all, so the intrinsics become a separate,
explicitly versioned calibration artefact that every downstream pose depends on.
This module is the single place that knows how pixels map to bearings.

Supported ``model`` values:

``pinhole``    OpenCV Brown-Conrady, 5 coefficients (k1 k2 p1 p2 k3).
``rational``   OpenCV rational model, 8 coefficients (adds k4 k5 k6).
``fisheye``    OpenCV/Kannala-Brandt equidistant, 4 coefficients (k1..k4).

Bearings are normalised image-plane coordinates on z=1, so a pose solved from
them is independent of the model once ``normalize`` has been applied.
"""
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np

MODELS = {"pinhole": 5, "rational": 8, "fisheye": 4}


class CameraModel:
    def __init__(self, K, distortion, width, height, model="pinhole", metadata=None):
        if model not in MODELS:
            raise ValueError(f"camera model must be one of {sorted(MODELS)}")
        self.model = model
        self.K = np.asarray(K, dtype=float).reshape(3, 3).copy()
        self.distortion = np.asarray(distortion, dtype=float).reshape(-1).copy()
        self.width, self.height = int(width), int(height)
        if self.distortion.size != MODELS[model]:
            raise ValueError(f"{model} needs exactly {MODELS[model]} distortion coefficients")
        if not np.isfinite(self.K).all() or not np.isfinite(self.distortion).all():
            raise ValueError("Non-finite intrinsics")
        if (self.K[0, 0] <= 0 or self.K[1, 1] <= 0 or self.K[2, 2] != 1
                or self.K[0, 1] != 0 or self.K[1, 0] != 0 or self.K[2, 0] != 0 or self.K[2, 1] != 0):
            raise ValueError("K must be [[fx,0,cx],[0,fy,cy],[0,0,1]] with positive focal lengths")
        if not (0 < self.width and 0 < self.height):
            raise ValueError("Image size must be positive")
        # A principal point far outside the sensor means the calibration was run at
        # a different resolution or FOV mode than the images being fed in now.
        if not (0 <= self.K[0, 2] <= self.width and 0 <= self.K[1, 2] <= self.height):
            raise ValueError("Principal point lies outside the image: intrinsics and "
                             "image size disagree (different resolution or FOV mode?)")
        self.metadata = dict(metadata or {})

    @property
    def fx(self):
        return float(self.K[0, 0])

    @property
    def fy(self):
        return float(self.K[1, 1])

    @property
    def cx(self):
        return float(self.K[0, 2])

    @property
    def cy(self):
        return float(self.K[1, 2])

    def horizontal_fov_deg(self):
        half = self.width / 2 / self.fx
        angle = np.arctan(half) if self.model != "fisheye" else half
        return float(np.degrees(2 * angle))

    def normalize(self, pixels):
        """Distorted pixels -> undistorted bearings on the z=1 plane (N,2)."""
        pixels = np.asarray(pixels, dtype=np.float64).reshape(-1, 1, 2)
        if self.model == "fisheye":
            out = cv2.fisheye.undistortPoints(pixels, self.K, self.distortion)
        else:
            out = cv2.undistortPoints(pixels, self.K, self.distortion)
        out = out.reshape(-1, 2)
        if not np.isfinite(out).all():
            raise ValueError("Undistortion diverged; check the distortion model and coefficients")
        return out

    def project(self, objects, rvec, tvec):
        """Object points in the target frame -> distorted pixels (N,2)."""
        objects = np.asarray(objects, dtype=np.float64).reshape(-1, 1, 3)
        rvec = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
        tvec = np.asarray(tvec, dtype=np.float64).reshape(3, 1)
        if self.model == "fisheye":
            # cv2.fisheye.projectPoints insists on the (1,N,3) layout.
            pixels = cv2.fisheye.projectPoints(objects.reshape(1, -1, 3), rvec, tvec,
                                               self.K, self.distortion)[0]
        else:
            pixels = cv2.projectPoints(objects, rvec, tvec, self.K, self.distortion)[0]
        return np.asarray(pixels, dtype=np.float64).reshape(-1, 2)

    def unproject_grid(self, stride=1):
        """Bearings for every (strided) pixel centre. Used by the renderer."""
        us = np.arange(0, self.width, stride, dtype=np.float64)
        vs = np.arange(0, self.height, stride, dtype=np.float64)
        grid = np.stack(np.meshgrid(us, vs), axis=-1).reshape(-1, 2)
        return self.normalize(grid).reshape(len(vs), len(us), 2)

    def supports_opencv_charuco_refinement(self):
        """CharucoDetector's camera-aware refinement assumes Brown-Conrady."""
        return self.model in ("pinhole", "rational")

    def to_dict(self):
        return {"model": self.model, "width": self.width, "height": self.height,
                "K": self.K.tolist(), "distortion": self.distortion.tolist(),
                "fx": self.fx, "fy": self.fy, "cx": self.cx, "cy": self.cy,
                "horizontal_fov_deg": self.horizontal_fov_deg(), **self.metadata}


def load_intrinsics(path, expected_size=None):
    """Read a calibration JSON written by `intrinsics`, and hash it for provenance.

    The hash goes into session.json so a result can always be traced back to the
    exact intrinsics that produced it.
    """
    path = Path(path)
    raw = path.read_bytes()
    data = json.loads(raw)
    for key in ("model", "K", "distortion", "width", "height"):
        if key not in data:
            raise ValueError(f"{path}: intrinsics file is missing '{key}'")
    metadata = {k: v for k, v in data.items()
                if k not in ("K", "distortion", "width", "height", "model")}
    metadata["intrinsics_path"] = str(path.resolve())
    metadata["intrinsics_sha256"] = hashlib.sha256(raw).hexdigest()
    model = CameraModel(data["K"], data["distortion"], data["width"], data["height"],
                        data["model"], metadata)
    if expected_size is not None and tuple(expected_size) != (model.width, model.height):
        raise ValueError(f"{path}: calibrated for {model.width}x{model.height} but the "
                         f"configured stream is {expected_size[0]}x{expected_size[1]}. "
                         "Intrinsics are only valid for the resolution and FOV mode "
                         "they were calibrated at; recalibrate or fix the stream.")
    return model
