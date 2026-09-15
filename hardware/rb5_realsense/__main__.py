import argparse
from datetime import datetime, timezone
import importlib.metadata
import json
from pathlib import Path
import sys
import time
import numpy as np
from scipy.spatial.transform import Rotation
from .dataset import atomic_json, export_csv
from .geometry import inverse, pose_distance


def load_config(path):
    config = json.loads(Path(path).read_text())
    if config["mount"] not in ("eye_to_hand", "eye_in_hand"):
        raise ValueError("Invalid camera mount")
    if not all(np.isfinite(v) and v > 0 for v in config["capture"].values()):
        raise ValueError("Capture thresholds must be positive finite numbers")
    if config["robot"]["timeout_s"] <= 0:
        raise ValueError("Robot timeout must be positive")
    return config


def doctor(args):
    import platform
    print(f"Python: {sys.executable} ({platform.python_version()})")
    missing = False
    for name in ("numpy", "scipy", "opencv-contrib-python", "pyrealsense2", "rbpodo"):
        try:
            print(f"{name}: {importlib.metadata.version(name)}")
        except importlib.metadata.PackageNotFoundError:
            print(f"{name}: NOT INSTALLED")
            missing = True
    if missing:
        return 1
    import cv2
    import rbpodo
    import pyrealsense2 as rs
    if not hasattr(cv2.aruco, "CharucoDetector") or not hasattr(rbpodo, "CobotData"):
        raise RuntimeError("Installed SDK API mismatch")
    devices = list(rs.context().query_devices())
    print("RealSense devices:", [d.get_info(rs.camera_info.name) for d in devices])
    if args.robot:
        from .devices import RobotReader
        robot = RobotReader(load_config(args.config)["robot"])
        data = robot.read()
        print("Active TCP [mm, deg]:", data["tcp_mm_deg"])
        print("Compare this pose and the active TCP offset with the teaching pendant.")
    return 0


def collect(args):
    import cv2
    from .board import make_board, estimate_board
    from .devices import RobotReader, ColorCamera, require_stationary
    config = load_config(args.config)
    if not config["board"]["dimensions_measured"]:
        raise ValueError("Measure the printed board, update square/marker lengths, then "
                         "set board.dimensions_measured=true")
    if config["robot"]["tcp_description"].startswith("Enter "):
        raise ValueError("Record the active TCP name/offset in robot.tcp_description")
    session = Path(args.output)
    if session.exists():
        raise ValueError("Use a new session output directory")
    board = make_board(config["board"])
    robot = RobotReader(config["robot"])
    camera = ColorCamera(config["camera"])
    try:
        session.mkdir(parents=True)
        (session / "samples").mkdir()
        (session / "images").mkdir()
        atomic_json(session / "session.json", {
            "schema_version": 1, "synthetic": False, "units": "metres",
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "config": config, "camera": camera.metadata,
            "pose_source": "rbpodo.sdata.tcp_pos; may be reference-based depending on firmware",
            "synchronization": "stationary window; camera and robot clocks are not synchronized"})
        accepted = []
        print("Move with the pendant and stop before capture. Space/Enter: capture; q: finish.")
        while True:
            if args.headless:
                if input(f"[{len(accepted)} samples] Enter / q: ").strip().lower() == "q":
                    break
            else:
                preview, _ = camera.frame()
                label = f"Samples: {len(accepted)} | Space: capture | q: finish"
                cv2.putText(preview, label, (15, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            .7, (0, 200, 0), 2)
                cv2.imshow("RB5 RealSense color", preview)
                key = cv2.waitKey(1) & 0xff
                if key in (ord("q"), 27) or cv2.getWindowProperty("RB5 RealSense color", cv2.WND_PROP_VISIBLE) < 1:
                    break
                if key not in (32, 10, 13):
                    continue
            try:
                reference = robot.read()
                deadline = time.monotonic() + config["capture"]["settle_s"]
                while time.monotonic() < deadline:
                    camera.frame()
                    current = robot.read()
                    require_stationary(reference, current, config["capture"])
                started = time.monotonic()
                before = robot.read()
                # Discard queued frames; all are acquired while the robot is held still.
                for _ in range(3):
                    image, frame_metadata = camera.frame()
                after = robot.read()
                if time.monotonic() - started > config["capture"]["max_capture_s"]:
                    raise ValueError("Capture latency exceeded max_capture_s")
                require_stationary(reference, before, config["capture"])
                require_stationary(reference, after, config["capture"])
                for old in accepted:
                    distance, angle = pose_distance(old, before["T"])
                    if (distance < config["capture"]["min_pose_translation_m"]
                            and angle < config["capture"]["min_pose_rotation_deg"]):
                        raise ValueError("Pose is too close to an already accepted sample")
                T_camera_board, quality = estimate_board(
                    image, board, camera.K, camera.distortion, config["capture"])
                index = len(accepted)
                image_path = session / "images" / f"{index:04d}.png"
                if not cv2.imwrite(str(image_path), image):
                    raise OSError("Image write failed")
                record = {"index": index, "T_base_tcp": before["T"].tolist(),
                          "T_camera_board": T_camera_board.tolist(), "frame": frame_metadata,
                          "quality": quality, "image": f"images/{index:04d}.png",
                          "robot_before": {k: v for k, v in before.items() if k != "T"},
                          "robot_after": {k: v for k, v in after.items() if k != "T"}}
                atomic_json(session / "samples" / f"{index:04d}.json", record)
                accepted.append(before["T"])
                print(f"Saved {index:04d}: {quality['corner_count']} corners, "
                      f"{quality['reprojection_rms_px']:.3f} px RMS")
            except (ValueError, TimeoutError) as error:
                print(f"Rejected: {error}")
        if accepted:
            print("Exported:", export_csv(session))
            print("Use 20–40 diverse poses, with rotations about multiple axes.")
    finally:
        camera.close()
        if not args.headless:
            cv2.destroyAllWindows()


def synthetic(args):
    config = load_config(args.config)
    rng = np.random.default_rng(850)
    def transform():
        T = np.eye(4)
        T[:3, :3] = Rotation.random(random_state=rng).as_matrix()
        T[:3, 3] = rng.uniform(-0.4, 0.4, 3)
        return T
    X, Y = transform(), transform()
    session = Path(args.output)
    session.mkdir(parents=True, exist_ok=False)
    (session / "samples").mkdir()
    atomic_json(session / "session.json", {"schema_version": 1, "synthetic": True,
                                           "units": "metres", "config": config})
    atomic_json(session / "ground_truth.json", {"X": X.tolist(), "Y": Y.tolist()})
    for i in range(30):
        A = transform()
        B = inverse(Y) @ A @ X
        C = B if config["mount"] == "eye_to_hand" else inverse(B)
        atomic_json(session / "samples" / f"{i:04d}.json",
                    {"T_base_tcp": A.tolist(), "T_camera_board": C.tolist()})
    print("Synthetic algebra test (not a robot trajectory):", export_csv(session))


def main():
    parser = argparse.ArgumentParser(description="RB5-850 + RealSense RWHE calibration")
    parser.add_argument("--config", default="hardware/rb5.example.json")
    commands = parser.add_subparsers(dest="command", required=True)
    check = commands.add_parser("doctor", help="Check SDKs and connected cameras")
    check.add_argument("--robot", action="store_true", help="Also read the configured robot")
    for name in ("board", "collect", "synthetic"):
        command = commands.add_parser(name)
        command.add_argument("--output", required=True)
        if name == "collect":
            command.add_argument("--headless", action="store_true", help="Use terminal Enter instead of an RGB window")
    export = commands.add_parser("export")
    export.add_argument("session")
    export.add_argument("--output", help="New output directory, for re-export/recovery")
    args = parser.parse_args()
    try:
        if args.command == "doctor":
            return doctor(args)
        if args.command == "board":
            from .board import write_board
            write_board(load_config(args.config)["board"], args.output)
        elif args.command == "collect":
            collect(args)
        elif args.command == "export":
            print(export_csv(args.session, args.output))
        else:
            synthetic(args)
    except (ValueError, OSError, RuntimeError, ImportError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    except (KeyboardInterrupt, EOFError):
        print("Stopped. Accepted JSON samples can be recovered with the export command.")
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
