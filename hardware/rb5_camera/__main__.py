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
    if config["mount"] not in ("eye_to_hand", "eye_in_hand", "unconfigured"):
        raise ValueError("Invalid camera mount")
    target_type = config.get("target", {"type": "charuco"})["type"]
    reference_type = config.get("reference", {"type": "none"})["type"]
    if target_type not in ("aruco", "charuco") or reference_type not in ("none", "charuco"):
        raise ValueError("Invalid target or reference type")
    if reference_type == "charuco" and (target_type != "aruco" or config["mount"] != "eye_to_hand"):
        raise ValueError("Fixed ChArUco reference requires eye_to_hand with an ArUco hand marker")
    if not all(np.isfinite(v) and v > 0 for v in config["capture"].values()):
        raise ValueError("Capture thresholds must be positive finite numbers")
    if config["robot"]["timeout_s"] <= 0:
        raise ValueError("Robot timeout must be positive")
    camera = config["camera"]
    if camera.get("backend", "realsense") not in ("realsense", "uvc"):
        raise ValueError("camera.backend must be 'realsense' or 'uvc'")
    for key in ("width", "height", "fps"):
        if not (isinstance(camera.get(key), (int, float)) and camera[key] > 0):
            raise ValueError(f"camera.{key} must be a positive number")
    if camera.get("backend") == "uvc" and not camera.get("intrinsics"):
        raise ValueError("The uvc backend needs camera.intrinsics: a capture box carries no "
                         "calibration. Run the 'intrinsics' command first.")
    return config


def require_layout(config):
    if config["mount"] == "unconfigured":
        raise ValueError("Select a connected calibration layout: eye_in_hand (camera on robot, "
                         "fixed ChArUco) or eye_to_hand (fixed camera, ChArUco on robot). "
                         "Fixed camera + fixed board + FK alone cannot determine camera-to-base.")


def fk_command(args):
    from .kinematics import RB5Kinematics
    config = load_config(args.config)
    fk = RB5Kinematics(config["robot"]["kinematics"])
    T = fk.forward_degrees(args.joints_deg)
    print("URDF base -> tip (maps tip coordinates into base; metres):")
    print(T)
    print("Model:", fk.metadata["model_name"], "SHA256:", fk.metadata["urdf_sha256"])
    if not fk.metadata["model_matches_robot"]:
        print("Offline model calculation only: model version/mapping has not been matched to the physical robot.")
    if args.output:
        path = Path(args.output)
        if path.exists():
            raise ValueError("Output already exists")
        path.parent.mkdir(parents=True, exist_ok=True)
        atomic_json(path, {"T_base_hand": T.tolist(), "joints_deg": args.joints_deg, "kinematics": fk.metadata})


def doctor(args):
    import platform
    config = load_config(args.config)
    backend = config["camera"].get("backend", "realsense")
    print(f"Python: {sys.executable} ({platform.python_version()})")
    print(f"Camera backend: {backend}")
    required = ["numpy", "scipy", "opencv-contrib-python", "rbpodo"]
    if backend == "realsense":
        required.append("pyrealsense2")
    missing = False
    for name in required:
        try:
            print(f"{name}: {importlib.metadata.version(name)}")
        except importlib.metadata.PackageNotFoundError:
            print(f"{name}: NOT INSTALLED")
            missing = True
    if missing:
        return 1
    import cv2
    import rbpodo
    if not hasattr(cv2.aruco, "CharucoDetector") or not hasattr(rbpodo, "CobotData"):
        raise RuntimeError("Installed SDK API mismatch")
    if backend == "realsense":
        import pyrealsense2 as rs
        devices = list(rs.context().query_devices())
        print("RealSense devices:", [d.get_info(rs.camera_info.name) for d in devices])
    else:
        from .cameras import describe_camera_config, require_gopro_acknowledgements
        model = describe_camera_config(config["camera"])
        require_gopro_acknowledgements(config["camera"])
        print(f"Intrinsics: {model.model} {model.width}x{model.height} "
              f"f=({model.fx:.2f}, {model.fy:.2f}) c=({model.cx:.2f}, {model.cy:.2f}) "
              f"hfov={model.horizontal_fov_deg():.1f} deg")
        print(f"Intrinsics SHA256: {model.metadata.get('intrinsics_sha256', '?')}")
        sigma = model.metadata.get("focal_relative_sigma")
        if sigma is not None:
            print(f"Focal 1-sigma: {sigma * 100:.3f}% -> about "
                  f"{sigma * 1000:.2f} mm of X error per metre of working distance")
        if args.camera:
            from .cameras import make_camera
            camera = make_camera(config["camera"])
            try:
                image, meta = camera.frame()
                print("Captured frame:", image.shape, meta)
            finally:
                camera.close()
    if args.robot:
        from .devices import RobotReader
        robot = RobotReader(load_config(args.config)["robot"])
        data = robot.read()
        print("Active TCP [mm, deg]:", data["tcp_mm_deg"])
        print("Calibration robot pose source:", data["pose_source"])
        print(data["T"])
        print("Compare only after matching URDF base/tip frames with the controller base/TCP offset.")
    return 0


def collect(args):
    import cv2
    from .board import make_target, make_reference, estimate_observations, target_config
    from .cameras import make_camera
    from .devices import RobotReader, require_stationary
    config = load_config(args.config)
    require_layout(config)
    target_kind = target_config(config)["type"]
    reference_kind = config.get("reference", {"type": "none"})["type"]
    dimensions = config["target"] if target_kind == "aruco" else config["board"]
    if not dimensions["dimensions_measured"]:
        raise ValueError("Measure the moving target and set its dimensions_measured=true "
                         "(target for ArUco, board for ChArUco)")
    if reference_kind == "charuco" and not config["board"]["dimensions_measured"]:
        raise ValueError("Set the existing fixed ChArUco board dimensions and board.dimensions_measured=true")
    if config["robot"].get("pose_source") == "urdf_fk":
        if not config["robot"]["kinematics"].get("model_matches_robot", False):
            raise ValueError("Match the URDF version, base/tip and encoder mapping to the physical robot, "
                             "then set robot.kinematics.model_matches_robot=true")
    elif config["robot"]["tcp_description"].startswith("Enter "):
        raise ValueError("Record the active TCP name/offset in robot.tcp_description")
    session = Path(args.output)
    if session.exists():
        raise ValueError("Use a new session output directory")
    target = make_target(config)
    reference_board = make_reference(config)
    robot = RobotReader(config["robot"])
    camera = make_camera(config["camera"])
    try:
        session.mkdir(parents=True)
        (session / "samples").mkdir()
        (session / "images").mkdir()
        if robot.fk is not None:
            (session / "robot_model.urdf").write_bytes(robot.fk.chain.model_bytes)
        atomic_json(session / "session.json", {
            "schema_version": 2, "synthetic": False, "units": "metres",
            "reference_frame": "charuco" if reference_kind == "charuco" else "camera",
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "config": config, "camera": camera.metadata,
            "robot_pose": robot.metadata,
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
                cv2.imshow("RB5 camera", preview)
                key = cv2.waitKey(1) & 0xff
                if key in (ord("q"), 27) or cv2.getWindowProperty("RB5 camera", cv2.WND_PROP_VISIBLE) < 1:
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
                T_camera_target, T_camera_reference, quality = estimate_observations(
                    image, target, reference_board, camera.model, config["capture"])
                index = len(accepted)
                image_path = session / "images" / f"{index:04d}.png"
                if not cv2.imwrite(str(image_path), image):
                    raise OSError("Image write failed")
                record = {"index": index, "T_base_hand": before["T"].tolist(),
                          "T_camera_target": T_camera_target.tolist(), "frame": frame_metadata,
                          "quality": quality, "image": f"images/{index:04d}.png",
                          "robot_before": {k: v for k, v in before.items() if k != "T"},
                          "robot_after": {k: v for k, v in after.items() if k != "T"}}
                if T_camera_reference is not None:
                    record["T_camera_reference"] = T_camera_reference.tolist()
                atomic_json(session / "samples" / f"{index:04d}.json", record)
                accepted.append(before["T"])
                print(f"Saved {index:04d}: visual target {quality['target']['reprojection_rms_px']:.3f} px RMS")
                if "reference" in quality:
                    print(f"  Fixed ChArUco: {quality['reference']['corner_count']} corners, "
                          f"{quality['reference']['reprojection_rms_px']:.3f} px RMS")
            except (ValueError, TimeoutError) as error:
                print(f"Rejected: {error}")
        if accepted:
            print("Exported:", export_csv(session))
            print("Use 20–40 diverse poses, with rotations about multiple axes.")
    finally:
        camera.close()
        if not args.headless:
            cv2.destroyAllWindows()


def intrinsics_command(args):
    from .board import target_config
    from .intrinsics import calibrate, write_calibration
    config = load_config(args.config)
    if target_config(config)["type"] != "charuco":
        raise ValueError("Intrinsic calibration uses the ChArUco board")
    if not config["board"]["dimensions_measured"]:
        raise ValueError("Measure the printed board and set board.dimensions_measured=true; "
                         "an unmeasured square size scales every length that follows")
    folder = Path(args.images)
    paths = sorted(q for q in folder.rglob("*")
                   if q.suffix.lower() in (".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"))
    if not paths:
        raise ValueError(f"No images under {folder}")
    camera, record = calibrate(paths, config["board"], args.model, args.min_corners)
    record["camera_description"] = config["camera"].get("description", "")
    record["acknowledgements"] = sorted(config["camera"].get("acknowledgements", []))
    written = write_calibration(record, args.output)
    print(f"{record['image_count']} views, RMS {record['reprojection_rms_px']:.4f} px "
          f"(worst view {record['worst_view_rms_px']:.4f} px)")
    print(f"{camera.model}: f=({camera.fx:.2f}, {camera.fy:.2f}) c=({camera.cx:.2f}, {camera.cy:.2f}) "
          f"hfov={camera.horizontal_fov_deg():.1f} deg")
    sigma = record.get("focal_relative_sigma")
    if sigma is not None:
        print(f"Focal 1-sigma {sigma * 100:.3f}%, i.e. roughly {sigma * 1000:.2f} mm of "
              "hand-eye translation error per metre of working distance")
    print(f"Wrote {written}. Point camera.intrinsics at it.")
    return 0


def simulate(args):
    """Render a virtual session and push it through the real detector and exporter."""
    import cv2
    from .board import make_target, estimate_observations, target_config
    from .kinematics import RB5Kinematics
    from .simulator import (BoardRenderer, board_extent, board_pose_for_stance,
                            default_true_camera, perturb_camera)
    config = load_config(args.config)
    require_layout(config)
    if config["mount"] != "eye_in_hand":
        raise ValueError("simulate models the eye-in-hand layout")
    if target_config(config)["type"] != "charuco":
        raise ValueError("simulate renders a ChArUco target")
    if config["robot"].get("pose_source") != "urdf_fk":
        raise ValueError("simulate computes A from URDF forward kinematics")
    session = Path(args.output)
    if session.exists():
        raise ValueError("Use a new session output directory")

    rng = np.random.default_rng(args.seed)
    fk = RB5Kinematics(config["robot"]["kinematics"])
    true_camera = default_true_camera(config["camera"]["width"], config["camera"]["height"],
                                      args.model, args.fov_deg)
    assumed_camera = perturb_camera(true_camera, args.focal_error, args.principal_error_px,
                                    args.distortion_error, rng)
    renderer = BoardRenderer(config["board"], true_camera, rng)
    target = make_target(config)

    X = np.eye(4)
    X[:3, :3] = Rotation.from_euler("xyz", [-8.0, 4.0, 15.0], degrees=True).as_matrix()
    X[:3, 3] = [0.035, -0.060, 0.045]
    nominal = np.array([0.0, -35.0, 75.0, 0.0, 70.0, 0.0])
    Y = board_pose_for_stance(fk.forward_degrees(nominal), X,
                              board_extent(config["board"]), args.distance_m)

    session.mkdir(parents=True)
    (session / "samples").mkdir()
    (session / "images").mkdir()
    (session / "robot_model.urdf").write_bytes(fk.chain.model_bytes)
    atomic_json(session / "session.json", {
        "schema_version": 3, "synthetic": True, "simulated_images": True, "units": "metres",
        "reference_frame": "camera",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "config": config,
        "camera": {**assumed_camera.to_dict(),
                   "focal_relative_sigma": args.focal_sigma},
        "true_camera": true_camera.to_dict(), "robot_pose": fk.metadata,
        "synchronization": "simulated; the robot is exactly stationary at each pose"})
    atomic_json(session / "ground_truth.json", {
        "X": X.tolist(), "Y": Y.tolist(),
        "X_name": "T_hand_camera", "Y_name": "T_base_board",
        "true_camera": true_camera.to_dict(), "assumed_camera": assumed_camera.to_dict()})

    accepted, attempts, rejected = [], 0, {}
    while len(accepted) < args.count and attempts < args.count * 25:
        attempts += 1
        joints = nominal + rng.uniform(-args.spread_deg, args.spread_deg, 6)
        A = fk.forward_degrees(joints)
        T_camera_board = inverse(X) @ inverse(A) @ Y
        try:
            if T_camera_board[2, 3] <= 0.05:
                raise ValueError("Board behind or too close to the camera")
            image = renderer.render(T_camera_board, args.blur_sigma, args.noise_sigma)
            # The detector and PnP run on the *assumed* intrinsics, exactly as they
            # would on the robot, where the true lens is never known.
            T_camera_target, _, quality = estimate_observations(
                image, target, None, assumed_camera, config["capture"])
        except ValueError as error:
            rejected[str(error)] = rejected.get(str(error), 0) + 1
            continue
        too_close = False
        for previous in accepted:
            distance, angle = pose_distance(previous, A)
            if (distance < config["capture"]["min_pose_translation_m"]
                    and angle < config["capture"]["min_pose_rotation_deg"]):
                too_close = True
        if too_close:
            rejected["pose too close to an accepted sample"] = \
                rejected.get("pose too close to an accepted sample", 0) + 1
            continue
        index = len(accepted)
        if not args.no_images:
            if not cv2.imwrite(str(session / "images" / f"{index:04d}.png"), image):
                raise OSError("Image write failed")
        atomic_json(session / "samples" / f"{index:04d}.json", {
            "index": index, "T_base_hand": A.tolist(),
            "T_camera_target": T_camera_target.tolist(),
            "T_camera_target_true": T_camera_board.tolist(),
            "joints_deg": joints.tolist(), "quality": quality,
            "image": f"images/{index:04d}.png"})
        accepted.append(A)
    if len(accepted) < args.count:
        print(f"Only rendered {len(accepted)}/{args.count} poses. Rejections: {rejected}",
              file=sys.stderr)
    if len(accepted) < 8:
        raise ValueError("Fewer than 8 usable simulated poses; widen spread_deg or the FOV")
    errors = [s for s in (json.loads((session / "samples" / f"{i:04d}.json").read_text())
                          for i in range(len(accepted)))]
    pnp_translation = [float(np.linalg.norm(np.array(s["T_camera_target"])[:3, 3]
                                            - np.array(s["T_camera_target_true"])[:3, 3]))
                       for s in errors]
    rms = [s["quality"]["target"]["reprojection_rms_px"] for s in errors]
    print(f"Rendered {len(accepted)} poses ({attempts} attempts). "
          f"Detector RMS {np.mean(rms):.3f} px, "
          f"PnP translation bias {np.mean(pnp_translation) * 1000:.2f} mm")
    print("Exported:", export_csv(session))
    return 0


def synthetic(args):
    config = load_config(args.config)
    require_layout(config)
    rng = np.random.default_rng(850)
    def transform():
        T = np.eye(4)
        T[:3, :3] = Rotation.random(random_state=rng).as_matrix()
        T[:3, 3] = rng.uniform(-0.4, 0.4, 3)
        return T
    X, Y = transform(), transform()
    reference_kind = config.get("reference", {"type": "none"})["type"]
    C_reference = transform() if reference_kind == "charuco" else None
    fk = None
    if config["robot"].get("pose_source") == "urdf_fk":
        from .kinematics import RB5Kinematics
        fk = RB5Kinematics(config["robot"]["kinematics"])
    session = Path(args.output)
    session.mkdir(parents=True, exist_ok=False)
    (session / "samples").mkdir()
    metadata = {"schema_version": 2, "synthetic": True, "units": "metres", "config": config}
    if fk is not None:
        metadata["robot_pose"] = fk.metadata
        (session / "robot_model.urdf").write_bytes(fk.chain.model_bytes)
    atomic_json(session / "session.json", metadata)
    truth = {"X": X.tolist(), "Y": Y.tolist()}
    if C_reference is not None:
        truth["T_base_camera"] = (Y @ inverse(C_reference)).tolist()
    atomic_json(session / "ground_truth.json", truth)
    for i in range(30):
        if fk is not None:
            joint_angles = rng.uniform(-90, 90, 6)
            A = fk.forward_degrees(joint_angles)
        else:
            A = transform()
        B = inverse(Y) @ A @ X
        C = B if config["mount"] == "eye_to_hand" else inverse(B)
        sample = {"T_base_hand": A.tolist(), "T_camera_target": C.tolist()}
        if fk is not None:
            sample["joints_deg"] = joint_angles.tolist()
        if C_reference is not None:
            if config["mount"] != "eye_to_hand":
                raise ValueError("Fixed ChArUco reference requires eye_to_hand")
            sample["T_camera_target"] = (C_reference @ B).tolist()
            sample["T_camera_reference"] = C_reference.tolist()
        atomic_json(session / "samples" / f"{i:04d}.json", sample)
    print("Synthetic algebra test (not a robot trajectory):", export_csv(session))


def main():
    parser = argparse.ArgumentParser(description="RB5-850 RWHE calibration acquisition")
    parser.add_argument("--config", default="hardware/rb5.example.json")
    commands = parser.add_subparsers(dest="command", required=True)
    check = commands.add_parser("doctor", help="Check SDKs, intrinsics and connected devices")
    check.add_argument("--robot", action="store_true", help="Also read the configured robot")
    check.add_argument("--camera", action="store_true", help="Also open the camera and grab a frame")
    fk_parser = commands.add_parser("fk", help="Compute URDF FK offline from six measured joint angles")
    fk_parser.add_argument("--joints-deg", nargs=6, type=float, required=True)
    fk_parser.add_argument("--output")
    for name in ("board", "marker", "collect", "synthetic"):
        command = commands.add_parser(name)
        command.add_argument("--output", required=True)
        if name == "collect":
            command.add_argument("--headless", action="store_true", help="Use terminal Enter instead of an RGB window")
    calib = commands.add_parser("intrinsics",
                                help="Calibrate intrinsics from a folder of ChArUco images")
    calib.add_argument("images", help="Folder of still images, all at one resolution/FOV mode")
    calib.add_argument("--output", required=True, help="Calibration JSON to write")
    calib.add_argument("--model", default="fisheye", choices=("pinhole", "rational", "fisheye"))
    calib.add_argument("--min-corners", type=int, default=12)

    sim = commands.add_parser("simulate",
                              help="Render a virtual session through the real detector")
    sim.add_argument("--output", required=True)
    sim.add_argument("--count", type=int, default=30)
    sim.add_argument("--seed", type=int, default=850)
    sim.add_argument("--model", default="fisheye", choices=("pinhole", "rational", "fisheye"))
    sim.add_argument("--fov-deg", type=float, default=120.0)
    sim.add_argument("--distance-m", type=float, default=0.55)
    sim.add_argument("--spread-deg", type=float, default=14.0)
    sim.add_argument("--blur-sigma", type=float, default=0.7)
    sim.add_argument("--noise-sigma", type=float, default=1.5)
    sim.add_argument("--focal-error", type=float, default=0.0,
                     help="Relative focal error of the assumed intrinsics, e.g. 0.01 for 1%%")
    sim.add_argument("--focal-sigma", type=float, default=0.002,
                     help="Declared 1-sigma focal uncertainty, as an intrinsics file carries")
    sim.add_argument("--principal-error-px", type=float, default=0.0)
    sim.add_argument("--distortion-error", type=float, default=0.0)
    sim.add_argument("--no-images", action="store_true", help="Do not write the rendered PNGs")

    export = commands.add_parser("export")
    export.add_argument("session")
    export.add_argument("--output", help="New output directory, for re-export/recovery")
    args = parser.parse_args()
    try:
        if args.command == "doctor":
            return doctor(args)
        if args.command == "intrinsics":
            return intrinsics_command(args)
        if args.command == "simulate":
            return simulate(args)
        if args.command == "fk":
            fk_command(args)
            return 0
        if args.command == "board":
            from .board import write_board
            write_board(load_config(args.config)["board"], args.output)
        elif args.command == "marker":
            from .board import write_marker
            write_marker(load_config(args.config)["target"], args.output)
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
