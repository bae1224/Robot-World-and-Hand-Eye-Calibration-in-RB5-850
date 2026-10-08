"""Manual shutter during a sampled stationary window, followed by explicit pairing."""
from datetime import datetime, timezone
from .feedback import MESSAGE_TYPE, CLOCK_SOURCE, validate_feedback

import json
from pathlib import Path
import re
import shutil
import time
import cv2
import numpy as np
from hardware.rb5_realsense.__main__ import load_config, require_layout
from hardware.rb5_realsense.dataset import atomic_json, export_csv
from hardware.rb5_realsense.devices import RobotReader, require_stationary
from hardware.rb5_realsense.geometry import rb_pose_to_transform, pose_distance, validate_transform
from hardware.rb5_realsense.kinematics import RB5Kinematics
from .camera import GoProCamera, sha256


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def checked_config(path):
    config = load_config(path)
    require_layout(config)
    if (config.get('camera', {}).get('type') != 'gopro_saved_images'
            or config.get('target', {}).get('type') != 'charuco'
            or config.get('reference', {}).get('type') != 'none'):
        raise ValueError('GoPro workflow requires saved images with a ChArUco target and no reference')
    return config


def require_robot_config(config):
    robot = config['robot']
    if robot.get('transport', 'rbpodo') not in ('ros2', 'rbpodo'):
        raise ValueError('robot.transport must be ros2 or rbpodo')
    if robot.get('pose_source') == 'urdf_fk':
        if not robot['kinematics'].get('model_matches_robot', False):
            raise ValueError('Match the physical robot model/base/tip/joint mapping, then set model_matches_robot=true')
    elif robot.get('pose_source') == 'controller_tcp':
        if not robot.get('tcp_description') or robot['tcp_description'].startswith('Enter '):
            raise ValueError('Record the active base-relative controller TCP name/offset in tcp_description')
    else:
        raise ValueError('robot.pose_source must be urdf_fk or controller_tcp')


def check_sample_id(sample_id):
    if not isinstance(sample_id, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', sample_id):
        raise ValueError('sample_id must contain only letters, digits, underscore or hyphen')
    return sample_id


def serializable_reading(reading):
    return {key: value.tolist() if isinstance(value, np.ndarray) else value
            for key, value in reading.items()}


def make_robot_reader(config):
    transport = config.get('transport', 'rbpodo')
    if transport == 'ros2':
        from .ros2_robot import ROS2RobotReader
        return ROS2RobotReader(config)
    if transport == 'rbpodo':
        return RobotReader(config)
    raise ValueError('robot.transport must be ros2 or rbpodo')


def close_robot_reader(robot):
    close = getattr(robot, 'close', None)
    if close is not None:
        close()


def record_pose(config, sample_id, output, duration_s=10.0):
    """Read-only controller access. Operator fires GoPro while READY is displayed."""
    require_robot_config(config)
    check_sample_id(sample_id)
    if not np.isfinite(duration_s) or duration_s <= 0:
        raise ValueError('duration must be positive finite seconds')
    output = Path(output)
    if output.exists():
        raise ValueError('Robot record already exists; choose a new output path')
    robot = make_robot_reader(config['robot'])
    try:
        reference = robot.read()
        deadline = time.monotonic() + config['capture']['settle_s']
        while time.monotonic() < deadline:
            require_stationary(reference, robot.read(), config['capture'])
            time.sleep(.05)
        before = robot.read()
        require_stationary(reference, before, config['capture'])
        started_utc = utc_now()
        readings = [serializable_reading(before)]
        print(f'READY {sample_id}: take ONE GoPro photo now; keep the robot still for {duration_s:g} s.', flush=True)
        deadline = time.monotonic() + duration_s
        while True:
            current = robot.read()
            require_stationary(reference, current, config['capture'])
            readings.append(serializable_reading(current))
            if time.monotonic() >= deadline:
                break
            time.sleep(.05)
        record = {'schema': 'rb5-gopro-robot/v1', 'sample_id': sample_id,
                  'window_started_utc': started_utc, 'window_finished_utc': utc_now(),
                  'synchronization': 'manual shutter during sampled stationary window; exposure time unverified',
                  'robot_config': config['robot'], 'robot_pose': robot.metadata,
                  'T_base_hand': before['T'].tolist(), 'readings': readings}
        if robot.fk is not None:
            record['urdf_xml'] = robot.fk.chain.model_bytes.decode('utf-8')
        output.parent.mkdir(parents=True, exist_ok=True)
        atomic_json(output, compact_robot_record(record, config, robot.fk))
    finally:
        close_robot_reader(robot)
    print(f'Saved {sample_id}: {output}. Pair it with the photo taken during READY.')


def load_robot_record(path, sample_id, config, fk):
    return validate_robot_record(json.loads(Path(path).read_text()), sample_id, config, fk)


def validate_robot_record(record, sample_id, config, fk):
    compact = record.get('schema') == 'rb5-gopro-robot/v2'
    if record.get('schema') not in ('rb5-gopro-robot/v1', 'rb5-gopro-robot/v2') or record.get('sample_id') != sample_id:
        raise ValueError(f'{sample_id}: robot record schema/sample_id mismatch')
    source = config['robot']['pose_source']
    if record.get('robot_pose', {}).get('pose_source') != source:
        raise ValueError(f'{sample_id}: robot pose source mismatch')
    if fk is not None:
        # Compare actual model content and mapping, not its filesystem location.
        for key in ('urdf_sha256', 'base_link', 'tip_link', 'encoder_joint_names',
                    'joint_signs', 'joint_offsets_deg'):
            if record['robot_pose'].get(key) != fk.metadata[key]:
                raise ValueError(f'{sample_id}: robot kinematics mismatch: {key}')
    elif record.get('robot_config', {}).get('tcp_description') != config['robot']['tcp_description']:
        raise ValueError(f'{sample_id}: active TCP description differs from recorded TCP')
    if compact:
        if 'readings' in record:
            raise ValueError(f'{sample_id}: compact records must contain only one representative pose')
        readings = [{**record['representative'], 'T': record['T_base_hand'], 'pose_source': source}]
    else:
        readings = record.get('readings', [])
        if len(readings) < 2:
            raise ValueError(f'{sample_id}: missing stationary-window readings')
    first = None
    last_time = -np.inf
    previous_feedback = None
    pose_metadata = record['robot_pose']
    receive_clock = pose_metadata.get('clock_source') == CLOCK_SOURCE
    if receive_clock and (pose_metadata.get('transport') != 'ros2'
                          or pose_metadata.get('message_type') != MESSAGE_TYPE):
        raise ValueError(f'{sample_id}: invalid feedback clock metadata')
    for reading in readings:
        joints = np.asarray(reading['joints_deg'], dtype=float)
        if joints.shape != (6,) or not np.isfinite(joints).all():
            raise ValueError(f'{sample_id}: invalid measured joints')
        T = fk.forward_degrees(joints) if fk else rb_pose_to_transform(reading['tcp_mm_deg'])
        if reading.get('pose_source') != source or not np.allclose(T, validate_transform(reading['T']), atol=1e-10, rtol=0):
            raise ValueError(f'{sample_id}: saved transform differs from raw robot measurement')
        timestamp = reading['controller_time_s']
        if receive_clock:
            validate_feedback(reading, previous_feedback,
                              float(reading['received_monotonic_s']) * 1e9,
                              float(config['robot'].get('ros2', {}).get('max_feedback_age_s', .2)))
            previous_feedback = reading
            if not np.isfinite(timestamp):
                raise ValueError(f'{sample_id}: invalid raw controller time')
        elif not np.isfinite(timestamp) or timestamp <= last_time:
            raise ValueError(f'{sample_id}: stale controller timestamps')
        last_time = timestamp
        current = {**reading, 'T': T}
        if first is None:
            first = current
        require_stationary(first, current, config['capture'])
    if not np.allclose(first['T'], validate_transform(record['T_base_hand']), atol=1e-10, rtol=0):
        raise ValueError(f'{sample_id}: inconsistent robot pose')
    if compact:
        validate_window_summary(record, config)
    return first['T'], record


def clock_observation(reading):
    return {key: reading[key] for key in (
        'controller_time_s', 'request_start_monotonic_s', 'received_monotonic_s',
        'feedback_stream_id', 'feedback_sequence', 'feedback_received_monotonic_ns') if key in reading}


def compact_robot_record(record, config, fk):
    """Validate a full v1 trace, then keep one pose and aggregate capture checks.

    The summary cannot independently reconstruct intermediate measurements.
    """
    validate_robot_record(record, record['sample_id'], config, fk)
    if record['schema'] == 'rb5-gopro-robot/v2':
        return record
    readings = record['readings']
    first = readings[0]
    reference = np.asarray(first['T'])
    summary = {
        'sample_count': len(readings),
        'max_translation_m': 0.0, 'max_rotation_deg': 0.0, 'max_joint_delta_deg': 0.0,
        'last_observation': clock_observation(readings[-1]),
        'validation_basis': 'all sampled readings checked before summarizing; intermediate poses not retained',
    }
    receive_clock = record['robot_pose'].get('clock_source') == CLOCK_SOURCE
    if receive_clock:
        summary['max_feedback_age_s'] = 0.0
    for reading in readings:
        distance, angle = pose_distance(reference, np.asarray(reading['T']))
        summary['max_translation_m'] = max(summary['max_translation_m'], float(distance))
        summary['max_rotation_deg'] = max(summary['max_rotation_deg'], float(angle))
        joint_delta = np.max(np.abs(np.asarray(first['joints_deg']) - reading['joints_deg']))
        summary['max_joint_delta_deg'] = max(summary['max_joint_delta_deg'], float(joint_delta))
        if receive_clock:
            age = float(reading['received_monotonic_s']) - reading['feedback_received_monotonic_ns'] / 1e9
            summary['max_feedback_age_s'] = max(summary['max_feedback_age_s'], age)
    result = {key: value for key, value in record.items() if key != 'readings'}
    result.update(schema='rb5-gopro-robot/v2',
                  representative={key: value for key, value in first.items()
                                  if key not in ('T', 'pose_source', 'transport', 'state_topic')},
                  stationary_window=summary)
    validate_robot_record(result, record['sample_id'], config, fk)
    return result


def validate_window_summary(record, config):
    summary = record['stationary_window']
    count = summary.get('sample_count')
    if type(count) is not int or count < 2:
        raise ValueError('Missing stationary-window sample count')
    for measured, limit in [('max_translation_m', 'max_motion_m'),
                            ('max_rotation_deg', 'max_motion_deg'),
                            ('max_joint_delta_deg', 'max_joint_motion_deg')]:
        value = summary[measured]
        if not np.isfinite(value) or not 0 <= value <= config['capture'][limit]:
            raise ValueError(f'Robot moved during capture: {measured}')
    first = record['representative']
    last = summary['last_observation']
    if record['robot_pose'].get('clock_source') == CLOCK_SOURCE:
        limit = float(config['robot'].get('ros2', {}).get('max_feedback_age_s', .2))
        validate_feedback(last, first, float(last['received_monotonic_s']) * 1e9, limit)
        age = summary['max_feedback_age_s']
        if not np.isfinite(age) or not 0 <= age <= limit:
            raise ValueError('Stale capture-window feedback')
        if last['feedback_sequence'] - first['feedback_sequence'] < count - 1:
            raise ValueError('Capture sample count exceeds unique feedback sequences')
    elif (not np.isfinite(last['controller_time_s'])
          or last['controller_time_s'] <= first['controller_time_s']):
        raise ValueError('Stale capture-window controller timestamps')
    if 'received_monotonic_s' in first:
        end = last['received_monotonic_s']
        if not np.isfinite(end) or end <= first['received_monotonic_s']:
            raise ValueError('Invalid capture-window receive interval')


def load_manifest(path):
    path = Path(path).resolve()
    manifest = json.loads(path.read_text())
    entries = manifest.get('samples', [])
    if not entries:
        raise ValueError('Manifest has no samples')
    ids, images, records = set(), set(), set()
    result = []
    for entry in entries:
        sample_id = check_sample_id(entry['sample_id'])
        image = (path.parent / entry['image']).resolve()
        record = (path.parent / entry['robot_record']).resolve()
        if sample_id in ids or image in images or record in records:
            raise ValueError('Duplicate sample_id, image or robot record in manifest')
        if not image.is_file() or not record.is_file():
            raise ValueError(f'{sample_id}: image or robot record does not exist')
        ids.add(sample_id)
        images.add(image)
        records.add(record)
        result.append((sample_id, image, record))
    return result


def import_images(config, manifest, output):
    from hardware.rb5_realsense.board import make_board
    require_robot_config(config)
    if not config['board']['dimensions_measured']:
        raise ValueError('Measure the printed board and set board.dimensions_measured=true')
    entries = load_manifest(manifest)
    session = Path(output)
    if session.exists():
        raise ValueError('Use a new session output directory')
    camera = GoProCamera(config['camera'])
    board = make_board(config['board'])
    fk = RB5Kinematics(config['robot']['kinematics']) if config['robot']['pose_source'] == 'urdf_fk' else None
    # Preflight all pairings and robot records before creating the session.
    pairs, digests = [], set()
    for sample_id, image_path, robot_path in entries:
        T, record = load_robot_record(robot_path, sample_id, config, fk)
        digest = sha256(image_path)
        if digest in digests:
            raise ValueError('Identical image bytes paired to multiple robot records')
        digests.add(digest)
        pairs.append((sample_id, image_path, robot_path, T, record, digest))
    session.mkdir(parents=True)
    for directory in ('samples', 'images', 'robot_records'):
        (session / directory).mkdir()
    shutil.copyfile(camera.path, session / 'intrinsics.json')
    shutil.copyfile(camera.validation_path, session / 'intrinsic_validation.json')
    shutil.copyfile(manifest, session / 'input_manifest.json')
    if fk is not None:
        (session / 'robot_model.urdf').write_bytes(fk.chain.model_bytes)
    atomic_json(session / 'session.json', {
        'schema_version': 2, 'synthetic': False, 'units': 'metres', 'created_utc': utc_now(),
        'config': config, 'camera': camera.metadata,
        'robot_pose': pairs[0][4]['robot_pose'],
        'synchronization': 'explicit sample_id pairing; operator must take photo inside recorded stationary window'})
    accepted, rejected, sample_ids = [], [], []
    for sample_id, image_path, robot_path, T, record, digest in pairs:
        try:
            image = camera.read(image_path)
            for old in accepted:
                distance, angle = pose_distance(old, T)
                if (distance < config['capture']['min_pose_translation_m']
                        and angle < config['capture']['min_pose_rotation_deg']):
                    raise ValueError('Robot pose too close to an already accepted sample')
            C, quality = camera.estimate(image, board, config['capture'])
        except (ValueError, cv2.error) as error:
            rejected.append({'sample_id': sample_id, 'reason': str(error)})
            print(f'Rejected {sample_id}: {error}')
            continue
        index = len(accepted)
        destination = Path('images') / f'{index:04d}{image_path.suffix}'
        shutil.copyfile(image_path, session / destination)
        shutil.copyfile(robot_path, session / 'robot_records' / f'{index:04d}.json')
        atomic_json(session / 'samples' / f'{index:04d}.json', {
            'index': index, 'sample_id': sample_id, 'image': str(destination),
            'source_image': str(image_path), 'image_sha256': digest,
            'robot_record_sha256': sha256(robot_path),
            'T_base_hand': T.tolist(), 'T_camera_target': C.tolist(), 'quality': quality,
            'robot_record': str(Path('robot_records') / f'{index:04d}.json')})
        accepted.append(T)
        sample_ids.append(sample_id)
        print(f'Accepted {sample_id}: {quality["target"]["reprojection_rms_px"]:.3f} px RMS')
    atomic_json(session / 'import_report.json', {'accepted_sample_ids': sample_ids, 'rejected': rejected})
    if not accepted:
        raise ValueError('No accepted samples; see import_report.json')
    combined = export_csv(session)
    atomic_json(combined / 'sample_ids.json', sample_ids)
    print(f'Exported {len(accepted)} pairs: {combined}. Collect 20–40 diverse stationary poses.')
    return combined

