#!/usr/bin/env python3
"""Prepare pinned sources and build a local ROS 2 overlay. Never connects to a robot."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SPEC = json.loads((ROOT / 'hardware/ros2/upstream.json').read_text())
RUNTIME = ROOT / '.runtime'


def run(*args, cwd=None):
    subprocess.run([str(arg) for arg in args], cwd=cwd, check=True)


def checkout(url, revision, path):
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        run('git', 'init', path)
        run('git', '-C', path, 'remote', 'add', 'origin', url)
        run('git', '-C', path, 'fetch', '--depth', '1', 'origin', revision)
        run('git', '-C', path, 'checkout', '--detach', 'FETCH_HEAD')
    head = subprocess.check_output(['git', '-C', str(path), 'rev-parse', 'HEAD'], text=True).strip()
    if head != revision:
        raise ValueError(f'{path}: expected commit {revision}, found {head}; no files overwritten')


def prepare():
    driver = RUNTIME / 'ros2_ws/src/rbpodo_ros2'
    checkout(SPEC['repository'], SPEC['commit'], driver)
    freshness_patch = ROOT / 'hardware/ros2' / SPEC['freshness_patch']
    # The second patch changes the first patch's lines. Check the final state first.
    if subprocess.run(['git', '-C', str(driver), 'apply', '--reverse', '--check', str(freshness_patch)],
                      capture_output=True).returncode == 0:
        return driver
    patch = ROOT / 'hardware/ros2' / SPEC['patch']
    if subprocess.run(['git', '-C', str(driver), 'apply', '--reverse', '--check', str(patch)],
                      capture_output=True).returncode:
        run('git', '-C', driver, 'apply', '--check', patch)
        run('git', '-C', driver, 'apply', patch)
    run('git', '-C', driver, 'apply', '--check', freshness_patch)
    run('git', '-C', driver, 'apply', freshness_patch)
    return driver


def cmake_build(source, build, prefix, jobs):
    run('cmake', '-S', source, '-B', build, '-DCMAKE_BUILD_TYPE=Release',
        f'-DCMAKE_INSTALL_PREFIX={prefix}', '-DBUILD_TESTING=OFF',
        '-DPython3_EXECUTABLE=/usr/bin/python3', '-DPYTHON_EXECUTABLE=/usr/bin/python3')
    run('cmake', '--build', build, '-j', str(jobs))
    run('cmake', '--install', build)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--messages-only', action='store_true', help='Build official message types for offline ROS tests')
    parser.add_argument('--jobs', type=int, default=2)
    args = parser.parse_args()
    if args.jobs < 1:
        raise ValueError('--jobs must be positive')
    driver = prepare()
    if args.prepare_only:
        print('Prepared patched source:', driver)
        return
    if os.environ.get('ROS_DISTRO') != 'humble':
        raise ValueError('Run source /opt/ros/humble/setup.bash first')
    if args.messages_only:
        cmake_build(driver / 'rbpodo_msgs', RUNTIME / 'ros2_msgs_build', RUNTIME / 'ros2_msgs', args.jobs)
        return
    required = ['ament_cmake_ros', 'hardware_interface', 'controller_manager',
                'joint_state_broadcaster', 'xacro', 'robot_state_publisher']
    missing = [pkg for pkg in required if subprocess.run(
        ['ros2', 'pkg', 'prefix', pkg], capture_output=True).returncode]
    if missing:
        raise ValueError('Missing ROS packages: ' + ', '.join(missing) + '\nInstall: sudo apt install '
                         + ' '.join('ros-humble-' + pkg.replace('_', '-') for pkg in missing))
    sdk = RUNTIME / 'rbpodo_cpp'
    checkout(SPEC['rbpodo_repository'], SPEC['rbpodo_commit'], sdk)
    sdk_prefix = RUNTIME / 'rbpodo_cpp_install'
    cmake_build(sdk, RUNTIME / 'rbpodo_cpp_build', sdk_prefix, args.jobs)
    # A small merged overlay avoids needing colcon/MoveIt for acquisition.
    prefix = RUNTIME / 'ros2_ws/install-feedback-v2'
    for name, value in (('CMAKE_PREFIX_PATH', f'{prefix}:{sdk_prefix}'), ('AMENT_PREFIX_PATH', str(prefix))):
        os.environ[name] = value + (':' + os.environ[name] if os.environ.get(name) else '')
    for package in ('rbpodo_msgs', 'rbpodo_hardware', 'rbpodo_description', 'rbpodo_bringup'):
        cmake_build(driver / package, RUNTIME / 'ros2_ws/build-feedback-v2' / package, prefix, args.jobs)
    print('Built ROS overlay:', prefix)
    print('Source with: source scripts/ros2_env.sh')


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f'Error: {error}', file=sys.stderr)
        sys.exit(1)
