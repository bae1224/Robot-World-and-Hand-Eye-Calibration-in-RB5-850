"""Operator-triggered capture with automatic IDs; never moves the robot."""
from pathlib import Path
import re
import numpy as np
from .workflow import record_pose, require_robot_config


def next_sample_id(directory):
    numbers = []
    for path in Path(directory).glob('pose*.json*'):
        match = re.fullmatch(r'pose([0-9]+)\.json(?:\.tmp)?', path.name)
        if match:
            numbers.append(int(match[1]))
    return f'pose{max(numbers, default=0) + 1:03d}'


def capture_poses(config, directory, duration=10):
    require_robot_config(config)
    if not np.isfinite(duration) or duration <= 0:
        raise ValueError('Duration must be positive finite seconds')
    directory = Path(directory)
    print(f'포즈 저장 폴더: {directory.resolve()}')
    print('로봇을 이동하고 정지 → Enter → READY에서 사진 촬영 → Saved까지 정지 유지')
    print('매번 Enter를 눌러 시작합니다. 자동 주기 촬영은 하지 않습니다. 종료: q 또는 Ctrl+C')
    while True:
        sample_id = next_sample_id(directory)
        try:
            answer = input(f'\n다음 {sample_id} | 준비되면 Enter, 종료는 q: ').strip().lower()
        except EOFError:
            return
        if answer in ('q', 'quit', 'exit'):
            return
        if answer:
            print('Enter 또는 q를 입력하세요.')
            continue
        try:
            record_pose(config, sample_id, directory / f'{sample_id}.json', duration)
        except (ValueError, OSError, RuntimeError, ImportError, KeyError) as error:
            print(f'{sample_id} 기록 실패: {error}')
            print('이번 촬영 쌍은 사용하지 말고, 문제를 해결한 뒤 다시 시작하세요.')
            continue
        print(f'{sample_id}에 대응하는 GoPro 사진 파일명을 메모하세요. 이제 다음 자세로 이동할 수 있습니다.')
