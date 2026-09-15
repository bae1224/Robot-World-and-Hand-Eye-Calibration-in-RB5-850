# RB5-850 + D435if: Eye-in-Hand RWHE 캘리브레이션

## 현재 변경 사항

배치는 **Eye-in-Hand로 확정**했습니다. 카메라는 로봇 손목에 단단히 장착하고,
준비된 ChArUco 타겟은 작업대에서 로봇 베이스에 대해 고정합니다.
로봇 자세 계산을 **측정 관절각 `rbpodo.sdata.jnt_ang` → URDF 순기구학 → T_base_hand**로
변경했습니다. 공식 `rb5_850e.urdf`를 포함했으며 U 버전은 제외했습니다.
이 구현은 URDF를 선택했으므로 MJCF/MuJoCo 설치는 필요하지 않습니다.
메시·충돌·동역학 시뮬레이션 없이 joint origin, axis, angle, fixed joint만 계산합니다.

손목 카메라에서 관측한 ChArUco 자세를 로봇 FK와 묶어 저장합니다.
손목-카메라 X와 베이스-타겟 Y를 동시에 추정하는 Robot-world/hand-eye 문제입니다.

## 1. 확정된 장치 배치

- D435if는 손목에 고정하며, 수집 중 장착 위치·각도가 변하지 않아야 합니다.
- ChArUco는 로봇 베이스에 대해 고정된 작업대에 둡니다.
- 손목 자세를 바꾸면서 카메라가 ChArUco를 관측합니다.
- 카메라 케이블이 장착부를 당겨 자세를 바꾸지 않도록 배치합니다.

`hardware/rb5.example.json`과 `hardware/rb5.local.json`은 모두
`mount=eye_in_hand`, `target.type=charuco`, `reference.type=none`입니다.
`reference.type=none`은 별도의 두 번째 시각 패턴을 사용하지 않는다는 뜻이며,
ChArUco 타겟은 `target`/`board` 설정으로 사용합니다.

## 2. 좌표계와 AX=YB

`T_parent_child`는 child 좌표의 점을 parent 좌표로 옮기는 4×4 변환입니다.
모든 이동 성분은 m입니다.

| 항목 | Eye-in-Hand |
|---|---|
| A | T_base_hand (URDF FK) |
| B | inverse(T_camera_board) |
| X | T_hand_camera |
| Y | T_base_board |

손목 카메라 구성은 `T_base_hand × T_hand_camera = T_base_board × T_board_camera`입니다.

현재 모델의 `base=link0`, `hand=tcp`를 사용합니다. 여기서 **hand는 URDF의 tip 링크**이며,
제어기에 현재 설정된 TCP라는 뜻이 아닙니다. 그래서 새 결과는 `T_tcp_*` 대신
`T_hand_*`로 저장합니다. 제어기 TCP offset과 비교하려면 두 기준 프레임을 먼저 맞추세요.
`tip_link`를 바꾸면 X의 기준도 바뀝니다.

RGB 관측은 RealSense color optical frame(x 오른쪽, y 아래쪽, z 전방)입니다.
depth/IMU/외함 좌표와 다릅니다. ChArUco 좌표는 인쇄물 좌상단 원점,
x 오른쪽, y 아래쪽, 평면 z=0입니다. PnP는 SDK RGB 내부 파라미터를 사용하며
현재 none/Brown-Conrady 왜곡 모델을 지원합니다.

## 3. 공식 모델과 정확도

- 파일: `hardware/models/rainbow/rb5_850e.urdf`
- 출처: [Rainbow Robotics 공식 rbpodo_ros2](https://github.com/RainbowRobotics/rbpodo_ros2/blob/5e8294a985e7ce5e20e70564c2681130afc5f502/rbpodo_description/robots/rb5_850e.urdf)
- 커밋: `5e8294a985e7ce5e20e70564c2681130afc5f502`
- 라이선스·해시: 같은 폴더의 `LICENSE`, `provenance.json`
- 원본 URDF는 수정하지 않았습니다. 참조하는 메시 파일은 FK에 필요하지 않아 포함하지 않았습니다.

공식 non-U 모델을 준비했지만 실제 RB5-850 장비와의 일치는 아직 확인되지 않았습니다.
사용 국가만으로 모델 버전을 판별하지 말고 명판·사양을 확인하세요.
모델 치수, 관절 영점/부호/순서, 실제 링크 변형이 정확도에 영향을 줍니다.
URDF 기반 계산이 마커 관측이나 제어기 모델보다 더 정확하다고 검증된 것은 아닙니다.

관절 변환은 다음처럼 명시합니다.

```text
q_urdf = deg2rad(joint_signs * measured_joint_degrees + joint_offsets_deg)
encoder 순서 = [base, shoulder, elbow, wrist1, wrist2, wrist3]
```

기본 부호는 모두 +1, offset은 모두 0도입니다. 이는 공식 joint 이름/단위에 맞춘
초기 매핑이며, 실물 영점 검증을 대체하지 않습니다. 제어기의 `jnt_ref`나 `tcp_pos`를
FK 입력으로 사용하지 않습니다. `tcp_pos`는 비교 자료로만 보관합니다.

## 4. 설치 및 오프라인 FK 실행

이전 설치의 NumPy/SciPy와 Python 표준 XML 파서로 실행됩니다. 추가 패키지나
가상환경, ROS 실행, MuJoCo 설치가 필요하지 않습니다. 새 PC 설치는 다음과 같습니다.

```bash
bash scripts/install_rb5_local.sh
```

로봇 연결 없이 주어진 각도로 FK를 계산합니다. 다음은 수학적 입력 예시이며
실제 로봇을 이 자세로 이동시키는 명령이 아닙니다.

```bash
python3 -m hardware.rb5_realsense fk --joints-deg 0 0 0 0 0 0
python3 -m hardware.rb5_realsense fk --joints-deg 10 -20 30 0 45 -10 --output results/rb5/fk-example.json
```

공식 모델의 영각 결과는 위치 `[0, -0.2074, 1.0969] m`, 회전 I입니다.
FK 파일에는 입력 관절각, T_base_hand, URDF 경로/해시, base/tip, 관절 매핑이 기록됩니다.

## 5. 실측 설정

`hardware/rb5.local.json`에서 다음을 입력하세요. 장착 방식은 이미 반영했습니다.

- `mount=eye_in_hand`: 확정된 손목 카메라 배치.
- `target.type=charuco`, `reference.type=none`: ArUco 없는 기본 경로.
- `robot.ip`: 실제 제어기 IP.
- `robot.pose_source=urdf_fk`.
- `robot.kinematics.urdf_path`: URDF 경로. 상대 경로는 저장소 루트 기준.
- `base_link`, `tip_link`, `joint_names`, `joint_signs`, `joint_offsets_deg`: 실물과 매핑.
- `robot.kinematics.model_matches_robot`: 모델/프레임/매핑을 확인한 후 true.
- `board.dictionary`, `squares_x/y`, `square_length_m`, `marker_length_m`,
  `legacy_pattern`, 필요시 `marker_ids`: 준비된 ChArUco의 실제 사양.
- `board.dimensions_measured`: 실측 후 true.
- `camera.serial`: 카메라 여러 대 연결 시 지정.

보드 기본값은 DICT_4X4_50, 5×7칸, 한 칸 30mm, 내부 마커 22mm의 예시입니다.
보유 보드 사양으로 확정된 값이 아닙니다. 칸 수는 내부 코너 개수가 아닌 전체 칸 수이며,
미터 단위 크기와 인쇄 패턴 규칙이 일치해야 합니다.

## 6. 연결 확인과 수집

```bash
python3 -m hardware.rb5_realsense --config hardware/rb5.local.json doctor --robot
python3 -m hardware.rb5_realsense --config hardware/rb5.local.json collect --output data/rb5/session01
```

수집기는 로봇 데이터 포트 5001을 읽습니다. 이동은 사용자가 티칭 펜던트로 수행합니다.
프리뷰에서 ChArUco가 보이는지 확인하고 정지 후 Space/Enter로 저장, q로 종료합니다.
화면 없는 환경은 `--headless`를 추가합니다.

서로 평행하지 않은 여러 축의 회전과 위치 변화를 포함해 20–40개 자세를 수집하세요.
정지 확인은 기본 0.6초, FK pose 변화 0.5mm/0.1도 이하, 측정 관절 변화 0.1도 이하입니다.
ChArUco 최소 8코너, 재투영 RMS 1px 이하를 요구합니다. 모호한 평면 PnP는 거부합니다.
카메라와 로봇은 하드웨어 동기화되지 않으므로 정지 구간 내 측정만 저장합니다.

```text
session01/
  session.json       # 모델 해시, 기준 링크, 관절 매핑, 카메라 내부 파라미터
  robot_model.urdf   # 해당 세션 계산에 사용한 모델 원본
  images/0000.png
  samples/0000.json  # T_base_hand, T_camera_target, 측정 관절각, 비교용 제어기 TCP, 시간
  combined/
    tag_0_cam_0_A.csv
    tag_0_cam_0_B.csv
    metadata.json
    convention.toml  # hand_frame=urdf_tip
```

CSV는 헤더 없는 `qw,qx,qy,qz,tx,ty,tz`, 같은 행이 같은 샘플입니다.
중단 후에는 완성된 JSON 샘플을 새 출력 폴더로 복구할 수 있습니다.

```bash
python3 -m hardware.rb5_realsense export data/rb5/session01 --output data/rb5/session01/recovered
```

## 7. 논문 알고리즘 실행

손목 카메라 예시:

```bash
bash scripts/julia_rb5.sh scripts/calibrate_rb5.jl data/rb5/session01/combined results/rb5/session01 eye_in_hand
```

입력의 배치 metadata와 마지막 인자가 다르면 오류를 냅니다.
결과는 `T_hand_camera.csv`, `T_base_board.csv`로 저장되며 `residuals.csv`, `report.toml`에
잔차와 수치적 최적성 검사가 포함됩니다.

원본 목적함수와 SO(3) 제약식을 그대로 사용합니다. 기본 가중치 κ=125, translation
σ=0.01m는 시작값이며 장비에서 측정한 잡음값이 아닙니다. 필요하면 마지막 두 인자로 조절합니다.
`numerically_certified=true`는 주어진 입력/목적함수의 수치적 전역 최적성 증거입니다.
틀린 URDF/단위/프레임이나 실제 캘리브레이션 정확도를 보증하지 않습니다.

## 8. 검증

```bash
python3 -m unittest discover -s tests -v
bash scripts/julia_rb5.sh tests/rb5_solver.jl
python3 -m hardware.rb5_realsense synthetic --output data/rb5/fk-demo
bash scripts/julia_rb5.sh scripts/calibrate_rb5.jl data/rb5/fk-demo/combined results/rb5/fk-demo eye_in_hand
```

합성 A는 URDF FK로 계산합니다. B는 알려진 X/Y에서 생성하므로 대수 검증용이며
카메라의 실제 가시성이나 로봇 이동 가능 경로를 나타내지 않습니다.
Python 테스트는 공식 모델 영각/기준 회전, 관절 매핑, joint origin/axis/fixed link,
측정 관절 사용, 기존 데이터 호환성을 확인합니다. 이 PC에 이미 설치된 Orocos KDL과
50개 관절 자세를 독립 비교했습니다. KDL이 없는 PC에서는 해당 비교만 skip됩니다.

이전 ArUco 구현은 과거 데이터 재현용으로 남아 있지만 현재 기본 설정/수집에는 쓰지 않습니다.
