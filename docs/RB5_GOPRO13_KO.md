# RB5-850 + GoPro HERO13 Black 저장 사진으로 RWHE

카메라는 로봇 손목에 장착하고 ChArUco 보드는 고정하는 `eye_in_hand` 구성입니다.
사진 모드·해상도·렌즈·보드는 intrinsic 촬영 때와 같습니다.
GoPro로 원본 JPG를 촬영하고 PC로 복사한 뒤, **촬영 당시 기록한 로봇 상태와 명시적으로 짝지어** 처리합니다.

## 반영한 내부 파라미터

`/media/bae/Workspace/intrinsic/output/fresh/calibration_fisheye_init1888/fisheye.json`을
`hardware/intrinsics/gopro13_photo_wide_fisheye.json`에 원본 그대로 복사했습니다.
별도 검증 보고서도 `gopro13_photo_wide_validation.json`으로 보존했습니다.
보정 JSON의 `UNVALIDATED`는 fitting 시점 상태이며, 이후 생성된 보고서의 `PASS` 및 보정 파일 SHA256 일치를 확인합니다.

- 사진: `photo_single`, `wide`, zoom 1.0, standard JPG, **5568 × 4872**.
- 픽셀: EXIF 자동 회전을 적용하지 않은 원본 저장 픽셀.
- K: `[[1742.110090393, 0, 2771.309741865], [0, 1741.460538459, 2441.211287230], [0, 0, 1]]`.
- fisheye D (`k1,k2,k3,k4`): `[-0.015998363346, -0.011090556745, 0.003714728984, -0.003040692303]`.
- 별도 검증: 13장, RMS 0.872481 px, PASS. 이는 픽셀 검증이며 로봇 외부 보정 정확도를 보증하지 않습니다.
- 보드: 14 × 9 squares, square 0.040 m, marker 0.024 m, `DICT_4X4_250`, family IDs 50–112, legacy pattern.

intrinsic 쪽의 sliced dictionary(offset=50)는 이 저장소에서 전체 dictionary와 명시적 `marker_ids`로 표현합니다.
두 정의로 출력되는 보드 이미지가 동일한지 테스트합니다.

`GoProCamera`는 원본 사진에서 ChArUco를 검출하고, `fisheye.undistortPoints`로 IPPE 초기 자세를 구한 뒤
**원본 픽셀 좌표에서 fisheye 재투영 오차를 최소화**합니다.
이 fisheye D를 일반 Brown 왜곡 계수로 `solvePnP`에 전달하면 안 됩니다.
크기가 다른 이미지는 거부합니다. 같은 크기라도 크롭·렌즈 변경·보정 후 재저장한 사진은 사용하지 마세요.

## 준비

로봇 상태 입력의 기본은 **ROS 2**입니다. 먼저 [ROS 2 설치·실행 안내](RB5_ROS2_KO.md)에 따라
패치한 드라이버를 준비하고, 기록할 터미널에서 `source scripts/ros2_env.sh`를 실행하세요.

저장소 루트에서 실행합니다. 현재 확인한 OpenCV/numpy/scipy 환경은 `/usr/bin/python3` (3.10)입니다.
`python3`가 Conda Python을 가리키면 필요한 패키지가 없을 수 있습니다.

```bash
/usr/bin/python3 -m pip install --user -r hardware/requirements-gopro.txt
cp hardware/rb5.gopro.json hardware/rb5.local.json
```

RealSense SDK는 GoPro 경로에 필요하지 않습니다. 기본 ROS backend에는 Python `rbpodo`도 필요하지 않습니다.
`robot.transport="rbpodo"`로 직접 연결할 때만 Python rbpodo를 별도 설치합니다.
Julia 최적화 의존성은 고정된 `environments/rb5/Project.toml`과 `Manifest.toml`을 사용합니다.
다른 PC에서 Julia/RWHE 의존성만 준비할 때는 다음 명령을 실행합니다.

```bash
bash scripts/install_julia_rb5.sh
bash scripts/julia_rb5.sh tests/rb5_solver.jl
```

Julia 1.10.10은 `~/.local/share/rwhe/julia-1.10.10`에,
패키지는 `~/.local/share/rwhe/julia-depot`에 설치됩니다.
`sudo`나 Conda 환경 변경은 필요하지 않습니다. 다운로드 SHA256을 검사한 후 설치합니다.


`hardware/rb5.local.json`에서 다음을 실제 장비에 맞게 작성합니다.

1. 기본 `robot.transport="ros2"`: 제어기 IP는 ROS 드라이버의 `robot_ip` launch 인자에 지정합니다.
   `robot.ros2.state_topic`은 `/rbpodo_hardware/measured_feedback`입니다.
   직접 SDK backend에서만 JSON `robot.ip`와 `port=5001`을 사용합니다.
2. 기본 `robot.pose_source = "urdf_fk"`: 실제 로봇이 포함된 `rb5_850e.urdf`와 일치하는지,
   base/tip 링크와 관절 순서·부호·영점을 확인한 후 `robot.kinematics.model_matches_robot = true`.
   **RB5-850이라는 이름만으로 E 모델과 동일하다고 가정하지 않습니다.**
3. 인쇄된 격자 40 mm/마커 24 mm 실측을 확인한 후 `board.dimensions_measured = true`.
   보정에 사용한 것과 같은 보드여도 실제 크기는 확인해야 합니다.

FK 모델 대신 제어기의 TCP 자세를 직접 사용하려면 `robot.pose_source = "controller_tcp"`로 바꾸고,
`robot.tcp_description`에 활성 TCP 이름·오프셋 및 베이스 기준임을 기록합니다.
사용자 좌표계/다른 기준의 자세를 베이스 좌표계인 것처럼 사용하면 안 됩니다.
수집 중 TCP 설정을 바꾸지 마세요. URDF의 `tcp` 링크와 제어기에서 설정한 TCP는 별개입니다.

## 로봇 pose의 출처: IK가 아니라 측정값 + FK

기본 경로는 RB5 제어기 → `rbpodo_ros2` → `/rbpodo_hardware/measured_feedback` → Python 구독기입니다.
ROS 메시지는 rad/m 단위이며 아래의 기존 기록용 degree/mm로 변환합니다.
직접 SDK backend는 `rbpodo.CobotData.request_data().sdata`를 이용합니다.

| 값 | 의미 | 코드에서 쓰는 방식 |
|---|---|---|
| `jnt_ang[6]` | 실제 엔코더 관절각, degree | 기본: URDF **정기구학(FK)** → `T_base_hand` |
| `tcp_pos[6]` | 엔코더 기반 TCP 위치·회전, mm/degree | 선택: 활성 TCP를 기준으로 `T_base_tcp` |
| `jnt_ref`, `tcp_ref` | 목표/reference 값 | 캘리브레이션 측정값으로 사용하지 않음 |

IK는 목표 자세에서 관절각을 계산하는 과정입니다. 촬영 당시 로봇 자세를 측정하는 기능이 아닙니다.
GoPro 기록 프로그램은 이동 명령을 보내지 않고 ROS 상태 토픽만 구독합니다.
공식 ROS 드라이버 시작 시 수행되는 운전 모드·속도비 설정은 ROS 2 안내서를 확인하세요.

공식 문서: [상태 데이터 구조와 단위](https://rainbowrobotics.github.io/rb_cobot_docs/technical_docs/data_structure),
[소켓 통신과 포트](https://rainbowrobotics.github.io/rb_cobot_docs/technical_docs/socket_communication).

## 촬영할 때 로봇 상태 기록

반복 촬영은 아래 스크립트를 한 번 실행하면 편리합니다. 어느 폴더에서도 실행할 수 있습니다.
ROS 드라이버는 별도 터미널에서 실행 중이어야 합니다.

```bash
bash /media/bae/Workspace/Calib/Robot-World-and-Hand-Eye-Calibration-in-RB5-850/scripts/gopro_capture.sh
```

프로젝트 폴더에서는 `bash scripts/gopro_capture.sh`만 실행하면 됩니다.
스크립트가 프로젝트 이동과 ROS 환경 설정을 수행합니다.
매 자세에서 로봇을 정지시킨 뒤 **Enter → READY에서 수동 촬영 → Saved 확인**을 반복하세요.
`q` 또는 `Ctrl+C`로 종료합니다. 기존 파일의 가장 큰 번호 다음부터 자동으로 이어지며,
중간에 삭제된 번호를 다시 사용하지 않습니다. 촬영 창은 기본 10초이고 자동 주기 저장은 하지 않습니다.
기록 실패 후에는 원인을 해결하고 Enter로 다시 촬영하세요. 포즈 ID와 사진 파일명은 계속 메모합니다.

단일 포즈를 직접 지정하려면 아래 기존 명령을 사용합니다.

펜던트로 로봇을 이동한 후 정지시킵니다. 다음 명령을 실행하고 **READY가 출력된 다음 10초 창 안에서 사진 한 장**을 촬영하세요.
완료 메시지가 나올 때까지 로봇과 고정 보드를 움직이지 않습니다. 자세마다 고유한 `sample_id`와 실제 사진 파일명을 메모합니다.

```bash
/usr/bin/python3 -m hardware.rb5_gopro --config hardware/rb5.local.json record-pose \
  --sample-id pose001 --duration 10 \
  --output data/rb5/gopro_raw/robot/pose001.json
```

`pose002`, `pose003` 등으로 반복합니다. 여러 축의 회전과 이동을 포함한 20–40개 자세를 수집하세요.
단순히 한 축으로 회전하거나 비슷한 자세만 반복하면 캘리브레이션이 퇴화할 수 있습니다.

새 기록(`rb5-gopro-robot/v2`)은 **READY 직전 대표 포즈 한 개**를 저장합니다.
`T_base_hand`는 한 번만 저장하며 `representative`에는 그 시점의 관절각·TCP·시간·수신 순번이 들어갑니다.
촬영 중 반복 측정은 그대로 검사하지만 파일에는 `stationary_window` 요약만 남깁니다.
요약에는 측정 횟수, 대표 자세 대비 최대 위치·회전·관절 변화, 마지막 수신 정보가 포함됩니다.
중간 측정값을 보관하지 않으므로 import에서는 대표 포즈를 재계산하고 요약을 검증합니다.
촬영 구간 전체를 원본 시계열로 다시 검증할 수는 없습니다. 기존 v1 기록은 계속 읽을 수 있습니다.
로봇 상태가 idle이 아니거나 simulation 모드, 오래된 데이터, 허용치 이상의 이동이면 기록을 거부합니다.
`--duration`은 수동 촬영 창이며 RealSense용 `capture.max_capture_s`와 별개입니다.

GoPro 원격 셔터 및 카메라/로봇 시계 동기화는 구현하지 않았습니다.
정지 여부는 반복 측정 시점에서 검사하며, 실제 노출이 READY 창 안에 있었는지는 촬영자가 확인해야 합니다.
사진 EXIF 시간만으로 자동 짝맞춤하지 않습니다.

**이미 찍은 사진에 대응하는 당시 로봇 기록이 없다면 지금 읽은 자세를 붙이면 안 됩니다.**
당시 로그가 남아 있다면 그 로그의 프레임·단위·시간을 확인해 변환해야 하며,
로그가 없다면 위 방식으로 다시 수집해야 합니다.

## 사진 가져오기 → 논문 솔버 입력

사진을 예를 들어 `data/rb5/gopro_raw/images/`로 원본 그대로 복사합니다.
`data/rb5/gopro_raw/manifest.json`에 실제 짝을 작성합니다. 경로는 manifest 폴더 기준입니다.

```json
{
  "samples": [
    {"sample_id": "pose001", "image": "images/GP010401.JPG", "robot_record": "robot/pose001.json"},
    {"sample_id": "pose002", "image": "images/GP010402.JPG", "robot_record": "robot/pose002.json"},
    {"sample_id": "pose003", "image": "images/GP010403.JPG", "robot_record": "robot/pose003.json"}
  ]
}
```

```bash
/usr/bin/python3 -m hardware.rb5_gopro --config hardware/rb5.local.json import \
  --manifest data/rb5/gopro_raw/manifest.json \
  --output data/rb5/gopro_session01

bash scripts/julia_rb5.sh scripts/calibrate_rb5.jl \
  data/rb5/gopro_session01/combined results/rb5/gopro_session01 eye_in_hand
```

Julia 실행은 기존 프로젝트와 Julia 런타임 설치가 필요합니다.
기본 런타임이 다른 곳에 있다면 `RWHE_JULIA`, `RWHE_JULIA_DEPOT`를 지정합니다.

중복 ID/사진/로봇 기록, FK 모델·TCP 불일치, 기록 변조/정지 검사 실패는 입력 오류로 처리합니다.
보드 검출 실패, 해상도 불일치, 과도한 재투영 오차, 평면 자세 모호성, 유사한 로봇 자세는 샘플을 거부하고
`import_report.json`에 이유를 기록합니다. 기본 재투영 RMS 상한은 1 px입니다.
거부된 사진과 연결된 로봇 pose도 함께 제외합니다. `combined/sample_ids.json`은 CSV 행 순서입니다.

세션에는 원본 사진·로봇 기록·intrinsic/검증 보고서·설정·URDF(FK 모드)를 복사하여 보존합니다.
입력 폴더는 수정하지 않으며 출력 폴더는 새 경로여야 합니다.

`T_parent_child`는 child 좌표를 parent 좌표로 옮기는 변환입니다.

| AX = YB 항목 | 변환 |
|---|---|
| A | `T_base_hand` (또는 controller TCP 모드의 `T_base_tcp`) |
| B | `inverse(T_camera_board)` |
| X | `T_hand_camera` (또는 `T_tcp_camera`) |
| Y | `T_base_board` |

CSV는 논문 구현의 기존 `qw,qx,qy,qz,tx,ty,tz` 순서이고 이동 단위는 m입니다.
결과는 FK 모드에서 `T_hand_camera.csv`, `T_base_board.csv`, `residuals.csv`, `report.toml`입니다.

## 검증 범위

```bash
/usr/bin/python3 -m unittest discover -s tests -v
```

합성 fisheye 투영의 자세 복원, 실제 보드와 동일한 마커 배치, 렌더링 사진부터 A/B CSV까지,
입력 순서를 바꾼 짝맞춤, 기록 무결성·정지 검사 및 기존 Python 테스트를 검증합니다.
실물 로봇 네트워크·관절 매핑·셔터 타이밍 및 최종 외부 보정 정확도는 실험에서 확인해야 합니다.

이번 변경 검증에서 Python 테스트 23개가 통과했습니다. intrinsic 폴더의 검출 성공 검증 사진 13장을
새 GoPro 경로로 재처리한 결과, 현 기본 상한 1 px에서 9장이 통과하고 4장(GP010398–GP010401)이
재투영 RMS 초과(약 1.025–1.346 px)로 제외됐습니다. 이 사진들은 intrinsic 검증용이며
촬영 당시 RB5 pose가 연결된 RWHE 데이터로 취급하지 않았습니다.
초기 GoPro 변경에서는 실물 통신 및 Julia 재실행을 검증하지 않았습니다.
이후 추가한 ROS 2 backend의 설치·검증 범위는 [ROS 2 안내서](RB5_ROS2_KO.md)를 따릅니다.

2026-10-08 Julia 환경 준비 완료: Julia 1.10.10과 고정 Manifest 의존성을 설치하고
51개 의존성의 사전 컴파일을 완료했습니다. 기존 Julia 솔버 테스트는 11/11 통과했습니다.
합성 FK 자세 30쌍으로 실제 `scripts/calibrate_rb5.jl` 명령을 실행해 결과 파일 4개 생성,
X/Y 변환의 합성 정답 일치 및 `numerically_certified=true`를 확인했습니다.
검증 산출물은 `.runtime/validation/julia-rwhe.KwqYs3/`에 있습니다.
이는 실행 환경·솔버 검증이며 실제 GoPro/RB5 캘리브레이션 정확도 검증은 아닙니다.
