# RB5-850 + RealSense D435if 재현 환경

## 구현 범위

기본 구성은 **Eye-to-Hand: 카메라 고정, 보드는 로봇 TCP에 고정**입니다.
Python으로 RGB 영상과 로봇 자세를 수집하고, 원본 Julia 코드의 목적함수와
회전 제약식으로 `AX = YB` 문제를 풉니다. 실측 보드 크기를 사용하므로 scale=1입니다.
단일 카메라·단일 보드 실험은 논문의 일반화 문제에 포함되는 특수한 경우입니다.
논문의 다중 센서/타깃·미지 스케일 실험 및 전체 표 재현은 원본 `experiments/`를 사용합니다.
새 실행기는 같은 SDP의 primal 문제를 COSMO로 풀며, 논문의 모든 실험이나 실행 시간을
동일하게 재현했다고 주장하지 않습니다.

## 1. 가상환경 없는 로컬 설치

Ubuntu 22.04 x86_64 / Python 3.10에서 저장소 루트에서 실행합니다.

```bash
bash scripts/install_rb5_local.sh
python3 -m hardware.rb5_realsense doctor
```

설치 경로와 내용:

- Python: `python3 -m pip install --user`, 기본 `~/.local/lib/python3.10/site-packages`.
- Python 패키지: NumPy, SciPy, OpenCV contrib, 공식 `pyrealsense2`, 공식 `rbpodo`.
- Julia 1.10.10: `~/.local/share/rwhe/julia-1.10.10` (공식 SHA256 검증).
- Julia 패키지: `~/.local/share/rwhe/julia-depot`, 환경 정의는 `environments/rb5/`.
- 설치 후 Python/Julia 검증을 수행합니다. 최초 설치는 다운로드·컴파일에 시간이 걸립니다.

sudo/apt, 가상환경, ROS 설치는 사용하지 않습니다. `--user` 패키지는 같은 사용자의
다른 Python 프로그램에서도 보이므로 버전을 `hardware/requirements-rb5.txt`에 기록했습니다.
Julia의 의존성 해석 결과는 설치 시 `environments/rb5/Manifest.toml`에 저장됩니다.
원본 루트 `Project.toml`에 포함된 논문용 플롯/실험 패키지는 이 실행 경로에 필요하지 않습니다.

장치가 없으면 doctor의 장치 목록은 비어 있을 수 있습니다. USB 권한 문제는 Python SDK
설치만으로 해결되지 않습니다. 실제 연결 시 권한 오류가 나면 공식
[Linux 설치 안내](https://github.com/realsenseai/librealsense/blob/master/doc/installation.md)의
udev 설정을 적용해야 합니다. 이 작업은 시스템 관리자 권한이 필요합니다.

## 2. 좌표계와 로봇 자세

`T_parent_child`는 **child 좌표의 점을 parent 좌표로 옮기는 행렬**입니다.
예: `p_base = T_base_camera @ p_camera`. 모든 저장 행렬의 이동 성분은 m입니다.

| 항목 | Eye-to-Hand 기본값 | Eye-in-Hand 선택 시 |
|---|---|---|
| A | T_base_tcp | T_base_tcp |
| B | T_camera_board | inverse(T_camera_board) |
| X 결과 | T_tcp_board | T_tcp_camera |
| Y 결과 | T_base_camera | T_base_board |

Eye-to-Hand에서는 `T_base_tcp · T_tcp_board = T_base_camera · T_camera_board`입니다.
카메라는 RealSense **color optical frame**(x 오른쪽, y 아래, z 전방)입니다.
depth/IMU/장치 외함 좌표와 다릅니다. depth 영상과 함께 쓰려면 SDK의 depth↔color
내부 외부파라미터를 추가로 합성해야 합니다.
보드 좌표는 OpenCV ChArUco 정의(평면 z=0, 인쇄물 좌상단 원점, x 오른쪽, y 아래)를 따릅니다.

RB 제어기 `tcp_pos`의 `[x,y,z,Rx,Ry,Rz]`를 mm/도에서 m/회전행렬로 바꿉니다.
회전은 **Rz(Rz) Ry(Ry) Rx(Rx)**이며 rotation vector가 아닙니다.
TCP는 제어기에 현재 설정된 툴 기준입니다. flange라고 가정하지 마세요.
수집 중 TCP offset, tool, user/base 설정을 바꾸지 마세요.
실측 시작 전 doctor 출력값을 티칭 펜던트의 **베이스 기준 TCP 표시값**과 비교하세요.
공식 SDK에는 `tcp_pos`가 일부 펌웨어에서 reference 값으로 덮어써질 수 있다는 주석이 있습니다.
원시 TCP, 측정 관절값, 타임스탬프를 함께 기록하고 정지 상태만 수집합니다.

## 3. 보드 생성과 설정

```bash
cp hardware/rb5.example.json hardware/rb5.local.json
python3 -m hardware.rb5_realsense board --output data/rb5/board
```

`data/rb5/board/board-print.svg`를 **100%, 페이지 맞춤 끔**으로 인쇄합니다.
기본 ChArUco는 5×7칸, 한 칸 30mm, 마커 22mm, 패턴 전체 150×210mm입니다.
흰 여백을 포함한 SVG 크기는 170×230mm입니다. `board.png`는 미리보기용입니다.
평평하고 휘지 않는 판에 부착하고 실제 칸·마커 크기를 재세요.
생성한 보드와 동일한 dictionary/칸 개수/legacyPattern=false를 사용해야 합니다.

`hardware/rb5.local.json` 수정:

- `robot.ip`: 티칭 펜던트 Setup/System에 표시된 실제 IP. PC와 통신 가능하도록 네트워크 설정.
- `robot.tcp_description`: 현재 TCP 이름과 offset 기록.
- `camera.serial`: 한 대이면 빈 값 가능. 여러 대이면 사용할 장치 serial 필수.
- `board.square_length_m`, `marker_length_m`: 실측 m 값.
- `board.dimensions_measured`: 실측 입력 후 `true`.
- `mount`: `eye_to_hand` 기본. 손목 장착 카메라면 `eye_in_hand`.

RGB 카메라 내부 파라미터는 실행한 스트림에서 읽고 기록합니다. 현재 구현은
none/Brown-Conrady 왜곡 모델을 지원하며 다른 모델을 임의 변환하지 않고 오류를 냅니다.
보드 자세는 RGB PnP로 추정하므로 depth, IMU 캘리브레이션은 필요하지 않습니다.
정면에 가까운 평면 타깃은 PnP 모호성이 있으므로 여러 기울기를 포함해 수집하고
저장된 `planar_hypothesis_errors_px`, 원본 영상, 최종 잔차를 함께 확인하세요.

## 4. 연결 확인과 정지 자세 수집

```bash
python3 -m hardware.rb5_realsense --config hardware/rb5.local.json doctor --robot
python3 -m hardware.rb5_realsense --config hardware/rb5.local.json collect --output data/rb5/session01
```

로봇 이동은 사용자가 티칭 펜던트로 수행합니다. 수집기는 TCP **데이터 포트 5001**을
읽으며 로봇 이동·모드 변경 명령을 보내지 않습니다.
장착물과 보드를 고정하고, 실시간 RGB 창을 보며 각 자세에서 멈춘 다음
영상 창에서 Space 또는 Enter를 누릅니다. 화면 없는 환경은 `--headless`를 추가해
터미널 Enter로 수집할 수 있습니다.
서로 평행하지 않은 여러 축의 회전과 위치 변화를 포함해 20–40개 자세를 수집하세요.
보드가 영상 안에 충분히 크게 보이고, 흐림·반사·가림이 없는 자세를 사용합니다.
같은 방향으로 평행이동만 하면 장착 위치를 결정할 수 없습니다.

기본 수집 기준: 정지 확인 0.6초, TCP 변화 0.5mm/0.1도 이하, 관절 변화 0.1도 이하,
ChArUco 코너 8개 이상, 재투영 RMS 1px 이하. 기준은 실험 품질에 따라 조절합니다.
샘플은 하드웨어 동기화되지 않습니다. 서로 다른 장치 시간을 직접 빼지 않으며,
정지 구간 내 로봇 측정과 새 RGB 프레임을 묶고 각 장치 시간·호스트 수신시간을 보관합니다.

`q`로 종료하면 다음 형식으로 CSV를 생성합니다.

```text
session01/
  session.json       # 설정, 카메라 내부 파라미터, 장치 정보
  images/0000.png    # RGB 원본
  samples/0000.json  # 원시 로봇값, 카메라 보드 자세, 품질·시간 정보
  combined/
    tag_0_cam_0_A.csv
    tag_0_cam_0_B.csv
    metadata.json
    convention.toml  # Julia에서 장착 방식·단위 일치 검사
```

CSV는 헤더 없이 `qw,qx,qy,qz,tx,ty,tz`; 같은 행 번호가 한 쌍입니다.
`tag_0`은 보드 전체 한 개를 뜻하며 개별 ChArUco 마커 ID가 아닙니다.
기존 세션을 덮어쓰지 않습니다. 중단된 세션의 완성된 JSON 샘플은 다음으로 복구합니다.

```bash
python3 -m hardware.rb5_realsense export data/rb5/session01 --output data/rb5/session01/recovered
```

## 5. 논문 목적함수로 캘리브레이션

```bash
bash scripts/julia_rb5.sh scripts/calibrate_rb5.jl data/rb5/session01/combined results/rb5/session01 eye_to_hand
```

새 결과 폴더에 `T_base_camera.csv`, `T_camera_base.csv`, `T_tcp_board.csv`,
`residuals.csv`, `report.toml`을 저장합니다. 각 변환은 4×4입니다.
가중치는 기본 κ=125, translation σ=0.01m이며 마지막 두 인자로 조절할 수 있습니다.
이는 시작값이며 해당 장비에서 측정한 잡음 표준편차라고 가정하지 않습니다.
입력 자세를 바꾸어 최종 정확도를 확인하는 별도 검증 데이터도 수집하세요.

`report.toml`은 SDP 제약 위반, lifted matrix의 rank 지표, dual slack 고유값,
primal 비용과 보수적으로 보정한 dual 하한 사이의 차이를 제공합니다.
`numerically_certified=true`는 선택한 목적함수의 전역 최적성에 대한 부동소수점
허용오차 내 증거입니다. 물리적 캘리브레이션 정확도 또는 엄밀한 산술 증명은 아닙니다.
false이면 추정 행렬은 참고용이며, 이를 전역 최적해로 보고하면 안 됩니다.

## 6. 하드웨어 없이 검증

```bash
python3 -m unittest discover -s tests -v
bash scripts/julia_rb5.sh tests/rb5_solver.jl
python3 -m hardware.rb5_realsense synthetic --output data/rb5/synthetic01
bash scripts/julia_rb5.sh scripts/calibrate_rb5.jl data/rb5/synthetic01/combined results/rb5/synthetic01 eye_to_hand
```

합성 데이터는 알려진 X/Y로 `AX=YB`를 생성한 대수 검증용이며 실제 로봇 이동 경로가 아닙니다.
Python 테스트는 단위·회전 순서·두 장착 방식·CSV 대응·정지/타임아웃 처리·보드 자세를 검증합니다.
Julia 테스트는 원본 제약식, 무잡음 복원, 잡음 입력, 비용 하한, 관측 불가능한 입력 거부를 검증합니다.

## 근거 문서

- 첨부 논문 v2: 식 (6)–(10), known-scale QCQP와 SDP relaxation.
- [원본 논문](https://arxiv.org/abs/2507.23045)
- [Rainbow 공식 rbpodo SDK](https://github.com/RainbowRobotics/rbpodo)
- [RB5-850 공식 ZYX 회전 예제](https://rainbowrobotics.github.io/rb_cobot_docs/ko/technical_docs/analytic_inverse_kinematics)
- [Rainbow TCP 데이터 정의](https://github.com/RainbowRobotics/rbpodo/blob/main/include/rbpodo/data_type.hpp)
- [RealSense 공식 SDK 및 D435if 지원](https://github.com/realsenseai/librealsense)
- [OpenCV ChArUco 검출](https://docs.opencv.org/4.11.0/df/d4a/tutorial_charuco_detection.html)
