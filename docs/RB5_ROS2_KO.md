# RB5-850 실측 상태를 ROS 2로 받아 GoPro 기록에 연결

현재 GoPro 기본 설정은 `robot.transport = "ros2"`입니다.
데이터 흐름은 **RB5 제어기 → 패치한 rbpodo_ros2 → measured_feedback → 실측 관절각 + FK → GoPro 기록**입니다.
사진 import와 Julia 솔버는 기존 방식 그대로입니다.

## 왜 별도 상태 토픽을 추가했나

확인한 공식 `rbpodo_ros2` 커밋은 `5e8294a985e7ce5e20e70564c2681130afc5f502`입니다.
이 버전의 `RBPodoHardwareInterface::read()`는 관절 position state에 `jnt_ref`를 넣습니다.
따라서 `/joint_states` 또는 `/rbpodo/joint_states`를 캘리브레이션 실측값으로 사용하지 않습니다.

두 패치 `measured_state.patch`, `feedback_freshness.patch`를 순서대로 적용합니다.

- 기록 토픽: `/rbpodo_hardware/measured_feedback`, 타입 `rbpodo_msgs/msg/MeasuredFeedback`.
- `state`: 공식 `SystemState`. `state.jnt_ang`는 실제 관절각(rad), 위치는 m 단위입니다.
- `sequence`: 새 데이터 응답을 수신할 때만 증가하는 순번.
- `received_monotonic_ns`: 응답 수신 시 Linux 단조 시계 값. 카메라 노출 시각이 아닙니다.
- `stream_id`: Linux boot ID와 드라이버 인스턴스 식별자.
- 약 50 Hz로 새 순번만 발행하며, 수신 후 100 ms가 지난 캐시 데이터는 발행하지 않습니다.
- `/rbpodo_hardware/measured_state`의 기존 `SystemState` 토픽은 관찰용으로 유지합니다.

2026-10-08 실물 UI 7.4.2 / 제어기 25040302 [7.4.2]에서 관절각·전류 응답은 갱신되지만
`SystemState.time`은 약 `3.07e-41`로 반복되는 현상을 관찰했습니다.
공개 문서는 float 초 단위를 설명하며, 이 버전의 별도 시간 규약은 확인하지 못했습니다.
따라서 이를 정수 시간으로 재해석하거나 PC 시간을 controller 시간으로 덮어쓰지 않습니다.
원래 `controller_time_s`는 그대로 저장하고, 수신 순번·수신 시각을 별도 보존·검증합니다.
이는 **새 응답이 PC에 도착했음**을 검사하며 제어기 내부 센서 샘플의 나이까지 증명하지 않습니다.

Python 구독기는 rad → degree, m → mm를 변환해 기존 FK·CSV 계약에 연결합니다.

[공식 드라이버](https://github.com/RainbowRobotics/rbpodo_ros2) ·
[확인한 관절 상태 구현](https://github.com/RainbowRobotics/rbpodo_ros2/blob/5e8294a985e7ce5e20e70564c2681130afc5f502/rbpodo_hardware/src/rbpodo_hardware_interface.cpp#L235-L246) ·
[공식 메시지 단위](https://github.com/RainbowRobotics/rbpodo_ros2/blob/5e8294a985e7ce5e20e70564c2681130afc5f502/rbpodo_msgs/msg/SystemState.msg).

## 설치 및 로컬 빌드

저장소 루트에서 실행합니다. Ubuntu 22.04 + ROS 2 Humble + `/usr/bin/python3` 기준입니다.
다른 PC에서 준비할 경우 아래 ros2_control 관련 패키지도 설치해야 합니다.
관리자 비밀번호 입력이 필요한 첫 단계는 사용자가 터미널에서 실행합니다.

```bash
sudo apt update
sudo apt install build-essential cmake git \
  ros-humble-ament-cmake-ros ros-humble-hardware-interface \
  ros-humble-controller-manager ros-humble-joint-state-broadcaster \
  ros-humble-xacro ros-humble-robot-state-publisher

source /opt/ros/humble/setup.bash
/usr/bin/python3 scripts/build_rbpodo_ros2.py
source scripts/ros2_env.sh
```

빌드 스크립트는 고정한 공식 소스를 `.runtime/ros2_ws/src/rbpodo_ros2`에 준비하고 패치를 적용합니다.
C++ `rbpodo` v0.16.14도 고정 커밋으로 받아 `.runtime/rbpodo_cpp_install`에 설치합니다.
빌드/설치 경로는 `.runtime/ros2_ws/build-feedback-v2`, `.runtime/ros2_ws/install-feedback-v2`입니다.
이전 실행 파일을 덮어쓰지 않습니다. 빌드 뒤 기존 launch/echo를 종료하고 각 터미널에서
`source scripts/ros2_env.sh` 후 다시 시작해야 새 드라이버가 적용됩니다.
`rbpodo_msgs`, `rbpodo_hardware`, `rbpodo_description`, `rbpodo_bringup`만 CMake로 빌드하므로
이 경로에는 colcon·MoveIt·Python rbpodo 패키지 설치가 필요하지 않습니다.

정확한 버전은 `hardware/ros2/upstream.json`에 기록했습니다.
이미 다른 버전의 소스가 해당 런타임 경로에 있으면 덮어쓰지 않고 오류를 냅니다.
`--prepare-only`는 소스와 패치만 준비하고, `--messages-only`는 로봇 없이 통신 테스트용 메시지만 빌드합니다.

## 실물 드라이버 시작

실제 RB5 모델 및 URDF 관절 매핑을 확인한 후 아래 명령에서 실제 제어기 IP를 입력합니다.
`robot_ip` launch 인자가 ROS 드라이버의 접속 주소입니다.
ROS backend에서는 JSON의 `robot.ip`로 별도 TCP 연결을 만들지 않습니다.

```bash
source scripts/ros2_env.sh
ros2 launch ./hardware/ros2/calibration.launch.py \
  model_id:=rb5_850e robot_ip:=192.168.0.10
```

위 IP는 예시입니다. 모델도 실제 보유 장비가 E 버전인지 확인해야 합니다.
이 launch는 상태 broadcaster만 켜며 position/trajectory controller를 활성화하지 않습니다.
기존 드라이버 인스턴스와 동시에 같은 로봇으로 띄우지 마세요.

**공식 드라이버 초기화는 운전 모드(Real)와 속도비(1.0)를 설정합니다.**
따라서 드라이버 시작 자체가 완전한 읽기 전용 동작은 아닙니다.
이번 작업 중 실물 드라이버를 실행하거나 로봇에 접속하지 않았습니다.
GoPro 기록 프로그램 자체는 토픽 구독만 하고 이동·설정 서비스는 호출하지 않습니다.

두 번째 터미널도 저장소 루트에서 같은 ROS 환경을 설정합니다.

```bash
source scripts/ros2_env.sh
ros2 topic info /rbpodo_hardware/measured_feedback --verbose
ros2 topic echo /rbpodo_hardware/measured_feedback --once
```

메시지의 `jnt_ang`과 펜던트의 실제 관절각을 단위 변환해 대조하세요.
반드시 실제 하드웨어의 publisher 하나만 있어야 합니다.
기록기와 드라이버는 같은 Linux PC/부팅 세션에서 실행해야 합니다. 다른 PC/boot ID는 거부합니다.

## GoPro 촬영 기록

`hardware/rb5.local.json`이 없다면 GoPro 기본 설정을 복사합니다.
이미 있는 파일은 덮어쓰지 말고 다음 항목을 반영하세요.

```json
{
  "transport": "ros2",
  "ros2": {"state_topic": "/rbpodo_hardware/measured_feedback", "max_feedback_age_s": 0.2},
  "pose_source": "urdf_fk"
}
```

위 항목은 전체 설정의 `robot` 객체 안에 넣습니다.
URDF 모델 확인 플래그와 보드 실측 설정은 [GoPro 안내서](RB5_GOPRO13_KO.md)를 따릅니다.

```bash
source scripts/ros2_env.sh
/usr/bin/python3 -m hardware.rb5_gopro --config hardware/rb5.local.json record-pose \
  --sample-id pose001 --duration 10 --output data/rb5/gopro_raw/robot/pose001.json
```

로봇을 정지시킨 상태에서 READY 표시가 나오면 GoPro로 사진 한 장을 촬영합니다.
사진과 기록을 manifest로 연결하는 후속 import/Julia 단계는 기존과 같습니다.
오프라인 import에는 ROS 환경이나 실행 중인 드라이버가 필요하지 않습니다.

`controller_tcp` 옵션도 같은 ROS 메시지의 `tcp_pos`를 이용할 수 있습니다.
이 경우 제어기 활성 TCP 이름·오프셋·베이스 기준을 `tcp_description`에 기록해야 합니다.
직접 SDK 연결이 필요한 경우에만 `transport = "rbpodo"`와 Python `rbpodo`를 사용합니다.
기존 SDK 기록도 import 가능합니다.

## 데이터 검사와 동기화 범위

수신 timeout, 증가하는 응답 순번·수신 시각, 같은 드라이버 인스턴스,
최대 수신 경과 시간(기본 0.2초), 실제 운전 모드, idle 및 관절각/FK 정지를 검사합니다.
ROS 구독 QoS는 reliable, volatile, keep-last 1입니다. 다중 publisher도 거부합니다.
새 v2 기록은 대표 포즈 한 개와 촬영 검사 요약만 저장합니다.
import에서는 대표 포즈, 처음·마지막 수신 정보 및 최대 움직임 요약을 검증합니다.
중간 원본 시계열을 저장하지 않으므로 전체 구간의 재검사는 불가능합니다.
기존 v1 기록을 읽을 때는 전체 측정값을 다시 검사합니다.
기존 SDK 기록은 기존 controller 시간 증가 검사를 유지합니다.

같은 PC의 최근 수신 여부를 확인할 뿐 카메라 노출 시각과 동기화하지는 않습니다.
rosbag 재생은 실물 수집 중 사용하지 마세요. 의도적으로 재작성한 메타데이터를 인증하지 않습니다.
로봇을 고정하고 READY 정지 창 안에 수동 촬영해야 합니다.

## 로봇 없는 검증

일반 단위·회귀 테스트는 ROS를 source하지 않아도 실행됩니다.

```bash
/usr/bin/python3 -m unittest discover -s tests -v
```

실제 ROS 메시지/DDS 테스트는 localhost의 별도 domain과 임시 topic을 사용합니다.

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 scripts/build_rbpodo_ros2.py --messages-only
AMENT_CURRENT_PREFIX="$PWD/.runtime/ros2_msgs" source .runtime/ros2_msgs/share/rbpodo_msgs/local_setup.bash
ROS_DOMAIN_ID=212 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 -m unittest discover \
  -s tests -p ros2_feedback_integration.py -v
```

이는 합성 메시지로 전송·단위 변환·기록 파일·오프라인 FK 일치·timeout·stale·simulation 거부를 확인합니다.
실제 RB5 관절 매핑/통신 및 물리 캘리브레이션 정확도 검증과는 별개입니다.

2026-10-08 실물 제어기 TCP 연결과 기존 드라이버 활성화를 확인했습니다.
위 시간 필드 문제를 반영해 별도 prefix에 C++ 드라이버/메시지를 다시 빌드했습니다.
Python 회귀 테스트 29개와 실제 DDS 합성 테스트 4개가 통과했습니다. 일정한 controller 시간에서도 기록·import가
동작하고, 오래된 응답/중복 순번/재시작을 거부하는지 검증합니다.
새 드라이버는 기존 실행 프로세스를 종료하고 다시 실행해야 적용됩니다.
실제 관절 매핑과 물리 캘리브레이션 정확도는 별도 검증해야 합니다.
