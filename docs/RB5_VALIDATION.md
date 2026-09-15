# 설치 및 검증 기록

검증일: 2026-09-15. 환경: Ubuntu 22.04 x86_64, `/usr/bin/python3` 3.10.12.

## 설치 완료

가상환경 없이 Python 사용자 영역(`~/.local`)에 설치했습니다.

| 패키지 | 설치 버전 |
|---|---|
| NumPy | 1.26.4 |
| SciPy | 1.15.3 |
| OpenCV contrib | 4.11.0.86 |
| pyrealsense2 | 2.56.5.9235 |
| rbpodo | 0.16.14 |
| Julia | 1.10.10 |
| COSMO | 0.8.11 |

Julia 및 패키지는 `~/.local/share/rwhe`에 설치했습니다.
해석된 Julia 의존성은 `environments/rb5/Manifest.toml`에 기록했습니다.
중간에 생성됐던 미완성 `.venv`는 제거했습니다.

## 현재 구성: URDF FK + ChArUco

- 공식 non-U `rb5_850e.urdf`를 포함했습니다. U 버전 파일은 제외했습니다.
- 측정 `jnt_ang`에서 FK를 계산하며 `tcp_pos`는 비교용으로 저장합니다.
- Python 17개 테스트 통과(skip 없음). 이미 설치된 Orocos KDL과 50개 자세에서
  FK를 비교해 절대 허용오차 1e-12 내에서 일치했습니다.
- URDF FK → 합성 A/B → Julia SDP → X/Y 파일 정답 비교를 두 배치 모두 수행했습니다.
  Eye-in-Hand와 Eye-to-Hand에서 X 최대 원소 오차 약 5.41e-10, Y 약 1.38e-11 이하입니다.
- 두 배치 모두 `numerically_certified=true`. 결과는 `results/rb5/fk-eye-in-hand01/`,
  `results/rb5/fk-eye-to-hand01/`에 있습니다. 단위·관절 매핑·frame 이름도 검사했습니다.
- 모델의 출처/버전/해시는 `hardware/models/rainbow/provenance.json`에 기록했습니다.
- 추가 Python 패키지는 설치하지 않았습니다. MJCF/MuJoCo는 사용하지 않습니다.

카메라·ChArUco 배치는 Eye-in-Hand로 확정했으며 기본·로컬 설정에 반영했습니다.
변경 후 Python 17개 테스트 및 기본 설정의 FK → CSV → Julia 실행을 다시 확인했습니다.
`results/rb5/eye-in-hand-default01/`의 X/Y가 합성 정답과 일치하며,
정규화 gap은 약 2.90e-7, `numerically_certified=true`입니다.
실제 로봇 모델 일치 여부, 장치 통신 및 실물 정확도는 아직 확인되지 않았습니다.
`model_matches_robot=false`와 보드 실측 미확인 상태는 유지합니다.

## 이전 구성 기록: 손목 ArUco + 고정 ChArUco 타겟

- Python 테스트 11개 통과(skip 없음): ArUco 자세 복원, ChArUco 검출,
  같은 프레임의 두 패턴 관측, 고정 보드 누락 거부, 중복 마커 거부,
  보유 보드의 dictionary/ID 설정, 단위·좌표 합성, 이전 세션 호환성.
- Julia 테스트 11개 통과(무잡음 복원, 잡음, 수치 하한, 관측 불가능 입력 거부).
- 설치된 rbpodo의 `CobotData` 생성자와 `request_data` 시그니처,
  `time/robot_state/real_vs_simulation_mode/tcp_pos/jnt_ang` 바인딩 확인.
- `B = inverse(T_camera_charuco) × T_camera_marker`로 만든 합성 CSV → Julia
  → 결과 CSV를 통한 정답 비교 통과. `T_tcp_marker` 최대 원소 오차 약 4.79e-13,
  `T_base_charuco` 약 7.63e-14, 추가 계산한 `T_base_camera` 약 8.99e-14.
  정규화한 비용 gap 약 1.09e-7, `numerically_certified=true`.
- ArUco 인쇄 예시 `data/rb5/aruco-example/marker-print.svg` 생성 완료.
  DICT_6X6_250, ID 0, 검은 사각형 한 변 40mm는 미확정 예시 사양입니다.
- Python 문법, shell 문법, 변경 파일 whitespace 검사 통과.

현재 구성의 합성 결과는 `results/rb5/aruco-charuco-synthetic01/report.toml`에 있습니다.
합성 오차는 실제 장비의 정확도를 나타내지 않습니다.
카메라 행렬은 SDP 변수 Y와 관측된 ChArUco 자세에서 추가 계산한 값입니다.
초기 단일 ChArUco 구성의 합성 결과는 `results/rb5/synthetic01/`에 보존되어 있습니다.

## 아직 확인하지 못한 부분

샌드박스 밖 USB 진단에서도 RealSense 장치 목록이 비어 있었습니다.
로봇 IP가 아직 지정되지 않아 실제 RB 통신 및 정지 자세 수집은 실행하지 않았습니다.
현재 ArUco 실제 사양과 준비된 ChArUco 사양도 미정입니다.
실제 장비에서 TCP 표시값 비교, 같은 영상에서 두 패턴 검출,
잔차 및 별도 자세 검증이 필요합니다.

전체 시스템 Python의 `pip check`에는 별도로 설치되어 있던 PyNaCl/cffi 관련 문제가
표시됩니다. 본 캘리브레이션의 SDK import 및 테스트는 위와 같이 통과했으며,
관련 없는 시스템 Python 패키지는 변경하지 않았습니다.
