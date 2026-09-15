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

## 통과한 검증

- Python 테스트 7개 통과(보드 영상 검출 및 metric PnP 포함, skip 없음).
- Julia 테스트 11개 통과(무잡음 복원, 잡음, 수치 하한, 관측 불가능 입력 거부).
- 설치된 rbpodo의 `CobotData` 생성자와 `request_data` 시그니처,
  `time/robot_state/real_vs_simulation_mode/tcp_pos/jnt_ang` 바인딩 확인.
- Python 합성 데이터 → CSV → Julia SDP → 결과 CSV를 통한 정답 비교 통과.
  `T_tcp_board` 최대 원소 오차 약 2.04e-9, `T_base_camera` 약 1.51e-10.
  정규화한 비용 gap 약 2.29e-11, `numerically_certified=true`.
- 인쇄용 `data/rb5/board/board-print.svg`와 미리보기 PNG 생성 완료.
- Python 문법, shell 문법, 변경 파일 whitespace 검사 통과.

합성 결과는 `results/rb5/synthetic01/report.toml`에 있습니다.
합성 오차는 실제 장비의 정확도를 나타내지 않습니다.

## 아직 확인하지 못한 부분

샌드박스 밖 USB 진단에서도 RealSense 장치 목록이 비어 있었습니다.
로봇 IP가 아직 지정되지 않아 실제 RB 통신 및 정지 자세 수집은 실행하지 않았습니다.
실제 장비에서 TCP 표시값 비교, RGB 스트림, 보드 검출, 잔차 및 별도 자세 검증이 필요합니다.

전체 시스템 Python의 `pip check`에는 별도로 설치되어 있던 PyNaCl/cffi 관련 문제가
표시됩니다. 본 캘리브레이션의 SDK import 및 테스트는 위와 같이 통과했으며,
관련 없는 시스템 Python 패키지는 변경하지 않았습니다.
