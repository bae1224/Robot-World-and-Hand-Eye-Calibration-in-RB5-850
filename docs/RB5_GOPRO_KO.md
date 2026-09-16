# RB5-850 + GoPro HERO13 + 캡처박스: Eye-in-Hand RWHE 캘리브레이션

RealSense 경로([RB5_D435if_KO.md](RB5_D435if_KO.md))는 그대로 둔 채, HDMI 캡처박스로
들어오는 UVC 카메라를 추가한 분기입니다. 설정 파일은 `hardware/rb5.gopro.json`입니다.

## 0. RealSense와 무엇이 다른가

가장 큰 차이는 카메라 종류가 아니라 **내부 파라미터의 출처**입니다.

| | RealSense D435if | GoPro HERO13 + 캡처박스 |
|---|---|---|
| K, 왜곡계수 | SDK가 제공 | **아무것도 없음 → 직접 캘리브레이션** |
| 왜곡 모델 | Brown-Conrady | 모드에 따라 fisheye(등거리) 또는 rational |
| 프레임 번호 | 있음 (staleness 검출) | 없음 → 큐 비우기로 대체 |
| 해상도 | SDK가 보장 | 박스가 다른 모드로 협상할 수 있음 → 검증 필요 |
| 프레임 내부 처리 | 없음 | **HyperSmooth / Horizon Lock → 반드시 OFF** |

## 1. 카메라에서 반드시 꺼야 하는 것

`camera.is_gopro=true`이면 다음을 `camera.acknowledgements`에 적기 전까지 수집이
거부됩니다.

- `stabilization_off` — HyperSmooth / 흔들림 보정
- `horizon_lock_off` — 수평 유지

이 둘은 **프레임마다 디지털 크롭과 회전을 겁니다.** 그러면 `T_hand_camera`(X)가
시간에 따라 변하므로 `AX = YB`의 강체 가정이 깨집니다. 문제는 이때도
재투영 오차, `AX-YB` 잔차, 최적성 인증서가 전부 정상으로 보인다는 점입니다.
이 파이프라인이 사후에 잡아낼 수 없는 유일한 고장이라 사전 확인으로 막습니다.

함께 확인할 것: 클린 HDMI(화면 오버레이 없음), 오토포커스/디지털 줌 없음,
해상도·FOV 모드 고정, 촬영 중 모드 변경 금지.

## 2. 내부 파라미터 캘리브레이션

ChArUco를 여러 각도·거리·화면 위치(특히 **가장자리와 모서리**)에서 15–30장 찍습니다.
캡처 체인은 실제 수집 때와 완전히 동일해야 합니다.

```bash
python3 -m hardware.rb5_camera --config hardware/rb5.gopro.json \
  intrinsics data/rb5/intrinsic-shots \
  --output hardware/intrinsics/gopro_hero13_linear_1080p.json --model fisheye
```

`--model`은 `fisheye`(Wide/SuperView 등 광각), `rational`(Linear 모드에서 fisheye가
과적합일 때), `pinhole`(좁은 화각) 중에서 고릅니다. 결과 JSON에는 K, 왜곡계수와 함께
**내부 파라미터 공분산**과 `focal_relative_sigma`가 기록됩니다.

출력에 이런 줄이 나옵니다.

```
Focal 1-sigma 0.184%, i.e. roughly 1.84 mm of hand-eye translation error per metre of working distance
```

이 환산이 핵심입니다. focal 상대오차 e는 PnP 깊이를 약 (1+e)배로 스케일하므로,
작업거리 d에서 X의 병진오차가 대략 `e × d` 만큼 생깁니다. 0.5 m에서 작업하면서
focal을 1% 틀리면 **약 5 mm**입니다. 그런데 재투영 오차는 거의 변하지 않아
`capture.max_reprojection_px` 게이트도, 전역 최적성 인증서도 이걸 잡아내지 못합니다.
(`tests/test_gopro.py::test_wrong_focal_length_biases_depth_and_passes_every_quality_gate`가
이 성질을 고정해 둡니다.)

`camera.intrinsics`에 이 파일 경로를 적으면 세션마다 SHA256이 `session.json`에
기록되어, 결과를 만든 내부 파라미터를 나중에 역추적할 수 있습니다.

## 3. 장치 확인

```bash
python3 -m hardware.rb5_camera --config hardware/rb5.gopro.json doctor --camera
python3 -m hardware.rb5_camera --config hardware/rb5.gopro.json doctor --robot
```

`doctor`는 백엔드에 맞는 패키지만 요구합니다(uvc 백엔드에서 pyrealsense2 불필요).
내부 파라미터 요약, 해상도 일치, GoPro 확인 항목, SHA256을 출력합니다.

`camera.device`는 캡처박스의 인덱스 또는 장치 경로입니다. Windows는
`"api": "msmf"` 또는 `"dshow"`, Linux는 `"v4l2"`가 안정적입니다.
`fourcc`는 가능하면 압축 없는 `"YUY2"`가 좋습니다. MJPEG 블록 아티팩트는
서브픽셀 코너 정확도를 직접 갉아먹습니다.

`drain_frames`는 캡처 전에 버릴 프레임 수입니다. 캡처박스는 프레임 번호를 주지
않으므로, 로봇이 자세에 도달하기 전의 오래된 프레임을 거르는 유일한 수단입니다.
기본값 6에서 시작해 박스 지연에 맞게 올리세요.

## 4. 수집과 계산

RealSense 경로와 동일합니다.

```bash
python3 -m hardware.rb5_camera --config hardware/rb5.gopro.json collect --output data/rb5/gopro01
bash scripts/julia_rb5.sh scripts/calibrate_rb5.jl data/rb5/gopro01/combined results/rb5/gopro01 eye_in_hand
```

Windows에서는 `powershell -ExecutionPolicy Bypass -File scripts\julia_rb5.ps1 ...`를
쓰면 됩니다.

## 5. 하드웨어 없이 검증하기

로봇과 카메라가 없어도 전체 경로를 실행할 수 있습니다. `simulate`는 가상의 광각
카메라로 ChArUco 인쇄물을 **실제로 렌더링**하고, 그 이미지를 실제 검출기와 PnP에
넣습니다. 기존 `synthetic`이 B를 대수적으로 만들어내는 것과 달리 렌즈 모델, 코너
검출, 서브픽셀 잡음까지 전부 지나갑니다.

```bash
python3 -m hardware.rb5_camera --config hardware/rb5.gopro.json \
  simulate --output data/rb5/sim01 --count 30
bash scripts/julia_rb5.sh scripts/calibrate_rb5.jl data/rb5/sim01/combined results/rb5/sim01 eye_in_hand
```

`--focal-error 0.01`처럼 주면 **참값 카메라로 렌더링하고 틀린 카메라로 푸는** 상황을
만들 수 있습니다. `ground_truth.json`에 참 X, Y와 참/가정 카메라가 모두 들어 있어
오차를 직접 잴 수 있습니다. `--principal-error-px`, `--distortion-error`도 같습니다.

## 6. 가중치: 상수에서 샘플별 정밀도로

논문 Problem 1은 측정마다 다른 κ_i, σ_i를 지원하는데 기존 RB5 경로는 상수
(κ=125, σ=0.01 m)를 썼고, 문서도 그것이 측정값이 아니라고 적고 있었습니다.

이제 각 뷰의 PnP Gauss-Newton 공분산에서 κ_i, τ_i를 뽑아 `combined/weights.csv`로
내보내고, `calibrate_rb5.jl`이 있으면 자동으로 사용합니다. 무시하려면
`--constant-weights`를 주거나 κ를 명시적으로 넘기면 됩니다.

```text
κ_i = 1 / (4 σ_R,i²)        σ_R: 회전 1-시그마 [rad]
τ_i = 1 / (2 σ_t,i²)        σ_t: 병진 1-시그마 [m], 역변환 지렛대항 포함
```

`1/(4σ²)`인 이유는 `src/calibration/robot_world_costs.jl`의 회전 블록이 식 (18b)의
1/2 없이 `κ·M'M`을 누적하기 때문입니다. 그래서 `report.toml`에는 넘긴 κ와 함께
논문 표기 기준값 `kappa_in_paper_notation = 2κ`를 같이 적습니다.

시뮬레이션(30자세, 120° fisheye, 1080p)에서 측정한 개선폭:

| 가중치 | X 오차 | Y 오차 |
|---|---|---|
| 상수 κ=125, σ=10 mm | 0.374 mm / 0.062° | 0.526 mm / 0.033° |
| 샘플별 PnP 공분산 | **0.141 mm / 0.042°** | **0.174 mm / 0.019°** |

한계는 문서에 적어둔 대로입니다. 논문 모델은 A(순기구학)를 무잡음으로 가정하므로
관절각·모델 오차가 B 쪽 잔차로 흘러 들어옵니다. 따라서 κ, τ의 **절대 크기**는
낙관적입니다. 추정값을 실제로 바꾸는 것은 샘플 간 **상대** 가중치이고, 그쪽은
영향을 받지 않습니다.

## 7. 솔버 견고성

COSMO의 기본 Ruiz 스케일링이 이 완화 문제의 일부 인스턴스에서 정체합니다.
실측성 데이터에서 `ITERATION_LIMIT`으로 끝나면서 feasibility 1e-4, dual slack -7e4를
내는데, 같은 문제가 스케일링을 끄면 0.1초 만에 1e-13으로 풀립니다.

그래서 `calibrate_rb5.jl`은 설정을 여러 개 시도하고, **솔버의 자기 보고 대신
독립적으로 재계산한 인증서**로 판정합니다. 반올림한 해가 원래 QCQP에 대해 실현
가능하고 그 비용이 유효한 쌍대 하한과 일치하면 그것이 인증서이며, ADMM 반복이
`OPTIMAL`을 선언했는지는 별개의 문제입니다. 사용된 설정은
`report.toml`의 `solver_configuration`, `solver_attempts`에 기록됩니다.

## 8. 아직 확인되지 않은 것

- 실제 HERO13, 실제 캡처박스, 실제 RB5 통신은 검증되지 않았습니다.
  이 브랜치의 모든 숫자는 시뮬레이션 결과입니다.
- `model_matches_robot=false`와 보드 실측 미확인 상태는 그대로입니다.
- `hardware/rb5.gopro.json`의 보드 사양(8×11칸, 40 mm)과 캡처 임계값은 예시입니다.
- HERO13의 각 FOV 모드에서 fisheye와 rational 중 무엇이 맞는지는 실제 캘리브레이션
  잔차로 비교해서 정해야 합니다.
