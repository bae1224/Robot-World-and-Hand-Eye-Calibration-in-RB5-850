# 2026-10-27 GoPro intrinsic

기본 및 로컬 GoPro 설정이 이 폴더의 새 보정을 사용합니다.

| 파일 | 내용 |
|---|---|
| `fisheye.json` | 원본 JPG용 K/D와 보드·촬영·검출 설정 |
| `validation.json` | 최종 테스트 16장 PASS, RMS 0.768199 px; 보정 SHA256 연결 |
| `model_selection.json` | 모델 비교 세트 16장; 가장자리/coverage 기준 FAIL도 보존 |
| `model_domain.json` | 관측 각도 범위와 고차 왜곡 다항식의 단조성 한계 |
| `manifest.json` | 원본 위치, 파일 해시, 분할 및 적용 조건 |
| `integration_check.json` | 실제 JPG 97장에 대한 현재 GoPro 검출·pose 경로 검사 |

보정·검증·모델 비교·영역 진단 파일은 `/media/bae/Workspace/intrinsic/output/2026-10-27/`
원본과 바이트 단위로 동일합니다. 실행 시 이 외부 폴더가 없어도 됩니다.
기존 9월 파일은 상위 폴더에 유지했습니다.

5568×4872 원본 JPG, Wide, zoom 1.0 및 동일 렌즈/보드에 적용합니다.
사용자는 ISO 범위만 100–800으로 변경했다고 확인했습니다. GPR 현상 영상에는 적용을 검증하지 않았습니다.
새 설정은 `max_ray_angle_deg=75.0`, 개별 pose RMS 상한은 기존 1.0 px입니다.
최종 테스트가 PASS여도 모든 개별 사진이 이 pose 채택 기준을 통과하는 것은 아닙니다.
약 82.52°에서 모델의 단조성이 깨지므로 극주변까지 외삽하지 않습니다.

검출에는 보정 파일의 최소 마커 둘레 비율 0.008, threshold 최대 창 63 px,
기본 검출기 fallback을 사용합니다. 실험 보드와 보정 보드의 실제 패턴과 기하도 검사합니다.

모델 비교 세트의 FAIL과 최종 테스트의 PASS는 서로 다른 고정 세트의 결과입니다.
두 세트 모두 같은 촬영 세션이며, 픽셀 오차 결과는 실제 RB5 pose 정확도 또는
논문 solver의 측정 정확도 보증을 의미하지 않습니다.

자세한 사용법: [GoPro 실행 안내](../../../docs/RB5_GOPRO13_KO.md).

현재 통합 검사: 97장 모두 코너 ID/좌표가 intrinsic과 정확히 일치했고, 개별 pose는 88장 채택 / 9장 거절입니다.
RMS 초과 7장, 각도 초과 1장, 면적 부족 1장이며, Python 테스트 34개를 통과했습니다.
