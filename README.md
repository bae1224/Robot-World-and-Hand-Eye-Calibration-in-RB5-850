# certifiable-rwhe-calibration

## RB5-850 + GoPro HERO13 Black: Eye-in-Hand RWHE 캘리브레이션

GoPro를 로봇 손목에 장착하고 기존 ChArUco 보드는 고정합니다.
GoPro로 저장한 **원본 사진 + 촬영 당시 RB5 상태 기록**을 연결하여 논문 솔버에 입력합니다.
[ROS 2 설치·실행](docs/RB5_ROS2_KO.md) · [GoPro 실행 안내](docs/RB5_GOPRO13_KO.md) · [GoPro 설정](hardware/rb5.gopro.json)

2026-10-27 재보정한 fisheye 결과(5568×4872 JPG, Wide, ISO 100–800)를 기본/로컬 GoPro 설정에 반영했습니다.
최종 테스트 16장 RMS는 0.768 px입니다. 보정과 동일한 작은 마커 검출 설정을 사용하고,
극주변 외삽을 피하기 위해 광축 75° 밖의 코너를 거절합니다.
[보정 파일·검증 범위](hardware/intrinsics/2026-10-27/README.md)를 확인하세요.
로봇 pose의 기본 입력은 패치한 `rbpodo_ros2`의 **실측 관절각 `jnt_ang` + URDF 정기구학(FK)**입니다.
`/rbpodo_hardware/measured_feedback`를 구독합니다. 공식 기본 joint_states의 `jnt_ref`는 사용하지 않습니다.
역기구학(IK)으로 촬영 당시 자세를 알아내는 방식이 아닙니다.
제어기의 활성 TCP 자세 `tcp_pos`를 직접 사용하는 옵션도 있습니다.

```bash
# ROS 2 안내에 따라 드라이버를 빌드·실행한 뒤, 모델/매핑과 보드 실측 정보를 설정
source scripts/ros2_env.sh
cp hardware/rb5.gopro.json hardware/rb5.local.json

# 로봇을 정지시키고 READY 표시 후 10초 안에 GoPro로 수동 촬영
/usr/bin/python3 -m hardware.rb5_gopro --config hardware/rb5.local.json record-pose \
  --sample-id pose001 --duration 10 --output data/rb5/gopro_raw/robot/pose001.json

# 여러 자세에서 반복 촬영한 뒤 사진 파일과 로봇 기록의 manifest 작성
/usr/bin/python3 -m hardware.rb5_gopro --config hardware/rb5.local.json import \
  --manifest data/rb5/gopro_raw/manifest.json --output data/rb5/gopro_session01

bash scripts/julia_rb5.sh scripts/calibrate_rb5.jl \
  data/rb5/gopro_session01/combined results/rb5/gopro_session01 eye_in_hand
```

Julia/RWHE 의존성 준비: `bash scripts/install_julia_rb5.sh`.
GoPro Python 의존성: `hardware/requirements-gopro.txt`. 실시간 기록에는 ROS 2 드라이버가 필요하며,
오프라인 사진 import에는 ROS·RealSense SDK·Python rbpodo가 필요하지 않습니다.
`A=T_base_hand`, `B=inverse(T_camera_board)`로 `AX=YB`를 풀어 `X=T_hand_camera`, `Y=T_base_board`를 추정합니다.
`T_parent_child`는 child 좌표를 parent 좌표로 옮기는 변환입니다.
결과: `T_hand_camera.csv`, `T_base_board.csv`, `residuals.csv`, `report.toml`.

기존 RealSense 실시간 수집 경로도 유지합니다:
[RealSense 실행 안내](docs/RB5_D435if_KO.md) · [기존 검증 기록](docs/RB5_VALIDATION.md).

Certifiably globally optimal generalized robot-world and hand-eye calibration. Our algorithm is the first extrinsic calibration method that can jointly solve for the poses of multiple sensors *and* targets, consider an unknown target scale, and provide a computational certificate of global optimality for its maximum likelihood objective function. 

## Usage
See `experiments/rw_multi_eye_multi_hand.jl` for an example of how to use our software on the processed real-world data in `data/real-world/`. Each `.csv` file in `data/real-world/combine/` contains $A$ or $B$ measuerement matrices for the tag and camera indexed in its filename. For example, the $i$ th row of `tag_0_cam_0_A.csv` contains 7 floating point numbers representing a pose matrix $A_i \in \mathrm{SE}(3)$ matrix in an $A_i X_0 = Y_0 B_i$ measurement for tag 0 and camera 0: the first four are $w$, $x$, $y$, and $z$ of a unit quaternion representing a rotation, and the last three are a position in $\mathbb{R}^3$.

##  Citation
If you use this work in your research, please cite the [following paper](https://arxiv.org/abs/2507.23045):

```bibtex
@article{wise2025certifably,
  title={A Certifably Correct Algorithm for Generalized Robot-World and Hand-Eye Calibration},
  author={Wise, Emmett and Kaveti, Pushyami and Chen, Qilong and Wang, Wenhao and Singh, Hanumant and Kelly, Jonathan and Rosen, David M and Giamou, Matthew},
  journal={arXiv preprint arXiv:2507.23045},
  year={2025}
}
```
