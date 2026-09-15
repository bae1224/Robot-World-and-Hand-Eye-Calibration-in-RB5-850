# certifiable-rwhe-calibration

## RB5-850 + RealSense D435if: Eye-in-Hand RWHE 캘리브레이션

카메라는 로봇 손목에 장착하고, ChArUco 타겟은 로봇 베이스에 대해 고정합니다.
기본·로컬 설정은 `mount=eye_in_hand`입니다.
로봇 자세는 측정 관절각 `jnt_ang` + 공식 `rb5_850e.urdf`로 계산합니다.
U 버전 모델은 포함하지 않습니다. ArUco를 사용하는 구성은 기본 경로에서 제외했습니다.
[한국어 실행 안내](docs/RB5_D435if_KO.md) · [검증 기록](docs/RB5_VALIDATION.md)

```bash
# 기존 설치로 실행 가능: 추가 패키지/가상환경 불필요
python3 -m hardware.rb5_realsense fk --joints-deg 0 0 0 0 0 0
```

`A X = Y B`로 손목-카메라와 베이스-타겟 변환을 동시에 추정합니다.
`T_parent_child`는 child 좌표를 parent 좌표로 옮기는 변환입니다.

| 항목 | 변환 | 입력/결과 |
|---|---|---|
| A | T_base_hand | 관절각 + URDF FK |
| B | inverse(T_camera_board) | ChArUco PnP 결과의 역변환 |
| X | T_hand_camera | 추정 결과 |
| Y | T_base_board | 추정 결과 |

`hardware/rb5.local.json`에 로봇 IP, 보유 ChArUco 사양·실측 크기를 입력하세요.
실제 로봇에 맞는 URDF 버전·기준 링크·관절 매핑도 확인해야 합니다. URDF의 `tcp` 링크와 제어기에
설정한 TCP가 같은 좌표계라고 가정하지 않습니다.

```bash
python3 -m hardware.rb5_realsense --config hardware/rb5.local.json collect --output data/rb5/session01
bash scripts/julia_rb5.sh scripts/calibrate_rb5.jl data/rb5/session01/combined results/rb5/session01 eye_in_hand
```

결과 확인: `T_hand_camera.csv`, `T_base_board.csv`, `residuals.csv`, `report.toml`

다른 노트북이나 pc 환경 에서 사용자 영역 설치:

```bash
bash scripts/install_rb5_local.sh
```

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
