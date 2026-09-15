# certifiable-rwhe-calibration

## RB5-850 + RealSense D435if (Eye-to-Hand)

한국어 설치·보드 준비·실측 데이터 수집·논문 알고리즘 실행 안내:
[docs/RB5_D435if_KO.md](docs/RB5_D435if_KO.md).
기본 배치는 고정 카메라 + 로봇 TCP에 장착한 ChArUco 보드입니다.

```bash
bash scripts/install_rb5_local.sh
python3 -m hardware.rb5_realsense doctor
```

Python은 가상환경 없이 사용자 영역에 설치합니다. Julia 실행기는 원본의
목적함수와 SO(3) 제약식을 사용하며, 단일 카메라·보드의 known-scale 문제를 풉니다.

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
