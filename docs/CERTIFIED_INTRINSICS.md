# 내부 파라미터까지 인증 가능한 RWHEC: 유도와 검증

논문은 intrinsic 추정을 명시적으로 범위 밖으로 둡니다(Section 2.1, Evangelista et al.
2023을 인용하며 "outside the scope of this work"). 이 문서는 그 빈칸을 어떻게 메울 수
있는지, 어디까지가 증명 가능하고 어디부터가 경험적인지를 구분해서 적습니다.

수치는 전부 이 저장소의 `simulate` 경로에서 재현됩니다.
검증 스크립트는 `experiments/radial_identifiability.py`입니다.

정리와 증명을 갖춘 조판본: **[certified_intrinsics_derivation.pdf](certified_intrinsics_derivation.pdf)** (10쪽).
원본은 같은 폴더의 `.html`이며, 다음으로 다시 생성합니다.

```bash
chrome --headless=new --no-pdf-header-footer --virtual-time-budget=25000 \n  --print-to-pdf=docs/certified_intrinsics_derivation.pdf \n  file://$PWD/docs/certified_intrinsics_derivation.html
```

---

## 0. 왜 이 문제를 풀어야 하는가

인증서는 내부 파라미터 오차를 **구조적으로** 볼 수 없습니다. 시뮬레이션
(1080p, 120° fisheye, 30자세, 작업거리 0.55 m):

| focal 오차 | X 오차 | 인증서 | normalized gap | 검출기 RMS | AX−YB 잔차 |
|---|---|---|---|---|---|
| 0.0 % | 0.14 mm | true | 3.6e−08 | 0.189 px | 0.70 mm |
| 0.5 % | 1.00 mm | true | 6.3e−09 | 0.198 px | 0.82 mm |
| 1.0 % | 2.01 mm | true | −2.6e−09 | 0.219 px | 1.15 mm |
| 2.0 % | 4.02 mm | true | 1.4e−10 | 0.277 px | 1.97 mm |
| 5.0 % | 9.54 mm | true | 5.5e−11 | 0.487 px | 4.55 mm |

이유는 명확합니다. focal 오차는 B에 **등방성 잡음이 아니라 계통 편향**으로 들어갑니다.
목적함수는 주어진 B에 대해 여전히 정확히 최소화되므로 gap은 1e−9로 닫히고,
재투영 오차는 focal이 5% 틀려도 0.49 px에 머물러 게이트를 통과합니다.

**최적성(optimality)과 정확도(accuracy)는 다른 질문이고, 인증서는 앞의 것만 답합니다.**
이 구분이 아래 전체 설계의 출발점입니다.

---

## 1. 그대로는 왜 안 되는가

논문의 인증 기계는 **QCQP**를 요구합니다. 2차 목적함수 + 2차 제약(SO(3)),
Shor relaxation → SDP, 여기에 Cifuentes et al. (2022) Thm 3.9(논문 Thm 5)의 파라메트릭
tightness와 redundant SO(3) 제약에 대한 ACQ(논문 Thm 6)가 얹힙니다.

재투영 오차는 이 구조를 세 곳에서 깨뜨립니다.

1. **(O1) 깊이로 나눔.** π(P) = (f_x P_x/P_z + c_x, …)는 유리함수입니다.
2. **(O2) 미지수끼리의 곱.** 분모를 없애도 f·(RP+t) 항이 남아 차수가 올라갑니다.
3. **(O3) 자세 자체가 곱.** eye-in-hand에서 카메라-보드 자세는
   `T_cb,i = X⁻¹ A_i⁻¹ Y` — 미지수 두 개의 곱입니다.

(O3)만 놓고 보면 논문이 이미 해결한 문제입니다(2차 항으로 흡수됨).
진짜 문제는 (O1)과 (O2), 즉 **내부 파라미터가 들어오는 순간**입니다.

---

## 2. 핵심 관찰: 잔차를 radial과 tangential로 쪼갠다

주점 c에 대해 ū = u − c라고 둡니다. **중심 대칭 왜곡을 가진 임의의 중심 카메라**에서
3차원 점 P = (P_x, P_y, P_z)의 상은 다음을 만족합니다.

```
ū  ∥  (P_x, P_y)                                            ... (★)
```

즉 **ū의 방향은 (P_x, P_y)의 방향과 같습니다.** ū의 크기 |ū| = ρ(θ)는 초점거리와 왜곡
프로파일에 의존하지만, **방향은 의존하지 않습니다.**

(★)이 성립하는 범위: pinhole + Brown radial, Kannala-Brandt equidistant,
division model, 그리고 **형태를 모르는 임의의 단조 radial 프로파일**까지 전부.

스칼라 방정식으로 쓰면

```
r = ū⊥ᵀ (P_x, P_y)ᵀ = ū_x P_y − ū_y P_x = 0,     ū⊥ = (−ū_y, ū_x)
```

이것이 1D radial camera 모델입니다(Thirthala & Pollefeys 2005; Camposeco et al.;
Larsson et al.). **P = R_c p + t_c는 (R_c, t_c)에 선형이므로, r은 선형이고 r²은 2차입니다.**
QCQP가 보존됩니다.

### 2.1 RWHEC 루프에 넣기

`T_cb,i = X⁻¹ A_i⁻¹ Y`를 전개하면

```
P_ij = R_Xᵀ R_Aiᵀ ( R_Y p_j + t_Y − t_Ai ) − R_Xᵀ t_X
```

이건 (R_X) × (R_Y, t_Y)의 **쌍선형**이라 P 자체가 2차, r은 2차, r²은 4차입니다.
그대로는 QCQP가 아닙니다.

**왼쪽에서 R_X를 곱해 풉니다.** w = (−ū_y, ū_x, 0)ᵀ ∈ R³로 두면

```
r_ij = wᵀ P_ij = wᵀ R_Xᵀ [ R_Aiᵀ ( R_Y p_j + t_Y − t_Ai ) − t_X ]
     = (R_X w)ᵀ [ R_Aiᵀ ( R_Y p_j + t_Y − t_Ai ) − t_X ]
```

대괄호 안은 (R_Y, t_Y, t_X)에 **선형**이고, 앞은 R_X에 **선형**입니다.
따라서 r_ij는 x = [vec(R_X); vec(R_Y); t_X; t_Y; s]의 **2차형식**이고, r²은 4차입니다.

4차를 2차로 되돌리는 표준 수단이 **lifting**입니다. 관측마다 스칼라 z_ij를 도입하고

```
minimize   Σ λ_ij z_ij²
s.t.       z_ij = (R_X w_ij)ᵀ [ R_Aiᵀ ( R_Y p_j + t_Y − t_Ai ) − t_X ]    (2차 등식)
           R_X, R_Y ∈ SO(3)                                               (논문 식 19)
```

목적함수 2차, 제약 2차 → **QCQP 복귀.** 논문의 SDP, 쌍대 인증서, ACQ 정리(Thm 6은
"SO(3)/SE(3)만 제약된 임의의 QCQP"에 적용된다고 명시)가 그대로 적용됩니다.
추가 변수 z는 **무제약**이라 ACQ의 Jacobian rank 논증을 해치지 않습니다.

**규모 문제와 해법.** z가 관측당 하나라 30뷰×70코너 = 2100개가 늘어납니다. 밀집
SDP로는 불가능하지만, 희소성 패턴이 **화살촉(arrowhead)** 구조입니다. 19×19 코어
블록에 크기 20짜리 clique 2100개가 붙는 형태이고 이는 **chordal**입니다. COSMO의
chordal decomposition이 정확히 이 경우를 위한 기능입니다.

**더 나은 대안 — order-2 moment relaxation.** lifting 대신 4차 다항식 최적화로 직접
보면, 변수 19개에 대한 2차 moment matrix 크기는 C(21,2) = **210**입니다. SO(3) 제약의
localizing matrix는 20×20. 210×210 SDP는 매우 가볍습니다. lifting(2120)보다 훨씬
낫습니다. tightness는 실증 문제지만 redundant SO(3) 제약이 유리하게 작용합니다.

---

## 3. 정확한 불가관측 방향 (핵심 결과)

radial 제약만으로 (X, Y)가 결정되는지가 다음 질문입니다. 답은 **"하나 빼고 전부"**이고,
그 하나는 정확히 특정할 수 있습니다.

> **명제.** T_z(δ)를 카메라 z축 방향 δ 평행이동이라 하자. 임의의 δ에 대해
> radial 잔차 r_ij는 X → X·T_z(δ)에 대해 **정확히 불변**이다.

**증명.**
```
T_cb,i(X·T_z(δ), Y) = (X T_z(δ))⁻¹ A_i⁻¹ Y = T_z(−δ) X⁻¹ A_i⁻¹ Y = T_z(−δ) T_cb,i(X, Y)
```
따라서 P_ij → P_ij − δ e₃ 이고 (P_x, P_y)는 변하지 않는다. r은 (P_x, P_y)에만
의존하므로 불변이다. ∎

X = [R_X | t_X]에 대해 X·T_z(δ) = [R_X | t_X + δ R_X e₃]이므로, 불가관측 방향은
**t_X가 R_X e₃(= 카메라 광축의 hand 좌표 표현) 방향으로 움직이는 1차원**입니다.
R_X도 Y도 영향받지 않습니다.

### 3.1 수치 검증

`experiments/radial_identifiability.py`, 30뷰 / 1674개 코너 관측:

```
카메라를 자기 광축 방향으로 밀었을 때 radial 잔차의 최대 변화
    delta = 1 mm     0.000e+00
    delta = 10 mm    2.163e-15
    delta = 100 mm   0.000e+00
    delta = 500 mm   2.163e-15          <- 잔차 자체의 rms는 2.834e-03
비교: 카메라 x축으로 100 mm     1.002e+00  (명백히 관측됨)

수치적 영벡터와 R_X e₃ 사이 각도 : 0.010 deg
radial Jacobian 조건수, 12 dof   : 519426.6
radial Jacobian 조건수, 11 dof   :     226.6
전체 재투영 Jacobian 조건수       :     134.2
```

**500 mm를 밀어도 잔차가 기계정밀도로 변하지 않습니다.** 근사적 퇴화가 아니라
정확한 불변입니다. 그리고 그 한 방향을 제거하면 조건수가 519,000 → **226.6**으로
떨어져 전체 재투영 잔차의 134.2와 같은 급이 됩니다. 즉 **나머지 11 dof는 잘 조건화되어
있고, 나쁜 것은 정확히 하나뿐**입니다.

### 3.2 왜 이게 좋은 소식인가

사라진 자유도는 **초점거리가 공급하는 바로 그 자유도**입니다. f를 (1+e)배 하면 모든
깊이가 (1+e)배 되므로, depth-focal 모호성과 radial 불가관측성은 같은 1차원입니다.
문제가 분리된다는 뜻입니다.

### 3.3 gauge fixing은 QCQP를 깨지 않는다

논문 Prop. 7은 **유일한** 최소점을 요구합니다. 1차원 해 다양체가 있으면 SDP 해가
rank-1이 아니게 되어 `sdp_rank_ratio` 검사가 실패합니다. 제약 하나를 추가해 고정합니다.

```
(R_X e₃)ᵀ t_X = γ
```

R_X e₃는 vec(R_X)의 7~9번째 성분이므로 이 식은 **x에 대한 2차형식**입니다.
QCQP가 유지되고, orbit에 정확히 횡단적이라 완전한 gauge fix입니다.

**게다가 γ를 바꿔 다시 풀 필요가 없습니다.** 비용이 δ에 대해 불변이므로,
**SDP 한 번이 전역 최적해의 1-파라미터 족 전체를 줍니다.** γ는 사후에 슬라이딩으로
얻습니다.

---

## 4. Stage B: 남은 자유도는 닫힌 형식에 가깝다

Stage A가 (R_X, R_Y, t_Y)와 t_X를 광축 방향 δ만 남기고 확정했습니다.
이제 버렸던 정보, 즉 **radial 크기** |ū| = ρ(θ)를 씁니다.

Kannala-Brandt: `r = f(θ + k₁θ³ + k₂θ⁵ + k₃θ⁷ + k₄θ⁹)`.
θ는 δ가 정해지면 확정되고, r은 **(f, f k₁, …, f k₄)에 선형**입니다.
따라서 δ 하나만 탐색하면 되고 안쪽은 최소자승 닫힌 형식입니다. **1차원 전역 문제.**

`experiments/radial_identifiability.py`의 Stage B 검증:

```
   delta [mm]   RMS [px]     f [px]
       -150     7.1477    1166.12
        -40     2.2406     983.83
        -10     0.6350     934.13
         -1     0.3378     919.22     <- 최소
          0     0.3434     917.57
         10     0.7614     901.01
         40     2.7079     851.36
        150    12.4270     669.98

복원된 delta : -1.000 mm  (참값 0)
복원된 f     : 919.22 px  (참값 916.73)  상대오차 +0.27 %
```

**δ가 실제로 고정됩니다.** ±150 mm에서 RMS가 12 px까지 오르므로 평평한 방향이
아닙니다. 다만 최소점 근처는 얕습니다(−1 mm에서 0.3378, 0 mm에서 0.3434).
이 얕음이 곧 고전적인 depth-focal 모호성이며, 이 설계의 가치는 그것을
20차원 번들 조정 안에 숨기는 대신 **1차원의 명시적이고 전역 탐색 가능한 양으로
끄집어낸다**는 데 있습니다.

f의 0.27% 오차는 위 표 기준으로 X에서 약 0.5 mm에 해당합니다.

---

## 5. 전체 알고리즘

```
입력: 이미지별 코너 픽셀 {u_ij}, 로봇 자세 {A_i}, 보드 점 {p_j}
      내부 파라미터 정보 없음

1. 주점 c를 초기화한다 (이미지 중심 또는 개략 캘리브레이션).
2. Stage A — radial QCQP:
     min Σ λ_ij z_ij²
     s.t. z_ij = (R_X w_ij)ᵀ[R_Aiᵀ(R_Y p_j + t_Y − t_Ai) − t_X]
          R_X, R_Y ∈ SO(3),  (R_X e₃)ᵀ t_X = 0
   → 논문의 SDP + 쌍대 인증서. f도 왜곡도 전혀 사용하지 않음.
   → (R_X, R_Y, t_Y) 인증된 전역 최적, t_X는 광축 방향 1자유도 남김.
3. Stage B — δ에 대한 1차원 전역 탐색, 안쪽은 (f, k₁..k₄) 선형 최소자승.
   → f, 왜곡, δ 확정 → X 완성.
4. (선택) c에 대한 2차원 외부 루프로 2~3 반복. 각 내부 문제가 전역 최적이므로
   branch-and-bound로 c까지 전역 보장 가능.
5. (선택) Stage C — 전체 재투영 목적함수로 국소 번들 조정. 2~4의 결과를 시드로.
```

**보장의 정확한 범위:**

| 자유도 | 보장 |
|---|---|
| R_X, R_Y, t_Y, t_X의 11 dof | 고정된 c에 대해 **인증된 전역 최적** (논문 기계 그대로) |
| f, k₁..k₄ | 고정된 δ에 대해 **닫힌 형식 전역 최적** |
| δ (1차원) | 조밀 탐색 + Newton. Lipschitz 구간 한계로 **전역 보장 가능** |
| c (2차원) | branch-and-bound로 **전역 보장 가능**, 단 내부 SDP 재해가 필요 |

즉 모든 자유도가 전역적이고, 비싼 부분은 19×19 급 SDP에 머뭅니다.
기존 joint intrinsic+extrinsic 방법(전부 국소 BA)보다 훨씬 강한 주장입니다.

**정직한 한계:**

- Stage A는 radial 성분만 쓰므로 **통계적으로 비효율적**입니다. 점마다 잔차의 두
  성분 중 하나를 버립니다. 조건수 비교(226.6 대 134.2)상 기하학은 멀쩡하지만,
  같은 데이터에서 full BA보다 분산이 큽니다. Stage C가 이를 회수합니다.
- 주점 근처(|ū| → 0) 점은 w → 0이라 정보를 주지 않습니다. 보드가 화면
  **가장자리**를 덮어야 합니다 — 왜곡 캘리브레이션에서 어차피 필요한 조건입니다.
- Stage C를 돌리면 그 결과 자체는 인증되지 않습니다. 인증된 시드에서 출발했고
  이동량이 작다는 논증은 강력하지만 엄밀하지 않습니다.
- Stage A의 relaxation이 tight한지는 **실증 문제**입니다. 논문의 Prop. 7이 저잡음에서
  보장하지만 임계 잡음 수준은 이 목적함수에 대해 다시 재야 합니다.

---

## 6. 당장 쓸 수 있는 부분: 인증 반경과 민감도

전체 재설계 없이도 지금 코드에서 나오는 두 숫자가 있습니다.

### 6.1 tightness 반경 τ_K (Weyl)

K̂에서 쌍대 인증서 S(λ̂) = Q(θ̂) − Σλ̂ᵢCᵢ ⪰ 0 이고 (알려진 kernel 방향을 제외한)
최소 고유값이 μ > 0이라 하자. 이 μ가 `report.toml`의 `dual_slack_min_eigenvalue`
입니다. Weyl 부등식에 의해 ‖ΔQ‖₂ < μ 이면 **같은 λ̂이 섭동된 문제도 인증**합니다.

```
‖ΔQ‖ ≤ L·‖ΔK‖,  L = ‖∂Q/∂K‖  (조립된 Q에 유한차분)
⟹  τ_K = μ / L
```

보고 형태: "‖K − K̂‖ ≤ τ_K 인 모든 K에 대해 전역 최적."
이를 내부 파라미터 캘리브레이션의 σ_K(이제 `intrinsics` 명령이 계산합니다)와 비교해
τ_K ≫ 3σ_K이면 인증이 캘리브레이션 불확실성에 견고하다고 말할 수 있습니다.

이것은 논문 Prop. 7 / Cifuentes Thm 3.9의 정성적 이웃 진술을
**계산 가능한 반경**으로 바꾼 것입니다. θ = vec({A_i, B_i})이고 B_i(K)가 K에 대해
매끄러우므로(PnP 정규방정식에 음함수 정리) θ̄ 이웃의 tightness가 K̄ 이웃으로 당겨집니다.

### 6.2 정확도 민감도 — 이미 구현했습니다

`intrinsic_sensitivity` (scripts/calibrate_rb5.jl). focal 상대오차 e는 모든 PnP 깊이를
약 (1+e)배로 스케일하고 bearing은 건드리지 않으므로, 데이터셋 차원에서는 **모든 B_i의
병진 성분 스케일링**과 정확히 같습니다. 그래서 ±σ에서 두 번 다시 풀면 됩니다.

```
Focal 1-sigma (0.2%) moves X by 0.532 mm, 0.0045 degrees -- accuracy, not optimality
```

경험적 스윕(0.2% 주입 → X 0.40 mm)과 잘 맞고 약간 보수적입니다.
**τ_K와 이 숫자는 다른 질문에 답합니다.** τ_K는 "SDP가 여전히 정확히 푸는가",
이 숫자는 "답이 얼마나 움직이는가"입니다. 논문을 쓴다면 이 둘을 섞지 않는 것이
가장 중요합니다.

---

## 7. 권장 순서

1. **(완료)** 시뮬레이션 기반 검증 경로, 샘플별 가중치, 솔버 견고성, 민감도 보고.
2. τ_K 구현 (§6.1). `dual_slack_min_eigenvalue`와 ∂Q/∂K만 있으면 되므로 작습니다.
   여기까지가 첫 결과물로 충분합니다.
3. Stage A 프로토타입: radial QCQP를 order-2 moment relaxation으로 먼저 구현하고
   (210×210), 참값 시뮬레이션에서 tightness를 확인.
   §3의 gauge fix가 rank-1을 회복시키는지 검증.
4. Stage B + δ 1차원 탐색을 붙여 end-to-end 정확도를 full BA와 비교.
5. c에 대한 branch-and-bound로 전역 보장을 닫음.

### 참고 문헌

- Cifuentes, Agarwal, Parrilo, Thomas (2022), *On the local stability of semidefinite
  relaxations* — 논문 Thm 5의 출처. §6.1의 근거.
- Thirthala & Pollefeys (2005), *The radial trifocal tensor* — 1D radial 카메라.
- Camposeco, Sattler, Pollefeys; Larsson et al., *Revisiting radial distortion absolute
  pose* — radial 제약 기반 최소 해법.
- Dümbgen, Holmes, Barfoot, *Toward globally optimal state estimation using automatically
  tightened semidefinite relaxations* (AutoTight / AutoTemplate) — lifting한 정식화에서
  어떤 redundant 제약이 tightness에 필요한지 **자동으로 찾아줍니다.** §2의 z 리프팅에
  가장 필요한 도구. 논문도 Dümbgen et al. 2023을 인용합니다.
- Fitzgibbon (2001), division model — 왜곡을 다항식으로 유지하는 표준 수단.
- Yang & Carlone, certifiable perception / TEASER — 이상치 대응이 필요해질 때.
- Evangelista et al. (2023) — 재투영 기반 generalized RWHEC, 국소 방법. 비교 대상.
