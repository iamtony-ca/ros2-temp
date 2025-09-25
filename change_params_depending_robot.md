좋아, 파라미터 전체를 훑어보면서 “로봇의 가로/세로/바퀴간격 등 치수 변화”가 생기면 꼭 손봐야 하거나 영향을 크게 받는 것들만 콕 집어 체크리스트로 정리해줄게. (ROS 2 Jazzy / Nav2 전제)

# 1) 코스트맵(가장 중요)

## 공통 개념

* **로봇 외곽 모델**은 `robot_radius` **또는** `footprint`(다각형) 중 하나로 정의합니다.

  * 원형 근사로도 충분하면 `robot_radius`만 쓰면 됨.
  * 직사각형/비대칭/게발형 등 특이 형상 → **다각형 `footprint` 전환** 권장.
* **inflation_layer의 inflation_radius**는 보통
  `inflation_radius ≥ inscribed_radius + safety_buffer`
  로 설정합니다. 로봇이 커지면 최소한 inscribed radius만큼은 같이 키워야 충돌 여유가 생겨요.
* **footprint_padding**(기본 0.01 m): 안 쓰고 있지만, 여유를 추가하고 싶으면 사용.

## 로컬 코스트맵 `local_costmap.local_costmap.ros__parameters`

* `robot_radius: 0.22`  ← 로봇이 커지거나 모양이 바뀌면 **반드시 수정**.

  * 다각형으로 바꿀 땐:

    ```yaml
    # robot_radius 제거하고 다음 추가
    footprint: [[x1,y1], [x2,y2], [x3,y3], [x4,y4]]
    footprint_padding: 0.01   # 선택
    ```
* `inflation_layer.inflation_radius: 0.70`  ← **로봇 반경 증가 시 상향**.

  * 예: 로봇 inscribed radius가 0.35m로 커졌고 안전여유 0.25m 원하면 → 0.60m 이상.
* `voxel_layer` / `obstacle_layer`는 직접적인 “크기”보다는 센서와 높이 범위에 영향.

  * 다만 `CostCritic`/컨트롤러가 footprint를 고려하도록 바꿀 경우, **충돌 주의 범위**가 footprint 기반이니 inflation/장애물 반영이 더 타이트해짐. 인접 장애물에서 빈번히 정지하면 **inflation_radius를 약간 늘리거나 cost_scaling_factor 조정** 고려.
* `width: 3`, `height: 3`  ← 로봇이 커져서 회전/후진 시 시야가 좁다면 **확대**(예: 4~5 m). 특히 긴 플랫폼은 회전 반경이 커져 전방만 보이면 늦게 감지합니다.
* `resolution: 0.05`  ← 로봇이 커지면 상대적으로 거친 해상도로도 괜찮지만, **협소 공간 통과가 중요**하면 더 **미세(0.03 등)** 로 조정 가능. (해상도 변경은 전체 계획/메모리 비용 영향)
* `static_layer` 그대로. (맵 자체와 크기는 무관)
* `filters.keepout_filter` 자체는 무관하나, 로봇이 커지면 **keepout 마스크 설계**에서 통로 폭 유효성 재점검 필요.

## 글로벌 코스트맵 `global_costmap.global_costmap.ros__parameters`

* `robot_radius: 0.22`  ← 로컬과 **동일하게 갱신**(또는 동일 footprint).
* `inflation_layer.inflation_radius: 0.7`  ← 위와 동일 원칙으로 **상향 필요**.
* `resolution: 0.05`  ← 협소구역이 많고 로봇이 커졌는데 길찾기가 자꾸 실패하면 **미세화** 고려.
* `track_unknown_space: true` 등은 무관.
* `filters`(keepout/speed) 동작 논리는 동일하지만 **유효 통로 폭**이 줄어드는 만큼 마스크/속도존 설계는 재검토.

# 2) 컨트롤러(MPPI)와 트래젝토리 검증

현재 설정상 **충돌 판단에서 footprint를 무시**하고 있어요. 로봇이 커지면 반드시 바꾸는 걸 추천합니다.

* `controller_server.FollowPath.TrajectoryValidator.consider_footprint: false`
  → **true** 로 변경 권장. (충돌 검증이 **원/footprint** 기준으로 정확)
* `controller_server.FollowPath.CostCritic.consider_footprint: false`
  → **true** 로 변경 권장. (비용 평가 시 footprint 반영)
* `controller_server.FollowPath.TrajectoryValidator.collision_lookahead_time: 2.0`
  로봇이 길어지고 속도도 유지한다면 **lookahead 시간↑**(예: 2.5~3.0s)로 더 먼 충돌을 선제 회피.
* `controller_server.FollowPath.prune_distance: 1.7`
  로봇 길이/관성 증가로 급선회가 어려우면 약간 **증가**해 경로 따라가기의 안정성 확보.
* `controller_server.FollowPath.PathAlignCritic.max_path_occupancy_ratio: 0.05`
  footprint를 고려하기 시작하면 경로 점유율이 쉽게 상승 → **완화(예: 0.08~0.12)** 고려.
* `controller_server.FollowPath.*Critic*.threshold_to_consider / offset_from_furthest / max_angle_to_furthest`
  로봇이 커지면 경로 중심선과 자세 정합에 더 민감해져 잦은 페널티가 걸릴 수 있습니다.
  → 필요 시 **threshold 완화**(0.1~0.3 정도) 또는 **offset 튜닝**.

> 참고: `motion_model: "DiffDrive"` 는 차체 크기와 무관. 다만 차체가 길어 turning in place 안정성이 떨어지는 환경이면 `wz_max(=1.9)`를 약간 낮추는 식으로 **동역학적 파라미터**를 따로 조정.

# 3) 플래너/스무더

* **플래너는 코스트맵 footprint/inflation을 그대로 사용**하므로 보통 별도 수정 없이 코스트맵만 맞추면 됨.
* `planner_server.GridBased.tolerance: 0.5`
  로봇이 커져 **목표점 근접이 어려워지면 약간 완화**(예: 0.6~0.8).
* `smoother_server.simple_smoother.enforce_path_inversion` 등은 크기보다는 주행 특성 문제. 좁은 곳에서 큰 로봇이 급커브를 많이 만들면 `refinement_num`을 늘려 **더 부드러운 경로** 유도.

# 4) Goal / Progress 체크

* `controller_server.general_goal_checker.xy_goal_tolerance: 0.25`
  로봇이 커지고 정확히 포즈를 맞추기 어려워지면 **완화**(예: 0.30~0.40).
  반대로 **도킹처럼 정밀** 필요 시는 **축소**.
* `controller_server.progress_checker.required_movement_radius: 0.5`
  큰 로봇이 좁은 구역에서 움직임이 작을 수 있음 → **약간 완화**(0.6~0.8)로 불필요한 실패 방지.

# 5) Collision Monitor / Behavior 서버 / 도킹

* `collision_monitor.FootprintApproach.footprint_topic: local_costmap/published_footprint`
  → **핵심**: 코스트맵의 footprint(또는 robot_radius) 변경이 여기에도 자동 반영됩니다.

  * `min_points: 6` 은 footprint 다각형 분해 최소점수. 복잡한 다각형이면 늘려도 OK.
  * `time_before_collision: 1.2` 는 로봇이 커져 **안전여유 더 필요**하면 1.5~2.0로.
* Behavior Server는 footprint 토픽만 참조. **별도 치수 파라미터 없음**.
* **도킹**

  * `docking_server.controller.dock_collision_threshold: 0.3`
    큰 플랫폼은 여유 좁으면 충돌 false positive 증가 → **완화/강화** 재조정.
  * `staging_x_offset: -0.7` 및 `external_detection_*` 오프셋은 **차체 길이 변화**에 맞게 재산정 필요.
  * `v_linear_max: 0.15` 등 속도는 크기 자체보다는 안전 여유와 관성에 의해 조절.

# 6) 그 외(상대적으로 영향 적음)

* `velocity_smoother`의 max/min vel/acc/dec 값은 **크기보다 동역학/안전정책**과 관계. 차체가 커지면서 질량/관성 증가로 **가감속 한계**를 낮출 필요는 있을 수 있음.
* 센서 레이어(`raytrace_max_range`, `obstacle_max_range`)는 크기와 직접 상관 없지만, 큰 로봇이 좁은 곳을 지나면 **조금 더 먼 레이 추적**이 유리할 때가 있어요.

---

## 실전 적용 순서(추천)

1. **footprint 정의부터 확정**

   * 원형 유지 시 `robot_radius`만 수정.
   * 다각형 예:

     ```yaml
     local_costmap:
       local_costmap:
         ros__parameters:
           footprint: [[0.40,0.30], [0.40,-0.30], [-0.40,-0.30], [-0.40,0.30]]
           footprint_padding: 0.02
     global_costmap:
       global_costmap:
         ros__parameters:
           footprint: [[0.40,0.30], [0.40,-0.30], [-0.40,-0.30], [-0.40,0.30]]
           footprint_padding: 0.02
     ```
2. **inflation 재조정**

   * `inflation_radius`를 inscribed radius + 안전여유(0.2~0.4 m 등)로.
3. **MPPI에서 footprint 고려 활성화**

   * `TrajectoryValidator.consider_footprint: true`
   * `CostCritic.consider_footprint: true`
4. **로컬 코스트맵 가시창 확대(필요 시)**

   * `width/height`를 3→4~5 m
5. **Goal/Progress/Planner tolerance 경미 조정**
6. **Collision Monitor 시간·임계값 재조정**, 도킹 오프셋/임계값 보정

필요하면, 너의 새 치수(길이·폭·코너 라운딩 등)를 알려줘. 위 체크리스트에 맞춰 **수정된 YAML 블록**을 바로 만들어 줄게.
############################  
좋아. “로봇 외형/축간거리/회전반경”이 바뀔 때 **직접적으로 재튜닝 영향이 큰 항목만** 콕 집어 줄게. (코스트맵 `robot_radius/footprint + inflation`은 전제 조건이므로 이미 맞춰둔 것으로 가정)

---

# MPPI (FollowPath1)

**키네마틱/충돌 관련**

* `TrajectoryValidator.consider_footprint: false` → **true 권장**
  로봇이 커지면 최적 궤적 충돌판정에 footprint 반영이 필수.
* `CostCritic.consider_footprint: true`(이미 true): 유지.
  큰 footprint로 인해 비용 급증 시 `cost_weight`를 **소폭 완화(예: 3.81→3.0~3.5)** 가능.
* `TrajectoryValidator.collision_lookahead_time: 2.0` → **(로봇길이 L, 최고속 v) 에서**
  `max(2.0,  (L*1.5)/v )` 정도로 상향. 길고 빠르면 2.5~3.0 s 권장.

**회전/기동성**

* `AckermannConstraints.min_turning_r: 0.2`
  실차 최소 회전반경 $R_\min$으로 갱신. **DiffDrive라도 실제 회전 공간 확보 목적**으로 유효.
  예) 길어진 로봇에서 in-place 회전이 충돌 위험이면 `R_\min ≈ inscribed_radius + 여유`.
* `wz_max: 1.9` / `az_max: 3.5`
  큰 차체(관성↑)면 **감소**(예: 1.2~1.6, 2.0~3.0)해서 급회전 억제.
* `vx_max / ax_max / ax_min / ay_*`
  질량↑/무게중심↑ → **가감속 한계 하향**(안전·슬립 방지).
  예) `vx_max: 0.5→0.35~0.45`, `ax_max: 3.0→2.0~2.5`.

**경로정렬/추종 크리틱**

* `PathAlignCritic.max_path_occupancy_ratio: 0.05`
  footprint 면적↑로 점유율 페널티 과도 → **0.08~0.12** 완화.
* `PathFollowCritic/PathAngleCritic.threshold_to_consider`
  큰 차체에서 미세 오차 페널티가 잦으면 **+0.1~0.3** 완화.

**샘플링/안정성**

* `prune_distance: 1.7`
  길이 L↑일수록 급한 약간의 변화가 부담 → **L의 2~3배** 수준으로 미세 상향(예: 2.0~2.5).
* `regenerate_noises: false`
  큰 차체 + 좁은 통로에서 local minimum이 잦으면 **true**로 바꿔 탐색 다양화.

---

# GracefulController (FollowPath2)

**시야/미리보기**

* `min_lookahead / max_lookahead (0.25 / 1.0)`
  길이 L↑ → **lookahead 상향**(대략 `min≈0.4~0.6`, `max≈1.2~1.8`).
  너무 길면 곡률 추종이 둔해지니 환경 따라 조정.

**초기/최종 정렬**

* `initial_rotation_threshold: 0.75`(rad)
  큰 로봇은 제자리 회전 충돌 가능 ↑ → **약간 낮추거나**(0.5~0.7) 혹은
  `v_angular_max`를 낮춰 느리게 회전(예: 5.0→1.5~2.5).

**속도 한계**

* `v_linear_max: 0.5` → **0.35~0.45**(관성, 안전거리 반영)
* `v_angular_max: 5.0` → **1.5~3.0**(과한 yaw rate 억제)
* `slowdown_radius: 1.5` → **L과 안전여유** 반영해 **1.8~2.5** 권장

**게인**

* `k_phi, k_delta, beta, lambda`
  큰 차체에서는 과제어/오버슈트 방지 위해 **소폭 완화**(k_* 약 10~20%↓)를 시작점으로.

---

# Regulated Pure Pursuit (FollowPath3)

**룩어헤드/속도 스케일링**

* `lookahead_dist / min_lookahead_dist / max_lookahead_dist (0.6 / 0.3 / 0.9)`
  L↑ → **각각 +0.1~0.3 m**. `use_velocity_scaled_lookahead_dist: true`로 켜서 속도에 따라 조정도 고려.

**곡률/회전**

* `regulated_linear_scaling_min_radius: 0.9`
  **실제 최소 회전반경 $R_\min$ 이상**으로 갱신. (예: R_\min=1.2m면 ≥1.2)
* `max_angular_accel: 3.2` / `rotate_to_heading_angular_vel: 1.8`
  큰 차체면 **낮춤**(2.0~2.5 / 1.0~1.5).

**장애물 여유**

* `min_distance_to_obstacle: 0.0` → **inscribed_radius 기반 여유**(예: 0.15~0.30)로 상향.
* `use_collision_detection: true` 유지. `max_allowed_time_to_collision_up_to_carrot: 1.0` → **1.5~2.0**로 선제 감속.

**접근/감속**

* `approach_velocity_scaling_dist: 0.6` → **L 및 v_max** 고려해 **0.8~1.2**로 상향.
* `regulated_linear_scaling_min_speed: 0.25`
  큰 차체는 저속 유지가 안전—**0.18~0.22**로 약간 낮추는 것도 방법.

---

# Smac Hybrid (GridBased1)

**핵심**

* `minimum_turning_radius: 0.40` → **실차 $R_\min$** 로 반드시 갱신.
  (DiffDrive라도 **충돌 여유를 포함한 실질 회전반경**으로 두면 경로가 현실적)
* `motion_model_for_search: "DUBIN"`
  후진을 쓰지 못하는 전제. **큰 로봇으로 후진이 필요**하면 `"REEDS_SHEPP"`로 교체.
* `angle_quantization_bins: 72`
  길어진 차체로 회전이 민감하면 **↑**(96~128)로 헤딩 해상도 향상(시간↑).

**회피/우회 성향**

* `cost_penalty: 2.0` / `use_quadratic_cost_penalty: False`
  큰 차체는 통로 중앙 성향이 중요 → `2.0~3.0` 범위 재검토.
* `analytic_expansion_*`
  로봇 커져 근접 단축 경로가 위험하면 `analytic_expansion_max_length` **감소**(3.0→2.0~2.5) 또는 `*_max_cost` **강화**.

---

# Smac Lattice (GridBased2/3)

**차량 모델·격자**

* `lattice_filepath: ""`
  **가장 중요**: **새 크기/회전반경/footprint**로 생성된 **전용 라티스 파일**을 사용해야 함.
  (턴 프리미티브는 휠베이스·R_\min·footprint에 맞춰 재생성 필요)
* `allow_reverse_expansion: false`
  큰 로봇에서 **후진이 필요**하면 true로.
* `rotation_penalty: 5.0`
  큰 차체에서 제자리 회전이 충돌 위험이면 **상향**(6~8)하여 회전 남발 억제.

**기타**

* `minimum_turning_radius`는 Lattice에는 직접 항목이 없고 **라티스 파일에 내재**. 파일 생성 시 R_\min 반영 필수.
* `coarse_search_resolution` / `goal_heading_mode`
  헤딩 정밀도가 더 필요하면 `coarse_search_resolution` ↓ 혹은 `ALL_DIRECTION` 검토(연산↑).

---

## 빠른 치트시트 (값을 “얼마”로 바꿔야 하나?)

* **실측 최소 회전반경** $R_\min$ 재산정 →

  * MPPI `AckermannConstraints.min_turning_r = R_\min`
  * Smac Hybrid `minimum_turning_radius = R_\min`
  * RPP `regulated_linear_scaling_min_radius ≥ R_\min`
  * **Smac Lattice**: 새로운 **라티스 파일** 생성 시 R_\min 반영
* **충돌 여유/시야**

  * Validator `collision_lookahead_time = max(2.0, 1.5·L / v_max)`
  * RPP `min_distance_to_obstacle = 0.15~0.30` (footprint·환경에 따라)
* **회전 동역학**

  * `wz_max` 1.2~1.6, `max_angular_accel` 2.0~2.5 (큰 차체)
* **룩어헤드**

  * Graceful/RPP `min/max_lookahead += 0.1~0.3 m` (L↑ 시)
* **속도/가감속**

  * `vx_max` 0.35~0.45, `ax_max` 2.0~2.5 (보수적으로 시작)
* **크리틱 완화**

  * MPPI `PathAlignCritic.max_path_occupancy_ratio = 0.08~0.12`
  * 각 `threshold_to_consider` **+0.1~0.3**

---

필요하면 **새 길이·폭·휠베이스/트레드·실측 R_\min·목표 v_max** 알려줘. 그 값 기준으로 위 파라미터를 **숫자까지 꽂아 넣은 YAML 패치**로 바로 정리해줄게.

################################  

안녕하세요! Nav2 전문가로서 답변해 드릴게요. 로봇의 크기가 변경되면 안전하고 효율적인 주행을 위해 `nav2_params.yaml` 파일의 여러 파라미터를 세심하게 조정해야 합니다. 단순히 로봇의 반경만 바꾸는 것으로는 부족합니다.

로봇 크기 변경 시, **가장 핵심적으로 고려해야 할 부분은 Costmap과 Controller 설정**입니다. 이 두 가지가 로봇의 물리적 공간 차지(footprint)와 움직임(kinematics)을 직접적으로 다루기 때문입니다.

아래에 제공해주신 `nav2_params.yaml` 파일을 기반으로 수정이 필요한 파라미터들을 꼼꼼하게 정리해 드릴게요. 🤖

---

### 1. Costmaps (local_costmap & global_costmap)

Costmap은 로봇의 크기를 가장 직접적으로 반영하는 곳입니다. 여기서 잘못 설정하면 충돌의 원인이 되거나, 갈 수 있는 길을 가지 못하게 됩니다.

-   `robot_radius`: **(가장 중요 ⭐⭐⭐)**
    -   **설명**: 로봇을 원형으로 근사했을 때의 반경(m)입니다. 현재 `0.22`로 설정되어 있습니다. 로봇의 크기가 바뀌면 **반드시** 이 값을 실제 크기에 맞게 수정해야 합니다.
    -   **팁**: 만약 로봇이 사각형처럼 원형이 아닌 경우, `robot_radius` 대신 `footprint` 파라미터를 사용하는 것이 훨씬 정확하고 안전합니다. `footprint`는 `[[x1, y1], [x2, y2], ...]` 형태로 로봇의 꼭짓점 좌표를 정의합니다.

-   `inflation_layer` -> `inflation_radius`: **(매우 중요 ⭐⭐)**
    -   **설명**: 장애물로부터 얼마나 떨어진 곳까지 비용(cost)을 부풀릴지 결정하는 값입니다. 이 반경은 로봇이 장애물에 너무 가깝게 붙지 않도록 하는 안전 여유 공간을 만듭니다.
    -   **조정**: 일반적으로 `inflation_radius`는 `robot_radius`보다 커야 합니다. 로봇이 커졌다면, 더 넓은 안전거리를 확보하기 위해 이 값을 늘려주는 것이 좋습니다. 예를 들어 `inflation_radius = robot_radius + 0.2` 와 같이 설정할 수 있습니다. 현재 로컬은 `0.70`, 글로벌은 `0.7`로 설정되어 있네요.

-   `local_costmap` -> `width`, `height`:
    -   **설명**: Local costmap의 크기입니다. 로봇이 주행하면서 주변을 탐색하는 영역이죠.
    -   **조정**: 로봇이 더 커지거나 빨라졌다면, 장애물을 미리 감지하고 회피 경로를 생성할 시간을 벌기 위해 이 영역을 더 넓게 (`width`, `height` 값을 증가) 설정하는 것을 고려해볼 수 있습니다.

---

### 2. Controller Server (MPPI Controller)

Controller는 로봇의 동역학 모델과 크기를 고려하여 실제로 모터를 어떻게 움직일지 결정합니다.

-   `FollowPath` (MPPIController) -> `AckermannConstraints` -> `min_turning_r`:
    -   **설명**: 로봇의 최소 회전 반경입니다.
    -   **조정**: 로봇이 커지면 보통 최소 회전 반경도 함께 커집니다. 이 값을 실제 로봇의 사양에 맞게 업데이트하지 않으면, 컨트롤러가 물리적으로 불가능한 경로를 생성하려고 시도하여 주행이 불안정해질 수 있습니다.

-   `FollowPath` (MPPIController) -> `TrajectoryValidator` & `CostCritic` -> `consider_footprint`:
    -   **설명**: 경로 유효성을 검사할 때, `robot_radius`를 쓸지(`false`), 실제 `footprint`를 쓸지(`true`) 결정합니다.
    -   **조정**: 현재 두 값 모두 `false`로 되어 있어, 충돌 검사를 원형(`robot_radius`) 기준으로 하고 있습니다. 만약 로봇이 비원형이고 `footprint` 파라미터를 사용하기로 했다면, 이 값을 `true`로 바꿔주어야 훨씬 정교한 충돌 감지가 가능합니다.

-   `FollowPath` (MPPIController) -> `vx_max`, `wz_max` 등 속도/가속도 관련 파라미터:
    -   **설명**: 로봇의 최대/최소 선속도, 각속도, 가속도 한계입니다.
    -   **조정**: 로봇이 더 커지고 무거워졌다면, 관성 때문에 기존과 같은 속도/가속도를 내기 어려울 수 있습니다. 로봇의 실제 동역학적 성능에 맞춰 이 값들을 현실적으로 낮추거나 조정해야 안정적인 제어가 가능합니다.

-   `general_goal_checker` -> `xy_goal_tolerance`, `yaw_goal_tolerance`:
    -   **설명**: 목표 지점에 얼마나 가까워져야 도착으로 인정할지에 대한 허용 오차입니다.
    -   **조정**: 로봇이 커졌다면, 아주 정밀하게 목표 지점에 맞추기 어려울 수 있습니다. 이 경우, 허용 오차(`tolerance`) 값을 약간 늘려주면 목표 달성 실패율을 줄일 수 있습니다.

---

### 3. Behavior Server

회피 및 복구 동작과 관련된 파라미터입니다.

-   `simulate_ahead_time`:
    -   **설명**: 회피 동작 시, 충돌을 예측하기 위해 몇 초 앞을 시뮬레이션할지 결정합니다.
    -   **조정**: 로봇이 크거나 빠를수록 제동거리가 길어집니다. 따라서 더 멀리 내다보고 미리 판단할 수 있도록 이 값을 늘려주는 것이 안전합니다.

-   `backup` -> `minimum_speed`:
    -   **설명**: 후진 복구 동작 시의 최소 속도입니다.
    -   **조정**: 더 크고 무거운 로봇은 너무 낮은 속도로 움직이면 모터가 떨리거나 불안정할 수 있습니다. 로봇이 부드럽게 움직일 수 있는 최소 속도로 값을 조정해야 합니다.

---

### 4. 기타 서버

-   `collision_monitor`:
    -   **설명**: 주행 중 실시간으로 충돌을 감지하여 로봇을 정지시키거나 속도를 줄이는 역할을 합니다.
    -   **조정**: `FootprintApproach`의 `time_before_collision` 파라미터는 충돌 예상 시간 몇 초 전에 반응할지를 결정합니다. 로봇이 커져서 멈추는 데 더 오랜 시간이 걸린다면 이 값을 늘려서 더 일찍 반응하도록 해야 합니다.

-   `docking_server`:
    -   **설명**: 로봇의 도킹 동작을 제어합니다.
    -   **조정**: `staging_x_offset`(도킹 전 대기 위치 오프셋)이나 `external_detection_translation_x`(외부 센서-로봇 중심 간 변환) 같은 값들은 로봇의 기구적인 형태에 매우 민감합니다. 로봇의 크기나 센서 위치가 바뀌었다면 **반드시** 재측정하여 수정해야 합니다.

### 요약 및 추천 절차

로봇 크기가 변경되었다면, 아래 순서로 파라미터를 점검 및 수정하는 것을 추천합니다.

1.  **Costmap 설정 (가장 먼저!)**: `local_costmap`과 `global_costmap` 양쪽의 `robot_radius` (또는 `footprint`로 교체)와 `inflation_radius`를 새로운 크기에 맞게 수정합니다.
2.  **Controller 설정**: `min_turning_r`을 포함한 로봇의 기구학/동역학적 한계(`vx_max` 등)를 업데이트합니다. 필요하다면 `consider_footprint`를 `true`로 변경합니다.
3.  **안전 관련 설정**: `collision_monitor`의 `time_before_collision`, `behavior_server`의 `simulate_ahead_time` 등을 조정하여 안전 여유를 확보합니다.
4.  **시뮬레이션 테스트**: 수정한 파라미터를 실제 로봇에 적용하기 전에, **반드시 시뮬레이션 환경에서 충분히 테스트**하여 예기치 않은 충돌이나 불안정한 움직임이 없는지 확인하세요.

이 가이드를 따라서 파라미터를 조정하시면, 크기가 변경된 로봇도 성공적으로 Nav2 스택을 사용하여 주행할 수 있을 겁니다. 궁금한 점이 있다면 언제든지 다시 질문해주세요!

###############################  

네, 좋습니다. 추가로 주신 파라미터들은 Nav2의 핵심인 **Local Controller** (경로 추종)와 **Global Planner** (경로 계획)에 대한 내용이군요. 로봇 크기가 변경되면 이 부분들은 매우 직접적인 영향을 받습니다.

플러그인별로 나누어 어떤 파라미터를 왜, 어떻게 바꿔야 하는지 상세히 알려드리겠습니다.

---

### Local Controllers (경로 추종)

Local Controller는 계획된 경로를 실시간으로 따라가며 로봇의 실제 움직임을 만들어냅니다. 따라서 로봇의 물리적 특성(크기, 속도, 회전반경)과 가장 밀접한 관련이 있습니다.

#### `FollowPath1: MPPIController`

MPPI는 수많은 후보 경로를 시뮬레이션하여 최적의 경로를 선택하는 강력한 컨트롤러입니다.

* `vx_max`, `wz_max`, `ax_max` 등 **속도/가속도 한계 파라미터**:
    * **설명**: 로봇의 최대/최소 선속도, 각속도, 가속도 한계입니다.
    * **조정 방향**: 로봇이 더 커지고 무거워졌다면, 관성 때문에 기존과 같은 속도/가속도를 내기 어렵습니다. **반드시** 로봇의 실제 동역학적 성능에 맞춰 이 값들을 현실적으로 조정해야 합니다.
* `AckermannConstraints` -> `min_turning_r`: **(매우 중요 ⭐⭐)**
    * **설명**: 로봇의 최소 회전 반경입니다.
    * **조정 방향**: 로봇이 커지면 **최소 회전 반경도 거의 항상 커집니다.** 이 값을 실제 로봇의 사양에 맞게 업데이트하지 않으면, 컨트롤러가 물리적으로 불가능한 경로를 생성하려고 시도하여 주행이 매우 불안정해집니다.
* `TrajectoryValidator` & `CostCritic` -> `consider_footprint`:
    * **설명**: 충돌 검사 시 원형(`robot_radius`)이 아닌 실제 로봇의 다각형 모양(`footprint`)을 사용할지 여부입니다.
    * **조정 방향**: 로봇이 크고 비원형이라면, Costmap에서 `footprint`를 정의하고 이 값을 **`true`로 설정**하는 것이 훨씬 안전하고 정교한 회피 기동을 가능하게 합니다.
* `collision_lookahead_time`:
    * **설명**: 충돌을 예측하기 위해 몇 초 앞의 미래 경로를 검사할지 결정합니다.
    * **조정 방향**: 크고 무거운 로봇은 제동 거리가 깁니다. 안전을 위해 이 값을 **증가시켜** 더 멀리 내다보고 미리 대응할 수 있도록 해야 합니다.

#### `FollowPath2: GracefulController`

* `min_lookahead`, `max_lookahead`:
    * **설명**: 경로를 따라가기 위해 얼마나 앞을 내다볼지에 대한 거리입니다.
    * **조정 방향**: 몸집이 큰 로봇은 일반적으로 더 긴 `lookahead` 거리를 가질 때 더 부드럽고 안정적인 주행을 보입니다. 이 값들을 **증가시키는 것**을 고려해 보세요.
* `slowdown_radius`:
    * **설명**: 목표 지점 몇 미터 전부터 감속을 시작할지 결정합니다.
    * **조정 방향**: 큰 로봇은 감속에 더 긴 거리가 필요하므로, 이 값을 **증가시켜** 더 일찍부터 부드럽게 감속하도록 해야 합니다.
* `v_linear_max`, `v_angular_max` 등 **속도 관련 파라미터**:
    * **설명**: 로봇의 운동학적 한계입니다.
    * **조정 방향**: MPPI와 마찬가지로, 새로운 로봇의 실제 성능에 맞춰 **반드시 수정**해야 합니다.

#### `FollowPath3: RegulatedPurePursuitController`

Pure Pursuit 계열 컨트롤러는 `lookahead` 거리가 주행 품질에 큰 영향을 줍니다.

* `lookahead_dist`, `min_lookahead_dist`, `max_lookahead_dist`:
    * **설명**: 경로 추종을 위한 전방 주시 거리입니다.
    * **조정 방향**: GracefulController와 동일하게, 로봇이 커졌다면 이 값들을 **증가시켜** 주행을 더 부드럽게 만들 수 있습니다.
* `max_allowed_time_to_collision_up_to_carrot`:
    * **설명**: 전방 주시 지점('carrot')까지 도달하는 동안 충돌까지 허용되는 최대 시간입니다.
    * **조정 방향**: 로봇이 커져서 반응이 느리다면, 안전 마진을 확보하기 위해 이 값을 **증가**시키는 것이 좋습니다.
* `regulated_linear_scaling_min_radius`:
    * **설명**: 경로의 곡률 반경이 이 값보다 작아지면(즉, 급커브) 속도를 줄이기 시작합니다.
    * **조정 방향**: 큰 로봇은 더 완만한 커브에서도 감속이 필요할 수 있습니다. 로봇의 최소 회전 반경과 연동하여 이 값을 **증가**시켜야 합니다.
* `min_distance_to_obstacle`:
    * **설명**: 로봇과 장애물 사이의 최소 허용 거리입니다.
    * **조정 방향**: 로봇 자체가 커졌으므로, 안전을 위해 이 값도 **반드시 늘려주어야** 합니다. `robot_radius`와 비슷한 개념의 안전 파라미터입니다.

---

### Global Planners (경로 계획)

Global Planner는 전체 맵을 보고 출발지부터 목적지까지의 경로를 생성합니다. 특히 SMAC Planner처럼 로봇의 움직임을 고려하는 Planner는 로봇 크기에 매우 민감합니다.

#### `GridBased1: SmacPlannerHybrid`

Hybrid A\* 기반의 Planner로, 로봇의 운동학적 제약을 고려하여 경로를 탐색합니다.

* `minimum_turning_radius`: **(가장 중요 ⭐⭐⭐)**
    * **설명**: 플래너가 경로를 생성할 때 고려하는 로봇의 최소 회전 반경입니다.
    * **조정 방향**: 이 값이 실제 로봇의 회전 반경과 다르면, 플래너는 **로봇이 절대 따라갈 수 없는 경로를 생성**하게 됩니다. 로봇의 크기가 바뀌었다면 **반드시, 정확하게** 이 값을 수정해야 합니다.
* `cost_penalty`:
    * **설명**: Costmap에서 비용이 높은 지역(장애물 주변)에 추가적인 패널티를 부여하여 경로가 장애물로부터 더 멀리, 통로의 중앙으로 생성되도록 유도합니다.
    * **조정 방향**: 로봇이 커졌다면 좁은 길을 통과하기가 더 위험해집니다. 이 값을 **약간 증가시켜** 플래너가 최대한 안전하고 넓은 경로를 선호하도록 만들 수 있습니다.

#### `GridBased2` & `GridBased3: SmacPlannerLattice`

Lattice Planner는 미리 정의된 움직임 패턴(Motion Primitives)의 조합으로 경로를 찾습니다.

* `lattice_filepath`: **(가장 중요 ⭐⭐⭐)**
    * **설명**: 사용할 Motion Primitives 파일(`*.mprim`)의 경로입니다.
    * **조정 방향**: 이 `mprim` 파일은 **특정 로봇의 `minimum_turning_radius`를 포함한 운동학적 특성에 맞춰서 생성**됩니다. 만약 로봇의 크기나 회전 반경이 바뀌었다면, 기존 파일은 더 이상 유효하지 않습니다. **반드시 새로운 로봇 사양에 맞는 `mprim` 파일을 새로 생성하고 그 경로를 여기에 지정**해야 합니다.

---

### 요약 및 핵심

1.  **운동학적 파라미터 일치**: 모든 Controller의 `vx_max`, `ax_max` 등 속도/가속도 관련 파라미터는 새 로봇의 실제 성능과 일치시켜야 합니다.
2.  **최소 회전 반경 통일**: `MPPIController`의 `min_turning_r`과 `SmacPlannerHybrid`의 `minimum_turning_radius`는 새 로봇의 실제 최소 회전 반경 값으로 **반드시 업데이트**해야 합니다.
3.  **Lattice 파일 재생성**: `SmacPlannerLattice`를 사용한다면, 로봇 크기가 변경될 경우 **`mprim` 파일을 새로 생성하는 것이 필수**입니다.
4.  **안전거리 확보**: `lookahead`, `slowdown_radius`, `min_distance_to_obstacle` 같은 거리 기반 파라미터들은 로봇 크기에 비례하여 **전반적으로 늘려주는 것**이 안전하고 안정적인 주행에 도움이 됩니다.
