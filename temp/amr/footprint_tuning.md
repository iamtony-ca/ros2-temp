
footprint 값이 nav2_params.yaml 안에서도 서로 다르고(0.635×0.63, 0.63×0.63), path_validator에도 0.6×0.6이 있으며, 관련 파일이 545개에 달합니다. 그래서 다섯 영역으로 나눠 병렬 검토하고, 마지막에 숫자 상수 전수 검사를 돌리겠습니다.

검토를 백그라운드에서 돌리고 있습니다(에이전트 11개).
- **나눈 영역 다섯:**
  - nav2 설정과 launch
  - path_validator 등 안전 노드
  - fleet/multi-agent
  - 직접 만든 nav 플러그인
  - 하드웨어, URDF, 스캔 필터, 도킹, sim 사본
- **검증:** 영역마다 반박 검증을 붙여 값, 현장에서 실제로 쓰는 파일인지, 크기 의존이 실제인지를 확인합니다.
- **마지막 단계:** 빠진 숫자 상수가 없는지 전수 검사를 합니다.

끝나면 반드시 바꿀 것, 바꾸는 게 좋은 것, 결정이 필요한 것으로 나눠 정리해 드리겠습니다.

검토 결과(에이전트 11개, 항목 107개)를 정리하고 있습니다. 결정이 필요한 항목과 열린 질문을 마저 봅니다.

결과를 정리하기 전에, 가장 중요한 항목 몇 개를 직접 다시 확인합니다.

amhs를 검토한 결과, `footprint:`를 고치는 것만으로는 부족하고 크기를 따로 적어 둔 곳이 여러 곳 더 있습니다. 검증을 거친 항목은 107개이고, 아래에 바꿔야 할 것부터 정리했습니다. amhs는 읽기만 했고 아무것도 고치지 않았습니다.

**고치기 전에 정할 것 (값이 이 답에 따라 달라집니다)**
1. **base_link 기준 대칭인지:** 지금 값은 앞 0.32, 뒤·좌우 0.315로 이미 비대칭입니다.
   - URDF상 base_link는 구동축 중심, 즉 회전 중심입니다.
   - 구동축에서 앞 끝, 뒤 끝, 좌우 끝을 따로 재 주세요. 비대칭이면 아래 다각형 모두에 실측 꼭짓점을 넣어야 합니다.
2. **돌출부가 들어간 값인지:** 범퍼, 매니퓰레이터·트레이의 주행 자세, D435 카메라가 해당됩니다. URDF에는 D435 네 대가 x ±0.35에 있어 0.32보다 3 cm 밖입니다.
3. **22대가 모두 같은 크기인지:** 섞이면 로봇마다 다른 footprint가 필요합니다.

**1. 반드시 바꿀 것 (footprint를 그대로 옮겨 적은 곳)**

| 위치 | 지금 | 바꿀 값 |
|---|---|---|
| `amr_bringup/params/nav2_params.yaml:1183` local costmap | 0.32/0.315 | ±0.32 |
| 같은 파일 `:1683` global costmap | 같음 | ±0.32 |
| 같은 파일 `:2812` static_nav costmap (BT의 static planner가 씀) | 같음 | ±0.32 |
| 같은 파일 `:2530` collision_monitor PolygonStop | 같음 | ±0.32 (줄 주석에 이미 있는 값) |
| `ammr_common_config/config/common_ammr.yaml` 22곳 | ±0.315 | ±0.32 |
| `ammr_common_config/config/common_ammr_default.yaml` 11곳 | ±0.315 | ±0.32 |
| `replan_monitor/config/path_validator.params.yaml:65` | ±0.315 | ±0.32 |

- `common_ammr.yaml`이 다른 로봇의 크기를 정하는 실제 값입니다. run_amr_bringup이 이 파일로 nav2_params.yaml의 로봇 블록을 덮어씁니다.
- 이 파일은 gitignore된 로봇별 파일이라 22대 각각에 배포해야 합니다. `common_ammr_default.yaml`은 git에 있는 원본이라, 고치지 않으면 새 로봇 설정이 옛 값으로 돌아갑니다.
- PolygonStop을 넓히기 전에 확인할 것이 있습니다. 스캔 경로에 몸체 자체 필터가 없고, `footprint_clearing_enabled`는 false, `obstacle_min_range`는 0입니다.
  - 정지 상태에서 0.315–0.32 사이에 자기 몸체 점이 잡히는지 먼저 봐 주세요. 잡히면 로봇이 스스로 정지할 수 있습니다.

**2. 크기에서 나온 값이라 바꾸는 게 좋은 것**
- **`replan_monitor/src/path_validator_node.cpp`의 0.424:** `:1979`, `:2754`의 `allowed_dist = 0.424 + 0.1`과 `:2472`, `:2478`, `:2569`, `:2575`의 `findNearestAgent(..., 0.524)`입니다.
  - 0.424는 예전 0.6 상자의 외접반경(0.3×√2)입니다. 0.63으로 바꿀 때도 갱신되지 않았습니다.
  - 새 값은 0.4525 + 0.1 ≈ 0.55입니다. agent_layer dilation 0.05까지 넣으면 약 0.60입니다.
  - 여섯 곳을 파라미터 하나로 묶기를 권합니다. C++라 빌드가 필요합니다.
- **`fleet_decision_node.py:2630`의 THRESHOLD 0.5 (현장 v1에서 동작):** 0.55 정도를 권합니다.
  - 모서리에서 부딪힐 때만 차이가 납니다. 이때 REROUTE 대신 우선순위 대기로 빠질 수 있습니다.
- **local `footprint_padding` 0.001 (`nav2_params.yaml:1184`):** 실행 중 이 값을 덮어쓰는 `roi_range_manager_node.py:57-58` 기본값(recovery 0.0, normal 0.001)도 같이 봐야 합니다.
  - local 격자가 2 cm라서, 몸체는 5 mm 커지지만 장애물로 칠해지는 고리는 15칸에서 16칸으로 2 cm 넓어집니다.
  - 0.32가 격자 경계라서 생기는 일입니다. padding 값을 고를 때 이 점을 피해 주세요.
  - padding은 0.0336 이하로 둬야 local inflation 0.5가 외접반경보다 큰 상태를 유지합니다.
- **collision_monitor의 PolygonSlow `:2509`, PolygonLimit `:2519`:** 같은 여유를 유지하려면 옆·뒤 좌표에 +0.005를 합니다.
  - PolygonLimit의 y 0.30은 몸체보다 좁습니다. 원래부터 그랬는데 의도인지 확인이 필요합니다.
  - PolygonSlow의 좌우 비대칭(0.342 대 0.352)이 오타인지도 봐 주세요.
- **collision_detector의 PolygonSafety `:2636`:** nav_safety_manager가 PLC 영역을 되돌리는 조건으로 씁니다. SICK 필드와 맞춰야 합니다.
- **SICK 안전 필드 (안전 담당이 결정):** ±0.32 기준이면 여유가 약 0.5 cm 줄어듭니다.
  - Narrow 왼쪽은 −0.2…+1.6 cm, Docking 뒤는 −2.1…+0.1 cm가 되어 일부가 덮이지 않습니다.
- **도킹:**
  - `lidar_auto_docking/config/autodock_params.yaml:158-159`, `:190-191`의 `robot_width` 0.6은 0.64로 바꿉니다.
  - `robot_length`를 0.64로 바꾸면 `field_end_margin`을 0.05로 같이 올려야 합니다. 그래야 dock 앞에서 서던 예전 버그를 피합니다.
  - `marker_auto_docking.yaml:3-4`의 0.5/0.5는 0.64로 바꿉니다. 그러면 태그 쪽 여유가 7 cm 줄고, 코드에 inflation 0.05가 하드코딩돼 있습니다.
- **현장 기동 스크립트에 없는 노드 (쓴다면 바꿈):**
  - `docking_sequence_qr.hpp:353-354`: 0.5/0.7 하드코딩. winros_bridge_to_dock이 부릅니다.
  - `ir_line_follower/config/params.yaml:25-27`
- **v2를 켤 때만 해당 (지금 `v2_enable` false):** `fleet_decision_node.py:2963`의 `+0.315`, `:5379`의 0.3.

**3. 확인 또는 튜닝 (지금 값으로도 동작)**
- **inflation_radius** local 0.5, global 0.61, static 0.7은 모두 새 외접반경보다 커서 그대로 둬도 됩니다. 여유만 약 0.4 cm씩 줄어듭니다.
- **global/static padding 0.02:** 계획 폭이 0.67 m에서 0.68 m가 됩니다.
  - 현장 keepout mask(`251223_map_keepout`)의 차선 폭이 이보다 넉넉한지 확인해 주세요. 이 파일은 repo에 없습니다.
- **RPP 쪽 `cost_scaling_dist` 0.4/0.55 (`:407`, `:880`):** 예전과 똑같이 하려면 +0.005입니다.
  - 별개 문제로, `inflation_cost_scaling_factor`의 코드 기본값 3.0이 costmap의 10.0과 다릅니다.
- **agent_layer `dilation_m` 0.05, `load_rear_smear_m` 0.35:** 이적재 때 뒤로 튀어나오는 길이를 따로 쟀는지 확인해 주세요.
- **자기 몸체가 스캔에 잡히는지 한 번 측정:** 잡히면 `obstacle_min_range`나 cartographer `min_range`를 올립니다.
- **자동으로 따라가는 것:** Graceful, MPPI CostCritic, RPP, behavior server, Smac, escape planner(canRotate)는 footprint를 읽어 쓰므로 값을 바꿀 필요가 없습니다.
- **현장에서 안 쓰이는 기본값:** nav2_params.yaml의 agent 블록과 path_validator의 0.3 상자 블록은 덮어써지므로 일관성 차원에서만 고치면 됩니다.
- **빌드:** symlink-install이 아니면 yaml만 고쳐도 해당 패키지를 다시 빌드해야 반영됩니다.
- **관제(Windows) 쪽:** 로봇 크기 설정이 있는지 따로 확인해 주세요. 이 저장소 밖입니다.

**4. sim 쪽 (amhs를 바꾸면 제가 맞출 수 있음)**
- sim은 현장 nav2_params.yaml을 그대로 쓰므로 위 1번을 바꾸면 따라갑니다.
- 따로 맞춰야 하는 것:
  - sim 로봇 22대의 template `amhs_overrides/common_ammr/robots_sim.yaml:16`
  - 로봇 몸체 모델: gz urdf의 0.63 상자, USD, `run_fleet.py:55`, simple spec ±0.315
  - 분석 도구의 하드코딩: 0.73, HALF 0.315, 0.4455, `GOAL_BLOCK_M` 0.75 등



##
##
##
##
##
먼저 근거 코드를 직접 확인하겠습니다.

두 항목 모두 장애물 근처에서 costmap 비용을 어떻게 쓰느냐에 관한 것입니다. 첫째는 경로를 고를 때, 둘째는 주행 중 속도를 줄일 때의 이야기입니다. 아래 숫자는 코드의 계산식에 지금 설정값을 넣어 계산한 것이고, 실차에서 잰 값은 아닙니다.

## 1. inflation_radius와 외접반경

**inflation의 의미:** costmap은 장애물 칸 둘레에 거리에 따라 비용을 칠합니다.
- 로봇 중심이 장애물에서 내접반경(몸체 중심에서 옆면까지, 약 0.32 m) 안에 있으면 반드시 닿으므로 최대 비용입니다.
- 그 밖으로는 `inflation_radius`까지 비용이 지수로 줄어듭니다(`cost_scaling_factor` 10.0).
- 그보다 멀면 비용 0입니다.

**외접반경이 기준인 이유:** 외접반경은 몸체 중심에서 모서리까지의 거리입니다. 로봇이 어느 방향을 보든 모서리가 닿지 않는 거리라는 뜻입니다.
- Smac 플래너나 MPPI는 어떤 칸의 비용이 외접반경 위치의 비용보다 낮으면 "어느 방향으로도 안 닿는다"고 보고 footprint 전체 충돌 검사를 건너뜁니다.
- `inflation_radius`가 외접반경보다 작으면 그 기준 비용이 0이 됩니다. 그러면 이 지름길을 못 써서 검사가 느려지고, Nav2가 경고를 냅니다.
- 그래서 "`inflation_radius` ≥ 외접반경"을 지켜야 하고, 지금은 모두 지킵니다.

| costmap | inflation_radius | 외접반경 (padding 포함) 예전 → 0.64 | 여유 예전 → 0.64 |
|---|---|---|---|
| local (padding 0.001) | 0.5 | 0.450 → 0.454 | 5.0 → 4.6 cm |
| global (padding 0.02) | 0.61 | 0.477 → 0.481 | 13.3 → 12.9 cm |
| static (padding 0.02) | 0.7 | 0.477 → 0.481 | 22.3 → 21.9 cm |

- "여유가 약 0.4 cm 준다"는 이 표의 마지막 열입니다. 비용이 칠해지는 띠 바깥 끝과 모서리 사이 거리가 조금 좁아질 뿐이라 바꿀 필요는 없습니다.
- local padding을 0.0336 넘게 키우면 local이 이 규칙을 어기게 됩니다. 앞 답에서 padding 상한을 말씀드린 이유입니다.

## 2. RPP의 장애물 근접 감속 (`cost_scaling_dist`)

**어떤 컨트롤러인가:** `nav2_params.yaml`의 두 컨트롤러입니다.
- `:407` ReverseRPP (advanced_regulated_pure_pursuit_controller)
- `:880` StraightRPP (straight_pure_pursuit_controller)
- 둘 다 `use_cost_regulated_linear_velocity_scaling: true`라서 장애물에 가까우면 속도를 줄입니다.

**동작 방식** (`regulation_functions.hpp`의 `costConstraint`):
1. 로봇 중심 칸의 costmap 비용을 읽습니다.
2. 위 inflation 식을 거꾸로 풀어 비용을 "장애물까지 거리"로 바꿉니다.

   거리 = (factor × 내접반경 − ln(비용) + ln(253)) / factor

3. 이 거리가 `cost_scaling_dist`(0.4 또는 0.55)보다 작으면 감속합니다.

**+0.005의 의미:** 감속 시작 거리는 로봇 중심 기준입니다. 몸체가 한쪽에 5 mm 커지면, 같은 0.4 m에서 감속을 시작해도 옆면과 장애물 사이는 5 mm 좁습니다. 예전과 같은 몸체 여유에서 감속하게 하려면 0.405, 0.555로 올리면 됩니다. 차이가 작아 그대로 둬도 됩니다.

**별개 문제: 비용을 거리로 바꿀 때 쓰는 계수가 다릅니다.**
- costmap은 `cost_scaling_factor` 10.0으로 비용을 칠합니다.
- RPP는 `inflation_cost_scaling_factor`로 거꾸로 푸는데, yaml에 이 키가 없어 코드 기본값 3.0을 씁니다.
- 계수가 다르면 RPP는 실제보다 장애물이 약 3.3배 멀다고 계산합니다. 그래서 설정한 거리보다 훨씬 가까이 가서야 감속합니다.

장애물이 옆에 있다고 보고 0.64 기준으로 계산하면 이렇습니다.

| 컨트롤러 | 설정 | 의도한 감속 시작 (옆면 여유) | 실제 (계수 3.0) |
|---|---|---|---|
| ReverseRPP | 0.4 | 7.9 cm | 2.3 cm |
| StraightRPP | 0.55 | 17.9 cm (inflation 끝 0.5에서 잘림) | 6.8 cm |

- footprint 변경과는 상관없이 원래 있던 어긋남입니다.
- 두 블록에 `inflation_cost_scaling_factor: 10.0`을 적으면 의도대로 동작합니다.
- 다만 그러면 감속이 지금보다 훨씬 일찍, 자주 걸립니다. 지금 현장 주행감이 이 값으로 맞춰져 있을 수 있어서, 바꾼다면 `cost_scaling_dist`와 `cost_scaling_gain`을 함께 다시 튜닝하고 sim에서 먼저 확인하기를 권합니다.



##
##
##
##
##
##
##
##
# footprint 0.64 x 0.64 영향 검토 — 전체 항목 (2026-11-25, 읽기만, 워크플로 렌즈 5 + 검증 5 + 전수 1)

렌즈별 원 보고서: nav2/ validator/ fleet/ plugins/ hw_sim/ (각 REPORT.md, verify/VERIFY.md), sweep/SWEEP.md. 아래는 검증을 거친 목록 (같은 항목이 여러 렌즈에 나올 수 있음).

## nav2

현장 체인: 현장 기동은 scripts/amr_batch/run_all_auto_default.sh:114 에서 시작한다. `ros2 launch amr_bringup run_amr_bringup.launch.py map:=... filter_mask_file:=...` 를 띄운다.  -> run_amr_bringup.launch.py:94 의 기본 params 는 params/nav2_params.yaml 이다. :46-47 에서 ammr_common_config/config/common_ammr.yaml 을 읽는다. :18-34 에서 robot_ids 를 가진 노드 아래로 로봇 블록을 deep-copy 한다. :79-85 에서 임시 yaml 을 bringup_launch.py 로 넘긴다.  -> bringup_launch.py:172-173 에서 use_composition 기본값은 True 다. :240-295 에서 같은 params 로 localization_launch, costmap_filter.launch, navigation_launch 를 띄운다.  -> navigation_launch.py:43-54 가 lifecycle 노드 10개를 띄우고, :169-187 과 :317-329 에서 static_nav/static_planner_server 를 띄운다. behavior_server 플러그인은 custom 인 ammr_behaviors(behavior_server::BehaviorServer) 다.  -> nav2_params.yaml:60 의 BT 는 moduler33_..._op_pause_gate.xml 이다.  같은 스크립트(:129-160)가 navigation_manager, roi_range_manager, nav_safety_manager, path_validator, fleet_decision, lidar_auto_docking, marker_auto_docking, velocity_modifier 도 띄운다.  sim 은 field_stack.launch.py:230-272 에서 현장 nav2_params.yaml 위에 sim_overlay*.yaml 을 얹는다. overlay 에는 footprint, padding, inflation, polygon 키가 없다(grep 확인). 그래서 sim 도 nav2_params.yaml 을 그대로 따라간다. sim 의 agent footprint 는 common_ammr_default.yaml 과 robots_sim.yaml:16 에서 온다.  현장 keepout mask 는 ${MY_WS}/src/assets/maps/251223_map_keepout.yaml 이다(run_all_auto_default.sh:44,56, map.cfg). repo 의 assets/maps 와 amr_bringup/maps 는 비어 있다.

| 우선 | 파일:줄 | 지금 | 의존 | 제안 |
|---|---|---|---|---|
| must | amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:1183 | local_costmap footprint "[[0.32, 0.315], [0.32, -0.315], [-0.315, -0.315], [-0.315, 0.315]]" (0.635x0.63) | base_link 기준 몸체 외곽 | "[[0.32, 0.32], [0.32, -0.32], [-0.32, -0.32], [-0.32, 0.32]]" 로 바꾼다. 실측이 base_link 기준으로 비대칭이면 앞 x, 뒤 x, 좌 y, 우 y 를 실측값 그대로 쓴다. [검증: confirmed] |
| must | amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:1683 | global_costmap footprint 이 1183 과 같은 0.32/0.315 문자열이다 (1682 에 '### need to check' 주석) | 몸체 외곽 | 1183 과 같은 값으로 바꾼다. [검증: confirmed] |
| must | amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:2812 | static_nav global_costmap footprint 이 0.32/0.315 문자열이다 | 몸체 외곽 | 1183 과 같은 값으로 바꾼다. [검증: confirmed] |
| must | amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:2530 | collision_monitor PolygonStop points "[[0.32, 0.315], [0.32, -0.315], [-0.315, -0.315], [-0.315, 0.315]]". 같은 줄 주석에 0.32 정사각형 안이 이미 있다 | 몸체 외곽 (여유 0) | 0.32 정사각형으로 바꾼다(주석의 안). 여유를 더할지는 판단할 일이다. 소스는 /scan_matched_points2 (2496) 다. 0.315~0.32 사이에 자기 몸체 점이 남아 있으면 새로 정지가 걸릴 수 있다. self-filter 를 먼저 확인한다. [검증: confirmed] |
| must | amhs/software/ammr_common_config/config/common_ammr.yaml:18,24,30,...,144 (22개) | AMMR_2P_TRAY1~22 footprint "[[0.315, 0.315], [0.315, -0.315], [-0.315, -0.315], [-0.315, 0.315]]", robot_radius 0.0 | 같은 기종인 다른 로봇의 몸체 외곽 (agent_layer 가 local/global costmap 에 칠한다) | 22개를 모두 0.32 정사각형(또는 실측 외곽)으로 바꾼다. 현장에서 실제로 쓰는 값은 이 파일이다. [검증: confirmed] |
| should | amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:1184 (+ amhs/software/navigation/navigation_manager/navigation_manager/roi_range_manager_node.py:57-58) | local footprint_padding 0.001. roi_range_manager 기본값은 recovery 0.0, normal 0.001 이다. config/roi_range_manager.yaml 은 target_node 만 덮어쓴다 | local 내접/외접 반경과 2 cm 격자의 253 고리 | padding 을 바꾸면 두 곳을 같이 바꾼다. 0.001 을 그대로 두면 내접반경이 0.316 에서 0.321 이 된다. 그러면 253 고리가 축 방향으로 15칸에서 16칸이 된다(+2 cm). 몸체는 0.5 cm 만 커졌는데 고리는 2 cm 커진다. [보정] RECOVERY 때 쓰는 padding 0.0 에서도 16칸이 된다(예전 15칸). 16x0.02 = 0.32 라서 경계값이다. 그래서 0.32 의 정수배 격자 경계를 피하는 padding 을 고르는 편이 안전하다. global 은 0.03 격자라서 11칸 그대로다. local inflation 0.5 가 외접 (0.32+p)·√2 이상이어야 하므로 p 는 0.0336 이하로 둔다. [검증: confirmed + 보정] |
| should | amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:2509, 2519 | PolygonSlow 는 y ±0.33, x ±0.43 팔각형이다. PolygonLimit 은 y ±0.30, x ±0.40 팔각형이다 | 몸체 + 여유. 새 몸체 기준 Slow 옆 여유는 1.0 cm(예전 1.5 cm), 뒤 여유는 11.0 cm(예전 11.5 cm)다. Limit 의 y 0.30 은 반폭 0.32 보다 작다 | 여유를 그대로 두려면 옆과 뒤 좌표에 +0.005 한다(예: Slow y 0.335, 뒤 x -0.435). 여유 크기는 튜닝할 일이다. Limit 이 옆을 덮지 않는 것은 예전(0.315)부터 그랬다. 의도인지 확인한다. Slow 의 좌우 비대칭(0.342 vs 0.352, 0.21 vs -0.2)이 오타인지도 확인한다. [검증: confirmed, Limit 은 기존 문제] |
| check | amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:407 (ReverseRPP), 880 (StraightRPP) | use_cost_regulated_linear_velocity_scaling true, cost_scaling_dist 0.4 / 0.55, inflation_cost_scaling_factor 미설정(코드 기본 3.0) | local 내접반경 (거리 역산식) | [overstated] 몸체 기준 여유가 5 mm 어긋나는 정도라서 꼭 바꿀 필요는 없다. 예전과 똑같이 맞추려면 0.405 / 0.555 로 한다. inflation_cost_scaling_factor(3.0) 와 layer 의 cost_scaling_factor(10.0) 가 다른 것은 원래 있던 별개 문제다. 맞출지 따로 정한다. 튜닝 항목이다. |
| check | amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:1496-1568, 2001-2073 | agent_layer 로봇 블록 9개 x 2 (local/global). 모두 0.315 정사각형이다. robot_ids(1492) 에 robot_001/robot_002/warehouse_bot_A 같은 예시도 들어 있다 | 다른 로봇 몸체 | [overstated] run_amr_bringup 으로 띄우면 common_ammr.yaml 이 robot_ids 와 블록을 덮어쓴다. 그래서 이 값은 죽은 기본값이다. bringup_launch.py 를 직접 띄울 때만 쓰인다. 일관성을 위해 0.32 로 맞춰 두면 좋다. |
| check | amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:1198, 1716, 2839 (cost_scaling_factor 1197, 1715, 2838) | inflation_radius: local 0.5 / global 0.61 / static 0.7, cost_scaling_factor 10.0 | 외접반경. 새 값은 local 0.454(padding 0.001), global/static 0.481(padding 0.02) | 셋 다 외접반경보다 커서 꼭 바꿀 필요는 없다. 여유가 local 5.0→4.6 cm, global 13.3→12.9 cm, static 22.3→21.9 cm 로 줄어든다. 비용 곡선을 예전과 같게 두려면 각각 +0.005 한다. 판단하고 튜닝할 항목이다. [검증: confirmed] |
| check | amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:1684, 2813 | global/static footprint_padding 0.02. 계획 폭이 0.67 m 에서 0.68 m 가 된다 | 통로·문 폭과 패딩 포함 몸체 폭의 비교 | 유지할지 줄일지는 현장 최소 통로 폭을 보고 정한다. [검증: confirmed] |
| check | amhs/scripts/amr_batch/run_all_auto_default.sh:44, 56, 114 (+ map.cfg, nav2_params.yaml:1700, 1706-1710, 2817, 2828-2832) | 현장 mask 는 ${MY_WS}/src/assets/maps/251223_map_keepout.yaml 이다. repo 에는 없다. lane_filter(KeepoutFilter)는 inflation 보다 앞에 있다 | keepout 차선 자유폭과 0.64 + 2·padding 의 비교 | mask 의 차선 폭이 0.68 m + 격자 이상인지 확인한다. 0.63 m 기준으로 딱 맞게 그렸다면 막힐 수 있다. [검증: confirmed. 원래 근거였던 bringup_launch.py:201-206 기본값은 현장에서 인자로 덮어써진다] |
| check | amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:2636 | collision_detector PolygonSafety "[[0.70, 0.39], [0.70, -0.39], [-0.51, -0.39], [-0.51, 0.39]]" (소스 /scan, action none) | 몸체 + 여유. 새 몸체 기준 옆 여유는 7.0 cm(예전 7.5), 뒤 여유는 19 cm(예전 19.5)다 | nav_safety_manager 가 PLC 영역 복원 조건으로 이 polygon 을 쓴다. PLC 축소 영역과 맞춰서 정한다. 여유를 그대로 두려면 옆 0.395, 뒤 -0.515 로 한다. [검증: confirmed] |
| check | amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:2543-2621 | dynamic_stop_narrow / dynamic_slowdown 는 enabled: false (2544, 2587) 다. stop_creeping_rotating 은 y ±0.28 로 몸체보다 작다(2575) | 몸체 + 여유 | 지금은 꺼져 있다. 켤 계획이면 새 몸체 기준으로 다시 그린다. [검증: confirmed] |
| check | amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:1143 (+ BT:835, BT:1001) | escape_max_len 0.45. BT 의 TruncatePathLocal distance_forward="0.45" 가 두 곳에 있다(835, 1001. finder 는 1001 만 적었다) | [overstated] 몸체 크기와 관계없다. 1142-1143 주석에 따르면 사용자가 정한 '최대 이동' 값이다 | 바꿀 필요 없다. canRotate 는 padding 을 포함한 외접반경을 자동으로 쓴다(navfn_planner.cpp:703-705). |
| check | amhs/software/navigation/amr_navigation2/ammr_navfn_escape_planner/include/ammr_navfn_escape_planner/navfn_planner.hpp:255 | double fp_front_{0.32}, fp_back_{-0.315}, fp_left_{0.315}, fp_right_{-0.315}; (하드코딩된 초기값) | 몸체 외곽(직사각형 가정, unpadded) | [누락 항목] 실제로는 tryEscapePlan 이 getUnpaddedRobotFootprint() 로 매번 덮어쓴다(navfn_planner.cpp:830-836). 그래서 동작은 바뀌지 않는다. 헷갈리지 않게 0.32 로 맞춰 두는 것은 선택이다. 이 경우 빌드가 필요하다. |
| check | amhs/software/navigation/amr_navigation2/amr_bringup/behavior_trees/moduler33_wo_agent_layer_replan_in_pause_timeout_pub_goals_rmv_goal_in_recovery_pause_hold_op_pause_gate.xml:1554-1559 (사용처 1105-1111 등) | Cfg_costmap_topic local_costmap/costmap_raw, Cfg_occupied_th 253 | local 253 고리 크기 | 값은 바꿀 필요 없다. local 253 고리가 2 cm 넓어지므로, 장애물이나 다른 로봇 옆에 있는 goal 이 예전보다 일찍 occupied 로 판정된다. 현장에서 지켜본다. [검증: confirmed] |
| check | amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:1480/1985, 1487/1992, 1490/1995 | agent_layer dilation_m 0.05, forward_smear_m 0.001/0.005, load_rear_smear_m 0.35 | 다른 로봇 몸체 여유. 몸체 polygon 자체는 common_ammr.yaml 에서 온다 | footprint(common_ammr.yaml)만 바꾸면 된다. load_rear_smear_m 은 LOADING/UNLOADING 때 뒤로 칠하는 길이다. 이 값이 몸체 반길이에서 나온 것인지만 확인한다. [보정] 코드 기본값은 0.4 (agent_layer.cpp:149) 이고 params 값은 0.35 다. |
| check | amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:1246, 1801 | nvblox_layer inflation_distance 0.34 (enabled: false, 1241/1796) | 내접반경. 새 값은 local 0.321, global 0.34 | 켤 때만 새 내접반경 이상인지 확인한다. 0.34 는 새 global 내접과 똑같다. [검증: confirmed] |
| check | amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:934, 974, 1103, 2772 / 397, 402, 528, 713, 870, 875 | Smac minimum_turning_radius 0.3. 'min_turning_radius = length /2 = 0.35 m' 주석이 있다. MPPI AckermannConstraints min_turning_r 0.35 다(motion_model 이 DiffDrive(512/697)라서 쓰이지 않는다). curvature_lookahead_dist 0.5/0.6 에는 padding 관련 주석이 있다 | 경로 곡률과 예전 몸체 길이 주석. 몸체 치수가 직접 들어가지는 않는다 | 값은 바꿀 필요 없다. 오래된 주석만 정리한다. padding 을 바꾸면 curvature_lookahead_dist 를 다시 튜닝할지 생각한다. [검증: confirmed] |
| check | amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:2682, 2688, 2714 | docking staging_x_offset -0.7, external_detection_translation_x -0.18, dock_collision_threshold 0.3 | 앞면에서 base_link 까지의 거리 | docks 가 정의돼 있지 않다(2697 주석). 현장은 lidar_auto_docking 과 marker_auto_docking 을 쓴다(run_all_auto_default.sh:129,135). 쓸 계획이 없으면 그대로 둔다. [검증: confirmed] |
| check | amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:250, 565, 750, 837, 1723, 1837, 2356-2357, 2379 | Graceful use_collision_detection, MPPI CostCritic consider_footprint, FastClear extra_margin_m 0.0, global zed_front_layer footprint_clearing_enabled true, behavior_server footprint 토픽, simulate_ahead_time 3.0 | costmap footprint 와 inflation_radius (자동 반영) | 값은 바꿀 필요 없다. footprint 만 바꾸면 자동으로 따라간다. 확인만 한다. [검증: confirmed. ammr_behaviors/behavior_server.cpp:176-191 과 ammr_straightline_planner/src/straight_line_planner.cpp:132 도 자동 반영된다] |

## validator

현장 체인: 현장은 scripts/amr_batch/run_all_auto_default.sh:154 에서 `ros2 launch replan_monitor run_path_validator.launch.py` 를 띄운다. 이 launch 는 :33-51 에서 ammr_common_config/config/common_ammr.yaml 을 읽는다. 그다음 self_machine_id, robot_ids, 로봇 블록을 path_validator.params.yaml 위에 덮어쓴다 (:11-26). 마지막으로 path_validator.launch.py 가 path_validator_node 를 띄운다 (CMakeLists.txt:79-83: path_validator_node.cpp, path_validator_main.cpp, costmap_mirror.cpp). nav2 도 같은 방식이다. run_amr_bringup.launch.py:46-68 이 common_ammr.yaml 을 nav2_params.yaml 위에 덮어쓴다. 그래서 nav2 agent_layer 의 로봇 블록 (nav2_params.yaml:1502-1568, 2007-2073) 도 죽은 기본값이다. 다른 로봇 정보는 winros_bridge 가 /multi_agent_infos 로 보낸다. 이때 footprint 는 빈 다각형이다 (winros_bridge/winros_bridge/src/winros_bridge.cpp:1613-1614). 같은 스크립트가 띄우는 것: velocity_modifier (:120), nav_safety_manager (:148), roi_range_manager (:160), fleet_decision (:157). collision_detector 는 navigation_launch.py:252-262 로 뜬다. sim 은 field_stack.launch.py:540-580 _path_validator 가 같은 덮어쓰기를 한다. 단 common 은 common_ammr_default.yaml + robots_sim.yaml 이고, template 이 같은 이름 블록을 이긴다. 설정은 share 디렉터리에서 읽는다 (ammr_common_config CMakeLists.txt:27, replan_monitor CMakeLists.txt:121).

| 우선 | 파일:줄 | 지금 | 의존 | 제안 |
|---|---|---|---|---|
| must | amhs/software/ammr_common_config/config/common_ammr.yaml:18,24,30,...,144 (22곳) | footprint: "[[0.315, 0.315], [0.315, -0.315], [-0.315, -0.315], [-0.315, 0.315]]" | 다른 로봇(agent) 발자국이다. nav2 agent_layer 와 path_validator 둘 다 이 값을 쓴다 | [[0.32,0.32],[0.32,-0.32],[-0.32,-0.32],[-0.32,0.32]] 로 바꾼다. gitignore 된 파일이라 (amhs/.gitignore:95) 로봇마다 고친다. [검증: confirmed] |
| must | amhs/software/ammr_common_config/config/common_ammr_default.yaml:15,23,31,39,47,55,64,72,82,91,98 | footprint 0.315 정사각형 (11곳) | 현장 common_ammr.yaml 의 git 원본이다. 실행 때 직접 읽지는 않는다 | 0.32 정사각형으로 바꾼다. [검증: confirmed] |
| must | amhs/software/replan_monitor/config/path_validator.params.yaml:65 | footprint: "[[0.315, 0.315], [0.315, -0.315], [-0.315, -0.315], [-0.315, 0.315]]" | 자기 로봇 발자국이다. Phase 2 경로 sweep 과 M-6 자기 발자국 무시 영역에 쓰인다 | 0.32 정사각형으로 바꾼다. nav2 global footprint (nav2_params.yaml:1683) 와 패딩만 뺀 같은 다각형으로 맞춘다. [검증: confirmed] |
| should | amhs/software/replan_monitor/src/path_validator_node.cpp:1979 | const double allowed_dist = 0.424 + 0.1; | agent 외접 반경 (0.424 = 0.3x√2, 예전 0.6 정사각형) + 지연 오차 0.1 이다. 칠해진 agent 모서리는 외접 + dilation 0.05 다 | 기계적 변경은 0.4525+0.1=0.5525 다. 판단 항목으로는 dilation (nav2_params.yaml:1985) 까지 넣어 0.4525+0.05+0.1≈0.60 을 권한다. agent_footprints_ 의 최대 꼭짓점 거리로 계산하면 더 좋다 (agent_layer.cpp:249-253 이 이미 그렇게 한다). [검증: confirmed, 여유 수치 보정: 지금 0.524-0.499=0.025, 새 크기 0.0215] |
| should | amhs/software/replan_monitor/src/path_validator_node.cpp:2472,2478,2569,2575 | findNearestAgent(goal.x, goal.y, 0.524) 하드코딩 | 목표점에서 agent 중심까지 거리다. 외접 + dilation + goal_doorstep_agent 0.05 로 정해진다 | 1979 행과 같은 값 (0.5525 또는 약 0.60) 으로 바꾸고 한 상수나 파라미터로 묶는다. [검증: confirmed. 입력은 hit 셀이 아니라 목표점이다] |
| should | amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:2636 | PolygonSafety points: "[[0.70, 0.39], [0.70, -0.39], [-0.51, -0.39], [-0.51, 0.39]]" | 몸체 바깥 여유 (지금 앞 0.38, 옆 0.075, 뒤 0.195) | 판단 항목이다. 같은 여유를 지키려면 옆 ±0.395, 뒤 -0.515 로 바꾼다. SICK 필드를 흉내 낸 값이면 SICK 설정에 맞춘다. [검증: confirmed, 활성] |
| should | amr-gz-sim-example/mobile_robot_gz_sim/amhs_overrides/common_ammr/robots_sim.yaml:16,8 | template footprint 0.315 정사각형, base_sha256_at_write b6079a6c… | sim agent 발자국 (sim 사본) | 16 행을 0.32 로 바꾼다. template 이 base 의 같은 이름 블록을 이기므로 sim 에서는 이 값이 실제 값이다. default.yaml 을 바꾸면 8 행 sha 도 갱신한다. [검증: confirmed] |
| should | amhs/software/multi_agent_behavior/multi_agent_behavior/fleet_decision_node.py:2963 | r = self.mutual_block_radius + 0.315 | 내 몸체 반폭 하드코딩 (fleet lens, finder 누락) | 0.32 로 바꾸거나 파라미터로 뺀다. validator lens 밖이므로 fleet lens 와 같이 본다 |
| check | amhs/software/replan_monitor/config/path_validator.params.yaml:66 (주석), 코드 path_validator_node.cpp:173 | # robot_radius: 0.30 (주석). 코드 기본값 declare_parameter("robot_radius", 0.1) | 자기 발자국 폴백. :65 문자열이 깨지면 반경 0.1 원으로 조용히 떨어진다 (finder 누락) | :65 를 고친 뒤 기동 로그에 'Invalid footprint string' 이 없는지 확인한다. 안전 쪽으로 robot_radius: 0.4525 를 넣어 둘 수 있다. 판단 항목이다 |
| check | amhs/software/replan_monitor/config/path_validator.params.yaml:101-173 (107,115,123,131,139,147,157,166,173) | agent 기본값 footprint 0.3 정사각형 (9곳) | agent 발자국 기본값. 현장 체인에서는 common_ammr.yaml 이 덮어써서 쓰이지 않는다 | 동작 영향은 없다. 정리 차원에서 0.32 로 바꾸거나 지운다. path_validator.launch.py 를 직접 띄울 때 (readme 예시) 만 쓰인다. [검증: overstated, should → check] |
| check | amhs/software/replan_monitor/src/path_validator_node.cpp:1683 | const double phase2_footprint_padding_m = 0.01; | 자기 발자국 패딩. 꼭짓점 방향으로 밀어 변은 약 0.007 커진다 | 튜닝 항목이다. 크기 변경만으로는 안 바꿔도 된다. [검증: confirmed] |
| check | amhs/software/replan_monitor/src/path_validator_node.cpp:1667,1670 | phase1_thr = 253; phase1_buffer_m = 0.001 (0셀) | nav2 global 내접 반경 (footprint+padding 0.02, 0.335 에서 0.34 로) | 코드는 그대로 둔다. nav2 global footprint 와 path_validator.params.yaml:65 를 같이 바꾼다. 추가 (기존 틈): 곡선·회전에서는 다각형이 외접 약 0.46 까지 나가는데 Phase 1 은 0.34 만 본다. [검증: confirmed] |
| check | amhs/software/replan_monitor/src/path_validator_node.cpp:1676 | const double backtrack_m = 0.55; | Phase 1 hit 보다 앞에서 다각형이 닿는 거리. 규칙은 패딩 포함 외접 ≈ 0.46 이상이다 | 0.55 로 충분하다. 바꿀 필요 없다. [검증: confirmed, 규칙 보정: 앞 반길이+step 이 아니라 패딩 포함 외접 기준] |
| check | amhs/software/replan_monitor/src/path_validator_node.cpp:1679 | const double heavy_block_area_m2 = 1.0; | 발자국 면적 (패딩 포함 약 0.43 m², 해상도 0.03 에서 약 475 셀 < 기준 1111 셀) | heavy 분기는 지금도 새 크기에서도 동작하지 않는다. 의도가 있다면 면적보다 작게 (예 0.2) 바꾼다. 판단 항목이다. [검증: confirmed] |
| check | amhs/software/replan_monitor/src/path_validator_node.cpp:1961 | if (hit_to_goal <= 0.5) is_goal_occupied = true; | 목표 자세 다각형의 끝 (외접 0.4525 + 패딩 0.007 ≈ 0.46) | 0.5 로 두어도 된다 (여유 약 0.04). 같은 여유 0.05 를 원하면 0.51~0.55. 판단 항목이다. [검증: confirmed] |
| check | amhs/software/replan_monitor/config/path_validator.params.yaml:92,93 | goal_doorstep_static_m: 0.1, goal_doorstep_agent_m: 0.05 | 정적 검사는 비용 253 을 보므로 nav2 global 내접을 자동으로 따른다. agent 검사는 칠해진 몸체를 본다 (finder 누락) | 코드와 값은 그대로 둔다. nav2 global footprint 를 바꾸면 목표 점유 판정이 5 mm 정도 민감해진다 |
| check | amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:1985,1995 | dilation_m: 0.05, load_rear_smear_m: 0.35 (global agent_layer) | agent 가 칠해지는 범위. 0.524 매칭 거리와 맞물린다 (finder 누락) | 크기 변경으로 바꿀 필요는 없다. 0.524 를 다시 정할 때 dilation 을 넣는다. LOADING/UNLOADING agent 의 뒤쪽 0.35 부분 hit 는 중심에서 0.524 보다 멀어 STATIC 으로 분류된다 (기존 동작) |
| check | amhs/software/multi_agent_behavior/config/fleet_decision.yaml:133,140,163 (+ fleet_decision_node.py:634) | mutual_block_radius_m 0.5, same_path_tube_m 0.6, slow_wait_min_body_m 2.2, retreat_body_r_m 0.32 (코드 기본값) | 몸체 간격 기준 값들이다 (fleet lens, finder 누락) | validator lens 밖이다. fleet lens 와 같이 다시 본다. retreat_body_r_m 은 이미 0.32 다 |
| check | amhs/software/replan_monitor/src/path_validator_node.cpp:2460 | if (dist_robot_to_goal <= 0.25) continue; // 내 발밑 | 목표 도착 허용 거리 성격이다. 몸체 크기와 직접 관계는 없다 | 바꿀 근거가 없다. [검증: overstated] |
| check | amhs/software/replan_monitor/config/path_validator.params.yaml:177 | self_footprint_margin_m: 0.05 | 위치 오차 여유. 크기와 무관 | 그대로 둔다. [검증: confirmed] |
| check | amhs/software/replan_monitor/config/path_validator.params.yaml:58,76 | agent_match_dilate_m: 0.001, agent_path_hit_dilate_m: 0.001 | agent 발자국 여유. 발자국 자체는 common_ammr.yaml 에서 온다 | 그대로 둔다. 경로 튜브 검사는 메시지 footprint 를 쓰는데, 현장 winros_bridge 가 빈 다각형을 보내서 늘 건너뛴다. [검증: confirmed, 경로 보정 winros_bridge/winros_bridge/src/winros_bridge.cpp] |
| check | amhs/software/replan_monitor/config/path_validator.params.yaml:15,47 | footprint_step_m: 0.15, agent_mask_manhattan_buffer: 5 (셀) | 샘플 간격과 셀 버퍼. 크기와 무관. 47 은 해상도에 따라 m 가 바뀐다 | 그대로 둔다. [검증: confirmed] |
| check | amhs/software/velocity_modifier/src/velocity_modifier_node.cpp:62 | declare_parameter<double>("wheel_separation", 0.526) | 바퀴 간격 (roboteq track_width). 외곽 0.64 와 다른 양 | 바퀴 간격을 새로 쟀을 때만 바꾼다. config 에 덮어쓰는 값은 없다. [검증: confirmed] |
| check | amhs/software/nav_safety_manager/nav_safety_manager/nav_safety_manager_node.py:108-111 | EXPECTED_RANGES set_area 1/0 패턴 | SICK PLC 안전 필드 세트 (모양은 하드웨어 설정) | 코드는 그대로 둔다. 새 몸체 (옆 0.32) 를 SICK 필드가 덮는지 SICK 설정에서 확인한다. [검증: confirmed] |
| check | amhs/software/navigation/navigation_manager/navigation_manager/roi_range_manager_node.py:57-58 | recovery_footprint_padding 0.0, normal_footprint_padding 0.001 | local costmap footprint_padding 을 상태에 따라 덮어쓴다 (run_all_auto_default.sh:160 으로 뜨고, roi_range_manager.yaml 에는 값이 없다) | nav2_params.yaml:1184 local footprint_padding 과 같은 값으로 유지한다. [검증: confirmed] |
| check | amhs/software/ammr_common_config/CMakeLists.txt, amhs/software/replan_monitor/CMakeLists.txt:27; 121 | install(DIRECTORY config ...) | 설정 반영 경로. launch 는 share 디렉터리에서 읽는다 (finder 누락) | symlink-install 이 아니면 yaml 을 고친 뒤 ammr_common_config, replan_monitor 를 다시 빌드해야 반영된다 |

## fleet

현장 체인: 현장 기동은 amhs/scripts/amr_batch/run_all_auto_default.sh 에서 시작한다 (run_all_auto.sh 도 같다). 확인한 경로는 아래와 같다. - :114 run_amr_bringup.launch.py 는 nav2_params.yaml 을 읽는다. 그 위에 ammr_common_config/config/common_ammr.yaml 의 self_machine_id, robot_ids, 로봇 블록을 덮는다 (:46-57). - :154 run_path_validator.launch.py 는 path_validator.params.yaml 을 읽고, 같은 common_ammr.yaml 로 덮는다 (:33-51). - :157 run_fleet_decision.launch.py 는 fleet_decision.yaml 을 쓰고, my_machine_id 만 넘긴다. - :123 winros_bridge, :169 roi_range_manager 도 같은 스크립트에서 뜬다.  agent_layer.cpp:216-257 은 robot_ids 에 있는 블록만 읽는다. 그래서 nav2_params.yaml 과 path_validator.params.yaml 안의 robot_001, robot_002, warehouse_bot_A, TRAY1..6 블록은 현장에서는 죽은 기본값이다.  common_ammr.yaml 은 .gitignore:95 대상이고 로봇마다 따로 만든 사본이다. git 이 추적하는 원본은 common_ammr_default.yaml 이다. 현장 launch 는 이 원본을 읽지 않는다. 대신 새 로봇의 사본을 만들 때 쓰고, sim 도 base 로 쓴다.  fleet_decision.yaml:111 이 v2_enable: false 라서 v2 함수는 현장에서 돌지 않는다 (_decide_v2 :3027, junction :5449 등).  이웃 크기는 실행 중에 주고받지 않는다. winros_bridge.cpp:1613-1614 가 빈 PolygonStamped 를 넣고, VehicleDrivingInfoData (data_types.hpp:362-392) 에는 크기 필드가 없다.  sim 의 field_stack.launch.py 는 현장 nav2_params.yaml 을 그대로 읽고, 그 위에 overlay 를 씌운다 (:230-444).

| 우선 | 파일:줄 | 지금 | 의존 | 제안 |
|---|---|---|---|---|
| must | /home/tony/docker/isaac-sim-620/volume/work/v2_ws/src/amhs/software/ammr_common_config/config/common_ammr.yaml:18,24,30,36,42,48,54,60,66,72,78,84,90,96,102,108,114,120,126,132,138,144 | footprint "[[0.315, 0.315], [0.315, -0.315], [-0.315, -0.315], [-0.315, 0.315]]" x 22 (0.63 x 0.63) | 이웃 로봇 몸체 다각형 (base_link 기준) | [확인] 내 footprint 와 같은 다각형으로 바꾼다. 대칭이면 [[0.32,0.32],[0.32,-0.32],[-0.32,-0.32],[-0.32,0.32]] 이다. 이 파일은 gitignore 대상이라 22대 로봇에서 각각 직접 고친다. [고칠 점] finder 는 '다시 빌드해야 한다' 고 했다. 하지만 --symlink-install 로 빌드했으면 다시 빌드할 필요가 없다. 이 PC 의 install 은 symlink 다. |
| should | /home/tony/docker/isaac-sim-620/volume/work/v2_ws/src/amhs/software/ammr_common_config/config/common_ammr_default.yaml:15,23,31,39,47,55,64,72 (예시 블록 82, 91) | footprint 0.315 정사각형. robot_ids 안에 robot_002 (machine_id 5, :87) 가 들어 있어 TRAY5 와 겹친다. | 이웃 몸체. 새 로봇 common_ammr.yaml 과 sim base 의 원본이다. | [새 항목] common_ammr.yaml 과 같은 다각형으로 바꾼다. 고치지 않으면 새로 만드는 로봇 사본과 sim 이 옛 0.63 상자로 돌아간다. |
| should | /home/tony/docker/isaac-sim-620/volume/work/v2_ws/src/amhs/software/replan_monitor/src/path_validator_node.cpp:1979, 2754, 2472, 2478, 2569, 2575 | allowed_dist = 0.424 + 0.1, findNearestAgent(..., 0.524) | 이웃 외접 반경. 0.424 는 옛 0.6 상자 값이다. 이웃 칠한 모서리는 0.4525 + dilation 0.05 = 0.5025 m 다. | [확인] 판단 항목이다. 규칙은 외접 반경 0.4525 + dilation_m 0.05 + 지연 0.1 이고, 약 0.60 이다. 6곳을 한 파라미터로 묶는 것이 좋다. [덧붙임] agent_mask_manhattan_buffer 5 셀 x 0.03 m = 0.15 m 만큼 바깥의 hit 도 이웃 표시로 본다. 그래서 지금 0.524 는 모서리 근처에서 여유가 0.02 m 뿐이다. 범위를 벗어나면 이웃을 정적 장애물로 처리한다. |
| should | /home/tony/docker/isaac-sim-620/volume/work/v2_ws/src/amhs/software/multi_agent_behavior/multi_agent_behavior/fleet_decision_node.py:2630 | THRESHOLD = 0.5 (v1, 현장에서 돈다) | 충돌 셀에서 이웃 경로점까지 거리. 몸체 모서리 hit 는 이웃 외접 반경 + dilation = 0.5025 다 (예전 0.4955). | [확인] 판단 항목이다. 0.55 정도를 권한다. 모서리 hit 에서만 차이가 나고, TYPE_3 REROUTE 대신 우선순위 대기로 빠질 수 있다. |
| should | /home/tony/docker/isaac-sim-620/volume/work/v2_ws/src/amhs/software/navigation/navigation_manager/navigation_manager/roi_range_manager_node.py:57-58, 129-158 | recovery_footprint_padding 0.0, normal_footprint_padding 0.001. 실행 중 local_costmap footprint_padding 을 SetParameters 로 덮는다. | 내 footprint 여유 (nav2_params.yaml:1184 local padding 0.001 과 같아야 한다) | [확인] local footprint_padding 을 바꾸면 이 기본값도 같이 바꾼다. config/roi_range_manager.yaml 에 이 키가 없어서 코드 기본값이 그대로 쓰인다. 0.64 가 최외곽 실측이면 지금 값을 유지해도 된다 (판단). |
| check | /home/tony/docker/isaac-sim-620/volume/work/v2_ws/src/amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:1502,1510,1518,1526,1534,1542,1552,1561,1568 (local) / 2007,2015,2023,2031,2039,2047,2057,2066,2073 (global); robot_002 machine_id 1557, 2062; robot_ids 1492, 1997 | agent_layer 이웃 블록 0.315 정사각형. robot_002 machine_id 5 가 TRAY5 와 겹친다. | 이웃 몸체. 현장에서는 common_ammr 가 덮으므로 죽은 기본값이다. | [과장, should 에서 check 로 내림] 현장 동작에는 영향이 없다. 헷갈리지 않게 TRAY 블록은 M1 값으로 맞추고, 예시 블록은 지우는 것을 권한다. bringup_launch.py 를 직접 띄울 때만 쓰인다. |
| check | /home/tony/docker/isaac-sim-620/volume/work/v2_ws/src/amhs/software/replan_monitor/config/path_validator.params.yaml:107,115,123,131,139,147,157,166,173 (robot_002 machine_id 162, robot_ids 97) | 이웃 블록 "[[0.3, 0.3], [0.3, -0.3], [-0.3, -0.3], [-0.3, 0.3]]" (0.6 x 0.6) | 이웃 몸체 (whoCoversPoint 판정). 현장에서는 run_path_validator 가 덮으므로 죽은 기본값이다. | [과장, should 에서 check 로 내림] M1 값으로 맞춘다. path_validator.launch.py 를 직접 띄울 때만 0.6 상자가 쓰인다. 내 footprint (:65) 는 replan 렌즈에서 다룬다. |
| check | /home/tony/docker/isaac-sim-620/volume/work/v2_ws/src/amhs/software/replan_monitor/config/path_validator.params.yaml:58, 47 | agent_match_dilate_m 0.001, agent_mask_manhattan_buffer 5 | 이웃 다각형 판정 여유. agent_layer 는 dilation_m 0.05 만큼 더 크게 칠한다. | [새 항목] 크기에서 나온 값은 아니다. 다만 0.001 과 0.05 가 어긋나 있어서, 칠한 바깥 띠는 findNearestAgent 0.524 로만 이웃으로 판정된다. dilation_m 을 키우거나 0.524 를 바꿀 때 이 두 값도 같이 본다 (판단). |
| check | /home/tony/docker/isaac-sim-620/volume/work/v2_ws/src/amhs/software/multi_agent_behavior/multi_agent_behavior/fleet_decision_node.py:2963 | r = self.mutual_block_radius + 0.315 (내 반폭 하드코딩) | 내 반폭 | [과장, should 에서 check 로 내림] 0.32 로 바꾸거나 retreat_body_r_m 을 쓴다. _target_blocked_by_me 는 _decide_v2 (:3027) 에서만 부르므로 v2_enable false 동안에는 돌지 않는다. v2 를 켜면 should 다. |
| check | /home/tony/docker/isaac-sim-620/volume/work/v2_ws/src/amhs/software/multi_agent_behavior/multi_agent_behavior/fleet_decision_node.py:5379, 5356, 5385 | _ahead_clear _map_free(x, y, 0.3), 이웃 0.8. _in_junction probe 0.3 | 내 반폭 0.32 (0.3 원은 0.6 몸체 기준이다) | [확인, check 로 내림] :5379 는 0.32 이상으로 하거나 yield_clearance_m 을 쓴다. :5356 은 교차로 탐지값이라 유지해도 된다 (판단). 둘 다 v2 junction_keep_clear (:5449 게이트) 에서만 쓴다. |
| check | /home/tony/docker/isaac-sim-620/volume/work/v2_ws/src/amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:1480, 1985 | agent_layer dilation_m: 0.05 | 이웃 위치 양자화 오차와 몸체 여유 | [확인] 0.64 가 범퍼까지 포함한 최외곽이면 유지한다. 돌출부가 빠진 값이면 그만큼 키운다 (판단). 키우면 path_validator 0.524 와 fleet 0.5 도 같이 키운다. |
| check | /home/tony/docker/isaac-sim-620/volume/work/v2_ws/src/amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:1490, 1995 | load_rear_smear_m: 0.35 (코드 기본값 0.4, agent_layer.cpp:149) | 이적재 중 뒤로 나오는 부분 | [확인] 0.64 와 따로 잰 값인지 확인하고 실측으로 맞춘다. |
| check | /home/tony/docker/isaac-sim-620/volume/work/v2_ws/src/amhs/software/multi_agent_behavior/config/fleet_decision.yaml:133, 140, 163, 168, 174, 221, 279 (+ node 621, 597, 5385) | mutual_block_radius_m 0.5, same_path_tube_m 0.6, slow_wait_min_body_m 2.2, predict_conflict_dist_m 0.8, yield_backoff_body_m 1.0, standoff_dist_m 1.4, cycle_swap_partner_goal_m 0.8, yield_goal_occupied_m 0.7, junction_exit_clear_m 0.8 (코드 기본값) | 몸체가 닿는 중심 거리 (면끼리 0.64, 모서리끼리 0.905) | [확인] 숫자를 바꿀 필요는 없다고 본다 (판단). 모두 v2 전용이다. v2 를 켜면 '닿을 수 있음' 뜻의 값 (0.7, 0.8) 은 모서리 접촉 0.905 보다 작다는 점을 다시 본다. |
| check | /home/tony/docker/isaac-sim-620/volume/work/v2_ws/src/amhs/software/multi_agent_behavior/multi_agent_behavior/fleet_decision_node.py:634, 683, 5488 | retreat_body_r_m 0.32 (yaml 에 없음), yield_clearance_m 0.35 | 내 반폭 (0.32 는 새 반폭과 같다) | [확인] 그대로 둔다. :634 주석의 '중심거리 0.73' 은 0.315 기준 계산이다. 새 크기에서는 0.74 다. 돌출부가 빠진 값이면 같이 키운다. v2 전용이다. |
| check | /home/tony/docker/isaac-sim-620/volume/work/v2_ws/src/amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:750, 1198, 1716 | YieldFollowPath CostCritic consider_footprint: true, local inflation_radius 0.5, global 0.61 | padding 을 포함한 외접 반경. local 은 (0.321,0.321) 이라 약 0.454, global 은 (0.34,0.34) 라 약 0.481 이다. | [확인] 둘 다 inflation 보다 작아서 지금은 괜찮다. inflation 이나 padding 을 바꿀 때만 다시 본다. nav2 렌즈와 겹친다. |
| check | /home/tony/docker/isaac-sim-620/volume/work/v2_ws/src/amhs/software/winros_bridge/winros_bridge/src/winros_bridge.cpp:1613-1614 (+1460, 1600) | MultiAgentInfo.footprint 를 빈 PolygonStamped 로 채운다. 내 자세는 base_link 원점을 map 으로 변환한 값이다. | 이웃 크기를 전달하는 방식 | [확인] 코드는 바꿀 필요가 없다. 이웃 크기는 각 로봇의 common_ammr.yaml 에서만 온다. 이웃 다각형은 base_link 기준 좌표로 쓴다. 관제 쪽 크기 설정은 따로 확인한다. |
| check | /home/tony/docker/isaac-sim-620/volume/work/v2_ws/src/amr-gz-sim-example/mobile_robot_gz_sim/amhs_overrides/common_ammr/robots_sim.yaml:16 | sim 22대 template footprint 0.315 정사각형 | sim 의 이웃 몸체 | [확인, 고칠 점] 현장 값과 맞춘다 (sim 사본). finder 가 sim base 라고 한 common_ammr_default.yaml 은 amhs 안의 git 원본이다. 그래서 별도 항목 (should) 으로 옮겼다. |
| check | /home/tony/docker/isaac-sim-620/volume/work/v2_ws/src/amr-gz-sim-example/amr_sim_robots/spec/simple.yaml:95-96 (+ mobile_robot_gz_sim/urdf/mobile_robot_gz.urdf.xacro:76-79) | sim 로봇 접촉 외형 contact_lower/upper x,y ±0.315 (0.63 상자) | sim 에서 이웃 몸체가 스캔에 보이는 크기, 물리 접촉 크기 | [새 항목] sim 결과를 현장과 맞추려면 ±0.32 로 바꾼다 (sim 사본, 현장 동작과는 무관하다). |

## plugins

현장 체인: 현장 체인: scripts/amr_batch/run_all_auto_default.sh 에서 ros2 launch amr_bringup run_amr_bringup.launch.py 를 띄운다. 기본 params 는 amr_bringup/params/nav2_params.yaml 이다 (run_amr_bringup.launch.py:94). 그다음 bringup_launch.py, navigation_launch.py 순서로 이어진다. run_amr_bringup.launch.py:18-34 와 :46-57 은 ammr_common_config/config/common_ammr.yaml 을 읽는다. 그리고 self_machine_id, self_type_id, robot_ids, 로봇별 블록(footprint 포함)을 메모리에서 덮어쓴다. 그래서 agent_layer 가 그리는 다른 로봇 footprint 는 common_ammr.yaml 이 단일 소스다. nav2_params.yaml 의 로봇 블록(P:1502-1568, 2007-2073)은 쓰이지 않는 기본값이다. controller_plugins (P:147) 는 FollowPath, RecoveryFollowPath1, RotationGraceful, ReverseRPP, StraightRPP, YieldFollowPath 이고 모두 쓰인다. planner_plugins 는 P:894 와 static_nav P:2752 다. multi_agent_nav2::AgentLayer 는 local(P:1453)과 global(P:1958)에서 enabled: true 다. finder 목록에서 빠져 있었는데 실제로 로드된다. sim 의 field_stack.launch.py 는 현장 nav2_params.yaml 을 쓴다. 신원 정보는 common_ammr_default.yaml 에 amhs_overrides/common_ammr/robots_sim.yaml 을 합쳐 만든다. (P = amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml)

| 우선 | 파일:줄 | 지금 | 의존 | 제안 |
|---|---|---|---|---|
| must | amhs/software/ammr_common_config/config/common_ammr.yaml:18,24,30,...,144 (6줄 간격, 22곳, AMMR_2P_TRAY1~22) | footprint: "[[0.315, 0.315], [0.315, -0.315], [-0.315, -0.315], [-0.315, 0.315]]" (0.63 정사각, 다른 로봇 크기) | 다른 로봇(agent)의 footprint 그 자체. agent_layer 가 다른 로봇 몸체를 이 다각형으로 costmap 에 칠한다 | [[0.32,0.32],[0.32,-0.32],[-0.32,-0.32],[-0.32,0.32]] 로 바꾼다. 실측이 base_link 기준 비대칭이면 실측 꼭짓점을 넣는다. gitignore 파일이라 로봇마다 직접 고쳐야 한다. 외접 반경은 꼭짓점에서 자동 계산된다 (finder 가 놓친 항목) |
| must | amhs/software/ammr_common_config/config/common_ammr_default.yaml:15,23,31,39,47,55,64,72,82,91,98 (machine_id 중복: :43 TRAY5 와 :87 robot_002 가 둘 다 5) | footprint 0.315 정사각 11곳. robot_ids 에 robot_001, robot_002, warehouse_bot_A 예시가 있다 | agent footprint. 이 파일은 현장 common_ammr.yaml 의 원본 템플릿이고 sim base 이기도 하다 | 11곳 모두 0.32 정사각(또는 실측 꼭짓점)으로 바꾼다. 이 파일을 복사해 robot_ids 를 손대지 않으면 machine_id 5 는 뒤에 오는 robot_002 블록으로 덮인다. 그러니 robot_002 도 같이 고치거나 지운다 |
| should | amr-gz-sim-example/mobile_robot_gz_sim/amhs_overrides/common_ammr/robots_sim.yaml:16 | template footprint: "[[0.315, 0.315], [0.315, -0.315], [-0.315, -0.315], [-0.315, 0.315]]" (sim 22대) | sim 에서 쓰는 agent footprint 사본 | 현장과 같은 새 footprint 로 바꾼다. base_sha256_at_write 검사가 있으면 default 를 바꾼 뒤 같이 갱신한다. sim urdf·spec 의 다른 0.315 는 sim 렌즈 소관이다 |
| check | amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:403-407 (ReverseRPP), 878-880 (StraightRPP) | cost_scaling_dist: 0.4 / 0.55, use_cost_regulated_linear_velocity_scaling: true (둘 다) | inscribed radius. 장애물 거리 추정이 로봇 중심 기준이라 옆면 여유 = cost_scaling_dist - inscribed 다 | 옆면 여유를 지금과 같게 두려면 0.405 / 0.555 로 +0.005 올린다. 그대로 둬도 된다 (튜닝 항목) |
| check | amhs/software/navigation/amr_navigation2/advanced_regulated_pure_pursuit_controller/src/parameter_handler.cpp (+ straight_pure_pursuit_controller/src/parameter_handler.cpp):74 (yaml 키 없음: P:371 ReverseRPP, P:845 StraightRPP) | inflation_cost_scaling_factor 기본 3.0. local costmap cost_scaling_factor 는 10.0 (P:1197) | footprint 와 무관한 예전부터 있던 불일치다. 거리 추정 공식에 inscribed 가 들어가지만 그 값은 자동 반영된다 | 판정 overstated (should 에서 check 로 내림). 0.64 변경과 별개다. 맞추려면 두 블록에 10.0 을 명시하고 cost_scaling_dist 를 다시 튜닝한다. sim 검증이 필요하다 (판단 항목) |
| check | amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml (코드: regulated_mppi_controller/src/critics/cost_critic.cpp, yield_mppi_controller 도 같음):1184, 1198 (코드 86-111); global 1684, 1716; static_nav 2813, 2839 | local padding 0.001 / inflation_radius 0.5, global padding 0.02 / 0.61, static_nav padding 0.02 / 0.7 | padding 을 포함한 circumscribed radius. 새 값은 local 0.4540, global·static 0.4808 이다 | 판정 overstated (should 에서 check 로 내림). 지금 값으로 모두 규칙을 만족하니 바꿀 필요가 없다. 규칙: inflation_radius >= sqrt(2)*(0.32+padding). local padding 이 0.0336 을 넘거나 global padding 이 0.111 을 넘을 때만 inflation_radius 를 올린다 |
| check | amhs/software/navigation/amr_navigation2/ammr_navfn_escape_planner/src/navfn_planner.cpp:700-716 (canRotate), 829-835 (직진 치수); yaml P:1138-1143 | canRotate 는 padded footprint 의 외접원, 직진 검사는 unpadded min/max 를 매 plan 마다 다시 잰다. escape_margin 0.15, escape_max_len 0.45, escape_max_dist 1.5 | footprint (자동). planner_server 는 global costmap 이라 padded 외접은 0.4808 이다 (옛 0.4773) | 코드와 값 모두 그대로 둔다. 판정 confirmed (보정): 회전 옆 여유는 padded 기준 0.161 (옛 0.162), unpadded 기준 0.1325 (옛 0.134). 직사각형이고 base_link 가 회전 중심이라는 가정이 들어 있다 |
| check | amhs/software/navigation/amr_navigation2/ammr_navfn_escape_planner/include/ammr_navfn_escape_planner/navfn_planner.hpp:255 (주석 navfn_planner.cpp:653, README.md:5) | double fp_front_{0.32}, fp_back_{-0.315}, fp_left_{0.315}, fp_right_{-0.315}; 주석에는 `정사각 차체(0.635x0.63, ... 옆 여유 0.134 m)` | 옛 footprint 치수. 초기값과 주석일 뿐이다 | 동작에는 영향이 없다. 정리하고 싶으면 {0.32,-0.32,0.32,-0.32} 로 바꾸고, 주석은 0.64x0.64, 0.1325 m 로 고친다 (선택) |
| check | amhs/software/navigation/amr_navigation2/amr_bringup/behavior_trees/moduler33_wo_agent_layer_replan_in_pause_timeout_pub_goals_rmv_goal_in_recovery_pause_hold_op_pause_gate.xml:835, 1001 | TruncatePathLocal distance_forward="0.45" | 근거로 보면 회복 기동 길이다. 옛 외접 반경 0.449 와 숫자가 비슷한 것은 우연일 가능성이 높다 | 그대로 두는 쪽이다. 외접 반경에 맞춘 값이라는 근거는 없다. 사용자 의도만 확인한다 |
| check | amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:934, 974, 1103, 2772 | minimum_turning_radius: 0.3 | 운동학 값이고 footprint 와 직접 관계는 없다. footprint 는 smac 이 매 plan 마다 갱신한다 | 그대로 둔다 (판단 항목) |
| check | amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:402, 875 (curvature_lookahead_dist), 397, 870 (regulated_linear_scaling_min_radius 주석) | curvature_lookahead_dist 0.5 / 0.6 에 `padding:0.05 -> 0.65` 주석, min_radius 1.5 / 2.0 에 `min_turning_radius = length /2 = 0.35 m` 주석 | 크기는 주석에만 나온다. 값 자체는 크기와 무관하다 | 값은 그대로 둔다. 원하면 주석의 length/2 를 0.32 로 고친다 (선택) |
| check | amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:1480, 1490 (local agent_layer), 1985, 1995 (global agent_layer) | dilation_m: 0.05, load_rear_smear_m: 0.35 | 크기에서 나온 값이 아니다. agent 몸체 footprint 위에 더하는 여유다 | 그대로 둬도 된다. agent footprint 를 0.64 로 바꾸면 칠해지는 몸체는 한쪽에 5 mm 커진다 (판단 항목) |
| check | amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:1502-1568, 2007-2073 (agent 로봇 블록 9개씩, machine_id 5 중복 1530/1557, 2035/2062) | footprint 0.315 정사각 (context 에서는 'collision-checker 블록'이라고 했지만 실제로는 agent_layer 로봇 블록이다) | agent footprint 의 쓰이지 않는 기본값. run_amr_bringup 이 common_ammr.yaml 로 덮어쓴다 | 현장 동작에는 영향이 없다. bringup_launch.py 를 직접 띄우는 경우에만 이 값과 machine_id 5 중복이 살아난다. 일관성을 위해서만 고친다 |

## hw_sim

현장 체인: 현장 기동은 amhs/scripts/amr_batch/run_all_auto.sh 다. robot_model 기본값은 v5 다 (54행). 주석을 빼면 run_all_auto_default.sh 와 같다. - 확인한 줄: 101행 mobile_robot_bringup (v5 xacro, mobile_robot_lidar.yaml, mobile_robot_motor_v5.yaml), 104행 scan_merger (scan_merger_ros2_real.yaml, merged frame base_link), 107행 pointcloud_to_laserscan (인라인 파라미터, range_min 0.1), 114행 run_amr_bringup (nav2_params.yaml 에 common_ammr.yaml 을 병합, run_amr_bringup.launch.py:46-58), 129행 lidar_auto_docking (autodock_params.yaml, autodock_launch.py:17), 135행 marker_auto_docking (marker_auto_docking.yaml, launch:13), 145행 safety_plc (yaml 이 비어 있음), 166/169/172행 zed 노드. - docking_sequence_qr 와 ir_line_follower 는 scripts/ 어디에도 기동 줄이 없다. 그런데 winros_bridge_to_dock_node.cpp:74-86 이 이 둘의 action 을 부른다. - sim 은 field_stack.launch.py 가 현장 nav2_params.yaml 에 sim_overlay*.yaml 을 얹는다. 오버레이에는 footprint 키가 없다. preflight_amhs.py:100 이 오버레이에서 footprint, footprint_padding, robot_radius 를 바꾸면 실패로 막는다. 그래서 amhs 를 고치면 sim 도 따라간다. - sim 로봇 기본값은 legacy 다 (launch_multi.sh:57-61). simple 과 cad 는 선택이다. 백엔드는 gz 와 isaac 둘 다 쓴다 (paths.sh:14).

| 우선 | 파일:줄 | 지금 | 의존 | 제안 |
|---|---|---|---|---|
| must | amhs/software/ammr_common_config/config/common_ammr_default.yaml (+ 현장 gitignore common_ammr.yaml):default 15,23,31,39,47,55,64,72,82,91,98; common_ammr.yaml 18-144 (6행 간격, 22 대) | footprint "[[0.315, 0.315], [0.315, -0.315], [-0.315, -0.315], [-0.315, 0.315]]", robot_radius 0.0 | agent footprint (다른 로봇의 크기) | [MISSED] ±0.32 정사각형으로 바꾼다. 비대칭으로 재기로 정하면 그 값을 쓴다. 기계적 변경이다. 이 블록은 run_amr_bringup 이 nav2 로봇 블록으로 복사한다. nav lens 와 겹칠 수 있다. sim 의 robots_sim.yaml 은 이 파일을 base 로 쓴다. |
| must | sim/mobile_robot_gz_sim/amhs_overrides/common_ammr/robots_sim.yaml:16 (8) | template footprint "[[0.315, 0.315], ...]". 이 template 으로 22 대를 만든다. base_sha256_at_write 는 b6079a6c… 다. | agent footprint | [confirmed] 현장 common_ammr 와 같은 새 값을 쓴다. 현장 default yaml 이 바뀌면 base_sha256_at_write 를 갱신한다. 그리고 common_ammr_sim.py 검사를 다시 돌린다. |
| must | sim/mobile_robot_gz_sim/urdf/mobile_robot_gz.urdf.xacro:162, 172 (주석 76-79, 158) | visual box 0.63 0.63 0.42, collision box 0.63 0.63 0.24 이다. origin x 는 0 이다. | 몸체 폭과 길이. 다른 로봇의 gpu_lidar 가 이 visual 을 본다. | [confirmed] 두 상자의 x 와 y 를 0.64 로 바꾼다. 앞뒤가 비대칭으로 측정되면 origin x 도 옮긴다. 캐스터 ±0.21 + 0.10 은 여전히 몸체 안이다. sim legacy 가 기본 로봇이다. |
| must | sim/isaac/urdf/mobile_robot_gz.urdf, isaac/urdf/mobile_robot_gz_rsp.urdf, isaac/usd/mobile_robot_gz/payloads/base.usda, isaac/run_fleet.py:urdf 35,47; usda 27,65,340; run_fleet 556 | box 0.63 이다. USD scale 은 (0.63,0.63,0.42) 와 (0.63,0.63,0.24) 다. extent 는 ±0.315 다. skirt 'full' 은 0.63 이다. | Isaac 쪽 legacy 몸체 | [confirmed] Isaac 백엔드를 쓸 때만 해당한다. 먼저 xacro 를 고친다. 그다음 import_robot.py 절차로 urdf 를 다시 만들고 USD 로 다시 import 한다. 파일을 손으로 고치지 않는다. run_fleet.py:556 의 'full' 값 0.63 은 0.64 로 바꾼다. 기본값 'inner' 0.43 은 자기 라이다 사각 안에 들어가게 정한 값이라 그대로 둔다. |
| should | amhs/software/docking/lidar_auto_docking/config/autodock_params.yaml:158-159, 190-191, 165 | docking_server 와 undocking_server 의 obstacle_detection 에 robot_width 0.6, robot_length 0.6 이 있다. field_end_margin 은 0.03 이다. 코드 기본값도 0.6 이다 (autodock.h:155-156, 424-425). | 로봇 폭과 반길이. 필드 폭은 ±W/2, 시작은 L/2, 먼 끝은 L/2 + (남은거리 - margin) 이다. | [confirmed] robot_width 는 0.64 로 바꾼다 (기계적 변경). robot_length 는 판단 항목이다. 0.64 로 바꾸면 먼 끝이 dock 쪽으로 2 cm 밀린다. 그러면 dock 하우징이 필드에 들어가 로봇이 dock 앞에서 서는 예전 버그가 다시 날 수 있다. 그래서 길이를 0.64 로 바꿀 때는 field_end_margin 을 0.05 로 같이 올린다. 그러면 먼 끝 위치가 지금과 같다. 아니면 길이를 0.6 으로 둔다. undocking 쪽은 margin 을 쓰지 않으므로 0.64 / 0.64 로 바꾸면 된다. |
| should | amhs/software/docking/marker_auto_docking/config/marker_auto_docking.yaml:3-4 (marker_auto_docking.cpp 44-47, 304, 334-352, 848-851) | robot_length 0.5, robot_width 0.5 이다. 코드에 robot_edge_inflation 0.05 가 하드코딩되어 있다. 선언 기본값은 0 이다 (hpp:295-296). | 로봇 모서리 4 점 (L/2, W/2). 장애물 필드 폭 (W/2). 필드 시작 거리 (sqrt(L²+W²)/2 = circumscribed). | [confirmed + 보강] 0.64 / 0.64 로 바꾼다. 그러면 필드 시작이 0.354 에서 0.4525 로 바뀌고, 필드는 한쪽이 7 cm 넓어진다. 모서리 판정 거리는 0.30 에서 0.37 로 커진다. 이 판정 (getSafetyVelocityLocal) 은 1 초 뒤 모서리가 태그 선 (tag_pose_base_link_) 을 넘는지 본다. 그래서 태그 쪽 여유가 7 cm 줄어든다. 감속과 제자리 회전이 더 자주 날 수 있다. 하드코딩된 inflation 0.05 를 0.02–0.03 으로 줄일지는 튜닝 항목이고 실차 시험이 필요하다. |
| should | amhs/software/ir_line_follower/config/params.yaml:25-27 | obstacle.distance 0.3, width 0.6, height 0.4 이다. 코드 기본값은 0.2 / 0.4 / 0.5 다 (line_follower_action_server.cpp:44-46). | 앞면 위치 (반길이 0.32)와 로봇 폭. 판정은 /scan 기준이다. /scan 은 merged scan 이고 frame 은 base_link 다 (scan_merger_ros2_real.yaml:11). | [confirmed] width 는 0.64 이상으로 둔다. 여유를 더 둘지는 판단 항목이다. distance 는 0.32 로 바꾼다. 주석은 'AMR 앞면 기준' 이라고 하지만 코드는 base_link 기준이다. 지금처럼 먼 끝을 0.70 에 두려면 height 를 0.38 로 한다. 이 노드는 run_all_auto.sh 에 없다. 그래서 따로 기동하는 경우에만 해당한다. |
| should | sim/amr_sim_robots/spec/simple.yaml + tools/check_spec.py:spec 95-96 (91, 110-112); check_spec 99, 104 | contact 상자 x/y 가 [-0.315, 0.315] 다. check_spec 에 0.315 가 하드코딩되어 있다. scanband 셸 반폭은 0.295 다 (CAD 값). | 몸체 반폭 (접촉 외형) | [overstated → should] SIM_ROBOT=simple 일 때만 쓴다. 기본값은 legacy 다. contact 는 ±0.32 로 바꾼다. check_spec 의 0.315 는 0.32 로 바꾸거나 spec 에서 읽게 한다. scanband 0.295 는 스캔 높이에서 실물을 재기 전까지 그대로 둔다. |
| should | sim/mobile_robot_gz_sim/scripts/verify/harness/multi/{mindist.py, longrun_watch.py, ab_compare.py, crossing_report.py, collision_backup_test.py, r2_driver.py}:mindist 25; longrun_watch 17; ab_compare 216; crossing_report 63; collision_backup_test 37; r2_driver 24 | 접촉 임계 0.73 (=0.63+0.10), 몸체 간격 d-0.63, HALF = 0.315 | 몸체 폭 0.63 / 반폭 0.315 | [confirmed] 0.74, d-0.64, HALF 0.32 로 바꾼다. 상수 하나로 모으는 것이 좋다. collision_curve_test.py:34 는 collision_backup_test 의 HALF 를 가져다 쓰므로 함께 바뀐다. sim 보고서의 정확도 항목이다. |
| should | sim/mobile_robot_gz_sim/scripts/verify/fleet_relay_server.py:407 | GOAL_BLOCK_M = 0.75 | 몸체 반폭 x 2 + 여유 (0.315*2 + 0.12) | [MISSED] 여유를 지금과 같게 두려면 0.76 으로 바꾼다. 이 릴레이는 sim 판마다 뜬다 (launch_multi.sh:570). |
| should | sim/mobile_robot_gz_sim/scripts/verify/verify_retreat_v2_units.py, scripts/verify/amhs_tests/amhs_verify_retreat_v2_units.py, scripts/verify/amhs_tests/unit/test_q58a_ended_path.py:verify_retreat 459-460 (468); amhs_verify_retreat 465-466 (474); test_q58a 293 | _lc_fp = [(0.32, 0.315), (0.32, -0.315), (-0.315, -0.315), (-0.315, 0.315)], rear = -0.315; r = mutual_block_radius + 0.315 | 현장 local footprint 사본, 몸체 반폭 | [MISSED] 현장 footprint 를 바꾸면 이 단위 시험의 사본도 같이 바꾼다 (rear -0.32, +0.32). 기대 범위 0.26–0.32 도 다시 본다. |
| should | sim/mobile_robot_gz_sim/scripts/verify/safety_fields/field_margins.py:152 | --half 0.315 (기본) | 몸체 반폭 | [confirmed] 0.32 로 바꾼다. 그다음 SAFETY_FIELDS_1102.md 2절 표를 다시 계산한다. |
| should | sim/mobile_robot_gz_sim/scripts/verify/harness/multi/experiments/body_overlap.py, scripts/verify/amhs_tests/unit/test_body_overlap.py:body_overlap 174, 330-331; test 25, 27 (62) | fp_x/fp_y 0.63, FP = 0.63, 반대각선 0.4455 | 몸체 정사각형, 반대각선 | [confirmed] 0.64 와 0.4525 로 바꾼다. 시험 기대값도 갱신한다 (0.8915 → 0.905). |
| check | SICK nanoScan3 / Flexi Soft 필드 (스캐너 안의 설정). 근거 문서: sim/mobile_robot_gz_sim/scripts/verify/safety_fields/SAFETY_FIELDS_1102.md:50-63, 78-81 | ±0.315 몸체를 기준으로 계산한 protective 여유다. 정지와 전진 때 옆이 1.8–3.6 cm 다. Narrow 왼쪽은 0.3–2.1 cm, Docking 뒤는 -1.6–0.6 cm, 후진 오른쪽은 -0.8–2.6 cm 다. | 몸체 반폭과 반길이 (필드와 몸체 변 사이 여유) | [confirmed] 실물은 같으므로 필드는 이미 ±0.32 몸체를 덮고 있었다. 숫자만 다시 읽으면 된다. 모든 변의 여유가 약 0.5 cm, 대각은 약 0.7 cm 줄어든다. Narrow 왼쪽은 -0.2–1.6, Docking 뒤는 -2.1–0.1, 후진 오른쪽은 -1.3 부터가 된다. 필드를 다시 설계할지는 안전 담당이 정한다 (Safety Designer 와 안전 평가가 필요하다). |
| check | amhs/mobile_robot/mobile_robot_description/urdf/include/mobile_robot_v5.urdf.xacro:161, 177, 194, 210 (12) | D435 카메라 4 대가 x ±0.35 에 있다. d435_camera 의 기본값은 true 다. ZED 는 ±0.29, 스캐너는 (±0.218, ∓0.218, 0.19) 에 있다. | 몸체 경계 (반길이 0.32) | [confirmed] D435 는 0.32 경계보다 3 cm 밖이다. realsense 와 D435 토픽은 run_all_auto.sh 에도 nav2_params 에도 없다. 실제로 달려 있으면 0.64 측정에 넣어야 한다. 달려 있지 않으면 TF 를 정리한다. 확인이 필요하다. |
| check | amhs/mobile_robot/mobile_robot_description/urdf/include/mobile_robot_v5.urdf.xacro:68-83, 30 (wheel.urdf.xacro:7) | visual 과 collision 은 mir_250_base.stl 이다. 다시 계산한 bbox 는 collision x ±0.3997, y ±0.2898 이고 +0.0376 offset 이 있다. 바퀴 dy 는 0.2226 이다. | 몸체 형상, base_link 와 회전 중심의 관계 | [confirmed] RViz 와 TF 에만 쓰이고 Nav2 에는 영향이 없다. 0.64 상자로 바꾸는 것은 선택이다. 바퀴 간격 0.445 와 roboteq track_width 0.526 (mobile_robot_motor_v5.yaml:13) 이 다르다. 그래서 이 URDF 로는 회전 중심을 판단할 수 없다. |
| check | amhs/software/docking/docking_sequence_qr/include/docking_sequence_qr/docking_sequence_qr.hpp:353-354 (cpp 111-114, 1406-1409, 1902) | const double robot_width = 0.5; const double robot_height = 0.7 (길이); inflation 0.05 하드코딩 | 로봇 폭과 길이 (모서리, 장애물 필드) | [confirmed] 이 노드를 쓰는 경우에만 0.64 / 0.64 로 바꾼다. 하드코딩 대신 파라미터로 빼는 것이 좋다. 길이가 0.7 에서 0.64 로 줄어드는 점에 주의한다. 이 노드는 어떤 기동 스크립트에도 없다. |
| check | amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:1239-1247, 1794-1802 | nvblox_layer 는 local 과 global 모두 enabled: false 다 (1241, 1796). inflation_distance 는 0.34 다 (1246, 1801). | inscribed (0.32) + 여유 | [confirmed + 보강] 같은 값이 global 1801 에도 있다. 지금은 둘 다 꺼져 있다. 다시 켤 때 0.345 정도로 할지 정한다 (판단 항목). |
| check | sim/mobile_robot_gz_sim/scripts/verify/harness/multi/bake_keepout_mask.py, build_network.py:bake 14, 28; build_network 118, 236, 280-286 | --strip 0.63 (로봇 폭 띠), 차선 ±0.40, 주석 'possible_inscribed_cost(약 61)' | 로봇 폭, inscribed/circumscribed (inflation 비용) | [confirmed, 일부 overstated] 1 cm 차이라 다시 구울지는 선택이다. global padding 0.02 와 csf 10 을 넣어 계산하면 possible_inscribed_cost 는 60.7 에서 61.6 이 된다 (정수로 60 에서 61). 띠 비용 62 와의 차이가 2 에서 1 로 줄지만 순서는 그대로다. 차선 ±0.40 에서 두 로봇 몸체 간격은 0.16 이 된다. 현장 keepout 지도 (251223_map_keepout) 는 repo 에 없다. 그래서 현장 띠가 로봇 폭이라는 말은 확인할 수 없다. |
| check | sim/mobile_robot_gz_sim/scripts/verify/{path_clearance.py, obstacle_manager.py, wiggle_box.py, inject_grazing_box.py}:path_clearance 93; obstacle_manager 54-55; wiggle_box 9; inject_grazing_box 70 | --footprint 0.63, ROBOT_FOOTPRINT = 0.63 (동적 장애물 크기 상한), 설명 0.315+0.15=0.465, help 'footprint 0.315' | 몸체 한 변 / 반폭 | [MISSED] 보고용이나 도구의 기본값이다. 0.64 로 맞출지는 선택이다. path_clearance 의 기본값은 0.64 로 바꾸는 것이 맞다. |
| check | sim/amr_sim_robots/urdf/cad.urdf.xacro:7 | cad_collision:=hull 이 기본이다 (CAD hull ±0.315). | CAD 형상 (실측과 다름) | [confirmed] 0.64 를 따르려면 cad_collision:=box 를 쓴다. box 는 spec contact 값을 쓴다. 아니면 CAD hull 을 키운다. 사용자가 정한다. SIM_ROBOT=cad 일 때만 해당한다. |
| check | amhs/software/docking/lidar_auto_docking/config/autodock_params.yaml:50 (48, 81, 172, 186) | docked_distance_threshold 0.35, connector_clearance_distance 0.5/0.42, dock_x_offset 0.127, undock_distance 0.5, undock_complete_distance 0.3 | dock 기하와 실측 접촉 위치. 로봇 실물은 바뀌지 않았다. | [confirmed, 바꾸지 않음] 실측으로 맞춘 값이다. 로봇 실물이 같으므로 그대로 둔다. docking_v5.yaml:19,22 의 distance_targetpose_* 도 같은 이유로 그대로 둔다. |
| check | amhs/hardware/sensors/zed_box_to_laser/config/zed_box_to_laser.yaml, three_scan_merger_ros2/params/scan_merger_ros2_real.yaml, pointcloud_to_laserscan_scan.launch.py, winros_bridge.cpp:zed 20; merger 18; p2l range_min; winros_bridge 1613-1614 | fixed_footprint_size 0.5, range_min 0.0 / 0.1, multi_agent_info.footprint 는 빈 polygon 이다. | 로봇 크기와 관련이 없다 (확인함) | [MISSED 확인, 바꾸지 않음] fixed_footprint_size 는 검출된 물체의 최소 크기다. 로봇 크기가 아니다. 스캔 경로에는 몸체 crop 이나 자기 필터가 없다. 스캐너는 몸체 안 (±0.218) 에 있다. agent 크기는 common_ammr 에서 온다. |
| check | sim/mobile_robot_gz_sim/config/safety_plc/nanoscan3_fields_1102.json:전체 | 현장 스캐너 필드 사본이다 (11-02 수집). | 현장 필드 설계 (몸체 크기와 간접적으로만 관련) | [confirmed] 에뮬레이터는 몸체 크기를 쓰지 않는다. 현장 필드를 다시 설계할 때만 make_field_config.py 로 다시 만든다. |

## sweep

현장 체인: 현장 체인: amhs/scripts/amr_batch/run_all_auto_default.sh:114 (run_all_auto.sh:114, mobile_robot_localization.sh:47 도 같다) -> amr_bringup/launch/run_amr_bringup.launch.py:94 (기본 params = amr_bringup/params/nav2_params.yaml, :46 에서 ammr_common_config/config/common_ammr.yaml 로 덮어쓴다) -> bringup_launch.py:239-259 -> navigation_launch.py:95. BT 는 nav2_params.yaml:60 의 moduler33_wo_agent_layer_..._op_pause_gate.xml. 나머지 노드는 같은 sh 의 101-172 줄이다. path_validator = replan_monitor run_path_validator.launch.py -> path_validator_node (CMakeLists 76-80, path_validator.params.yaml). fleet = multi_agent_behavior run_fleet_decision.launch.py (fleet_decision.yaml). docking = lidar_auto_docking autodock_launch.py 와 marker_auto_docking.launch.py. localization = mobile_robot_localization_norviz.launch.py:27 -> cartographer_localization.lua. URDF = mobile_robot_bringup.launch.py:21 의 robot_model 기본값 v5 -> mobile_robot_v5.urdf.xacro. sim overlay (mobile_robot_gz_sim/params/sim_overlay*.yaml) 는 footprint 를 덮어쓰지 않는다. 그래서 sim 도 현장 nav2_params.yaml 값을 그대로 쓴다.

| 우선 | 파일:줄 | 지금 | 의존 | 제안 |
|---|---|---|---|---|
| check | amhs/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml:1421, 1447 (local obstacle_layer) / 1920, 1947 (global obstacle_layer); zed layer 도 같다: local 1261/1274, 1281/1294, 1302/1315, 1322/1335, 1344/1357, global 1816/1829, 1858/1871, 1878/1891, 1900/1913 | footprint_clearing_enabled: false, obstacle_min_range: 0.0 | 몸체 외곽과 센서 위치의 관계 (self-hit). footprint 로 지우지 않아서, 센서가 몸체를 보면 그 점이 그대로 장애물이 된다. | 0.64 로 바꾼 뒤 정지 상태에서 /scan_matched_points2 와 zed 점이 0.64 상자 안에 찍히는지 본다. 찍히면 센서별 obstacle_min_range 를 '센서에서 몸체 외곽까지 거리' 보다 조금 크게 올린다. 아니면 footprint_clearing_enabled 를 true 로 바꾼다 (그러면 footprint 크기가 곧 지우는 범위가 된다). 판단과 측정이 필요한 항목이다. 기계적으로 바꾸는 값이 아니다. |
| check | amhs/mobile_robot/mobile_robot_bringup/config/cartographer_localization.lua:min_range 줄이 없다 (72 는 max_range 만, 35 num_laser_scans = 2). cartographer_mapping.lua:67 도 같다. | 기본값 min_range = 0. (amhs/software/cartographer/cartographer/configuration_files/trajectory_builder_2d.lua:17) | 라이다에서 몸체 외곽까지 거리 (self-hit). 이미 목록에 있는 merger range_min, p2l range_min 과 같은 종류다. 다만 원시 f_scan/b_scan 을 받는 다른 경로다. | 원시 nanoscan scan 에 몸체 점이 있으면 TRAJECTORY_BUILDER_2D.min_range 를 그 거리보다 조금 크게 둔다 (예: 0.05-0.15 m, 실측으로 정한다). 없으면 그대로 둔다. 측정 항목이다. |
| check | amhs/software/docking/marker_auto_docking/config/marker_auto_docking.yaml:13 | distance_targetpose_tag: 0.4 #0.365 (base_frame: base_link, :32) | base_link 에서 앞 외곽까지 거리 (지금 0.32). 몸체와 tag 사이 여유는 0.4 - 0.32 = 0.08 m 다. | 앞쪽 반길이가 0.32 그대로면 바꾸지 않는다. 실측에서 앞이 0.32 보다 길면 (base_link 가 상자 중심이 아니면) 그만큼 여유가 준다. 그때는 도킹 정지 위치를 다시 맞춘다. 판단 항목이다. |
| check | amr-gz-sim-example/isaac/measure/validate/approach_stops.py (+ scan_spot.py, mobile_robot_gz_sim/scripts/verify/fake_fleet_server.py):approach_stops.py:39, 48; scan_spot.py:73, 141; fake_fleet_server.py:63 (주석) | h=0.315, abs(lx) < 0.315 and abs(ly) < 0.315; box_segs(0,0,0,0.63,0.63); 주석 'footprint 0.63x0.63' | 몸체 반폭과 반길이 (sim 측정과 판정 기준) | sim 몸체 (amr_sim_robots/spec/simple.yaml:95-96, 이미 목록에 있다) 를 0.32 로 바꿀 때 같이 0.32 / 0.64 로 바꾼다. 안 바꾸면 몸체 간격 측정이 면마다 5 mm 낙관적으로 나온다. sim 전용이고 현장 코드는 아니다. |

열린 질문:
- 회전 중심: 구동 바퀴 joint 가 x = 0.0 이다 (amhs/mobile_robot/mobile_robot_description/urdf/include/wheel.urdf.xacro:7). 그러니 base_link 는 구동축 중심이고, 차동 구동의 회전 중심이다. 하지만 실측 상자 중심이 구동축에서 얼마나 떨어져 있는지는 URDF 로 알 수 없다. 지금 footprint (앞 0.32 / 뒤 0.315) 는 상자 중심이 축보다 2.5 mm 앞이라는 뜻이다. 실측 때 '구동축에서 앞 끝', '구동축에서 뒤 끝', '중심선에서 좌/우 끝' 을 따로 재야 한다.
- URDF 몸체는 실측 근거로 쓸 수 없다. base_link visual/collision 은 MiR250 메시다 (mobile_robot_v5.urdf.xacro:68-83). 메시 bbox 를 계산하면 x ±0.400, y ±0.290 이고, +0.0376 앞으로 밀려 있다 (x -0.362 ... +0.438).
- 튀어나온 것: v5 URDF 의 D435 카메라 4 대가 x = ±0.35 에 있다 (mobile_robot_v5.urdf.xacro:161, 177, 194, 210; :12 d435_camera default=true, :157 xacro:if). 0.32 보다 3 cm 밖이다. 실제로 달려 있는지, 0.64 실측에 들어갔는지 확인해야 한다. ZED (±0.29, :241-259) 와 라이다 (중심에서 0.308) 는 안쪽이다.
- 위쪽 구조물: 로봇 종류가 AMR_MANIPULATOR_2P_TRAY 다 (nav2_params.yaml:1464). sim CAD 메모에 '상부 상자 6 개 (z ≥ 0.64, 선반·벽·안테나)' (amr_sim_robots/spec/simple.yaml:92) 와 'arm/gripper' (isaac/tools/build_cad_fleet_robot.py:419) 가 있다. 0.64 가 바닥 몸체만 잰 값인지, 팔·선반·트레이가 주행 자세에서 바깥으로 나오는지 확인해야 한다. 나오면 footprint 는 그 외곽 기준으로 잡아야 한다.
- 범퍼: 0.64 가 범퍼를 포함한 값인지 확인해야 한다. 범퍼가 따로 있으면 그것까지 footprint 에 넣어야 한다.
- self-hit: footprint_clearing_enabled 가 전부 false 이고 obstacle_min_range 와 cartographer min_range 가 0 이다. 정지 상태에서 센서가 자기 몸체를 보는지 한 번 측정해야 N1, N2 를 정할 수 있다.
