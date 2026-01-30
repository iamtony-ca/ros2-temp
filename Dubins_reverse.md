`nav2_smac_planner/src/node_hybrid.cpp` 파일이 핵심 파일이 맞습니다. 이 파일 안에 있는 `HybridMotionTable::initDubin` 함수가 로봇의 움직임(Motion Primitives)을 정의합니다.

기본적으로 Dubins 모델은 전진(Forward)만 허용하지만, 이를 수정하여 **후진(Reverse) 전용 Dubins 모델**로 변경하려면, 생성되는 경로 조각(primitive)의 x좌표를 반전시키고 방향(Direction) 타입을 변경해야 합니다.

수정이 필요한 부분은 **`HybridMotionTable::initDubin` 함수 내부**입니다.

### 수정 코드 가이드

`nav2_smac_planner/src/node_hybrid.cpp` 파일에서 약 **113번째 줄** 근처의 `initDubin` 함수 내용을 아래와 같이 수정하세요.

#### [수정 전 (Original)]

```cpp
  // ... (앞부분 생략)
  const float delta_dist = hypotf(delta_x, delta_y);

  projections.clear();
  projections.reserve(3);
  // 기존: 전진, 전진+좌회전, 전진+우회전
  projections.emplace_back(delta_dist, 0.0, 0.0, TurnDirection::FORWARD);  // Forward
  projections.emplace_back(delta_x, delta_y, increments, TurnDirection::LEFT);  // Left
  projections.emplace_back(delta_x, -delta_y, -increments, TurnDirection::RIGHT);  // Right

  if (search_info.allow_primitive_interpolation && increments > 1.0f) {
    // Create primitives that are +/- N to fill in search space to use all set angular quantizations
    // Allows us to create N many primitives so that each search iteration can expand into any angle
    // bin possible with the minimum turning radius constraint, not just the most extreme turns.
    projections.reserve(3 + (2 * (increments - 1)));
    for (unsigned int i = 1; i < static_cast<unsigned int>(increments); i++) {
      const float angle_n = static_cast<float>(i) * bin_size;
      const float turning_rad_n = delta_dist / (2.0f * sin(angle_n / 2.0f));
      const float delta_x_n = turning_rad_n * sin(angle_n);
      const float delta_y_n = turning_rad_n - (turning_rad_n * cos(angle_n));
      projections.emplace_back(
        delta_x_n, delta_y_n, static_cast<float>(i), TurnDirection::LEFT);  // Left
      projections.emplace_back(
        delta_x_n, -delta_y_n, -static_cast<float>(i), TurnDirection::RIGHT);  // Right
    }
  }
  // ... (뒷부분 생략)

```

#### [수정 후 (Modified for Reverse Only)]

아래 코드로 해당 블록을 교체하시면 됩니다. `delta_x`에 마이너스를 붙여 뒤로 가게 하고, `TurnDirection`을 `REV` 계열로 변경했습니다.

```cpp
  // ... (앞부분 생략)
  const float delta_dist = hypotf(delta_x, delta_y);

  projections.clear();
  projections.reserve(3);
  
  // [수정됨] 후진 전용 경로 생성
  // 1. 후진 직진 (Reverse Straight)
  projections.emplace_back(-delta_dist, 0.0, 0.0, TurnDirection::REVERSE); 
  
  // 2. 후진 좌회전 (Reverse Left)
  // Reeds-Shepp 로직 참조: x는 반전(-), y는 그대로, 각도는 감소(-)
  projections.emplace_back(-delta_x, delta_y, -increments, TurnDirection::REV_LEFT); 
  
  // 3. 후진 우회전 (Reverse Right)
  // Reeds-Shepp 로직 참조: x는 반전(-), y는 반전(-), 각도는 증가(+)
  projections.emplace_back(-delta_x, -delta_y, increments, TurnDirection::REV_RIGHT);

  if (search_info.allow_primitive_interpolation && increments > 1.0f) {
    projections.reserve(3 + (2 * (increments - 1)));
    for (unsigned int i = 1; i < static_cast<unsigned int>(increments); i++) {
      const float angle_n = static_cast<float>(i) * bin_size;
      const float turning_rad_n = delta_dist / (2.0f * sin(angle_n / 2.0f));
      const float delta_x_n = turning_rad_n * sin(angle_n);
      const float delta_y_n = turning_rad_n - (turning_rad_n * cos(angle_n));
      
      // [수정됨] 보간된 경로도 후진으로 변경
      projections.emplace_back(
        -delta_x_n, delta_y_n, -static_cast<float>(i), TurnDirection::REV_LEFT);  // Rev Left
      projections.emplace_back(
        -delta_x_n, -delta_y_n, static_cast<float>(i), TurnDirection::REV_RIGHT);  // Rev Right
    }
  }
  // ... (뒷부분 생략)

```

### 변경 사항 설명

1. **X 좌표 반전 (`delta_x` -> `-delta_x`)**: 로봇 기준 로컬 좌표계에서 X가 양수면 전진, 음수면 후진입니다. 모든 primitive의 X값을 음수로 변경했습니다.
2. **TurnDirection 변경**:
* `FORWARD` -> `REVERSE`
* `LEFT` -> `REV_LEFT`
* `RIGHT` -> `REV_RIGHT`
* 이 변경은 Cost 함수(`getTraversalCost`)에서 `reverse_penalty`(후진 페널티)가 올바르게 적용되도록 보장합니다.


3. **회전 각도 부호 조정**: 후진 시 핸들을 왼쪽으로 꺾으면(REV_LEFT), 차체는 뒤로 가면서 방향(Heading)은 시계 방향(음수)으로 회전합니다. 따라서 `REV_LEFT`에는 `-increments`, `REV_RIGHT`에는 `+increments`를 적용했습니다. (이는 같은 파일 내 `initReedsShepp` 함수의 로직과 동일하게 맞춘 것입니다.)

이 코드를 적용하고 다시 빌드하시면, `Motion Model` 파라미터를 `DUBIN`으로 설정했을 때 로봇이 **후진 경로만 생성**하게 됩니다.
#################  
#####################  
########################  
정확한 지적입니다. **"path가 비효율적이고 이상하며, 전진과 후진이 섞이는 현상"**이 발생하는 이유는 제가 앞서 드린 코드 수정이 **'움직임(Primitives)'**만 후진으로 바꿨을 뿐, 로봇의 **'두뇌(Heuristic 및 Analytic Expansion)'**는 여전히 **전진 전용 Dubins 모델**을 사용하고 있기 때문입니다.

이로 인해 두 가지 심각한 충돌이 발생합니다.

1. **비효율적인 경로 (Heuristic 문제):**
* **상황:** 로봇이 뒤로 1m만 가면 목표인데, `DubinsStateSpace`(전진 모델)는 "전진으로 한 바퀴 뱅글 돌아서 뒤로 와야 해"라고 계산하여 거리를 20m로 예측합니다.
* **결과:** A* 알고리즘은 바로 뒤로 가는 쉬운 길을 놔두고, Heuristic이 예측한 값과 비슷한 경로를 찾으려다 보니 이상하게 꼬인 경로를 생성합니다. (Heuristic이 실제 비용보다 높게 예측하면 A*는 최적성을 잃습니다.)


2. **전진/후진 섞임 (Analytic Expansion 문제):**
* **상황:** 탐색 마지막 단계에서 `Analytic Expansion`(도착지점으로 바로 연결하는 곡선 계산)이 실행됩니다. 이때 여전히 전진 Dubins 모델을 사용하므로, 마지막 구간에서 갑자기 **전진** 경로를 생성해 붙여버립니다.



---

### 해결 방법

이 문제를 해결하기 위해서는 `initDubin` 함수 내에서 **State Space(경로 계산 모델)**를 변경해야 합니다. OMPL에는 "후진 전용 Dubins"가 없으므로, 전/후진을 모두 고려하여 **최단 거리**를 계산해주는 **`ReedsSheppStateSpace`**로 교체해야 합니다.

**`nav2_smac_planner/src/node_hybrid.cpp`** 파일의 `initDubin` 함수를 아래와 같이 수정하십시오.

#### 수정할 파일: `nav2_smac_planner/src/node_hybrid.cpp`

**`initDubin` 함수 내부 (약 150번째 줄 근처)**

```cpp
// [기존 코드]
// state_space = std::make_shared<ompl::base::DubinsStateSpace>(min_turning_radius);

// [수정 코드]
// DubinsStateSpace 대신 ReedsSheppStateSpace를 사용하여 
// Heuristic 계산 시 후진 가능성을 열어줍니다. 
// 이렇게 해야 Heuristic이 "뒤로 가면 가깝다"는 것을 인지하여 경로가 꼬이지 않습니다.
state_space = std::make_shared<ompl::base::ReedsSheppStateSpace>(min_turning_radius);

```

### 최종 수정된 `initDubin` 함수 전체 코드

아래 코드를 복사하여 `initDubin` 함수 전체를 덮어쓰시면 됩니다. (Primitives 수정과 StateSpace 수정이 모두 포함되어 있습니다.)

```cpp
void HybridMotionTable::initDubin(
  unsigned int & size_x_in,
  unsigned int & /*size_y_in*/,
  unsigned int & num_angle_quantization_in,
  SearchInfo & search_info)
{
  size_x = size_x_in;
  change_penalty = search_info.change_penalty;
  non_straight_penalty = search_info.non_straight_penalty;
  cost_penalty = search_info.cost_penalty;
  reverse_penalty = search_info.reverse_penalty;
  travel_distance_reward = 1.0f - search_info.retrospective_penalty;
  downsample_obstacle_heuristic = search_info.downsample_obstacle_heuristic;
  use_quadratic_cost_penalty = search_info.use_quadratic_cost_penalty;

  if (num_angle_quantization_in == num_angle_quantization &&
    min_turning_radius == search_info.minimum_turning_radius &&
    motion_model == MotionModel::DUBIN)
  {
    return;
  }

  num_angle_quantization = num_angle_quantization_in;
  num_angle_quantization_float = static_cast<float>(num_angle_quantization);
  min_turning_radius = search_info.minimum_turning_radius;
  motion_model = MotionModel::DUBIN;

  float angle = 2.0 * asin(sqrt(2.0) / (2 * min_turning_radius));
  bin_size =
    2.0f * static_cast<float>(M_PI) / static_cast<float>(num_angle_quantization);
  float increments;
  if (angle < bin_size) {
    increments = 1.0f;
  } else {
    increments = ceil(angle / bin_size);
  }
  angle = increments * bin_size;

  const float delta_x = min_turning_radius * sin(angle);
  const float delta_y = min_turning_radius - (min_turning_radius * cos(angle));
  const float delta_dist = hypotf(delta_x, delta_y);

  projections.clear();
  projections.reserve(3);
  
  // [수정 1] Primitives를 후진(Reverse) 전용으로 변경
  // 전진(Forward) -> 후진(Reverse)
  projections.emplace_back(-delta_dist, 0.0, 0.0, TurnDirection::REVERSE); 
  // 좌회전(Left) -> 후진 좌회전(Rev Left): x반전, 각도 반전
  projections.emplace_back(-delta_x, delta_y, -increments, TurnDirection::REV_LEFT); 
  // 우회전(Right) -> 후진 우회전(Rev Right): x반전, y반전, 각도 유지(증가)
  projections.emplace_back(-delta_x, -delta_y, increments, TurnDirection::REV_RIGHT);

  if (search_info.allow_primitive_interpolation && increments > 1.0f) {
    projections.reserve(3 + (2 * (increments - 1)));
    for (unsigned int i = 1; i < static_cast<unsigned int>(increments); i++) {
      const float angle_n = static_cast<float>(i) * bin_size;
      const float turning_rad_n = delta_dist / (2.0f * sin(angle_n / 2.0f));
      const float delta_x_n = turning_rad_n * sin(angle_n);
      const float delta_y_n = turning_rad_n - (turning_rad_n * cos(angle_n));
      
      // [수정 2] 보간(Interpolation) 경로도 후진으로 변경
      projections.emplace_back(
        -delta_x_n, delta_y_n, -static_cast<float>(i), TurnDirection::REV_LEFT);
      projections.emplace_back(
        -delta_x_n, -delta_y_n, static_cast<float>(i), TurnDirection::REV_RIGHT);
    }
  }

  // [수정 3] 핵심: State Space를 ReedsShepp으로 변경
  // DubinsStateSpace(전진 전용)를 사용하면 Heuristic이 엉망이 되어 경로가 꼬입니다.
  // ReedsSheppStateSpace는 후진을 포함한 최단 거리를 계산하므로 Heuristic이 정확해집니다.
  state_space = std::make_shared<ompl::base::ReedsSheppStateSpace>(min_turning_radius);

  // Precompute projection deltas (기존 코드 유지)
  delta_xs.resize(projections.size());
  delta_ys.resize(projections.size());
  trig_values.resize(num_angle_quantization);

  for (unsigned int i = 0; i != projections.size(); i++) {
    delta_xs[i].resize(num_angle_quantization);
    delta_ys[i].resize(num_angle_quantization);

    for (unsigned int j = 0; j != num_angle_quantization; j++) {
      double cos_theta = cos(bin_size * j);
      double sin_theta = sin(bin_size * j);
      if (i == 0) {
        trig_values[j] = {cos_theta, sin_theta};
      }
      delta_xs[i][j] = projections[i]._x * cos_theta - projections[i]._y * sin_theta;
      delta_ys[i][j] = projections[i]._x * sin_theta + projections[i]._y * cos_theta;
    }
  }

  // Travel costs 계산 (기존 코드 유지)
  travel_costs.resize(projections.size());
  for (unsigned int i = 0; i != projections.size(); i++) {
    const TurnDirection turn_dir = projections[i]._turn_dir;
    if (turn_dir != TurnDirection::FORWARD && turn_dir != TurnDirection::REVERSE) {
      const float angle = projections[i]._theta * bin_size;
      const float turning_rad = delta_dist / (2.0f * sin(angle / 2.0f));
      travel_costs[i] = turning_rad * angle;
    } else {
      travel_costs[i] = delta_dist;
    }
  }
}

```

### ⚠️ 필수 추가 설정: Analytic Expansion 끄기

위의 코드 수정을 해도, `ReedsSheppStateSpace`는 **전진**도 허용하기 때문에, 경로의 마지막 부분에서 Analytic Expansion이 "전진으로 들어가는 게 더 빠르네?"라고 판단하면 **전진 경로를 슬쩍 끼워 넣을 수 있습니다.**

완벽하게 **후진만** 하는 경로를 얻으려면, **파라미터 파일(`nav2_params.yaml`)**에서 Analytic Expansion 기능을 꺼야 합니다.

`nav2_params.yaml` 파일에서 `SmacPlannerHybrid` 설정을 찾아 아래 파라미터를 수정하세요.

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner/SmacPlannerHybrid"
      motion_model_for_search: "DUBIN"  # 수정된 코드를 타도록 DUBIN 유지
      # ... (다른 설정들)
      
      # [중요] Analytic Expansion을 비활성화해야 전진 경로가 섞이지 않습니다.
      analytic_expansion_ratio: 0.0     # 0.0으로 설정하거나 매우 큰 값으로 설정
      analytic_expansion_max_length: 0.0 # 0.0으로 설정하여 기능 끄기

```

**정리:**

1. **코드 수정 (`initDubin`)**: Primitives는 후진으로, `state_space`는 `ReedsShepp`으로 변경 (Heuristic 정상화).
2. **파라미터 수정 (`yaml`)**: `analytic_expansion` 관련 파라미터를 0으로 설정하여 전진 경로 생성 차단.

이렇게 하면 **효율적인 후진 전용 경로**만 생성됩니다.
