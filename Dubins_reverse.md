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
