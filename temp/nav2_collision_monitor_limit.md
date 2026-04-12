로봇의 주행 곡률(Curvature)을 유지하기 위해 Jazzy 최신 로직을 이식하는 가장 깔끔한 방법은 `processStopSlowdownLimit` 함수의 `Limit` 처리 부분을 수정하는 것입니다.


### 수정 코드 (processStopSlowdownLimit 부분)

`collision_monitor_node.cpp` 파일의 해당 섹션을 아래 내용으로 교체하세요.

```cpp
    } else {  // Limit
      // Compute linear velocity
      const double linear_vel = std::hypot(velocity.x, velocity.y);
      Velocity safe_vel;
      double ratio = 1.0;

      // 1. 선속도 제한을 위한 비율 계산
      if (linear_vel != 0.0) {
        ratio = std::min(ratio, polygon->getLinearLimit() / linear_vel);
      }

      // 2. 각속도 제한을 위한 비율 계산 (추가된 부분)
      if (std::abs(velocity.tw) > 1e-6) {
        ratio = std::min(ratio, polygon->getAngularLimit() / std::abs(velocity.tw));
      }

      // 3. 0.0 ~ 1.0 사이로 값 고정
      ratio = std::clamp(ratio, 0.0, 1.0);

      // 4. 모든 속도 성분에 동일한 비율을 곱하여 곡률 유지
      safe_vel = velocity * ratio;

      // Check that currently calculated velocity is safer than
      // chosen for previous shapes one
      if (safe_vel < robot_action.req_vel) {
        robot_action.polygon_name = polygon->getName();
        robot_action.action_type = LIMIT;
        robot_action.req_vel = safe_vel;
        return true;
      }
    }
```

---

### 주요 변경 사항 설명

1.  **독립적 Clamp 제거**: 기존의 `safe_vel.tw = std::clamp(...)` 부분을 삭제했습니다. 이 부분이 비율을 깨뜨리는 주원인이었기 때문입니다.
2.  **통합 Ratio 계산**: `linear_limit`과 `angular_limit` 중 어떤 것이 더 엄격한지(더 작은 비율을 만드는지) `std::min`을 통해 하나의 `ratio` 값을 결정합니다.
3.  **벡터 스케일링**: 결정된 단일 `ratio`를 `velocity * ratio` 연산을 통해 선속도와 각속도에 동시에 적용합니다. 
    * 이 연산은 내부적으로 `safe_vel.x = velocity.x * ratio;`, `safe_vel.tw = velocity.tw * ratio;`를 수행하므로 회전 반경 $R = V/\omega$가 일정하게 유지됩니다.

### 수정 후 기대 효과

이 수정을 거치면 로봇이 회전하는 도중 `Limit` 구역(예: 좁은 복도나 인파 밀집 지역)에 진입하더라도, **로봇이 가려던 원래의 궤적(Arc)을 그대로 따라가면서 전체적인 속도만 부드럽게 줄어들게 됩니다.** 로봇이 갑자기 안쪽으로 파고드는 현상을 완벽히 방지할 수 있습니다.
