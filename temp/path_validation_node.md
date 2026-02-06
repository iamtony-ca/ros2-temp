코드 전반 구조/락 설계는 꽤 탄탄한 편인데, **“바로 크래시/오동작으로 이어질 수 있는 치명 버그 1개”**가 보여요. 그 다음으로는 TF 타임스탬프/파라미터/타이머 그룹 쪽에서 “가끔 안 맞는” 류의 문제 가능성이 있습니다.

---

## 1) 치명 버그: 이웃 셀 `mapToWorld()`에 음수 인덱스가 들어갈 수 있음

`agent_hit_around()`에서 8방향 이웃을 검사할 때:

```cpp
cm->mapToWorld(static_cast<int>(mx_c)+o[0], static_cast<int>(my_c)+o[1], wxx, wyy);
```

문제는 `Costmap2D::mapToWorld()` 시그니처가 보통 **(unsigned int, unsigned int, ...)** 이라서,
예를 들어 `mx_c == 0`이고 `o[0] == -1`이면 `-1`이 **unsigned로 변환되어 매우 큰 값**이 되고, 결과적으로:

* 엉뚱한 좌표가 나오거나
* 내부에서 bounds 가정이 깨져서 런타임 문제(특히 디버그 빌드에서 assert)로 이어질 수 있어요.

### ✅ 안전한 수정 예시 (권장)

이웃 좌표를 `int`로 계산하고, **bounds 체크 후에만** `mapToWorld(unsigned)`로 넘기세요.

```cpp
auto inBounds = [&](int x, int y) {
  return x >= 0 && y >= 0 &&
         x < static_cast<int>(cm->getSizeInCellsX()) &&
         y < static_cast<int>(cm->getSizeInCellsY());
};

for (auto & o : OFFS) {
  int nx = static_cast<int>(mx_c) + o[0];
  int ny = static_cast<int>(my_c) + o[1];
  if (!inBounds(nx, ny)) continue;

  double wxx, wyy;
  cm->mapToWorld(static_cast<unsigned int>(nx),
                 static_cast<unsigned int>(ny),
                 wxx, wyy);

  auto h2 = whoCoversPoint(wxx, wyy);
  if (!h2.empty()) return h2;
}
```

이 버그는 `validateWithPoints()` / `validateWithFootprint()` 둘 다 동일 패턴이 있으니 **두 군데 다** 고쳐야 합니다.

---

## 2) TF 조회 타임스탬프가 “현재 시각 now()”라서 가끔 실패할 수 있음

```cpp
base_in.header.stamp = this->now();
lookupTransform(global_frame_, base_frame_, base_in.header.stamp, 0.2s)
```

이 방식은 TF가 “그 정확한 시간”의 변환을 못 주면 실패합니다(특히 sensor/odom tf가 약간 늦거나, sim time/real time 섞였을 때).

### ✅ 더 안정적인 방식

* 최신 TF를 원하면 `tf2::TimePointZero` (ROS2에서는 `rclcpp::Time(0,0,...)` 쓰기도 함)로 “latest available”를 요청하는 패턴이 더 안정적이에요.
* 또는 path pose stamp와 일관된 시간 사용.

예:

```cpp
auto tf = tf_buffer_->lookupTransform(global_frame_, base_frame_,
                                      tf2::TimePointZero,
                                      rclcpp::Duration::from_seconds(0.2));
```

(프로젝트 컨벤션에 따라 “latest” 사용이 괜찮은지 결정하면 됩니다.)

---

## 3) `triggerReplan()` 안의 flag reset timer가 콜백그룹 밖에서 돌아감 (동시성/순서 이슈 가능)

여기:

```cpp
flag_reset_timer_ = this->create_wall_timer(
  std::chrono::milliseconds(flag_pulse_ms_),
  [weak_pub]() { ... });
```

다른 타이머는 `timer_callback_group_`로 넣었는데, 이 타이머는 **기본 콜백 그룹**으로 들어가요. 보통 큰 문제는 없지만, executor 구성/콜백 그룹 정책에 따라:

* reset 타이머가 기대한 스레드/순서로 실행되지 않거나
* “replan true → false pulse” 타이밍이 들쭉날쭉해질 수 있습니다.

### ✅ 권장

여기도 동일하게 callback group 지정:

```cpp
flag_reset_timer_ = this->create_wall_timer(
  std::chrono::milliseconds(flag_pulse_ms_),
  [weak_pub]() { ... },
  timer_callback_group_);
```

---

## 4) 에이전트 footprint 로딩: radius=0이면 퇴화 footprint 가능

YAML에서 footprint가 파싱 실패하면 `use_radius=true`로 가는데, `robot_radius` 기본값이 0.0이라면:

```cpp
makeFootprintFromRadius(data.radius); // radius=0.0 가능
```

이 경우 footprint가 의미 없는 형태가 될 수 있고, 결과적으로 에이전트 충돌 판정이 “안 되거나 이상하게” 됩니다.

### ✅ 권장 안전장치

* radius 최소값 clamp
* footprint 파싱 실패 시 경고 + skip(또는 기본 반지름 적용)

예:

```cpp
if (data.use_radius) {
  if (data.radius <= 1e-3) {
    RCLCPP_WARN_ONCE(..., "radius too small ...");
    return {};
  }
}
```

---

## 5) `updateObstacleDatabase()`의 ROI worldToMap 실패 처리

```cpp
if (!worldToMap(min_x,min_y) || !worldToMap(max_x,max_y)) return;
```

map 경계에 걸리면 한쪽이 실패하면서 DB 업데이트가 통째로 스킵될 수 있어요.
지금은 로봇이 맵 경계 근처에 가면 장애물 DB가 “갱신이 끊겨서” 성숙도 로직이 흔들릴 수 있습니다.

### ✅ 권장

여기는 `worldToMapEnforceBounds()` 계열로 clamp해서 계속 돌리는 게 안정적입니다.

---

## 6) 성능/락 관점 (버그는 아니지만 개선 포인트)

* `validateWithPoints()`에서 매 포즈마다:

  * `worldToMap` 위해 costmap lock
  * `isBlockedCellKernel()`에서 또 lock
    → 실시간 빈도가 높으면 lock contention이 커질 수 있어요.

이미 DB 업데이트에서 `costmap_`를 snapshot으로 잡는 패턴을 쓰셨으니, path validate에서도 비슷하게:

* 콜백 시작 시 `std::shared_ptr<Costmap2D> cm` 스냅샷 확보
* 루프 안에서는 락 없이 `cm->...` 사용

으로 바꾸면 latency가 안정됩니다. (지금도 “동작”은 하겠지만, 부하가 늘면 타이밍 흔들림 가능)

---

# 결론 (우선순위)

1. **[필수] 이웃 셀 `mapToWorld()` 음수 인덱스 버그 수정** ← 이건 실제로 터질 수 있습니다.
2. **TF lookup을 latest로**(또는 stamp 전략 재정의)해서 간헐 실패 줄이기
3. `flag_reset_timer_`도 callback group 지정
4. agent radius=0 보호 로직 추가
5. ROI worldToMap 실패 시 전체 스킵 방지

원하면, 제가 **(1) 이웃 셀 버그를 포함해서** `agent_hit_around()` 두 군데를 안전하게 리팩토링한 패치 형태로(최소 수정 diff 스타일) 딱 만들어 드릴게요.
