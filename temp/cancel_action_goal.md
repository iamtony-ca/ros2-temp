네, handle 참조를 잃어도 **서버 쪽에는 goal이 살아있으니** 외부에서 취소할 방법이 있습니다. 다만 "우아한 방법"과 "확실한 방법"이 나뉩니다.

## 1. CLI로 cancel — 가장 간단

`navigate_through_poses` 서버는 여전히 그 goal을 EXECUTING으로 들고 있으므로, 같은 action 이름으로 cancel을 걸 수 있습니다. rclpy/ros2cli에는 진행 중 goal을 취소하는 방법이 몇 가지 있습니다.

가장 확실한 건 nav2가 제공하는 **`lifecycle`/BT 중단 계열**보다, 그냥 그 노드를 다루는 겁니다. 다만 `ros2 action` CLI는 사실 send_goal 위주라, **임의의 진행 중 goal을 CLI 한 줄로 cancel하는 공식 서브커맨드는 없습니다.** (`ros2 action send_goal`은 새 goal을 보내는 것이고, cancel 전용 CLI 커맨드는 jazzy 기준 제공되지 않습니다.) 이 점은 정확히 짚어둡니다 — "CLI 한 줄 cancel"은 기대만큼 깔끔하지 않습니다.

대신 실무적으로 쓰는 방법들:

## 2. 새 goal을 보내서 밀어내기 (당신이 이미 겪은 그 동작)

로그에서 봤던 바로 그 메커니즘입니다. `bt_navigator`는 단일 goal 서버라, 새 goal을 보내면 진행 중 goal이 자동으로 ABORTED로 밀려납니다.

```bash
ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses \
  "{poses: [{header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}}}]}"
```

즉 "현재 위치 근처로 가는 짧은 goal"을 보내면 옛 goal이 밀려나 멈춥니다. 다만 이건 로봇을 아주 잠깐 새 goal로 움직이게 하므로, 안전 정지 목적으론 부적합합니다.

## 3. BT 자체를 멈추기 — nav2의 정공법

nav2에는 진행 중 네비게이션을 멈추는 표준 경로가 있습니다.

- **`ros2 lifecycle set /bt_navigator deactivate`** — bt_navigator를 비활성화하면 진행 중 BT가 중단됩니다. 다만 lifecycle 전이라 무겁고, 다시 activate 해야 다음 주행이 됩니다.

```bash
ros2 lifecycle set /bt_navigator deactivate
```

이건 확실히 멈추지만, 노드 상태를 바꾸는 거라 운영 중 상시 쓰기엔 부담이 있습니다.

## 4. controller 출력을 직접 끊기 — 가장 확실한 "멈춤"

goal cancel과 별개로, **로봇을 물리적으로 멈추는** 가장 확실한 방법은 controller가 내보내는 `cmd_vel`을 차단하는 겁니다. nav2는 이를 위해 **`nav2_collision_monitor`나 velocity smoother 앞단의 정지 신호**를 쓰지만, 가장 단순하게는 zero twist를 덮어쓰는 방법이 있습니다:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

단, 이건 controller가 다음 주기에 다시 자기 `cmd_vel`을 내보내므로 **일회성으론 안 멈춥니다.** 지속 발행(`--rate`)하거나, twist_mux 같은 우선순위 먹스에서 상위 채널로 덮어써야 실효가 있습니다. 당신 시스템에 이미 `twist_mux`가 있다면 거기에 emergency stop 채널을 두는 게 정석입니다.

## 근본 해법: handle을 잃지 않도록 노드 구조를 고치기

위 방법들은 전부 "이미 handle을 잃은 뒤의 응급 대응"이라 깔끔하지 않습니다. 실제로 당신이 필요한 건 **handle을 애초에 잃지 않는 것**입니다. 로그의 문제는 `_move_response_callback`이 새 handle로 옛 handle을 덮으면서 참조를 잃은 데서 왔죠.

앞서 (a) 방식이 이걸 정확히 막습니다 — 겹친 move가 들어오면 **새 goal을 보내기 전에**, 아직 살아있는 `_goal_handle`로 `cancel_goal_async()`를 호출하니 참조를 잃을 일이 없습니다. 즉 "handle 잃은 뒤 어떻게 취소하나"를 고민할 상황 자체를 없애는 게 맞는 방향입니다.

만약 이미 잃어버린 상황을 코드 안에서 방어적으로 복구하고 싶다면, **handle을 멤버에 하나 더 백업**해두는 패턴이 있습니다:

```cpp
// 개념 예시 (C++ 스타일 기준)
// _goal_handle 을 덮어쓰기 전에, 이전 것을 별도로 보관해 cancel
rclcpp_action::ClientGoalHandle<NavigateThroughPoses>::SharedPtr prev = goal_handle_;
if (prev) {
    nav_client_->async_cancel_goal(prev);  // 덮어쓰기 전에 취소
}
goal_handle_ = new_handle;
```

Python이라면:

```python
        with self._state_lock:
            prev = self._goal_handle          # 덮어쓰기 전 백업
            self._goal_handle = goal_handle
        if prev is not None and prev is not goal_handle:
            prev.cancel_goal_async()          # 옛 handle 취소
```

핵심은 **덮어쓰기 직전에 이전 handle로 cancel을 거는 것**입니다. 이러면 참조를 잃기 전에 취소하니, 나중에 CLI로 응급 대응할 필요가 없습니다.

## 정리

- handle을 잃어도 서버 goal은 살아있으므로 외부 취소가 가능은 합니다: 새 goal로 밀어내기(2), `bt_navigator` lifecycle deactivate(3), `cmd_vel` 차단(4).
- 다만 **cancel 전용 CLI 한 줄은 jazzy에 없습니다** — 이 점은 솔직하게 말씀드립니다. 위 우회들이 현실적 수단입니다.
- 가장 확실한 물리적 정지는 `cmd_vel` 경로 차단(twist_mux 등)이고, lifecycle deactivate는 네비게이션 자체를 멈춥니다.
- **근본 해법은 handle을 잃지 않는 것** — 덮어쓰기 직전 cancel 패턴이나 (a) 방식의 겹침 처리로 애초에 참조 유실을 막는 게 정석입니다.

참고 문서: nav2의 lifecycle 관리는 [Nav2 lifecycle 문서](https://docs.nav2.org/concepts/index.html), bt_navigator 동작은 [configuring-bt-navigator](https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html), rclpy action cancel API는 [rclpy Actions 튜토리얼](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)을 참고하세요.

한 가지 확인 — 지금 필요한 게 "이미 잃어버린 handle의 응급 취소"인가요, 아니면 "앞으로 handle을 안 잃도록 하는 구조 수정"인가요? 후자라면 덮어쓰기 직전 cancel 패턴을 현재 `_move_response_callback`에 맞춰 정확히 짜드릴 수 있습니다.
