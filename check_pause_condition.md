Seri, 이 에러의 원인은 \*\*`BT::ConditionNode::halt()`가 v4에서 `final`\*\*로 선언돼 있어서, 파생 클래스에서 오버라이드할 수 없기 때문이에요. 그래서 아래처럼 `void halt() override`를 넣으면 컴파일 타임에 막힙니다. BehaviorTree.CPP v4에선(특히 Nav2도) **컨디션 노드는 `halt()`를 직접 재정의하지 않고**, 다음 틱에 재진입할 때 초기화/리셋을 합니다. 예를 들어 Nav2의 `IsBatteryLowCondition`도 `halt()`를 건드리지 않고 `tick()` 시작에서 `if (!BT::isStatusActive(status())) { … }`로 재초기화해요. ([api.nav2.org][1])

아래처럼 고치면 빌드 에러 없이 의도한 “halt 시 래치 리셋” 효과를 얻을 수 있습니다.

---

## 최소 수정안 (권장)

### 1) `halt()` 제거

헤더에서 이 부분을 **삭제**하세요.

```cpp
// ❌ 지우기
void halt() override
{
  BT::ConditionNode::halt();
  last_flag_.store(false, std::memory_order_relaxed);
}
```

### 2) `tick()`의 재진입(=halt 이후) 가드에서 리셋

Nav2 패턴을 그대로 씁니다. 첫 틱이거나 방금 halt된 후 재진입이면 `status()`가 active가 아니므로 여기서 내부 상태를 초기화합니다.
또, 기존에 넣어둔 `reset` 포트(edge-trigger) 처리는 그대로 유지합니다.

```cpp
BT::NodeStatus CheckPauseCondition::tick()
{
  // ---- 재진입(첫 진입/방금 halt됨) 시 1회 초기화 ----
  if (!BT::isStatusActive(status())) {
    // 필요 시 포트 재읽기 (옵션)
    getInput("flag_topic", flag_topic_);
    getInput("latch", latch_);
    getInput("transient_local", transient_local_);

    // 내부 래치/상태 리셋
    last_flag_.store(false, std::memory_order_relaxed);
    prev_reset_ = false;
  }

  // 콜백 처리
  callback_group_executor_.spin_some();

  // --- 블랙보드 리셋 트리거 (edge-trigger) ---
  bool reset_in = false;
  getInput("reset", reset_in);
  if (reset_in && !prev_reset_) {
    last_flag_.store(false, std::memory_order_relaxed);
  }
  prev_reset_ = reset_in;

  // --- latch 동작 ---
  if (latch_) {
    return last_flag_.load(std::memory_order_relaxed) ? BT::NodeStatus::SUCCESS
                                                      : BT::NodeStatus::FAILURE;
  } else {
    return last_flag_.exchange(false, std::memory_order_acq_rel) ? BT::NodeStatus::SUCCESS
                                                                 : BT::NodeStatus::FAILURE;
  }
}
```

이 방식은 Nav2의 컨디션 노드들이 `halt()` 없이도 **다음 틱에 스스로 초기화**하는 패턴과 동일합니다. ([api.nav2.org][1])

---

## QoS 설정 버그도 하나 있어요

현재 코드:

```cpp
rclcpp::QoS qos(rclcpp::KeepLast(1));
qos.transient_local().reliable();
if (!transient_local_) {
  qos.reliable(); // DDS latched QoS (주석과 실제 코드 불일치)
}
```

여기선 `transient_local_`이 `false`여도 위에서 이미 `transient_local()`을 호출해버려요. 의도대로라면 아래처럼 쓰는 게 맞습니다.

```cpp
rclcpp::QoS qos(rclcpp::KeepLast(1));
qos.reliable();
if (transient_local_) {
  qos.transient_local();  // latched QoS를 원할 때만 적용
}
```

Nav2의 배터리 컨디션 예제는 자체 QoS 래퍼를 쓰지만(`StandardTopicQoS`), 핵심은 **재진입 시 초기화하고 `tick()` 안에서 `spin_all/ some`으로 콜백을 처리**한다는 점이에요. ([api.nav2.org][1])

---

## 만약 “halt 직후 즉시(clean) 리셋”이 꼭 필요하다면?

컨디션 노드는 `halt()`가 final이라 훅을 못답니다. 그런 즉시 클린업이 반드시 필요하면(다음 틱까지 기다리기 싫다면):

* 노드 타입을 `BT::StatefulActionNode` / `BT::ActionNodeBase`로 바꾸고 **`onHalted()`** 훅에서 리셋하세요. (Action류는 `onStart/onRunning/onHalted` 라이프사이클 훅을 제공합니다.) ([behaviortree.dev][2])
* 다만 **컨디션은 원칙상 RUNNING을 반환하지 않는** 게 일반적이므로, 의미상 컨디션을 액션으로 바꿀지 여부는 트리 설계와 맞춰 보세요. Nav2도 대부분 컨디션을 ConditionNode로 유지하고, 재진입 초기화 패턴으로 해결합니다. ([Medium][3])

---

## 참고: Nav2 / BT.CPP 동작 맥락

* Nav2는 액션 종료 시 **트리 전체 `haltTree()` 호출**로 모든 노드의 halt가 불립니다. 그래서 재진입 가드가 있는 노드는 다음 틱에 깔끔히 초기화돼요. 여러분의 노드도 같은 패턴으로 잘 맞습니다. ([docs.nav2.org][4])

---

필요하시면 제가 위 패치 반영한 전체 파일(diff)도 만들어 드릴게요. 지금 바로 적용하실 최소 변경은 **`halt()` 제거 + `tick()`의 재진입 가드에서 `last_flag_` 리셋**입니다.

[1]: https://api.nav2.org/nav2-rolling/html/is__battery__low__condition_8cpp_source.html "Nav2 Navigation Stack - rolling: nav2_behavior_tree/plugins/condition/is_battery_low_condition.cpp Source File"
[2]: https://www.behaviortree.dev/docs/tutorial-basics/tutorial_04_sequence/?utm_source=chatgpt.com "Reactive and Asynchronous behaviors"
[3]: https://medium.com/%40huawei.zhu/behavior-tree-library-8cf31acb3772?utm_source=chatgpt.com "Overview — Behavior tree library"
[4]: https://docs.nav2.org/migration/Iron.html?utm_source=chatgpt.com "Iron to Jazzy"
