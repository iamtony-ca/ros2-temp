좋아, blackboard 변수 1개만 받아서 즉시 초기화(level-triggered reset)하도록 코드를 전면 정리했어요.
reset 포트가 true이면 latch 여부와 무관하게 즉시 latched 상태를 해제합니다. (다음 tick부터 정상 평가)

유지: latch(논리적 래치), transient_local(DDS 래치), halt() 시 강제 초기화

추가: reset(bool, 기본 false) — 블랙보드에서 받는 단 하나의 변수



---
```cpp
check_flag_condition.hpp

#pragma once

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <atomic>

namespace amr_bt_nodes
{

class CheckFlagCondition : public BT::ConditionNode
{
public:
  CheckFlagCondition(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      // ROS 2 node 핸들
      BT::InputPort<rclcpp::Node::SharedPtr>("node", "Shared ROS2 node"),

      // 구독할 flag 토픽
      BT::InputPort<std::string>("flag_topic", "/mission_flag", "Topic name to subscribe for flag"),

      // 논리적 래치: true가 한번 들어오면 false가 들어오기 전까지 SUCCESS 유지
      BT::InputPort<bool>("latch", true, "If true, SUCCESS state is latched until flag becomes false"),

      // DDS 래치(transient_local) 사용 여부
      BT::InputPort<bool>("transient_local", false, "Use DDS transient_local (latched QoS)"),

      // ★ 블랙보드 리셋 트리거(단 하나의 변수): true면 즉시 latched 상태 초기화
      BT::InputPort<bool>("reset", false, "If true, reset internal latched state immediately")
    };
  }

  BT::NodeStatus tick() override;

  // 트리/서브트리 정지 시 강제 초기화(취소/프리엠프/중단 등)
  void halt() override
  {
    BT::ConditionNode::halt();
    last_flag_.store(false, std::memory_order_relaxed);
  }

private:
  void flagCallback(const std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flag_sub_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  std::string flag_topic_;
  bool latch_{true};
  bool transient_local_{false};

  std::atomic<bool> last_flag_{false}; // 최신 flag 상태 (thread-safe)
};

} // namespace amr_bt_nodes

```
---

```cpp
check_flag_condition.cpp

#include "amr_bt_nodes/check_flag_condition.hpp"

namespace amr_bt_nodes
{

CheckFlagCondition::CheckFlagCondition(const std::string& name,
                                       const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config)
{
  // 필수 입력: node
  if (!getInput("node", node_)) {
    throw BT::RuntimeError("[CheckFlagCondition] Missing required input [node]");
  }

  // 선택 입력들
  getInput("flag_topic", flag_topic_);
  getInput("latch", latch_);
  getInput("transient_local", transient_local_);

  // 콜백 그룹 & 전용 executor
  callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  // 구독 옵션 및 QoS
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.reliable();
  if (transient_local_) {
    qos.transient_local(); // DDS latched QoS
  }

  // flag 구독
  flag_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      flag_topic_, qos,
      std::bind(&CheckFlagCondition::flagCallback, this, std::placeholders::_1),
      sub_options);

  // 초기 latched 샘플 처리
  callback_group_executor_.spin_some();

  RCLCPP_INFO(node_->get_logger(),
              "[CheckFlagCondition] Subscribed to topic: %s", flag_topic_.c_str());
}

void CheckFlagCondition::flagCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  last_flag_.store(msg->data, std::memory_order_relaxed);
  RCLCPP_DEBUG(node_->get_logger(), "[CheckFlagCondition] flag=%s",
               msg->data ? "true" : "false");
}

BT::NodeStatus CheckFlagCondition::tick()
{
  // 콜백 처리
  callback_group_executor_.spin_some();

  // --- 블랙보드 리셋 트리거 처리 (level-triggered) ---
  bool do_reset = false;
  if (getInput("reset", do_reset) && do_reset) {
    // latch 여부와 무관하게 내부 latched 상태를 즉시 초기화
    last_flag_.store(false, std::memory_order_relaxed);
  }

  // --- 논리적 latch 동작 ---
  if (latch_) {
    // true가 유지되는 동안 SUCCESS
    if (last_flag_.load(std::memory_order_relaxed)) {
      return BT::NodeStatus::SUCCESS;
    }
  } else {
    // 1회성 이벤트 소비: true를 한번 보고 바로 false로 소모
    if (last_flag_.exchange(false, std::memory_order_acq_rel)) {
      return BT::NodeStatus::SUCCESS;
    }
  }

  return BT::NodeStatus::FAILURE;
}

} // namespace amr_bt_nodes
```

---

```cpp
플러그인 등록

#include "behaviortree_cpp/bt_factory.h"
#include "amr_bt_nodes/check_flag_condition.hpp"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::CheckFlagCondition>("CheckFlagCondition");
}
```

---

BT XML 사용 예 (reset = 블랙보드 변수 1개)
```xml
<Sequence>
  <!-- 최초 체크 (reset 포트 연결) -->
  <CheckFlagCondition
      node="{node}"
      flag_topic="/controller_pause_flag"
      latch="true"
      transient_local="false"
      reset="{controller_pause_reset}" />

  <LogTextAction message="[PAUSED] Controller is paused. Halting motion." interval_s="1.0"/>

  <SequenceWithMemory>
    <CancelControl/>

    <!-- 취소 직후, latched 상태 즉시 해제 -->
    <Script code="controller_pause_reset := true" />

    <Repeat num_cycles="-1">
      <Sequence>
        <CheckFlagCondition
            node="{node}"
            flag_topic="/controller_pause_flag"
            latch="true"
            transient_local="false"
            reset="{controller_pause_reset}" />
        <Wait wait_duration="0.1" />
      </Sequence>
    </Repeat>
  </SequenceWithMemory>
</Sequence>
```
controller_pause_reset(블랙보드 단 하나의 변수)을 true로 올리면, 해당 tick에서 즉시 초기화됩니다.

이후 필요 시 <Script code="controller_pause_reset := false"/>로 내리면 됩니다.

추가로, Nav2가 트리를 멈추는 상황(취소/프리엠프/중단)에서는 halt()로도 한 번 더 안전하게 초기화됩니다.


필요하면 reset의 기본값을 false로 두었으므로, 블랙보드에 변수가 없을 때도 안전하게 동작합니다.


###################



좋아! 요청하신 대로 클래스를 **`CheckPauseCustomCondition`** 로 바꾸고, **`reset_seq`(int, 에지-트리거 리셋)** 로직과 **prime 처리**, **halt 시 강제 초기화**까지 포함한 “완성 코드”를 제공합니다. 그대로 붙여 넣어 빌드하면 됩니다.

---

### `check_pause_custom_condition.hpp`

```cpp
#pragma once

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <atomic>
#include <string>

namespace amr_bt_nodes
{

class CheckPauseCustomCondition : public BT::ConditionNode
{
public:
  CheckPauseCustomCondition(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      // ROS 2 node 핸들 (설명 문자열만; 디폴트 값 넣지 말 것)
      BT::InputPort<rclcpp::Node::SharedPtr>("node", "Shared ROS2 node"),

      // 구독할 flag 토픽
      BT::InputPort<std::string>("flag_topic", "/mission_flag",
                                 "Topic name to subscribe for flag (std_msgs/Bool)"),

      // 논리적 래치: true가 들어온 뒤 false가 들어오기 전까지 SUCCESS 유지
      BT::InputPort<bool>("latch", true,
                          "If true, SUCCESS stays latched while flag remains true"),

      // DDS 래치(transient_local) 사용 여부
      BT::InputPort<bool>("transient_local", false,
                          "Use DDS transient_local (latched QoS)"),

      // ★ 블랙보드 단일 변수(정수). 값이 '변할 때'만 1회 리셋 (에지-트리거)
      BT::InputPort<int>("reset_seq", 0,
                         "Reset latched state when this integer value changes")
    };
  }

  BT::NodeStatus tick() override;

  // 트리/서브트리 정지 시(취소/프리엠프/중단 등) 강제 초기화
  void halt() override
  {
    BT::ConditionNode::halt();
    last_flag_.store(false, std::memory_order_relaxed);
    // primed_는 유지(다음 tick에서 동일 reset_seq라면 추가 리셋 없음)
  }

private:
  void flagCallback(const std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flag_sub_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  std::string flag_topic_;
  bool latch_{true};
  bool transient_local_{false};

  std::atomic<bool> last_flag_{false};  // 최신 flag 상태 (thread-safe)

  // reset_seq 에지 감지용
  int  last_seen_reset_seq_{0};
  bool primed_{false};                  // 최초 1회 prime: 현재 값으로 동기화만 하고 리셋은 안 함
};

}  // namespace amr_bt_nodes
```

---

### `check_pause_custom_condition.cpp`

```cpp
#include "amr_bt_nodes/check_pause_custom_condition.hpp"

namespace amr_bt_nodes
{

CheckPauseCustomCondition::CheckPauseCustomCondition(const std::string& name,
                                                     const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config)
{
  // 필수 입력: node
  if (!getInput("node", node_)) {
    throw BT::RuntimeError("[CheckPauseCustomCondition] Missing required input [node]");
  }

  // 선택 입력들
  getInput("flag_topic", flag_topic_);
  getInput("latch", latch_);
  getInput("transient_local", transient_local_);

  // 콜백 그룹 & 전용 executor
  callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  // 구독 옵션 및 QoS
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.reliable();
  if (transient_local_) {
    qos.transient_local(); // DDS latched QoS
  }

  // flag 구독
  flag_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      flag_topic_, qos,
      std::bind(&CheckPauseCustomCondition::flagCallback, this, std::placeholders::_1),
      sub_options);

  // 초기 콜백 처리(필요 시 latched 샘플 즉시 수신)
  callback_group_executor_.spin_some();

  RCLCPP_INFO(node_->get_logger(),
              "[CheckPauseCustomCondition] Subscribed to topic: %s", flag_topic_.c_str());
}

void CheckPauseCustomCondition::flagCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  // 단순 상태 플래그이므로 relaxed면 충분
  last_flag_.store(msg->data, std::memory_order_relaxed);
  RCLCPP_DEBUG(node_->get_logger(), "[CheckPauseCustomCondition] flag=%s",
               msg->data ? "true" : "false");
}

BT::NodeStatus CheckPauseCustomCondition::tick()
{
  // 콜백 처리
  callback_group_executor_.spin_some();

  // --- reset_seq 에지-트리거 처리 ---
  int seq = 0;
  if (getInput("reset_seq", seq)) {
    if (!primed_) {
      // 최초 1회: 현재 값으로 동기화만 하고 리셋은 하지 않음
      last_seen_reset_seq_ = seq;
      primed_ = true;
    } else if (seq != last_seen_reset_seq_) {
      // 값이 바뀐 순간에만 1회 리셋
      last_seen_reset_seq_ = seq;
      last_flag_.store(false, std::memory_order_relaxed);
      RCLCPP_DEBUG(node_->get_logger(),
                   "[CheckPauseCustomCondition] reset_seq changed -> latched state cleared");
    }
  }

  // --- 논리적 latch 동작 ---
  if (latch_) {
    // true가 유지되는 동안 SUCCESS
    if (last_flag_.load(std::memory_order_relaxed)) {
      return BT::NodeStatus::SUCCESS;
    }
  } else {
    // 1회성 이벤트 소비: true를 한번 보고 바로 false로 소모
    if (last_flag_.exchange(false, std::memory_order_acq_rel)) {
      return BT::NodeStatus::SUCCESS;
    }
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace amr_bt_nodes
```

---

### 플러그인 등록 (`plugin.cpp`)

```cpp
#include "behaviortree_cpp/bt_factory.h"
#include "amr_bt_nodes/check_pause_custom_condition.hpp"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::CheckPauseCustomCondition>("CheckPauseCustomCondition");
}
```

---

### XML 사용 예 (당신의 기존 트리 레이아웃 유지, 최소 변경)

```xml
<Sequence>
  <!-- reset_seq 포트만 추가 -->
  <CheckPauseCustomCondition
      flag_topic="/controller_pause_flag"
      node="{node}"
      latch="true"
      reset_seq="{controller_pause_reset_seq}" />

  <LogTextAction message="[PAUSED] Controller is paused. Halting motion." interval_s="1.0"/>

  <SequenceWithMemory>
    <CancelControl/>

    <!-- 취소 직후: 에지 트리거 1회 발생 -->
    <Script code="controller_pause_reset_seq := controller_pause_reset_seq + 1"/>

    <Repeat num_cycles="-1">
      <Sequence>
        <CheckPauseCustomCondition
            flag_topic="/controller_pause_flag"
            latch="true"
            node="{node}"
            reset_seq="{controller_pause_reset_seq}" />
        <Wait wait_duration="0.1" />
      </Sequence>
    </Repeat>
  </SequenceWithMemory>
</Sequence>
```

> 체크포인트
>
> * 플러그인 심볼 이름이 `CheckPauseCustomCondition` 인지 확인 (XML 태그와 동일).
> * `controller_pause_reset_seq` 키는 **항상 int**로만 사용하세요.
> * `node="{node}"` 포트는 실제로 `rclcpp::Node::SharedPtr`를 공급해야 합니다.

필요하면 CMakeLists에 새 파일들 추가만 해주면 끝!



#########################


좋은 질문이에요. 그 코드는 \*\*“레벨 트리거(level-triggered) 강제 초기화”\*\*를 구현합니다. 흐름을 한 줄씩 풀면:

```cpp
bool do_reset = false;
if (getInput("reset", do_reset) && do_reset) {
  last_flag_.store(false, std::memory_order_relaxed);
}
```

* `getInput("reset", do_reset)` : 블랙보드의 `reset` 변수를 읽어옵니다.
* `&& do_reset` : 그 값이 \*\*true인 “상태(레벨)”\*\*이면,
* `last_flag_.store(false, ...)` : 내부 latched 상태(`last_flag_`)를 **무조건 false로 강제로 클리어**합니다.
  → 이 순간부터 아래 latch 로직과 무관하게, 이번 tick에서는 `SUCCESS`가 아니라 `FAILURE`로 떨어지게 됩니다.

핵심은 \*\*“레벨 트리거”\*\*라는 점이에요.

* `reset`이 **true로 유지되어 있는 동안**엔 매 tick 마다 `last_flag_`가 계속 `false`로 강제 초기화됩니다.
  → 즉, **리셋 유지(hold reset)** 효과가 납니다.
* `reset`을 **false로 내려야**만, 이후부터 다시 토픽에서 온 `true`를 반영해 `SUCCESS`가 가능해집니다.

그래서, 만약 의도가 \*\*“한 번만 초기화(펄스)하고 바로 원래 모드로 복귀”\*\*라면 아래 중 하나로 바꾸는 게 더 직관적입니다.

---

### 옵션 1) “엣지 트리거(상승 에지)”로 바꾸기

`reset`이 **false→true**로 바뀌는 **순간**에만 초기화하고, true가 계속 유지돼도 추가 초기화는 하지 않음.

```cpp
// 클래스 멤버
bool prev_reset_ {false};

// tick() 안
bool reset_in = false;
getInput("reset", reset_in);
if (reset_in && !prev_reset_) {                // false -> true 상승 에지에서만
  last_flag_.store(false, std::memory_order_relaxed);
}
prev_reset_ = reset_in;
```

* 장점: BT에서 `reset`을 길게 true로 둬도 **한 번만** 초기화됩니다.
* 단점: 다시 초기화하려면 `reset`을 **false로 내렸다가** 다시 **true로 올려야** 합니다.

---

### 옵션 2) “카운터(단일 변수 int)”로 바꾸기

여전히 **변수는 1개**만 쓰면서, 값이 **변할 때마다** 초기화:

```cpp
// 포트: BT::InputPort<int>("reset_seq", 0, "...");
// 멤버
int last_seen_reset_seq_ {-1};

// tick() 안
int seq = 0;
if (getInput("reset_seq", seq) && seq != last_seen_reset_seq_) {
  last_seen_reset_seq_ = seq;
  last_flag_.store(false, std::memory_order_relaxed);
}
```

* 장점: BT에서 `<Script code="reset_seq := reset_seq + 1"/>` 한 줄만 쓰면 **매번 “한 번만”** 깔끔히 초기화됩니다(되돌릴 필요 없음).
* 단점: bool이 아니라 int지만, **여전히 블랙보드 변수는 1개**입니다.

---

### 언제 레벨 트리거가 유용한가?

* “취소/중단 구간 동안은 **계속** 해제 상태를 유지하고 싶다” 같이 **구간 보장**이 필요할 때.
* 예: `<Repeat num_cycles="-1">` 루프 내부에서 잠시 멈춤을 강제 유지하고 싶으면 `reset=true`를 유지하면 됩니다.

반대로, **한 번만 초기화하고 바로 원래 동작으로 돌아가고 싶다면** 레벨 트리거(현재 코드)는 적합하지 않고, **엣지 트리거(옵션 1)** 또는 \*\*카운터(옵션 2)\*\*가 의도에 더 맞습니다.

