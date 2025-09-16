네, Nav2 및 BehaviorTree.Cpp 전문가로서 요청하신 `BoolPublisherAction` BT 노드를 완벽하게 만들어 드리겠습니다.

제공해주신 `StringPublisherAction` 코드는 ROS 2 노드 핸들을 BT 노드로 전달하고, 지정된 횟수만큼 메시지를 발행하는 매우 훌륭한 패턴을 따르고 있습니다. 이 구조를 그대로 활용하여 메시지 타입만 `std_msgs::msg::String`에서 `std_msgs::msg::Bool`로 변경하고, 관련 포트 및 로직을 수정하면 됩니다.

아래에 `bool_publisher_action.hpp` 헤더 파일과 `bool_publisher_action.cpp` 소스 파일의 전체 코드를 제시합니다.

-----

### 1\. 헤더 파일 (`include/amr_bt_nodes/bool_publisher_action.hpp`)

`std_msgs/msg/string.hpp`를 `std_msgs/msg/bool.hpp`로 변경하고, 클래스 이름과 퍼블리셔 타입을 수정합니다.

```cpp
// include/amr_bt_nodes/bool_publisher_action.hpp

#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp" // String -> Bool 메시지 헤더로 변경

namespace amr_bt_nodes
{

/**
 * @brief Tick이 호출되면 그 즉시, 지정된 횟수만큼 boolean 메시지를 연속 발행하는 SyncActionNode
 */
class BoolPublisherAction : public BT::SyncActionNode
{
public:
  BoolPublisherAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  // SyncActionNode의 핵심. 모든 로직이 이 안에서 완료됩니다.
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_; // String -> Bool 타입으로 변경
};

} // namespace amr_bt_nodes
```

### 2\. 소스 파일 (`src/bool_publisher_action.cpp`)

소스 파일에서는 생성자, `providedPorts`, `tick` 함수 내부의 로직과 타입, 그리고 플러그인 등록 부분을 모두 `Bool` 타입에 맞게 수정합니다.

```cpp
// src/bool_publisher_action.cpp

#include "amr_bt_nodes/bool_publisher_action.hpp"

namespace amr_bt_nodes
{

BoolPublisherAction::BoolPublisherAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  if (!getInput("node", node_)) {
    throw BT::RuntimeError("[BoolPublisherAction] Missing required input 'node'");
  }

  std::string topic_name;
  getInput("topic_name", topic_name);
  if (topic_name.empty()) {
    // 기본 토픽 이름 변경
    topic_name = "/bt_bool_publisher";
    RCLCPP_WARN(
      node_->get_logger(),
      "[BoolPublisherAction] 'topic_name' is not provided. Using default: '%s'", topic_name.c_str());
  }
  
  // QoS 설정
  rclcpp::QoS qos_profile(10);
  qos_profile.reliable();

  // 퍼블리셔 타입을 Bool로 변경
  publisher_ = node_->create_publisher<std_msgs::msg::Bool>(topic_name, qos_profile);

  RCLCPP_INFO(
    node_->get_logger(),
    "[BoolPublisherAction] Created. Publishing to topic: '%s'", topic_name.c_str());
}

BT::PortsList BoolPublisherAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "The ROS 2 node handle."),
    BT::InputPort<std::string>("topic_name", "/bt_bool_publisher", "Topic name to publish the bool to."),
    // message 포트의 타입을 bool로 변경
    BT::InputPort<bool>("message", "The boolean message to publish."),
    BT::InputPort<int>("num_publishes", 1, "Number of times to publish the message in a burst.")
  };
}

BT::NodeStatus BoolPublisherAction::tick()
{
  int num_publishes;
  if (!getInput("num_publishes", num_publishes) || num_publishes <= 0) {
    RCLCPP_ERROR(node_->get_logger(), "[BoolPublisherAction] 'num_publishes' must be a positive integer.");
    return BT::NodeStatus::FAILURE;
  }

  bool message_to_publish;
  if (!getInput("message", message_to_publish)) {
    RCLCPP_ERROR(node_->get_logger(), "[BoolPublisherAction] Missing required input 'message'.");
    return BT::NodeStatus::FAILURE;
  }

  // 메시지 타입을 std_msgs::msg::Bool로 변경
  std_msgs::msg::Bool msg;
  msg.data = message_to_publish;

  // 로그 메시지에서 bool 값을 명확하게 (true/false) 출력하도록 수정
  RCLCPP_INFO(
    node_->get_logger(),
    "[BoolPublisherAction] Publishing message '%s' %d times in a burst.",
    message_to_publish ? "true" : "false", num_publishes);
    
  for (int i = 0; i < num_publishes; ++i) {
    publisher_->publish(msg);
  }

  // 모든 작업이 tick() 안에서 완료되었으므로 SUCCESS 반환
  return BT::NodeStatus::SUCCESS;
}

} // namespace amr_bt_nodes


// 플러그인 등록
#include "behaviortree_cpp/bt_factory.h"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  // 클래스 이름을 BoolPublisherAction으로 변경하여 등록
  factory.registerNodeType<amr_bt_nodes::BoolPublisherAction>("BoolPublisherAction");
}
```

### 주요 변경 사항 요약

1.  **메시지 타입 변경**: `std_msgs::msg::String`을 사용하는 모든 부분을 `std_msgs::msg::Bool`로 교체했습니다. (`#include`, `rclcpp::Publisher`, `std_msgs::msg::... msg;`)
2.  **클래스 이름 변경**: `StringPublisherAction`을 `BoolPublisherAction`으로 일관되게 변경했습니다.
3.  **입력 포트 타입 변경**: `providedPorts()`에서 `message` 포트의 타입을 `std::string`에서 `bool`로 변경했습니다. BT.CPP는 `bool` 타입을 기본적으로 지원하므로 별도의 변환이 필요 없습니다.
4.  **기본 토픽 이름 변경**: 혼동을 피하기 위해 기본 토픽 이름을 `/bt_string_publisher`에서 `/bt_bool_publisher`로 수정했습니다.
5.  **로그 메시지 개선**: `tick()` 함수 내에서 `bool` 값을 출력할 때 `true` 또는 `false` 문자열로 명확하게 표시되도록 삼항 연산자를 사용했습니다.
6.  **플러그인 등록**: `BT_RegisterNodesFromPlugin` 함수에서 등록하는 노드 타입을 `BoolPublisherAction`으로 수정했습니다.

이제 이 코드를 ROS 2 패키지에 추가하고 빌드하면 `BoolPublisherAction`이라는 이름으로 Behavior Tree XML 파일에서 사용할 수 있습니다.

### XML에서 사용 예시

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <BoolPublisherAction topic_name="/clear_costmap" message="true" num_publishes="2" node="{ros_node}"/>
    </Sequence>
  </BehaviorTree>
</root>
```

궁금한 점이 있다면 언제든지 다시 질문해 주세요.

#############

네, 물론입니다. BehaviorTree.Cpp 노드, 특히 `SyncActionNode`의 `tick()` 함수 내에서 `sleep`을 추가하는 것은 신중하게 접근해야 합니다.

요청하신 대로 `for` 루프에 `sleep`을 넣는 방법과, 왜 그것이 문제가 될 수 있는지, 그리고 BehaviorTree.Cpp에서 권장하는 더 나은 대안은 무엇인지 단계별로 설명해 드리겠습니다.

### 1\. 직접적인 방법: `std::this_thread::sleep_for` 사용하기

가장 간단하게는 이전에 설명드린 `std::this_thread::sleep_for`를 `for` 루프 안에 직접 추가하는 것입니다.

**수정된 `src/bool_publisher_action.cpp`의 `tick()` 함수:**

```cpp
// ... 필요한 헤더에 추가 ...
#include <chrono>
#include <thread>

// ...

BT::NodeStatus BoolPublisherAction::tick()
{
  // ... (입력 포트 읽는 부분은 동일) ...

  int num_publishes;
  if (!getInput("num_publishes", num_publishes) || num_publishes <= 0) {
    RCLCPP_ERROR(node_->get_logger(), "[BoolPublisherAction] 'num_publishes' must be a positive integer.");
    return BT::NodeStatus::FAILURE;
  }

  bool message_to_publish;
  if (!getInput("message", message_to_publish)) {
    RCLCPP_ERROR(node_->get_logger(), "[BoolPublisherAction] Missing required input 'message'.");
    return BT::NodeStatus::FAILURE;
  }

  // --- 새로운 입력 포트: 발행 간격 ---
  int publish_interval_ms;
  getInput("publish_interval_ms", publish_interval_ms);
  if (publish_interval_ms < 0) {
    RCLCPP_WARN(node_->get_logger(), "[BoolPublisherAction] 'publish_interval_ms' is negative, treating as 0.");
    publish_interval_ms = 0;
  }
  // ---------------------------------

  std_msgs::msg::Bool msg;
  msg.data = message_to_publish;

  RCLCPP_INFO(
    node_->get_logger(),
    "[BoolPublisherAction] Publishing '%s' %d times with %dms interval.",
    message_to_publish ? "true" : "false", num_publishes, publish_interval_ms);
    
  for (int i = 0; i < num_publishes; ++i) {
    publisher_->publish(msg);
    // 첫 발행 후 마지막을 제외하고 sleep
    if (publish_interval_ms > 0 && i < num_publishes - 1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(publish_interval_ms));
    }
  }

  return BT::NodeStatus::SUCCESS;
}
```

**새로운 포트를 추가하기 위해 `providedPorts()`도 수정해야 합니다:**

```cpp
// src/bool_publisher_action.cpp

// ...

BT::PortsList BoolPublisherAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "The ROS 2 node handle."),
    BT::InputPort<std::string>("topic_name", "/bt_bool_publisher", "Topic name to publish the bool to."),
    BT::InputPort<bool>("message", "The boolean message to publish."),
    BT::InputPort<int>("num_publishes", 1, "Number of times to publish the message in a burst."),
    // --- 새로운 포트 추가 ---
    BT::InputPort<int>("publish_interval_ms", 0, "Interval between publishes in milliseconds.")
  };
}
```

-----

### ⚠️ 중요: `SyncActionNode`에서 `sleep` 사용의 문제점

위의 방법은 동작은 하지만, `SyncActionNode`의 `tick()` 함수를 장시간 점유(blocking)하게 만듭니다.

  - **`SyncActionNode`의 특징**: `tick()` 함수가 호출되면, 함수가 `SUCCESS` 또는 `FAILURE`를 반환할 때까지 **전체 Behavior Tree의 실행이 멈춥니다.**
  - **문제점**: 만약 `num_publishes`가 10이고 `publish_interval_ms`가 100이라면, `tick()` 함수는 약 1초 동안 실행됩니다. 그 1초 동안 Behavior Tree는 다른 노드를 전혀 실행하지 못하고 "얼어있는" 상태가 됩니다. 이는 로봇의 반응성을 크게 떨어뜨리는 원인이 됩니다.

로봇 제어와 같이 실시간성이 중요한 시스템에서는 이러한 blocking 방식은 피해야 합니다.

-----

### 2\. 권장하는 방법: `StatefulActionNode` (비동기 방식) 사용하기

`sleep`처럼 시간이 걸리는 작업을 처리하기 위해 BehaviorTree.Cpp는 `StatefulActionNode`(이전 버전에서는 `AsyncActionNode`)를 제공합니다. 이 노드는 `tick()`이 여러 번 호출되는 동안 내부 상태를 유지하며 작업을 수행합니다.

  - **동작 방식**:
    1.  `onStart()`: 노드가 처음 실행될 때 한 번 호출됩니다. 여기서 초기 설정을 합니다.
    2.  `onRunning()`: 노드가 `RUNNING` 상태일 때 매 `tick`마다 호출됩니다. 여기서 실제 작업(메시지 발행, 시간 확인 등)을 수행합니다.
    3.  `onHalted()`: 다른 노드에 의해 실행이 중단될 때 호출됩니다. 정리 작업을 합니다.

`StatefulActionNode`를 사용하면 `tick()` 함수가 즉시 `RUNNING` 상태를 반환하므로 전체 트리가 멈추지 않습니다.

#### `StatefulActionNode`로 리팩토링한 코드

**1. 헤더 파일 수정 (`include/amr_bt_nodes/bool_publisher_action.hpp`)**

  - `BT::SyncActionNode` -\> `BT::StatefulActionNode` 로 변경
  - 상태를 저장할 멤버 변수 추가
  - `tick()` 대신 `onStart()`, `onRunning()`, `onHalted()` 오버라이드

<!-- end list -->

```hpp
#pragma once

// SyncActionNode -> StatefulActionNode
#include "behaviortree_cpp/stateful_action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace amr_bt_nodes
{

class BoolPublisherAction : public BT::StatefulActionNode
{
public:
  BoolPublisherAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  // StatefulActionNode의 가상 함수들
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;

  // 상태를 저장하기 위한 멤버 변수들
  int num_publishes_;
  int publish_count_;
  std::chrono::milliseconds publish_interval_;
  std::chrono::steady_clock::time_point last_publish_time_;
  std_msgs::msg::Bool msg_to_publish_;
};

} // namespace amr_bt_nodes
```

**2. 소스 파일 수정 (`src/bool_publisher_action.cpp`)**

  - 생성자에서 `BT::SyncActionNode` -\> `BT::StatefulActionNode` 변경
  - `tick()` 함수를 `onStart()`, `onRunning()`, `onHalted()` 로 분리하여 구현

<!-- end list -->

```cpp
#include "amr_bt_nodes/bool_publisher_action.hpp"

namespace amr_bt_nodes
{

// 부모 클래스 변경
BoolPublisherAction::BoolPublisherAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  // 생성자 로직은 거의 동일
  if (!getInput("node", node_)) {
    throw BT::RuntimeError("[BoolPublisherAction] Missing required input 'node'");
  }
  // ... (토픽 이름, 퍼블리셔 생성 등은 동일)
}

BT::PortsList BoolPublisherAction::providedPorts()
{
  // 포트 리스트는 이전과 동일하게 publish_interval_ms 추가
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "The ROS 2 node handle."),
    BT::InputPort<std::string>("topic_name", "/bt_bool_publisher", "Topic name to publish the bool to."),
    BT::InputPort<bool>("message", "The boolean message to publish."),
    BT::InputPort<int>("num_publishes", 1, "Number of times to publish the message."),
    BT::InputPort<int>("publish_interval_ms", 0, "Interval between publishes in milliseconds.")
  };
}

BT::NodeStatus BoolPublisherAction::onStart()
{
  // 노드가 처음 실행될 때 입력 포트에서 값을 읽어와 멤버 변수에 저장
  if (!getInput("num_publishes", num_publishes_) || num_publishes_ <= 0) {
    RCLCPP_ERROR(node_->get_logger(), "[BoolPublisherAction] 'num_publishes' must be a positive integer.");
    return BT::NodeStatus::FAILURE;
  }
  
  bool message_data;
  if (!getInput("message", message_data)) {
    RCLCPP_ERROR(node_->get_logger(), "[BoolPublisherAction] Missing required input 'message'.");
    return BT::NodeStatus::FAILURE;
  }
  msg_to_publish_.data = message_data;

  int interval_ms;
  getInput("publish_interval_ms", interval_ms);
  publish_interval_ = std::chrono::milliseconds(interval_ms < 0 ? 0 : interval_ms);

  RCLCPP_INFO(
    node_->get_logger(),
    "[BoolPublisherAction] Starting to publish '%s' %d times with %ldms interval.",
    msg_to_publish_.data ? "true" : "false", num_publishes_, publish_interval_.count());

  // 상태 변수 초기화
  publish_count_ = 0;

  // 첫 번째 메시지는 즉시 발행
  publisher_->publish(msg_to_publish_);
  publish_count_++;
  last_publish_time_ = std::chrono::steady_clock::now();
  
  // 목표 횟수가 1번이면 바로 SUCCESS
  if (publish_count_ >= num_publishes_) {
    return BT::NodeStatus::SUCCESS;
  }

  // 아직 더 발행해야 하므로 RUNNING 상태 반환
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus BoolPublisherAction::onRunning()
{
  // 현재 시간이 마지막 발행 시간 + 간격보다 지났는지 확인
  auto now = std::chrono::steady_clock::now();
  if (now > last_publish_time_ + publish_interval_) {
    publisher_->publish(msg_to_publish_);
    publish_count_++;
    last_publish_time_ = now;
  }

  // 목표 횟수를 채웠으면 SUCCESS 반환
  if (publish_count_ >= num_publishes_) {
    return BT::NodeStatus::SUCCESS;
  }
  
  // 아직 더 발행해야 하므로 RUNNING 상태 유지
  return BT::NodeStatus::RUNNING;
}

void BoolPublisherAction::onHalted()
{
  // 노드가 중단되었을 때 호출됨 (예: Fallback 노드에 의해)
  RCLCPP_INFO(node_->get_logger(), "[BoolPublisherAction] Halted. Published %d times.", publish_count_);
}

} // namespace amr_bt_nodes

// ... (플러그인 등록 부분은 동일) ...
```

### 결론

| 접근 방식                          | 장점                                        | 단점                                                               | 추천 상황                                     |
| ---------------------------------- | ------------------------------------------- | ------------------------------------------------------------------ | --------------------------------------------- |
| **`SyncActionNode` + `sleep`** | 구현이 매우 간단하고 직관적임.              | Behavior Tree 전체를 blocking하여 시스템 반응성을 저하.            | **비추천**. 아주 짧은(수 ms 이내) sleep일 경우에만 고려. |
| **`StatefulActionNode` (비동기)** | **Non-blocking**. BT의 실행을 막지 않음.      | `SyncActionNode`보다 코드가 복잡하고 상태 관리가 필요.             | **강력 추천**. 시간이 걸리는 모든 작업에 적합.    |

로보틱스 분야에서는 시스템의 반응성이 매우 중요하므로, **`StatefulActionNode`를 사용하는 두 번째 방법을 강력히 추천합니다.** 이 방식은 BehaviorTree.Cpp 프레임워크를 올바르게 활용하는 방법입니다.
