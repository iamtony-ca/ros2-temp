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
