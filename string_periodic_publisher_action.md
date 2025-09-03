네, 알겠습니다. 클래스 이름을 `StringPeriodicPublisherAction`으로 변경하여 최종 코드를 다시 전달해 드리겠습니다.

-----

### 최종 코드 (`StringPeriodicPublisherAction`)

#### 1\. 헤더 파일 (`string_periodic_publisher_action.hpp`)

```cpp
// include/amr_bt_nodes/string_periodic_publisher_action.hpp

#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace amr_bt_nodes
{

/**
 * @brief 설정된 간격(interval)마다 N개의 메시지를 Burst 형태로 발행하는 SyncActionNode.
 * 이 작업은 명확한 종료 조건 없이 계속 반복됩니다.
 */
class StringPeriodicPublisherAction : public BT::SyncActionNode
{
public:
  StringPeriodicPublisherAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  // 마지막으로 'Burst' 발행이 일어난 시간을 기록
  rclcpp::Time last_burst_time_;
};

} // namespace amr_bt_nodes
```

#### 2\. 소스 파일 (`string_periodic_publisher_action.cpp`)

```cpp
// src/string_periodic_publisher_action.cpp

#include "amr_bt_nodes/string_periodic_publisher_action.hpp"
#include "behaviortree_cpp/bt_factory.h"

namespace amr_bt_nodes
{

StringPeriodicPublisherAction::StringPeriodicPublisherAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  if (!getInput("node", node_)) {
    throw BT::RuntimeError("[StringPeriodicPublisherAction] Missing required input 'node'");
  }

  // 생성자에서 last_burst_time_을 초기화하여 첫 tick에서는 항상 발행되도록 함
  last_burst_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());

  std::string topic_name;
  getInput("topic_name", topic_name);
  if (topic_name.empty()) {
    topic_name = "/bt_string_publisher";
    RCLCPP_WARN(
      node_->get_logger(),
      "[StringPeriodicPublisherAction] 'topic_name' is not provided. Using default: '%s'", topic_name.c_str());
  }
  
  rclcpp::QoS qos_profile(10);
  qos_profile.reliable();
  publisher_ = node_->create_publisher<std_msgs::msg::String>(topic_name, qos_profile);

  RCLCPP_INFO(
    node_->get_logger(),
    "[StringPeriodicPublisherAction] Created. Publishing to topic: '%s'", topic_name.c_str());
}

BT::PortsList StringPeriodicPublisherAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "The ROS 2 node."),
    BT::InputPort<std::string>("topic_name", "/bt_string_publisher", "Topic name to publish."),
    BT::InputPort<std::string>("message", "The string message to publish."),
    BT::InputPort<int>("num_publishes", 1, "Number of messages to publish in each burst."),
    BT::InputPort<double>("publish_interval", 1.0, "The minimum interval in seconds between bursts.")
  };
}

BT::NodeStatus StringPeriodicPublisherAction::tick()
{
  double interval;
  if (!getInput("publish_interval", interval)) {
     RCLCPP_ERROR(node_->get_logger(), "[StringPeriodicPublisherAction] Missing required input 'publish_interval'.");
     return BT::NodeStatus::FAILURE;
  }

  // 시간 간격 체크 로직
  rclcpp::Time current_time = node_->get_clock()->now();
  if ((current_time - last_burst_time_).seconds() >= interval) {
    // 시간이 되면 Burst 발행을 수행
    int num_publishes;
    if (!getInput("num_publishes", num_publishes) || num_publishes <= 0) {
      RCLCPP_ERROR(node_->get_logger(), "[StringPeriodicPublisherAction] 'num_publishes' must be a positive integer.");
      return BT::NodeStatus::FAILURE;
    }

    std::string message_to_publish;
    if (!getInput("message", message_to_publish)) {
      RCLCPP_ERROR(node_->get_logger(), "[StringPeriodicPublisherAction] Missing required input 'message'.");
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "[StringPeriodicPublisherAction] Interval met. Publishing '%s' %d times.",
      message_to_publish.c_str(), num_publishes);

    std_msgs::msg::String msg;
    msg.data = message_to_publish;
    for (int i = 0; i < num_publishes; ++i) {
      publisher_->publish(msg);
    }

    // 마지막 Burst 발행 시간을 현재 시간으로 갱신
    last_burst_time_ = current_time;
  }
  
  // 실제 발행 여부와 관계없이 항상 SUCCESS를 반환하여 BT의 흐름을 막지 않음
  return BT::NodeStatus::SUCCESS;
}

} // namespace amr_bt_nodes

// 플러그인 등록
extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::StringPeriodicPublisherAction>("StringPeriodicPublisherAction");
}
```

#### 3\. 사용 예시 (Behavior Tree XML)

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
        <StringPeriodicPublisherAction
            node="{node}"
            topic_name="/robot_status"
            message="keep_alive"
            num_publishes="3"
            publish_interval="5.0" />

        </Sequence>
  </BehaviorTree>
</root>
```

요청하신 이름으로 모두 변경했습니다.







네, 알겠습니다. `StringPeriodicPublisherAction` 노드를 위한 완벽한 `plugin.xml` 파일과, 실제 사용 시나리오를 가정한 `bt.xml` 예제를 제공해 드리겠습니다.

-----

### 1\. 플러그인 XML (`bt_plugins.xml`)

이 파일은 ROS 2 시스템이 여러분의 커스텀 BT 노드를 라이브러리에서 찾아 로드할 수 있도록 하는 "지도" 역할을 합니다. 보통 패키지 루트에 위치시키고 `CMakeLists.txt`를 통해 설치합니다.

**파일명: `bt_plugins.xml`**

```xml
<library path="amr_bt_nodes_string_periodic_publisher_action">
  <class name="StringPeriodicPublisherAction" type="amr_bt_nodes::StringPeriodicPublisherAction" base_class_type="BT::ActionNode">
    <description>
      지정된 시간 간격(interval)마다 N개의 메시지를 Burst 형태로 주기적으로 발행하는 SyncActionNode 플러그인입니다.
    </description>
  </class>

  </library>
```

### 2\. Behavior Tree 사용 예시 XML (`usage_example.xml`)

아래 예시는 로봇이 순찰 임무를 수행하는 동안, 5초마다 자신의 상태("PATROLLING")를 2번씩 발행하여 시스템에 알리는 상황을 가정합니다. `StringPeriodicPublisherAction`이 `SyncActionNode`이므로 다른 행동을 막지 않는다는 점을 명확히 보여줍니다.

**파일명: `usage_example.xml`**

```xml
<root main_tree_to_execute="PatrolMission">
  <BehaviorTree ID="PatrolMission">
    <RateController hz="1.0">
      <ReactiveFallback name="MissionControl">
        
        <StringPeriodicPublisherAction
            node="{node}"
            topic_name="/robot_heartbeat"
            message="PATROLLING"
            num_publishes="2"
            publish_interval="5.0" />

        <Sequence name="PatrolRoute">
          <SaySomething message="Starting patrol... Next waypoint: A" />
          <FollowPath path="{path_to_A}" />
          <SaySomething message="Arrived at A. Next waypoint: B" />
          <FollowPath path="{path_to_B}" />
        </Sequence>
      </ReactiveFallback>
    </RateController>
  </BehaviorTree>


  <BehaviorTree ID="PatrolMission_Parallel">
    <RateController hz="1.0">
      <Parallel success_threshold="1">

        <StringPeriodicPublisherAction
            node="{node}"
            topic_name="/robot_heartbeat"
            message="PATROLLING"
            num_publishes="2"
            publish_interval="5.0" />

        <Sequence name="PatrolRoute">
          <SaySomething message="Starting patrol to A..." />
          <FollowPath path="{path_to_A}" />
          <SaySomething message="Arrived at A. Moving to B..." />
          <FollowPath path="{path_to_B}" />
        </Sequence>

      </Parallel>
    </RateController>
  </BehaviorTree>

</root>
```

### 설명

  * **`bt_plugins.xml`**: 이 파일의 `library path`는 `CMakeLists.txt`에서 정의한 라이브러리 이름과 반드시 일치해야 합니다. 이 파일을 `CMakeLists.txt`의 `install` 명령과 `ament_export_interfaces` 또는 `pluginlib_export_plugin_description_file`을 통해 올바르게 설치하고 내보내야 Nav2와 같은 시스템이 플러그인을 찾을 수 있습니다.
  * **`usage_example.xml`**:
      * 첫 번째 예시(`PatrolMission`)는 `Fallback` 아래에 두어 노드의 기본적인 사용법을 보여주지만, `StringPeriodicPublisherAction`이 항상 `SUCCESS`를 반환하기 때문에 실제로는 순찰 로직이 실행되지 않을 수 있다는 문제점을 함께 보여줍니다.
      * 두 번째 예시(`PatrolMission_Parallel`)가 훨씬 더 현실적이고 올바른 사용법입니다. `Parallel` 노드를 사용하여 "주기적인 상태 발행"과 "실제 경로 이동"이라는 두 가지 행동을 **동시에** 수행할 수 있습니다. 이것이 바로 `SyncActionNode`를 사용해 BT 흐름을 막지 않도록 설계한 주된 이유입니다.

