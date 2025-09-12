좋아, blackboard 변수 1개만 받아서 즉시 초기화(level-triggered reset)하도록 코드를 전면 정리했어요.
reset 포트가 true이면 latch 여부와 무관하게 즉시 latched 상태를 해제합니다. (다음 tick부터 정상 평가)

유지: latch(논리적 래치), transient_local(DDS 래치), halt() 시 강제 초기화

추가: reset(bool, 기본 false) — 블랙보드에서 받는 단 하나의 변수



---

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


---

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


---

플러그인 등록

#include "behaviortree_cpp/bt_factory.h"
#include "amr_bt_nodes/check_flag_condition.hpp"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::CheckFlagCondition>("CheckFlagCondition");
}


---

BT XML 사용 예 (reset = 블랙보드 변수 1개)

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

controller_pause_reset(블랙보드 단 하나의 변수)을 true로 올리면, 해당 tick에서 즉시 초기화됩니다.

이후 필요 시 <Script code="controller_pause_reset := false"/>로 내리면 됩니다.

추가로, Nav2가 트리를 멈추는 상황(취소/프리엠프/중단)에서는 halt()로도 한 번 더 안전하게 초기화됩니다.


필요하면 reset의 기본값을 false로 두었으므로, 블랙보드에 변수가 없을 때도 안전하게 동작합니다.

