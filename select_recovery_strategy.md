네, 좋습니다. `Switch` 구조로 변경하는 것은 매우 현명한 결정입니다. 요청하신 대로 `select_recovery_strategy` 노드의 전체 소스코드와 이를 사용하는 `IntelligentRecoverySubtree` BT XML을 완벽하게 작성해 드리겠습니다.

아래 코드는 Nav2 Humble/Jazzy 환경과 최신 C++(17) 표준을 준수하여 작성되었습니다.

-----

### 1\. 커스텀 노드 소스코드 (`select_recovery_strategy`)

이 노드는 플래너/컨트롤러 에러 코드와 로봇의 상태 플래그를 종합적으로 판단하여 가장 적절한 복구 전략의 이름을 문자열로 반환하는 역할을 합니다.

#### **`select_recovery_strategy.hpp` (헤더 파일)**

```cpp
#pragma once

#include <string>
#include <atomic>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace your_namespace
{

class SelectRecoveryStrategy : public BT::SyncActionNode
{
public:
  SelectRecoveryStrategy(
    const std::string & name,
    const BT::NodeConfiguration & config,
    rclcpp::Node::SharedPtr node)
  : BT::SyncActionNode(name, config),
    node_(node),
    is_stuck_{false}
  {
    // Progress checker의 is_stuck 토픽을 구독합니다.
    is_stuck_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/progress_checker/is_stuck",
      rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        this->is_stuck_.store(msg->data, std::memory_order_release);
      });
  }

  // 노드가 사용하는 포트 목록을 정의합니다.
  static BT::PortsList providedPorts()
  {
    return {
      // Input Ports from Blackboard
      BT::InputPort<int>("compute_path_error_code", "Planner-side error code"),
      BT::InputPort<int>("follow_path_error_code", "Controller-side error code"),
      
      // Error code constants from Blackboard (for comparison)
      BT::InputPort<int>("cfg_err_no_valid"),
      BT::InputPort<int>("cfg_err_start_occ"),
      BT::InputPort<int>("cfg_err_goal_occ"),
      // 아래는 예시로 추가한 컨트롤러 에러 코드입니다. 필요에 맞게 추가/수정하세요.
      BT::InputPort<int>("cfg_err_oscillation", 501, "Example controller oscillation error"),
      BT::InputPort<int>("cfg_err_path_tolerance", 502, "Example path tolerance error"),

      // Output Port to Blackboard
      BT::OutputPort<std::string>("selected_strategy", "The name of the recovery case to execute")
    };
  }

  // 노드의 실제 실행 로직
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_stuck_sub_;
  std::atomic<bool> is_stuck_;
};

}  // namespace your_namespace
```

#### **`select_recovery_strategy.cpp` (구현 파일)**

```cpp
#include "your_namespace/select_recovery_strategy.hpp"

namespace your_namespace
{

BT::NodeStatus SelectRecoveryStrategy::tick()
{
  // 블랙보드에서 에러 코드와 설정값들을 읽어옵니다.
  const auto compute_error = getInput<int>("compute_path_error_code");
  const auto follow_error = getInput<int>("follow_path_error_code");

  const auto no_valid_err = getInput<int>("cfg_err_no_valid");
  const auto start_occ_err = getInput<int>("cfg_err_start_occ");
  const auto goal_occ_err = getInput<int>("cfg_err_goal_occ");
  const auto oscillation_err = getInput<int>("cfg_err_oscillation");
  const auto tolerance_err = getInput<int>("cfg_err_path_tolerance");

  std::string selected_strategy = "DEFAULT"; // 기본값 설정

  // --- 복구 전략 선택 로직 (우선순위가 중요합니다) ---
  // 1. 경로 계획(Planner) 실패는 가장 근본적인 문제이므로 최우선으로 확인합니다.
  if (compute_error && compute_error.value() != 0) {
    if (compute_error.value() == no_valid_err.value()) {
      selected_strategy = "NO_VALID_PATH";
    } else if (compute_error.value() == start_occ_err.value()) {
      selected_strategy = "START_OCCUPIED";
    } else if (compute_error.value() == goal_occ_err.value()) {
      selected_strategy = "GOAL_OCCUPIED";
    }
  }
  // 2. 로봇이 물리적으로 갇힌(stuck) 상태는 심각하므로 그 다음으로 확인합니다.
  // exchange는 현재 값을 읽어오면서 동시에 false로 아토믹하게 설정하여, 플래그를 "소비"합니다.
  else if (is_stuck_.exchange(false, std::memory_order_acq_rel))
  {
    selected_strategy = "ROBOT_STUCK";
  }
  // 3. 경로 추종(Controller) 실패를 확인합니다.
  else if (follow_error && follow_error.value() != 0) {
    if (follow_error.value() == oscillation_err.value()) {
      selected_strategy = "OSCILLATION";
    } else if (follow_error.value() == tolerance_err.value()) {
      selected_strategy = "PATH_TOLERANCE_VIOLATED";
    }
  }

  // 선택된 전략을 블랙보드에 기록합니다.
  setOutput("selected_strategy", selected_strategy);
  RCLCPP_INFO(node_->get_logger(), "[RecoverySelector] Selected strategy: '%s'", selected_strategy.c_str());

  // 이 노드의 역할은 '선택'이므로, 선택을 완료했다면 항상 SUCCESS를 반환합니다.
  return BT::NodeStatus::SUCCESS;
}

}  // namespace your_namespace

// --- 플러그인 등록을 위한 코드 ---
// 이 코드는 별도의 파일에 있거나, nav2_behavior_tree 패키지의 일부로 통합되어야 합니다.
// #include "behaviortree_cpp/bt_factory.h"
// #include "your_namespace/select_recovery_strategy.hpp"
//
// ...
// factory.registerNodeType<your_namespace::SelectRecoveryStrategy>(
//   "SelectRecoveryStrategy", rclcpp::Node::SharedPtr(node));
// ...
```

**[중요]** 위 코드를 사용하려면 `nav2_behavior_tree`와 같은 패키지 내에 소스코드를 추가하고, `CMakeLists.txt`와 `plugin.xml`에 해당 노드를 BT 플러그인으로 등록하는 과정이 필요합니다.

-----

### 2\. Behavior Tree XML (`IntelligentRecoverySubtree`)

기존의 `Fallback` 구조를 `Switch`를 사용하도록 변경한 최종 XML입니다.

```xml
<BehaviorTree ID="IntelligentRecoverySubtree">
  <Sequence name="RecoverySequence">
    <LogTextAction name="RecoveryBanner"
      message="[!!! RECOVERY !!!] Main navigation failed. Initiating intelligent recovery logic."
      interval_s="1.0" />
    
    <Switch name="IntelligentRecoverySwitch" variable="{selected_strategy}">

      <SelectRecoveryStrategy name="SelectStrategy"
                              compute_path_error_code="{compute_path_error_code}"
                              follow_path_error_code="{follow_path_error_code}"
                              cfg_err_no_valid="{cfg_err_no_valid}"
                              cfg_err_start_occ="{cfg_err_start_occ}"
                              cfg_err_goal_occ="{cfg_err_goal_occ}"
                              selected_strategy="{selected_strategy}" />

      <Case case="NO_VALID_PATH">
        <Sequence name="NoValidPathActions">
          <LogTextAction name="NoValidPathLog"
            message="[RECOVERY] Case: 'No valid path' detected. Resolving..." interval_s="1.0" />
          <Fallback name="NoValidPathResolution">
            <Sequence name="GoalOccupiedBranch">
              <IsGoalsOccupiedCondition name="IsGoalSafe" goals="{goals}" costmap_topic="{cfg_costmap_topic}"
                occupied_cost_threshold="{cfg_occupied_threshold}" occupied_goals="{obstructed_goals}"
                unoccupied_goals="{unoccupied_goals}" />
              <LogTextAction name="GoalOccupiedLog" message=" -> Cause: Goal occupied. Waiting or removing goals." interval_s="1.0" />
              <RoundRobin name="GoalOccupiedRecovery">
                <Wait name="WaitToClear" wait_duration="{cfg_wait_default}" />
                <SubTree name="RemoveGoals_3" ID="RemoveGoalsSubtree" _autoremap="true" num_goals_to_remove="3" />
              </RoundRobin>
            </Sequence>
            <Sequence name="PathBlockedBranch">
              <LogTextAction name="PathBlockedLog" message=" -> Cause: Path blocked but goal clear. Clearing costmaps + maneuver." interval_s="1.0" />
              <RoundRobin name="PathBlockedRecovery">
                <SubTree name="ClearPair" ID="ClearCostmapsPairSubtree" _autoremap="true" />
                <SubTree name="Maneuver_PlanA" ID="ManeuverSubtree" _autoremap="true" planner_id="{cfg_recov_planner_A}" controller_id="{cfg_recov_controller}" />
              </RoundRobin>
            </Sequence>
          </Fallback>
        </Sequence>
      </Case>

      <Case case="ROBOT_STUCK">
        <Sequence name="RobotIsStuckActions">
          <LogTextAction name="RobotStuckLog" message="[RECOVERY] Case: Robot appears stuck. Executing maneuver." interval_s="1.0" />
          <RoundRobin name="RobotIsStuckRecoveryActions">
            <SubTree name="Maneuver_PlanA@Stuck" ID="ManeuverSubtree" _autoremap="true" planner_id="{cfg_recov_planner_A}" controller_id="{cfg_recov_controller}" />
            <SubTree name="Maneuver_PlanB@Stuck" ID="ManeuverSubtree" _autoremap="true" planner_id="{cfg_recov_planner_B}" controller_id="{cfg_recov_controller}" />
          </RoundRobin>
        </Sequence>
      </Case>

      <Case case="START_OCCUPIED">
        <Sequence name="StartOccupiedActions">
          <LogTextAction name="StartOccupiedLog" message="[RECOVERY] Case: 'Start is Occupied' detected. Trying maneuvers..." interval_s="1.0" />
          <RoundRobin name="StartOccupiedRecovery">
            <SubTree name="Maneuver_PlanA@Start" ID="ManeuverSubtree" _autoremap="true" planner_id="{cfg_recov_planner_A}" controller_id="{cfg_recov_controller}" />
            <SubTree name="Maneuver_PlanB@Start" ID="ManeuverSubtree" _autoremap="true" planner_id="{cfg_recov_planner_B}" controller_id="{cfg_recov_controller}" />
          </RoundRobin>
        </Sequence>
      </Case>
      
      <Case case="GOAL_OCCUPIED">
        <Sequence name="GoalOccupiedActions">
          <LogTextAction name="GoalOccupiedLog" message="[RECOVERY] Case: 'Goal is Occupied' detected. Waiting or removing goals." interval_s="1.0" />
          <RoundRobin name="GoalOccupiedRecovery">
            <Wait name="WaitToClear2" wait_duration="{cfg_wait_default}" />
            <SubTree name="RemoveGoals_3@Goal" ID="RemoveGoalsSubtree" _autoremap="true" num_goals_to_remove="3" />
          </RoundRobin>
        </Sequence>
      </Case>

      <Case case="OSCILLATION">
        <Sequence name="OscillationActions">
          <LogTextAction name="OscillationLog" message="[RECOVERY] Case: Controller is oscillating. Clearing local costmap and waiting." interval_s="1.0" />
          <ClearEntireCostmap name="ClearLocalCostmap@Oscillation" service_name="/local_costmap/clear_entirely_local_costmap" />
          <Wait name="WaitAfterClear" wait_duration="{cfg_wait_default}" />
        </Sequence>
      </Case>

      <Default>
        <Sequence name="DefaultFailureCase">
          <LogTextAction name="DefaultFailureLog" message="[RECOVERY] Case: Default failure. Running general recovery sequence." interval_s="1.0" />
          <RoundRobin name="DefaultFailureRecoveryActions">
            <SubTree name="ClearPair@Default" ID="ClearCostmapsPairSubtree" _autoremap="true" />
            <Wait name="DefaultWait" wait_duration="{cfg_wait_default}" />
            <SubTree name="Maneuver_PlanA@Default" ID="ManeuverSubtree" _autoremap="true" planner_id="{cfg_recov_planner_A}" controller_id="{cfg_recov_controller}" />
          </RoundRobin>
        </Sequence>
      </Default>

    </Switch>
  </Sequence>
</BehaviorTree>
```

이 구조를 적용하시면, 훨씬 더 예측 가능하고, 디버깅이 쉬우며, 견고한 복구 시스템을 갖추게 될 것입니다.

##################

네, 알겠습니다. 요청하신 대로 네임스페이스를 `amr_bt_nodes`로, 클래스 이름을 `SelectRecoveryStrategyAction`으로 변경하여 완벽하게 다듬어진 최종 코드를 제공해 드리겠습니다.

-----

### 1\. 커스텀 노드 최종 소스코드

#### **`select_recovery_strategy_action.hpp` (헤더 파일)**

```cpp
#pragma once

#include <string>
#include <atomic>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace amr_bt_nodes
{

/**
 * @class SelectRecoveryStrategyAction
 * @brief A BT::SyncActionNode that selects an appropriate recovery strategy
 * based on planner/controller error codes and robot status flags.
 *
 * This node reads various error codes and status flags from the blackboard,
 * evaluates them based on a predefined priority, and outputs the name of the
 * selected recovery strategy as a string to the `selected_strategy` port.
 * It is designed to be the first child of a <Switch> node to enable
 * deterministic, condition-based recovery behaviors.
 */
class SelectRecoveryStrategyAction : public BT::SyncActionNode
{
public:
  /**
   * @brief A constructor for amr_bt_nodes::SelectRecoveryStrategyAction
   * @param name The name of the node
   * @param config The node configuration
   * @param node A shared pointer to the ROS 2 node
   */
  SelectRecoveryStrategyAction(
    const std::string & name,
    const BT::NodeConfiguration & config,
    rclcpp::Node::SharedPtr node);

  /**
   * @brief Defines the ports that this node uses.
   * @return BT::PortsList The list of ports.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief The main execution logic of the node.
   * @return BT::NodeStatus SUCCESS after selecting a strategy.
   */
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_stuck_sub_;
  std::atomic<bool> is_stuck_;
};

}  // namespace amr_bt_nodes
```

#### **`select_recovery_strategy_action.cpp` (구현 파일)**

```cpp
#include "amr_bt_nodes/select_recovery_strategy_action.hpp"

namespace amr_bt_nodes
{

SelectRecoveryStrategyAction::SelectRecoveryStrategyAction(
  const std::string & name,
  const BT::NodeConfiguration & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config),
  node_(node),
  is_stuck_{false}
{
  // Subscribe to the /progress_checker/is_stuck topic to monitor robot's state.
  is_stuck_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/progress_checker/is_stuck",
    rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      // Store the received value atomically.
      this->is_stuck_.store(msg->data, std::memory_order_release);
    });
  
  RCLCPP_INFO(node_->get_logger(), R"(Initialized BT node "SelectRecoveryStrategyAction")");
}

BT::PortsList SelectRecoveryStrategyAction::providedPorts()
{
  return {
    // Input Ports to read from the Blackboard
    BT::InputPort<int>("compute_path_error_code", 0, "Planner-side error code"),
    BT::InputPort<int>("follow_path_error_code", 0, "Controller-side error code"),
    
    // Error code constants to compare against, also read from the Blackboard
    BT::InputPort<int>("cfg_err_no_valid"),
    BT::InputPort<int>("cfg_err_start_occ"),
    BT::InputPort<int>("cfg_err_goal_occ"),
    // Example controller error codes. Modify as needed for your specific controller.
    BT::InputPort<int>("cfg_err_oscillation", 501, "Example controller oscillation error"),
    BT::InputPort<int>("cfg_err_path_tolerance", 502, "Example path tolerance error"),

    // Output Port to write the result to the Blackboard
    BT::OutputPort<std::string>("selected_strategy", "The name of the recovery case to execute")
  };
}

BT::NodeStatus SelectRecoveryStrategyAction::tick()
{
  // Read error codes and configuration values from the input ports.
  const auto compute_error = getInput<int>("compute_path_error_code");
  const auto follow_error = getInput<int>("follow_path_error_code");

  const auto no_valid_err = getInput<int>("cfg_err_no_valid");
  const auto start_occ_err = getInput<int>("cfg_err_start_occ");
  const auto goal_occ_err = getInput<int>("cfg_err_goal_occ");
  const auto oscillation_err = getInput<int>("cfg_err_oscillation");
  const auto tolerance_err = getInput<int>("cfg_err_path_tolerance");

  std::string selected_strategy = "DEFAULT"; // Default strategy if no other condition is met.

  // --- Recovery Strategy Selection Logic (Priority is important) ---

  // 1. Planner failures are the most critical, so check them first.
  if (compute_error && compute_error.value() != 0) {
    if (compute_error.value() == no_valid_err.value()) {
      selected_strategy = "NO_VALID_PATH";
    } else if (compute_error.value() == start_occ_err.value()) {
      selected_strategy = "START_OCCUPIED";
    } else if (compute_error.value() == goal_occ_err.value()) {
      selected_strategy = "GOAL_OCCUPIED";
    }
  }
  // 2. A physically stuck robot is the next priority.
  //    The exchange() operation atomically reads the current value and sets it to false,
  //    effectively "consuming" the flag for this tick.
  else if (is_stuck_.exchange(false, std::memory_order_acq_rel))
  {
    selected_strategy = "ROBOT_STUCK";
  }
  // 3. Controller failures during path following are checked last.
  else if (follow_error && follow_error.value() != 0) {
    if (follow_error.value() == oscillation_err.value()) {
      selected_strategy = "OSCILLATION";
    } else if (follow_error.value() == tolerance_err.value()) {
      selected_strategy = "PATH_TOLERANCE_VIOLATED";
    }
  }

  // Write the chosen strategy to the blackboard for the <Switch> node to use.
  setOutput("selected_strategy", selected_strategy);
  RCLCPP_DEBUG(node_->get_logger(), "[RecoverySelector] Selected strategy: '%s'", selected_strategy.c_str());

  // This node's job is only to select a strategy, so it always returns SUCCESS upon completion.
  return BT::NodeStatus::SUCCESS;
}

}  // namespace amr_bt_nodes
```

**[참고]** 이 노드를 BT 팩토리에 등록할 때는 다음과 같이 호출합니다:

```cpp
// In your BT node registration file
factory.registerNodeType<amr_bt_nodes::SelectRecoveryStrategyAction>(
  "SelectRecoveryStrategyAction", rclcpp::Node::SharedPtr(node));
```

-----

### 2\. Behavior Tree XML (`IntelligentRecoverySubtree`)

위에서 만든 `SelectRecoveryStrategyAction` 노드를 사용하도록 수정한 BT XML입니다.

```xml
<BehaviorTree ID="IntelligentRecoverySubtree">
  <Sequence name="RecoverySequence">
    <LogTextAction name="RecoveryBanner"
      message="[!!! RECOVERY !!!] Main navigation failed. Initiating intelligent recovery logic."
      interval_s="1.0" />
    
    <Switch name="IntelligentRecoverySwitch" variable="{selected_strategy}">

      <SelectRecoveryStrategyAction name="SelectStrategy"
                                    compute_path_error_code="{compute_path_error_code}"
                                    follow_path_error_code="{follow_path_error_code}"
                                    cfg_err_no_valid="{cfg_err_no_valid}"
                                    cfg_err_start_occ="{cfg_err_start_occ}"
                                    cfg_err_goal_occ="{cfg_err_goal_occ}"
                                    cfg_err_oscillation="{cfg_err_oscillation}"
                                    cfg_err_path_tolerance="{cfg_err_path_tolerance}"
                                    selected_strategy="{selected_strategy}" />

      <Case case="NO_VALID_PATH">
        </Case>

      <Case case="ROBOT_STUCK">
        </Case>

      <Case case="START_OCCUPIED">
        </Case>
      
      <Case case="GOAL_OCCUPIED">
        </Case>

      <Case case="OSCILLATION">
        </Case>

      <Case case="PATH_TOLERANCE_VIOLATED">
        <Sequence name="PathToleranceActions">
          <LogTextAction name="PathToleranceLog" message="[RECOVERY] Case: Path tolerance violated. Re-planning." interval_s="1.0" />
          <SubTree name="ClearPair" ID="ClearCostmapsPairSubtree" _autoremap="true" />
        </Sequence>
      </Case>

      <Default>
        </Default>

    </Switch>
  </Sequence>
</BehaviorTree>
```

이제 이 코드들을 프로젝트에 통합하시면 원하시는 대로 견고하고 예측 가능한 복구 시스템이 완성됩니다.

