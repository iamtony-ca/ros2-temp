네, 알겠습니다. 개선된 로직을 반영하여 `hpp` 헤더 파일과 `cpp` 소스 파일을 분리한 전체 코드를 상세한 주석과 함께 제공해 드리겠습니다.

코드를 헤더(.hpp)와 소스(.cpp)로 분리하는 것은 좋은 프로그래밍 습관입니다.

  * **`.hpp` (헤더 파일):** 클래스의 구조와 인터페이스(어떤 함수와 변수가 있는지)를 정의합니다.
  * **`.cpp` (소스 파일):** 헤더에 선언된 함수의 실제 동작 로직을 구현합니다.

이렇게 분리하면 코드의 가독성이 높아지고, 컴파일 시간을 단축하며, 관리가 용이해집니다.

-----

### 1\. 헤더 파일: `is_goals_occupied_condition.hpp`

이 파일은 `IsGoalsOccupiedCondition` 클래스의 선언을 담고 있습니다. 어떤 입력/출력 포트가 있는지, 어떤 함수가 멤버로 있는지 정의합니다.

```cpp
#ifndef AMR_BT_NODES__IS_GOALS_OCCUPIED_CONDITION_HPP_
#define AMR_BT_NODES__IS_GOALS_OCCUPIED_CONDITION_HPP_

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/node.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Forward declaration to reduce header dependency
namespace nav2_costmap_2d
{
class Costmap2DROS;
}

namespace amr_bt_nodes
{

/**
 * @brief BT::ConditionNode to check if any of the provided goals are in an occupied space in the costmap.
 * @details 이 노드는 'goals' 입력 포트로 받은 목표 지점들 중 하나라도
 * costmap 상에서 설정된 임계값(threshold) 이상의 비용을 가지는지 확인합니다.
 * 점유된 목표를 발견하면 SUCCESS를, 모든 목표가 안전하면 FAILURE를 반환합니다.
 */
class IsGoalsOccupiedCondition : public BT::ConditionNode
{
public:
  /**
   * @brief 생성자
   * @param condition_name 노드의 이름
   * @param conf BT 노드 설정 객체
   */
  IsGoalsOccupiedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief 노드가 사용하는 포트들을 정의하는 정적 메서드
   * @return BT::PortsList 포트 목록
   */
  static BT::PortsList providedPorts();

protected:
  /**
   * @brief 노드의 주기적인 실행 로직 (tick)
   * @return BT::NodeStatus 노드 실행 결과 (SUCCESS, FAILURE, RUNNING)
   */
  BT::NodeStatus tick() override;

private:
  // --- 멤버 변수 ---

  // 리소스(node, costmap) 초기화 여부를 나타내는 플래그
  bool initialized_{false};
  
  // ROS 2 노드 핸들. 로깅 및 기타 ROS 기능에 사용.
  rclcpp::Node::SharedPtr node_;
  
  // Nav2 Costmap ROS 래퍼. Costmap 데이터에 접근하는 데 사용.
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
};

}  // namespace amr_bt_nodes

#endif  // AMR_BT_NODES__IS_GOALS_OCCUPIED_CONDITION_HPP_
```

-----

### 2\. 소스 파일: `is_goals_occupied_condition.cpp`

이 파일은 `is_goals_occupied_condition.hpp`에 선언된 함수들의 실제 구현을 담고 있습니다.

```cpp
#include "amr_bt_nodes/is_goals_occupied_condition.hpp"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace amr_bt_nodes
{

// 생성자 구현
IsGoalsOccupiedCondition::IsGoalsOccupiedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  // 실제 리소스 초기화는 tick()에서 지연 로딩(lazy loading) 방식으로 수행
}

// 포트 정의 구현
BT::PortsList IsGoalsOccupiedCondition::providedPorts()
{
  return {
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
      "goals", "Vector of goals to check in the costmap"),
    
    BT::InputPort<int>(
      "occupied_cost_threshold",
      nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, // 기본값: 내접 장애물 비용
      "Cost value to consider a goal as occupied"),
      
    BT::InputPort<bool>(
      "invert_logic", false, "If true, SUCCESS if all goals are free, else FAILURE"),
      
    BT::OutputPort<geometry_msgs::msg::PoseStamped>(
      "occupied_goal", "The first goal found to be occupied")
  };
}

// tick() 메서드 구현 (핵심 로직)
BT::NodeStatus IsGoalsOccupiedCondition::tick()
{
  // --- 1. 초기화 (Lazy Loading) ---
  // 노드가 처음 실행될 때만 ROS 노드와 Costmap 같은 리소스를 블랙보드에서 가져옵니다.
  // 이렇게 하면 BT가 생성되는 시점에 리소스가 없더라도 에러가 발생하지 않습니다.
  if (!initialized_) {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    costmap_ros_ = config().blackboard->get<std::shared_ptr<nav2_costmap_2d::Costmap2DROS>>(
        "global_costmap");

    if (!node_ || !costmap_ros_) {
      auto logger = node_ ? node_->get_logger() : rclcpp::get_logger("IsGoalsOccupiedCondition");
      RCLCPP_ERROR(logger, "Failed to get required resources ('node', 'global_costmap') from blackboard");
      return BT::NodeStatus::FAILURE;
    }
    initialized_ = true;
  }

  // --- 2. 입력 포트에서 데이터 읽기 ---
  std::vector<geometry_msgs::msg::PoseStamped> goals;
  if (!getInput<std::vector<geometry_msgs::msg::PoseStamped>>("goals", goals) || goals.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Input 'goals' is empty or not provided. Returning FAILURE.");
    return BT::NodeStatus::FAILURE;
  }

  int threshold;
  getInput("occupied_cost_threshold", threshold);
  
  bool invert_logic;
  getInput("invert_logic", invert_logic);

  // --- 3. 핵심 로직: Costmap 점유 상태 확인 ---
  auto costmap = costmap_ros_->getCostmap();
  // Costmap에 여러 스레드가 동시에 접근하는 것을 방지하기 위해 뮤텍스 잠금
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  bool any_goal_occupied = false;
  for (const auto & goal : goals) {
    unsigned int mx, my;

    // worldToMap: 월드 좌표를 맵 셀 좌표로 변환. 맵 밖에 있으면 false 반환.
    // getCost: 해당 맵 셀의 비용(0~255)을 반환.
    // 조건: 맵을 벗어났거나, 비용이 임계값 이상이면 '점유'된 것으로 간주.
    if (!costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my) || 
        costmap->getCost(mx, my) >= threshold) 
    {
      any_goal_occupied = true;
      // 점유된 첫 번째 목표를 출력 포트를 통해 블랙보드에 기록
      setOutput("occupied_goal", goal);
      RCLCPP_INFO(
        node_->get_logger(), 
        "Goal (%.2f, %.2f) is considered occupied.", goal.pose.position.x, goal.pose.position.y);
      // 점유된 목표를 하나라도 찾았으므로 더 이상 검사할 필요 없이 반복 종료
      break; 
    }
  }

  lock.unlock(); // 뮤텍스 잠금 해제

  // --- 4. 최종 결과 반환 ---
  // invert_logic 값에 따라 최종 결과(SUCCESS/FAILURE)를 결정
  if (invert_logic) {
    // 로직 반전 모드: '모든' 목표가 안전해야 SUCCESS
    // any_goal_occupied가 true이면(점유된게 있으면) FAILURE, false이면 SUCCESS
    return any_goal_occupied ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  } else {
    // 기본 모드: '하나라도' 점유되었다면 SUCCESS
    // any_goal_occupied가 true이면 SUCCESS, false이면 FAILURE
    return any_goal_occupied ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
}

}  // namespace amr_bt_nodes
```

이 두 파일을 `amr_bt_nodes` 패키지의 `include/amr_bt_nodes/`와 `src/` 디렉터리에 각각 위치시키고 `CMakeLists.txt`에 `is_goals_occupied_condition.cpp` 파일을 빌드 타겟에 추가하면 됩니다.


```

```






네, 그럼요. 커스텀 BT 노드를 Nav2 시스템에 정식으로 등록하기 위한 **플러그인 XML**과, 이 노드를 실제 Behavior Tree에서 어떻게 활용하는지 보여주는 **사용 예시 BT XML**을 모두 제공해 드리겠습니다.

-----

### 1\. 플러그인 등록 XML

이 파일은 ROS 2의 `pluginlib` 시스템에 우리가 만든 BT 노드를 알려주는 역할을 합니다. Nav2의 BT Navigator는 이 파일을 통해 실행 시점에 커스텀 노드가 포함된 라이브러리를 동적으로 로드할 수 있습니다.

**파일 이름:** `amr_bt_nodes_plugins.xml` (원하는 이름으로 지정 가능)
**위치:** 패키지의 루트 디렉터리 (예: `your_ros2_ws/src/amr_bt_nodes_package/`)

```xml
<library path="amr_bt_nodes">
  <class name="IsGoalsOccupiedCondition"
         type="amr_bt_nodes::IsGoalsOccupiedCondition"
         base_class_type="BT::ConditionNode">
    <description>
      Checks if any of the provided goals are in an occupied space.
    </description>
  </class>

</library>
```

#### **플러그인을 시스템에 알리기 위한 추가 설정**

위 XML 파일이 ROS 2 시스템에 의해 발견되려면 `package.xml`과 `CMakeLists.txt`에 다음 내용을 추가해야 합니다.

1.  **`package.xml` 에 추가:**
    `<export>` 태그 내부에 플러그인 XML 파일의 위치를 알려줍니다.

    ```xml
    <export>
      <build_type>ament_cmake</build_type>
      <behaviortree_cpp plugin_xml="amr_bt_nodes_plugins.xml"/>
    </export>
    ```

2.  **`CMakeLists.txt` 에 추가:**
    `pluginlib`이 XML 파일을 찾을 수 있도록 설치(install) 과정에 등록합니다.

    ```cmake
    # ... (find_package 등 다른 내용) ...

    # 플러그인 XML 파일을 설치
    install(
      FILES amr_bt_nodes_plugins.xml
      DESTINATION share/${PROJECT_NAME}
    )

    # ... (ament_package() 앞에 위치) ...
    ```

-----

### 2\. 사용 예시 BT XML

이 예시는 `ComputePathToPose`를 실행하기 전에, 목표 지점이 점유되었는지 **미리 확인**하는 간단하고 실용적인 시나리오를 보여줍니다.

**파일 이름:** `Maps_to_pose_with_precheck.xml`

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="1" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithPrecheck">
        <CreateGoalsVector goal="{goal}" goals="{goals}"/>

        <IsGoalsOccupiedCondition 
            name="IsGoalSafe"
            goals="{goals}"
            invert_logic="true"
            occupied_cost_threshold="253"/>

        <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
        <FollowPath path="{path}" controller_id="FollowPath"/>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <ClearEntireCostmap name="ClearCostmap" service_name="global_costmap/clear_entirely_global_costmap"/>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

#### **예시 XML 설명**

1.  **`CreateGoalsVector`**: Nav2에 기본적으로 포함된 BT 노드입니다. 블랙보드의 단일 목표(`goal`)를 벡터 형태(`goals`)로 변환하여 우리 커스텀 노드의 입력 형식에 맞춰줍니다.
2.  **`IsGoalsOccupiedCondition`**:
      * `invert_logic="true"`: 이 BT는 "목표가 안전할 때" 경로 계획을 계속 진행해야 합니다. 우리 노드는 기본적으로 "점유되었을 때" `SUCCESS`를 반환하므로, 로직을 반전시켜 "안전할 때(`FAILURE` 조건)" `SUCCESS`를 반환하도록 만들었습니다. 이것이 `PipelineSequence`가 올바르게 동작하게 하는 핵심입니다.
      * `name="IsGoalSafe"`: BT 모니터링 도구(Groot 등)에서 노드를 쉽게 식별할 수 있도록 별칭을 부여했습니다.
3.  **`PipelineSequence`**: 이 제어 노드는 자식 노드들을 순서대로 실행합니다. 만약 `IsGoalsOccupiedCondition`이 `FAILURE`를 반환하면 (즉, 목표가 점유되었다면), 뒤따르는 `ComputePathToPose`와 `FollowPath`는 실행되지 않고 시퀀스가 즉시 중단됩니다.
4.  **`RecoveryNode`**: 전체 네비게이션 시퀀스가 실패했을 경우(예: 목표 지점이 점유되어 경로 계획 실패), 복구 행동(`ReactiveFallback`)을 한 번 시도하도록 합니다. 이 예시에서는 전역 Costmap을 초기화하는 것을 복구 전략으로 사용했습니다.

