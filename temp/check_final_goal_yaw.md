안녕하세요! ROS 2 Jazzy 및 Nav2, BehaviorTree.CPP 환경에 맞춰 요청하신 **Custom Action BT Node (`CheckFinalGoalYaw`)**를 구현해 드립니다.

요청하신 대로 **마지막 Goal**과 **현재 Pose**의 Yaw 차이를 계산하고, 특정 임계값(0.2도) 이내인지 판단하는 로직을 담았습니다. `SyncActionNode`를 상속받아 구현하는 것이 가장 효율적입니다 (연산이 즉시 끝나기 때문입니다).

### 1. 헤더 파일 (`check_final_goal_yaw.hpp`)

파일 경로 예시: `amr_bt_nodes/include/amr_bt_nodes/check_final_goal_yaw.hpp`

```cpp
#ifndef AMR_BT_NODES__CHECK_FINAL_GOAL_YAW_HPP_
#define AMR_BT_NODES__CHECK_FINAL_GOAL_YAW_HPP_

#include <string>
#include <vector>
#include <cmath>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace amr_bt_nodes
{

/**
 * @brief Checks if the yaw difference between the input pose and the last goal
 * is within a tight threshold (0.2 degrees).
 */
class CheckFinalGoalYaw : public BT::SyncActionNode
{
public:
  CheckFinalGoalYaw(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
        "input_goals", "List of goals to check the last one"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "input_pose", "Current pose of the robot"),
      BT::OutputPort<double>(
        "result", "1.0 if within 0.2 degrees, 0.0 otherwise"),
      BT::OutputPort<double>(
        "difference_radian", "Yaw difference in radians (normalized)")
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace amr_bt_nodes

#endif  // AMR_BT_NODES__CHECK_FINAL_GOAL_YAW_HPP_

```

---

### 2. 소스 파일 (`check_final_goal_yaw.cpp`)

파일 경로 예시: `amr_bt_nodes/src/check_final_goal_yaw.cpp`

**핵심 로직 설명:**

1. **Quaternion to Yaw 변환:** `tf2` 라이브러리를 사용하여 쿼터니언에서 Yaw(Euler Z)를 정확히 추출합니다.
2. **각도 정규화 (Normalization):** 각도 차이를 계산할 때 `-π ~ +π` 범위로 정규화하여, 179도와 -179도의 차이가 358도가 아닌 2도로 계산되도록 처리했습니다. (로봇 공학 필수)
3. **Degree 변환 및 비교:** 정규화된 Radian 차이를 Degree로 변환하여 `0.2`도 임계값과 비교합니다.

```cpp
#include "amr_bt_nodes/check_final_goal_yaw.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace amr_bt_nodes
{

CheckFinalGoalYaw::CheckFinalGoalYaw(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::NodeStatus CheckFinalGoalYaw::tick()
{
  std::vector<geometry_msgs::msg::PoseStamped> goals;
  geometry_msgs::msg::PoseStamped current_pose;

  // 1. Input Port 데이터 가져오기
  if (!getInput("input_goals", goals)) {
    RCLCPP_ERROR(rclcpp::get_logger("CheckFinalGoalYaw"), "Missing required input [input_goals]");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("input_pose", current_pose)) {
    RCLCPP_ERROR(rclcpp::get_logger("CheckFinalGoalYaw"), "Missing required input [input_pose]");
    return BT::NodeStatus::FAILURE;
  }

  if (goals.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("CheckFinalGoalYaw"), "Input goals vector is empty");
    return BT::NodeStatus::FAILURE;
  }

  // 2. 마지막 Goal 가져오기
  const auto & final_goal = goals.back();

  // 3. Yaw 계산 (Helper Lambda)
  auto get_yaw = [](const geometry_msgs::msg::Pose & pose) -> double {
    tf2::Quaternion q(
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  };

  double goal_yaw = get_yaw(final_goal.pose);
  double current_yaw = get_yaw(current_pose.pose);

  // 4. 차이 계산 (Goal - Input)
  double diff = goal_yaw - current_yaw;

  // 5. 각도 정규화 (Normalize to -PI ~ +PI)
  // 3.14와 -3.14의 차이가 6.28이 아니라 0에 가깝게 계산되도록 함
  while (diff > M_PI) diff -= 2.0 * M_PI;
  while (diff < -M_PI) diff += 2.0 * M_PI;

  // 6. Output 세팅 (difference_radian)
  setOutput("difference_radian", diff);

  // 7. 조건 검사 (0.0 ~ 0.2 degree)
  // Radian to Degree 변환
  double diff_deg = std::abs(diff * (180.0 / M_PI));
  double result_value = 0.0;

  // degree 기준으로 0.2도 이하인지 확인
  if (diff_deg <= 0.2) {
    result_value = 1.0;
  } else {
    result_value = 0.0;
  }

  setOutput("result", result_value);

  // 디버깅용 로그 (필요시 주석 해제)
  /*
  RCLCPP_INFO(rclcpp::get_logger("CheckFinalGoalYaw"), 
    "Goal Yaw: %.4f, Curr Yaw: %.4f, Diff(rad): %.4f, Diff(deg): %.4f, Result: %.1f", 
    goal_yaw, current_yaw, diff, diff_deg, result_value);
  */

  return BT::NodeStatus::SUCCESS;
}

}  // namespace amr_bt_nodes

// 플러그인 등록 매크로
#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<amr_bt_nodes::CheckFinalGoalYaw>("CheckFinalGoalYaw");
}

```

---

### 3. `CMakeLists.txt` 설정 (참고용)

Nav2 Jazzy 환경에서 빌드하려면 `tf2`, `tf2_geometry_msgs` 의존성이 필요합니다.

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(amr_bt_nodes SHARED
  src/check_final_goal_yaw.cpp
  # ... 다른 소스 파일들
)

target_include_directories(amr_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(amr_bt_nodes
  rclcpp::rclcpp
  behaviortree_cpp::behaviortree_cpp
  geometry_msgs::geometry_msgs
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
)

# 플러그인으로 동작하므로 심볼 강제 로딩 불필요 (보통은 런타임 로드)
ament_export_dependencies(
  rclcpp
  behaviortree_cpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS amr_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()

```

### 4. Behavior Tree XML 예시

Nav2의 XML 파일에서 이 노드를 사용하는 예시입니다.

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <CheckFinalGoalYaw 
          input_goals="{goals}" 
          input_pose="{pose}" 
          result="{yaw_check_result}" 
          difference_radian="{yaw_diff}" />
          
      <BlackboardCheckDouble 
          value_A="{yaw_check_result}" 
          value_B="0.5" 
          return_on_mismatch="FAILURE">
          <AlwaysSuccess/>
      </BlackboardCheckDouble>
      
    </Sequence>
  </BehaviorTree>
</root>

```

이 코드는 요청하신 스펙(Input/Output 타입, Yaw 차이 계산, 0.2도 임계값 체크)을 정확히 따르며, ROS 2 Nav2 환경에서 즉시 사용할 수 있도록 작성되었습니다.
