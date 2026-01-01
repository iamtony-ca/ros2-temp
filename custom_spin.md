Nav2의 **Spin Action Server**(`nav2_behaviors`)와 **Spin Action Client**(`nav2_behavior_tree`)의 기능을 하나로 합친 **`CustomSpinAction` BT Node**를 작성해 드립니다.

이 노드는 Action 통신을 사용하지 않고, BT 노드 내부에서 직접 `cmd_vel`을 계산하고 `collision_check`를 수행하여 제어 메시지를 발행합니다.

### 특징 및 구조

1. **Inheritance**: `BT::StatefulActionNode`를 상속받아 비동기(Async)로 동작합니다.
2. **Direct Control**: `cmd_vel`을 직접 발행합니다.
3. **Collision Checking**: `nav2_costmap_2d`의 `CostmapSubscriber`를 사용하여 로컬 코스트맵 토픽을 구독하고, 미래 경로를 시뮬레이션하여 충돌을 감지합니다.
4. **Parameter Handling**: 기존 Spin 플러그인과 동일하게 `min_rotational_vel` 등의 파라미터를 ROS 파라미터 서버(보통 `bt_navigator` 노드)에서 가져옵니다.

---

### 1. 헤더 파일 (`custom_spin_action.hpp`)

```cpp
#ifndef AMR_BT_NODES__CUSTOM_SPIN_ACTION_HPP_
#define AMR_BT_NODES__CUSTOM_SPIN_ACTION_HPP_

#include <string>
#include <memory>
#include <cmath>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"

namespace amr_bt_nodes
{

/**
 * @brief Custom Spin Action Node that directly controls the robot
 * merging functionality of SpinAction Client and Spin Behavior Server.
 */
class CustomSpinAction : public BT::StatefulActionNode
{
public:
  CustomSpinAction(
    const std::string & name,
    const BT::NodeConfiguration & config);

  ~CustomSpinAction() override = default;

  // BT 노드 초기화 및 포트 정의
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("spin_dist", 1.57, "Spin distance in radians"),
      BT::InputPort<double>("time_allowance", 10.0, "Allowed time for spinning"),
      BT::InputPort<bool>("is_recovery", true, "True if recovery (for logging only)"),
      BT::OutputPort<int>("error_code_id", "The spin behavior error code")
    };
  }

  // 노드 실행 시작 시 1회 호출
  BT::NodeStatus onStart() override;

  // 노드가 RUNNING 상태일 때 반복 호출 (메인 제어 루프)
  BT::NodeStatus onRunning() override;

  // 노드가 중단되었을 때 호출 (Preempt)
  void onHalted() override;

private:
  // 내부 동작 헬퍼 함수
  void stopRobot();
  bool isCollisionFree(
    const double & relative_yaw,
    const geometry_msgs::msg::Twist & cmd_vel,
    geometry_msgs::msg::Pose2D & pose2d);
  
  // 파라미터 로드
  void getParameters();

  // ROS 관련 멤버 변수
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  
  // 충돌 감지 관련 멤버 변수 (Simulation 방식)
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::CostmapSubscriber>> collision_checker_;

  // 제어 변수 (Spin Logic)
  double min_rotational_vel_;
  double max_rotational_vel_;
  double rotational_acc_lim_;
  double cmd_yaw_;
  double prev_yaw_;
  double relative_yaw_;
  double simulate_ahead_time_;
  
  std::string robot_base_frame_;
  std::string costmap_frame_;
  double transform_tolerance_;
  double cycle_frequency_; // Hz (Loop rate estimation)

  rclcpp::Time end_time_;
  rclcpp::Duration command_time_allowance_{0, 0};
  bool is_recovery_;
};

}  // namespace amr_bt_nodes

#endif  // AMR_BT_NODES__CUSTOM_SPIN_ACTION_HPP_

```

---

### 2. 소스 파일 (`custom_spin_action.cpp`)

```cpp
#include "amr_bt_nodes/custom_spin_action.hpp"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

using namespace std::chrono_literals;

namespace amr_bt_nodes
{

CustomSpinAction::CustomSpinAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config),
  min_rotational_vel_(0.4),
  max_rotational_vel_(1.0),
  rotational_acc_lim_(3.2),
  cmd_yaw_(0.0),
  prev_yaw_(0.0),
  relative_yaw_(0.0),
  simulate_ahead_time_(2.0),
  cycle_frequency_(10.0)
{
  // 1. Blackboard에서 ROS Node 및 TF 가져오기
  if (!config.blackboard->get("node", node_)) {
    throw std::runtime_error("Missing 'node' in blackboard");
  }
  if (!config.blackboard->get("tf_buffer", tf_)) {
    throw std::runtime_error("Missing 'tf_buffer' in blackboard");
  }

  // 2. 파라미터 로드
  getParameters();

  // 3. Publisher 생성
  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  // 4. Collision Checker 초기화 (Local Costmap Topic 구독)
  // 주의: 토픽 이름은 시스템 구성에 따라 다를 수 있음 (기본값: local_costmap/costmap_raw)
  std::string costmap_topic = "local_costmap/costmap_raw";
  std::string footprint_topic = "local_costmap/published_footprint";

  costmap_sub_ = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
    node_, costmap_topic);
  footprint_sub_ = std::make_shared<nav2_costmap_2d::FootprintSubscriber>(
    node_, footprint_topic, *tf_);
  
  collision_checker_ = std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::CostmapSubscriber>>(
    costmap_sub_, footprint_sub_, "CustomSpin");
}

void CustomSpinAction::getParameters()
{
  // 기존 Spin 플러그인과 동일한 파라미터 이름을 사용하여 호환성 유지
  nav2_util::declare_parameter_if_not_declared(node_, "spin.simulate_ahead_time", rclcpp::ParameterValue(2.0));
  node_->get_parameter("spin.simulate_ahead_time", simulate_ahead_time_);

  nav2_util::declare_parameter_if_not_declared(node_, "spin.max_rotational_vel", rclcpp::ParameterValue(1.0));
  node_->get_parameter("spin.max_rotational_vel", max_rotational_vel_);

  nav2_util::declare_parameter_if_not_declared(node_, "spin.min_rotational_vel", rclcpp::ParameterValue(0.4));
  node_->get_parameter("spin.min_rotational_vel", min_rotational_vel_);

  nav2_util::declare_parameter_if_not_declared(node_, "spin.rotational_acc_lim", rclcpp::ParameterValue(3.2));
  node_->get_parameter("spin.rotational_acc_lim", rotational_acc_lim_);
  
  // 기본 프레임 설정
  nav2_util::declare_parameter_if_not_declared(node_, "robot_base_frame", rclcpp::ParameterValue("base_link"));
  node_->get_parameter("robot_base_frame", robot_base_frame_);

  nav2_util::declare_parameter_if_not_declared(node_, "transform_tolerance", rclcpp::ParameterValue(0.1));
  node_->get_parameter("transform_tolerance", transform_tolerance_);
}

BT::NodeStatus CustomSpinAction::onStart()
{
  // 1. Input Port 읽기
  if (!getInput("spin_dist", cmd_yaw_)) {
    RCLCPP_ERROR(node_->get_logger(), "CustomSpinAction: spin_dist missing");
    return BT::NodeStatus::FAILURE;
  }
  
  double time_allowance_sec;
  getInput("time_allowance", time_allowance_sec);
  command_time_allowance_ = rclcpp::Duration::from_seconds(time_allowance_sec);
  end_time_ = node_->now() + command_time_allowance_;

  getInput("is_recovery", is_recovery_); // 로그용 혹은 추후 확장용

  // 2. 초기 Pose 및 Yaw 획득
  geometry_msgs::msg::PoseStamped current_pose;
  // Local Frame이 없으므로 Global(map) 혹은 Odom 프레임 기준으로 현재 각도만 알면 됨.
  // 여기서는 편의상 odom 프레임을 사용하거나, costmap_sub_에서 프레임 ID를 가져옴.
  costmap_frame_ = costmap_sub_->getCostmap()->header.frame_id; // 보통 odom

  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, costmap_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_ERROR(node_->get_logger(), "CustomSpinAction: Failed to get current pose");
    setOutput("error_code_id", 101); // TF Error Code
    return BT::NodeStatus::FAILURE;
  }

  prev_yaw_ = tf2::getYaw(current_pose.pose.orientation);
  relative_yaw_ = 0.0;

  RCLCPP_INFO(node_->get_logger(), "CustomSpinAction: Start spinning %.2f rad", cmd_yaw_);
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CustomSpinAction::onRunning()
{
  // 1. 시간 초과 확인
  rclcpp::Duration time_remaining = end_time_ - node_->now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN(node_->get_logger(), "CustomSpinAction: Timeout");
    setOutput("error_code_id", 201); // Timeout Error Code
    return BT::NodeStatus::FAILURE;
  }

  // 2. 현재 Pose 및 Yaw 갱신
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, costmap_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_ERROR(node_->get_logger(), "CustomSpinAction: Failed to get pose in loop");
    return BT::NodeStatus::FAILURE;
  }

  const double current_yaw = tf2::getYaw(current_pose.pose.orientation);
  double delta_yaw = current_yaw - prev_yaw_;

  // 각도 정규화 (Normalize to -PI ~ PI)
  if (std::abs(delta_yaw) > M_PI) {
    delta_yaw = std::copysign(2 * M_PI - std::abs(delta_yaw), prev_yaw_);
  }

  relative_yaw_ += delta_yaw;
  prev_yaw_ = current_yaw;

  // 3. 목표 도달 확인
  double remaining_yaw = std::abs(cmd_yaw_) - std::abs(relative_yaw_);
  if (remaining_yaw < 1e-4) { // Tolerance
    stopRobot();
    setOutput("error_code_id", 0); // SUCCESS
    return BT::NodeStatus::SUCCESS;
  }

  // 4. 속도 계산 (Trapezoidal Velocity Profile: v = sqrt(2*a*s))
  // [수정 제안 적용: 미세 구간 P-Control]
  double vel = 0.0;
  double smooth_threshold = 0.1; // 0.1 rad 이하에서는 부드럽게

  if (remaining_yaw < smooth_threshold) {
    vel = remaining_yaw * 2.0; // Gain tuning 필요
  } else {
    vel = std::sqrt(2 * rotational_acc_lim_ * remaining_yaw);
  }

  // 속도 제한 (Clamp)
  vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

  // 5. Cmd Vel 메시지 생성
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  // 부호(Sign) 적용: 목표 방향에 맞춰 회전
  cmd_vel->angular.z = std::copysign(vel, cmd_yaw_); 

  // 6. 충돌 감지 (Simulation Check)
  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.pose.position.x;
  pose2d.y = current_pose.pose.position.y;
  pose2d.theta = current_yaw;

  if (!isCollisionFree(relative_yaw_, *cmd_vel, pose2d)) {
    stopRobot();
    RCLCPP_WARN(node_->get_logger(), "CustomSpinAction: Collision Detected ahead!");
    setOutput("error_code_id", 102); // Collision Error Code
    return BT::NodeStatus::FAILURE;
  }

  // 7. 명령 발행
  vel_pub_->publish(std::move(cmd_vel));

  return BT::NodeStatus::RUNNING;
}

void CustomSpinAction::onHalted()
{
  stopRobot();
  RCLCPP_INFO(node_->get_logger(), "CustomSpinAction: Halted");
}

void CustomSpinAction::stopRobot()
{
  geometry_msgs::msg::Twist stop_cmd;
  stop_cmd.linear.x = 0.0;
  stop_cmd.angular.z = 0.0;
  vel_pub_->publish(stop_cmd);
}

bool CustomSpinAction::isCollisionFree(
  const double & relative_yaw,
  const geometry_msgs::msg::Twist & cmd_vel,
  geometry_msgs::msg::Pose2D & pose2d)
{
  // Nav2 Spin.cpp의 로직을 그대로 구현
  int cycle_count = 0;
  double sim_position_change;
  const int max_cycle_count = static_cast<int>(cycle_frequency_ * simulate_ahead_time_);
  geometry_msgs::msg::Pose2D init_pose = pose2d;
  
  // Costmap Update가 필요할 때만 Lock을 걸기 위해 fetch_data 사용 안함 (Subscriber 방식은 항상 최신)
  // FootprintCollisionChecker가 내부적으로 costmap lock을 처리함

  while (cycle_count < max_cycle_count) {
    sim_position_change = cmd_vel.angular.z * (cycle_count / cycle_frequency_);
    pose2d.theta = init_pose.theta + sim_position_change;
    cycle_count++;

    // 이미 회전한 거리보다 시뮬레이션 거리가 짧으면 패스 (의미상)
    // 원본 로직: 목표 각도 넘어서는지 체크인듯 함
    if (std::abs(relative_yaw) - std::abs(sim_position_change) <= 0.) {
      break;
    }

    // FootprintCollisionChecker 사용
    // CostmapSubscriber를 통해 현재 코스트맵을 가져와서 충돌 체크
    if (collision_checker_->inCollision(
        pose2d.x, pose2d.y, pose2d.theta, collision_checker_->getFootprint(), *costmap_sub_->getCostmap())) 
    {
      return false;
    }
  }
  return true;
}

}  // namespace amr_bt_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<amr_bt_nodes::CustomSpinAction>("CustomSpinAction");
}

```

### 3. CMakeLists.txt 설정 (필수 의존성)

이 코드는 `nav2_costmap_2d`의 API를 직접 사용하므로 빌드 설정에 추가해야 합니다.

```cmake
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
# ... 기타 rclcpp, tf2 등

add_library(amr_bt_nodes SHARED
  src/custom_spin_action.cpp
  # ...
)

target_link_libraries(amr_bt_nodes
  nav2_costmap_2d::nav2_costmap_2d
  nav2_util::nav2_util
  # ...
)

```

### 구현 요약

1. **Stateful Node**: `onStart`, `onRunning`, `onHalted` 구조로 구현하여 BT 틱마다 제어 루프가 돕니다.
2. **Costmap Access**: `CostmapSubscriber`를 생성하여 `/local_costmap/costmap_raw` 토픽을 구독합니다. 서버 없이도 독립적으로 충돌 체크가 가능합니다.
3. **Simulation**: Nav2 `Spin` 플러그인의 `isCollisionFree` 로직을 그대로 가져왔으며, `FootprintCollisionChecker`를 사용하여 미래의 로봇 자세(`pose2d`)에서 충돌 여부를 검사합니다.
4. **Parameter**: `bt_navigator` 노드의 파라미터로 `spin.min_rotational_vel` 등을 선언하고 읽어옵니다.

이 코드를 사용하면 별도의 Action Server 통신 오버헤드 없이 BT 내부에서 즉각적인 Spin 제어가 가능합니다.
##############################################  
#####################################################3  
######################################################3  
네, 개발하신 **Custom BT Nodes** (`CheckFinalGoalYaw`, `CustomSpinAction`)를 실제 ROS 2 Nav2 환경에서 빌드하고 실행하기 위한 전체 패키지 구성을 작성해 드립니다.

패키지 이름은 **`amr_bt_nodes`**라고 가정하겠습니다.

---

### 1. `package.xml`

ROS 2 의존성을 정의합니다. `nav2_costmap_2d`와 `nav2_util`이 필수입니다.

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>amr_bt_nodes</name>
  <version>0.0.1</version>
  <description>Custom Behavior Tree Nodes for AMR Navigation</description>
  <maintainer email="user@todo.todo">User</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>behaviortree_cpp</depend>
  <depend>geometry_msgs</depend>
  <depend>nav2_util</depend>
  <depend>nav2_costmap_2d</depend>
  <depend>nav2_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>

```

---

### 2. `CMakeLists.txt`

빌드 설정 파일입니다. 라이브러리를 생성하고 설치 경로를 지정합니다.

```cmake
cmake_minimum_required(VERSION 3.8)
project(amr_bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# Create Shared Library
add_library(${PROJECT_NAME} SHARED
  src/check_final_goal_yaw.cpp
  src/custom_spin_action.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  behaviortree_cpp::behaviortree_cpp
  geometry_msgs::geometry_msgs
  nav2_util::nav2_util
  nav2_costmap_2d::nav2_costmap_2d
  nav2_msgs::nav2_msgs
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
)

# Install Rules
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  behaviortree_cpp
  geometry_msgs
  nav2_util
  nav2_costmap_2d
)

ament_package()

```

---

### 3. `nav2_params.yaml` (중요)

`bt_navigator`가 커스텀 플러그인을 로드하도록 설정해야 합니다.
**`plugin_lib_names`**에 우리가 만든 라이브러리(`amr_bt_nodes`)를 추가하고, `CustomSpinAction`이 사용할 파라미터를 정의합니다.

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    
    # [중요 1] 커스텀 플러그인 라이브러리 등록
    # CMakeLists.txt에서 add_library로 만든 이름과 동일해야 함
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      # ... (기타 기본 노드들) ...
      - nav2_spin_action_bt_node
      - nav2_remove_passed_goals_action_bt_node
      - nav2_goal_checker_selector_bt_node
      - amr_bt_nodes  # <--- 우리가 만든 패키지의 라이브러리 이름

    # [중요 2] CustomSpinAction 파라미터 설정
    # CustomSpinAction.cpp의 getParameters()에서 읽어오는 값들
    spin:
      simulate_ahead_time: 2.0
      max_rotational_vel: 0.5    # 안전을 위해 낮춤
      min_rotational_vel: 0.1    # 오버슈트 방지를 위해 낮춤
      rotational_acc_lim: 0.05   # 매우 부드러운 정지를 위해 낮춤
    
    # Costmap Subscriber가 사용할 토픽 이름 확인 필요
    # CustomSpinAction 소스 코드 내에서 "local_costmap/costmap_raw"를 구독하므로
    # 로컬 코스트맵 노드 이름과 일치해야 함.

local_costmap:
  local_costmap:
    ros__parameters:
      # ... (기존 코스트맵 설정) ...

```

---

### 4. `bt.xml`

이제 Action Server를 통하지 않고, 직접 제어하는 `CustomSpinAction`을 사용하는 BT XML입니다.

```xml
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <Sequence>
            <FollowPath path="{path}" controller_id="{selected_controller}" 
                        goal_checker_id="general_goal_checker" 
                        error_code_id="{follow_path_error_code}"/>
            
            <GetCurrentPose output_pose="{pose}"/>
            
            <CheckFinalGoalYaw 
                  input_goals="{goals}" 
                  input_pose="{pose}" 
                  result="{yaw_check_result}" 
                  difference_radian="{yaw_diff}" />

            <Precondition name="SmootherPreconditionManeuver"
                  if="yaw_check_result == 0.0" else="SUCCESS">
              
              <CustomSpinAction 
                  spin_dist="{yaw_diff}" 
                  time_allowance="5.0"
                  is_recovery="false" 
                  error_code_id="{spin_error_code}"/>
                  
            </Precondition>
          </Sequence>
          
          <Sequence>
            <WouldAControllerRecoveryHelp error_code="{follow_path_error_code}"/>
            <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
          </Sequence>
        </RecoveryNode>
      </PipelineSequence>
      
      </RecoveryNode>
  </BehaviorTree>
</root>

```

### 5. 실행 전 체크리스트

1. **빌드:** `colcon build --packages-select amr_bt_nodes`
2. **환경 설정:** `source install/setup.bash`
3. **라이브러리 로드:** `bt_navigator` 실행 로그에 `[amr_bt_nodes] Loaded plugin...` 과 같은 메시지가 뜨거나 에러가 없는지 확인하세요.
4. **토픽 이름:** `CustomSpinAction.cpp` 코드 안에 하드코딩된 토픽 이름(`local_costmap/costmap_raw`)이 실제 실행 중인 로컬 코스트맵 토픽과 일치하는지 `ros2 topic list`로 꼭 확인하세요. 만약 다르다면 소스 코드에서 수정하거나 파라미터화해야 합니다.
