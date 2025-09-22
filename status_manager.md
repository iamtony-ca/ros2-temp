네, 알겠습니다. `Maps_through_poses` 액션을 지원하도록 기존 코드를 완벽하게 수정하고 통합하겠습니다.

`Maps_to_pose` (단일 목표), `follow_waypoints` (경로점 추종), `Maps_through_poses` (경유지 내비게이션)는 서로 다른 최상위 내비게이션 작업입니다. 이들을 모두 올바르게 처리하고, 특히 `FOLLOWING_WAYPOINTS` 상태를 이 두 가지 경로점 추종 액션에 맞게 통합하여 전체 로직을 업데이트했습니다.

아래는 `/navigate_through_poses/_action/status` 토픽 구독을 추가하고 로직에 완벽히 반영한 최종 코드입니다.

-----

### `status_manager.hpp` (수정)

`Maps_through_poses`를 위한 구독, 콜백, 상태 플래그, Goal ID 추적 변수를 추가합니다.

```cpp
#pragma once

// ... (includes and enum are the same as your provided code) ...
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/msg/behavior_tree_log.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include <string>
#include <mutex>
#include <optional>
#include <vector>
#include <map>
#include <atomic>
#include <set>

enum class RobotStatus : uint8_t {
    IDLE,
    RECEIVED_GOAL,
    PLANNING,
    DRIVING,
    FOLLOWING_WAYPOINTS,
    PAUSED,
    COLLISION_IMMINENT,
    RECOVERY_FAILURE,
    RECOVERY_RUNNING,
    RECOVERY_SUCCESS,
    SUCCEEDED,
    FAILED,
    CANCELED
};

class StatusManager : public rclcpp::Node
{
public:
    using GoalStatusArray = action_msgs::msg::GoalStatusArray;
    using BehaviorTreeLog = nav2_msgs::msg::BehaviorTreeLog;

    explicit StatusManager(const rclcpp::NodeOptions & options);

private:
    // --- 주요 로직 ---
    void evaluate_and_publish_if_changed();
    RobotStatus determine_current_status(RobotStatus last_known_status);

    // --- 콜백 함수 ---
    void timer_callback();
    void nav_to_pose_status_callback(const GoalStatusArray::SharedPtr msg);
    void waypoints_status_callback(const GoalStatusArray::SharedPtr msg);
    void nav_through_poses_status_callback(const GoalStatusArray::SharedPtr msg); // *** 추가 ***
    void compute_path_status_callback(const GoalStatusArray::SharedPtr msg);
    void follow_path_status_callback(const GoalStatusArray::SharedPtr msg);
    void bt_log_callback(const BehaviorTreeLog::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void collision_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void pause_flag_callback(const std_msgs::msg::Bool::SharedPtr msg);

    // --- 유틸리티 및 초기화 함수 ---
    void query_initial_node_states();
    std::string status_to_string(RobotStatus status);
    bool is_bt_node_running(const std::string& node_name) const;

    // --- 상태 저장을 위한 멤버 변수 ---
    std::recursive_mutex status_mutex_;
    RobotStatus current_status_{RobotStatus::IDLE};
    std::atomic<bool> are_core_nodes_active_{false};
    std::atomic<bool> is_robot_stopped_{true};
    std::atomic<bool> is_collision_imminent_{false};
    std::atomic<bool> is_paused_{false};

    // 액션 상태 플래그
    std::atomic<bool> is_nav_executing_{false};
    std::atomic<bool> is_waypoints_executing_{false};
    std::atomic<bool> is_nav_through_poses_executing_{false}; // *** 추가 ***
    std::atomic<bool> is_planning_sub_action_executing_{false};
    std::atomic<bool> is_driving_sub_action_executing_{false};
    
    // BT 로그 및 복구 상태
    std::vector<std::string> running_bt_nodes_;
    std::atomic<bool> is_in_recovery_context_{false};
    
    // 각 액션별 Goal ID 및 최종 상태
    std::optional<action_msgs::msg::GoalStatus> latest_terminal_status_;
    std::optional<unique_identifier_msgs::msg::UUID> active_nav_goal_id_;
    std::optional<unique_identifier_msgs::msg::UUID> active_waypoints_goal_id_;
    std::optional<unique_identifier_msgs::msg::UUID> active_nav_through_poses_goal_id_; // *** 추가 ***

    // --- ROS 인터페이스 ---
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::Subscription<GoalStatusArray>::SharedPtr nav_to_pose_status_sub_;
    rclcpp::Subscription<GoalStatusArray>::SharedPtr waypoints_status_sub_;
    rclcpp::Subscription<GoalStatusArray>::SharedPtr nav_through_poses_status_sub_; // *** 추가 ***
    rclcpp::Subscription<GoalStatusArray>::SharedPtr compute_path_status_sub_;
    rclcpp::Subscription<GoalStatusArray>::SharedPtr follow_path_status_sub_;
    // ... (나머지 인터페이스는 동일) ...
    rclcpp::Subscription<BehaviorTreeLog>::SharedPtr bt_log_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> lifecycle_clients_;
    std::vector<std::string> core_node_names_ = { "bt_navigator", "planner_server", "controller_server", "amcl" };
};
```

-----

### `status_manager.cpp` (수정)

생성자에 구독을 추가하고, 새로운 콜백 함수를 구현하며, 메인 결정 로직에 `FOLLOWING_WAYPOINTS`를 통합합니다.

```cpp
#include "robot_monitoring/status_manager.hpp"
#include "action_msgs/msg/goal_status.hpp"
#include <thread>

using GoalStatus = action_msgs::msg::GoalStatus;

// UUID 비교를 위한 헬퍼 함수 (cpp 파일 상단에 배치)
bool is_same_goal_id(const unique_identifier_msgs::msg::UUID& id1, const unique_identifier_msgs::msg::UUID& id2) {
    return std::equal(std::begin(id1.uuid), std::end(id1.uuid), std::begin(id2.uuid));
}

StatusManager::StatusManager(const rclcpp::NodeOptions & options)
: Node("status_manager", options)
{
    status_publisher_ = this->create_publisher<std_msgs::msg::String>("/robot_status", rclcpp::SystemDefaultsQoS());
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&StatusManager::timer_callback, this));

    nav_to_pose_status_sub_ = this->create_subscription<GoalStatusArray>(
        "/navigate_to_pose/_action/status", 10, std::bind(&StatusManager::nav_to_pose_status_callback, this, std::placeholders::_1));
    waypoints_status_sub_ = this->create_subscription<GoalStatusArray>(
        "/follow_waypoints/_action/status", 10, std::bind(&StatusManager::waypoints_status_callback, this, std::placeholders::_1));
    
    // *** 추가: navigate_through_poses 구독 ***
    nav_through_poses_status_sub_ = this->create_subscription<GoalStatusArray>(
        "/navigate_through_poses/_action/status", 10, std::bind(&StatusManager::nav_through_poses_status_callback, this, std::placeholders::_1));

    compute_path_status_sub_ = this->create_subscription<GoalStatusArray>(
        "/compute_path_to_pose/_action/status", 10, std::bind(&StatusManager::compute_path_status_callback, this, std::placeholders::_1));
    // ... (나머지 생성자 코드는 동일) ...
    follow_path_status_sub_ = this->create_subscription<GoalStatusArray>(
        "/follow_path/_action/status", 10, std::bind(&StatusManager::follow_path_status_callback, this, std::placeholders::_1));
    bt_log_sub_ = this->create_subscription<BehaviorTreeLog>(
        "/behavior_tree_log", 10, std::bind(&StatusManager::bt_log_callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&StatusManager::odom_callback, this, std::placeholders::_1));
    collision_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/is_collision_imminent", rclcpp::SystemDefaultsQoS(), std::bind(&StatusManager::collision_callback, this, std::placeholders::_1));
    pause_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/pause_robot", rclcpp::SystemDefaultsQoS(), std::bind(&StatusManager::pause_flag_callback, this, std::placeholders::_1));

    for (const auto& name : core_node_names_) {
        lifecycle_clients_.push_back(this->create_client<lifecycle_msgs::srv::GetState>(name + "/get_state"));
    }
    std::thread{ [this]() { this->query_initial_node_states(); }}.detach();
    RCLCPP_INFO(this->get_logger(), "StatusManager Node has been started.");
}

RobotStatus StatusManager::determine_current_status(RobotStatus last_known_status)
{
    // 1. 최우선: 안전/명령 오버라이드
    if (is_collision_imminent_.load()) return RobotStatus::COLLISION_IMMINENT;
    if (is_paused_.load()) return RobotStatus::PAUSED;

    // 2. 진행 중인 작업: navigate_to_pose
    if (is_nav_executing_.load()) {
        // ... (내부 로직은 이전과 동일) ...
        if (is_driving_sub_action_executing_.load()) return RobotStatus::DRIVING;
        if (is_planning_sub_action_executing_.load()) return RobotStatus::PLANNING;
        if (last_known_status == RobotStatus::PLANNING || last_known_status == RobotStatus::DRIVING) {
            return last_known_status;
        }
        if (is_bt_node_running("NavigateWithReplanning")) return RobotStatus::RECEIVED_GOAL;
        return RobotStatus::RECEIVED_GOAL;
    }

    // *** 수정: 3. 진행 중인 작업: follow_waypoints 또는 navigate_through_poses ***
    if (is_waypoints_executing_.load() || is_nav_through_poses_executing_.load()) {
        // 두 액션 모두 FOLLOWING_WAYPOINTS 상태로 통합
        return RobotStatus::FOLLOWING_WAYPOINTS;
    }

    // 4. 작업 종료 상태
    if (latest_terminal_status_.has_value()) {
        // ... (내부 로직은 이전과 동일) ...
        auto status_val = latest_terminal_status_->status;
        latest_terminal_status_.reset();
        switch (status_val) {
            case GoalStatus::STATUS_SUCCEEDED: return RobotStatus::SUCCEEDED;
            case GoalStatus::STATUS_CANCELED: return RobotStatus::CANCELED;
            case GoalStatus::STATUS_ABORTED: return RobotStatus::FAILED;
        }
    }
    
    // 5. 기본 상태: IDLE
    if (are_core_nodes_active_.load() && is_robot_stopped_.load()) {
        return RobotStatus::IDLE;
    }

    // 6. 폴백
    if (!are_core_nodes_active_.load()) {
        return RobotStatus::FAILED;
    }
    return last_known_status;
}

// *** 추가: navigate_through_poses_status_callback 구현 ***
void StatusManager::nav_through_poses_status_callback(const GoalStatusArray::SharedPtr msg)
{
    bool was_active = is_nav_through_poses_executing_.load();
    bool is_active_now = false;
    std::optional<action_msgs::msg::GoalStatus> final_status;

    if (!active_nav_through_poses_goal_id_.has_value()) {
        for (const auto& status : msg->status_list) {
            if (status.status == GoalStatus::STATUS_EXECUTING) {
                is_active_now = true;
                std::lock_guard<std::recursive_mutex> lock(status_mutex_);
                active_nav_through_poses_goal_id_ = status.goal_info.goal_id;
                break;
            }
        }
    } else {
        is_active_now = true;
    }

    if (active_nav_through_poses_goal_id_.has_value()) {
        for (const auto& status : msg->status_list) {
            if (is_same_goal_id(status.goal_info.goal_id, active_nav_through_poses_goal_id_.value())) {
                if (status.status == GoalStatus::STATUS_CANCELED) {
                    final_status = status;
                    break;
                }
                if (status.status == GoalStatus::STATUS_SUCCEEDED || status.status == GoalStatus::STATUS_ABORTED) {
                    final_status = status;
                }
            }
        }
    }

    if (final_status.has_value()) {
        is_active_now = false;
        std::lock_guard<std::recursive_mutex> lock(status_mutex_);
        latest_terminal_status_ = final_status;
        active_nav_through_poses_goal_id_.reset();
    }

    if (was_active != is_active_now) {
        is_nav_through_poses_executing_.store(is_active_now);
    }
    
    if (!was_active && is_active_now) {
        std::lock_guard<std::recursive_mutex> lock(status_mutex_);
        latest_terminal_status_.reset();
    }

    evaluate_and_publish_if_changed();
}


// *** 수정: waypoints_status_callback 에도 Goal ID 추적 로직 추가 ***
void StatusManager::waypoints_status_callback(const GoalStatusArray::SharedPtr msg)
{
    bool was_active = is_waypoints_executing_.load();
    bool is_active_now = false;
    std::optional<action_msgs::msg::GoalStatus> final_status;

    if (!active_waypoints_goal_id_.has_value()) {
        for (const auto& status : msg->status_list) {
            if (status.status == GoalStatus::STATUS_EXECUTING) {
                is_active_now = true;
                std::lock_guard<std::recursive_mutex> lock(status_mutex_);
                active_waypoints_goal_id_ = status.goal_info.goal_id;
                break;
            }
        }
    } else {
        is_active_now = true;
    }

    if (active_waypoints_goal_id_.has_value()) {
        for (const auto& status : msg->status_list) {
            if (is_same_goal_id(status.goal_info.goal_id, active_waypoints_goal_id_.value())) {
                if (status.status == GoalStatus::STATUS_CANCELED) {
                    final_status = status;
                    break;
                }
                if (status.status == GoalStatus::STATUS_SUCCEEDED || status.status == GoalStatus::STATUS_ABORTED) {
                    final_status = status;
                }
            }
        }
    }

    if (final_status.has_value()) {
        is_active_now = false;
        std::lock_guard<std::recursive_mutex> lock(status_mutex_);
        latest_terminal_status_ = final_status;
        active_waypoints_goal_id_.reset();
    }

    if (was_active != is_active_now) {
        is_waypoints_executing_.store(is_active_now);
    }
    
    if (!was_active && is_active_now) {
        std::lock_guard<std::recursive_mutex> lock(status_mutex_);
        latest_terminal_status_.reset();
    }

    evaluate_and_publish_if_changed();
}


// ... (나머지 함수들은 이전과 동일하게 유지됩니다) ...
// nav_to_pose_status_callback, timer_callback, evaluate_and_publish_if_changed,
// compute_path_status_callback, follow_path_status_callback, bt_log_callback,
// odom_callback, collision_callback, pause_flag_callback, 
// query_initial_node_states, is_bt_node_running, status_to_string
```





네, 정확히 맞습니다. 현재 코드의 `std::recursive_mutex`가 데이터의 동시 접근을 막아주는 보호장치 역할을 하므로 데이터가 깨지는 문제는 발생하지 않습니다. 사용자의 분석이 정확합니다.

하지만 여기서 \*\*콜백 그룹(Callback Group)\*\*을 사용하는 것은 ROS 2의 실행기(Executor)에게 "이 콜백들을 어떻게 실행할지" 명확한 규칙을 알려주어 코드를 더 안정적이고 예측 가능하게 만드는 모범 사례(Best Practice)입니다.

현재 코드에 **단일 `MutuallyExclusive` 콜백 그룹**을 적용하여 동시성 이슈를 원천적으로 차단하고, 놓치는 토픽 없이 모든 이벤트를 순차적으로 안전하게 처리하도록 코드를 다시 수정해 드리겠습니다.

### 수정 사항

1.  `MutuallyExclusive` 콜백 그룹을 하나 생성합니다.
2.  모든 구독(`Subscription`)과 타이머(`Timer`)를 이 그룹에 할당합니다.
3.  **효과:** `MultiThreadedExecutor`를 사용하더라도, ROS 2는 이 그룹에 속한 콜백들을 **절대로 동시에 실행하지 않고, 하나씩 순서대로 처리**합니다. 따라서 뮤텍스가 없어도 콜백 실행 순서로 인한 레이스 컨디션이 발생하지 않습니다. (물론, 만일을 대비해 뮤텍스를 유지하는 것이 더 안전합니다.)

-----

### `status_manager.hpp` (수정)

단일 콜백 그룹을 위한 멤버 변수를 추가합니다.

```cpp
#pragma once

// ... (includes and enum are the same as your provided code) ...
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
// ...

class StatusManager : public rclcpp::Node
{
public:
    // ... (using declarations are the same) ...
    explicit StatusManager(const rclcpp::NodeOptions & options);

private:
    // ... (function declarations are the same) ...

    // --- 상태 저장을 위한 멤버 변수 ---
    std::recursive_mutex status_mutex_;
    // ... (other member variables are the same) ...

    // --- ROS 인터페이스 ---
    rclcpp::CallbackGroup::SharedPtr callback_group_; // *** FIX: 콜백 그룹 포인터 추가 ***
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    // ... (rest of the interface declarations are the same) ...
};
```

### `status_manager.cpp` (수정)

생성자에서 콜백 그룹을 생성하고 모든 구독과 타이머에 할당합니다.

```cpp
#include "robot_monitoring/status_manager.hpp"
#include "action_msgs/msg/goal_status.hpp"
#include <thread>

// ... (is_same_goal_id helper function and using declarations are the same) ...

StatusManager::StatusManager(const rclcpp::NodeOptions & options)
: Node("status_manager", options)
{
    // *** FIX: 단일 MutuallyExclusive 콜백 그룹 생성 ***
    // 이 그룹에 속한 모든 콜백은 한 번에 하나씩만 순차적으로 실행됩니다.
    callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // 모든 구독에 적용할 옵션에 콜백 그룹을 할당합니다.
    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = callback_group_;

    // ROS 인터페이스 초기화
    status_publisher_ = this->create_publisher<std_msgs::msg::String>("/robot_status", rclcpp::SystemDefaultsQoS());
    
    // *** FIX: 타이머에 콜백 그룹 할당 ***
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), 
        std::bind(&StatusManager::timer_callback, this), 
        callback_group_);

    // *** FIX: 모든 구독 생성 시 sub_options를 통해 콜백 그룹 할당 ***
    nav_to_pose_status_sub_ = this->create_subscription<GoalStatusArray>(
        "/navigate_to_pose/_action/status", 10, std::bind(&StatusManager::nav_to_pose_status_callback, this, std::placeholders::_1), sub_options);
    
    waypoints_status_sub_ = this->create_subscription<GoalStatusArray>(
        "/follow_waypoints/_action/status", 10, std::bind(&StatusManager::waypoints_status_callback, this, std::placeholders::_1), sub_options);
    
    nav_through_poses_status_sub_ = this->create_subscription<GoalStatusArray>(
        "/navigate_through_poses/_action/status", 10, std::bind(&StatusManager::nav_through_poses_status_callback, this, std::placeholders::_1), sub_options);

    compute_path_status_sub_ = this->create_subscription<GoalStatusArray>(
        "/compute_path_to_pose/_action/status", 10, std::bind(&StatusManager::compute_path_status_callback, this, std::placeholders::_1), sub_options);
    
    follow_path_status_sub_ = this->create_subscription<GoalStatusArray>(
        "/follow_path/_action/status", 10, std::bind(&StatusManager::follow_path_status_callback, this, std::placeholders::_1), sub_options);
    
    bt_log_sub_ = this->create_subscription<BehaviorTreeLog>(
        "/behavior_tree_log", 10, std::bind(&StatusManager::bt_log_callback, this, std::placeholders::_1), sub_options);
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&StatusManager::odom_callback, this, std::placeholders::_1), sub_options);
    
    collision_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/is_collision_imminent", rclcpp::SystemDefaultsQoS(), std::bind(&StatusManager::collision_callback, this, std::placeholders::_1), sub_options);
    
    pause_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/pause_robot", rclcpp::SystemDefaultsQoS(), std::bind(&StatusManager::pause_flag_callback, this, std::placeholders::_1), sub_options);

    // Lifecycle 클라이언트는 콜백이 아니므로 그룹 설정이 필요 없습니다.
    for (const auto& name : core_node_names_) {
        lifecycle_clients_.push_back(this->create_client<lifecycle_msgs::srv::GetState>(name + "/get_state"));
    }
    
    std::thread{ [this]() { this->query_initial_node_states(); }}.detach();
    RCLCPP_INFO(this->get_logger(), "StatusManager Node has been started with a MutuallyExclusive CallbackGroup.");
}

// ... (나머지 모든 함수의 구현은 이전 답변과 완전히 동일합니다) ...
```

이제 이 코드는 ROS 2 실행기 레벨에서 모든 이벤트(토픽 메시지, 타이머)를 순차적으로 처리하도록 보장받습니다. 말씀하신 대로 토픽을 놓치는 일 없이, 들어온 순서대로 하나씩 안전하게 처리하게 됩니다.

##########################  

좋아요—전체적으로 구조는 잘 잡았고, Nav2 액션/BT 상태를 한데 모아 “한 줄짜리 상태”로 내보내는 목적에 부합합니다. 다만 실제 운용에서 틈새가 몇 군데 있어요. 아래는 “바로 체감되는” 것부터 순서대로 정리했어요. (짤막한 패치 코드 포함)

---

# 핵심 개선 포인트 (High-impact)

1. ## 액션 goal 추적: `EXECUTING`만 보면 놓칩니다

지금 `active_*_goal_id_`를 `STATUS_EXECUTING`에서만 설정합니다. 하지만 Nav2 goal은 `ACCEPTED → EXECUTING → (CANCELING) → {SUCCEEDED|CANCELED|ABORTED}`로 흐르며, `EXECUTING`으로 가지 못하고 바로 `ABORTED`나 `CANCELED`되는 케이스가 있습니다. 이러면 최종 상태를 영영 놓칠 수 있어요.
액션 상태 상수는 `action_msgs/GoalStatus`에 정의되어 있고(예: `STATUS_ACCEPTED=1`, `STATUS_EXECUTING=2`, `STATUS_CANCELED=5`, `STATUS_ABORTED=6`), 상태 스트림은 모든 액션 서버에 `/<action_name>/_action/status`로 노출됩니다. ([docs.ros2.org][1])

**개선 패치(요지):**

* `ACCEPTED`에서도 `active_*_goal_id_`를 잡도록 확장
* 여러 goal 동시 존재 시 최신(goal\_info.stamp 가 가장 최신) goal만 “활성”으로 간주
* `CANCELING`은 활성 상태 유지로 보되, `CANCELED/ABORTED/SUCCEEDED`가 뜨면 즉시 종료 처리

```cpp
// 공통 유틸 (헤더/소스 적절히 배치)
static bool is_active_status(int8_t s) {
  using GS = action_msgs::msg::GoalStatus;
  return s == GS::STATUS_ACCEPTED || s == GS::STATUS_EXECUTING || s == GS::STATUS_CANCELING;
}
static bool is_terminal_status(int8_t s) {
  using GS = action_msgs::msg::GoalStatus;
  return s == GS::STATUS_SUCCEEDED || s == GS::STATUS_CANCELED || s == GS::STATUS_ABORTED;
}
static bool newer(const builtin_interfaces::msg::Time& a, const builtin_interfaces::msg::Time& b){
  return (a.sec > b.sec) || (a.sec == b.sec && a.nanosec > b.nanosec);
}

// 예: nav_to_pose_status_callback() 안의 골자만 교체
void StatusManager::nav_to_pose_status_callback(const GoalStatusArray::SharedPtr msg) {
  using GS = action_msgs::msg::GoalStatus;
  bool is_active_now = false;
  std::optional<GS> final_status;

  // 1) 최신 goal 선택(ACCEPTED/EXECUTING/CANCELING 중에서 stamp 가장 최신)
  std::optional<unique_identifier_msgs::msg::UUID> newest_goal;
  builtin_interfaces::msg::Time newest_stamp{};
  for (const auto& s : msg->status_list) {
    if (is_active_status(s.status) && (!newest_goal || newer(s.goal_info.stamp, newest_stamp))) {
      newest_goal = s.goal_info.goal_id;
      newest_stamp = s.goal_info.stamp;
    }
  }
  if (newest_goal) {
    is_active_now = true;
    if (!active_nav_goal_id_ || !is_same_goal_id(*newest_goal, *active_nav_goal_id_)) {
      std::lock_guard<std::recursive_mutex> lk(status_mutex_);
      active_nav_goal_id_ = newest_goal;
      latest_terminal_status_.reset(); // 새 goal 시작 → 이전 종결 플래그 클리어
    }
  }

  // 2) 추적 중 goal의 최종 상태 탐지(최우선 CANCELED)
  if (active_nav_goal_id_) {
    for (const auto& s : msg->status_list) {
      if (!is_same_goal_id(s.goal_info.goal_id, *active_nav_goal_id_)) continue;
      if (s.status == GS::STATUS_CANCELED) { final_status = s; break; }
      if (s.status == GS::STATUS_SUCCEEDED || s.status == GS::STATUS_ABORTED) final_status = s;
    }
  }

  if (final_status) {
    is_active_now = false;
    std::lock_guard<std::recursive_mutex> lk(status_mutex_);
    latest_terminal_status_ = final_status;
    active_nav_goal_id_.reset();
  }

  is_nav_executing_.store(is_active_now);
  evaluate_and_publish_if_changed();
}
```

> 참고: `GoalInfo`에 `stamp`가 있어 최신 goal 판별에 쓰기 좋습니다. ([docs.ros2.org][2])

---

2. ## BT 런타임 상태 추적: “증분 로그”를 **스냅샷**처럼 쓰면 오검출

`/behavior_tree_log`의 `event_log`는 “그 틱에서 바뀐 노드들만” 실립니다. 지금처럼 콜백마다 `running_bt_nodes_.clear()` 후 `RUNNING`만 담으면, 한 번 `RUNNING`으로 진입한 노드가 다음 틱에 변화가 없을 때 목록에서 사라져 “안 돌고 있다”고 오판합니다. `BehaviorTreeStatusChange` 메시지의 필드는 문자열 `"IDLE|RUNNING|SUCCESS|FAILURE"`이고, 이건 “상태 변경 이벤트”임을 암시합니다. ([ROS Documentation][3])

**개선 패치(요지):**

* `unordered_map<std::string, std::string> bt_states_`로 **누적 상태맵** 유지
* 이번 이벤트로 바뀐 노드만 갱신, `RUNNING` 여부는 맵을 조회
* `is_bt_node_running()`는 맵 기반으로 변경

```cpp
// 멤버로 교체
std::unordered_map<std::string, std::string> bt_states_;

void StatusManager::bt_log_callback(const BehaviorTreeLog::SharedPtr msg) {
  std::lock_guard<std::recursive_mutex> lk(status_mutex_);
  bool recovery_changed = false;
  RobotStatus new_status = current_status_;

  static const std::unordered_set<std::string> recovery_seq = {
    "ShortRecoverySequence1","ShortRecoverySequence2","ShortRecoverySequenceTotal1"
  };

  for (const auto& ev : msg->event_log) {
    bt_states_[ev.node_name] = ev.current_status; // 누적 업데이트

    if (recovery_seq.count(ev.node_name)) {
      if (ev.current_status == "RUNNING")      { new_status = RobotStatus::RECOVERY_RUNNING; recovery_changed = true; }
      else if (ev.current_status == "SUCCESS") { new_status = RobotStatus::RECOVERY_SUCCESS; recovery_changed = true; }
      else if (ev.current_status == "FAILURE") { new_status = RobotStatus::RECOVERY_FAILURE; recovery_changed = true; }
    }
  }

  if (recovery_changed && current_status_ != new_status) {
    current_status_ = new_status;
    std_msgs::msg::String out; out.data = status_to_string(current_status_);
    RCLCPP_INFO(get_logger(), "Robot Status Changed (by Recovery BT) -> %s", out.data.c_str());
    status_publisher_->publish(out);
  }

  // 필요 시: evaluate( ) 호출
  evaluate_and_publish_if_changed();
}

bool StatusManager::is_bt_node_running(const std::string& node_name) const {
  auto it = bt_states_.find(node_name);
  return it != bt_states_.end() && it->second == "RUNNING";
}
```

---

3. ## `RECEIVED_GOAL` 판정은 BT 노드명 대신 **액션 상태**에 의존

지금은 `NavigateWithReplanning` 노드가 RUNNING이면 `RECEIVED_GOAL`로 분류하는데, 트리 이름/구조가 바뀌면 깨집니다. 액션 상태 배열에서 `STATUS_ACCEPTED`가 등장하면 그 순간을 `RECEIVED_GOAL`로 두는 편이 이식성 높고 정확합니다. 상수 정의는 위 문서 참고. ([docs.ros2.org][1])

---

4. ## 락 구조 단순화(재귀 뮤텍스 제거)

여러 콜백에서 `status_mutex_` 잡은 채 `evaluate_and_publish_if_changed()`를 다시 호출해서 **재귀 락**이 필요해졌습니다.
권장 패턴은:

* “상태 플래그 갱신”은 (원자/짧은 구간)에서만 잠깐 잠그거나 `std::atomic`으로 처리
* `evaluate_and_publish_if_changed()` 내부만 **단일 지점**에서 락 획득
* 콜백에서는 락 없이 `evaluate...` 호출

이렇게 바꾸면 `std::mutex`로 충분하고 데드락/우발적 재귀 진입 위험이 줄어듭니다.

---

5. ## 퍼블리시 정책: 이벤트 + 하트비트 구분 & QoS 개선

지금은 타이머가 1초마다 무조건 동일 상태를 퍼블리시합니다. 구독자 입장에선 중복 스팸일 수 있어요.

* 상태 변경 시에는 즉시 퍼블리시 (지금처럼)
* 하트비트는 **옵션**으로 별도 토픽(`/robot_status/heartbeat`) or 동일 토픽에 **주기 N초로만**
* QoS는 “마지막 상태를 새 구독자가 즉시 받도록” `transient_local` 권장

```cpp
auto qos = rclcpp::QoS(1).transient_local().reliable();
status_publisher_ = create_publisher<std_msgs::msg::String>("/robot_status", qos);
```

---

6. ## 라이프사이클 감시: detatch thread 지양 + 동적 갱신

`query_initial_node_states()`를 detach 스레드에서 한 번만 평가하면, 이후 노드가 죽거나 재활성화돼도 `are_core_nodes_active_`가 업데이트되지 않습니다. 또한 노드 종료 시 detach 스레드가 남아있을 수 있어요.

* **대안 A:** `/<node>/transition_event`(lifecycle\_msgs/TransitionEvent) 구독으로 활성/비활성 상태를 지속 반영(초기 상태는 `GetState` 서비스로 1회 질의) ([docs.ros2.org][4])
* **대안 B:** Nav2 LifecycleManager 클라이언트를 써서 매 N초 `is_active()` 폴링 (간단) ([api.nav2.org][5])

둘 중 하나로 옮기고, detach 대신 타이머 기반 폴링을 추천합니다.

---

7. ## IDLE 판정 안정화(속도 저주파 필터 & 파라미터화)

`|v|<0.01 && |w|<0.01` 한 방이면 노이즈/양자화로 튀어요. 간단한 지수평활(EMA)나 0.2–0.3s 히스테리시스 홀드, 임계값 파라미터화를 권장.

```cpp
// 파라미터로 노출
double stop_lin_thresh_{0.02}, stop_ang_thresh_{0.02}, stop_hold_sec_{0.3};
rclcpp::Time last_motion_time_;
void StatusManager::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
  const auto now = now();
  const bool moving = std::fabs(msg->twist.twist.linear.x)  > stop_lin_thresh_ ||
                      std::fabs(msg->twist.twist.angular.z) > stop_ang_thresh_;
  if (moving) last_motion_time_ = now;
  is_robot_stopped_.store((now - last_motion_time_).seconds() > stop_hold_sec_);
}
```

---

# 기타 권장사항 (Nice-to-have)

* **상태 토픽 타입**: 문자열 대신 `uint8`(enum) 또는 커스텀 msg(문자열+코드)로 내보내면 소비측 파싱이 견고해집니다.
* **리커버리 표시 유지시간**: `RECOVERY_*`는 너무 순간적일 수 있으니 0.3–0.5s 유지(타임스탬프로 만료)하면 UI/로그 가독성↑.
* **토픽 이름 파라미터화**: `bt_log_topic`(기본 `behavior_tree_log`)를 파라미터로—일부 설정은 `bt_navigator` 네임스페이스가 붙습니다. (BT 로그 메시지 정의는 nav2\_msgs에 문서화) ([ROS Documentation][6])
* **컴포넌트화**: 컨테이너 사용 시 `rclcpp_components` 등록 매크로 추가.
* **헤더 누락**: `unique_identifier_msgs/msg/uuid.hpp` 명시 include 권장.

---

# `determine_current_status()` 로직에 대한 짧은 제안

* 우선순위는 그대로 두되, `RECEIVED_GOAL` 판단을 “액션에 `ACCEPTED` 존재” 여부로 바꾸세요.
* `FOLLOWING_WAYPOINTS` 상태는 `is_waypoints_executing_`이 true일 때 우선 적용 (하위 플래그는 동일하게 PLANNING/DRIVING로 override).
* `latest_terminal_status_`는 **읽었을 때만 reset**하는 현재 방식이 적절합니다.

---

필요하면 위 패치를 반영한 정리 버전(hpp/cpp)로도 뽑아줄게요. 참고 문서 몇 개 남겨둡니다:

* `BehaviorTreeLog` / `BehaviorTreeStatusChange` 메시지 정의(상태 문자열) ([ROS Documentation][6])
* 액션 상태 상수(`GoalStatus`), `GoalInfo.stamp`, `_action/status` 모니터링 관례 ([docs.ros2.org][1])
* Nav2 BT 개요/기본 트리 문서(트리 이름 의존 위험성 참고) ([docs.nav2.org][7])
* Lifecycle 이벤트/서비스(TransitionEvent/GetState, LifecycleManager client) ([docs.ros2.org][8])

원하시면 “파라미터 선언 + QoS + 타이머 기반 라이프사이클 폴링”까지 포함한 최소 예제도 바로 만들어 드릴게요.

[1]: https://docs.ros2.org/foxy/api/action_msgs/msg/GoalStatus.html "msg/GoalStatus Documentation"
[2]: https://docs.ros2.org/foxy/api/action_msgs/msg/GoalInfo.html?utm_source=chatgpt.com "action_msgs/msg/GoalInfo Message"
[3]: https://docs.ros.org/en/iron/p/nav2_msgs/interfaces/msg/BehaviorTreeStatusChange.html "BehaviorTreeStatusChange — nav2_msgs 1.2.10 documentation"
[4]: https://docs.ros2.org/foxy/api/lifecycle_msgs/msg/TransitionEvent.html?utm_source=chatgpt.com "lifecycle_msgs/msg/TransitionEvent Message"
[5]: https://api.nav2.org/nav2-jazzy/html/classnav2__lifecycle__manager_1_1LifecycleManagerClient.html?utm_source=chatgpt.com "nav2_lifecycle_manager LifecycleManagerClient"
[6]: https://docs.ros.org/en/iron/p/nav2_msgs/interfaces/msg/BehaviorTreeLog.html "BehaviorTreeLog — nav2_msgs 1.2.10 documentation"
[7]: https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html?utm_source=chatgpt.com "Behavior-Tree Navigator"
[8]: https://docs.ros2.org/foxy/api/lifecycle_msgs/index-msg.html?utm_source=chatgpt.com "lifecycle_msgs Message / Service / Action Documentation"


##################  
좋아. 요청대로 **A방식(콜백은 Reentrant 유지, 공유 상태 접근은 모두 mutex로 보호)** 을 반영해서, `RECEIVED_GOAL` 원샷 트리거 + 콜백 그룹 + 레이스 방지까지 모두 포함한 **완성본 hpp/cpp**를 아래에 정리했어. 그대로 빌드해서 쓰면 된다.

---

### `status_manager.hpp`

```cpp
#pragma once

#include <atomic>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>
#include <set>
#include <unordered_map>
#include <algorithm>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/msg/goal_status.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/msg/behavior_tree_log.hpp"

#include "lifecycle_msgs/srv/get_state.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

enum class RobotStatus : uint8_t {
  IDLE,
  RECEIVED_GOAL,
  PLANNING,
  DRIVING,
  FOLLOWING_WAYPOINTS,
  PAUSED,
  COLLISION_IMMINENT,
  RECOVERY_FAILURE,
  RECOVERY_RUNNING,
  RECOVERY_SUCCESS,
  SUCCEEDED,
  FAILED,
  CANCELED
};

class StatusManager : public rclcpp::Node
{
public:
  using GoalStatusArray = action_msgs::msg::GoalStatusArray;
  using BehaviorTreeLog = nav2_msgs::msg::BehaviorTreeLog;

  explicit StatusManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // === 주기/평가 ===
  void timer_callback();
  void evaluate_and_publish_if_changed();
  RobotStatus determine_current_status(RobotStatus last_known_status);

  // === 콜백 ===
  void nav_to_pose_status_callback(const GoalStatusArray::SharedPtr msg);
  void waypoints_status_callback(const GoalStatusArray::SharedPtr msg);
  void nav_through_poses_status_callback(const GoalStatusArray::SharedPtr msg);
  void compute_path_poses_status_callback(const GoalStatusArray::SharedPtr msg);
  void compute_path_status_callback(const GoalStatusArray::SharedPtr msg);
  void follow_path_status_callback(const GoalStatusArray::SharedPtr msg);
  void bt_log_callback(const BehaviorTreeLog::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void collision_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void pause_flag_callback(const std_msgs::msg::Bool::SharedPtr msg);

  // === 유틸 ===
  void query_initial_node_states();
  std::string status_to_string(RobotStatus status);
  bool is_bt_node_running(const std::string& node_name) const;
  inline bool is_terminal_robot_status(RobotStatus s) const {
    return s == RobotStatus::IDLE || s == RobotStatus::SUCCEEDED ||
           s == RobotStatus::FAILED || s == RobotStatus::CANCELED;
  }

  // === 동기화/상태 (모든 공유 상태는 status_mutex_로 보호) ===
  mutable std::mutex status_mutex_;
  RobotStatus current_status_{RobotStatus::IDLE};

  bool are_core_nodes_active_{false};
  bool is_robot_stopped_{true};
  bool is_collision_imminent_{false};
  bool is_paused_{false};

  bool is_nav_executing_{false};
  bool is_waypoints_executing_{false};
  bool is_nav_through_poses_executing_{false};
  bool is_planning_sub_action_executing_{false};
  bool is_driving_sub_action_executing_{false};

  std::vector<std::string> running_bt_nodes_;
  bool is_in_recovery_context_{false};

  std::optional<action_msgs::msg::GoalStatus> latest_terminal_status_;
  std::optional<unique_identifier_msgs::msg::UUID> active_nav_goal_id_;
  std::optional<unique_identifier_msgs::msg::UUID> active_waypoints_goal_id_;
  std::optional<unique_identifier_msgs::msg::UUID> active_nav_through_poses_goal_id_;

  // === RECEIVED_GOAL 원샷 트리거 ===
  bool received_goal_armed_{true};   // 터미널 상태에서 arm
  bool bt_log_seen_in_cycle_{false}; // 디버깅용

  // === ROS I/F ===
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;

  rclcpp::Subscription<GoalStatusArray>::SharedPtr nav_to_pose_status_sub_;
  rclcpp::Subscription<GoalStatusArray>::SharedPtr waypoints_status_sub_;
  rclcpp::Subscription<GoalStatusArray>::SharedPtr nav_through_poses_status_sub_;
  rclcpp::Subscription<GoalStatusArray>::SharedPtr compute_path_poses_status_sub_;
  rclcpp::Subscription<GoalStatusArray>::SharedPtr compute_path_status_sub_;
  rclcpp::Subscription<GoalStatusArray>::SharedPtr follow_path_status_sub_;
  rclcpp::Subscription<BehaviorTreeLog>::SharedPtr bt_log_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> lifecycle_clients_;
  std::vector<std::string> core_node_names_ = {
    "bt_navigator", "planner_server", "controller_server", "amcl"
  };

  // === 콜백 그룹 ===
  rclcpp::CallbackGroup::SharedPtr cbg_bt_;
  rclcpp::CallbackGroup::SharedPtr cbg_actions_; // Reentrant (A방식) + mutex 보호
  rclcpp::CallbackGroup::SharedPtr cbg_misc_;
  rclcpp::CallbackGroup::SharedPtr cbg_clients_;
};
```

---

### `status_manager.cpp`

```cpp
#include "robot_monitoring/status_manager.hpp"

using GoalStatus = action_msgs::msg::GoalStatus;

static bool is_same_goal_id(const unique_identifier_msgs::msg::UUID& id1,
                            const unique_identifier_msgs::msg::UUID& id2)
{
  return std::equal(id1.uuid.begin(), id1.uuid.end(), id2.uuid.begin());
}

StatusManager::StatusManager(const rclcpp::NodeOptions & options)
: Node("status_manager", options)
{
  // === 콜백 그룹 ===
  cbg_bt_      = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbg_actions_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);  // A방식
  cbg_misc_    = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  cbg_clients_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // === 퍼블리셔 (상태 캐시) ===
  auto status_qos = rclcpp::QoS(1).reliable().transient_local();
  status_publisher_ = this->create_publisher<std_msgs::msg::String>("/robot_status", status_qos);

  // === 타이머 ===
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&StatusManager::timer_callback, this),
    cbg_misc_
  );

  // === 서브스크립션 옵션 ===
  rclcpp::SubscriptionOptions so_actions; so_actions.callback_group = cbg_actions_;
  rclcpp::SubscriptionOptions so_bt;     so_bt.callback_group      = cbg_bt_;
  rclcpp::SubscriptionOptions so_misc;   so_misc.callback_group    = cbg_misc_;

  // === 액션 상태 토픽 ===
  nav_to_pose_status_sub_ = this->create_subscription<GoalStatusArray>(
    "/navigate_to_pose/_action/status", 10,
    std::bind(&StatusManager::nav_to_pose_status_callback, this, std::placeholders::_1),
    so_actions);

  waypoints_status_sub_ = this->create_subscription<GoalStatusArray>(
    "/follow_waypoints/_action/status", 10,
    std::bind(&StatusManager::waypoints_status_callback, this, std::placeholders::_1),
    so_actions);

  nav_through_poses_status_sub_ = this->create_subscription<GoalStatusArray>(
    "/navigate_through_poses/_action/status", 10,
    std::bind(&StatusManager::nav_through_poses_status_callback, this, std::placeholders::_1),
    so_actions);

  compute_path_poses_status_sub_ = this->create_subscription<GoalStatusArray>(
    "/compute_path_through_poses/_action/status", 10,
    std::bind(&StatusManager::compute_path_poses_status_callback, this, std::placeholders::_1),
    so_actions);

  compute_path_status_sub_ = this->create_subscription<GoalStatusArray>(
    "/compute_path_to_pose/_action/status", 10,
    std::bind(&StatusManager::compute_path_status_callback, this, std::placeholders::_1),
    so_actions);

  follow_path_status_sub_ = this->create_subscription<GoalStatusArray>(
    "/follow_path/_action/status", 10,
    std::bind(&StatusManager::follow_path_status_callback, this, std::placeholders::_1),
    so_actions);

  // === BT 로그 ===
  bt_log_sub_ = this->create_subscription<BehaviorTreeLog>(
    "/behavior_tree_log", 10,
    std::bind(&StatusManager::bt_log_callback, this, std::placeholders::_1),
    so_bt);

  // === 기타 ===
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&StatusManager::odom_callback, this, std::placeholders::_1),
    so_misc);

  collision_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/is_collision_imminent", rclcpp::SystemDefaultsQoS(),
    std::bind(&StatusManager::collision_callback, this, std::placeholders::_1),
    so_misc);

  pause_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/pause_robot", rclcpp::SystemDefaultsQoS(),
    std::bind(&StatusManager::pause_flag_callback, this, std::placeholders::_1),
    so_misc);

  // === 라이프사이클 상태 질의 클라이언트 ===
  for (const auto& name : core_node_names_) {
    auto client = this->create_client<lifecycle_msgs::srv::GetState>(
      name + "/get_state",
      rmw_qos_profile_services_default,
      cbg_clients_);
    lifecycle_clients_.push_back(client);
  }

  // 초기 활성 여부 확인 (별도 스레드)
  std::thread{ [this]() { this->query_initial_node_states(); } }.detach();

  RCLCPP_INFO(this->get_logger(), "StatusManager Node has been started.");
}

// === 타이머: 현 상태 주기 퍼블리시 ===
void StatusManager::timer_callback()
{
  std_msgs::msg::String msg;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    msg.data = status_to_string(current_status_);
  }
  status_publisher_->publish(msg);
}

// === 상태 평가 & 변경 시 퍼블리시 ===
void StatusManager::evaluate_and_publish_if_changed()
{
  std::optional<std_msgs::msg::String> to_pub;

  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    RobotStatus new_status = determine_current_status(current_status_);

    // 터미널 상태로 돌아오면 RECEIVED_GOAL 트리거 재-무장
    if (is_terminal_robot_status(new_status)) {
      received_goal_armed_ = true;
      bt_log_seen_in_cycle_ = false;
    }

    if (new_status != current_status_) {
      current_status_ = new_status;
      std_msgs::msg::String s; s.data = status_to_string(current_status_);
      RCLCPP_INFO(this->get_logger(), "Robot Status Changed -> %s", s.data.c_str());
      to_pub = s;
    }
  }

  if (to_pub) status_publisher_->publish(*to_pub);
}

// === 상태 결정 ===
RobotStatus StatusManager::determine_current_status(RobotStatus last_known_status)
{
  // 공유 상태 읽기 전부 락 안에서 호출되므로 race 없음
  if (is_collision_imminent_) return RobotStatus::COLLISION_IMMINENT;
  if (is_paused_)             return RobotStatus::PAUSED;

  if (is_nav_executing_) {
    if (last_known_status == RobotStatus::RECOVERY_RUNNING) return last_known_status;
    if (is_driving_sub_action_executing_)  return RobotStatus::DRIVING;
    if (is_planning_sub_action_executing_) return RobotStatus::PLANNING;
    if (last_known_status == RobotStatus::PLANNING || last_known_status == RobotStatus::DRIVING)
      return last_known_status;
  }

  if (is_waypoints_executing_ || is_nav_through_poses_executing_) {
    if (last_known_status == RobotStatus::RECOVERY_RUNNING) return last_known_status;
    if (is_driving_sub_action_executing_)  return RobotStatus::DRIVING;
    if (is_planning_sub_action_executing_) return RobotStatus::PLANNING;
    if (last_known_status == RobotStatus::PLANNING || last_known_status == RobotStatus::DRIVING)
      return last_known_status;
    // 필요시 FOLLOWING_WAYPOINTS로 별도 분리 가능
  }

  if (latest_terminal_status_.has_value()) {
    auto s = latest_terminal_status_->status;
    latest_terminal_status_.reset();
    switch (s) {
      case GoalStatus::STATUS_SUCCEEDED: return RobotStatus::SUCCEEDED;
      case GoalStatus::STATUS_CANCELED:  return RobotStatus::CANCELED;
      case GoalStatus::STATUS_ABORTED:   return RobotStatus::FAILED;
      default: break;
    }
  }

  if (are_core_nodes_active_ && is_robot_stopped_)
    return RobotStatus::IDLE;

  if (!are_core_nodes_active_) return RobotStatus::FAILED;
  return last_known_status;
}

// === 액션 상태 콜백들 (A방식: 공유 상태 전체 mutex 보호) ===

void StatusManager::nav_through_poses_status_callback(const GoalStatusArray::SharedPtr msg)
{
  bool was_active;
  {
    std::lock_guard<std::mutex> lk(status_mutex_);
    was_active = is_nav_through_poses_executing_;
  }

  bool is_active_now = false;
  std::optional<action_msgs::msg::GoalStatus> final_status;

  {
    std::lock_guard<std::mutex> lk(status_mutex_);
    if (!active_nav_through_poses_goal_id_) {
      for (const auto& status : msg->status_list) {
        if (status.status == GoalStatus::STATUS_EXECUTING) {
          is_active_now = true;
          active_nav_through_poses_goal_id_ = status.goal_info.goal_id;
          latest_terminal_status_.reset();
          break;
        }
      }
    } else {
      is_active_now = true;
    }

    if (active_nav_through_poses_goal_id_) {
      for (const auto& status : msg->status_list) {
        if (is_same_goal_id(status.goal_info.goal_id, *active_nav_through_poses_goal_id_)) {
          if (status.status == GoalStatus::STATUS_CANCELED) { final_status = status; break; }
          if (status.status == GoalStatus::STATUS_SUCCEEDED || status.status == GoalStatus::STATUS_ABORTED) {
            final_status = status;
          }
        }
      }
    }

    if (final_status) {
      is_active_now = false;
      latest_terminal_status_ = final_status;
      active_nav_through_poses_goal_id_.reset();
    }

    if (was_active != is_active_now) is_nav_through_poses_executing_ = is_active_now;
  }

  evaluate_and_publish_if_changed();
}

void StatusManager::waypoints_status_callback(const GoalStatusArray::SharedPtr msg)
{
  bool was_active;
  {
    std::lock_guard<std::mutex> lk(status_mutex_);
    was_active = is_waypoints_executing_;
  }

  bool is_active_now = false;
  std::optional<action_msgs::msg::GoalStatus> final_status;

  {
    std::lock_guard<std::mutex> lk(status_mutex_);
    if (!active_waypoints_goal_id_) {
      for (const auto& status : msg->status_list) {
        if (status.status == GoalStatus::STATUS_EXECUTING) {
          is_active_now = true;
          active_waypoints_goal_id_ = status.goal_info.goal_id;
          latest_terminal_status_.reset();
          break;
        }
      }
    } else {
      is_active_now = true;
    }

    if (active_waypoints_goal_id_) {
      for (const auto& status : msg->status_list) {
        if (is_same_goal_id(status.goal_info.goal_id, *active_waypoints_goal_id_)) {
          if (status.status == GoalStatus::STATUS_CANCELED) { final_status = status; break; }
          if (status.status == GoalStatus::STATUS_SUCCEEDED || status.status == GoalStatus::STATUS_ABORTED) {
            final_status = status;
          }
        }
      }
    }

    if (final_status) {
      is_active_now = false;
      latest_terminal_status_ = final_status;
      active_waypoints_goal_id_.reset();
    }

    if (was_active != is_active_now) is_waypoints_executing_ = is_active_now;
  }

  evaluate_and_publish_if_changed();
}

void StatusManager::nav_to_pose_status_callback(const GoalStatusArray::SharedPtr msg)
{
  bool was_active;
  {
    std::lock_guard<std::mutex> lk(status_mutex_);
    was_active = is_nav_executing_;
  }

  bool is_active_now = false;
  std::optional<action_msgs::msg::GoalStatus> final_status;

  {
    std::lock_guard<std::mutex> lk(status_mutex_);
    if (!active_nav_goal_id_) {
      for (const auto& status : msg->status_list) {
        if (status.status == GoalStatus::STATUS_EXECUTING) {
          is_active_now = true;
          active_nav_goal_id_ = status.goal_info.goal_id;
          latest_terminal_status_.reset();
          break;
        }
      }
    } else {
      is_active_now = true;
    }

    if (active_nav_goal_id_) {
      for (const auto& status : msg->status_list) {
        if (is_same_goal_id(status.goal_info.goal_id, *active_nav_goal_id_)) {
          if (status.status == GoalStatus::STATUS_CANCELED) { final_status = status; break; }
          if (status.status == GoalStatus::STATUS_SUCCEEDED || status.status == GoalStatus::STATUS_ABORTED) {
            final_status = status;
          }
        }
      }
    }

    if (final_status) {
      is_active_now = false;
      latest_terminal_status_ = final_status;
      active_nav_goal_id_.reset();
    }

    if (was_active != is_active_now) is_nav_executing_ = is_active_now;
  }

  evaluate_and_publish_if_changed();
}

void StatusManager::compute_path_poses_status_callback(const GoalStatusArray::SharedPtr msg)
{
  bool is_active = false;
  for (const auto& s : msg->status_list) {
    if (s.status == GoalStatus::STATUS_EXECUTING) { is_active = true; break; }
  }
  {
    std::lock_guard<std::mutex> lk(status_mutex_);
    is_planning_sub_action_executing_ = is_active;
  }
  evaluate_and_publish_if_changed();
}

void StatusManager::compute_path_status_callback(const GoalStatusArray::SharedPtr msg)
{
  bool is_active = false;
  for (const auto& s : msg->status_list) {
    if (s.status == GoalStatus::STATUS_EXECUTING) { is_active = true; break; }
  }
  {
    std::lock_guard<std::mutex> lk(status_mutex_);
    is_planning_sub_action_executing_ = is_active;
  }
  evaluate_and_publish_if_changed();
}

void StatusManager::follow_path_status_callback(const GoalStatusArray::SharedPtr msg)
{
  bool is_active = false;
  for (const auto& s : msg->status_list) {
    if (s.status == GoalStatus::STATUS_EXECUTING) { is_active = true; break; }
  }
  {
    std::lock_guard<std::mutex> lk(status_mutex_);
    is_driving_sub_action_executing_ = is_active;
  }
  evaluate_and_publish_if_changed();
}

void StatusManager::bt_log_callback(const BehaviorTreeLog::SharedPtr msg)
{
  std::optional<std_msgs::msg::String> to_pub;

  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    bt_log_seen_in_cycle_ = true;

    // (1) 원샷 RECEIVED_GOAL: 터미널 상태 && 아직 arm 상태일 때만 1회 발화
    if (received_goal_armed_ && is_terminal_robot_status(current_status_)) {
      current_status_ = RobotStatus::RECEIVED_GOAL;
      received_goal_armed_ = false;
      std_msgs::msg::String s; s.data = status_to_string(current_status_);
      RCLCPP_INFO(this->get_logger(), "Robot Status Changed (BT first seen) -> %s", s.data.c_str());
      to_pub = s;
    }

    // (2) 리커버리 이벤트 감지 + 현재 RUNNING 노드 리스트 업데이트(증분 로그 기반)
    static const std::set<std::string> recovery_sequence_nodes = {
      "ShortRecoverySequence1",
      "ShortRecoverySequence2",
      "ShortRecoverySequenceTotal1",
    };

    bool status_changed_by_recovery = false;
    RobotStatus new_status = current_status_;

    running_bt_nodes_.clear();
    bool recovery_flag_check = false;

    for (const auto& event : msg->event_log) {
      if (event.current_status == "RUNNING") {
        running_bt_nodes_.push_back(event.node_name);
        if (recovery_sequence_nodes.count(event.node_name)) {
          recovery_flag_check = true;
        }
      }
      if (recovery_sequence_nodes.count(event.node_name)) {
        if (event.current_status == "RUNNING")      { new_status = RobotStatus::RECOVERY_RUNNING; status_changed_by_recovery = true; }
        else if (event.current_status == "SUCCESS")  { new_status = RobotStatus::RECOVERY_SUCCESS; status_changed_by_recovery = true; }
        else if (event.current_status == "FAILURE")  { new_status = RobotStatus::RECOVERY_FAILURE; status_changed_by_recovery = true; }
      }
    }

    is_in_recovery_context_ = recovery_flag_check;

    if (status_changed_by_recovery && current_status_ != new_status) {
      current_status_ = new_status;
      std_msgs::msg::String s; s.data = status_to_string(current_status_);
      RCLCPP_INFO(this->get_logger(), "Robot Status Changed (by Recovery BT) -> %s", s.data.c_str());
      to_pub = s;  // 덮어씀
    }
  }

  if (to_pub) status_publisher_->publish(*to_pub);

  // 최종 일관성 유지
  evaluate_and_publish_if_changed();
}

void StatusManager::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const double lin = std::abs(msg->twist.twist.linear.x);
  const double ang = std::abs(msg->twist.twist.angular.z);
  {
    std::lock_guard<std::mutex> lk(status_mutex_);
    is_robot_stopped_ = (lin < 0.01 && ang < 0.01);
  }
}

void StatusManager::collision_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lk(status_mutex_);
    is_collision_imminent_ = msg->data;
  }
  evaluate_and_publish_if_changed();
}

void StatusManager::pause_flag_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lk(status_mutex_);
    is_paused_ = msg->data;
  }
  evaluate_and_publish_if_changed();
}

void StatusManager::query_initial_node_states()
{
  bool all_active = true;
  for (size_t i = 0; i < core_node_names_.size(); ++i) {
    while (!lifecycle_clients_[i]->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for %s service.", core_node_names_[i].c_str());
        {
          std::lock_guard<std::mutex> lk(status_mutex_);
          are_core_nodes_active_ = false;
        }
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for %s service...", core_node_names_[i].c_str());
    }

    auto req = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto result_future = lifecycle_clients_[i]->async_send_request(req);
    auto result = result_future.get();
    if (!result || result->current_state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      all_active = false;
      RCLCPP_WARN(this->get_logger(), "Node %s is not active.", core_node_names_[i].c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Node %s is active.", core_node_names_[i].c_str());
    }
  }
  {
    std::lock_guard<std::mutex> lk(status_mutex_);
    are_core_nodes_active_ = all_active;
  }
  RCLCPP_INFO(this->get_logger(), "Core nodes active status: %s", all_active ? "true" : "false");
  evaluate_and_publish_if_changed();
}

bool StatusManager::is_bt_node_running(const std::string& node_name) const
{
  std::lock_guard<std::mutex> lk(status_mutex_);
  return std::find(running_bt_nodes_.begin(), running_bt_nodes_.end(), node_name) != running_bt_nodes_.end();
}

std::string StatusManager::status_to_string(RobotStatus status)
{
  switch (status) {
    case RobotStatus::IDLE:                return "IDLE";
    case RobotStatus::RECEIVED_GOAL:       return "RECEIVED_GOAL";
    case RobotStatus::PLANNING:            return "PLANNING";
    case RobotStatus::DRIVING:             return "DRIVING";
    case RobotStatus::FOLLOWING_WAYPOINTS: return "FOLLOWING_WAYPOINTS";
    case RobotStatus::PAUSED:              return "PAUSED";
    case RobotStatus::COLLISION_IMMINENT:  return "COLLISION_IMMINENT";
    case RobotStatus::RECOVERY_FAILURE:    return "RECOVERY_FAILURE";
    case RobotStatus::RECOVERY_RUNNING:    return "RECOVERY_RUNNING";
    case RobotStatus::RECOVERY_SUCCESS:    return "RECOVERY_SUCCESS";
    case RobotStatus::SUCCEEDED:           return "SUCCEEDED";
    case RobotStatus::FAILED:              return "FAILED";
    case RobotStatus::CANCELED:            return "CANCELED";
    default:                               return "UNKNOWN";
  }
}
```

---

### 메모

* **실행기**: 반드시 `MultiThreadedExecutor`(혹은 `component_container_mt`) 사용 권장.
* **스레드 안전**: Reentrant 그룹이라도 모든 공유 상태는 `status_mutex_`로 감싸도록 통일했어. 각 콜백에서 읽기/판정/쓰기 모두 mutex 범위 안에서 수행.
* **RECEIVED\_GOAL**: `/behavior_tree_log`가 처음 수신되는 시점에, 현재 로봇 상태가 터미널이면 1회만 `RECEIVED_GOAL`로 전이 → 액션 종료로 터미널 상태 복귀 시 arm 재장전.

필요하면 `CMakeLists.txt` / `package.xml` / `component` 등록까지 같이 만들어줄게.


#################  


좋아—세 가지(⟨CANCELED, FAILED, SUCCEEDED⟩) 상태로 “전이되는 순간”에 BT 누적 캐시(`bt_states_`)를 싹 비우면 돼. 방법은 간단해:

* `bt_states_`를 멤버로 유지(이미 있다면 재사용).
* **상태 전이 로직 한 곳**(= `evaluate_and_publish_if_changed()`)에서 새 상태가 `CANCELED|FAILED|SUCCEEDED`일 때 `bt_states_`와 보조 캐시들(`running_bt_nodes_`, `is_in_recovery_context_`)을 초기화.
* 스레드 안전을 위해 **status\_mutex\_가 잠긴 구간 안에서** 초기화.

아래는 필요한 최소 패치야. (네가 마지막에 사용 중인 “A방식” 코드 기준)

---

### 1) 헤더에 `bt_states_`와 초기화 헬퍼 추가

```cpp
// status_manager.hpp (멤버들 중)
#include <unordered_map>

// ...

private:
  // ... 기존 멤버들 ...
  std::vector<std::string> running_bt_nodes_;
  bool is_in_recovery_context_{false};

  // BT 상태 누적 맵: node_name -> "IDLE|RUNNING|SUCCESS|FAILURE"
  std::unordered_map<std::string, std::string> bt_states_;

  // 내부 유틸: mutex가 잡힌 상태에서 BT 캐시 초기화
  void reset_bt_state_cache_locked_();
```

---

### 2) 소스에 초기화 함수 구현

```cpp
// status_manager.cpp
void StatusManager::reset_bt_state_cache_locked_()
{
  // status_mutex_ 가 이미 잠겨 있어야 함!
  bt_states_.clear();
  running_bt_nodes_.clear();
  is_in_recovery_context_ = false;
}
```

---

### 3) 상태 전이 지점에서 초기화 트리거

`evaluate_and_publish_if_changed()` 안에서 “상태가 바뀌었고, 새 상태가 ⟨SUCCEEDED|FAILED|CANCELED⟩이면” 초기화:

```cpp
void StatusManager::evaluate_and_publish_if_changed()
{
  std::optional<std_msgs::msg::String> to_pub;

  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    RobotStatus new_status = determine_current_status(current_status_);

    // 터미널(넓은 의미)에서 RECEIVED_GOAL 재무장
    if (is_terminal_robot_status(new_status)) {
      received_goal_armed_ = true;
      bt_log_seen_in_cycle_ = false;
    }

    if (new_status != current_status_) {
      // === 여기서 "세 가지" 터미널에 한해 BT 캐시 초기화 ===
      if (new_status == RobotStatus::SUCCEEDED ||
          new_status == RobotStatus::FAILED   ||
          new_status == RobotStatus::CANCELED) {
        reset_bt_state_cache_locked_();
      }

      current_status_ = new_status;
      std_msgs::msg::String s; s.data = status_to_string(current_status_);
      RCLCPP_INFO(this->get_logger(), "Robot Status Changed -> %s", s.data.c_str());
      to_pub = s;
    }
  }

  if (to_pub) status_publisher_->publish(*to_pub);
}
```

> 이렇게 하면 `latest_terminal_status_`를 통해 결과가 결정되든, 다른 경로로 터미널이 되든 **항상 한 곳**에서 BT 캐시가 정리돼요.

---

### 4) BT 로그 콜백에서 `bt_states_` 누적 업데이트 (선택 사항)

지금은 `running_bt_nodes_`만 쓰지만, 너가 말한 누적 맵을 계속 쓰고 싶다면 `/behavior_tree_log`에서 **증분 이벤트**를 누적 반영하면 돼:

```cpp
void StatusManager::bt_log_callback(const BehaviorTreeLog::SharedPtr msg)
{
  std::optional<std_msgs::msg::String> to_pub;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    bt_log_seen_in_cycle_ = true;

    // 원샷 RECEIVED_GOAL
    if (received_goal_armed_ && is_terminal_robot_status(current_status_)) {
      current_status_ = RobotStatus::RECEIVED_GOAL;
      received_goal_armed_ = false;
      std_msgs::msg::String s; s.data = status_to_string(current_status_);
      RCLCPP_INFO(this->get_logger(), "Robot Status Changed (BT first seen) -> %s", s.data.c_str());
      to_pub = s;
    }

    // 누적 맵 / 러닝 노드 업데이트
    static const std::set<std::string> recovery_nodes = {
      "ShortRecoverySequence1","ShortRecoverySequence2","ShortRecoverySequenceTotal1",
    };
    bool status_changed_by_recovery = false;
    RobotStatus new_status = current_status_;

    running_bt_nodes_.clear();
    bool recovery_flag_check = false;

    for (const auto& ev : msg->event_log) {
      // 누적 맵 업데이트: 항상 마지막 current_status로 갱신
      bt_states_[ev.node_name] = ev.current_status;

      if (ev.current_status == "RUNNING") {
        running_bt_nodes_.push_back(ev.node_name);
        if (recovery_nodes.count(ev.node_name)) recovery_flag_check = true;
      }
      if (recovery_nodes.count(ev.node_name)) {
        if (ev.current_status == "RUNNING")      { new_status = RobotStatus::RECOVERY_RUNNING; status_changed_by_recovery = true; }
        else if (ev.current_status == "SUCCESS")  { new_status = RobotStatus::RECOVERY_SUCCESS; status_changed_by_recovery = true; }
        else if (ev.current_status == "FAILURE")  { new_status = RobotStatus::RECOVERY_FAILURE; status_changed_by_recovery = true; }
      }
    }

    is_in_recovery_context_ = recovery_flag_check;

    if (status_changed_by_recovery && current_status_ != new_status) {
      current_status_ = new_status;
      std_msgs::msg::String s; s.data = status_to_string(current_status_);
      RCLCPP_INFO(this->get_logger(), "Robot Status Changed (by Recovery BT) -> %s", s.data.c_str());
      to_pub = s; // 덮어씀
    }
  }

  if (to_pub) status_publisher_->publish(*to_pub);
  evaluate_and_publish_if_changed();
}
```

---

### 5) (옵션) IDLE에서도 초기화하고 싶다면?

요청은 “canceled/failed/succeeded에서 초기화”였지만, IDLE 진입 시에도 깨끗하게 시작하고 싶으면 조건을 한 줄 추가하면 돼:

```cpp
if (new_status == RobotStatus::SUCCEEDED ||
    new_status == RobotStatus::FAILED   ||
    new_status == RobotStatus::CANCELED ||
    new_status == RobotStatus::IDLE) {   // ← 선택
  reset_bt_state_cache_locked_();
}
```

---

이렇게 하면 **세 터미널 상태로 전이되는 순간**에 `bt_states_`는 물론 `running_bt_nodes_`/`is_in_recovery_context_`까지 싹 초기화돼서, 다음 사이클에서 BT 로그가 들어오면 항상 깔끔한 상태로 “원샷 RECEIVED\_GOAL → 후속 상태” 흐름이 이어집니다.

