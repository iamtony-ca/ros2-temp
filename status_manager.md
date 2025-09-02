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
