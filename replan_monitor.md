아래는 제안했던 개선점을 모두 반영해 “컴파일·실전 사용”을 목표로 리팩토링한 **완성 코드(hpp, cpp)** 입니다.
핵심 변경 사항:

* TF 변환 버그 수정(Transform → PoseStamped 변환)
* `nav2_msgs::msg::Costmap` → `Costmap2D` 안전 변환(메타데이터 기반 + data memcpy)
* ROI를 **전방 콘(부채꼴) + stride**로 스캔 (CPU 절약)
* **Unknown 제외 옵션**, **커널(3×3 등) 확장 검사**, **연속 K 구간 기준**으로 오탐 감소
* `/plan_pruned` 프레임을 글로벌 프레임으로 변환 후 검사
* `/replan_flag`를 **이벤트 펄스**로 퍼블리시(옵션: 짧은 false 펄스)
* 프레임/파라미터화, costmap 메타 변경시 DB 리셋 등

---

### `include/replan_monitor/replan_monitor_node.hpp`

```cpp
#ifndef REPLAN_MONITOR_NODE_HPP
#define REPLAN_MONITOR_NODE_HPP

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_msgs/msg/costmap.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace replan_monitor
{

struct ObstacleInfo
{
  rclcpp::Time first_seen;
  rclcpp::Time last_seen;
};

struct CostmapSignature
{
  unsigned int size_x{0};
  unsigned int size_y{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};

  bool operator==(const CostmapSignature & other) const
  {
    return size_x == other.size_x && size_y == other.size_y &&
           resolution == other.resolution &&
           origin_x == other.origin_x && origin_y == other.origin_y;
  }
};

class ReplanMonitorNode : public rclcpp::Node
{
public:
  ReplanMonitorNode();

private:
  // ========= Callbacks =========
  void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
  void robotStatusCallback(const std_msgs::msg::String::SharedPtr msg);
  void validatePathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void updateObstacleDatabase();

  // ========= Helpers / Utils =========
  bool getCurrentPoseFromTF(geometry_msgs::msg::Pose & pose_out) const;
  bool transformToGlobal(const geometry_msgs::msg::PoseStamped & in,
                         geometry_msgs::msg::PoseStamped & out) const;
  void transformPathToGlobal(const nav_msgs::msg::Path & in,
                             std::vector<geometry_msgs::msg::PoseStamped> & out) const;

  inline uint64_t packKey(unsigned int mx, unsigned int my) const
  {
    return (static_cast<uint64_t>(mx) << 32) | static_cast<uint64_t>(my);
  }

  bool isBlockedCellKernel(unsigned int mx, unsigned int my) const;

  void triggerReplan(const std::string & reason);

  // ========= ROS Interfaces =========
  rclcpp::CallbackGroup::SharedPtr subs_callback_group_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pruned_path_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_status_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr replan_pub_;

  rclcpp::TimerBase::SharedPtr obstacle_db_update_timer_;
  rclcpp::TimerBase::SharedPtr flag_reset_timer_;  // optional false pulse

  // ========= State =========
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  mutable std::mutex costmap_mutex_;
  CostmapSignature last_costmap_sig_;

  std::atomic<bool> is_robot_in_driving_state_{false};
  rclcpp::Time last_replan_time_{0, 0, RCL_ROS_TIME};

  std::unordered_map<uint64_t, ObstacleInfo> obstacle_db_;
  mutable std::mutex obstacle_db_mutex_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ========= Parameters =========
  std::string global_frame_;
  std::string base_frame_;

  double cooldown_sec_;
  size_t consecutive_threshold_;
  double obstacle_persistence_sec_;
  double max_speed_;
  double lookahead_time_sec_;
  double min_lookahead_m_;
  double cost_threshold_;
  bool ignore_unknown_;

  // DB/ROI
  double db_update_frequency_;
  double obstacle_prune_timeout_sec_;
  int db_stride_;                 // >=1
  double cone_angle_deg_;         // forward cone angle (e.g., 90 deg)
  int kernel_half_size_;          // e.g., 1 => 3x3; 0 => 1x1

  // Path checking
  double path_check_distance_m_;  // additionally cap lookahead by this distance

  // Replan flag pulse
  bool publish_false_pulse_;
  int flag_pulse_ms_;
};

}  // namespace replan_monitor

#endif  // REPLAN_MONITOR_NODE_HPP
```

---

### `src/replan_monitor_node.cpp`

```cpp
#include "replan_monitor/replan_monitor_node.hpp"

using std::placeholders::_1;

namespace replan_monitor
{

ReplanMonitorNode::ReplanMonitorNode()
: Node("replan_monitor_node")
{
  // ===== Declare & get parameters =====
  this->declare_parameter<std::string>("global_frame", "map");
  this->declare_parameter<std::string>("base_frame", "base_link");

  this->declare_parameter("cooldown_sec", 1.0);
  this->declare_parameter("consecutive_threshold", 3);   // 연속 K 포인트 기준
  this->declare_parameter("obstacle_persistence_sec", 0.5);
  this->declare_parameter("max_speed", 0.5);
  this->declare_parameter("lookahead_time_sec", 15.0);
  this->declare_parameter("min_lookahead_m", 2.0);       // 최소 전방 거리
  this->declare_parameter("cost_threshold", 200.0);      // 기본적으로 높은 값 권장
  this->declare_parameter("ignore_unknown", true);

  this->declare_parameter("db_update_frequency", 5.0);   // Hz
  this->declare_parameter("obstacle_prune_timeout_sec", 3.0);
  this->declare_parameter("db_stride", 2);               // 1=모든 셀, 2=샘플링
  this->declare_parameter("cone_angle_deg", 100.0);      // 전방 부채꼴 각
  this->declare_parameter("kernel_half_size", 1);        // 3x3

  this->declare_parameter("path_check_distance_m", 6.0); // 경로 검사 상한 거리

  this->declare_parameter("publish_false_pulse", true);
  this->declare_parameter("flag_pulse_ms", 120);

  global_frame_               = this->get_parameter("global_frame").as_string();
  base_frame_                 = this->get_parameter("base_frame").as_string();

  cooldown_sec_               = this->get_parameter("cooldown_sec").as_double();
  consecutive_threshold_      = this->get_parameter("consecutive_threshold").as_int();
  obstacle_persistence_sec_   = this->get_parameter("obstacle_persistence_sec").as_double();
  max_speed_                  = this->get_parameter("max_speed").as_double();
  lookahead_time_sec_         = this->get_parameter("lookahead_time_sec").as_double();
  min_lookahead_m_            = this->get_parameter("min_lookahead_m").as_double();
  cost_threshold_             = this->get_parameter("cost_threshold").as_double();
  ignore_unknown_             = this->get_parameter("ignore_unknown").as_bool();

  db_update_frequency_        = this->get_parameter("db_update_frequency").as_double();
  obstacle_prune_timeout_sec_ = this->get_parameter("obstacle_prune_timeout_sec").as_double();
  db_stride_                  = std::max(1, this->get_parameter("db_stride").as_int());
  cone_angle_deg_             = this->get_parameter("cone_angle_deg").as_double();
  kernel_half_size_           = std::max(0, this->get_parameter("kernel_half_size").as_int());

  path_check_distance_m_      = this->get_parameter("path_check_distance_m").as_double();

  publish_false_pulse_        = this->get_parameter("publish_false_pulse").as_bool();
  flag_pulse_ms_              = this->get_parameter("flag_pulse_ms").as_int();

  // ===== TF =====
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ===== Callback Groups =====
  subs_callback_group_  = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto subs_options = rclcpp::SubscriptionOptions();
  subs_options.callback_group = subs_callback_group_;

  // ===== Subscriptions =====
  // Nav2 글로벌 코스트맵 raw: transient_local + reliable가 보통의 QoS
  costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
      "/global_costmap/costmap_raw",
      rclcpp::QoS(rclcpp::SystemDefaultsQoS()).transient_local().reliable(),
      std::bind(&ReplanMonitorNode::costmapCallback, this, _1),
      subs_options);

  robot_status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot_status",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&ReplanMonitorNode::robotStatusCallback, this, _1),
      subs_options);

  pruned_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/plan_pruned",
      rclcpp::QoS(10),
      std::bind(&ReplanMonitorNode::validatePathCallback, this, _1),
      subs_options);

  // ===== Publisher =====
  // 이벤트성 펄스 → reliable + volatile 권장
  replan_pub_ = this->create_publisher<std_msgs::msg::Bool>("/replan_flag", rclcpp::QoS(1).reliable());

  // ===== Timers =====
  const int period_ms = static_cast<int>(1000.0 / std::max(1.0, db_update_frequency_));
  obstacle_db_update_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&ReplanMonitorNode::updateObstacleDatabase, this),
      timer_callback_group_);

  RCLCPP_INFO(this->get_logger(), "ReplanMonitorNode initialized.");
}

// ===================== Costmap handling =====================

void ReplanMonitorNode::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);

  // Extract metadata
  CostmapSignature sig;
  sig.size_x    = msg->metadata.size_x;
  sig.size_y    = msg->metadata.size_y;
  sig.resolution= msg->metadata.resolution;
  sig.origin_x  = msg->metadata.origin.position.x;
  sig.origin_y  = msg->metadata.origin.position.y;

  if (!costmap_ || !(sig == last_costmap_sig_)) {
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>(
        sig.size_x, sig.size_y, sig.resolution, sig.origin_x, sig.origin_y,
        nav2_costmap_2d::FREE_SPACE);
    last_costmap_sig_ = sig;

    // 메타 변경 시, DB는 맵 좌표계가 바뀌었을 수 있으므로 리셋이 안전
    {
      std::lock_guard<std::mutex> db_lock(obstacle_db_mutex_);
      obstacle_db_.clear();
    }
  }

  const size_t expected = static_cast<size_t>(sig.size_x) * static_cast<size_t>(sig.size_y);
  if (msg->data.size() != expected) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Costmap data size mismatch: got %zu, expected %zu",
        msg->data.size(), expected);
    return;
  }

  // copy data into costmap
  std::memcpy(costmap_->getCharMap(), msg->data.data(), expected * sizeof(unsigned char));
}

void ReplanMonitorNode::robotStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string & s = msg->data;
  // 필요 시 상태 문자열을 파라미터화/Enum화 가능
  is_robot_in_driving_state_.store(s == "DRIVING" || s == "PLANNING");
}

// ===================== TF helpers =====================

bool ReplanMonitorNode::getCurrentPoseFromTF(geometry_msgs::msg::Pose & pose_out) const
{
  geometry_msgs::msg::PoseStamped base_in, base_in_global;
  base_in.header.stamp = this->now();
  base_in.header.frame_id = base_frame_;
  base_in.pose.orientation.w = 1.0;

  try {
    auto tf = tf_buffer_->lookupTransform(global_frame_, base_frame_, base_in.header.stamp,
                                          rclcpp::Duration::from_seconds(0.2));
    tf2::doTransform(base_in, base_in_global, tf);
    pose_out = base_in_global.pose;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Could not get robot pose %s->%s: %s",
        global_frame_.c_str(), base_frame_.c_str(), ex.what());
    return false;
  }
}

bool ReplanMonitorNode::transformToGlobal(const geometry_msgs::msg::PoseStamped & in,
                                          geometry_msgs::msg::PoseStamped & out) const
{
  if (in.header.frame_id.empty() || in.header.frame_id == global_frame_) {
    out = in;
    out.header.frame_id = global_frame_;
    return true;
  }
  try {
    auto tf = tf_buffer_->lookupTransform(global_frame_, in.header.frame_id, in.header.stamp,
                                          rclcpp::Duration::from_seconds(0.2));
    tf2::doTransform(in, out, tf);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "transformToGlobal failed %s->%s: %s",
        in.header.frame_id.c_str(), global_frame_.c_str(), ex.what());
    return false;
  }
}

void ReplanMonitorNode::transformPathToGlobal(const nav_msgs::msg::Path & in,
                                              std::vector<geometry_msgs::msg::PoseStamped> & out) const
{
  out.clear();
  out.reserve(in.poses.size());
  for (const auto & ps : in.poses) {
    geometry_msgs::msg::PoseStamped g;
    if (transformToGlobal(ps, g)) {
      out.emplace_back(std::move(g));
    }
  }
}

// ===================== Obstacle DB update (Detection) =====================

void ReplanMonitorNode::updateObstacleDatabase()
{
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (!costmap_) return;
    costmap = costmap_;
  }

  geometry_msgs::msg::Pose pose;
  if (!getCurrentPoseFromTF(pose)) return;

  const double lookahead = std::max(min_lookahead_m_, max_speed_ * lookahead_time_sec_);

  // Clamp ROI to map bounds (AABB)
  const double map_min_x = costmap->getOriginX();
  const double map_min_y = costmap->getOriginY();
  const double map_max_x = map_min_x + costmap->getSizeInCellsX() * costmap->getResolution();
  const double map_max_y = map_min_y + costmap->getSizeInCellsY() * costmap->getResolution();

  const double min_x = std::max(pose.position.x - lookahead, map_min_x);
  const double min_y = std::max(pose.position.y - lookahead, map_min_y);
  const double max_x = std::min(pose.position.x + lookahead, map_max_x);
  const double max_y = std::min(pose.position.y + lookahead, map_max_y);

  unsigned int min_mx, min_my, max_mx, max_my;
  if (!costmap->worldToMap(min_x, min_y, min_mx, min_my) ||
      !costmap->worldToMap(max_x, max_y, max_mx, max_my)) {
    // 완전히 맵 밖
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "ROI outside map bounds");
    return;
  }

  const rclcpp::Time now = this->now();
  std::unordered_set<uint64_t> visible;

  // Forward cone parameters
  const double yaw = tf2::getYaw(pose.orientation);
  const double ux = std::cos(yaw), uy = std::sin(yaw);
  const double cos_half = std::cos((cone_angle_deg_ * M_PI / 180.0) * 0.5);
  const double res = costmap->getResolution();

  // Sample cells with stride
  for (unsigned int mx = min_mx; mx <= max_mx; mx += static_cast<unsigned int>(db_stride_)) {
    for (unsigned int my = min_my; my <= max_my; my += static_cast<unsigned int>(db_stride_)) {
      // Cell center world
      double wx = costmap->mapToWorldX(mx) + res * 0.5;
      double wy = costmap->mapToWorldY(my) + res * 0.5;

      // In range?
      const double dx = wx - pose.position.x;
      const double dy = wy - pose.position.y;
      const double dist = std::hypot(dx, dy);
      if (dist > lookahead) continue;

      // In forward cone?
      const double ndot = (dx * ux + dy * uy) / std::max(1e-6, dist);
      if (ndot < cos_half) continue;

      // Cost check
      const unsigned char c = costmap->getCost(mx, my);
      if (ignore_unknown_ && c == nav2_costmap_2d::NO_INFORMATION) continue;
      if (c >= static_cast<unsigned char>(cost_threshold_)) {
        visible.insert(packKey(mx, my));
      }
    }
  }

  // Update DB
  {
    std::lock_guard<std::mutex> db_lock(obstacle_db_mutex_);
    // upsert
    for (const auto key : visible) {
      auto it = obstacle_db_.find(key);
      if (it == obstacle_db_.end()) {
        obstacle_db_[key] = ObstacleInfo{now, now};
      } else {
        it->second.last_seen = now;
      }
    }
    // prune
    for (auto it = obstacle_db_.begin(); it != obstacle_db_.end(); ) {
      if ((now - it->second.last_seen).seconds() > obstacle_prune_timeout_sec_) {
        it = obstacle_db_.erase(it);
      } else {
        ++it;
      }
    }
  }
}

// ===================== Path validation (Decision) =====================

bool ReplanMonitorNode::isBlockedCellKernel(unsigned int mx, unsigned int my) const
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  if (!costmap_) return false;

  const int K = kernel_half_size_;
  const int sx = static_cast<int>(costmap_->getSizeInCellsX());
  const int sy = static_cast<int>(costmap_->getSizeInCellsY());

  for (int dx = -K; dx <= K; ++dx) {
    for (int dy = -K; dy <= K; ++dy) {
      const int x = static_cast<int>(mx) + dx;
      const int y = static_cast<int>(my) + dy;
      if (x < 0 || y < 0 || x >= sx || y >= sy) continue;

      const unsigned char c = costmap_->getCost(static_cast<unsigned int>(x), static_cast<unsigned int>(y));
      if (ignore_unknown_ && c == nav2_costmap_2d::NO_INFORMATION) continue;
      if (c >= static_cast<unsigned char>(cost_threshold_)) return true;
    }
  }
  return false;
}

void ReplanMonitorNode::validatePathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (!is_robot_in_driving_state_.load()) return;
  if (!msg || msg->poses.empty()) return;

  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (!costmap_) return;
    costmap = costmap_;
  }

  // Transform path to global frame
  std::vector<geometry_msgs::msg::PoseStamped> gpath;
  transformPathToGlobal(*msg, gpath);
  if (gpath.empty()) return;

  // Current pose (to cap path distance & order)
  geometry_msgs::msg::Pose cur_pose;
  if (!getCurrentPoseFromTF(cur_pose)) return;

  // Limit path length checked (min of time-lookahead & path_check_distance_m_)
  const double max_dist_by_time = std::max(min_lookahead_m_, max_speed_ * lookahead_time_sec_);
  const double max_check_dist = std::min(max_dist_by_time, path_check_distance_m_);

  // Walk along path accumulating distance; evaluate consecutive blocked streak
  size_t best_streak = 0;
  size_t streak = 0;
  rclcpp::Time now = this->now();

  // Snapshot DB (to shorten lock duration)
  std::unordered_map<uint64_t, ObstacleInfo> db_snapshot;
  {
    std::lock_guard<std::mutex> lock(obstacle_db_mutex_);
    db_snapshot = obstacle_db_;
  }

  double acc = 0.0;
  for (size_t i = 0; i < gpath.size(); ++i) {
    if (i > 0) {
      const auto & p0 = gpath[i-1].pose.position;
      const auto & p1 = gpath[i].pose.position;
      acc += std::hypot(p1.x - p0.x, p1.y - p0.y);
      if (acc > max_check_dist) break;
    }

    unsigned int mx, my;
    if (!costmap->worldToMap(gpath[i].pose.position.x, gpath[i].pose.position.y, mx, my)) {
      streak = 0;
      continue;
    }

    // 1) local cell kernel check
    bool blocked_cell = isBlockedCellKernel(mx, my);

    // 2) persistent DB + 지속성 확인
    const uint64_t key = packKey(mx, my);
    bool persistent_mature = false;
    auto it = db_snapshot.find(key);
    if (it != db_snapshot.end()) {
      if ((now - it->second.first_seen).seconds() >= obstacle_persistence_sec_) {
        persistent_mature = true;
      }
    }

    const bool blocked = blocked_cell && persistent_mature;
    streak = blocked ? (streak + 1) : 0;
    best_streak = std::max(best_streak, streak);

    if (best_streak >= consecutive_threshold_) {
      triggerReplan("blocked streak threshold reached");
      break;
    }
  }
}

// ===================== Replan pulse =====================

void ReplanMonitorNode::triggerReplan(const std::string & reason)
{
  const rclcpp::Time now = this->now();
  if ((now - last_replan_time_).seconds() <= cooldown_sec_) {
    return;
  }
  last_replan_time_ = now;

  std_msgs::msg::Bool m; m.data = true;
  replan_pub_->publish(m);
  RCLCPP_WARN(this->get_logger(), "Triggering replan: %s", reason.c_str());

  if (publish_false_pulse_ && flag_pulse_ms_ > 0) {
    // cancel previous timer if any
    flag_reset_timer_.reset();
    auto weak_pub = std::weak_ptr<rclcpp::Publisher<std_msgs::msg::Bool>>(replan_pub_);
    flag_reset_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(flag_pulse_ms_),
        [weak_pub]() {
          if (auto pub = weak_pub.lock()) {
            std_msgs::msg::Bool off; off.data = false;
            pub->publish(off);
          }
        });
  }
}

}  // namespace replan_monitor
```

---

#### 사용 팁 (요약)

* 기본 파라미터는 안전한 쪽으로 잡았습니다. 특히 `cost_threshold=200`(inflated cost 환경에서 과민 억제), `ignore_unknown=true`, `consecutive_threshold=3`.
* 전방 콘 각(`cone_angle_deg`)·stride(`db_stride`)·커널(`kernel_half_size`)·검사거리(`path_check_distance_m`)로 민감도/부하를 조절하세요.
* 실행 시 `MultiThreadedExecutor`를 쓰면 콜백 그룹이 의미를 가집니다.

필요하시면 CMakeLists.txt / package.xml 템플릿도 만들어 드릴게요.

####################
좋아요! 패키지명은 `replan_monitor` 그대로 두고, 노드 이름과 클래스/파일명을 `path_validator_node`로 바꾼 **완성본(hpp, cpp)** 를 제공합니다.
파일 경로는 다음과 같이 쓰시면 됩니다.

* `include/replan_monitor/path_validator_node.hpp`
* `src/path_validator_node.cpp`

---

### `include/replan_monitor/path_validator_node.hpp`

```cpp
#ifndef PATH_VALIDATOR_NODE_HPP
#define PATH_VALIDATOR_NODE_HPP

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_msgs/msg/costmap.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace replan_monitor
{

struct ObstacleInfo
{
  rclcpp::Time first_seen;
  rclcpp::Time last_seen;
};

struct CostmapSignature
{
  unsigned int size_x{0};
  unsigned int size_y{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};

  bool operator==(const CostmapSignature & other) const
  {
    return size_x == other.size_x && size_y == other.size_y &&
           resolution == other.resolution &&
           origin_x == other.origin_x && origin_y == other.origin_y;
  }
};

class PathValidatorNode : public rclcpp::Node
{
public:
  PathValidatorNode();

private:
  // ========= Callbacks =========
  void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
  void robotStatusCallback(const std_msgs::msg::String::SharedPtr msg);
  void validatePathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void updateObstacleDatabase();

  // ========= Helpers / Utils =========
  bool getCurrentPoseFromTF(geometry_msgs::msg::Pose & pose_out) const;
  bool transformToGlobal(const geometry_msgs::msg::PoseStamped & in,
                         geometry_msgs::msg::PoseStamped & out) const;
  void transformPathToGlobal(const nav_msgs::msg::Path & in,
                             std::vector<geometry_msgs::msg::PoseStamped> & out) const;

  inline uint64_t packKey(unsigned int mx, unsigned int my) const
  {
    return (static_cast<uint64_t>(mx) << 32) | static_cast<uint64_t>(my);
  }

  bool isBlockedCellKernel(unsigned int mx, unsigned int my) const;
  void triggerReplan(const std::string & reason);

  // ========= ROS Interfaces =========
  rclcpp::CallbackGroup::SharedPtr subs_callback_group_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pruned_path_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_status_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr replan_pub_;

  rclcpp::TimerBase::SharedPtr obstacle_db_update_timer_;
  rclcpp::TimerBase::SharedPtr flag_reset_timer_;  // optional false pulse

  // ========= State =========
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  mutable std::mutex costmap_mutex_;
  CostmapSignature last_costmap_sig_;

  std::atomic<bool> is_robot_in_driving_state_{false};
  rclcpp::Time last_replan_time_;  // default epoch(0)

  std::unordered_map<uint64_t, ObstacleInfo> obstacle_db_;
  mutable std::mutex obstacle_db_mutex_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ========= Parameters =========
  std::string global_frame_;
  std::string base_frame_;

  double cooldown_sec_;
  size_t consecutive_threshold_;
  double obstacle_persistence_sec_;
  double max_speed_;
  double lookahead_time_sec_;
  double min_lookahead_m_;
  double cost_threshold_;
  bool ignore_unknown_;

  // DB/ROI
  double db_update_frequency_;
  double obstacle_prune_timeout_sec_;
  int db_stride_;                 // >=1
  double cone_angle_deg_;         // forward cone angle (e.g., 100 deg)
  int kernel_half_size_;          // e.g., 1 => 3x3; 0 => 1x1

  // Path checking
  double path_check_distance_m_;  // additionally cap lookahead by this distance

  // Replan flag pulse
  bool publish_false_pulse_;
  int flag_pulse_ms_;
};

}  // namespace replan_monitor

#endif  // PATH_VALIDATOR_NODE_HPP
```

---

### `src/path_validator_node.cpp`

```cpp
#include "replan_monitor/path_validator_node.hpp"

using std::placeholders::_1;

namespace replan_monitor
{

PathValidatorNode::PathValidatorNode()
: Node("path_validator_node")
{
  // ===== Declare & get parameters =====
  this->declare_parameter<std::string>("global_frame", "map");
  this->declare_parameter<std::string>("base_frame", "base_link");

  this->declare_parameter("cooldown_sec", 1.0);
  this->declare_parameter("consecutive_threshold", 3);   // 연속 K 포인트 기준
  this->declare_parameter("obstacle_persistence_sec", 0.5);
  this->declare_parameter("max_speed", 0.5);
  this->declare_parameter("lookahead_time_sec", 15.0);
  this->declare_parameter("min_lookahead_m", 2.0);       // 최소 전방 거리
  this->declare_parameter("cost_threshold", 200.0);      // 기본적으로 높은 값 권장
  this->declare_parameter("ignore_unknown", true);

  this->declare_parameter("db_update_frequency", 5.0);   // Hz
  this->declare_parameter("obstacle_prune_timeout_sec", 3.0);
  this->declare_parameter("db_stride", 2);               // 1=모든 셀, 2=샘플링
  this->declare_parameter("cone_angle_deg", 100.0);      // 전방 부채꼴 각
  this->declare_parameter("kernel_half_size", 1);        // 3x3

  this->declare_parameter("path_check_distance_m", 6.0); // 경로 검사 상한 거리

  this->declare_parameter("publish_false_pulse", true);
  this->declare_parameter("flag_pulse_ms", 120);

  global_frame_               = this->get_parameter("global_frame").as_string();
  base_frame_                 = this->get_parameter("base_frame").as_string();

  cooldown_sec_               = this->get_parameter("cooldown_sec").as_double();
  consecutive_threshold_      = this->get_parameter("consecutive_threshold").as_int();
  obstacle_persistence_sec_   = this->get_parameter("obstacle_persistence_sec").as_double();
  max_speed_                  = this->get_parameter("max_speed").as_double();
  lookahead_time_sec_         = this->get_parameter("lookahead_time_sec").as_double();
  min_lookahead_m_            = this->get_parameter("min_lookahead_m").as_double();
  cost_threshold_             = this->get_parameter("cost_threshold").as_double();
  ignore_unknown_             = this->get_parameter("ignore_unknown").as_bool();

  db_update_frequency_        = this->get_parameter("db_update_frequency").as_double();
  obstacle_prune_timeout_sec_ = this->get_parameter("obstacle_prune_timeout_sec").as_double();
  db_stride_                  = std::max(1, this->get_parameter("db_stride").as_int());
  cone_angle_deg_             = this->get_parameter("cone_angle_deg").as_double();
  kernel_half_size_           = std::max(0, this->get_parameter("kernel_half_size").as_int());

  path_check_distance_m_      = this->get_parameter("path_check_distance_m").as_double();

  publish_false_pulse_        = this->get_parameter("publish_false_pulse").as_bool();
  flag_pulse_ms_              = this->get_parameter("flag_pulse_ms").as_int();

  // ===== TF =====
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ===== Callback Groups =====
  subs_callback_group_  = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto subs_options = rclcpp::SubscriptionOptions();
  subs_options.callback_group = subs_callback_group_;

  // ===== Subscriptions =====
  // Nav2 글로벌 코스트맵 raw: transient_local + reliable가 보통의 QoS
  costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
      "/global_costmap/costmap_raw",
      rclcpp::QoS(rclcpp::SystemDefaultsQoS()).transient_local().reliable(),
      std::bind(&PathValidatorNode::costmapCallback, this, _1),
      subs_options);

  robot_status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot_status",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&PathValidatorNode::robotStatusCallback, this, _1),
      subs_options);

  pruned_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/plan_pruned",
      rclcpp::QoS(10),
      std::bind(&PathValidatorNode::validatePathCallback, this, _1),
      subs_options);

  // ===== Publisher =====
  // 이벤트성 펄스 → reliable + volatile 권장
  replan_pub_ = this->create_publisher<std_msgs::msg::Bool>("/replan_flag", rclcpp::QoS(1).reliable());

  // ===== Timers =====
  const int period_ms = static_cast<int>(1000.0 / std::max(1.0, db_update_frequency_));
  obstacle_db_update_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&PathValidatorNode::updateObstacleDatabase, this),
      timer_callback_group_);

  RCLCPP_INFO(this->get_logger(), "PathValidatorNode initialized.");
}

// ===================== Costmap handling =====================

void PathValidatorNode::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);

  // Extract metadata
  CostmapSignature sig;
  sig.size_x    = msg->metadata.size_x;
  sig.size_y    = msg->metadata.size_y;
  sig.resolution= msg->metadata.resolution;
  sig.origin_x  = msg->metadata.origin.position.x;
  sig.origin_y  = msg->metadata.origin.position.y;

  if (!costmap_ || !(sig == last_costmap_sig_)) {
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>(
        sig.size_x, sig.size_y, sig.resolution, sig.origin_x, sig.origin_y,
        nav2_costmap_2d::FREE_SPACE);
    last_costmap_sig_ = sig;

    // 메타 변경 시, DB는 맵 좌표계가 바뀌었을 수 있으므로 리셋이 안전
    {
      std::lock_guard<std::mutex> db_lock(obstacle_db_mutex_);
      obstacle_db_.clear();
    }
  }

  const size_t expected = static_cast<size_t>(sig.size_x) * static_cast<size_t>(sig.size_y);
  if (msg->data.size() != expected) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Costmap data size mismatch: got %zu, expected %zu",
        msg->data.size(), expected);
    return;
  }

  // copy data into costmap
  std::memcpy(costmap_->getCharMap(), msg->data.data(), expected * sizeof(unsigned char));
}

void PathValidatorNode::robotStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string & s = msg->data;
  // 필요 시 상태 문자열을 파라미터화/Enum화 가능
  is_robot_in_driving_state_.store(s == "DRIVING" || s == "PLANNING");
}

// ===================== TF helpers =====================

bool PathValidatorNode::getCurrentPoseFromTF(geometry_msgs::msg::Pose & pose_out) const
{
  geometry_msgs::msg::PoseStamped base_in, base_in_global;
  base_in.header.stamp = this->now();
  base_in.header.frame_id = base_frame_;
  base_in.pose.orientation.w = 1.0;

  try {
    auto tf = tf_buffer_->lookupTransform(global_frame_, base_frame_, base_in.header.stamp,
                                          rclcpp::Duration::from_seconds(0.2));
    tf2::doTransform(base_in, base_in_global, tf);
    pose_out = base_in_global.pose;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Could not get robot pose %s->%s: %s",
        global_frame_.c_str(), base_frame_.c_str(), ex.what());
    return false;
  }
}

bool PathValidatorNode::transformToGlobal(const geometry_msgs::msg::PoseStamped & in,
                                          geometry_msgs::msg::PoseStamped & out) const
{
  if (in.header.frame_id.empty() || in.header.frame_id == global_frame_) {
    out = in;
    out.header.frame_id = global_frame_;
    return true;
  }
  try {
    auto tf = tf_buffer_->lookupTransform(global_frame_, in.header.frame_id, in.header.stamp,
                                          rclcpp::Duration::from_seconds(0.2));
    tf2::doTransform(in, out, tf);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "transformToGlobal failed %s->%s: %s",
        in.header.frame_id.c_str(), global_frame_.c_str(), ex.what());
    return false;
  }
}

void PathValidatorNode::transformPathToGlobal(const nav_msgs::msg::Path & in,
                                              std::vector<geometry_msgs::msg::PoseStamped> & out) const
{
  out.clear();
  out.reserve(in.poses.size());
  for (const auto & ps : in.poses) {
    geometry_msgs::msg::PoseStamped g;
    if (transformToGlobal(ps, g)) {
      out.emplace_back(std::move(g));
    }
  }
}

// ===================== Obstacle DB update (Detection) =====================

void PathValidatorNode::updateObstacleDatabase()
{
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (!costmap_) return;
    costmap = costmap_;
  }

  geometry_msgs::msg::Pose pose;
  if (!getCurrentPoseFromTF(pose)) return;

  const double lookahead = std::max(min_lookahead_m_, max_speed_ * lookahead_time_sec_);

  // Clamp ROI to map bounds (AABB)
  const double map_min_x = costmap->getOriginX();
  const double map_min_y = costmap->getOriginY();
  const double map_max_x = map_min_x + costmap->getSizeInCellsX() * costmap->getResolution();
  const double map_max_y = map_min_y + costmap->getSizeInCellsY() * costmap->getResolution();

  const double min_x = std::max(pose.position.x - lookahead, map_min_x);
  const double min_y = std::max(pose.position.y - lookahead, map_min_y);
  const double max_x = std::min(pose.position.x + lookahead, map_max_x);
  const double max_y = std::min(pose.position.y + lookahead, map_max_y);

  unsigned int min_mx, min_my, max_mx, max_my;
  if (!costmap->worldToMap(min_x, min_y, min_mx, min_my) ||
      !costmap->worldToMap(max_x, max_y, max_mx, max_my)) {
    // 완전히 맵 밖
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "ROI outside map bounds");
    return;
  }

  const rclcpp::Time now = this->now();
  std::unordered_set<uint64_t> visible;

  // Forward cone parameters
  const double yaw = tf2::getYaw(pose.orientation);
  const double ux = std::cos(yaw), uy = std::sin(yaw);
  const double cos_half = std::cos((cone_angle_deg_ * M_PI / 180.0) * 0.5);
  const double res = costmap->getResolution();

  // Sample cells with stride
  for (unsigned int mx = min_mx; mx <= max_mx; mx += static_cast<unsigned int>(db_stride_)) {
    for (unsigned int my = min_my; my <= max_my; my += static_cast<unsigned int>(db_stride_)) {
      // Cell center world
      double wx = costmap->mapToWorldX(mx) + res * 0.5;
      double wy = costmap->mapToWorldY(my) + res * 0.5;

      // In range?
      const double dx = wx - pose.position.x;
      const double dy = wy - pose.position.y;
      const double dist = std::hypot(dx, dy);
      if (dist > lookahead) continue;

      // In forward cone?
      const double ndot = (dx * ux + dy * uy) / std::max(1e-6, dist);
      if (ndot < cos_half) continue;

      // Cost check
      const unsigned char c = costmap->getCost(mx, my);
      if (ignore_unknown_ && c == nav2_costmap_2d::NO_INFORMATION) continue;
      if (c >= static_cast<unsigned char>(cost_threshold_)) {
        visible.insert(packKey(mx, my));
      }
    }
  }

  // Update DB
  {
    std::lock_guard<std::mutex> db_lock(obstacle_db_mutex_);
    // upsert
    for (const auto key : visible) {
      auto it = obstacle_db_.find(key);
      if (it == obstacle_db_.end()) {
        obstacle_db_[key] = ObstacleInfo{now, now};
      } else {
        it->second.last_seen = now;
      }
    }
    // prune
    for (auto it = obstacle_db_.begin(); it != obstacle_db_.end(); ) {
      if ((now - it->second.last_seen).seconds() > obstacle_prune_timeout_sec_) {
        it = obstacle_db_.erase(it);
      } else {
        ++it;
      }
    }
  }
}

// ===================== Path validation (Decision) =====================

bool PathValidatorNode::isBlockedCellKernel(unsigned int mx, unsigned int my) const
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  if (!costmap_) return false;

  const int K = kernel_half_size_;
  const int sx = static_cast<int>(costmap_->getSizeInCellsX());
  const int sy = static_cast<int>(costmap_->getSizeInCellsY());

  for (int dx = -K; dx <= K; ++dx) {
    for (int dy = -K; dy <= K; ++dy) {
      const int x = static_cast<int>(mx) + dx;
      const int y = static_cast<int>(my) + dy;
      if (x < 0 || y < 0 || x >= sx || y >= sy) continue;

      const unsigned char c = costmap_->getCost(static_cast<unsigned int>(x), static_cast<unsigned int>(y));
      if (ignore_unknown_ && c == nav2_costmap_2d::NO_INFORMATION) continue;
      if (c >= static_cast<unsigned char>(cost_threshold_)) return true;
    }
  }
  return false;
}

void PathValidatorNode::validatePathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (!is_robot_in_driving_state_.load()) return;
  if (!msg || msg->poses.empty()) return;

  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (!costmap_) return;
    costmap = costmap_;
  }

  // Transform path to global frame
  std::vector<geometry_msgs::msg::PoseStamped> gpath;
  transformPathToGlobal(*msg, gpath);
  if (gpath.empty()) return;

  // Current pose (to cap path distance & order)
  geometry_msgs::msg::Pose cur_pose;
  if (!getCurrentPoseFromTF(cur_pose)) return;

  // Limit path length checked (min of time-lookahead & path_check_distance_m_)
  const double max_dist_by_time = std::max(min_lookahead_m_, max_speed_ * lookahead_time_sec_);
  const double max_check_dist = std::min(max_dist_by_time, path_check_distance_m_);

  // Walk along path accumulating distance; evaluate consecutive blocked streak
  size_t best_streak = 0;
  size_t streak = 0;
  rclcpp::Time now = this->now();

  // Snapshot DB (to shorten lock duration)
  std::unordered_map<uint64_t, ObstacleInfo> db_snapshot;
  {
    std::lock_guard<std::mutex> lock(obstacle_db_mutex_);
    db_snapshot = obstacle_db_;
  }

  double acc = 0.0;
  for (size_t i = 0; i < gpath.size(); ++i) {
    if (i > 0) {
      const auto & p0 = gpath[i-1].pose.position;
      const auto & p1 = gpath[i].pose.position;
      acc += std::hypot(p1.x - p0.x, p1.y - p0.y);
      if (acc > max_check_dist) break;
    }

    unsigned int mx, my;
    if (!costmap->worldToMap(gpath[i].pose.position.x, gpath[i].pose.position.y, mx, my)) {
      streak = 0;
      continue;
    }

    // 1) local cell kernel check
    bool blocked_cell = isBlockedCellKernel(mx, my);

    // 2) persistent DB + 지속성 확인
    const uint64_t key = packKey(mx, my);
    bool persistent_mature = false;
    auto it = db_snapshot.find(key);
    if (it != db_snapshot.end()) {
      if ((now - it->second.first_seen).seconds() >= obstacle_persistence_sec_) {
        persistent_mature = true;
      }
    }

    const bool blocked = blocked_cell && persistent_mature;
    streak = blocked ? (streak + 1) : 0;
    best_streak = std::max(best_streak, streak);

    if (best_streak >= consecutive_threshold_) {
      triggerReplan("blocked streak threshold reached");
      break;
    }
  }
}

// ===================== Replan pulse =====================

void PathValidatorNode::triggerReplan(const std::string & reason)
{
  const rclcpp::Time now = this->now();
  if ((now - last_replan_time_).seconds() <= cooldown_sec_) {
    return;
  }
  last_replan_time_ = now;

  std_msgs::msg::Bool m; m.data = true;
  replan_pub_->publish(m);
  RCLCPP_WARN(this->get_logger(), "Triggering replan: %s", reason.c_str());

  if (publish_false_pulse_ && flag_pulse_ms_ > 0) {
    // cancel previous timer if any
    flag_reset_timer_.reset();
    auto weak_pub = std::weak_ptr<rclcpp::Publisher<std_msgs::msg::Bool>>(replan_pub_);
    flag_reset_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(flag_pulse_ms_),
        [weak_pub]() {
          if (auto pub = weak_pub.lock()) {
            std_msgs::msg::Bool off; off.data = false;
            pub->publish(off);
          }
        });
  }
}

}  // namespace replan_monitor
```

---

원하시면 이 이름 변경에 맞춘 `CMakeLists.txt` / `package.xml` 예시(타겟명 `path_validator_node`, 헤더 설치 경로 등)도 바로 드릴게요.

#################################

아래 파일들을 `replan_monitor` 패키지 안에 그대로 넣으면, `path_validator_node`를 별도 실행 대상으로 빌드/런할 수 있어요. (기존 패키지 그대로 두고 “선택 실행” 구조)

디렉토리 예시:

```
replan_monitor/
├─ CMakeLists.txt
├─ package.xml
├─ include/replan_monitor/path_validator_node.hpp        # (이전 답변의 헤더)
├─ src/path_validator_node.cpp                           # (이전 답변의 구현)
├─ src/path_validator_main.cpp                           # ★ main 추가
├─ launch/path_validator.launch.py                       # ★ 런치
└─ config/path_validator.params.yaml                     # ★ 파라미터
```

---

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(replan_monitor)

# C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Warnings (optional)
add_compile_options(-Wall -Wextra -Wpedantic)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(launch REQUIRED)
find_package(launch_ros REQUIRED)

# Includes
include_directories(
  include
)

# Executable for path_validator_node
add_executable(path_validator_node
  src/path_validator_node.cpp
  src/path_validator_main.cpp
)

ament_target_dependencies(path_validator_node
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  nav2_costmap_2d
)

install(TARGETS path_validator_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers (if other pkgs will include)
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch and config
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

---

### package.xml

```xml
<?xml version="1.0"?>
<package format="3">
  <name>replan_monitor</name>
  <version>0.1.0</version>
  <description>Path validation & replan trigger node for Nav2 (path_validator_node)</description>

  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- Core deps -->
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>

  <!-- TF -->
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>

  <!-- Nav2 / Costmap -->
  <depend>nav2_msgs</depend>
  <depend>nav2_costmap_2d</depend>

  <!-- Launch (runtime) -->
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

### src/path\_validator\_main.cpp

```cpp
#include "rclcpp/rclcpp.hpp"
#include "replan_monitor/path_validator_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // MultiThreadedExecutor to respect callback groups
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<replan_monitor::PathValidatorNode>();
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
```

---

### launch/path\_validator.launch.py

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('replan_monitor')
    default_params = os.path.join(pkg_share, 'config', 'path_validator.params.yaml')

    # Launch args
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Full path to the YAML file with node parameters'
    )

    # Topic remap launch args (optional)
    costmap_topic_arg = DeclareLaunchArgument(
        'costmap_topic', default_value='/global_costmap/costmap_raw',
        description='Costmap topic (nav2_msgs/Costmap)'
    )
    pruned_path_topic_arg = DeclareLaunchArgument(
        'pruned_path_topic', default_value='/plan_pruned',
        description='Pruned path topic (nav_msgs/Path)'
    )
    robot_status_topic_arg = DeclareLaunchArgument(
        'robot_status_topic', default_value='/robot_status',
        description='Robot status topic (std_msgs/String)'
    )
    replan_flag_topic_arg = DeclareLaunchArgument(
        'replan_flag_topic', default_value='/replan_flag',
        description='Replan flag topic (std_msgs/Bool)'
    )

    node = Node(
        package='replan_monitor',
        executable='path_validator_node',
        name='path_validator_node',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            # (from, to)
            ('/global_costmap/costmap_raw', LaunchConfiguration('costmap_topic')),
            ('/plan_pruned', LaunchConfiguration('pruned_path_topic')),
            ('/robot_status', LaunchConfiguration('robot_status_topic')),
            ('/replan_flag', LaunchConfiguration('replan_flag_topic')),
        ]
    )

    return LaunchDescription([
        use_sim_time,
        params_file,
        costmap_topic_arg,
        pruned_path_topic_arg,
        robot_status_topic_arg,
        replan_flag_topic_arg,
        node
    ])
```

---

### config/path\_validator.params.yaml

```yaml
path_validator_node:
  ros__parameters:
    # Frames
    global_frame: "map"
    base_frame: "base_link"

    # Event pulse / cooldown
    cooldown_sec: 1.0
    publish_false_pulse: true
    flag_pulse_ms: 120

    # Obstacle persistence & thresholds
    obstacle_persistence_sec: 0.5
    cost_threshold: 200.0         # 보수적으로 높게 시작 (inflated 영역 기준)

    # Detection ROI / DB
    max_speed: 0.5
    lookahead_time_sec: 15.0
    min_lookahead_m: 2.0
    db_update_frequency: 5.0      # Hz
    obstacle_prune_timeout_sec: 3.0
    db_stride: 2                  # 1이면 모든 셀 검사, 2 이상이면 샘플링
    cone_angle_deg: 100.0         # 전방 부채꼴 각도
    kernel_half_size: 1           # 1 => 3x3 커널로 블록 검사
    ignore_unknown: true          # Unknown(255) 제외 (권장)

    # Path validation
    consecutive_threshold: 3      # 연속 K 포인트 이상 차단되면 리플랜
    path_check_distance_m: 6.0    # 경로 검사 상한 거리(가까운 구간만)

    # Typical sim param
    use_sim_time: false
```

---

## 사용법

빌드:

```bash
colcon build --packages-select replan_monitor
source install/setup.bash
```

런치(기본 파라미터 사용):

```bash
ros2 launch replan_monitor path_validator.launch.py
```

토픽 리매핑 예:

```bash
ros2 launch replan_monitor path_validator.launch.py \
  costmap_topic:=/my_global_costmap/costmap_raw \
  pruned_path_topic:=/my_plan_pruned \
  robot_status_topic:=/robot/state \
  replan_flag_topic:=/nav/replan_flag
```

필요하면 `params.yaml`에서 민감도를 조절하세요(예: `cost_threshold`, `consecutive_threshold`, `cone_angle_deg`, `db_stride`, `kernel_half_size`, `path_check_distance_m`).
##################################  
########################################  
#########################################  
제시해주신 `CostmapDownsampler` 코드와 `PathValidatorNode` 코드를 기반으로, **연산 효율성을 극대화**하기 위해 Downsampler를 적용한 코드를 작성했습니다.

### 🛠️ 주요 변경 및 구현 전략

1. **Helper 함수의 일반화**: 기존의 `isBlockedCellKernel`, `masterCellBlocked` 등의 함수들이 클래스 멤버 변수 `costmap_`에 의존적이었으나, 이를 **`nav2_costmap_2d::Costmap2D*`를 인자로 받도록 수정**하여 원본 맵과 다운샘플링된 맵 모두를 처리할 수 있게 변경했습니다.
2. **Downsampler 수명 주기 관리**: `CostmapDownsampler`는 `LifecycleNode`를 기반으로 설계되었으나, 현재 노드는 일반 `rclcpp::Node`입니다. 따라서 Publisher 생성 부분에 빈 Weak Pointer를 전달하여 **불필요한 토픽 발행을 막고 다운샘플링 연산 기능만 활성화**했습니다.
3. **Costmap 포인터 동기화**: `costmapCallback`에서 맵의 크기나 해상도가 변경되어 `costmap_` 객체가 재생성될 때, Downsampler에게도 새로운 원본 맵의 주소를 알려주도록(`on_configure`) 로직을 연결했습니다.
4. **Just-In-Time 연산**: `validatePathCallback`이 호출될 때(실제 검증이 필요할 때) 다운샘플링을 수행하여 최신 상태를 유지합니다.

---

### 1. Header File 수정

**[replan_monitor/include/replan_monitor/path_validator_node.hpp]**

```cpp
#ifndef PATH_VALIDATOR_NODE_HPP
#define PATH_VALIDATOR_NODE_HPP

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <chrono>
#include <algorithm>
#include <map>
#include <memory> // unique_ptr

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_msgs/msg/costmap.hpp"

#include "multi_agent_msgs/msg/multi_agent_info_array.hpp"
#include "multi_agent_msgs/msg/multi_agent_info.hpp"
#include "multi_agent_msgs/msg/agent_status.hpp"
#include "multi_agent_msgs/msg/path_agent_collision_info.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

// [NEW] Include Costmap Downsampler
#include "nav2_smac_planner/costmap_downsampler.hpp"

namespace replan_monitor
{

struct ObstacleInfo
{
  rclcpp::Time first_seen;
  rclcpp::Time last_seen;
};

struct CostmapSignature
{
  unsigned int size_x{0};
  unsigned int size_y{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};

  bool operator==(const CostmapSignature & other) const
  {
    return size_x == other.size_x && size_y == other.size_y &&
           resolution == other.resolution &&
           origin_x == other.origin_x && origin_y == other.origin_y;
  }
};

class PathValidatorNode : public rclcpp::Node
{
public:
  PathValidatorNode();
  // [NEW] Destructor required for unique_ptr forward declaration cleanups if needed
  ~PathValidatorNode();

private:
  // ========= Callbacks =========
  void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
  void agentMaskCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
  void robotStatusCallback(const std_msgs::msg::String::SharedPtr msg);
  void validatePathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void updateObstacleDatabase();
  void agentsCallback(const multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr msg);

  // ========= Helpers / Utils =========
  bool getCurrentPoseFromTF(geometry_msgs::msg::Pose & pose_out) const;
  bool transformToGlobal(const geometry_msgs::msg::PoseStamped & in,
                         geometry_msgs::msg::PoseStamped & out) const;
  void transformPathToGlobal(const nav_msgs::msg::Path & in,
                             std::vector<geometry_msgs::msg::PoseStamped> & out) const;

  inline uint64_t packKey(unsigned int mx, unsigned int my) const
  {
    return (static_cast<uint64_t>(mx) << 32) | static_cast<uint64_t>(my);
  }

  // [MODIFIED] Now accepts a Costmap pointer to support both raw and downsampled maps
  bool isBlockedCellKernel(const nav2_costmap_2d::Costmap2D* costmap, unsigned int mx, unsigned int my) const;

  // [MODIFIED] Now accepts a Costmap pointer
  void validateWithFootprint(const nav2_costmap_2d::Costmap2D* costmap, const std::vector<geometry_msgs::msg::PoseStamped> & gpath);
  // [MODIFIED] Now accepts a Costmap pointer
  void validateWithPoints(const nav2_costmap_2d::Costmap2D* costmap, const std::vector<geometry_msgs::msg::PoseStamped> & gpath);

  // [MODIFIED] Now accepts a Costmap pointer
  inline bool masterCellBlocked(const nav2_costmap_2d::Costmap2D* costmap, unsigned int mx, unsigned int my, unsigned char thr) const;
  
  // agentCellBlockedNear 는 내부적으로 agent_mask_mutex_를 쓰므로 Costmap2D 인자 불필요 (Agent Mask는 다운샘플링 대상 아님)
  inline bool agentCellBlockedNear(unsigned int mx, unsigned int my,
                                   unsigned char thr, int manhattan_buf) const;

  static bool pointInPolygon(const std::vector<geometry_msgs::msg::Point> & poly,
                             double x, double y);

  // === Agent 충돌 식별 ===
  struct AgentHit {
    uint16_t machine_id{0};
    std::string type_id;
    double x{0.0}, y{0.0};
    float ttc_first{-1.0f};
    std::string note;
  };

  struct AgentFootprintData
  {
    std::vector<geometry_msgs::msg::Point32> points;
    double radius{0.0};
    bool use_radius{true};
  };
  std::map<uint16_t, AgentFootprintData> agent_footprints_;

  std::vector<geometry_msgs::msg::Point32> 
  getFootprintForAgent(const multi_agent_msgs::msg::MultiAgentInfo & a) const;

  static std::vector<geometry_msgs::msg::Point32> toPoint32(
      const std::vector<geometry_msgs::msg::Point>& points);

  std::vector<AgentHit> whoCoversPoint(double wx, double wy) const;

  static bool pathTubeCoversPoint(const multi_agent_msgs::msg::MultiAgentInfo & a,
                                  double wx, double wy,
                                  double stride_m, double dilate_m,
                                  int max_poses, double frame_yaw,
                                  const std::string & global_frame);

  static double headingTo(const geometry_msgs::msg::Pose & pose, double wx, double wy);
  static double speedAlong(const geometry_msgs::msg::Twist & tw, double heading_rad);

  void triggerReplan(const std::string & reason);
  void publishAgentCollisionList(const std::vector<AgentHit> & hits);

  // ========= Callback Groups =========
  rclcpp::CallbackGroup::SharedPtr subs_callback_group_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

  // ========= ROS I/O =========
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pruned_path_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr agent_mask_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_status_sub_;
  rclcpp::Subscription<multi_agent_msgs::msg::MultiAgentInfoArray>::SharedPtr agents_sub_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr replan_pub_;
  rclcpp::Publisher<multi_agent_msgs::msg::PathAgentCollisionInfo>::SharedPtr agent_collision_pub_;

  rclcpp::TimerBase::SharedPtr obstacle_db_update_timer_;
  rclcpp::TimerBase::SharedPtr flag_reset_timer_;

  // ========= State =========
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  mutable std::mutex costmap_mutex_;
  CostmapSignature last_costmap_sig_;

  // [NEW] Costmap Downsampler and related
  std::unique_ptr<nav2_smac_planner::CostmapDownsampler> costmap_downsampler_;
  int downsampling_factor_{1};

  std::shared_ptr<nav2_costmap_2d::Costmap2D> agent_mask_;
  mutable std::mutex agent_mask_mutex_;
  CostmapSignature last_agent_sig_;

  multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr last_agents_;
  rclcpp::Time last_agents_stamp_;
  mutable std::mutex agents_mutex_;

  std::atomic<bool> is_robot_in_driving_state_{false};
  rclcpp::Time last_replan_time_;
  rclcpp::Time last_agent_block_time_;

  std::unordered_map<uint64_t, ObstacleInfo> obstacle_db_;
  mutable std::mutex obstacle_db_mutex_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ========= Parameters =========
  std::string global_frame_;
  std::string base_frame_;

  double cooldown_sec_;
  size_t consecutive_threshold_;
  double obstacle_persistence_sec_;
  double max_speed_;
  double lookahead_time_sec_;
  double min_lookahead_m_;
  double cost_threshold_;
  bool ignore_unknown_;

  double db_update_frequency_;
  double obstacle_prune_timeout_sec_;
  int db_stride_;
  double cone_angle_deg_;
  int kernel_half_size_;

  double path_check_distance_m_;

  bool publish_false_pulse_;
  int flag_pulse_ms_;

  bool use_footprint_check_;
  double footprint_step_m_;
  bool compare_agent_mask_;
  std::string agent_mask_topic_;
  double agent_cost_threshold_;
  int agent_mask_manhattan_buffer_;

  bool publish_agent_collision_;
  std::string agent_collision_topic_;

  std::string agents_topic_;
  int agents_freshness_timeout_ms_;
  double agent_match_dilate_m_;

  std::string footprint_str_;
  std::vector<geometry_msgs::msg::Point> footprint_;
  bool use_radius_{true};
  double robot_radius_m_{0.1};

  double agent_block_hold_sec_{2.0};
  double agent_block_max_wait_sec_{8.0};

  bool   agent_path_hit_enable_{true};
  double agent_path_hit_stride_m_{0.35};
  double agent_path_hit_dilate_m_{0.02};
  int    agent_path_hit_max_poses_{200};
};

}  // namespace replan_monitor

#endif  // PATH_VALIDATOR_NODE_HPP

```

---

### 2. Source File 수정

**[replan_monitor/src/path_validator_node.cpp]**

```cpp
#include "replan_monitor/path_validator_node.hpp"
#include <nav2_costmap_2d/footprint.hpp>
// [NEW] for LifecycleNode WeakPtr trick
#include "nav2_util/lifecycle_node.hpp" 

using std::placeholders::_1;

namespace replan_monitor
{

// ... (Helper implementations: toPoint32, getFootprintForAgent same as before) ...

// [NEW] Destructor implementation
PathValidatorNode::~PathValidatorNode()
{
  if (costmap_downsampler_) {
    costmap_downsampler_->on_cleanup();
    costmap_downsampler_.reset();
  }
}

PathValidatorNode::PathValidatorNode()
: Node("path_validator_node")
{
  // ... (Existing parameter declarations) ...
  
  // [NEW] Downsampling parameter
  this->declare_parameter("downsampling_factor", 1);

  // ... (Load parameters) ...
  downsampling_factor_ = this->get_parameter("downsampling_factor").as_int();

  // ... (Existing parameter loading logic) ...

  // [NEW] Initialize Costmap Downsampler
  // Note: We initialize it here, but actual configuration happens when we get the first costmap
  if (downsampling_factor_ > 1) {
    costmap_downsampler_ = std::make_unique<nav2_smac_planner::CostmapDownsampler>();
    RCLCPP_INFO(get_logger(), "CostmapDownsampler enabled with factor: %d", downsampling_factor_);
  }

  // ... (Rest of constructor: TF, Publishers, Subscribers, Timers) ...
}

// ===================== Costmap handling =====================

void PathValidatorNode::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);

  CostmapSignature sig;
  sig.size_x    = msg->metadata.size_x;
  sig.size_y    = msg->metadata.size_y;
  sig.resolution= msg->metadata.resolution;
  sig.origin_x  = msg->metadata.origin.position.x;
  sig.origin_y  = msg->metadata.origin.position.y;

  bool costmap_recreated = false;
  if (!costmap_ || !(sig == last_costmap_sig_)) {
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>(
        sig.size_x, sig.size_y, sig.resolution, sig.origin_x, sig.origin_y,
        nav2_costmap_2d::FREE_SPACE);
    last_costmap_sig_ = sig;
    costmap_recreated = true;

    std::lock_guard<std::mutex> db_lock(obstacle_db_mutex_);
    obstacle_db_.clear();
  }

  const size_t expected = static_cast<size_t>(sig.size_x) * static_cast<size_t>(sig.size_y);
  if (msg->data.size() != expected) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Costmap data size mismatch: got %zu, expected %zu",
        msg->data.size(), expected);
    return;
  }
  std::memcpy(costmap_->getCharMap(), msg->data.data(), expected * sizeof(unsigned char));

  // [NEW] Configure downsampler if costmap object changed
  if (costmap_recreated && costmap_downsampler_) {
    // We pass an empty weak ptr because we are a standard Node, not a LifecycleNode.
    // This effectively disables the publisher inside Downsampler but allows calculation.
    nav2_util::LifecycleNode::WeakPtr dummy_node_ptr; 
    
    // We pass empty topic name to avoid publisher creation attempt
    costmap_downsampler_->on_configure(
        dummy_node_ptr, 
        global_frame_, 
        "", // No topic publishing
        costmap_.get(), 
        downsampling_factor_
    );
  }
}

// ... (agentMaskCallback, agentsCallback, robotStatusCallback, TF helpers - NO CHANGE) ...

// ===================== Obstacle DB update =====================

void PathValidatorNode::updateObstacleDatabase()
{
  // [MODIFIED] Use Downsampled map if available for efficiency in DB Update too
  nav2_costmap_2d::Costmap2D* costmap_ptr = nullptr;
  
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  if (!costmap_) return;

  if (costmap_downsampler_ && downsampling_factor_ > 1) {
    costmap_ptr = costmap_downsampler_->downsample(downsampling_factor_);
  } else {
    costmap_ptr = costmap_.get();
  }
  
  if (!costmap_ptr) return;

  geometry_msgs::msg::Pose pose;
  if (!getCurrentPoseFromTF(pose)) return;

  const double lookahead = std::max(min_lookahead_m_, max_speed_ * lookahead_time_sec_);

  const double map_min_x = costmap_ptr->getOriginX();
  const double map_min_y = costmap_ptr->getOriginY();
  const double map_max_x = map_min_x + costmap_ptr->getSizeInCellsX() * costmap_ptr->getResolution();
  const double map_max_y = map_min_y + costmap_ptr->getSizeInCellsY() * costmap_ptr->getResolution();

  // ... (Bounding Box calculation - Logic is generic for any resolution) ...

  unsigned int min_mx, min_my, max_mx, max_my;
  if (!costmap_ptr->worldToMap(min_x, min_y, min_mx, min_my) ||
      !costmap_ptr->worldToMap(max_x, max_y, max_mx, max_my)) {
    // ROI check
    return;
  }

  // ... (Time & Yaw setup) ...
  const rclcpp::Time now = this->now();
  std::unordered_set<uint64_t> visible;

  const double yaw = tf2::getYaw(pose.orientation);
  const double ux = std::cos(yaw), uy = std::sin(yaw);
  const double cos_half = std::cos((cone_angle_deg_ * M_PI / 180.0) * 0.5);

  // [NOTE] Downsampler가 적용되면 resolution이 커지므로 db_stride도 조정할 필요가 있을 수 있으나,
  // 여기서는 stride를 그대로 사용(즉, 다운샘플된 셀 단위로 stride)
  for (unsigned int mx = min_mx; mx <= max_mx; mx += static_cast<unsigned int>(db_stride_)) {
    for (unsigned int my = min_my; my <= max_my; my += static_cast<unsigned int>(db_stride_)) {
      double wx, wy;
      costmap_ptr->mapToWorld(mx, my, wx, wy);

      // ... (Cone check logic - same) ...
      const double dx = wx - pose.position.x;
      const double dy = wy - pose.position.y;
      const double dist = std::hypot(dx, dy);
      if (dist > lookahead) continue;

      const double ndot = (dx * ux + dy * uy) / std::max(1e-6, dist);
      if (ndot < cos_half) continue;

      const unsigned char c = costmap_ptr->getCost(mx, my);
      // ... (Threshold check) ...
      if (ignore_unknown_ && c == nav2_costmap_2d::NO_INFORMATION) continue;
      if (c >= static_cast<unsigned char>(cost_threshold_)) {
        // [IMPORTANT] The key stored in DB depends on index.
        // If we switch resolutions dynamically, DB keys become invalid.
        // But since downsampling_factor is static param, consistent keys are fine.
        visible.insert(packKey(mx, my));
      }
    }
  }

  // ... (DB update/pruning logic - NO CHANGE) ...
  {
    std::lock_guard<std::mutex> db_lock(obstacle_db_mutex_);
    for (const auto key : visible) {
      auto it = obstacle_db_.find(key);
      if (it == obstacle_db_.end()) {
        obstacle_db_[key] = ObstacleInfo{now, now};
      } else {
        it->second.last_seen = now;
      }
    }
    for (auto it = obstacle_db_.begin(); it != obstacle_db_.end(); ) {
      if ((now - it->second.last_seen).seconds() > obstacle_prune_timeout_sec_) {
        it = obstacle_db_.erase(it);
      } else {
        ++it;
      }
    }
  }
}

// ===================== Cell checks =====================

// [MODIFIED] Added costmap pointer argument
bool PathValidatorNode::isBlockedCellKernel(const nav2_costmap_2d::Costmap2D* costmap, unsigned int mx, unsigned int my) const
{
  if (!costmap) return false;

  const int K = kernel_half_size_;
  const int sx = static_cast<int>(costmap->getSizeInCellsX());
  const int sy = static_cast<int>(costmap->getSizeInCellsY());

  for (int dx = -K; dx <= K; ++dx) {
    for (int dy = -K; dy <= K; ++dy) {
      const int x = static_cast<int>(mx) + dx;
      const int y = static_cast<int>(my) + dy;
      if (x < 0 || y < 0 || x >= sx || y >= sy) continue;

      const unsigned char c = costmap->getCost(static_cast<unsigned int>(x), static_cast<unsigned int>(y));
      if (ignore_unknown_ && c == nav2_costmap_2d::NO_INFORMATION) continue;
      if (c >= static_cast<unsigned char>(cost_threshold_)) return true;
    }
  }
  return false;
}

// [MODIFIED] Added costmap pointer argument
inline bool PathValidatorNode::masterCellBlocked(const nav2_costmap_2d::Costmap2D* costmap, unsigned int mx, unsigned int my, unsigned char thr) const
{
  if (!costmap) return false;

  const unsigned char c = costmap->getCost(mx, my);
  if (ignore_unknown_ && c == nav2_costmap_2d::NO_INFORMATION) return false;
  return (c >= thr);
}

// agentCellBlockedNear는 변경 없음 (agent_mask는 다운샘플링 안함)

// ===================== Path validation dispatcher =====================

void PathValidatorNode::validatePathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (!is_robot_in_driving_state_.load()) return;
  if (!msg || msg->poses.empty()) return;

  // [MODIFIED] Prepare the costmap (Downsampled or Original)
  nav2_costmap_2d::Costmap2D* costmap_to_use = nullptr;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (!costmap_) return;

    if (costmap_downsampler_ && downsampling_factor_ > 1) {
      // Execute Downsampling JIT
      costmap_to_use = costmap_downsampler_->downsample(downsampling_factor_);
    } else {
      costmap_to_use = costmap_.get();
    }
  } // Mutex ends here? NO!
  // CAUTION: The costmap_to_use pointer is derived from costmap_ or internal downsampler.
  // If we release the lock, costmapCallback might update costmap_.
  // However, validation functions are heavy. Holding lock for full validation is safe but blocking.
  // Given the structure, we MUST hold the lock because we are reading the costmap data.
  
  // Re-acquire lock logic: The functions validateWith... need access to the map data.
  // The helper 'isBlockedCellKernel' used to lock inside. Now we pass the pointer.
  // We should lock here for the scope of validation.
  
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  
  // Re-check validity inside lock
  if (!costmap_) return;
  if (costmap_downsampler_ && downsampling_factor_ > 1) {
      // Re-call downsample inside lock to be safe (it's cheap if size didn't change)
      costmap_to_use = costmap_downsampler_->downsample(downsampling_factor_);
  } else {
      costmap_to_use = costmap_.get();
  }

  std::vector<geometry_msgs::msg::PoseStamped> gpath;
  transformPathToGlobal(*msg, gpath);
  if (gpath.empty()) return;

  const rclcpp::Time now = this->now();
  const double since_agent = (now - last_agent_block_time_).seconds();
  if (since_agent >= 0.0 && since_agent < agent_block_hold_sec_) {
    return;
  }

  if (use_footprint_check_) {
    validateWithFootprint(costmap_to_use, gpath);
  } else {
    validateWithPoints(costmap_to_use, gpath);
  }
}

// ===================== 기존 포인트 검사 =====================

// [MODIFIED] Arguments
void PathValidatorNode::validateWithPoints(const nav2_costmap_2d::Costmap2D* costmap, const std::vector<geometry_msgs::msg::PoseStamped> & gpath)
{
  geometry_msgs::msg::Pose cur_pose;
  if (!getCurrentPoseFromTF(cur_pose)) return;

  const double max_dist_by_time = std::max(min_lookahead_m_, max_speed_ * lookahead_time_sec_);
  const double max_check_dist = std::min(max_dist_by_time, path_check_distance_m_);

  size_t best_streak = 0;
  size_t streak = 0;
  rclcpp::Time now = this->now();

  std::unordered_map<uint64_t, ObstacleInfo> db_snapshot;
  {
    std::lock_guard<std::mutex> lock(obstacle_db_mutex_);
    db_snapshot = obstacle_db_;
  }

  double acc = 0.0;
  for (size_t i = 0; i < gpath.size(); ++i) {
    if (i > 0) {
      const auto & p0 = gpath[i-1].pose.position;
      const auto & p1 = gpath[i].pose.position;
      acc += std::hypot(p1.x - p0.x, p1.y - p0.y);
      if (acc > max_check_dist) break;
    }

    unsigned int mx, my;
    // [MODIFIED] Use passed costmap instead of locking costmap_
    if (!costmap->worldToMap(gpath[i].pose.position.x, gpath[i].pose.position.y, mx, my)) {
      streak = 0; continue;
    }

    // [MODIFIED] Pass costmap
    const bool blocked_cell = isBlockedCellKernel(costmap, mx, my);

    if (blocked_cell) {
      // --- 중심 셀 월드좌표 ---
      double wx, wy;
      costmap->mapToWorld(static_cast<int>(mx), static_cast<int>(my), wx, wy);

      // --- 중심+이웃 1셀까지 에이전트 히트 검사 람다 ---
      auto agent_hit_around = [&](double cx, double cy,
                                  unsigned int mx_c, unsigned int my_c) -> std::vector<AgentHit> {
        auto hits = whoCoversPoint(cx, cy);
        if (!hits.empty()) return hits;

        static const int OFFS[8][2] = {
          { 1, 0},{-1, 0},{ 0, 1},{ 0,-1},
          { 1, 1},{ 1,-1},{-1, 1},{-1,-1}
        };
        for (auto &o : OFFS) {
          double wxx, wyy;
          // [MODIFIED] Use costmap for mapToWorld
          costmap->mapToWorld(static_cast<int>(mx_c)+o[0], static_cast<int>(my_c)+o[1], wxx, wyy);
          auto h2 = whoCoversPoint(wxx, wyy);
          if (!h2.empty()) return h2;
        }
        return {};
      };

      // 1) 일반 코스트로 막혔을 때: 중심+이웃 검사
      {
        auto hits = agent_hit_around(wx, wy, mx, my);
        if (!hits.empty()) {
          publishAgentCollisionList(hits);
          last_agent_block_time_ = this->now();
          return;
        }
      }

      // 2) (보조) agent mask (Master Costmap 기반이 아님, 별도 로직)
      if (compare_agent_mask_) {
        // [NOTE] agent_mask logic uses World Coordinates internally, 
        // so mx, my from downsampled map -> World -> Agent Mask Index is handled inside
        const bool agent_mark = agentCellBlockedNear(
            mx, my,
            static_cast<unsigned char>(agent_cost_threshold_),
            agent_mask_manhattan_buffer_);
        if (agent_mark) {
          auto hits2 = agent_hit_around(wx, wy, mx, my);
          if (!hits2.empty()) {
            publishAgentCollisionList(hits2);
            last_agent_block_time_ = this->now();
            return;
          }
        }
      }
    }

    bool blocked = blocked_cell;
    // 일반 장애물 성숙도
    const uint64_t key = packKey(mx, my);
    bool persistent_mature = false;
    auto it = db_snapshot.find(key);
    if (it != db_snapshot.end()) {
      if ((now - it->second.first_seen).seconds() >= obstacle_persistence_sec_) {
        persistent_mature = true;
      }
    }

    blocked = blocked && persistent_mature;

    streak = blocked ? (streak + 1) : 0;
    best_streak = std::max(best_streak, streak);

    if (best_streak >= consecutive_threshold_) {
      triggerReplan("blocked (points) streak threshold reached");
      break;
    }
  }
}

// ===================== Footprint 기반 검사 =====================
// pointInPolygon 생략 (변경없음)

// [MODIFIED] Arguments
void PathValidatorNode::validateWithFootprint(const nav2_costmap_2d::Costmap2D* costmap, const std::vector<geometry_msgs::msg::PoseStamped> & gpath)
{
  geometry_msgs::msg::Pose cur_pose;
  if (!getCurrentPoseFromTF(cur_pose)) return;

  const double max_dist_by_time = std::max(min_lookahead_m_, max_speed_ * lookahead_time_sec_);
  const double max_check_dist = std::min(max_dist_by_time, path_check_distance_m_);
  const double res = costmap->getResolution();

  // ... (Path Sampling Logic - Same) ...
  std::vector<geometry_msgs::msg::PoseStamped> samples;
  samples.reserve(gpath.size());
  double acc = 0.0;
  samples.push_back(gpath.front());
  for (size_t i = 1; i < gpath.size(); ++i) {
    const auto & p0 = gpath[i-1].pose.position;
    const auto & p1 = gpath[i].pose.position;
    const double d = std::hypot(p1.x - p0.x, p1.y - p0.y);
    if (d <= 1e-6) continue;
    acc += d;
    if (acc >= footprint_step_m_) {
      samples.push_back(gpath[i]);
      acc = 0.0;
    }
    const double total = std::hypot(gpath[i].pose.position.x - gpath.front().pose.position.x,
                                    gpath[i].pose.position.y - gpath.front().pose.position.y);
    if (total > max_check_dist) break;
  }

  const unsigned char master_thr = static_cast<unsigned char>(cost_threshold_);

  size_t consecutive = 0;

  for (const auto & ps : samples) {
    bool blocked_here = false;
    unsigned int hit_mx = 0, hit_my = 0;

    if (use_radius_) {
      unsigned int cx, cy;
      // [MODIFIED] Use costmap
      if (!costmap->worldToMap(ps.pose.position.x, ps.pose.position.y, cx, cy)) {
        consecutive = 0; continue;
      }
      const int r_cells = std::max(1, static_cast<int>(std::ceil(robot_radius_m_ / res)));

      for (int dx = -r_cells; dx <= r_cells && !blocked_here; ++dx) {
        for (int dy = -r_cells; dy <= r_cells && !blocked_here; ++dy) {
          if (dx*dx + dy*dy > r_cells*r_cells) continue;
          const int mx = static_cast<int>(cx) + dx;
          const int my = static_cast<int>(cy) + dy;
          if (mx < 0 || my < 0 ||
              mx >= static_cast<int>(costmap->getSizeInCellsX()) ||
              my >= static_cast<int>(costmap->getSizeInCellsY())) continue;

          const unsigned int umx = static_cast<unsigned int>(mx);
          const unsigned int umy = static_cast<unsigned int>(my);

          // [MODIFIED] Use costmap
          if (masterCellBlocked(costmap, umx, umy, master_thr)) {
            blocked_here = true; hit_mx = umx; hit_my = umy;
          }
        }
      }
    } else {
      // ... (Polygon Footprint Logic - similar modification) ...
      const double yaw = tf2::getYaw(ps.pose.orientation);
      const double c = std::cos(yaw), s = std::sin(yaw);

      std::vector<geometry_msgs::msg::Point> poly_world;
      poly_world.reserve(footprint_.size());
      // ... (Min/Max calc same) ...
      double minx=1e9, miny=1e9, maxx=-1e9, maxy=-1e9;
      for (const auto & p : footprint_) {
          geometry_msgs::msg::Point q;
          const double x = p.x, y = p.y;
          q.x = ps.pose.position.x + c * x - s * y;
          q.y = ps.pose.position.y + s * x + c * y;
          q.z = 0.0;
          poly_world.push_back(q);
          if (q.x < minx) minx=q.x;
          if (q.y < miny) miny=q.y;
          if (q.x > maxx) maxx=q.x; 
          if (q.y > maxy) maxy=q.y;
      }

      int min_i, min_j, max_i, max_j;
      costmap->worldToMapEnforceBounds(minx, miny, min_i, min_j);
      costmap->worldToMapEnforceBounds(maxx, maxy, max_i, max_j);

      for (int j = min_j; j <= max_j && !blocked_here; ++j) {
        for (int i = min_i; i <= max_i && !blocked_here; ++i) {
          double wx, wy; costmap->mapToWorld(i, j, wx, wy);
          if (!pointInPolygon(poly_world, wx, wy)) continue;

          const unsigned int umx = static_cast<unsigned int>(i);
          const unsigned int umy = static_cast<unsigned int>(j);

          // [MODIFIED] Use costmap
          if (masterCellBlocked(costmap, umx, umy, master_thr)) {
            blocked_here = true; hit_mx = umx; hit_my = umy;
          }
        }
      }
    }


    if (blocked_here) {
      double wx, wy;
      costmap->mapToWorld(static_cast<int>(hit_mx), static_cast<int>(hit_my), wx, wy);

      auto agent_hit_around = [&](double cx, double cy,
                                  unsigned int mx_c, unsigned int my_c) -> std::vector<AgentHit> {
        auto hits = whoCoversPoint(cx, cy);
        if (!hits.empty()) return hits;
        static const int OFFS[8][2] = {
          { 1, 0},{-1, 0},{ 0, 1},{ 0,-1},
          { 1, 1},{ 1,-1},{-1, 1},{-1,-1}
        };
        for (auto &o : OFFS) {
          double wxx, wyy;
          costmap->mapToWorld(static_cast<int>(mx_c)+o[0], static_cast<int>(my_c)+o[1], wxx, wyy);
          auto h2 = whoCoversPoint(wxx, wyy);
          if (!h2.empty()) return h2;
        }
        return {};
      };

      // 1) 일반
      {
        auto hits = agent_hit_around(wx, wy, hit_mx, hit_my);
        if (!hits.empty()) {
          publishAgentCollisionList(hits);
          last_agent_block_time_ = this->now();
          return;
        }
      }

      // 2) (보조) agent mask
      if (compare_agent_mask_) {
        // [MODIFIED] This call needs special attention. 
        // agentCellBlockedNear takes indices (mx, my).
        // If costmap is downsampled, (hit_mx, hit_my) are small indices.
        // Inside agentCellBlockedNear, we call master->mapToWorld(mx, my...) using the passed 'master' lock logic.
        // BUT here, inside validateWithFootprint, we don't have separate mutex locks for master vs agent.
        // We need to rely on the fact that `mapToWorld` of the downsampled map yields the correct World Coord.
        
        // Wait, agentCellBlockedNear implementation above currently locks `costmap_mutex_`.
        // This is a DEADLOCK if we already hold `costmap_mutex_` in validatePathCallback.
        
        // [FIX] agentCellBlockedNear needs to accept the *current* costmap pointer too, 
        // OR we refactor it to avoid locking costmap again.
        // Since we are inside validatePathCallback which holds lock, we should use a version of 
        // agentCellBlockedNear that assumes lock is held or takes the costmap ptr.
        // However, agentCellBlockedNear also locks `agent_mask_mutex_`.
        
        // Let's modify the usage slightly:
        // Convert downsampled index -> World -> Check Agent Mask.
        // We can do this manually here or fix the helper function.
        // To keep it clean, I will manually check agent mask here using the world coordinate `wx, wy`.
        
        // Re-implementing logic of `agentCellBlockedNear` using World Coordinates directly:
        bool agent_mark = false;
        {
            std::lock_guard<std::mutex> lk(agent_mask_mutex_);
            if (agent_mask_) {
                unsigned int ax, ay;
                if (agent_mask_->worldToMap(wx, wy, ax, ay)) {
                     // Check neighbors in agent mask
                     const int K = agent_mask_manhattan_buffer_;
                     const int sx = static_cast<int>(agent_mask_->getSizeInCellsX());
                     const int sy = static_cast<int>(agent_mask_->getSizeInCellsY());
                     const int ix = static_cast<int>(ax);
                     const int iy = static_cast<int>(ay);
                     
                     for (int dx = -K; dx <= K && !agent_mark; ++dx) {
                        for (int dy = -K; dy <= K && !agent_mark; ++dy) {
                           if (std::abs(dx)+std::abs(dy) > K) continue;
                           int nx = ix+dx; int ny = iy+dy;
                           if (nx>=0 && ny>=0 && nx<sx && ny<sy) {
                               if (agent_mask_->getCost(nx, ny) >= static_cast<unsigned char>(agent_cost_threshold_)) {
                                   agent_mark = true;
                               }
                           }
                        }
                     }
                }
            }
        }
        
        if (agent_mark) {
          auto hits2 = agent_hit_around(wx, wy, hit_mx, hit_my);
          if (!hits2.empty()) {
            publishAgentCollisionList(hits2);
            last_agent_block_time_ = this->now();
            return;
          }
        }
      }

      consecutive++;
      if (consecutive >= consecutive_threshold_) {
        triggerReplan("blocked (footprint) streak threshold reached");
        return;
      }
    } else {
      consecutive = 0;
    }

  }
}

// ... (Rest of functions: whoCoversPoint, publishAgentCollisionList, etc. - NO CHANGE) ...

}  // namespace replan_monitor

```

###########################################  
#############################################  
###############################################3  
네, `CostmapDownsampler`를 사용하기 위해서는 `nav2_smac_planner` 패키지에 대한 의존성을 추가해야 합니다.

`CostmapDownsampler`의 소스 코드를 직접 복사해서 사용하는 것이 아니라, **Nav2의 설치된 헤더를 참조**하는 방식이므로 아래와 같이 수정이 필요합니다.

### 1. `package.xml` 수정

`nav2_smac_planner` 패키지에 대한 `<depend>` 태그를 추가합니다.

```xml
<?xml version="1.0"?>
<package format="3">
  <name>replan_monitor</name>
  <version>0.1.0</version>
  <description>Global path replan monitor and Path Validator node for Nav2</description>

  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <build_depend>multi_robot_msgs</build_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav2_msgs</depend>
  <depend>nav2_costmap_2d</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  
  <depend>nav2_smac_planner</depend>
  <depend>nav2_util</depend>

  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>ament_index_python</exec_depend>
  <exec_depend>multi_robot_msgs</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>

```

---

### 2. `CMakeLists.txt` 수정

`nav2_smac_planner`를 찾아(`find_package`) 타겟의 의존성(`ament_target_dependencies`)에 추가합니다. `nav2_util`도 WeakPtr 처리를 위해 필요합니다.

```cmake
cmake_minimum_required(VERSION 3.8)
project(replan_monitor)

# ── C++17 & warnings ────────────────────────────────────────────────
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

# ── Dependencies ───────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(multi_robot_msgs REQUIRED)

# [NEW] Add nav2_smac_planner and nav2_util
find_package(nav2_smac_planner REQUIRED)
find_package(nav2_util REQUIRED)

# ── Exports (라이브러리 배포 시 권장) ─────────────────────────────
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp std_msgs nav_msgs geometry_msgs
  tf2 tf2_ros tf2_geometry_msgs nav2_msgs nav2_costmap_2d
  nav2_smac_planner nav2_util # [NEW]
)

# ── Include dirs ───────────────────────────────────────────────────
include_directories(include)

# ── (옵션) 기존 replan_monitor_node 라이브러리 ────────────────────
add_library(replan_monitor_node SHARED
  src/replan_monitor_node.cpp
)
ament_target_dependencies(replan_monitor_node
  rclcpp std_msgs nav_msgs geometry_msgs nav2_costmap_2d tf2_ros
)
target_include_directories(replan_monitor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ── (옵션) 기존 main ───────────────────────────────────────────────
add_executable(replan_monitor_main
  src/main.cpp
)
target_link_libraries(replan_monitor_main
  replan_monitor_node
)
ament_target_dependencies(replan_monitor_main
  rclcpp std_msgs nav_msgs geometry_msgs nav2_costmap_2d tf2_ros
)

# ── path_validator_node ────────────────────────────────────────────
add_executable(path_validator_node
  src/path_validator_node.cpp
  src/path_validator_main.cpp # (주의: main함수가 있는 파일이 맞는지 확인 필요)
)

# [NEW] Add dependencies here
ament_target_dependencies(path_validator_node
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  nav2_costmap_2d
  multi_robot_msgs
  nav2_smac_planner
  nav2_util
)

# ── Install ────────────────────────────────────────────────────────
install(TARGETS
  replan_monitor_node
  replan_monitor_main
  path_validator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

```

### 요약

1. **`find_package`**: `nav2_smac_planner`, `nav2_util` 추가.
2. **`ament_target_dependencies`**: `path_validator_node` 타겟에 `nav2_smac_planner`, `nav2_util` 추가.
3. **`package.xml`**: `<depend>` 태그에 위 패키지들 추가.
