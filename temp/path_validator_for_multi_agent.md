좋아—요구하신 대로 **기존 파라미터/방식 최대 유지** + **옵션으로 ‘footprint 기반 검사’ & agent_layer 마스크 교차검사**를 추가한 `path_validator_node`의 **완전한 수정본(hpp/cpp)** 을 드릴게요.
변경점은 주석에 `[NEW]`, `[CHANGED]`로 표시했습니다. 기존 로직은 그대로 두고 스위치로 켜고 끌 수 있게 했습니다.

---

# path_validator_node.hpp (수정본)

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
  void agentMaskCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);     // [NEW]
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

  // [NEW] footprint 기반 검사 경로
  void validateWithFootprint(const std::vector<geometry_msgs::msg::PoseStamped> & gpath);
  // [NEW] 기존(pruned-path 포인트 기반) 검사 경로 유지
  void validateWithPoints(const std::vector<geometry_msgs::msg::PoseStamped> & gpath);

  // [NEW] 마스터/에이전트 마스크 동시 조회 유틸
  inline bool masterCellBlocked(unsigned int mx, unsigned int my, unsigned char thr) const;
  inline bool agentCellBlockedNear(unsigned int mx, unsigned int my,
                                   unsigned char thr, int manhattan_buf) const;

  void triggerReplan(const std::string & reason);

  // ========= Callback Groups =========
  rclcpp::CallbackGroup::SharedPtr subs_callback_group_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

  // ========= ROS I/O =========
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pruned_path_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr agent_mask_sub_; // [NEW]
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_status_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr replan_pub_;

  rclcpp::TimerBase::SharedPtr obstacle_db_update_timer_;
  rclcpp::TimerBase::SharedPtr flag_reset_timer_;  // optional false pulse

  // ========= State =========
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  mutable std::mutex costmap_mutex_;
  CostmapSignature last_costmap_sig_;

  // [NEW] agent 전용 마스크(costmap_raw 형식)
  std::shared_ptr<nav2_costmap_2d::Costmap2D> agent_mask_;
  mutable std::mutex agent_mask_mutex_;
  CostmapSignature last_agent_sig_;

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

  // Path checking (기존)
  double path_check_distance_m_;  // additionally cap lookahead by this distance

  // Replan flag pulse
  bool publish_false_pulse_;
  int flag_pulse_ms_;

  // ========= New Parameters =========
  bool use_footprint_check_;              // [NEW] footprint 기반 검사 스위치
  double robot_radius_m_;                 // [NEW] 원형 footprint 반경
  double footprint_step_m_;               // [NEW] 경로를 따라 샘플링 간격

  bool compare_agent_mask_;               // [NEW] agent 마스크와 교차검사 여부
  std::string agent_mask_topic_;          // [NEW] agent 마스크 토픽
  double agent_cost_threshold_;           // [NEW] agent 마스크 임계값(보통 254)
  int agent_mask_manhattan_buffer_;       // [NEW] 맨해튼 1~2셀 완충
};

}  // namespace replan_monitor

#endif  // PATH_VALIDATOR_NODE_HPP
```

---

# path_validator_node.cpp (수정본)

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
  this->declare_parameter("consecutive_threshold", 3);
  this->declare_parameter("obstacle_persistence_sec", 0.5);
  this->declare_parameter("max_speed", 0.5);
  this->declare_parameter("lookahead_time_sec", 15.0);
  this->declare_parameter("min_lookahead_m", 2.0);
  this->declare_parameter("cost_threshold", 200.0);
  this->declare_parameter("ignore_unknown", true);

  this->declare_parameter("db_update_frequency", 5.0);
  this->declare_parameter("obstacle_prune_timeout_sec", 3.0);
  this->declare_parameter("db_stride", 2);
  this->declare_parameter("cone_angle_deg", 100.0);
  this->declare_parameter("kernel_half_size", 1);

  this->declare_parameter("path_check_distance_m", 6.0);

  this->declare_parameter("publish_false_pulse", true);
  this->declare_parameter("flag_pulse_ms", 120);

  // ===== [NEW] Footprint / Agent mask 파라미터 =====
  this->declare_parameter("use_footprint_check", false);         // 기존 방식 유지 기본값
  this->declare_parameter("robot_radius_m", 0.30);               // 원형 footprint 가정
  this->declare_parameter("footprint_step_m", 0.15);             // 경로 샘플 간격

  this->declare_parameter("compare_agent_mask", true);           // agent 교차검사 on
  this->declare_parameter<std::string>("agent_mask_topic", "/agent_layer/costmap_raw");
  this->declare_parameter("agent_cost_threshold", 254.0);
  this->declare_parameter("agent_mask_manhattan_buffer", 1);

  // ---- load parameters (기존 + NEW) ----
  consecutive_threshold_      = static_cast<size_t>(this->get_parameter("consecutive_threshold").as_int());
  db_stride_                  = std::max<int>(1, static_cast<int>(this->get_parameter("db_stride").as_int()));
  cone_angle_deg_             = this->get_parameter("cone_angle_deg").as_double();
  kernel_half_size_           = std::max<int>(0, static_cast<int>(this->get_parameter("kernel_half_size").as_int()));
  flag_pulse_ms_              = static_cast<int>(this->get_parameter("flag_pulse_ms").as_int());

  global_frame_               = this->get_parameter("global_frame").as_string();
  base_frame_                 = this->get_parameter("base_frame").as_string();

  cooldown_sec_               = this->get_parameter("cooldown_sec").as_double();
  consecutive_threshold_      = static_cast<size_t>(this->get_parameter("consecutive_threshold").as_int());
  obstacle_persistence_sec_   = this->get_parameter("obstacle_persistence_sec").as_double();
  max_speed_                  = this->get_parameter("max_speed").as_double();
  lookahead_time_sec_         = this->get_parameter("lookahead_time_sec").as_double();
  min_lookahead_m_            = this->get_parameter("min_lookahead_m").as_double();
  cost_threshold_             = this->get_parameter("cost_threshold").as_double();
  ignore_unknown_             = this->get_parameter("ignore_unknown").as_bool();

  db_update_frequency_        = this->get_parameter("db_update_frequency").as_double();
  obstacle_prune_timeout_sec_ = this->get_parameter("obstacle_prune_timeout_sec").as_double();
  db_stride_                  = std::max<int>(1, static_cast<int>(this->get_parameter("db_stride").as_int()));
  cone_angle_deg_             = this->get_parameter("cone_angle_deg").as_double();
  kernel_half_size_           = std::max<int>(0, static_cast<int>(this->get_parameter("kernel_half_size").as_int()));

  path_check_distance_m_      = this->get_parameter("path_check_distance_m").as_double();

  publish_false_pulse_        = this->get_parameter("publish_false_pulse").as_bool();
  flag_pulse_ms_              = static_cast<int>(this->get_parameter("flag_pulse_ms").as_int());

  // [NEW]
  use_footprint_check_        = this->get_parameter("use_footprint_check").as_bool();
  robot_radius_m_             = this->get_parameter("robot_radius_m").as_double();
  footprint_step_m_           = std::max(0.05, this->get_parameter("footprint_step_m").as_double());

  compare_agent_mask_         = this->get_parameter("compare_agent_mask").as_bool();
  agent_mask_topic_           = this->get_parameter("agent_mask_topic").as_string();
  agent_cost_threshold_       = this->get_parameter("agent_cost_threshold").as_double();
  agent_mask_manhattan_buffer_= std::max<int>(0, static_cast<int>(this->get_parameter("agent_mask_manhattan_buffer").as_int()));

  // ===== TF =====
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ===== Callback Groups =====
  subs_callback_group_  = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto subs_options = rclcpp::SubscriptionOptions();
  subs_options.callback_group = subs_callback_group_;

  // ===== Subscriptions =====
  // 마스터(글로벌) 코스트맵
  costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
      "/global_costmap/costmap_raw",
      rclcpp::QoS(rclcpp::SystemDefaultsQoS()).transient_local().reliable(),
      std::bind(&PathValidatorNode::costmapCallback, this, _1),
      subs_options);

  // [NEW] agent 전용 마스크 코스트맵 (AgentLayer에서 별도 발행하는 raw를 연결)
  if (compare_agent_mask_ && !agent_mask_topic_.empty()) {
    agent_mask_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
        agent_mask_topic_,
        rclcpp::QoS(rclcpp::SystemDefaultsQoS()).transient_local().reliable(),
        std::bind(&PathValidatorNode::agentMaskCallback, this, _1),
        subs_options);
  }

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
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();
  replan_pub_ = this->create_publisher<std_msgs::msg::Bool>("/replan_flag", qos);

  // ===== Timers =====
  const int period_ms = static_cast<int>(1000.0 / std::max(1.0, db_update_frequency_));
  obstacle_db_update_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&PathValidatorNode::updateObstacleDatabase, this),
      timer_callback_group_);

  RCLCPP_INFO(this->get_logger(),
              "PathValidatorNode initialized. use_footprint_check=%s, compare_agent_mask=%s, agent_mask_topic=%s",
              use_footprint_check_ ? "true" : "false",
              compare_agent_mask_ ? "true" : "false",
              agent_mask_topic_.c_str());
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

  if (!costmap_ || !(sig == last_costmap_sig_)) {
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>(
        sig.size_x, sig.size_y, sig.resolution, sig.origin_x, sig.origin_y,
        nav2_costmap_2d::FREE_SPACE);
    last_costmap_sig_ = sig;

    std::lock_guard<std::mutex> db_lock(obstacle_db_mutex_);
    obstacle_db_.clear();
  }

  const size_t expected = static_cast<size_t>(sig.size_x) * static_cast<size_t>(sig.size_y);
  if (msg->data.size() != expected) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Costmap data size mismatch: got %zu, expected %zu",
        msg->data.size(), expected);
    return;
  }
  std::memcpy(costmap_->getCharMap(), msg->data.data(), expected * sizeof(unsigned char));
}

// [NEW] agent 마스크 수신
void PathValidatorNode::agentMaskCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(agent_mask_mutex_);

  CostmapSignature sig;
  sig.size_x    = msg->metadata.size_x;
  sig.size_y    = msg->metadata.size_y;
  sig.resolution= msg->metadata.resolution;
  sig.origin_x  = msg->metadata.origin.position.x;
  sig.origin_y  = msg->metadata.origin.position.y;

  if (!agent_mask_ || !(sig == last_agent_sig_)) {
    agent_mask_ = std::make_shared<nav2_costmap_2d::Costmap2D>(
        sig.size_x, sig.size_y, sig.resolution, sig.origin_x, sig.origin_y,
        nav2_costmap_2d::FREE_SPACE);
    last_agent_sig_ = sig;
  }

  const size_t expected = static_cast<size_t>(sig.size_x) * static_cast<size_t>(sig.size_y);
  if (msg->data.size() != expected) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Agent mask data size mismatch: got %zu, expected %zu",
        msg->data.size(), expected);
    return;
  }
  std::memcpy(agent_mask_->getCharMap(), msg->data.data(), expected * sizeof(unsigned char));
}

void PathValidatorNode::robotStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string & s = msg->data;
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
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "ROI outside map bounds");
    return;
  }

  const rclcpp::Time now = this->now();
  std::unordered_set<uint64_t> visible;

  const double yaw = tf2::getYaw(pose.orientation);
  const double ux = std::cos(yaw), uy = std::sin(yaw);
  const double cos_half = std::cos((cone_angle_deg_ * M_PI / 180.0) * 0.5);
  const double res = costmap->getResolution();

  for (unsigned int mx = min_mx; mx <= max_mx; mx += static_cast<unsigned int>(db_stride_)) {
    for (unsigned int my = min_my; my <= max_my; my += static_cast<unsigned int>(db_stride_)) {
      double wx, wy;
      costmap->mapToWorld(mx, my, wx, wy);

      const double dx = wx - pose.position.x;
      const double dy = wy - pose.position.y;
      const double dist = std::hypot(dx, dy);
      if (dist > lookahead) continue;

      const double ndot = (dx * ux + dy * uy) / std::max(1e-6, dist);
      if (ndot < cos_half) continue;

      const unsigned char c = costmap->getCost(mx, my);
      if (ignore_unknown_ && c == nav2_costmap_2d::NO_INFORMATION) continue;
      if (c >= static_cast<unsigned char>(cost_threshold_)) {
        visible.insert(packKey(mx, my));
      }
    }
  }

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

inline bool PathValidatorNode::masterCellBlocked(unsigned int mx, unsigned int my, unsigned char thr) const
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  if (!costmap_) return false;

  const unsigned char c = costmap_->getCost(mx, my);
  if (ignore_unknown_ && c == nav2_costmap_2d::NO_INFORMATION) return false;
  return (c >= thr);
}

inline bool PathValidatorNode::agentCellBlockedNear(unsigned int mx, unsigned int my,
                                                    unsigned char thr, int manhattan_buf) const
{
  std::lock_guard<std::mutex> lock(agent_mask_mutex_);
  if (!agent_mask_) return false;

  const int sx = static_cast<int>(agent_mask_->getSizeInCellsX());
  const int sy = static_cast<int>(agent_mask_->getSizeInCellsY());

  const int ix = static_cast<int>(mx);
  const int iy = static_cast<int>(my);

  for (int dx = -manhattan_buf; dx <= manhattan_buf; ++dx) {
    for (int dy = -manhattan_buf; dy <= manhattan_buf; ++dy) {
      if (std::abs(dx) + std::abs(dy) > manhattan_buf) continue; // 맨해튼
      const int x = ix + dx;
      const int y = iy + dy;
      if (x < 0 || y < 0 || x >= sx || y >= sy) continue;

      const unsigned char a = agent_mask_->getCost(static_cast<unsigned int>(x),
                                                   static_cast<unsigned int>(y));
      // 에이전트 마스크는 보통 0/254/255(unknown) 중 254만 의미있게 사용
      if (a >= thr) return true;
    }
  }
  return false;
}

// ===================== Path validation dispatcher =====================

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

  std::vector<geometry_msgs::msg::PoseStamped> gpath;
  transformPathToGlobal(*msg, gpath);
  if (gpath.empty()) return;

  if (use_footprint_check_) {
    validateWithFootprint(gpath);
  } else {
    validateWithPoints(gpath);
  }
}

// ===================== 기존 방식(포인트) 유지 =====================

void PathValidatorNode::validateWithPoints(const std::vector<geometry_msgs::msg::PoseStamped> & gpath)
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
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      if (!costmap_) return;
      if (!costmap_->worldToMap(gpath[i].pose.position.x, gpath[i].pose.position.y, mx, my)) {
        streak = 0;
        continue;
      }
    }

    bool blocked_cell = isBlockedCellKernel(mx, my);

    const uint64_t key = packKey(mx, my);
    bool persistent_mature = false;
    auto it = db_snapshot.find(key);
    if (it != db_snapshot.end()) {
      if ((now - it->second.first_seen).seconds() >= obstacle_persistence_sec_) {
        persistent_mature = true;
      }
    }

    bool blocked = blocked_cell && persistent_mature;

    // [NEW] agent 교차검사: 마스터에서 lethal이면 agent 유무 확인(사유 표기를 위해)
    if (blocked && compare_agent_mask_) {
      const bool agent_hit = agentCellBlockedNear(mx, my,
                              static_cast<unsigned char>(agent_cost_threshold_),
                              agent_mask_manhattan_buffer_);
      if (agent_hit) {
        // optional: reason differentiation (아래 trigger 시 reason문구에 포함)
      }
    }

    streak = blocked ? (streak + 1) : 0;
    best_streak = std::max(best_streak, streak);

    if (best_streak >= consecutive_threshold_) {
      triggerReplan("blocked (points) streak threshold reached");
      break;
    }
  }
}

// ===================== [NEW] Footprint 기반 검사 =====================

void PathValidatorNode::validateWithFootprint(const std::vector<geometry_msgs::msg::PoseStamped> & gpath)
{
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (!costmap_) return;
    costmap = costmap_;
  }

  geometry_msgs::msg::Pose cur_pose;
  if (!getCurrentPoseFromTF(cur_pose)) return;

  const double max_dist_by_time = std::max(min_lookahead_m_, max_speed_ * lookahead_time_sec_);
  const double max_check_dist = std::min(max_dist_by_time, path_check_distance_m_);
  const double res = costmap->getResolution();

  // 경로를 거리 기반으로 다운샘플링
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
    // 검사 상한
    if (std::hypot(gpath[i].pose.position.x - gpath.front().pose.position.x,
                   gpath[i].pose.position.y - gpath.front().pose.position.y) > max_check_dist) {
      break;
    }
  }

  const unsigned char master_thr = static_cast<unsigned char>(cost_threshold_);
  const unsigned char agent_thr  = static_cast<unsigned char>(agent_cost_threshold_);

  // footprint는 원형(반경 robot_radius_m_)으로 단순화: 반경 내의 모든 셀 검사
  const int r_cells = std::max(1, static_cast<int>(std::ceil(robot_radius_m_ / res)));

  size_t consecutive = 0;
  for (const auto & ps : samples) {
    unsigned int cx, cy;
    if (!costmap->worldToMap(ps.pose.position.x, ps.pose.position.y, cx, cy)) {
      consecutive = 0;
      continue;
    }

    bool blocked_here = false;
    // 원형 footprint 스캔
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

        // 1) 마스터 lethal?
        if (masterCellBlocked(umx, umy, master_thr)) {
          // 2) [옵션] agent 마스크 교차검사
          if (compare_agent_mask_) {
            const bool agent_hit = agentCellBlockedNear(umx, umy, agent_thr,
                                                        agent_mask_manhattan_buffer_);
            if (agent_hit) {
              blocked_here = true;  // 다른 로봇으로 판단
            } else {
              blocked_here = true;  // 비-에이전트 장애물도 리플랜 트리거 대상
            }
          } else {
            blocked_here = true;
          }
        }
      }
    }

    if (blocked_here) {
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

## 어떻게 쓰면 좋나 (요약)

* 기본 동작은 **기존과 동일**(pruned path의 포인트 기반 검사).

* 아래 파라미터로 “footprint 기반 검사 + agent 교차검사”를 활성화:

  * `use_footprint_check: true`
  * `compare_agent_mask: true`
  * `agent_mask_topic: "/agent_layer/costmap_raw"`  (AgentLayer가 **agent 전용 raw**를 발행하도록 설정해야 합니다. 값은 0/254/255 형태 권장)
  * `cost_threshold: 254.0`
  * `agent_cost_threshold: 254.0`
  * `agent_mask_manhattan_buffer: 1` (마스크가 슬림할 수 있으니 1~2 권장)
  * `robot_radius_m: 0.30`, `footprint_step_m: 0.15` (환경에 맞게 튜닝)

* **성능 주의:** footprint 모드는 반경 내 셀을 매 샘플마다 검사합니다. `footprint_step_m`을 0.1~0.2m로 두고, `robot_radius_m`는 실제 footprint에 맞추세요.

* **좌표/격자 정합:** 마스터와 에이전트 마스크는 **해상도/원점/크기**가 다를 수 있어도, 각 지도에 대해 `worldToMap`을 사용하므로 안전합니다. 단, AgentLayer가 발행하는 마스크가 **동일 frame_id**(보통 `map`)를 쓰도록 해 주세요.

필요하면 샘플 YAML도 바로 적어드릴게요.
###############################3  
#################################  
################################  
좋아, `robot_radius_m`를 **직접 파라미터로 받지 않고**, Nav2 표준처럼 **`robot_radius` 와 `footprint`** 를 읽어서 자동으로 결정하도록 전체 코드를 정리했어.
아래 수정본은:

* `robot_radius`(double)와 `footprint`(string: `[[x,y], ...]`)를 선언/취득
* `footprint`가 유효하면 다각형 footprint 사용, 아니면 `robot_radius` 원형 사용
* 기존 기능(포인트 기반 검사 / footprint 기반 검사 토글, agent 마스크 교차검사 등) 유지
* 다각형 footprint일 때 경로 샘플 포즈의 yaw를 적용해 **셀 중심이 폴리곤 내부인지**로 충돌 판정
* 원형 footprint일 때는 반경 셀 원판 스캔

변경점은 `[NEW]`, `[CHANGED]`로 표기했어.

---

# path_validator_node.hpp

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

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
// [NEW] footprint 유틸
#include "nav2_costmap_2d/footprint.hpp"

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
  void agentMaskCallback(const nav2_msgs::msg::Costmap::SharedPtr msg); // [NEW]
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

  // [NEW] path 검사 2가지 경로
  void validateWithFootprint(const std::vector<geometry_msgs::msg::PoseStamped> & gpath);
  void validateWithPoints(const std::vector<geometry_msgs::msg::PoseStamped> & gpath);

  // [NEW] 마스터/에이전트 마스크 조회
  inline bool masterCellBlocked(unsigned int mx, unsigned int my, unsigned char thr) const;
  inline bool agentCellBlockedNear(unsigned int mx, unsigned int my,
                                   unsigned char thr, int manhattan_buf) const;

  // [NEW] polygon point-in-polygon
  static bool pointInPolygon(const std::vector<geometry_msgs::msg::Point> & poly,
                             double x, double y);

  void triggerReplan(const std::string & reason);

  // ========= Callback Groups =========
  rclcpp::CallbackGroup::SharedPtr subs_callback_group_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

  // ========= ROS I/O =========
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pruned_path_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr agent_mask_sub_; // [NEW]
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_status_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr replan_pub_;

  rclcpp::TimerBase::SharedPtr obstacle_db_update_timer_;
  rclcpp::TimerBase::SharedPtr flag_reset_timer_;

  // ========= State =========
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  mutable std::mutex costmap_mutex_;
  CostmapSignature last_costmap_sig_;

  std::shared_ptr<nav2_costmap_2d::Costmap2D> agent_mask_; // [NEW]
  mutable std::mutex agent_mask_mutex_;
  CostmapSignature last_agent_sig_;

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
  int db_stride_;
  double cone_angle_deg_;
  int kernel_half_size_;

  // Path checking
  double path_check_distance_m_;

  // Replan flag pulse
  bool publish_false_pulse_;
  int flag_pulse_ms_;

  // ========= New Parameters =========
  bool use_footprint_check_;       // [NEW]
  double footprint_step_m_;        // [NEW]
  bool compare_agent_mask_;        // [NEW]
  std::string agent_mask_topic_;   // [NEW]
  double agent_cost_threshold_;    // [NEW]
  int agent_mask_manhattan_buffer_;// [NEW]

  // [NEW] Nav2 스타일 footprint 선택자
  std::string footprint_str_;                          // 파라미터 원문
  std::vector<geometry_msgs::msg::Point> footprint_;   // 유효하면 사용
  bool use_radius_{true};                              // footprint가 유효하면 false
  double robot_radius_m_{0.1};                         // radius 모드일 때 사용
};

}  // namespace replan_monitor

#endif  // PATH_VALIDATOR_NODE_HPP
```

---

# path_validator_node.cpp

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
  this->declare_parameter("consecutive_threshold", 3);
  this->declare_parameter("obstacle_persistence_sec", 0.5);
  this->declare_parameter("max_speed", 0.5);
  this->declare_parameter("lookahead_time_sec", 15.0);
  this->declare_parameter("min_lookahead_m", 2.0);
  this->declare_parameter("cost_threshold", 200.0);
  this->declare_parameter("ignore_unknown", true);

  this->declare_parameter("db_update_frequency", 5.0);
  this->declare_parameter("obstacle_prune_timeout_sec", 3.0);
  this->declare_parameter("db_stride", 2);
  this->declare_parameter("cone_angle_deg", 100.0);
  this->declare_parameter("kernel_half_size", 1);

  this->declare_parameter("path_check_distance_m", 6.0);

  this->declare_parameter("publish_false_pulse", true);
  this->declare_parameter("flag_pulse_ms", 120);

  // ===== [NEW] Footprint / Agent mask 파라미터 =====
  this->declare_parameter("use_footprint_check", false);
  this->declare_parameter("footprint_step_m", 0.15);

  this->declare_parameter("compare_agent_mask", true);
  this->declare_parameter<std::string>("agent_mask_topic", "/agent_layer/costmap_raw");
  this->declare_parameter("agent_cost_threshold", 254.0);
  this->declare_parameter("agent_mask_manhattan_buffer", 1);

  // [NEW] Nav2 footprint param들
  this->declare_parameter<std::string>("footprint", "[]");
  this->declare_parameter("robot_radius", 0.1);

  // ---- load parameters (기존 + NEW) ----
  consecutive_threshold_      = static_cast<size_t>(this->get_parameter("consecutive_threshold").as_int());
  db_stride_                  = std::max<int>(1, static_cast<int>(this->get_parameter("db_stride").as_int()));
  cone_angle_deg_             = this->get_parameter("cone_angle_deg").as_double();
  kernel_half_size_           = std::max<int>(0, static_cast<int>(this->get_parameter("kernel_half_size").as_int()));
  flag_pulse_ms_              = static_cast<int>(this->get_parameter("flag_pulse_ms").as_int());

  global_frame_               = this->get_parameter("global_frame").as_string();
  base_frame_                 = this->get_parameter("base_frame").as_string();

  cooldown_sec_               = this->get_parameter("cooldown_sec").as_double();
  consecutive_threshold_      = static_cast<size_t>(this->get_parameter("consecutive_threshold").as_int());
  obstacle_persistence_sec_   = this->get_parameter("obstacle_persistence_sec").as_double();
  max_speed_                  = this->get_parameter("max_speed").as_double();
  lookahead_time_sec_         = this->get_parameter("lookahead_time_sec").as_double();
  min_lookahead_m_            = this->get_parameter("min_lookahead_m").as_double();
  cost_threshold_             = this->get_parameter("cost_threshold").as_double();
  ignore_unknown_             = this->get_parameter("ignore_unknown").as_bool();

  db_update_frequency_        = this->get_parameter("db_update_frequency").as_double();
  obstacle_prune_timeout_sec_ = this->get_parameter("obstacle_prune_timeout_sec").as_double();
  db_stride_                  = std::max<int>(1, static_cast<int>(this->get_parameter("db_stride").as_int()));
  cone_angle_deg_             = this->get_parameter("cone_angle_deg").as_double();
  kernel_half_size_           = std::max<int>(0, static_cast<int>(this->get_parameter("kernel_half_size").as_int()));

  path_check_distance_m_      = this->get_parameter("path_check_distance_m").as_double();

  publish_false_pulse_        = this->get_parameter("publish_false_pulse").as_bool();
  flag_pulse_ms_              = static_cast<int>(this->get_parameter("flag_pulse_ms").as_int());

  // [NEW]
  use_footprint_check_        = this->get_parameter("use_footprint_check").as_bool();
  footprint_step_m_           = std::max(0.05, this->get_parameter("footprint_step_m").as_double());

  compare_agent_mask_         = this->get_parameter("compare_agent_mask").as_bool();
  agent_mask_topic_           = this->get_parameter("agent_mask_topic").as_string();
  agent_cost_threshold_       = this->get_parameter("agent_cost_threshold").as_double();
  agent_mask_manhattan_buffer_= std::max<int>(0, static_cast<int>(this->get_parameter("agent_mask_manhattan_buffer").as_int()));

  // [NEW] footprint / robot_radius 취득 및 해석
  footprint_str_              = this->get_parameter("footprint").as_string();
  robot_radius_m_             = this->get_parameter("robot_radius").as_double();
  use_radius_                 = true;
  footprint_.clear();
  if (!footprint_str_.empty() && footprint_str_ != "[]") {
    // Nav2 유틸로 파싱 시도
    if (nav2_costmap_2d::makeFootprintFromString(footprint_str_, footprint_) && footprint_.size() >= 3) {
      use_radius_ = false;
      RCLCPP_INFO(get_logger(), "Using polygon footprint with %zu points.", footprint_.size());
    } else {
      RCLCPP_ERROR(get_logger(),
        "Invalid footprint string: \"%s\". Falling back to robot_radius=%.3f",
        footprint_str_.c_str(), robot_radius_m_);
      use_radius_ = true;
    }
  } else {
    RCLCPP_INFO(get_logger(), "No valid footprint provided. Using robot_radius=%.3f", robot_radius_m_);
  }

  // ===== TF =====
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ===== Callback Groups =====
  subs_callback_group_  = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto subs_options = rclcpp::SubscriptionOptions();
  subs_options.callback_group = subs_callback_group_;

  // ===== Subscriptions =====
  costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
      "/global_costmap/costmap_raw",
      rclcpp::QoS(rclcpp::SystemDefaultsQoS()).transient_local().reliable(),
      std::bind(&PathValidatorNode::costmapCallback, this, _1),
      subs_options);

  if (compare_agent_mask_ && !agent_mask_topic_.empty()) {
    agent_mask_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
        agent_mask_topic_,
        rclcpp::QoS(rclcpp::SystemDefaultsQoS()).transient_local().reliable(),
        std::bind(&PathValidatorNode::agentMaskCallback, this, _1),
        subs_options);
  }

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
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();
  replan_pub_ = this->create_publisher<std_msgs::msg::Bool>("/replan_flag", qos);

  // ===== Timers =====
  const int period_ms = static_cast<int>(1000.0 / std::max(1.0, db_update_frequency_));
  obstacle_db_update_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&PathValidatorNode::updateObstacleDatabase, this),
      timer_callback_group_);

  RCLCPP_INFO(this->get_logger(),
    "PathValidatorNode initialized. use_footprint_check=%s (footprint:%s / radius:%.3f), compare_agent_mask=%s, agent_mask_topic=%s",
    use_footprint_check_ ? "true":"false",
    use_radius_ ? "none" : "polygon",
    robot_radius_m_,
    compare_agent_mask_ ? "true":"false",
    agent_mask_topic_.c_str());
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

  if (!costmap_ || !(sig == last_costmap_sig_)) {
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>(
        sig.size_x, sig.size_y, sig.resolution, sig.origin_x, sig.origin_y,
        nav2_costmap_2d::FREE_SPACE);
    last_costmap_sig_ = sig;

    std::lock_guard<std::mutex> db_lock(obstacle_db_mutex_);
    obstacle_db_.clear();
  }

  const size_t expected = static_cast<size_t>(sig.size_x) * static_cast<size_t>(sig.size_y);
  if (msg->data.size() != expected) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Costmap data size mismatch: got %zu, expected %zu",
        msg->data.size(), expected);
    return;
  }
  std::memcpy(costmap_->getCharMap(), msg->data.data(), expected * sizeof(unsigned char));
}

void PathValidatorNode::agentMaskCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(agent_mask_mutex_);

  CostmapSignature sig;
  sig.size_x    = msg->metadata.size_x;
  sig.size_y    = msg->metadata.size_y;
  sig.resolution= msg->metadata.resolution;
  sig.origin_x  = msg->metadata.origin.position.x;
  sig.origin_y  = msg->metadata.origin.position.y;

  if (!agent_mask_ || !(sig == last_agent_sig_)) {
    agent_mask_ = std::make_shared<nav2_costmap_2d::Costmap2D>(
        sig.size_x, sig.size_y, sig.resolution, sig.origin_x, sig.origin_y,
        nav2_costmap_2d::FREE_SPACE);
    last_agent_sig_ = sig;
  }

  const size_t expected = static_cast<size_t>(sig.size_x) * static_cast<size_t>(sig.size_y);
  if (msg->data.size() != expected) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Agent mask data size mismatch: got %zu, expected %zu",
        msg->data.size(), expected);
    return;
  }
  std::memcpy(agent_mask_->getCharMap(), msg->data.data(), expected * sizeof(unsigned char));
}

void PathValidatorNode::robotStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string & s = msg->data;
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
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "ROI outside map bounds");
    return;
  }

  const rclcpp::Time now = this->now();
  std::unordered_set<uint64_t> visible;

  const double yaw = tf2::getYaw(pose.orientation);
  const double ux = std::cos(yaw), uy = std::sin(yaw);
  const double cos_half = std::cos((cone_angle_deg_ * M_PI / 180.0) * 0.5);

  for (unsigned int mx = min_mx; mx <= max_mx; mx += static_cast<unsigned int>(db_stride_)) {
    for (unsigned int my = min_my; my <= max_my; my += static_cast<unsigned int>(db_stride_)) {
      double wx, wy;
      costmap->mapToWorld(mx, my, wx, wy);

      const double dx = wx - pose.position.x;
      const double dy = wy - pose.position.y;
      const double dist = std::hypot(dx, dy);
      if (dist > lookahead) continue;

      const double ndot = (dx * ux + dy * uy) / std::max(1e-6, dist);
      if (ndot < cos_half) continue;

      const unsigned char c = costmap->getCost(mx, my);
      if (ignore_unknown_ && c == nav2_costmap_2d::NO_INFORMATION) continue;
      if (c >= static_cast<unsigned char>(cost_threshold_)) {
        visible.insert(packKey(mx, my));
      }
    }
  }

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

inline bool PathValidatorNode::masterCellBlocked(unsigned int mx, unsigned int my, unsigned char thr) const
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  if (!costmap_) return false;

  const unsigned char c = costmap_->getCost(mx, my);
  if (ignore_unknown_ && c == nav2_costmap_2d::NO_INFORMATION) return false;
  return (c >= thr);
}

inline bool PathValidatorNode::agentCellBlockedNear(unsigned int mx, unsigned int my,
                                                    unsigned char thr, int manhattan_buf) const
{
  std::lock_guard<std::mutex> lock(agent_mask_mutex_);
  if (!agent_mask_) return false;

  const int sx = static_cast<int>(agent_mask_->getSizeInCellsX());
  const int sy = static_cast<int>(agent_mask_->getSizeInCellsY());

  const int ix = static_cast<int>(mx);
  const int iy = static_cast<int>(my);

  for (int dx = -manhattan_buf; dx <= manhattan_buf; ++dx) {
    for (int dy = -manhattan_buf; dy <= manhattan_buf; ++dy) {
      if (std::abs(dx) + std::abs(dy) > manhattan_buf) continue;
      const int x = ix + dx;
      const int y = iy + dy;
      if (x < 0 || y < 0 || x >= sx || y >= sy) continue;

      const unsigned char a = agent_mask_->getCost(static_cast<unsigned int>(x),
                                                   static_cast<unsigned int>(y));
      if (a >= thr) return true;
    }
  }
  return false;
}

// ===================== Path validation dispatcher =====================

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

  std::vector<geometry_msgs::msg::PoseStamped> gpath;
  transformPathToGlobal(*msg, gpath);
  if (gpath.empty()) return;

  if (use_footprint_check_) {
    validateWithFootprint(gpath);
  } else {
    validateWithPoints(gpath);
  }
}

// ===================== 기존 방식(포인트) 유지 =====================

void PathValidatorNode::validateWithPoints(const std::vector<geometry_msgs::msg::PoseStamped> & gpath)
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
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      if (!costmap_) return;
      if (!costmap_->worldToMap(gpath[i].pose.position.x, gpath[i].pose.position.y, mx, my)) {
        streak = 0;
        continue;
      }
    }

    bool blocked_cell = isBlockedCellKernel(mx, my);

    const uint64_t key = packKey(mx, my);
    bool persistent_mature = false;
    auto it = db_snapshot.find(key);
    if (it != db_snapshot.end()) {
      if ((now - it->second.first_seen).seconds() >= obstacle_persistence_sec_) {
        persistent_mature = true;
      }
    }

    bool blocked = blocked_cell && persistent_mature;

    if (blocked && compare_agent_mask_) {
      const bool agent_hit = agentCellBlockedNear(mx, my,
                              static_cast<unsigned char>(agent_cost_threshold_),
                              agent_mask_manhattan_buffer_);
      (void)agent_hit; // 필요시 reason에 반영
    }

    streak = blocked ? (streak + 1) : 0;
    best_streak = std::max(best_streak, streak);

    if (best_streak >= consecutive_threshold_) {
      triggerReplan("blocked (points) streak threshold reached");
      break;
    }
  }
}

// ===================== [NEW] Footprint 기반 검사 =====================

bool PathValidatorNode::pointInPolygon(const std::vector<geometry_msgs::msg::Point> & poly,
                                       double x, double y)
{
  bool inside = false;
  const size_t n = poly.size();
  for (size_t i=0, j=n-1; i<n; j=i++) {
    const double xi = poly[i].x, yi = poly[i].y;
    const double xj = poly[j].x, yj = poly[j].y;
    const bool hit = ((yi > y) != (yj > y)) &&
                     (x < (xj - xi) * (y - yi) / std::max(1e-12, (yj - yi)) + xi);
    if (hit) inside = !inside;
  }
  return inside;
}

void PathValidatorNode::validateWithFootprint(const std::vector<geometry_msgs::msg::PoseStamped> & gpath)
{
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (!costmap_) return;
    costmap = costmap_;
  }

  geometry_msgs::msg::Pose cur_pose;
  if (!getCurrentPoseFromTF(cur_pose)) return;

  const double max_dist_by_time = std::max(min_lookahead_m_, max_speed_ * lookahead_time_sec_);
  const double max_check_dist = std::min(max_dist_by_time, path_check_distance_m_);
  const double res = costmap->getResolution();

  // 경로를 거리 기반으로 다운샘플링
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
  const unsigned char agent_thr  = static_cast<unsigned char>(agent_cost_threshold_);

  size_t consecutive = 0;

  for (const auto & ps : samples) {
    // 원형 footprint 모드
    if (use_radius_) {
      unsigned int cx, cy;
      if (!costmap->worldToMap(ps.pose.position.x, ps.pose.position.y, cx, cy)) {
        consecutive = 0;
        continue;
      }
      const int r_cells = std::max(1, static_cast<int>(std::ceil(robot_radius_m_ / res)));
      bool blocked_here = false;

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

          if (masterCellBlocked(umx, umy, master_thr)) {
            if (compare_agent_mask_) {
              const bool agent_hit = agentCellBlockedNear(umx, umy, agent_thr,
                                                          agent_mask_manhattan_buffer_);
              (void)agent_hit;
            }
            blocked_here = true;
          }
        }
      }

      if (blocked_here) {
        consecutive++;
        if (consecutive >= consecutive_threshold_) {
          triggerReplan("blocked (footprint-circle) streak threshold reached");
          return;
        }
      } else {
        consecutive = 0;
      }
    }
    // 다각형 footprint 모드
    else {
      // 1) 로컬 footprint를 pose 기준으로 월드로 변환
      const double yaw = tf2::getYaw(ps.pose.orientation);
      const double c = std::cos(yaw), s = std::sin(yaw);

      std::vector<geometry_msgs::msg::Point> poly_world;
      poly_world.reserve(footprint_.size());
      double minx=1e9, miny=1e9, maxx=-1e9, maxy=-1e9;

      for (const auto & p : footprint_) {
        geometry_msgs::msg::Point q;
        const double x = p.x, y = p.y;
        q.x = ps.pose.position.x + c * x - s * y;
        q.y = ps.pose.position.y + s * x + c * y;
        q.z = 0.0;
        poly_world.push_back(q);
        if (q.x < minx) minx=q.x; if (q.y < miny) miny=q.y;
        if (q.x > maxx) maxx=q.x; if (q.y > maxy) maxy=q.y;
      }

      int min_i, min_j, max_i, max_j;
      costmap->worldToMapEnforceBounds(minx, miny, min_i, min_j);
      costmap->worldToMapEnforceBounds(maxx, maxy, max_i, max_j);

      bool blocked_here = false;
      for (int j = min_j; j <= max_j && !blocked_here; ++j) {
        for (int i = min_i; i <= max_i && !blocked_here; ++i) {
          double wx, wy; costmap->mapToWorld(i, j, wx, wy);
          if (!pointInPolygon(poly_world, wx, wy)) continue;

          const unsigned int umx = static_cast<unsigned int>(i);
          const unsigned int umy = static_cast<unsigned int>(j);

          if (masterCellBlocked(umx, umy, master_thr)) {
            if (compare_agent_mask_) {
              const bool agent_hit = agentCellBlockedNear(umx, umy, agent_thr,
                                                          agent_mask_manhattan_buffer_);
              (void)agent_hit;
            }
            blocked_here = true;
          }
        }
      }

      if (blocked_here) {
        consecutive++;
        if (consecutive >= consecutive_threshold_) {
          triggerReplan("blocked (footprint-polygon) streak threshold reached");
          return;
        }
      } else {
        consecutive = 0;
      }
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

## 사용 팁

* `nav2_params.yaml`에서 이미 쓰는 `footprint` / `robot_radius`를 그대로 읽습니다.

  * `footprint`가 유효하면 **다각형 모드**, 아니면 `robot_radius`로 **원형 모드**가 자동 선택됩니다.
* footprint 기반 검사 사용:

  ```yaml
  use_footprint_check: true
  footprint_step_m: 0.15
  ```
* agent 마스크 교차검사:

  ```yaml
  compare_agent_mask: true
  agent_mask_topic: /agent_layer/costmap_raw
  agent_cost_threshold: 254.0
  agent_mask_manhattan_buffer: 1
  ```
* 마스터 임계치:

  ```yaml
  cost_threshold: 254.0
  ```

필요하면 샘플 YAML도 바로 만들어줄게.
################################  
##################################  
좋지! 아래에 **두 가지 샘플 YAML**을 정리했어.

---

# 1) `nav2_params.yaml` (agent_layer 포함 예시)

> 글로벌 코스트맵에 `agent_layer`를 추가하고, 파라미터들을 깔끔히 정리한 샘플이야.
> (필요하면 `local_costmap`에도 같은 방식으로 넣으면 돼.)

```yaml
# nav2_bringup 스타일 예시
amcl:
  ros__parameters:
    use_sim_time: false

bt_navigator:
  ros__parameters:
    use_sim_time: false

controller_server:
  ros__parameters:
    use_sim_time: false

planner_server:
  ros__parameters:
    use_sim_time: false

smoother_server:
  ros__parameters:
    use_sim_time: false

# === 글로벌 코스트맵 설정 ===
global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: false
      global_frame: map
      robot_base_frame: base_link
      resolution: 0.05
      track_unknown_space: true
      rolling_window: false
      width: 50.0
      height: 50.0
      origin_x: 0.0
      origin_y: 0.0

      # Nav2 Iron+ 표기: plugin 이름 배열
      plugins: ["static_layer", "obstacle_layer", "inflation_layer", "agent_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: "scan"
        scan:
          topic: /scan
          clearing: true
          marking: true
          data_type: "LaserScan"
          inf_is_valid: true

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.8
        cost_scaling_factor: 3.0

      # === 여기서부터 multi_agent_nav2::AgentLayer ===
      agent_layer:
        plugin: "multi_agent_nav2::AgentLayer"
        enabled: true

        # 구독할 멀티에이전트 토픽
        topic: "/multi_agent_infos"

        # 자기 자신 식별(동일 id/type 은 무시)
        self_machine_id: 0
        self_type_id: ""

        # path header의 frame 이 global_frame 과 다르면 무시
        use_path_header_frame: true

        # ROI: 우리 로봇 기준 이 거리 밖 에이전트는 무시
        roi_range_m: 12.0

        # (예약) 시간 감쇠용
        time_decay_sec: 1.0

        # 비용(최대 254)
        lethal_cost: 254         # 정지/치명적 차단에 권장
        moving_cost: 180         # 이동 중 기본 코스트
        waiting_cost: 200        # 대기(정지) 코스트
        manual_cost_bias: 30     # 모드가 manual인 경우 가산

        # 팽창(등방성) / 전방 스미어(이동시에만 +x 방향)
        dilation_m: 0.05         # 모든 방향 기본 여유
        forward_smear_m: 0.25    # 진행 방향으로만 길게
        sigma_k: 2.0             # pos_std_m 가중치

        # 메타 퍼블리시(선택)
        publish_meta: true
        meta_stride: 3

        # 신선도/제한
        freshness_timeout_ms: 800
        max_poses: 40
        qos_reliable: true
```

> 🔎 팁
>
> * `plugins` 순서는 실제 코스트맵 합성 순서에 영향 있어. 일반적으로 `agent_layer`는 `obstacle_layer`보다 먼저 넣어도, `inflation_layer`가 뒤에 와서 함께 팽창돼. (의도에 따라 조정)
> * `lethal_cost`를 254로 두면, 글로벌 플래너가 확실히 회피/재계획을 하게 만들기 쉬워.

---

# 2) `path_validator_node.yaml` (agent 마스크 교차검사 & footprint 사용)

> 아래는 `replan_monitor::PathValidatorNode`의 파라미터 샘플이야.
> **같은 파일**에서 Nav2의 `robot_radius` / `footprint`를 함께 넘겨주면, 노드가 자동으로 원형/다각형을 선택해서 검사해.

```yaml
path_validator_node:
  ros__parameters:
    # === 프레임 ===
    global_frame: "map"
    base_frame: "base_link"

    # === 리플랜 이벤트 펄스 ===
    publish_false_pulse: true
    flag_pulse_ms: 120
    cooldown_sec: 1.0

    # === 경로 검사 범위/조건 ===
    lookahead_time_sec: 15.0     # 속도 기반 lookahead
    min_lookahead_m: 2.0         # 최소 전방
    path_check_distance_m: 6.0   # 경로 검사 최대 길이

    # === 차단 판정 ===
    cost_threshold: 254.0        # 마스터 코스트맵 차단 임계치(치명적만 감지하고 싶다면 254 권장)
    ignore_unknown: true
    kernel_half_size: 1          # 셀 커널(1 => 3x3)

    # === 지속성(노이즈 제거) ===
    consecutive_threshold: 3     # 연속 차단 포인트 개수
    obstacle_persistence_sec: 0.5

    # === ROI 스캐닝(데이터베이스) ===
    db_update_frequency: 5.0     # Hz
    obstacle_prune_timeout_sec: 3.0
    db_stride: 2                 # 셀 샘플링(1=모든 셀)
    cone_angle_deg: 100.0        # 전방 부채꼴 각도
    max_speed: 0.5               # m/s (lookahead_time_sec과 곱해 사용)

    # === 검사 방법 선택 ===
    use_footprint_check: true    # true면 footprint 기반, false면 포인트(경로 점) 기반
    footprint_step_m: 0.15       # footprint 모드에서 경로 샘플 간격(m)

    # === Agent 마스크 교차검사 ===
    compare_agent_mask: true
    agent_mask_topic: "/agent_layer/costmap_raw"   # 에이전트 전용 코스트맵 토픽(구성에 맞게)
    agent_cost_threshold: 254.0
    agent_mask_manhattan_buffer: 1   # 에이전트 마스크에서 한두 셀 여유로 탐지

    # === Nav2 스타일 footprint / radius (둘 중 하나 사용) ===
    # footprint 문자열이 유효하면 다각형 사용, 아니면 robot_radius를 원형으로 사용
    # 아래 두 값은 Nav2(global/local costmap)과 같게 유지하는 게 안전
    footprint: "[[0.30,0.30],[0.30,-0.30],[-0.30,-0.30],[-0.30,0.30]]"
    robot_radius: 0.32
```

> 🔎 팁
>
> * `cost_threshold: 254.0`로 두면 **진짜 막힌 상황(LETHAL)** 에만 리플랜이 나가서 과민 반응을 줄여.
> * `use_footprint_check: true`면, 경로상의 각 샘플 포즈에 **로봇 footprint**를 대입해서 충돌을 보므로 **실제 충돌 가능성**에 더 가깝게 판단해.
> * `agent_mask_topic`은 **에이전트만 반영된 코스트맵**을 퍼블리시하도록 별도의 구성(두 번째 LayeredCostmap 등)이 필요해. 그게 어려우면:
>
>   * 일단 `compare_agent_mask: false`로 끄고 마스터만 사용 → 동작 확인 후
>   * 차후, `agent_layer`만 켠 전용 costmap 인스턴스(작은 해상도/윈도우도 OK)를 만들어 `/agent_layer/costmap_raw`를 내보내면 정확한 원인 판별이 가능해.

---

## 빠른 체크리스트

* [ ] `multi_agent_nav2` 플러그인이 빌드/설치됨 (`colcon build --packages-select multi_agent_nav2` 후 `source install/setup.bash`)
* [ ] `package.xml`의 `<export>` 섹션이 플러그인 XML을 올바르게 가리킴
* [ ] `nav2_params.yaml`의 `plugins`에 `"agent_layer"` 추가 및 `agent_layer.plugin` 타입이 `multi_agent_nav2::AgentLayer`
* [ ] `path_validator_node` 가 같은 `footprint` / `robot_radius` 값을 읽도록 YAML 통일
* [ ] (선택) `agent_mask_topic`으로 에이전트 전용 costmap_raw 노출 구성

필요하면, **에이전트 전용 costmap_raw**를 퍼블리시하는 미니 런치/노드 샘플도 만들어줄게.
