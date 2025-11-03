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
#####################  
########################  
#######################  
좋아, `PathValidatorNode`에 **MultiAgentInfoArray**를 구독해 “에이전트가 원인인 충돌”을 잡았을 때 **PathAgentCollisionInfo**를 꽉 채워 퍼블리시하도록 전면 수정해놨어.
핵심은 `master(=global_costmap/costmap_raw)`가 차단이고, `agent_mask`(=AgentLayer 전용 costmap)가 그 근방에서 lethal 이상이면 “타 로봇 원인”으로 간주 → 그 좌표를 기준으로 최근 `MultiAgentInfoArray`에서 **발자국(footprint)으로 실제 겹치는 에이전트**를 찾아 `machine_id / type_id / x / y / ttc_first / note`를 채운다(여러 로봇이 겹치면 배열에 모두 추가).

> ⚠️ 아래 코드에서 메시지 include는 네가 실제 만든 패키지명으로 바꿔줘요.
> 예시: `#include "replan_monitor_msgs/msg/path_agent_collision_info.hpp"`

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

// ★ 네 패키지명으로 교체
#include "replan_monitor_msgs/msg/path_agent_collision_info.hpp"

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

  bool isBlockedCellKernel(unsigned int mx, unsigned int my) const;

  void validateWithFootprint(const std::vector<geometry_msgs::msg::PoseStamped> & gpath);
  void validateWithPoints(const std::vector<geometry_msgs::msg::PoseStamped> & gpath);

  inline bool masterCellBlocked(unsigned int mx, unsigned int my, unsigned char thr) const;
  inline bool agentCellBlockedNear(unsigned int mx, unsigned int my,
                                   unsigned char thr, int manhattan_buf) const;

  static bool pointInPolygon(const std::vector<geometry_msgs::msg::Point> & poly,
                             double x, double y);

  // === Agent 충돌 식별 ===
  struct AgentHit {
    uint16_t machine_id{0};
    std::string type_id;
    double x{0.0}, y{0.0};     // 충돌 포인트
    float ttc_first{-1.0f};    // 간이 TTC (정면 접근 & 속도>0일 때)
    std::string note;
  };

  std::vector<AgentHit> whoCoversPoint(double wx, double wy) const;
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
  rclcpp::Publisher<replan_monitor_msgs::msg::PathAgentCollisionInfo>::SharedPtr agent_collision_pub_;

  rclcpp::TimerBase::SharedPtr obstacle_db_update_timer_;
  rclcpp::TimerBase::SharedPtr flag_reset_timer_;

  // ========= State =========
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  mutable std::mutex costmap_mutex_;
  CostmapSignature last_costmap_sig_;

  std::shared_ptr<nav2_costmap_2d::Costmap2D> agent_mask_;
  mutable std::mutex agent_mask_mutex_;
  CostmapSignature last_agent_sig_;

  // 최신 MultiAgentInfoArray
  multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr last_agents_;
  rclcpp::Time last_agents_stamp_;
  mutable std::mutex agents_mutex_;

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

  // Footprint / Agent mask / Output
  bool use_footprint_check_;
  double footprint_step_m_;
  bool compare_agent_mask_;
  std::string agent_mask_topic_;
  double agent_cost_threshold_;
  int agent_mask_manhattan_buffer_;

  // 충돌 메시지
  bool publish_agent_collision_;
  std::string agent_collision_topic_;

  // MultiAgent 구독
  std::string agents_topic_;
  int agents_freshness_timeout_ms_;
  double agent_match_dilate_m_;   // 충돌판정 시 footprint 소폭 확장

  // Nav2 footprint
  std::string footprint_str_;
  std::vector<geometry_msgs::msg::Point> footprint_;
  bool use_radius_{true};
  double robot_radius_m_{0.1};
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
  // ===== 기본 파라미터 =====
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

  // ===== NEW: Footprint / Agent mask / Output =====
  this->declare_parameter("use_footprint_check", false);
  this->declare_parameter("footprint_step_m", 0.15);

  this->declare_parameter("compare_agent_mask", true);
  this->declare_parameter<std::string>("agent_mask_topic", "/agent_layer/costmap_raw");
  this->declare_parameter("agent_cost_threshold", 254.0);
  this->declare_parameter("agent_mask_manhattan_buffer", 1);

  this->declare_parameter("publish_agent_collision", true);
  this->declare_parameter<std::string>("agent_collision_topic", "/path_agent_collision_info");

  // MultiAgent 구독
  this->declare_parameter<std::string>("agents_topic", "/multi_agent_infos");
  this->declare_parameter("agents_freshness_timeout_ms", 800);
  this->declare_parameter("agent_match_dilate_m", 0.05);

  // Nav2 footprint 스타일
  this->declare_parameter<std::string>("footprint", "[]");
  this->declare_parameter("robot_radius", 0.1);

  // ---- load parameters ----
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

  use_footprint_check_        = this->get_parameter("use_footprint_check").as_bool();
  footprint_step_m_           = std::max(0.05, this->get_parameter("footprint_step_m").as_double());

  compare_agent_mask_         = this->get_parameter("compare_agent_mask").as_bool();
  agent_mask_topic_           = this->get_parameter("agent_mask_topic").as_string();
  agent_cost_threshold_       = this->get_parameter("agent_cost_threshold").as_double();
  agent_mask_manhattan_buffer_= std::max<int>(0, static_cast<int>(this->get_parameter("agent_mask_manhattan_buffer").as_int()));

  publish_agent_collision_    = this->get_parameter("publish_agent_collision").as_bool();
  agent_collision_topic_      = this->get_parameter("agent_collision_topic").as_string();

  agents_topic_               = this->get_parameter("agents_topic").as_string();
  agents_freshness_timeout_ms_= this->get_parameter("agents_freshness_timeout_ms").as_int();
  agent_match_dilate_m_       = this->get_parameter("agent_match_dilate_m").as_double();

  // Nav2 footprint / radius
  footprint_str_              = this->get_parameter("footprint").as_string();
  robot_radius_m_             = this->get_parameter("robot_radius").as_double();
  use_radius_                 = true;
  footprint_.clear();
  if (!footprint_str_.empty() && footprint_str_ != "[]") {
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

  agents_sub_ = this->create_subscription<multi_agent_msgs::msg::MultiAgentInfoArray>(
      agents_topic_,
      rclcpp::QoS(10).best_effort(),   // 퍼블리셔 QoS에 맞춰 조절
      std::bind(&PathValidatorNode::agentsCallback, this, _1),
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

  // ===== Publishers =====
  {
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local().reliable();
    replan_pub_ = this->create_publisher<std_msgs::msg::Bool>("/replan_flag", qos);
  }
  if (publish_agent_collision_) {
    agent_collision_pub_ = this->create_publisher<replan_monitor_msgs::msg::PathAgentCollisionInfo>(
        agent_collision_topic_, rclcpp::QoS(10).reliable());
  }

  // ===== Timers =====
  const int period_ms = static_cast<int>(1000.0 / std::max(1.0, db_update_frequency_));
  obstacle_db_update_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&PathValidatorNode::updateObstacleDatabase, this),
      timer_callback_group_);

  RCLCPP_INFO(this->get_logger(),
    "PathValidatorNode ready. footprint=%s (radius=%.3f), use_footprint_check=%s, compare_agent_mask=%s, pub_agent_collision=%s, agents_topic=%s",
    (use_radius_ ? "radius" : "polygon"), robot_radius_m_,
    (use_footprint_check_ ? "true":"false"),
    (compare_agent_mask_ ? "true":"false"),
    (publish_agent_collision_ ? "true":"false"),
    agents_topic_.c_str());
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

// ===================== Agents handling =====================

void PathValidatorNode::agentsCallback(const multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(agents_mutex_);
  last_agents_ = msg;
  last_agents_stamp_ = msg->header.stamp;
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

// ===================== 기존 포인트 검사 =====================

void PathValidatorNode::validateWithPoints(const std::vector<geometry_msgs::msg::PoseStamped> & gpath)
{
  geometry_msgs::msg::Pose cur_pose;
  if (!getCurrentPoseFromTF(cur_pose)) return;

  const double max_dist_by_time = std::max(min_lookahead_m_, max_speed_ * lookahead_time_sec_);
  const double max_check_dist = std::min(max_dist_by_time, path_check_distance_m_);

  size_t best_streak = 0;
  size_t streak = 0;
  rclcpp::Time now = this->now();

  // DB 스냅샷
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
      if (agent_hit && publish_agent_collision_) {
        double wx, wy;
        {
          std::lock_guard<std::mutex> lock(costmap_mutex_);
          costmap_->mapToWorld(mx, my, wx, wy);
        }
        // MultiAgentInfoArray로 실제 가해자들 매칭
        auto hits = whoCoversPoint(wx, wy);
        if (!hits.empty()) publishAgentCollisionList(hits);
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

// ===================== Footprint 기반 검사 =====================

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

  // 경로 샘플링
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
    bool blocked_here = false;
    unsigned int hit_mx = 0, hit_my = 0;

    if (use_radius_) {
      unsigned int cx, cy;
      if (!costmap->worldToMap(ps.pose.position.x, ps.pose.position.y, cx, cy)) {
        consecutive = 0;
        continue;
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

          if (masterCellBlocked(umx, umy, master_thr)) {
            blocked_here = true;
            hit_mx = umx; hit_my = umy;
          }
        }
      }
    } else {
      // 다각형 footprint → 월드 폴리곤
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

      for (int j = min_j; j <= max_j && !blocked_here; ++j) {
        for (int i = min_i; i <= max_i && !blocked_here; ++i) {
          double wx, wy; costmap->mapToWorld(i, j, wx, wy);
          if (!pointInPolygon(poly_world, wx, wy)) continue;

          const unsigned int umx = static_cast<unsigned int>(i);
          const unsigned int umy = static_cast<unsigned int>(j);

          if (masterCellBlocked(umx, umy, master_thr)) {
            blocked_here = true;
            hit_mx = umx; hit_my = umy;
          }
        }
      }
    }

    if (blocked_here) {
      if (compare_agent_mask_) {
        const bool agent_hit = agentCellBlockedNear(hit_mx, hit_my, agent_thr,
                                                    agent_mask_manhattan_buffer_);
        if (agent_hit && publish_agent_collision_) {
          double wx, wy; {
            std::lock_guard<std::mutex> lock(costmap_mutex_);
            costmap->mapToWorld(hit_mx, hit_my, wx, wy);
          }
          auto hits = whoCoversPoint(wx, wy);
          if (!hits.empty()) publishAgentCollisionList(hits);
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

// ===================== Agent 충돌 식별/퍼블리시 =====================

double PathValidatorNode::headingTo(const geometry_msgs::msg::Pose & pose, double wx, double wy)
{
  const double yaw = tf2::getYaw(pose.orientation);
  const double dx = wx - pose.position.x;
  const double dy = wy - pose.position.y;
  const double tgt = std::atan2(dy, dx);
  double d = tgt - yaw;
  // wrap to [-pi,pi]
  while (d > M_PI) d -= 2*M_PI;
  while (d < -M_PI) d += 2*M_PI;
  return d; // 타겟 방향이 로봇 heading 대비 얼마만큼 틀어져 있는지
}

double PathValidatorNode::speedAlong(const geometry_msgs::msg::Twist & tw, double heading_rad)
{
  // base_link 기준 x 전진 속도를 heading 기준으로 투영(단순화)
  // heading_rad=0이면 정면
  const double v = tw.linear.x;
  return v * std::cos(heading_rad);
}

std::vector<PathValidatorNode::AgentHit> PathValidatorNode::whoCoversPoint(double wx, double wy) const
{
  std::vector<AgentHit> out;

  std::lock_guard<std::mutex> lk(agents_mutex_);
  if (!last_agents_) return out;

  // freshness 체크
  if ((this->now() - last_agents_stamp_).nanoseconds() >
       static_cast<int64_t>(agents_freshness_timeout_ms_) * 1000000LL) {
    return out;
  }

  // frame 확인(선택): 다르면 스킵
  if (!last_agents_->header.frame_id.empty() &&
      last_agents_->header.frame_id != global_frame_) {
    return out;
  }

  for (const auto & a : last_agents_->agents) {
    // footprint 다각형을 소폭 확장해 매칭 관용도 확보
    const auto & fp = a.footprint.polygon.points;
    if (fp.size() < 3) continue;

    // 로컬 footprint → 등방성 소확장
    std::vector<geometry_msgs::msg::Point> poly_local; poly_local.reserve(fp.size());
    // centroid
    double cx=0, cy=0;
    for (auto & p : fp) { cx += p.x; cy += p.y; }
    cx /= static_cast<double>(fp.size());
    cy /= static_cast<double>(fp.size());
    for (auto & p : fp) {
      double vx = p.x - cx, vy = p.y - cy;
      double n = std::hypot(vx, vy); if (n < 1e-6) n = 1.0;
      geometry_msgs::msg::Point q;
      q.x = p.x + agent_match_dilate_m_ * (vx / n);
      q.y = p.y + agent_match_dilate_m_ * (vy / n);
      q.z = 0.0;
      poly_local.push_back(q);
    }

    // 로컬→월드
    const double yaw = tf2::getYaw(a.current_pose.pose.orientation);
    const double c = std::cos(yaw), s = std::sin(yaw);
    std::vector<geometry_msgs::msg::Point> poly_world; poly_world.reserve(poly_local.size());
    for (auto & p : poly_local) {
      geometry_msgs::msg::Point q;
      q.x = a.current_pose.pose.position.x + c * p.x - s * p.y;
      q.y = a.current_pose.pose.position.y + s * p.x + c * p.y;
      q.z = 0.0;
      poly_world.push_back(q);
    }

    if (pointInPolygon(poly_world, wx, wy)) {
      AgentHit hit;
      hit.machine_id = a.machine_id;
      hit.type_id = a.type_id;
      hit.x = wx; hit.y = wy;
      // 간이 TTC: 현재 헤딩 쪽으로 접근 중일 때만 거리/속력
      const double head = headingTo(a.current_pose.pose, wx, wy);
      const double v_along = speedAlong(a.current_twist, head);
      if (v_along > 0.05) { // 임계속도
        const double dx = wx - a.current_pose.pose.position.x;
        const double dy = wy - a.current_pose.pose.position.y;
        const double dist = std::hypot(dx, dy);
        hit.ttc_first = static_cast<float>(dist / v_along);
        hit.note = "agent footprint overlap; TTC estimated";
      } else {
        hit.ttc_first = -1.0f;
        hit.note = "agent footprint overlap; TTC unknown";
      }
      out.emplace_back(std::move(hit));
    }
  }

  return out;
}

void PathValidatorNode::publishAgentCollisionList(const std::vector<AgentHit> & hits)
{
  if (!publish_agent_collision_ || !agent_collision_pub_) return;
  if (hits.empty()) return;

  replan_monitor_msgs::msg::PathAgentCollisionInfo msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = global_frame_;

  for (const auto & h : hits) {
    msg.machine_id.push_back(h.machine_id);
    msg.type_id.push_back(h.type_id);
    msg.x.push_back(h.x);
    msg.y.push_back(h.y);
    msg.ttc_first.push_back(h.ttc_first);
    msg.note.push_back(h.note);
  }

  agent_collision_pub_->publish(msg);
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

* 파라미터:

  * `agent_mask_topic` 는 AgentLayer가 퍼블리시하는 `/agent_layer/costmap_raw` 로 맞춰야 해.
  * `agents_topic` 는 `/multi_agent_infos`.
  * `agent_match_dilate_m` 로 매칭 관용도(충돌판정 시 footprint 살짝 키움)를 조절.
  * `agents_freshness_timeout_ms` 안에만 들어온 `MultiAgentInfoArray` 를 신뢰.
  * TTC는 간이 계산이라 필요하면 실제 경로 예측 기반으로 고도화 가능.

* CMake/패키지:

  * `multi_agent_msgs`, `replan_monitor_msgs` 를 `ament_target_dependencies()` 와 `package.xml` 의 `build_depend`, `exec_depend` 에 추가해줘.

원하는 흐름(마스터 차단 → agent_mask 확인 → 실제 MultiAgent로 가해자 식별 → 커스텀 메시지 퍼블리시)을 그대로 구현해 두었어.
테스트하면서 매칭이 과하거나 부족하면 `agent_mask_manhattan_buffer`, `agent_match_dilate_m`, `agent_cost_threshold`를 먼저 손봐줘.
#####################  
######################  
좋아! 아래에 **`path_validator_node`**용 샘플 `yaml`과, 포함된 **모든 파라미터 설명**을 정리했어. 그대로 붙여 넣고 값만 너 환경에 맞게 바꾸면 된다.

---

# 샘플 `nav2_params.yaml` (혹은 별도 params 파일)

```yaml
path_validator_node:
  ros__parameters:

    # --- 좌표계 / TF ---
    global_frame: "map"
    base_frame: "base_link"

    # --- 리플랜 트리거 일반 조건 ---
    cooldown_sec: 1.0                 # 리플랜 트리거 쿨다운(초)
    consecutive_threshold: 3           # 연속 차단 포인트 개수 임계값
    obstacle_persistence_sec: 0.5      # 차단 셀의 지속 시간 요구치(초)
    max_speed: 0.5                     # 로봇 속도 상한(lookahead 계산용)
    lookahead_time_sec: 15.0           # 속도 기반 lookahead 시간(초)
    min_lookahead_m: 2.0               # lookahead 최소 거리(m)
    cost_threshold: 200.0              # 마스터 코스트맵 차단 임계(권장: 200~254)
    ignore_unknown: true               # NO_INFORMATION(255) 무시 여부

    # --- 차단 후보 DB(ROI 스캔) ---
    db_update_frequency: 5.0           # DB 갱신 주기(Hz)
    obstacle_prune_timeout_sec: 3.0    # DB에서 관측 끊긴 셀 제거 시간(초)
    db_stride: 2                       # ROI 스캔 격자 스텝(1=전셀, 2=샘플링)
    cone_angle_deg: 100.0              # 전방 부채꼴 각도(도)
    kernel_half_size: 1                # 2*K+1 커널(1 => 3x3)로 주변 셀 검사

    # --- 경로 검사 범위 ---
    path_check_distance_m: 6.0         # 경로 따라 검사하는 최대 거리(m)

    # --- 리플랜 플래그 퍼블리시 ---
    publish_false_pulse: true          # true 펄스 후 false로 클리어할지
    flag_pulse_ms: 120                 # false 펄스 지연(ms)

    # --- 검사 방식 전환(포인트 vs 로봇 footprint) ---
    use_footprint_check: false         # true면 footprint 방식, false면 점방식
    footprint_step_m: 0.15             # footprint 방식에서 경로 샘플 간격(m)

    # --- 에이전트 마스크(AgentLayer 코스트맵) 비교 ---
    compare_agent_mask: true
    agent_mask_topic: "/agent_layer/costmap_raw"  # AgentLayer가 퍼블리시하는 마스크 토픽
    agent_cost_threshold: 254.0        # 에이전트 마스크에서 차단으로 간주하는 임계
    agent_mask_manhattan_buffer: 1     # 맨해튼 반경(셀) 내에 마스크가 있으면 히트로 간주

    # --- MultiAgentInfoArray 연동(가해자 식별용) ---
    agents_topic: "/multi_agent_infos" # MultiAgentInfoArray 토픽
    agents_freshness_timeout_ms: 800   # 최신 에이전트 정보 허용 지연(ms)
    agent_match_dilate_m: 0.05         # 가해자 매칭 시 footprint 소확장(m)

    # --- 충돌 메타 퍼블리시(커스텀 메시지) ---
    publish_agent_collision: true
    agent_collision_topic: "/path_agent_collision_info"

    # --- Nav2 footprint/radius (둘 중 하나만 실사용) ---
    # footprint 가 유효한 다각형 문자열이면 그것을 사용,
    # 아니면 robot_radius 를 원형 발자국으로 사용
    footprint: "[]"                    # 예: "[[0.3,0.25],[0.3,-0.25],[-0.3,-0.25],[-0.3,0.25]]"
    robot_radius: 0.30                 # footprint 미지정 시 사용되는 반지름(m)
```

> 노드 이름(`path_validator_node`)은 실행 파일에서 쓰는 실제 이름과 맞춰줘.

---

# 파라미터 상세 설명

### 좌표계 / TF

* **`global_frame`** *(string, default: `"map"`)*
  내부 모든 연산(코스트맵, 경로, 에이전트)을 맞추는 기준 프레임.
* **`base_frame`** *(string, default: `"base_link"`)*
  로봇의 베이스 링크 프레임. 현재 포즈 TF 조회에 사용.

### 리플랜 트리거 일반 조건

* **`cooldown_sec`** *(double, 1.0)*
  리플랜 트리거 후 다음 트리거까지 최소 대기시간.
* **`consecutive_threshold`** *(int, 3)*
  경로를 따라 연속으로 “차단” 판정된 샘플 수가 이 값 이상이면 리플랜.
* **`obstacle_persistence_sec`** *(double, 0.5)*
  동일 셀이 일정 시간 이상 연속으로 차단이어야 “진짜 장애물”로 간주.
* **`max_speed`** *(double, 0.5 m/s)*
  lookahead 거리 산정(= `max_speed * lookahead_time_sec`)에 사용되는 상한.
* **`lookahead_time_sec`** *(double, 15.0)*
  속도 기반 lookahead 시간. 실제 검사 거리는 `min_lookahead_m`와의 max를 사용.
* **`min_lookahead_m`** *(double, 2.0 m)*
  lookahead 최소 거리 하한.
* **`cost_threshold`** *(double, 200.0)*
  글로벌(마스터) 코스트맵에서 차단으로 간주할 최소 코스트.

  * 레이저/장애물/인플레이션 포함 전체 코스트 기준.
  * 엄격하게 하려면 254, 살짝 여유는 200~240 권장.
* **`ignore_unknown`** *(bool, true)*
  코스트 255(NO_INFORMATION)를 무시할지 여부.

### 차단 후보 DB (전방 ROI 스캔)

* **`db_update_frequency`** *(double, 5.0 Hz)*
  전방 ROI를 스캔해 “차단 후보 셀” DB를 갱신하는 주기.
* **`obstacle_prune_timeout_sec`** *(double, 3.0)*
  DB에 저장된 셀이 이 시간 동안 안 보이면 제거.
* **`db_stride`** *(int, 2)*
  ROI 스캔 시 샘플링 스텝(셀 단위). 1이면 모든 셀 검사.
* **`cone_angle_deg`** *(double, 100.0°)*
  로봇 헤딩 기준 전방 부채꼴 각. 전방성 강조용.
* **`kernel_half_size`** *(int, 1)*
  포인트 검사에서 주변 `(2K+1)x(2K+1)` 셀을 함께 차단으로 볼지.

### 경로 검사 범위

* **`path_check_distance_m`** *(double, 6.0 m)*
  경로를 따라 검사하는 최대 거리(lookahead 거리와의 min 사용).

### 리플랜 플래그 퍼블리시

* **`publish_false_pulse`** *(bool, true)*
  `/replan_flag`를 true로 쏜 뒤, 잠시 후 false 한번 더 쏘아주는지.
* **`flag_pulse_ms`** *(int, 120 ms)*
  false 펄스 지연 시간.

### 검사 방식 전환

* **`use_footprint_check`** *(bool, false)*

  * `false`: 경로의 포인트(혹은 소커널)만 검사 → 가볍고 빠름.
  * `true`: 로봇 **footprint**(원형/다각형)를 경로 샘플마다 채워서 검사 → 충돌 실제성↑.
* **`footprint_step_m`** *(double, 0.15 m)*
  footprint 방식에서 경로 샘플 간격. 너무 작으면 무거워짐.

### 에이전트 마스크(AgentLayer) 비교

* **`compare_agent_mask`** *(bool, true)*
  마스터 코스트맵에서 차단이 보이면, **에이전트 마스크**에서도 같은 위치가 lethal인지 확인.
  → 다른 로봇이 원인인지 판별.
* **`agent_mask_topic`** *(string, `"/agent_layer/costmap_raw"`)*
  `AgentLayer`가 퍼블리시하는 전용 costmap_raw 토픽.
* **`agent_cost_threshold`** *(double, 254.0)*
  에이전트 마스크에서 차단으로 볼 최소 코스트(보통 lethal).
* **`agent_mask_manhattan_buffer`** *(int, 1)*
  (mx,my) 주변 맨해튼 반경 안에 에이전트 마스크가 있으면 히트로 인정.
  센서/해상도 오차 흡수.

### MultiAgentInfoArray 연동 (가해자 식별)

* **`agents_topic`** *(string, `"/multi_agent_infos"`)*
  다른 로봇 상태/발자국/경로가 들어 있는 토픽.
* **`agents_freshness_timeout_ms`** *(int, 800 ms)*
  최신 에이전트 메시지가 이 시간 이내여야 가해자 판정에 사용.
* **`agent_match_dilate_m`** *(double, 0.05 m)*
  가해자 매칭 시 상대 footprint를 소폭 키워 겹침 허용오차 확보.

### 충돌 메타 퍼블리시 (커스텀 메시지)

* **`publish_agent_collision`** *(bool, true)*
  가해자 판정이 나면 `PathAgentCollisionInfo`를 퍼블리시할지.
* **`agent_collision_topic`** *(string, `"/path_agent_collision_info"`)*
  커스텀 메시지 퍼블리시 토픽.

### Nav2 footprint / radius

* **`footprint`** *(string, `"[]"`)*
  다각형 footprint 문자열. 유효하면 **이 값을 우선** 사용.
  예시:
  `[[0.30,0.25],[0.30,-0.25],[-0.30,-0.25],[-0.30,0.25]]`
* **`robot_radius`** *(double, 0.30 m)*
  `footprint`가 비었거나 파싱 실패 시 **원형 footprint**로 사용되는 반지름.

---

## 튜닝 가이드 (짧게)

* **민감도(리플랜 많이 뜸)**
  `cost_threshold` 낮춤(200→170), `consecutive_threshold` 낮춤(3→2),
  `kernel_half_size`↑, `use_footprint_check`=true, `footprint_step_m`↓.
* **보수적(오탐 줄임)**
  `cost_threshold`↑(200→254), `consecutive_threshold`↑,
  `obstacle_persistence_sec`↑, `agent_mask_manhattan_buffer`↓.
* **에이전트 판별 강도**
  더 강하게: `agent_cost_threshold`=254, `agent_mask_manhattan_buffer`=0~1
  느슨하게: `agent_cost_threshold`=200, `agent_mask_manhattan_buffer`=2~3
* **성능**
  `use_footprint_check=false`(가벼움), `db_stride`↑, `footprint_step_m`↑.

필요하면 네 현재 맵 해상도, footprint, 마스크 특성에 맞춰 초깃값을 추천해줄게!
#####################  
######################  
#####################  
```
[초안 custom msg(PathAgentCollisionInfo.msg)]
# 코스트 셀과 결부된 "로봇 유래" 메타정보 (희소 형태)
# 좌표: map 좌표계(world) 

std_msgs/Header header         # frame_id: map
uint16[] machine_id
string[] type_id

# 수치 요약(있으면 채움)
float64[] x                      # 경로상의 최초 충돌 지점 x
float64[] y                      # 경로상의 최초 충돌 지점 y 
float32[]  ttc_first                     # 최초 추돌 예상시각 [s], 모르면 음수
string[]   note 
```
########################  
#########################  
#########################  
이 에러는 **서로 다른 시간 소스(Clock Type)의 rclcpp::Time 끼리 뺄셈**을 해서 터진 겁니다.
메시지 타임스탬프(예: `msg->header.stamp`)는 “소스가 없는” `builtin_interfaces/Time` 이고, 당신 노드의 `this->now()` 는 노드의 Clock 타입(ROS_TIME / SYSTEM_TIME)을 따릅니다. 지금 코드에서

* `last_agents_stamp_ = msg->header.stamp;`
* `if ((this->now() - last_agents_stamp_).nanoseconds() > ...) { ... }`

처럼 섞어 쓰면서 **[1 != 2]** (예: ROS_TIME vs SYSTEM_TIME) 충돌이 났습니다.
`last_replan_time_`의 초기값(디폴트 생성)과 `this->now()`의 Clock 타입이 달라도 같은 문제가 납니다.

아래처럼 “**항상 같은 Clock Type으로 래핑**”해주면 해결됩니다.

---

# 필수 패치(최소 수정)

### 1) 생성자에서 기준 Clock Type 확보 & 초기화

```cpp
// ctor 끝부분 어딘가에서, 반드시 한 번 실행
const rcl_clock_type_t CT = this->get_clock()->get_clock_type();

// last_replan_time_을 노드 clock 타입으로 초기화(에폭 0)
last_replan_time_ = rclcpp::Time(0, 0, CT);
```

### 2) Agents 콜백에서 stamp를 노드 clock 타입으로 변환해 저장

```cpp
void PathValidatorNode::agentsCallback(
    const multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(agents_mutex_);
  last_agents_ = msg;

  // ★ 핵심: 노드 clock 타입으로 래핑
  const auto CT = this->get_clock()->get_clock_type();
  last_agents_stamp_ = rclcpp::Time(msg->header.stamp, CT);
}
```

### 3) freshness 체크도 같은 타입으로

```cpp
// whoCoversPoint() 내부 freshness 체크 부분을 다음처럼 유지하면 OK
if ((this->now() - last_agents_stamp_).nanoseconds() >
     static_cast<int64_t>(agents_freshness_timeout_ms_) * 1000000LL) {
  return out;
}
```

### 4) replan 쿨다운도 같은 타입으로

```cpp
void PathValidatorNode::triggerReplan(const std::string & reason)
{
  const rclcpp::Time now = this->now();  // 노드 clock
  // last_replan_time_은 ctor에서 같은 clock으로 초기화됨
  if ((now - last_replan_time_).seconds() <= cooldown_sec_) {
    return;
  }
  last_replan_time_ = now;
  ...
}
```

> 위 네 군데만 고치면 `can't subtract times with different time sources`는 사라집니다.

---

# 권장(선택) 개선

* **지속시간 측정(쿨다운, DB 만료 등)은 STEADY_CLOCK 사용**이 가장 안전합니다(시뮬/ROS time 변경 영향 없음).
  예: `rclcpp::Clock steady_clock(RCL_STEADY_TIME);`를 멤버로 두고,
  `rclcpp::Time now_steady = steady_clock.now();` 로 쿨다운/DB시간을 관리.
  (메시지/헤더에 넣는 stamp 는 기존대로 `this->now()` 유지)

* 다른 메시지 stamp 를 내부 상태로 저장할 때도 동일 원칙:
  `rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type())`

---

# 왜 이런 일이 생기나?

* ROS2 에서 `rclcpp::Time` 은 **Clock Type** 을 가진 시간입니다(ROS_TIME, SYSTEM_TIME, STEADY_TIME 등).
* `this->now()` 는 노드의 현재 Clock(보통 시스템 시간, `use_sim_time=true`면 ROS 시간).
* 반면 `builtin_interfaces/Time` → `rclcpp::Time` 로 암시 변환하면 **SYSTEM_TIME** 로 잡히는 경우가 많습니다.
* 그래서 ROS_TIME(1) - SYSTEM_TIME(2) 처럼 타입이 다르면 예외가 납니다.

---

필요하면 제가 **STEADY_CLOCK 버전으로 리팩터링한 diff**도 바로 만들어 줄게요.
######################  
#######################  
########################  
좋아, 증상 3가지를 각각 정리해서 **원인 → 수정 포인트(코드) → 체크리스트**로 바로 잡아줄게.

---

# 1) `what(): can't subtract times with different time sources` 해결

`last_replan_time_`만 초기화해 준 걸로 1차 해결됐지만, **`MultiAgentInfoArray`의 `header.stamp`와 노드 clock 타입이 다르면** 똑같은 문제가 다른 경로(가해자 식별 freshness 체크)에서도 다시 터질 수 있어요.
→ **에이전트 메시지의 stamp도 노드 clock 타입으로 래핑해서 저장**해야 합니다.

### 수정 (CPP)

```cpp
// 기존
// last_agents_stamp_ = msg->header.stamp;

// 변경
last_agents_stamp_ = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
```

그리고 비교부에서도 굳이 캐스팅할 필요 없이 지금처럼 `this->now()`와 뺄셈하면 clock type이 맞으니 OK.

---

# 2) 에이전트 충돌 메타가 안 퍼블리시되는 원인 (마스터 vs 에이전트 costmap 인덱스 불일치)

현재 코드는 마스터 코스트맵에서 찾은 `(mx,my)` **인덱스를 그대로** 에이전트 마스크에서 검사해요.
하지만 **두 코스트맵의 origin / resolution / size가 조금이라도 다르면** 같은 인덱스가 다른 세계좌표를 가리킵니다. 그래서 agent 마스크 히트를 **영영 못 잡는** 경우가 생겨요.

### 해결 전략

* 마스터에서 차단 셀을 찾으면 **반드시 (wx, wy) 월드좌표**로 바꾸고,
* 에이전트 마스크는 **그 월드좌표를 `worldToMap`** 해서 자기 인덱스 공간으로 변환한 뒤 주변을 검사해야 합니다.

### 헤더(HPP) 변경

```cpp
// 기존
inline bool agentCellBlockedNear(unsigned int mx, unsigned int my,
                                 unsigned char thr, int manhattan_buf) const;

// 추가 (월드좌표 기반 검사)
bool agentMaskBlockedNearWorld(double wx, double wy,
                               unsigned char thr, int manhattan_buf) const;
```

### 구현(CPP) 추가

```cpp
bool PathValidatorNode::agentMaskBlockedNearWorld(double wx, double wy,
                                                  unsigned char thr, int manhattan_buf) const
{
  std::lock_guard<std::mutex> lock(agent_mask_mutex_);
  if (!agent_mask_) return false;

  unsigned int cx, cy;
  if (!agent_mask_->worldToMap(wx, wy, cx, cy)) {
    return false;
  }

  const int sx = static_cast<int>(agent_mask_->getSizeInCellsX());
  const int sy = static_cast<int>(agent_mask_->getSizeInCellsY());
  const int ix = static_cast<int>(cx);
  const int iy = static_cast<int>(cy);

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
```

### 호출부 변경(포인트 방식)

```cpp
// 기존 (마스터 인덱스 바로 검사)
const bool agent_hit = agentCellBlockedNear(mx, my,
                         static_cast<unsigned char>(agent_cost_threshold_),
                         agent_mask_manhattan_buffer_);

// 변경 (월드좌표로 변환 후 에이전트 마스크 검사)
double wx, wy;
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  costmap_->mapToWorld(mx, my, wx, wy);
}
const bool agent_hit = agentMaskBlockedNearWorld(wx, wy,
                        static_cast<unsigned char>(agent_cost_threshold_),
                        agent_mask_manhattan_buffer_);
```

### 호출부 변경(footprint 방식)

```cpp
// 기존
const bool agent_hit = agentCellBlockedNear(hit_mx, hit_my, agent_thr,
                                            agent_mask_manhattan_buffer_);

// 변경
double wx, wy; {
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  costmap->mapToWorld(hit_mx, hit_my, wx, wy);
}
const bool agent_hit = agentMaskBlockedNearWorld(wx, wy, agent_thr,
                                                 agent_mask_manhattan_buffer_);
```

이렇게 하면 **마스터/에이전트 코스트맵 메타데이터가 달라도** 정확히 매칭돼서, agent 유래 충돌을 안정적으로 검출하고 `PathAgentCollisionInfo`가 퍼집니다.

> 팁: 우선 값이 잘 나오나 보려면 `agent_cost_threshold`를 임시로 `200.0` 정도로 낮춰 테스트해 보세요. 이후 254로 올려 타이트하게 쓰면 됩니다.

---

# 3) “다른 로봇보다 약간 크게 생긴 코스트가 안 사라짐” 원인

이건 `path_validator_node`가 아니라 **`AgentLayer` 구현 방식** 때문에 생길 확률이 매우 높습니다.

* 지금 `AgentLayer`가 **`master_grid.setCost()`로 직접 마스터에 그려 넣는 방식**(Max-merge 수동 구현)일 텐데,
  이 방법은 **에이전트가 사라져도 지난 프레임 셀을 지울 방법이 없습니다.**
* 정석은: **이 레이어의 내부 costmap(`costmap_`)에만 그림 → 각 주기마다 해당 레이어 내부맵을 초기화(clear) → 마지막에 `updateWithMax()`(혹은 Overwrite)** 로 마스터에 합치는 구조입니다. 이렇게 해야 **에이전트가 사라지면 자동으로 흔적이 지워**져요.

### 빠른 임시 방편 (AgentLayer 쪽)

* 최소한 `updateCosts()` 시작 시 **지난 번 touched bounds를 FREE_SPACE로 초기화**하고 다시 그리도록 바꾸세요.
  하지만 이것도 안전치 않습니다(정확한 bound 계산/클리어가 어렵고 race가 생길 수 있음).
* **권장:** Nav2 튜토리얼 패턴처럼 **내부맵 사용 + `updateWithMax()`** 로 리팩터링하세요.

---

# 4) 전체 패치 요약 (넣어야 할 것만 정리)

아래 4군데만 바꾸면 됩니다.

## (A) `agentsCallback`에서 stamp 저장

```cpp
void PathValidatorNode::agentsCallback(
  const multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(agents_mutex_);
  last_agents_ = msg;
  last_agents_stamp_ = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
}
```

## (B) HPP에 월드좌표 기반 검사 시그니처 추가

```cpp
bool agentMaskBlockedNearWorld(double wx, double wy,
                               unsigned char thr, int manhattan_buf) const;
```

## (C) CPP에 `agentMaskBlockedNearWorld` 구현 추가

(위에 제시한 그대로)

## (D) 두 군데 호출부 수정

* `validateWithPoints()` 내부: 마스터 인덱스→월드좌표로 바꿔 `agentMaskBlockedNearWorld()` 호출
* `validateWithFootprint()` 내부: `hit_mx, hit_my`→월드좌표→`agentMaskBlockedNearWorld()` 호출

---

# 5) 빠른 체크리스트

* [ ] `agent_mask_topic`의 코스트맵 메타(해상도/원점/크기)가 글로벌과 다를 수 있습니다 → **월드좌표로 매칭**하도록 위 패치 적용.
* [ ] `agent_cost_threshold`를 200으로 낮춰 먼저 **히트되는지 확인**, 이후 254로 조정.
* [ ] `agents_freshness_timeout_ms`를 1000~1500ms로 잠깐 넉넉하게 주고 실시간 히트 확인.
* [ ] 충돌 발생 시 `PathAgentCollisionInfo` 실제 값(IDs, x,y, ttc, note) 확인.
* [ ] 지워지지 않는 큰 코스트는 **AgentLayer 합성 방식 리팩터**(내부맵 사용 + `updateWithMax()`).

필요하면 AgentLayer 쪽도 “내부맵 + 클리어 + updateWithMax”로 바꾼 완전체 코드를 다시 내줄게.

  #################  
  ##################  
오케이—지금 상황은 “마스터 코스트맵으로 막힘은 인지 → 리플랜 트리거 OK”인데, “agent 유래인지 판별해서 PathAgentCollisionInfo 퍼블리시”가 안 되는 거죠. 대부분 아래 3가지 중 하나 때문에 그렇습니다.

1. **에이전트 마스크(costmap_raw)와 마스터 코스트맵의 좌표 변환/메타 불일치**
2. **에이전트 마스크 자체가 안 들어오거나(latest 없는 상태) QoS 미스매치**
3. **마스크 확인 통과에 실패해서(약간 비켜간 셀/threshold/버퍼 문제) footprint 직접 대조까지 안 가는 로직 흐름**

아래 “확인 체크리스트”를 먼저 돌리고, 바로 효과가 나는 **코드 패치(디버그 로그 + 강건화)** 를 붙여드립니다.

---

# 빠른 확인 체크리스트 (1분 점검)

* `ros2 topic echo /agent_layer/costmap_raw/metadata | head -n 30`
  `resolution, origin(x,y), size_x,y` 값이 **/global_costmap/costmap_raw** 와 다를 수 있어요. 다르면 **반드시 월드좌표로 매칭**해야 합니다(아래 패치에서 해결).

* `ros2 topic hz /agent_layer/costmap_raw`
  수신이 0Hz면 마스크가 안 들어옵니다 → `compare_agent_mask:=false` 로 먼저 테스트해 보세요(아래 ❷).

* `ros2 topic echo /multi_agent_infos | head`
  프레임이 `map` 인지, 타임스탬프가 최신인지 확인. Stale이면 히트 안 납니다(아래에서 freshness 로그 추가).

* `agent_cost_threshold` 임시로 **180** 까지 낮추고, `agent_mask_manhattan_buffer:=2` 로 넉넉하게.

---

# 바로 적용하는 코드 패치 (핵심 4가지)

## ❶ 마스크-확인: 반드시 월드좌표로 검사 + 진단 로그 추가

이미 월드좌표 기반 함수 추가하셨다면, **호출부에서 로그와 fallback을 추가**하세요.

```cpp
// (validateWithPoints / validateWithFootprint)에서 blocked_here가 true일 때:

double wx, wy;
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  costmap->mapToWorld(hit_mx, hit_my, wx, wy);
}

// 1) 에이전트 마스크 사용 시: 월드좌표로 검사
bool agent_hit_mask = false;
if (compare_agent_mask_) {
  if (!agent_mask_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "[agent-mask] agent_mask_ not ready yet (no messages on %s)",
      agent_mask_topic_.c_str());
  } else {
    agent_hit_mask = agentMaskBlockedNearWorld(
        wx, wy,
        static_cast<unsigned char>(agent_cost_threshold_),
        agent_mask_manhattan_buffer_);
    RCLCPP_DEBUG(get_logger(),
      "[agent-mask] @ (%.3f, %.3f) hit=%s thr=%.0f buf=%d",
      wx, wy, agent_hit_mask ? "true":"false",
      agent_cost_threshold_, agent_mask_manhattan_buffer_);
  }
}
```

## ❷ 마스크 없을 때도 **footprint 직접 대조로 에이전트 판정**(옵션)

마스크가 없거나 히트 실패해도, 진짜 에이전트이면 **footprint 겹침(whoCoversPoint)** 으로 잡을 수 있어야 합니다.
아래처럼 **새 파라미터**를 하나 두고, false로 두면 footprint만으로도 충돌 메시지를 퍼블리시하게 만드세요.

```cpp
// HPP에 파라미터 추가
bool require_agent_mask_confirmation_{true};

// 생성자에서 declare + get:
declare_parameter("require_agent_mask_confirmation", true);
require_agent_mask_confirmation_ = get_parameter("require_agent_mask_confirmation").as_bool();

// 퍼블리시 분기:
bool should_publish_agent = false;
if (compare_agent_mask_) {
  if (agent_hit_mask) {
    should_publish_agent = true;
  } else if (!require_agent_mask_confirmation_) {
    RCLCPP_DEBUG(get_logger(), "[agent-mask] no hit, but fallback to footprint-match is enabled");
    should_publish_agent = true;  // 마스크 미확인이라도 footprint로 시도
  }
} else {
  should_publish_agent = true; // 마스크 비교 자체를 끔
}

if (should_publish_agent && publish_agent_collision_) {
  auto hits = whoCoversPoint(wx, wy);
  RCLCPP_DEBUG(get_logger(), "[whoCoversPoint] matches=%zu", hits.size());
  if (!hits.empty()) {
    publishAgentCollisionList(hits);
  } else {
    RCLCPP_DEBUG(get_logger(), "[whoCoversPoint] no agent polygon covers the point");
  }
}
```

> 테스트 팁: 일단 `compare_agent_mask:=false` 로 돌려보면, **마스크 없이 footprint로만** 잡는지 바로 확인 가능합니다. 이게 되면 마스크 경로/메타/QoS 문제로 좁혀집니다.

## ❸ `MultiAgentInfoArray` freshness & frame 진단 로그

```cpp
std::lock_guard<std::mutex> lk(agents_mutex_);
if (!last_agents_) {
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[agents] no data yet on %s", agents_topic_.c_str());
  return out;
}
const auto age_ns = (this->now() - last_agents_stamp_).nanoseconds();
if (age_ns > static_cast<int64_t>(agents_freshness_timeout_ms_) * 1000000LL) {
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
    "[agents] stale: age=%.3fs > %dms", age_ns*1e-9, agents_freshness_timeout_ms_);
  return out;
}
if (!last_agents_->header.frame_id.empty() && last_agents_->header.frame_id != global_frame_) {
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
    "[agents] frame mismatch: agents='%s' vs global='%s'",
    last_agents_->header.frame_id.c_str(), global_frame_.c_str());
  return out;
}
```

## ❹ `last_agents_stamp_` clock type 맞추기

```cpp
last_agents_stamp_ = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
```

---

# 예상되는 원인별 “즉시 효과” 튜닝

* **마스크가 안 온다/메타 다르다** → `compare_agent_mask:=false` 먼저 시도.
  이 상태에서 충돌 메시지가 뜨면, 문제는 마스크 경로/메타(QoS 포함)입니다.

* **셀 살짝 빗나감** → `agent_mask_manhattan_buffer:=2` 또는 `3`.
  코스트맵 해상도가 0.05m라면, 2~3 셀 버퍼는 0.1~0.15m 여유.

* **threshold 과도** → `agent_cost_threshold:=180 ~ 220`에서 먼저 감도 확인.

* **footprint 매칭이 빡빡** → `agent_match_dilate_m:=0.10 ~ 0.15` 로 키워서 도형을 살짝 확장.

---

# “커진 코스트가 안 없어지는 현상” 재확인

그건 `AgentLayer`가 **마스터 그리드에 직접 setCost**하는 패턴일 때 흔히 생깁니다.
가능한 빨리 **내부 costmap_ 사용 + 각 사이클 클리어 + `updateWithMax()`** 구조로 바꾸세요.
(원하시면 이 부분 완전체 코드도 다시 정리해 드릴게요.)

---

# 최소 실행 예시(강추 파라미터)

```bash
# 1) 먼저 마스크 비교 끄고 footprint 매칭만으로 충돌 퍼블리시가 되는지 확인
ros2 run replan_monitor path_validator_node \
  --ros-args -p compare_agent_mask:=false \
             -p publish_agent_collision:=true \
             -p use_footprint_check:=true \
             -p footprint_step_m:=0.1 \
             -p cost_threshold:=200.0 \
             -p consecutive_threshold:=2 \
             -p agent_match_dilate_m:=0.12
```

→ 이 상태에서 **충돌 메시지 뜨면** 마스크쪽 문제 확정.

```bash
# 2) 마스크 다시 켜되, 확인 강제하지 않고(footprint fallback 허용)
ros2 run replan_monitor path_validator_node \
  --ros-args -p compare_agent_mask:=true \
             -p require_agent_mask_confirmation:=false \
             -p agent_mask_topic:=/agent_layer/costmap_raw \
             -p agent_cost_threshold:=200.0 \
             -p agent_mask_manhattan_buffer:=2
```

→ 여기서도 잘 뜨면, 마지막으로 `require_agent_mask_confirmation:=true` 로 엄격 모드 복귀.

---

필요하면, 수정한 조각을 붙여서 **통째 파일** 형태로도 정리해 줄게요.
일단 위 패치 4개만 넣고 `compare_agent_mask:=false`로 테스트 → OK 확인 → 점차 엄격하게 올리는 순서로 가면 빠르게 원인이 좁혀집니다.
####################  
#####################  
#######################  
좋지! 아래는 “에이전트 홀드(hold) 타임”을 포함해 정리한 **완성본**입니다.
바뀐 점 요약:

* 새 파라미터: `agent_block_hold_sec`, `agent_block_max_wait_sec`
* 에이전트 유래 충돌을 퍼블리시하면 즉시 `last_agent_block_time_` 갱신 후 **콜백 종료(return)** → 같은 주기에서 리플랜 방지
* 이후 들어오는 콜백에서 `hold` 구간이면 전체 판단 **스킵**. `max_wait`를 넘기면 리플랜 다시 허용
* `last_replan_time_`은 노드 clock type과 동일하게 초기화

---

### path_validator_node.hpp

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

// ★ 네 패키지명으로 교체
#include "replan_monitor_msgs/msg/path_agent_collision_info.hpp"

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

  bool isBlockedCellKernel(unsigned int mx, unsigned int my) const;

  void validateWithFootprint(const std::vector<geometry_msgs::msg::PoseStamped> & gpath);
  void validateWithPoints(const std::vector<geometry_msgs::msg::PoseStamped> & gpath);

  inline bool masterCellBlocked(unsigned int mx, unsigned int my, unsigned char thr) const;
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

  std::vector<AgentHit> whoCoversPoint(double wx, double wy) const;
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
  rclcpp::Publisher<replan_monitor_msgs::msg::PathAgentCollisionInfo>::SharedPtr agent_collision_pub_;

  rclcpp::TimerBase::SharedPtr obstacle_db_update_timer_;
  rclcpp::TimerBase::SharedPtr flag_reset_timer_;

  // ========= State =========
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  mutable std::mutex costmap_mutex_;
  CostmapSignature last_costmap_sig_;

  std::shared_ptr<nav2_costmap_2d::Costmap2D> agent_mask_;
  mutable std::mutex agent_mask_mutex_;
  CostmapSignature last_agent_sig_;

  // 최신 MultiAgentInfoArray
  multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr last_agents_;
  rclcpp::Time last_agents_stamp_;
  mutable std::mutex agents_mutex_;

  std::atomic<bool> is_robot_in_driving_state_{false};
  rclcpp::Time last_replan_time_;        // replan 쿨다운 기준
  rclcpp::Time last_agent_block_time_;   // ★ 에이전트 홀드 기준

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

  // Footprint / Agent mask / Output
  bool use_footprint_check_;
  double footprint_step_m_;
  bool compare_agent_mask_;
  std::string agent_mask_topic_;
  double agent_cost_threshold_;
  int agent_mask_manhattan_buffer_;

  // 충돌 메시지
  bool publish_agent_collision_;
  std::string agent_collision_topic_;

  // MultiAgent 구독
  std::string agents_topic_;
  int agents_freshness_timeout_ms_;
  double agent_match_dilate_m_;

  // Nav2 footprint
  std::string footprint_str_;
  std::vector<geometry_msgs::msg::Point> footprint_;
  bool use_radius_{true};
  double robot_radius_m_{0.1};

  // ★ NEW: 에이전트 홀드 파라미터
  double agent_block_hold_sec_{2.0};
  double agent_block_max_wait_sec_{8.0};
};

}  // namespace replan_monitor

#endif  // PATH_VALIDATOR_NODE_HPP
```

---

### path_validator_node.cpp

```cpp
#include "replan_monitor/path_validator_node.hpp"

using std::placeholders::_1;

namespace replan_monitor
{

PathValidatorNode::PathValidatorNode()
: Node("path_validator_node")
{
  // ===== 기본 파라미터 =====
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

  // ===== Footprint / Agent mask / Output =====
  this->declare_parameter("use_footprint_check", false);
  this->declare_parameter("footprint_step_m", 0.15);

  this->declare_parameter("compare_agent_mask", true);
  this->declare_parameter<std::string>("agent_mask_topic", "/agent_layer/costmap_raw");
  this->declare_parameter("agent_cost_threshold", 254.0);
  this->declare_parameter("agent_mask_manhattan_buffer", 1);

  this->declare_parameter("publish_agent_collision", true);
  this->declare_parameter<std::string>("agent_collision_topic", "/path_agent_collision_info");

  // MultiAgent 구독
  this->declare_parameter<std::string>("agents_topic", "/multi_agent_infos");
  this->declare_parameter("agents_freshness_timeout_ms", 800);
  this->declare_parameter("agent_match_dilate_m", 0.05);

  // Nav2 footprint 스타일
  this->declare_parameter<std::string>("footprint", "[]");
  this->declare_parameter("robot_radius", 0.1);

  // ★ NEW: 에이전트 홀드 파라미터
  this->declare_parameter("agent_block_hold_sec", 2.0);
  this->declare_parameter("agent_block_max_wait_sec", 8.0);

  // ---- load parameters ----
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

  use_footprint_check_        = this->get_parameter("use_footprint_check").as_bool();
  footprint_step_m_           = std::max(0.05, this->get_parameter("footprint_step_m").as_double());

  compare_agent_mask_         = this->get_parameter("compare_agent_mask").as_bool();
  agent_mask_topic_           = this->get_parameter("agent_mask_topic").as_string();
  agent_cost_threshold_       = this->get_parameter("agent_cost_threshold").as_double();
  agent_mask_manhattan_buffer_= std::max<int>(0, static_cast<int>(this->get_parameter("agent_mask_manhattan_buffer").as_int()));

  publish_agent_collision_    = this->get_parameter("publish_agent_collision").as_bool();
  agent_collision_topic_      = this->get_parameter("agent_collision_topic").as_string();

  agents_topic_               = this->get_parameter("agents_topic").as_string();
  agents_freshness_timeout_ms_= this->get_parameter("agents_freshness_timeout_ms").as_int();
  agent_match_dilate_m_       = this->get_parameter("agent_match_dilate_m").as_double();

  // Nav2 footprint / radius
  footprint_str_              = this->get_parameter("footprint").as_string();
  robot_radius_m_             = this->get_parameter("robot_radius").as_double();
  use_radius_                 = true;
  footprint_.clear();
  if (!footprint_str_.empty() && footprint_str_ != "[]") {
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

  // ★ NEW: 홀드 파라미터
  agent_block_hold_sec_     = this->get_parameter("agent_block_hold_sec").as_double();
  agent_block_max_wait_sec_ = this->get_parameter("agent_block_max_wait_sec").as_double();

  // ===== TF =====
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ★ clock type 맞춰 초기화
  last_replan_time_       = rclcpp::Time(0,0,this->get_clock()->get_clock_type());
  last_agent_block_time_  = rclcpp::Time(0,0,this->get_clock()->get_clock_type());

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

  agents_sub_ = this->create_subscription<multi_agent_msgs::msg::MultiAgentInfoArray>(
      agents_topic_,
      rclcpp::QoS(10).best_effort(),
      std::bind(&PathValidatorNode::agentsCallback, this, _1),
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

  // ===== Publishers =====
  {
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local().reliable();
    replan_pub_ = this->create_publisher<std_msgs::msg::Bool>("/replan_flag", qos);
  }
  if (publish_agent_collision_) {
    agent_collision_pub_ = this->create_publisher<replan_monitor_msgs::msg::PathAgentCollisionInfo>(
        agent_collision_topic_, rclcpp::QoS(10).reliable());
  }

  // ===== Timers =====
  const int period_ms = static_cast<int>(1000.0 / std::max(1.0, db_update_frequency_));
  obstacle_db_update_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&PathValidatorNode::updateObstacleDatabase, this),
      timer_callback_group_);

  RCLCPP_INFO(this->get_logger(),
    "PathValidatorNode ready. footprint=%s (radius=%.3f), use_footprint_check=%s, compare_agent_mask=%s, pub_agent_collision=%s, agents_topic=%s, hold=%.2fs max_wait=%.2fs",
    (use_radius_ ? "radius" : "polygon"), robot_radius_m_,
    (use_footprint_check_ ? "true":"false"),
    (compare_agent_mask_ ? "true":"false"),
    (publish_agent_collision_ ? "true":"false"),
    agents_topic_.c_str(),
    agent_block_hold_sec_, agent_block_max_wait_sec_);
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

// ===================== Agents handling =====================

void PathValidatorNode::agentsCallback(const multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(agents_mutex_);
  last_agents_ = msg;
  last_agents_stamp_ = msg->header.stamp;
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

  // ★ NEW: 에이전트 홀드 타임이 유효하면 판단 스킵
  const rclcpp::Time now = this->now();
  const double since_agent = (now - last_agent_block_time_).seconds();
  if (since_agent >= 0.0 && since_agent < agent_block_hold_sec_) {
    RCLCPP_DEBUG(get_logger(), "[hold] agent-block hold active (%.2fs < %.2fs)",
                 since_agent, agent_block_hold_sec_);
    return;
  }
  // 너무 오래 버티면(막혀있음): 홀드 무시하고 정상 로직 진행
  // if (since_agent > agent_block_max_wait_sec_) { // 옵션: 로그만
  //   RCLCPP_INFO(get_logger(), "[hold] exceeded max wait (%.2fs > %.2fs), replan allowed",
  //               since_agent, agent_block_max_wait_sec_);
  // }

  if (use_footprint_check_) {
    validateWithFootprint(gpath);
  } else {
    validateWithPoints(gpath);
  }
}

// ===================== 기존 포인트 검사 =====================

void PathValidatorNode::validateWithPoints(const std::vector<geometry_msgs::msg::PoseStamped> & gpath)
{
  geometry_msgs::msg::Pose cur_pose;
  if (!getCurrentPoseFromTF(cur_pose)) return;

  const double max_dist_by_time = std::max(min_lookahead_m_, max_speed_ * lookahead_time_sec_);
  const double max_check_dist = std::min(max_dist_by_time, path_check_distance_m_);

  size_t best_streak = 0;
  size_t streak = 0;
  rclcpp::Time now = this->now();

  // DB 스냅샷
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
      if (agent_hit && publish_agent_collision_) {
        double wx, wy;
        {
          std::lock_guard<std::mutex> lock(costmap_mutex_);
          costmap_->mapToWorld(mx, my, wx, wy);
        }
        auto hits = whoCoversPoint(wx, wy);
        if (!hits.empty()) {
          publishAgentCollisionList(hits);
          // ★ NEW: 홀드 시작 & 같은 사이클 종료
          last_agent_block_time_ = this->now();
          return;
        }
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

// ===================== Footprint 기반 검사 =====================

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

  // 경로 샘플링
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
    bool blocked_here = false;
    unsigned int hit_mx = 0, hit_my = 0;

    if (use_radius_) {
      unsigned int cx, cy;
      if (!costmap->worldToMap(ps.pose.position.x, ps.pose.position.y, cx, cy)) {
        consecutive = 0;
        continue;
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

          if (masterCellBlocked(umx, umy, master_thr)) {
            blocked_here = true;
            hit_mx = umx; hit_my = umy;
          }
        }
      }
    } else {
      // 다각형 footprint → 월드 폴리곤
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

      for (int j = min_j; j <= max_j && !blocked_here; ++j) {
        for (int i = min_i; i <= max_i && !blocked_here; ++i) {
          double wx, wy; costmap->mapToWorld(i, j, wx, wy);
          if (!pointInPolygon(poly_world, wx, wy)) continue;

          const unsigned int umx = static_cast<unsigned int>(i);
          const unsigned int umy = static_cast<unsigned int>(j);

          if (masterCellBlocked(umx, umy, master_thr)) {
            blocked_here = true;
            hit_mx = umx; hit_my = umy;
          }
        }
      }
    }

    if (blocked_here) {
      if (compare_agent_mask_) {
        const bool agent_hit = agentCellBlockedNear(hit_mx, hit_my, agent_thr,
                                                    agent_mask_manhattan_buffer_);
        if (agent_hit && publish_agent_collision_) {
          double wx, wy; {
            std::lock_guard<std::mutex> lock(costmap_mutex_);
            costmap->mapToWorld(hit_mx, hit_my, wx, wy);
          }
          auto hits = whoCoversPoint(wx, wy);
          if (!hits.empty()) {
            publishAgentCollisionList(hits);
            // ★ NEW: 홀드 시작 & 같은 사이클 종료
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

// ===================== Agent 충돌 식별/퍼블리시 =====================

double PathValidatorNode::headingTo(const geometry_msgs::msg::Pose & pose, double wx, double wy)
{
  const double yaw = tf2::getYaw(pose.orientation);
  const double dx = wx - pose.position.x;
  const double dy = wy - pose.position.y;
  const double tgt = std::atan2(dy, dx);
  double d = tgt - yaw;
  while (d > M_PI) d -= 2*M_PI;
  while (d < -M_PI) d += 2*M_PI;
  return d;
}

double PathValidatorNode::speedAlong(const geometry_msgs::msg::Twist & tw, double heading_rad)
{
  const double v = tw.linear.x;
  return v * std::cos(heading_rad);
}

std::vector<PathValidatorNode::AgentHit> PathValidatorNode::whoCoversPoint(double wx, double wy) const
{
  std::vector<AgentHit> out;

  std::lock_guard<std::mutex> lk(agents_mutex_);
  if (!last_agents_) return out;

  if ((this->now() - last_agents_stamp_).nanoseconds() >
       static_cast<int64_t>(agents_freshness_timeout_ms_) * 1000000LL) {
    return out;
  }

  if (!last_agents_->header.frame_id.empty() &&
      last_agents_->header.frame_id != global_frame_) {
    return out;
  }

  for (const auto & a : last_agents_->agents) {
    const auto & fp = a.footprint.polygon.points;
    if (fp.size() < 3) continue;

    // footprint 소확장
    std::vector<geometry_msgs::msg::Point> poly_local; poly_local.reserve(fp.size());
    double cx=0, cy=0;
    for (auto & p : fp) { cx += p.x; cy += p.y; }
    cx /= static_cast<double>(fp.size());
    cy /= static_cast<double>(fp.size());
    for (auto & p : fp) {
      double vx = p.x - cx, vy = p.y - cy;
      double n = std::hypot(vx, vy); if (n < 1e-6) n = 1.0;
      geometry_msgs::msg::Point q;
      q.x = p.x + agent_match_dilate_m_ * (vx / n);
      q.y = p.y + agent_match_dilate_m_ * (vy / n);
      q.z = 0.0;
      poly_local.push_back(q);
    }

    const double yaw = tf2::getYaw(a.current_pose.pose.orientation);
    const double c = std::cos(yaw), s = std::sin(yaw);
    std::vector<geometry_msgs::msg::Point> poly_world; poly_world.reserve(poly_local.size());
    for (auto & p : poly_local) {
      geometry_msgs::msg::Point q;
      q.x = a.current_pose.pose.position.x + c * p.x - s * p.y;
      q.y = a.current_pose.pose.position.y + s * p.x + c * p.y;
      q.z = 0.0;
      poly_world.push_back(q);
    }

    if (pointInPolygon(poly_world, wx, wy)) {
      AgentHit hit;
      hit.machine_id = a.machine_id;
      hit.type_id = a.type_id;
      hit.x = wx; hit.y = wy;

      const double head = headingTo(a.current_pose.pose, wx, wy);
      const double v_along = speedAlong(a.current_twist, head);
      if (v_along > 0.05) {
        const double dx = wx - a.current_pose.pose.position.x;
        const double dy = wy - a.current_pose.pose.position.y;
        const double dist = std::hypot(dx, dy);
        hit.ttc_first = static_cast<float>(dist / v_along);
        hit.note = "agent footprint overlap; TTC estimated";
      } else {
        hit.ttc_first = -1.0f;
        hit.note = "agent footprint overlap; TTC unknown";
      }
      out.emplace_back(std::move(hit));
    }
  }

  return out;
}

void PathValidatorNode::publishAgentCollisionList(const std::vector<AgentHit> & hits)
{
  if (!publish_agent_collision_ || !agent_collision_pub_) return;
  if (hits.empty()) return;

  replan_monitor_msgs::msg::PathAgentCollisionInfo msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = global_frame_;

  for (const auto & h : hits) {
    msg.machine_id.push_back(h.machine_id);
    msg.type_id.push_back(h.type_id);
    msg.x.push_back(h.x);
    msg.y.push_back(h.y);
    msg.ttc_first.push_back(h.ttc_first);
    msg.note.push_back(h.note);
  }

  agent_collision_pub_->publish(msg);
}

// ===================== Replan pulse =====================

void PathValidatorNode::triggerReplan(const std::string & reason)
{
  const rclcpp::Time now = this->now();

  // ★ 홀드: 에이전트 알림 이후 일정 시간은 리플랜 방지
  const double since_agent = (now - last_agent_block_time_).seconds();
  if (since_agent >= 0.0 && since_agent < agent_block_hold_sec_) {
    RCLCPP_DEBUG(get_logger(),
      "Replan suppressed by agent-block hold (%.2fs < %.2fs)",
      since_agent, agent_block_hold_sec_);
    return;
  }
  // 너무 오래 막혔으면 리플랜 허용 (max_wait 넘어가면 hold 무시)
  // (명시적 체크는 생략해도 since_agent<hold 구간만 막으므로 충분)

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

### 사용 팁

* 우선 `agent_mask_manhattan_buffer`와 `agent_match_dilate_m`는 너무 크지 않게(예: 2~3, 0.1~0.2) 낮추고,
  이번 홀드 로직이 들어간 상태에서 감도를 다시 맞추세요.
* 홀드 시간은 `agent_block_hold_sec: 2.0`부터 시작, 상황에 맞춰 1.0~4.0s 사이에서 튜닝 권장.
* “정말로 오랫동안 막힌” 상황에서도 리플랜을 허용하려면 상위 로직에서 대응하지 못한 케이스일 수 있으니,
  `agent_block_max_wait_sec`를 6~12초 사이로 주고 운영해 보세요.
