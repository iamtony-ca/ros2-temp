

---

# `path_validator_node.hpp` (교체)

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
#include "multi_agent_msgs/msg/path_agent_collision_info.hpp"

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

  // wx, wy를 커버하는 agent의 footprint 또는 truncated_path 튜브를 찾아 리턴
  std::vector<AgentHit> whoCoversPoint(double wx, double wy) const;

  // 내부: 경로 튜브(footprint를 얇게 확장)에서 포함 여부 검사
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

  std::shared_ptr<nav2_costmap_2d::Costmap2D> agent_mask_;
  mutable std::mutex agent_mask_mutex_;
  CostmapSignature last_agent_sig_;

  // 최신 MultiAgentInfoArray
  multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr last_agents_;
  rclcpp::Time last_agents_stamp_;
  mutable std::mutex agents_mutex_;

  std::atomic<bool> is_robot_in_driving_state_{false};
  rclcpp::Time last_replan_time_;        // replan 쿨다운 기준
  rclcpp::Time last_agent_block_time_;   // 에이전트 홀드 기준

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

  // 에이전트 홀드 파라미터
  double agent_block_hold_sec_{2.0};
  double agent_block_max_wait_sec_{8.0};

  // === NEW: 에이전트 경로 튜브 매칭 파라미터 ===
  bool   agent_path_hit_enable_{true};
  double agent_path_hit_stride_m_{0.35};
  double agent_path_hit_dilate_m_{0.02};
  int    agent_path_hit_max_poses_{200};
};

}  // namespace replan_monitor

#endif  // PATH_VALIDATOR_NODE_HPP
```

---

# `path_validator_node.cpp` (교체)

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

  // 에이전트 홀드
  this->declare_parameter("agent_block_hold_sec", 2.0);
  this->declare_parameter("agent_block_max_wait_sec", 8.0);

  // === NEW: 에이전트 경로 튜브 매칭 ===
  this->declare_parameter("agent_path_hit_enable", true);
  this->declare_parameter("agent_path_hit_stride_m", 0.35);
  this->declare_parameter("agent_path_hit_dilate_m", 0.02);
  this->declare_parameter("agent_path_hit_max_poses", 200);

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

  // 홀드
  agent_block_hold_sec_     = this->get_parameter("agent_block_hold_sec").as_double();
  agent_block_max_wait_sec_ = this->get_parameter("agent_block_max_wait_sec").as_double();

  // 경로 튜브 매칭
  agent_path_hit_enable_      = this->get_parameter("agent_path_hit_enable").as_bool();
  agent_path_hit_stride_m_    = this->get_parameter("agent_path_hit_stride_m").as_double();
  agent_path_hit_dilate_m_    = this->get_parameter("agent_path_hit_dilate_m").as_double();
  agent_path_hit_max_poses_   = this->get_parameter("agent_path_hit_max_poses").as_int();

  // ===== TF =====
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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
    agent_collision_pub_ = this->create_publisher<multi_agent_msgs::msg::PathAgentCollisionInfo>(
        agent_collision_topic_, rclcpp::QoS(10).reliable());
  }

  // ===== Timers =====
  const int period_ms = static_cast<int>(1000.0 / std::max(1.0, db_update_frequency_));
  obstacle_db_update_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&PathValidatorNode::updateObstacleDatabase, this),
      timer_callback_group_);

  RCLCPP_INFO(this->get_logger(),
    "PathValidatorNode ready. agent_path_hit_enable=%s stride=%.2f dilate=%.2f maxposes=%d",
    (agent_path_hit_enable_ ? "true":"false"),
    agent_path_hit_stride_m_, agent_path_hit_dilate_m_, agent_path_hit_max_poses_);
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
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
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
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
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
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Could not get robot pose %s->%s: %s",
        global_frame_.c_str(), base_frame_.c_str(), ex.what());
    return false;
  }
}

bool PathValidatorNode::transformToGlobal(const geometry_msgs::msg::PoseStamped & in,
                                          geometry_msgs::msg::PoseStamped & out) const
{
  if (in.header.frame_id.empty() || in.header.frame_id == global_frame_) {
    out = in; out.header.frame_id = global_frame_; return true;
  }
  try {
    auto tf = tf_buffer_->lookupTransform(global_frame_, in.header.frame_id, in.header.stamp,
                                          rclcpp::Duration::from_seconds(0.2));
    tf2::doTransform(in, out, tf);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "transformToGlobal failed %s->%s: %s",
        in.header.frame_id.c_str(), global_frame_.c_str(), ex.what());
    return false;
  }
}

void PathValidatorNode::transformPathToGlobal(const nav_msgs::msg::Path & in,
                                              std::vector<geometry_msgs::msg::PoseStamped> & out) const
{
  out.clear(); out.reserve(in.poses.size());
  for (const auto & ps : in.poses) {
    geometry_msgs::msg::PoseStamped g;
    if (transformToGlobal(ps, g)) out.emplace_back(std::move(g));
  }
}

// ===================== Obstacle DB update =====================

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

  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (!costmap_) return;
  }

  std::vector<geometry_msgs::msg::PoseStamped> gpath;
  transformPathToGlobal(*msg, gpath);
  if (gpath.empty()) return;

  const rclcpp::Time now = this->now();
  const double since_agent = (now - last_agent_block_time_).seconds();
  if (since_agent >= 0.0 && since_agent < agent_block_hold_sec_) {
    RCLCPP_DEBUG(get_logger(), "[hold] agent-block hold active (%.2fs < %.2fs)",
                 since_agent, agent_block_hold_sec_);
    return;
  }

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
      if (!costmap_->worldToMap(gpath[i].pose.position.x, gpath[i].pose.position.y, mx, my)) {
        streak = 0; continue;
      }
    }

    const bool blocked_cell = isBlockedCellKernel(mx, my);

    // ★ 우선순위: 에이전트 히트 여부를 먼저 확인 (agent mask 유무와 무관)
    if (blocked_cell) {
      double wx, wy;
      {
        std::lock_guard<std::mutex> lock(costmap_mutex_);
        costmap_->mapToWorld(mx, my, wx, wy);
      }
      auto hits = whoCoversPoint(wx, wy);
      if (!hits.empty()) {
        publishAgentCollisionList(hits);
        last_agent_block_time_ = this->now();
        return; // 에이전트 충돌로 확정 → 일반 장애물 로직으로 가지 않음
      }
    }

    // (필요 시) agent mask 보조 판정
    bool blocked = blocked_cell;
    if (blocked && compare_agent_mask_) {
      const bool agent_mark = agentCellBlockedNear(mx, my,
                            static_cast<unsigned char>(agent_cost_threshold_),
                            agent_mask_manhattan_buffer_);
      if (agent_mark) {
        double wx, wy;
        { std::lock_guard<std::mutex> lock(costmap_mutex_); costmap_->mapToWorld(mx, my, wx, wy); }
        auto hits = whoCoversPoint(wx, wy);
        if (!hits.empty()) {
          publishAgentCollisionList(hits);
          last_agent_block_time_ = this->now();
          return;
        }
      }
    }

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

          if (masterCellBlocked(umx, umy, master_thr)) {
            blocked_here = true; hit_mx = umx; hit_my = umy;
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
            blocked_here = true; hit_mx = umx; hit_my = umy;
          }
        }
      }
    }

    if (blocked_here) {
      // ★ 우선순위: 에이전트 히트 먼저
      double wx, wy; { std::lock_guard<std::mutex> lock(costmap_mutex_); costmap->mapToWorld(hit_mx, hit_my, wx, wy); }
      auto hits = whoCoversPoint(wx, wy);
      if (!hits.empty()) {
        publishAgentCollisionList(hits);
        last_agent_block_time_ = this->now();
        return;
      }

      // (보조) agent mask가 있으면 한 번 더 확인
      if (compare_agent_mask_) {
        const bool agent_mark = agentCellBlockedNear(hit_mx, hit_my, agent_thr,
                                                     agent_mask_manhattan_buffer_);
        if (agent_mark) {
          auto hits2 = whoCoversPoint(wx, wy);
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

// ===================== Agent 충돌 식별 =====================

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

bool PathValidatorNode::pathTubeCoversPoint(const multi_agent_msgs::msg::MultiAgentInfo & a,
                                            double wx, double wy,
                                            double stride_m, double dilate_m,
                                            int max_poses, double /*frame_yaw*/,
                                            const std::string & /*global_frame*/)
{
  const auto & path = a.truncated_path;
  if (path.poses.empty()) return false;

  const int limit = std::min<int>(path.poses.size(), std::max(1, max_poses));
  // 거리 기반 스트라이드
  double acc = 0.0;
  auto prev = path.poses[0].pose.position;

  for (int i = 0; i < limit; ++i) {
    const auto & ps = path.poses[i].pose;

    if (i > 0) {
      const auto & cur = ps.position;
      acc += std::hypot(cur.x - prev.x, cur.y - prev.y);
      if (acc < std::max(0.05, stride_m)) continue;
      acc = 0.0;
      prev = cur;
    }

    // footprint를 얇게 등방성 확장한 로컬 폴리곤
    const auto & fp = a.footprint.polygon.points;
    if (fp.size() < 3) continue;

    // 로컬 dilate
    std::vector<geometry_msgs::msg::Point> poly_local; poly_local.reserve(fp.size());
    double cx=0, cy=0;
    for (auto & p : fp) { cx += p.x; cy += p.y; }
    cx /= static_cast<double>(fp.size());
    cy /= static_cast<double>(fp.size());
    for (auto & p : fp) {
      double vx = p.x - cx, vy = p.y - cy;
      double n = std::hypot(vx, vy); if (n < 1e-6) n = 1.0;
      geometry_msgs::msg::Point q;
      q.x = p.x + dilate_m * (vx / n);
      q.y = p.y + dilate_m * (vy / n);
      q.z = 0.0;
      poly_local.push_back(q);
    }

    // 로컬 → 월드 변환(경로 pose 기준, 스미어 없음)
    const double yaw = tf2::getYaw(ps.orientation);
    const double c = std::cos(yaw), s = std::sin(yaw);
    std::vector<geometry_msgs::msg::Point> poly_world; poly_world.reserve(poly_local.size());
    for (auto & p : poly_local) {
      geometry_msgs::msg::Point q;
      q.x = ps.position.x + c * p.x - s * p.y;
      q.y = ps.position.y + s * p.x + c * p.y;
      q.z = 0.0;
      poly_world.push_back(q);
    }

    if (pointInPolygon(poly_world, wx, wy)) {
      return true;
    }
  }
  return false;
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

    // 1) 현재 위치 footprint(소확장) 커버?
    {
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
        continue; // footprint에 걸리면 굳이 path 튜브 검사 불필요
      }
    }

    // 2) (NEW) truncated_path 튜브 커버?
    if (agent_path_hit_enable_) {
      const bool covered = pathTubeCoversPoint(
          a, wx, wy,
          agent_path_hit_stride_m_,
          agent_path_hit_dilate_m_,
          agent_path_hit_max_poses_,
          /*frame_yaw=*/0.0, global_frame_);
      if (covered) {
        AgentHit hit;
        hit.machine_id = a.machine_id;
        hit.type_id = a.type_id;
        hit.x = wx; hit.y = wy;
        hit.ttc_first = -1.0f; // 경로 포즈에서 TTC는 애매 → -1로
        hit.note = "agent truncated_path overlap";
        out.emplace_back(std::move(hit));
      }
    }
  }

  return out;
}

// ===================== Replan pulse =====================

void PathValidatorNode::publishAgentCollisionList(const std::vector<AgentHit> & hits)
{
  if (!publish_agent_collision_ || !agent_collision_pub_) return;
  if (hits.empty()) return;

  multi_agent_msgs::msg::PathAgentCollisionInfo msg;
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

void PathValidatorNode::triggerReplan(const std::string & reason)
{
  const rclcpp::Time now = this->now();

  // 홀드: 에이전트 알림 이후 일정 시간은 리플랜 방지
  const double since_agent = (now - last_agent_block_time_).seconds();
  if (since_agent >= 0.0 && since_agent < agent_block_hold_sec_) {
    RCLCPP_DEBUG(get_logger(),
      "Replan suppressed by agent-block hold (%.2fs < %.2fs)",
      since_agent, agent_block_hold_sec_);
    return;
  }

  if ((now - last_replan_time_).seconds() <= cooldown_sec_) return;
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

## 변경 요약

* `whoCoversPoint()`

  * 기존: **에이전트 현재 footprint**만 커버 체크
  * 변경: **현재 footprint + truncated_path 튜브** 모두 체크

    * 튜브는 `agent_path_hit_stride_m` 간격으로 경로 포즈를 샘플링하고, 각 포즈에서 **footprint를 `agent_path_hit_dilate_m` 만큼 등방성으로 얇게 확장**해 포함 여부 확인
* `validateWithPoints()` / `validateWithFootprint()`

  * 셀 차단이 발견되면 **무조건 먼저 `whoCoversPoint()`로 agent-hit 판정** → 맞으면 즉시 `publishAgentCollisionList()` 후 **리턴**
  * (옵션) agent_mask가 있으면 보조 확인만 수행. 결과가 agent라면 동일하게 **즉시 리턴**
  * 이렇게 해서 **agent_layer가 truncated_path를 칠해도** 일반 장애물로 잘못 분류되지 않음
* 파라미터 추가

  * `agent_path_hit_enable` (true)
  * `agent_path_hit_stride_m` (0.35m)
  * `agent_path_hit_dilate_m` (0.02m)
  * `agent_path_hit_max_poses` (200)

---

## 튜닝 팁

* **경로 튜브가 너무 넓게 맞는다** → `agent_path_hit_dilate_m` 줄이기(예: 0.01) 또는 `agent_path_hit_stride_m` 키우기(예: 0.5)
* **경로 튜브 누락이 있다** → `agent_path_hit_stride_m` 줄이기(예: 0.2)나 `agent_path_hit_max_poses` 늘리기
* `agent_layer`에서 경로 코스트가 강해서 과차단되면, layer 쪽 `path_cost`를 약하게 조정(예: 140~160)하거나 `path_dilation_m` 줄이기

필요하면 `whoCoversPoint()`에 **TTC 근사(경로 기반)** 도 추가해줄 수 있어. 지금은 footprint TTC만 추정하고 경로 튜브는 `-1`로 뒀어(안전한 기본).
##################  
#####################  
#####################  
# PathValidatorNode 소프트웨어 문서 (기술 해설)

본 문서는 `replan_monitor::PathValidatorNode`의 **의도(intent)**, **데이터 흐름(data flow)**, **동작 로직(algorithms)**, **쓰레딩/락 모델(threading & locks)**, 및 **각 메서드별 상세 설명**을 제공합니다. 코드의 모든 함수는 ROS2(Node, Publisher/Subscriber/Timer), Nav2(Costmap2D, Footprint), TF2, 그리고 사용자 정의 메시지(`multi_agent_msgs`)를 사용합니다. 영어 기술 용어는 그대로 유지합니다.

---

## 1) 역할 개요 (Overview)

`PathValidatorNode`는 주행 중인 로봇의 **현재 계획 경로(nav_msgs/Path)** 상에 **충돌 위험**이 있는지 온라인으로 판정합니다. 충돌 요인은 크게 두 가지:

1. **일반 장애물(Master costmap)**: `/global_costmap/costmap_raw`에서 수신한 costmap 상 `cost_threshold` 이상인 셀.
2. **에이전트(Agent)**: 다른 로봇/사람 등 외부 동적 객체.

   * **footprint overlap**(현재 pose 기준 footprint의 dilatation)
   * **truncated_path tube overlap**(경로 포즈마다 얇게 확장한 footprint 튜브)

검출 결과가 **에이전트 유발**이면 상세 정보를 `/path_agent_collision_info`로 publish하고, 일반 장애물이 일정 횟수(consecutive) 이상 연속되면 `/replan_flag`를 토글해 **리플랜(replan)**을 유도합니다. 또한, 에이전트 충돌 알림 이후 일정 시간은 replan을 막는 **hold(Agent block hold)**를 둡니다.

---

## 2) ROS I/O 및 주요 파라미터

### 구독(Subscriptions)

* `"/global_costmap/costmap_raw"`: `nav2_msgs::msg::Costmap`
* `agent_mask_topic` (기본: `"/agent_layer/costmap_raw"`): `nav2_msgs::msg::Costmap`
  (에이전트 레이어 costmap, 보조 판정에 사용)
* `"/multi_agent_infos"`: `multi_agent_msgs::msg::MultiAgentInfoArray`
* `"/robot_status"`: `std_msgs::msg::String` (예: `"DRIVING"`, `"PLANNING"`)
* `"/plan_pruned"`: `nav_msgs::msg::Path` (검증 대상 경로)

### 퍼블리시(Publishers)

* `"/replan_flag"`: `std_msgs::msg::Bool` (리플랜 트리거)
* `"/path_agent_collision_info"`: `multi_agent_msgs::msg::PathAgentCollisionInfo` (에이전트 충돌 상세)

### 타이머(Timers)

* `obstacle_db_update_timer_`: 가시 영역(ROI) 내 **고비용 셀**을 데이터베이스에 누적 & pruning

### 주요 파라미터(발췌)

* `cost_threshold`: 일반 장애물로 간주할 cost 기준(기본 254)
* `agent_cost_threshold`: 에이전트 mask에서 “agent mark”로 인정할 cost 기준
* `agent_mask_manhattan_buffer`: agent mask 근방 맨해튼 거리 버퍼(보조 판정)
* `agent_match_dilate_m`: footprint overlap 판정 시 footprint를 등방성으로 확장
* `agent_path_hit_*`: truncated_path 튜브 관련 stride/dilate/max poses
* `consecutive_threshold`: 일반 장애물 연속 임계치(도달 시 replan)
* `obstacle_persistence_sec`: **장애물 성숙도(persistence)** 요구 시간(빨리 사라지는 노이즈 억제)
* `agent_block_hold_sec`: 에이전트 충돌 알림 후 replan 억제(hold) 시간

---

## 3) 쓰레딩 & 락 모델 (Threading & Locking)

* **costmap** 접근: `costmap_mutex_`
* **agent_mask** 접근: `agent_mask_mutex_`
* **agents (MultiAgentInfoArray)** 접근: `agents_mutex_`
* **obstacle_db** 접근: `obstacle_db_mutex_`

콜백은 ROS의 multi-threaded executor에서 병행 호출 가능하므로, 모든 공유 상태는 mutex로 보호됩니다. **락 순서 교착(deadlock) 방지**를 위해, 일부 함수(`agentCellBlockedNear`)는 **로컬 shared_ptr 복사 후** 락 해제 상태로 좌표 변환을 수행합니다.

---

## 4) 데이터 구조

* `ObstacleInfo{ first_seen, last_seen }`: ROI 스캔 시 보이는 고비용 셀의 “성숙도 판단”에 사용
* `CostmapSignature`: costmap 메타데이터 서명 (size, resolution, origin) 변화 감지
* `AgentHit{ machine_id, type_id, x, y, ttc_first, note }`: 에이전트 충돌 보고용

---

## 5) 동작 흐름 (High-level Flow)

1. **Input 수신**

   * 최신 master costmap / agent mask / agents / robot status / path 수신 및 저장.
2. **주기적 DB 업데이트**

   * `updateObstacleDatabase()`가 ROI 내 고비용 셀을 스캔하여 `obstacle_db_`에 누적(성숙도 관리).
3. **경로 검증 요청**

   * `validatePathCallback()`에서 path를 global로 변환 후 **footprint 기반** 또는 **points 기반** 검사로 분기.
4. **충돌 판정 우선순위**

   * 셀/footprint가 막힘이면 먼저 **whoCoversPoint**로 **에이전트 유발 여부**를 검사(+ 이웃 8방향 보강).
   * 에이전트로 확정되면 충돌 리스트 publish & hold 시간 시작.
   * 아니라면 일반 장애물로 카운트(성숙도 조건 충족 시).
   * **연속 임계치** 도달 시 replan publish(단, hold 중이면 suppress).
5. **출력**

   * `/path_agent_collision_info` 또는 `/replan_flag` 발행.

---

## 6) 메서드 상세 (by file order)

### 생성자 `PathValidatorNode::PathValidatorNode()`

* **역할**: 파라미터 declare & load, 콜백/타이머/퍼블리셔 초기화, TF buffer 준비.
* **입력/출력**: 없음 (Node 초기화)
* **부작용**: ROS 엔티티 생성, 로그 출력.
* **주의**: 파라미터 `footprint`가 유효하지 않으면 `use_radius_ = true`로 fallback.

---

### `void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)`

* **역할**: master costmap 최신화. 메타가 바뀌면 새 그리드 할당, `obstacle_db_` 초기화.
* **입력**: Nav2 Costmap 메시지
* **출력/상태**: `costmap_` 갱신, `last_costmap_sig_` 갱신
* **락**: `costmap_mutex_` 사용
* **주의**: 데이터 길이 검증, mismatch 시 warn 후 return.

---

### `void agentMaskCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)`

* **역할**: 에이전트 레이어 costmap(agent mask) 최신화.
* **입력**: Nav2 Costmap 메시지
* **출력/상태**: `agent_mask_`, `last_agent_sig_` 갱신
* **락**: `agent_mask_mutex_`

---

### `void agentsCallback(const multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr msg)`

* **역할**: 최신 MultiAgent 정보 보관.
* **입력**: `MultiAgentInfoArray`
* **출력/상태**: `last_agents_`, `last_agents_stamp_`
* **락**: `agents_mutex_`

---

### `void robotStatusCallback(const std_msgs::msg::String::SharedPtr msg)`

* **역할**: 로봇 상태 문자열 기반 주행/계획 여부 플래그 설정.
* **입력**: `"DRIVING"` 또는 `"PLANNING"` 등
* **출력/상태**: `is_robot_in_driving_state_` (atomic)

---

### `void validatePathCallback(const nav_msgs::msg::Path::SharedPtr msg)`

* **역할**: 경로 유효성 검사 dispatcher.
* **입력**: 검증 대상 path
* **동작**:

  1. 주행 상태가 아니면 return.
  2. costmap 존재 확인.
  3. path를 **global frame**으로 변환.
  4. **agent block hold** 시간 내면 suppress.
  5. `use_footprint_check_`에 따라 `validateWithFootprint()` 또는 `validateWithPoints()` 호출.
* **출력**: 내부적으로 agent collision publish 또는 replan publish가 발생할 수 있음.

---

### `void updateObstacleDatabase()`

* **역할**: ROI 내 `cost_threshold` 이상의 master costmap 셀을 **가시 영역 & 시야 각도** 제한으로 스캔하여 `obstacle_db_` 갱신. 오래된 항목은 prune.
* **입력**: 없음(내부에서 TF로 현재 pose 획득)
* **출력/상태**: `obstacle_db_` (first_seen / last_seen 관리)
* **락**: `costmap_mutex_`, `obstacle_db_mutex_`
* **알고리즘**:

  * `lookahead = max(min_lookahead_m_, max_speed_ * lookahead_time_sec_)`
  * ROI = 현재 위치 주변 정사각 + 전방 시야(cone) + stride 스캔
  * `visible` set에 고비용 셀 key 삽입 → DB에 병합
  * `obstacle_prune_timeout_sec_` 초 이상 안 보이면 삭제

---

### `bool getCurrentPoseFromTF(geometry_msgs::msg::Pose & pose_out) const`

* **역할**: `base_frame_` → `global_frame_` TF 조회로 현재 pose.
* **성공/실패**: TF 예외시 warn throttle, false 반환.

---

### `bool transformToGlobal(const geometry_msgs::msg::PoseStamped & in, geometry_msgs::msg::PoseStamped & out) const`

* **역할**: 입력 pose를 global frame으로 변환. 이미 global이거나 빈 frame이면 그대로.
* **실패 시**: TF 예외 로그 및 false.

---

### `void transformPathToGlobal(const nav_msgs::msg::Path & in, std::vector<geometry_msgs::msg::PoseStamped> & out) const`

* **역할**: path 내 모든 pose를 global frame으로 변환하여 벡터에 저장.
* **주의**: 변환 실패 pose는 skip.

---

### `inline uint64_t packKey(unsigned int mx, unsigned int my) const`

* **역할**: 2D index를 64-bit key로 pack (DB map key).

---

### `bool isBlockedCellKernel(unsigned int mx, unsigned int my) const`

* **역할**: master costmap에서 `(mx,my)` 주변 **kernel(±K)** 윈도우 내 `cost_threshold` 이상 존재 여부.
* **락**: `costmap_mutex_`
* **용도**: points 기반 검사에서 근접 커널로 보수적으로 “막힘” 판정.

---

### `inline bool masterCellBlocked(unsigned int mx, unsigned int my, unsigned char thr) const`

* **역할**: master costmap 단일 셀에서 `thr` 이상인지 즉시 판정.
* **락**: `costmap_mutex_`

---

### `inline bool agentCellBlockedNear(unsigned int mx, unsigned int my, unsigned char thr, int manhattan_buf) const`

* **역할**: **좌표계를 정합**하여 master `(mx,my)`의 **월드 좌표를 agent_mask index로 변환** 후, 맨해튼 버퍼 내에서 `thr` 이상 존재 여부 확인(보조 판정).
* **중요 포인트**:

  * **교착 회피**: `costmap_`과 `agent_mask_`를 각각 잠깐씩 락 잡아 **로컬 shared_ptr**로 복사 후 락 해제.
  * `master->mapToWorld()` → `agent->worldToMap()` 순서로 index 변환.
* **반환**: agent mask가 agent로 마킹했는지 **근방에서** 확인.

---

### `void validateWithPoints(const std::vector<geometry_msgs::msg::PoseStamped> & gpath)`

* **역할**: path를 **points**(pose 위치) 단위로 샘플링 없이 그대로 순회, 각 점이 **막힘**인지 판정.
* **알고리즘**:

  1. **거리 제한**: `max_check_dist = min(max_speed * lookahead_time, path_check_distance_m_)` 이내만 검사.
  2. 각 path point를 map index로 변환 실패 시 streak reset.
  3. `isBlockedCellKernel()`로 근방 커널 검사.
  4. **막힘이면**:

     * 중심 셀 및 **8-이웃 셀** 월드좌표로 **whoCoversPoint()** 호출 → **에이전트 히트 우선** 판정.
     * 필요 시 `agentCellBlockedNear()`로 agent mask 보조 확인 후 재검사.
     * **에이전트면**: `publishAgentCollisionList()` & `last_agent_block_time_` 갱신 → return.
     * **아니면**: 일반 장애물로 간주. 단, `updateObstacleDatabase()`에서 누적된 `obstacle_db_`의 `first_seen` 기준으로 **성숙도(obstacle_persistence_sec_)** 충족 시에만 streak 카운트.
  5. `consecutive_threshold_` 도달 시 `triggerReplan()` 호출.
* **출력**: 필요 시 agent collision publish 또는 replan publish.
* **주의**: agent 우선 판정으로 **오분류(일반 장애물로 잘못 카운트)**를 줄임.

---

### `static bool pointInPolygon(const std::vector<geometry_msgs::msg::Point> & poly, double x, double y)`

* **역할**: Ray-casting으로 점 포함 여부.
* **용도**: footprint/world polygon 내부 포함 판정.

---

### `void validateWithFootprint(const std::vector<geometry_msgs::msg::PoseStamped> & gpath)`

* **역할**: path를 **footprint**(원형 또는 다각형)로 샘플링하면서 **면적 충돌**을 판정.
* **알고리즘**:

  1. 경로를 `footprint_step_m_` 간격으로 샘플링(총 검사 거리 `max_check_dist` 제한).
  2. **원형(radius)**: 샘플 pose의 원형 footprint를 raster-space로 변환하여 원 디스크 내부 셀 중 `masterCellBlocked()` 여부 확인.
  3. **다각형(footprint_)**: pose의 yaw로 **월드 폴리곤** 변환 → bounding box 래스터 스캔 → `pointInPolygon()`로 내부인 셀만 `masterCellBlocked()` 검사.
  4. **막힘이면** points 방식과 동일하게 **에이전트 우선 판정**(중심+8-이웃 `whoCoversPoint()`), 필요 시 agent mask 보조, 아니면 일반 장애물 streak 카운트.
  5. `consecutive_threshold_` 도달 시 replan 트리거.
* **장점**: 차량 체적 반영으로 false negative 감소.
* **주의**: 폴리곤 방식은 계산량이 커 샘플링 간격과 kernel 크기 조율 권장.

---

### `static double headingTo(const geometry_msgs::msg::Pose & pose, double wx, double wy)`

* **역할**: pose yaw 대비 (pose→(wx,wy)) 방위각 차이(−π~π).
* **용도**: `speedAlong()`과 결합하여 상대 점으로의 진행 성분 추정.

---

### `static double speedAlong(const geometry_msgs::msg::Twist & tw, double heading_rad)`

* **역할**: 선속도 `tw.linear.x`와 heading 각도의 cos를 곱해 **목표점 방향 성분 속도** 산출.
* **용도**: 에이전트의 **TTC(first)** 추정(단순 선형 모델).

---

### `static bool pathTubeCoversPoint(const multi_agent_msgs::msg::MultiAgentInfo & a, double wx, double wy, double stride_m, double dilate_m, int max_poses, double frame_yaw, const std::string & global_frame)`

* **역할**: 에이전트 `truncated_path`의 각 pose에서 **얇게 확장된 footprint**(isotropic dilation `dilate_m`)를 월드로 투영하고 **stride_m** 간격으로 샘플링하여 `(wx, wy)` 포함 여부 확인.
* **반환**: 포함 시 true (즉, **경로 튜브와 겹침**)
* **주의**: TTC는 추정하지 않음(정적 오버랩만).

---

### `std::vector<AgentHit> whoCoversPoint(double wx, double wy) const`

* **역할**: `(wx, wy)`를 **커버하는 에이전트**를 탐색하여 리스트로 반환.
* **순서**:

  1. 최신 `last_agents_` freshness 검사 및 frame 일치 확인.
  2. 각 agent에 대해:

     * **현재 footprint(소확장: `agent_match_dilate_m_`)** 월드 변환 → `pointInPolygon()` → 포함 시 `AgentHit` 구성.

       * 진행 성분 속도(`speedAlong`)가 유의미하면 `TTC = dist / v_along`(>0.05 m/s), 아니면 -1.
     * (미포함 & `agent_path_hit_enable_`)이면 **경로 튜브 검사**(`pathTubeCoversPoint`) → 포함 시 `AgentHit`(TTC -1)
* **반환**: 0개 이상 `AgentHit`(중복 가능).
* **주의**: 결과를 사용해 **에이전트 충돌 우선 판정**을 수행.

---

### `void publishAgentCollisionList(const std::vector<AgentHit> & hits)`

* **역할**: `PathAgentCollisionInfo`에 충돌 목록을 채워 publish.
* **입력**: AgentHit 벡터 (비어있으면 return)
* **출력**: 토픽 publish (frame은 `global_frame_`)
* **주의**: `publish_agent_collision_`이 false면 무시.

---

### `void triggerReplan(const std::string & reason)`

* **역할**: replan을 트리거하는 `/replan_flag` publish.
* **로직**:

  * **에이전트 hold** 시간 내면 **suppress**.
  * **cooldown_sec_** 내 중복 트리거 방지.
  * `publish_false_pulse_`이면 짧은 타이머로 false를 재발행(펄스 형태).
* **출력**: 로그 및 Bool publish.

---

## 7) 검증 로직의 핵심 포인트 (Design Intent)

* **에이전트-우선 판정**: 막힘 감지 시 **항상 먼저 에이전트 히트**를 검사(중심+8-이웃).
  → 실제로 agent 유발 충돌인데 일반 장애물로 오분류되는 문제 감소.
* **좌표계 정합**: `agentCellBlockedNear()`에서 **master↔agent_mask index 변환**을 월드좌표 경유로 수행.
  → 서로 origin/resolution이 달라도 올바른 근방 탐색 보장.
* **성숙도(obstacle_persistence)**: 일반 장애물은 **일정 시간 지속될 때에만** 유효 카운트.
  → 센서 노이즈/스파이크 억제.
* **hold & cooldown**: replan 난발 방지, agent 알림 이후 잠깐 replan 억제.

---

## 8) 파라미터 튜닝 팁 (Tuning Notes)

* `cost_threshold`(254): LETHAL(254)만 막힘으로. 민감도 높이려면 200~254 조정.
* `kernel_half_size`(2): points 검사의 보수성. 늘릴수록 false positive↑.
* `consecutive_threshold`(2~3): 리플랜 트리거 민감도.
* `obstacle_persistence_sec`(≥1.0): 낮추면 반응 빠르나 노이즈↑.
* `agent_match_dilate_m`(0.05~0.15): footprint overlap 관대함. 커지면 agent hit↑.
* `agent_path_hit_dilate_m`(0.03~0.06): 튜브 두께. 과도하면 과차단.
* `agent_mask_manhattan_buffer`(2~5): agent_mask 근방 허용 거리. 과대 설정은 오검↑.
* `agent_block_hold_sec`(2.0~): 에이전트 감지 후 리플랜 억제 길이.

---

## 9) 에지 케이스 & 주의사항

* **프레임 불일치**: `whoCoversPoint()`는 `last_agents_->header.frame_id`가 `global_frame_`과 다르면 early return. (agent source에서 frame 일치 보장 필요)
* **경계 인덱스**: 이웃(±1) 검사 시 map 경계 밖 접근을 피하기 위한 index bound 체크가 권장됩니다.
* **비정상 path**: path가 비어있거나 모두 동일점이면 샘플링/검사가 skip될 수 있음.
* **TF 실패**: TF timeout 시 검사 skip → 주행 상태라도 결과가 없을 수 있음.

---

## 10) 통합(Integration) 체크리스트

* 글로벌 프레임(`global_frame_`)과 costmap/agent_mask/agents 메시지의 frame 일치.
* `agent_layer`의 raster가 실제 agent footprint/경로를 반영하도록 설정 (`dilation`, `forward smear`는 layer 쪽 정책).
* `/robot_status` 발행이 주행 상태를 정확히 반영.
* replan 소비 측(Planner/BT)이 `/replan_flag` Bool 펄스를 적절히 처리.

---

이 문서는 구현 코드의 현재 로직을 바탕으로 작성되었습니다. 특정 운영 환경(센서 update rate, costmap resolution, agent density, corridor width)에 따라 파라미터 최적점은 달라질 수 있으며, 위 **튜닝 팁**을 기준으로 점진 조정하시길 권장합니다.
####################  
####################  
#####################  
좋아, 아래 5개 함수의 **의도 → 입력/출력 → 내부 단계 → 파라미터 영향 → 복잡도/주의점** 순서로, 실제 코드 흐름을 그대로 풀어서 설명할게. (기술 용어는 그대로 둠)

---

# isBlockedCellKernel(mx, my)

**역할**
단일 셀 `(mx, my)` **주변 커널(±K)** 안에 `cost_threshold_` 이상(=장애물로 간주)인 master costmap 셀이 하나라도 있으면 **true**를 반환. 즉, **보수적인 근방 차단 판정**.

**입력 / 출력**

* 입력: `mx, my` (master costmap index)
* 출력: bool (커널 내 차단 존재 여부)

**내부 로직 (코드 그대로 단계화)**

1. `costmap_mutex_` 락 → `costmap_` 존재 확인. 없으면 false.
2. `K = kernel_half_size_`.
3. `x = mx + dx`, `y = my + dy`를 `dx, dy ∈ [-K, K]`로 이중 루프.
4. 맵 경계 밖이면 continue.
5. `c = costmap_->getCost(x,y)` 취득.

   * `ignore_unknown_ == true`이고 `c == NO_INFORMATION`이면 continue.
   * `c >= cost_threshold_`이면 **즉시 true**.
6. 루프가 끝나면 false.

**파라미터 영향**

* `kernel_half_size_`: 커널 반경. 클수록 보수적(=false negative↓, false positive↑).
* `cost_threshold_`: 차단 민감도. 낮출수록 차단 판정↑.
* `ignore_unknown_`: Unknown 셀 무시 여부.

**복잡도 / 주의점**

* 시간복잡도: `O((2K+1)^2)` per query.
* 경계 체크 필수(코드에 있음).
* **용도**: points 검증에서 “점 하나”가 아닌 “근방 패치”로 판단하여 미세한 스파이크/오차를 흡수.

---

# masterCellBlocked(mx, my, thr)

**역할**
master costmap의 **단일 셀**이 주어진 임계치 `thr` 이상인지 **즉시 판정**.

**입력 / 출력**

* 입력: `mx, my`, `thr` (보통 `static_cast<unsigned char>(cost_threshold_)`)
* 출력: bool

**내부 로직**

1. `costmap_mutex_` 락 → 존재 확인.
2. `c = costmap_->getCost(mx, my)`.
3. `ignore_unknown_`이고 `c == NO_INFORMATION`이면 false.
4. `c >= thr`면 true, 아니면 false.

**파라미터 영향**

* `thr`가 `cost_threshold_`에 해당.
* `ignore_unknown_`의 처리 동일.

**복잡도 / 주의점**

* `O(1)`
* **validateWithFootprint**에서 폴리곤 내부의 **각 래스터 셀을 빠르게 검사**할 때 핵심.

---

# pointInPolygon(poly, x, y)

**역할**
**Ray casting** 방식으로 `(x,y)` 점이 다각형 `poly` 내부에 있는지 체크. 내부면 true.

**입력 / 출력**

* 입력: `poly`(월드 좌표의 polygon vertex들), `x, y`
* 출력: bool

**내부 로직**

1. `inside=false`, `n = poly.size()`.
2. 인덱스 `(i, j)`를 `(0..n-1, n-1)`로 돌면서 각 edge `(xi, yi)→(xj, yj)`에 대해:

   * y-수직 교차 조건: `((yi > y) != (yj > y))`
   * 교차 x좌표 계산:
     `x_cross = (xj - xi)*(y - yi)/(yj - yi + ε) + xi`  (코드에선 `max(1e-12, yj-yi)`로 분모 보호)
   * `x < x_cross`면 `inside = !inside` 토글.
3. 루프 종료 후 `inside` 반환.

**파라미터 영향**

* 없음. 다만 입력 `poly`는 **월드 좌표계**여야 정확.

**복잡도 / 주의점**

* 시간복잡도: `O(n)` (n=vertex 수)
* **수평/수직 edge의 특이 케이스**를 `1e-12`로 나눗셈 보호.
* **validateWithFootprint**에서 **footprint polygon 내부 raster cell**인지 필터링하는 데 사용.

---

# agentCellBlockedNear(mx, my, thr, manhattan_buf)

**역할**
master costmap의 인덱스 `(mx, my)`에 대응하는 **월드좌표 (wx, wy)**를 **agent_mask**의 index `(ax, ay)`로 변환한 뒤, 그 **맨해튼 반경** `manhattan_buf` 안에서 `thr` 이상인 agent mask 셀이 있는지 확인.
→ **서로 다른 costmap(해상도/원점 다름) 사이 좌표 정합**을 올바르게 처리.

**입력 / 출력**

* 입력: master index `(mx, my)`, `thr`, `manhattan_buf`
* 출력: bool (근방 agent mark 존재 여부)

**내부 로직**

1. **교착 회피 패턴**:

   * `costmap_mutex_`로 `costmap_`을 로컬 복사(`master`)
   * `agent_mask_mutex_`로 `agent_mask_` 로컬 복사(`agent`)
     (락 해제 후 진행)
2. `master->mapToWorld(mx, my, wx, wy)`로 **master index → world**.
3. `agent->worldToMap(wx, wy, ax, ay)`로 **world → agent_mask index**. 실패 시 false.
4. `ix = ax, iy = ay` 기준으로 `|dx|+|dy| ≤ manhattan_buf`인 **맨해튼 이웃**을 모두 검사:

   * 경계 밖 skip
   * `a = agent->getCost(x, y)`
     `a >= thr` 발견 즉시 true.
5. 끝까지 없으면 false.

**파라미터 영향**

* `manhattan_buf`: 근방 허용 크기. 클수록 agent 간주 확률↑ (과대 시 오검↑).
* `thr`: `agent_cost_threshold_` 적용이 일반적.

**복잡도 / 주의점**

* 시간복잡도: `O(manhattan_buf^2)` 정도(정확히는 다이아몬드 영역의 셀 수)
* **좌표계 정합이 핵심**: master↔agent mask가 다른 해상도/원점이어도 **월드 좌표를 경유**하므로 정확.
* 이 함수는 **보조 증거**로 쓰이고, 실제 에이전트 판정은 `whoCoversPoint()`가 최우선.

---

# validateWithFootprint(gpath)

**역할**
경로를 **footprint(원형 radius 또는 다각형)** 기준으로 따라가며 **면적 충돌**을 판정. **에이전트 히트가 우선**이며, 그렇지 않은 경우 **일반 장애물**로 누적하여 replan 트리거.

**입력 / 출력**

* 입력: `gpath` (global frame으로 변환된 Path의 PoseStamped 벡터)
* 출력: 없음(내부에서 필요 시 publish/return)

**핵심 아이디어**

* 경로를 `footprint_step_m_` 간격으로 **샘플링**.
* 각 샘플 pose에 대해 **footprint가 커버하는 래스터 셀** 중 `masterCellBlocked()`가 하나라도 있으면 “막힘”.
* 막힘이면 **에이전트 먼저 확인**(중심+8-이웃 `whoCoversPoint()`), 필요 시 `agentCellBlockedNear()` 보강.
* 에이전트면 충돌 정보 publish 후 종료, 아니면 일반 장애물 연속 카운트.
* 연속 카운트가 `consecutive_threshold_` 도달 시 replan 트리거.

**내부 로직 (코드 단계화)**

1. `costmap_` 로컬 복사 및 존재 확인, 현재 pose(TF) 확보.
2. 검사 거리 한계 계산:
   `max_check_dist = min( max(min_lookahead_m_, max_speed_*lookahead_time_sec_), path_check_distance_m_ )`.
3. **샘플링**: 누적 거리 `acc`로 `footprint_step_m_` 간격이 될 때마다 `samples`에 push. 첫 pose 포함.
4. `master_thr = cost_threshold_`, `agent_thr = agent_cost_threshold_`.
5. `consecutive=0`.
6. `samples` 순회:

   * **원형 사용(use_radius_)**이면:

     * `cx,cy = worldToMap(ps.position)` 실패 시 `consecutive=0; continue`
     * `r_cells = ceil(robot_radius_m_ / resolution)`
     * 디스크 내부 `(dx,dy)`에 대해 index `mx=cx+dx, my=cy+dy`:

       * 경계 체크
       * `masterCellBlocked(umx, umy, master_thr)`이면 `blocked_here=true; hit_mx,hit_my` 저장
   * **다각형 footprint**이면:

     * pose yaw로 footprint를 **월드 좌표로 회전/평행이동** → `poly_world`
     * `poly_world`의 bounding box를 `worldToMapEnforceBounds`로 래스터 범위 `(min_i..max_i, min_j..max_j)` 결정
     * 그 범위 셀들을 순회:

       * `mapToWorld(i, j, wx, wy)`
       * `pointInPolygon(poly_world, wx, wy)`가 **내부**인 셀만 `masterCellBlocked()` 검사
       * 차단 셀 발견 시 `blocked_here=true; hit_mx,hit_my` 저장
   * **막힘이면**(blocked_here):

     1. `cm = costmap_` 로컬 복사, `mapToWorld(hit_mx,hit_my)`로 `(wx,wy)`
     2. **에이전트 우선 검사**:

        * 람다 `agent_hit_around(cx,cy,mx,my)`로 **중심 + 8-이웃** 각각에 대해 `whoCoversPoint()` 호출
        * 결과가 비어있지 않으면 `publishAgentCollisionList()` 하고 `last_agent_block_time_` 갱신 후 **return**
     3. **보조(agent mask)**:

        * `compare_agent_mask_`가 true이고 `agentCellBlockedNear(hit_mx, hit_my, agent_thr, agent_mask_manhattan_buffer_)`가 true면, 다시 `agent_hit_around()` 시도 → hit이면 publish+return
     4. **일반 장애물 누적**: `consecutive++`; `consecutive >= consecutive_threshold_`면 `triggerReplan()` 하고 return
   * **안 막히면**: `consecutive = 0` 리셋
7. 루프 종료 시 추가 동작 없음.

**파라미터 영향**

* `use_radius_`, `robot_radius_m_`: 원형/다각형 경로.
* `footprint_step_m_`: 샘플 간격(작을수록 촘촘).
* `cost_threshold_`, `agent_cost_threshold_`, `agent_mask_manhattan_buffer_`.
* `consecutive_threshold_`, `path_check_distance_m_`, `min_lookahead_m_`, `lookahead_time_sec_`, `max_speed_`.

**복잡도 / 주의점**

* 원형: `O(#samples * r_cells^2)`
* 다각형: `O(#samples * bbox_cells)` (bbox 안에서 polygon test까지 수행)
* **중요**: 폴리곤은 **월드 좌표**에서 point-in-polygon → 라스터 접근 순서가 올바름.
* **에이전트 우선**: 일반 장애물로 잘못 카운트되는 문제를 줄이기 위한 핵심 설계.

---

## 빠른 요약 표

| 함수                    | 핵심 목적                                   | 좌표계                        | 락                        | 시간복잡도(개략)            | 주요 파라미터                                                           |
| --------------------- | --------------------------------------- | -------------------------- | ------------------------ | -------------------- | ----------------------------------------------------------------- |
| isBlockedCellKernel   | 커널(±K) 내 고비용 셀 존재                       | map index                  | costmap                  | O(K²)                | kernel_half_size_, cost_threshold_, ignore_unknown_               |
| masterCellBlocked     | 단일 셀 고비용 여부                             | map index                  | costmap                  | O(1)                 | thr(=cost_threshold_)                                             |
| pointInPolygon        | 점의 폴리곤 내부 여부                            | world                      | 없음                       | O(n)                 | (없음)                                                              |
| agentCellBlockedNear  | master셀 대응 agent_mask 근방의 agent mark 여부 | master idx→world→agent idx | costmap+agent_mask(로컬복사) | O(buf²)              | agent_cost_threshold_, agent_mask_manhattan_buffer                |
| validateWithFootprint | footprint 면적 충돌 검사 (에이전트 우선)            | world↔map 혼합               | 다수                       | 샘플 수·footprint 크기 의존 | footprint_step_m_, cost/agent thresholds, consecutive_threshold 등 |

필요하면 위 설명을 바탕으로 **튜닝 가이드**나 **디버깅 체크리스트**도 정리해줄게.
###########################  
############################  
###########################  
좋아. 아래에 **whoCoversPoint()**를 “코드가 실제로 하는 일” 중심으로 단계별로 풀고, 이어서 **에이전트 판정 전체 플로우**(validateWithFootprint/validateWithPoints 안에서의 의사결정 우선순위 포함)를 코드 기반으로 정리해줄게.

---

# whoCoversPoint(wx, wy): “이 월드 좌표를 커버하는 에이전트 찾기”

**목적**
주어진 월드 좌표 `(wx, wy)`가 **어떤 에이전트의 현재 footprint** 또는 **에이전트 truncated_path 기반 튜브**에 포함되는지 판정하고, 히트한 에이전트들의 메타데이터(기계 ID, 타입, 메모 등)를 `AgentHit` 목록으로 반환.

## 입력/출력

* 입력: `wx, wy` (월드 좌표)
* 출력: `std::vector<AgentHit>`
  (0개면 “해당 점을 커버하는 에이전트 없음”)

## 내부 로직: 코드 흐름 그대로

1. **동기화/신선도 체크**

   * `agents_mutex_` 락을 잡고 `last_agents_` 스냅샷을 읽음. 없으면 빈 벡터 반환.
   * `agents_freshness_timeout_ms_`보다 오래된 메시지면(=신선도 만료) 빈 벡터 반환.
   * `last_agents_->header.frame_id`가 설정되어 있고 `global_frame_`과 **불일치**하면(프레임 불일치) 빈 벡터 반환.

     > 즉, **좌표계 mismatch는 즉시 중단**.

2. **모든 에이전트 순회 (`for (const auto& a : last_agents_->agents)`)**

   * `a.footprint.polygon.points`가 3점 미만이면 스킵(폴리곤 성립X).

3. **(1차) “현재 위치 footprint”로 커버 여부**

   * 에이전트 footprint를 **소확장**(=agent_match_dilate_m_만큼 등방성 확장)

     * 무게중심 `(cx, cy)` 계산
     * 각 꼭짓점 `p`에서 중심까지 방향 벡터 `(vx, vy)` 단위화 → `p + agent_match_dilate_m_ * (vx, vy)`
   * 에이전트 **현재 pose**(`a.current_pose.pose`)의 yaw로 로컬 폴리곤을 **월드 좌표**로 변환
     `q = R(yaw) * p_local + t(current_pose)`
   * `pointInPolygon(poly_world, wx, wy)`가 **true**면:

     * `AgentHit hit` 구성:

       * `machine_id`, `type_id`, `x=wx`, `y=wy`
       * **TTC 추정**:

         * `headingTo(a.current_pose.pose, wx, wy)`로 대상점에 대한 상대 heading 구함
         * `speedAlong(a.current_twist, heading)` = `linear.x * cos(heading)`
         * `v_along > 0.05`면 `dist / v_along`으로 `ttc_first` 추정, 아니면 `-1(unknown)`
       * `note = "agent footprint overlap; TTC estimated/unknown"`
     * `out.emplace_back(hit)` 하고 **다음 에이전트로 continue**

       > 현재 footprint에 걸리면 **path 튜브 검사까지 갈 필요 없음** (최우선)

4. **(2차) “truncated_path 튜브”로 커버 여부 (옵션)**

   * `agent_path_hit_enable_`이 **true**면 `pathTubeCoversPoint(a, wx, wy, stride, dilate, maxposes, ...)` 호출:

     * `a.truncated_path`의 poses를 **거리 누적 기반 stride**(`agent_path_hit_stride_m_`)로 **희소 샘플링**
     * 각 샘플 pose에서 **footprint를 얇게 등방성 확장**(`agent_path_hit_dilate_m_`)
     * 그 폴리곤을 pose yaw로 **월드 좌표로 변환**
     * `pointInPolygon(poly_world, wx, wy)`가 **true**인 포즈 하나라도 있으면 **covered**
   * covered면 `AgentHit` 구성(footprint와 유사하되 `ttc_first = -1`, `note = "agent truncated_path overlap"`), `out.emplace_back(hit)`.

5. **반환**

   * 모든 에이전트를 검사한 후 `out` 반환.
   * 비어있으면 “에이전트가 점을 덮지 않음”.

## 설계 포인트 / 파라미터 영향

* **우선순위**: “현재 footprint” 히트가 **path 튜브**보다 **우선** (코드상 continue;로 바로 다음 에이전트로 넘어감).
* `agent_match_dilate_m_`: 현재 footprint를 **약간 부풀려서** 검출 민감도를 높임(오검 vs 미검 트레이드오프).
* `agent_path_hit_*`: 튜브 폭(dilate), 샘플 간격(stride), 최대 포즈 수(max_poses)로 **탐지 밀도/속도** 제어.
* 프레임/신선도 체크가 **초전**에 걸리므로, 메시지 stale/프레임 mismatch면 무조건 “미검”.

---

# “Agent 판정” 전체 플로우 (validate 계열에서의 의사결정 우선순위)

에이전트 판정은 **경로 검증 루프**(points 또는 footprint 기반) 중 “차단 징후”가 관측될 때마다 **가장 먼저** 시도됩니다. 두 validate 함수의 구조는 거의 동일하며, 차이는 **차단을 감지하는 방법**뿐입니다.

아래는 **validateWithFootprint**를 예로 든 전체 의사결정. (validateWithPoints도 `isBlockedCellKernel()`로 차단 감지하는 점만 다르고, 이후 로직은 동일)

## 0) 사전조건

* 로봇 상태가 `DRIVING` 또는 `PLANNING`인지 확인(아니면 return)
* replan 억제 홀드: 최근 `last_agent_block_time_` 이후 `agent_block_hold_sec_` 이내면 **검사 자체를 return** (불필요한 재플래그 방지)

## 1) 경로를 샘플링

* `footprint_step_m_` 간격으로 `samples`를 생성
* 검사 거리 상한 `max_check_dist` 내에서만 전진

## 2) 각 샘플 pose에서 **차단 감지**

* **원형 footprint 모드**: `r_cells = ceil(robot_radius_m_ / resolution)`인 **원형 디스크** 안의 셀들에 대해 `masterCellBlocked()`를 검사
* **다각형 footprint 모드**: footprint를 월드 좌표 폴리곤으로 만들고, **그 폴리곤 내부 래스터 셀**만 `masterCellBlocked()` 검사
* 하나라도 threshold 이상이면 `blocked_here = true` 및 `(hit_mx, hit_my)` 저장

## 3) 차단 발생 시: **Agent 우선 판정**

* `(hit_mx, hit_my)`를 월드 `(wx, wy)`로 변환
* **우선** `whoCoversPoint(wx, wy)`를 호출하기 전에, 구현에서는 **‘중심+8방향 1셀 이웃’**까지 확대해서 체크하는 람다 `agent_hit_around()`를 사용:

  * 중심 `(wx, wy)`에 대해 `whoCoversPoint()` → **hit면 즉시** `publishAgentCollisionList(hits); last_agent_block_time_=now; return;`
  * hit이 없으면 `(hit_mx, hit_my)`의 8-neighborhood 각각을 월드 좌표로 바꿔 `whoCoversPoint()` 반복
    → **어느 한 곳이라도 hit**면 동일하게 publish+return
  * 이 확장은 **rasterization/샘플링 경계 오차**를 줄이려는 장치

> 즉, **실제 에이전트 판정의 1순위는 언제나 whoCoversPoint**이며, 단일 점 오차를 줄이려고 **이웃까지 검사**해주는 형태.

## 4) (보조) agent mask 근거로 재시도

* `compare_agent_mask_ == true` 이고 `agentCellBlockedNear(hit_mx, hit_my, agent_cost_threshold_, agent_mask_manhattan_buffer_)`가 **true**면
  → 다시 `agent_hit_around()`로 `whoCoversPoint()` 재시도
  → hit면 publish+return
* **의미**: master costmap 상의 고코스트가 “agent layer”의 표식과 **정합**될 때, 위치 오차로 놓친 케이스를 **한 번 더 잡아내기** 위한 보조 근거

## 5) 에이전트가 아니면 → **일반 장애물 누적**

* 위 단계들에서 에이전트로 확정되지 않았다면, 이 차단은 **일반 장애물**로 간주
* `consecutive++`; 만약 `consecutive >= consecutive_threshold_` 이면 `triggerReplan("blocked (footprint) streak threshold reached")` 하고 return
* 막힘이 해소되면 `consecutive = 0`으로 리셋

## 6) 종료 조건

* 루프를 다 돌아도 replan 트리거 없이 끝나면 함수 종료(=경로상 안전 또는 아직 성숙치 않은 장애물)

---

# 왜 “whoCoversPoint() 우선”이 중요한가?

* **agentCellBlockedNear()는 서로 다른 좌표계/해상도의 costmap 간의 근방 표식 존재 여부**만 확인합니다.
  → “이 셀 주변에 agent layer가 있다”는 **약한 신호**.
* 반면 **whoCoversPoint()는 실제 에이전트의 기하(footprint) 또는 미래 궤적(tube)의 “정밀 공간 포함 관계”**를 확인합니다.
  → “에이전트의 형상/경로가 이 점을 실제로 덮는다”는 **강한 신호**.
* 따라서, **오검/미검 트레이드오프** 관점에서 whoCoversPoint() 결과를 **최우선**으로, agentCellBlockedNear()는 **보조 증거**로 쓰는 것이 맞습니다. (코드가 그렇게 되어 있음)

---

# 성능/튜닝 포인트

* **민감도 조정**

  * `agent_match_dilate_m_`: 현재 footprint의 **소확장** 폭. 너무 크면 오검↑, 너무 작으면 미검↑.
  * `agent_path_hit_dilate_m_`: 튜브 폭. 교차 판단의 관대함/보수성 조절.
  * `agent_path_hit_stride_m_`: 튜브 샘플 간격. 작을수록 정확하지만 비용↑.
  * `agent_mask_manhattan_buffer_`: 보조 근거 반경. 너무 크면 agent 아닌데도 agent로 오해할 수 있음.

* **경계/좌표계**

  * whoCoversPoint()는 오직 **월드 좌표 기반**으로 폴리곤 포함을 판정하므로, `last_agents_` 프레임이 `global_frame_`과 달라지면 **전체 skip**됩니다. 프레임 정합을 반드시 보장하세요.
  * agentCellBlockedNear()가 master↔agent_mask의 **origin/resolution mismatch**를 월드 좌표 경유로 해결하는 점이 장점.

* **지연/갱신**

  * `agents_freshness_timeout_ms_`에 의해 낡은 MultiAgent 메시지는 무시. 실제 환경에서 에이전트가 빠르게 움직이면 너무 짧게 잡을 경우 **간헐적 미검**이 발생할 수 있음.

---

필요하면 위 흐름을 **시퀀스 다이어그램**이나 **flow chart** 형태로도 정리해줄게.
#########################  
##########################  
###########################  
좋은 포인트야. 요약하면:

* **“마스크만으로 에이전트 판정이 가능한가?” → 가능은 함.**
  `agent_layer/costmap_raw`가 **현재 pose + truncated_path**를 footprint(+ dilate)로 채워서 **254** 같은 강한 cost로 찍고 있다면, **존재 판정(=여기 에이전트가 있다)** 자체는 `agent_mask`만으로 충분해.

* **그런데, path_validator_node가 지금 하는 “에이전트 판정”은 단순 존재 판정이 아니라 “귀속 + TTC 산출”까지 포함**돼.
  즉, *어느* 에이전트가 덮었는지(`machine_id`, `type_id`)와 **TTC**를 만들어서 `/path_agent_collision_info`로 내보내려면, **현재 구조에선 기하학적 매칭(whoCoversPoint)** 이 필요했어. 왜냐면 **agent_mask만으로는 ID/TTC 정보가 없기 때문**이야.

아래에 근거/배경과 선택지를 정리해줄게.

---

## 왜 현재 코드는 whoCoversPoint()를 최우선으로 쓰는가?

1. **agent_mask는 “존재”만 알려줌**

   * 네 코드의 `AgentLayer::updateCosts()`는 master_grid에 코스트를 찍을 뿐이야.
   * `AgentLayerMeta`도 퍼블리시하지만, **지금 구현에선** `machine_id=0`, `phase=0` 등 **빈 값**으로 보내고 있지? (의도적으로 경량화/샘플링만)
   * 따라서 **어느 에이전트인지**를 알 수 없고, 당연히 **TTC**도 계산할 재료가 없어.

2. **whoCoversPoint()는 MultiAgentInfoArray를 직접 읽어 기하학적으로 “귀속”을 판정**

   * (a) 현재 footprint(조금 dilate)로 포함 여부 확인
   * (b) 옵션으로 truncated_path 튜브 포함 여부 확인
   * 포함되면 **machine_id/type_id**를 붙이고, 현재 속도/heading으로 **TTC**를 추정해서 `PathAgentCollisionInfo`에 넣어줘.
   * 이게 지금 **ID/TTC를 채우는 유일한 경로**야.

3. **agentCellBlockedNear()는 보조**

   * master 코스트에 막힘이 보이면, **이게 에이전트 때문일 “가능성”**을 agent_mask에서 근방 표식(맨해튼 버퍼)로 확인하는 **힌트**로만 써.
   * 하지만 **최종 판정/귀속**은 whoCoversPoint()가 함.

---

## “마스크만으로” 판정하고 싶다면 생기는 빈칸

* **ID가 없음** → `PathAgentCollisionInfo.machine_id/type_id`를 채울 수 없음.
* **TTC가 없음** → `current_twist`/heading 같은 동역학 정보를 agent_mask에서 얻을 수 없으니까.
* 결국 지금 구조를 유지한 채로 whoCoversPoint()를 제거하면, `/path_agent_collision_info`가 **빈/부정확한 정보**가 돼.

---

## 가능한 설계 선택지 (장단점 포함)

### A) **Mask-only Gate (가장 단순)**

* 로직: `blocked_cell` 발견 → `agentCellBlockedNear()`만 보고 **에이전트 충돌로 확정** → **ID/TTC 없이** 간단한 메시지(또는 ID를 0/unknown 처리)로 publish → 리플랜/홀드 수행.
* 장점: 구현 간단, 계산 가벼움.
* 단점: **ID/TTC 유실**. 다운스트림(모니터링/로깅/휴리스틱)이 빈약해짐. 여러 에이전트가 겹치는 상황에서 설명력 저하.

### B) **Mask + 근접 귀속(Heuristic)**

* 로직: `agentCellBlockedNear()`가 true면, **가까운 에이전트 한 놈**을 `MultiAgentInfoArray`에서 **거리 최솟값**으로 간이 매칭 → 그 에이전트의 `machine_id/type_id` 채움, **TTC는 근사치**(현 위치에서의 heading/speed 기준) 계산.
  (즉, whoCoversPoint() 대신 **근접 귀속**만 수행)
* 장점: whoCoversPoint()보다 가볍고, **ID/TTC는 유지**.
* 단점: **정밀도↓** (실제 footprint/튜브가 아닌데도 근접하면 오귀속 가능). 특히 밀집/교차 상황에서 취약.

### C) **Mask + Meta(권장: 구조 개선)**

* **AgentLayer가 “메타를 셀 단위로 귀속해서” 같이 보내주도록 확장**:

  * `AgentLayer::rasterizeAgentPath()`에서 셀을 set할 때 **그 셀을 찍은 에이전트의 `machine_id/type_id`**도 함께 기록.
  * 퍼블리시 방식:

    * (1) 지금 있는 `AgentLayerMetaArray`에 **실제 기입**: 현재는 0으로 채우지만, 여기서 **진짜 machine_id/phase/mode …** 를 넣어주기.

      * 단, 모든 셀을 퍼블리시하면 트래픽이 커지니 **stride**(메타 샘플링)와 ROI를 잘 조절.
    * (2) 더 나은 방법: **별도 grid 메시지**(예: `nav2_msgs::msg::Costmap`와 동일 geometry, payload는 `uint16` machine_id map) 를 퍼블리시. type_id는 별도 사전(dict)로 보조.
* **PathValidatorNode 쪽 변경**:

  * `agent_layer_meta`(또는 “ID grid”)를 구독해서, `(mx,my)` → `(machine_id,type_id)` **룩업 테이블**을 만들어 둠.
  * 이제 `blocked_cell` 발견 시 **whoCoversPoint 없이**도 즉시 ID를 뽑아 `/path_agent_collision_info` 채움.
  * **TTC**는 `MultiAgentInfoArray`에서 해당 `machine_id`의 `current_pose/current_twist`를 읽어 계산 (지금과 동일한 식).
* 장점: **가장 정확하고 빠름**(룩업), PathValidator에서의 기하학 연산 제거, agent_mask의 판정보다 **귀속 품질 대폭↑**.
* 단점: AgentLayer/메시지 포맷 변경(작업량), 퍼블리시 데이터양 관리 필요.

### D) **현 구조 유지(기하학 우선) + “마스크는 필터” (지금과 유사)**

* 현재처럼 **whoCoversPoint() 최우선**, `agentCellBlockedNear()`는 **보조**.
* 장점: **정밀도 최상**(footprint/튜브 포함 판정).
* 단점: 계산비용(폴리곤 포함 반복)과 약간의 복잡성.

---

## “그럼 지금 코드에서 매핑은 어떻게 하고 있나?”

* **ID 매핑**: `whoCoversPoint()`가 `last_agents_`(= `/multi_agent_infos`)의 각 에이전트에 대해

  * “현재 footprint(소확장)” 포함? → **그 에이전트 hit**
  * 아니면 “truncated_path 튜브” 포함? → **그 에이전트 hit**
  * → hit이면 `machine_id/type_id`를 **직접 채움**
* **TTC 계산**: 같은 함수 내에서

  * `headingTo(current_pose, wx, wy)`와 `speedAlong(current_twist, heading)`로 **목표점에 대한 진행 성분 속도**를 구해 `dist / v_along`으로 근사.
  * 경로 튜브로 잡힌 경우는 포즈별 속도 관계가 모호하여 **-1**로 둠.

즉, **귀속/ID/TTC는 전부 whoCoversPoint()가 MultiAgentInfo와의 기하학적 조인을 통해 만든다**가 핵심.

---

## 실전 제안

* **진짜로 “마스크만으로” 가고 싶다면** → **C안**을 강추:

  1. `AgentLayer::fillFootprintAt()`에서 set한 `(i,j)` 셀에 **machine_id/type_id**를 연결해서 저장.

     * 지금은 `meta_hits`에 `(mx,my)`만 모아 `AgentLayerMetaArray`로 “샘플”만 보내고 있음.
     * 여기에 **해당 셀을 찍은 agent의 정보**를 채워 넣어.
  2. PathValidatorNode에 `agent_layer_meta` 구독 추가 → `(mx,my) -> (id,type)` 해시맵 유지.
  3. `validateWith*()`에서 `blocked_cell` 시

     * `agentCellBlockedNear()`로 **mask 존재** 확인
     * **룩업으로 ID/type 회수** → `/path_agent_collision_info` 채우기
     * **TTC는 MultiAgentInfoArray에서 해당 ID의 pose/twist로 계산** (지금 함수 재사용)

* **작게만 바꾸고 싶다면**(빠른 타협):

  * whoCoversPoint()는 유지하되, **순서를 바꿔 마스크 먼저 검사**:

    * `blocked_cell` → `agentCellBlockedNear()`가 **false**면 **바로 일반 장애물 경로**
    * `true`일 때만 whoCoversPoint() 수행(현재도 거의 이렇게 하고 있지만, “중심+이웃 whoCoversPoint 먼저” 블록을 **주석 해제**했으면 그 순서를 **mask 먼저**로 옮겨 최적화)
  * 이렇게 하면 **기하 연산을 “mask로 좁혀진 후보에서만” 수행**하므로 비용이 줄고, ID/TTC 품질은 유지돼.

---

## 작은 코드 가이드 (타협안: “mask 먼저”)

`validateWithFootprint()`/`validateWithPoints()`의 blocked_here 블록 안에서:

```cpp
// 1) 먼저 agent_mask로 빠르게 후보 필터링
bool agent_mark = false;
if (compare_agent_mask_) {
  agent_mark = agentCellBlockedNear(hit_mx, hit_my,
              static_cast<unsigned char>(agent_cost_threshold_),
              agent_mask_manhattan_buffer_);
}

if (!agent_mark) {
  // 마스크에 없으니 일반 장애물 누적
  consecutive++;
  if (consecutive >= consecutive_threshold_) {
    triggerReplan("blocked (footprint) streak threshold reached");
    return;
  }
  continue;
}

// 2) 마스크에 있으면 정밀 매칭 (ID/TTC 회수)
auto hits = agent_hit_around(wx, wy, hit_mx, hit_my); // 내부가 whoCoversPoint() 호출
if (!hits.empty()) {
  publishAgentCollisionList(hits);
  last_agent_block_time_ = this->now();
  return;
}

// 마스크가 있는데 whoCoversPoint 미히트 → 경계/프레임/타이밍 오차.
// 보수적으로 일반 장애물로 처리하거나, 버퍼를 키워 한 번 더 재시도.
```

* 이 구조면 **마스크에 걸리지 않으면** whoCoversPoint() 자체를 안 돌리고,
* **마스크에 걸린 셀만** 정밀 판정을 해서 **ID/TTC 정확도**를 보존.

---

## 결론

* 네 말처럼, **agent_mask가 “경로까지 반영된 footprint 튜브”를 lethal로 올바르게 칠하고 있다면 “존재 판정”은 mask만으로 충분**해.
* 하지만 **ID/TTC까지 필요한 현재 아키텍처**에서는 **mask만으론 정보가 부족**해서

  * **(권장)** AgentLayer가 **셀-메타(=ID/type/phase/속도 등)** 를 함께 퍼블리시하도록 확장하고 PathValidator가 룩업하는 구조(C안),
  * **혹은** 지금처럼 **whoCoversPoint()로 기하 매칭**해서 ID/TTC를 붙이는 구조(D안),
  * **어느 하나**를 선택해야 해.
* **빨리 단순화**하려면 “마스크로 후보 필터 → 후보에만 whoCoversPoint”로 바꾸는 타협안을 적용해도 체감비용은 꽤 줄어들 거야.

##############################  
################################  
##################################  
좋은 질문! 요점은 **`agentCellBlockedNear()`는 “여기 셀 주변에 에이전트 마스크가 있나?”만 빠르게 알려주는 힌트(booster)이고**, **최종 에이전트 판정(귀속/ID/TTC 산출)은 반드시 `whoCoversPoint()`가 한다**는 거예요. 코드 흐름 기준으로 깔끔히 정리해볼게요.

---

# 1) 각 함수의 역할

## `whoCoversPoint(wx, wy)`

* 입력: **월드 좌표 한 점**.
* 동작:

  1. `/multi_agent_infos`의 각 에이전트에 대해
     (a) **현재 footprint(소확장)**로 point-in-polygon 검사 → 포함되면 **HIT**
     (b) (옵션) **truncated_path 튜브** 포함 검사 → 포함되면 **HIT**
  2. **HIT이면** 그 에이전트의 `machine_id`, `type_id`를 채우고,
     현재 pose/속도(`current_pose/current_twist`)로 **TTC**(v‖ 성분으로 dist/v 계산)까지 산출.
* 핵심: **귀속(어느 에이전트인지) + TTC 산출**이 가능한 **유일한** 경로.

## `agentCellBlockedNear(mx, my, thr, manhattan_buf)`

* 입력: **마스터 costmap 셀 좌표**(mx,my).
* 동작:

  1. 마스터 grid의 (mx,my)를 **world 좌표 → agent_mask grid 좌표**로 변환.
  2. **맨해튼 거리 ≤ `manhattan_buf`** 이웃에 대해 agent_mask cost ≥ `thr`가 있으면 **true**.
* 핵심: **“이 근방이 에이전트 마스크로 칠해졌는지”만** 알려주는 **존재 힌트**.

  * **어느 에이전트인지 알 수 없음**(ID 없음).
  * **TTC 계산 불가**(속도/방향 정보 없음).

---

# 2) 두 함수가 만나는 지점 (코드 흐름)

두 경로(포인트 검사/footprint 검사) 모두 **“막힌 셀”**을 발견하면 동일한 패턴을 탑니다.

### A. `validateWithPoints()` 내부 (요약)

1. `blocked_cell = isBlockedCellKernel(mx, my)`로 **마스터 costmap**에서 막힘 감지.
2. 막혔다면:

   * (a) **기하 우선 시도**:

     ```cpp
     auto hits = agent_hit_around(wx, wy, mx, my); // 내부에서 whoCoversPoint() 호출
     if (!hits.empty()) { publishAgentCollisionList(hits); ... return; }
     ```

     → 중심 + 8이웃까지 **whoCoversPoint()**로 **정밀 귀속** 시도.
   * (b) **보조(힌트) 시도**:

     ```cpp
     if (compare_agent_mask_) {
       bool mark = agentCellBlockedNear(mx, my, agent_thr, agent_mask_manhattan_buffer_);
       if (mark) {
         auto hits2 = agent_hit_around(wx, wy, mx, my); // 다시 whoCoversPoint()
         if (!hits2.empty()) { publish...; return; }
       }
     }
     ```

     → **마스크에 찍혀 있으면** 한 번 더 귀속 재시도(경계/정렬 오차 보완).
   * (c) 그래도 에이전트 귀속 실패 → **일반 장애물 누적 처리**(streak/threshold로 replan).

### B. `validateWithFootprint()` 내부 (요약)

* 구조가 동일합니다. 차이는 **“막힌 셀”을 찾는 전 단계**에서 로봇 footprint를 월드 폴리곤으로 rasterize해 교차 검사를 한다는 점뿐이고,
* **막힘 이후의 흐름은 위와 동일**:

  1. **기하 우선** (whoCoversPoint)
  2. **마스크 힌트**(true일 때 **다시** whoCoversPoint)
  3. 실패 시 일반 장애물 카운팅

---

# 3) “보조(힌트)”라는 말의 정확한 의미

* **결정권이 없다**: `agentCellBlockedNear()` 결과가 **true라도** 그 자체로 “에이전트 충돌 확정”을 하지 않음.
  반드시 **whoCoversPoint()의 HIT**가 있어야 **에이전트로 판정/귀속/발행**.
* **용도는 두 가지**:

  1. **후보 축소 / 재시도 트리거**: 연속 격자/프레임 정렬 오차로 `whoCoversPoint()`가 중심 포인트에서 놓쳤을 때,
     **마스크가 근방에 있으면** “여긴 에이전트 가능성이 높다” → **이웃 포함 재시도**를 하게 만드는 스위치.
  2. **연산 최적화 게이트(선택)**: 구현을 바꾸면

     * “마스크가 false면 아예 whoCoversPoint()를 안 돌린다” 식으로 **게이트**로도 쓰기 좋음(후술).

---

# 4) 왜 이렇게 나눴나? (설계 의도)

* **마스크는 ‘존재 신호’만 제공** → 빠르고 보수적. 하지만 **ID/TTC 없음**.
* **whoCoversPoint는 ‘정밀 귀속’ + TTC 제공** → 계산 더 듦. 하지만 **결과 품질**(ID/TTC/설명력)이 높음.
* 그래서 “**마스크는 힌트**, **기하는 확정**” 구조로 **정확도와 비용을 절충**한 것.

---

# 5) 파라미터 튜닝 포인트

* `agent_mask_manhattan_buffer`

  * **픽셀 단위** 맨해튼 버퍼. agent_layer의 dilation/forward smear, 해상도 차이, TF 타이밍 오차를 흡수.
  * 너무 작으면 힌트가 자주 실패, 너무 크면 불필요한 재시도(=기하 연산) 증가.
* `agent_cost_threshold`

  * agent_mask에서 **에이전트로 간주할 최소 코스트**. lethal=254를 쓰면 명확.
  * layer가 여러 개면 **합성 코스트가 올라가는 경우**도 있으니 주의.

---

# 6) 추천 가능한 흐름 최적화 (원 코드와의 차이만)

현 코드는 “기하 우선 → 마스크 보조” 순서지만, **비용을 더 줄이고 싶다면**:

```cpp
// [제안] 마스크 먼저 확인 → 후보가 아니면 바로 '일반 장애물' 처리
bool agent_mark = compare_agent_mask_ &&
                  agentCellBlockedNear(hit_mx, hit_my, agent_thr, agent_mask_manhattan_buffer_);

if (!agent_mark) {
  // 마스크에 없으니 일반 장애물로 누적
  consecutive++;
  if (consecutive >= consecutive_threshold_) { triggerReplan(...); return; }
  continue;
}

// 마스크에 있는 셀만 정밀 귀속(=whoCoversPoint) 실행
auto hits = agent_hit_around(wx, wy, hit_mx, hit_my); // 내부 whoCoversPoint()
if (!hits.empty()) { publishAgentCollisionList(hits); last_agent_block_time_ = now; return; }
// 여전히 미귀속 → 경계/프레임 오차. 보수적으로 일반 장애물 처리 or 버퍼 키워 1회 재시도.
```

* 효과: **whoCoversPoint() 호출 빈도↓**, 전체 비용↓
* 전제: **agent_layer가 truncated_path까지 충분히 칠**하고 있을 것.

---

# 7) 간단한 시나리오로 이해하기

* **정적 장애물**(벽/파렛트):

  * master는 막힘, **agent_mask는 비어 있음** → `agentCellBlockedNear=false` → **일반 장애물**로 누적.
* **진짜 에이전트**(다른 로봇):

  * master는 막힘, **agent_mask=true** → whoCoversPoint() **HIT** → **ID/TTC 포함** 충돌 메시지 발행.
* **경계/정렬 오차**(셀/프레임 차):

  * master 막힘, agent_mask true인데 중심점에 대해 **즉시 HIT가 안 날 수 있음**
    → **이웃 8방향** 재시도 + 마스크 힌트로 한 번 더 whoCoversPoint() → 보정.

---

# 8) 한 줄 정의

* **`agentCellBlockedNear()` = “여기 주변, 에이전트 ‘있을 확률 높음’”이라는 **빠른 신호** (ID/TTC 없음).
* **`whoCoversPoint()` = “정말 에이전트고, 그중 **누구**이며 **TTC**는 얼마인지”까지 **확정**하는 판정기**.

이렇게 이해하면 코드가 훨씬 깔끔하게 들어올 거예요.
#####################################################  
#####################################################  
#####################################################  
#####################################################  
좋아, 현재 소스에서 **에이전트(agent) 판정이 언제·어떻게 일어나는지**를 `agentCellBlockedNear()`와 `whoCoversPoint()`까지 포함해 **함수 호출 순서 그대로** 풀어서 설명할게. (설명은 한국어, 기술어는 영어 그대로)

---

# 전체 플로우 개요 (실제 코드 흐름 기준)

## 0) 전제: 콜백/상태 업데이트

* `/global_costmap/costmap_raw` → `costmapCallback()`가 **master costmap**을 메모리에 유지.
* `/agent_layer/costmap_raw`(옵션) → `agentMaskCallback()`가 **agent_mask**를 메모리에 유지.
* `/multi_agent_infos` → `agentsCallback()`가 **마지막 MultiAgentInfoArray**와 타임스탬프를 저장.
* `/robot_status` → `robotStatusCallback()`가 **DRIVING/PLANNING 상태**에서만 유효 경로 검증이 돌게 함.
* 주기 타이머 → `updateObstacleDatabase()`는 **치사 코스트 셀을 시간 기반으로 DB에 성숙도(persistence)**를 기록(일반 장애물 판정용).

## 1) 진입점: `validatePathCallback(Path)`

* 운행 상태가 아니면 return.
* `Path`를 **global frame**으로 변환(`transformPathToGlobal()`).
* **agent-block hold**(최근 에이전트 충돌 통지 후 `agent_block_hold_sec_` 동안)는 리턴(리플랜 억제).
* 파라미터 `use_footprint_check_`에 따라:

  * `true` → `validateWithFootprint(gpath)`
  * `false` → `validateWithPoints(gpath)`

이후는 두 함수 각각의 **“막힘 감지 → 에이전트 판정 → 일반 장애물 처리”**의 동일한 3단계를 따릅니다.

---

# A) `validateWithPoints()` 경로 (포인트 기반)

### A-1. 루프 전 준비

* 내 로봇 현재 pose(TF) 취득.
* 최대 검사 거리 계산:
  `max_check_dist = min( max(min_lookahead_m_, max_speed_ * lookahead_time_sec_), path_check_distance_m_ )`
* `obstacle_db_` 스냅샷 획득(락 최소화) → **일반 장애물 성숙도** 판정에 사용.

### A-2. 경로상의 포즈 순회

* 누적거리 `acc`가 `max_check_dist`를 넘으면 중단.
* 각 포즈의 월드좌표 → **master costmap index (mx,my)** 매핑 시도.

### A-3. “막힘” 1차 감지: `isBlockedCellKernel(mx,my)`

* `kernel_half_size_ = K` 만큼 **(2K+1)×(2K+1)** 주변을 검사.
* `ignore_unknown_`이면 **NO_INFORMATION** 무시.
* 하나라도 `cost >= cost_threshold_`면 **blocked=true**.
  (즉, “우리 경로가 **치사/높은 cost** 영역과 겹친다”)

### A-4. 막혔다면 → **에이전트 판정 시퀀스**

> 여기서부터 **에이전트인지 일반 장애물인지**를 가릅니다. 순서는 “기하학적 귀속(정밀) → 마스크 힌트(보조)”입니다.

1. **정밀 귀속(기하) 우선**

   * 해당 셀의 월드 좌표 `(wx,wy)` 계산.
   * 람다 `agent_hit_around(cx,cy,mx,my)` 호출:

     * 중심점 `(cx,cy)`에 대해 `whoCoversPoint()` 실행 → **HIT**면 **에이전트 확정**.
     * 실패 시 8방향 이웃 셀의 월드 좌표로 **재시도**.
   * 하나라도 HIT가 있으면:

     * `publishAgentCollisionList(hits)`로 `/path_agent_collision_info` 발행
       (여기서 **machine_id, type_id, (가능하면) TTC**까지 채워짐)
     * `last_agent_block_time_ = now`
     * `return` (에이전트 충돌 확정 → 일반 장애물 로직으로 가지 않음)

2. **보조(힌트): agent mask 근방 존재 여부 확인**

   * `compare_agent_mask_ == true`일 때만 수행.
   * `agentCellBlockedNear(mx,my, agent_cost_threshold_, agent_mask_manhattan_buffer_)`:

     * master 셀을 월드→agent_mask index로 변환.
     * **맨해튼 거리 ≤ `agent_mask_manhattan_buffer_`** 이웃에서 **mask cost ≥ `agent_cost_threshold_`**가 하나라도 있으면 **true**.
   * `true`이면 **다시** `agent_hit_around(...)`로 정밀 귀속을 재시도:

     * HIT면 **(위와 동일)** 충돌 정보 발행 후 return.

3. **여기까지 에이전트 미귀속** → **일반 장애물로 취급**

   * `obstacle_db_` 스냅샷에서 `(mx,my)`의 **first_seen**이 `obstacle_persistence_sec_` 이상인지 확인 → **성숙도(persistent_mature)** 확보.
   * `blocked = blocked_cell && persistent_mature`.
   * `consecutive_threshold_`만큼 연속되면 `triggerReplan("blocked (points) ...")`.

> 요약: **에이전트는 반드시 `whoCoversPoint()`가 최종 판정**(ID/TTC 포함).
> `agentCellBlockedNear()`는 “근방에 agent_mask가 있네?”라는 **힌트**로만 사용되어 **정밀 귀속을 유도**합니다.

---

# B) `validateWithFootprint()` 경로 (로봇 footprint 기반 raster 검사)

### B-1. 경로 샘플링

* `footprint_step_m_` 간격으로 gpath를 샘플링(`samples`).
* 최대 검사 거리(`max_check_dist`) 초과 시 중단.

### B-2. 샘플 포즈마다 내 로봇 **footprint 스캔**

* 원형(radius) 모드이면:

  * `r_cells = ceil(robot_radius_m_ / resolution)`
  * 원판 내 격자를 훑으며 `masterCellBlocked(mx,my, master_thr)` 검사.
* 다각형 footprint이면:

  * footprint를 **pose의 yaw**로 회전·평행이동해 **월드 폴리곤** 생성.
  * bbox를 맵 인덱스로 투영 후, 포함 셀에 대해 `masterCellBlocked(...)` 검사.
* 하나라도 맞으면 `blocked_here=true`, 해당 **히트셀(mx,my)** 저장.

### B-3. 막혔다면 → **에이전트 판정 시퀀스** (A-4와 동일)

1. **정밀 귀속**: `whoCoversPoint()`(중심+8이웃) → HIT면 충돌 발행 후 return
2. **마스크 힌트**: `agentCellBlockedNear(...)`가 true면 **다시** 정밀 귀속 재시도 → HIT면 발행 후 return
3. 실패 시 **일반 장애물 누적** → `consecutive_threshold_` 도달 시 `triggerReplan(...)`

---

# C) `whoCoversPoint(wx,wy)` 내부 동작(정밀 귀속)

1. **신선도 검사**

   * `last_agents_`가 없거나 `agents_freshness_timeout_ms_`를 초과했거나, frame 불일치면 **빈 결과** 반환.

2. **모든 에이전트 반복**

   * footprint 점 3개 미만이면 skip.

   **(1) 현재 위치 footprint(소확장) 포함 여부**

   * `agent_match_dilate_m_`만큼 **등방성 dilate**한 후
     `current_pose`의 yaw로 회전, 평행이동 → **월드 폴리곤**.
   * `pointInPolygon(poly_world, wx, wy)`이면 **HIT**:

     * `machine_id`, `type_id`, `(wx,wy)` 채움.
     * **TTC**: `headingTo(current_pose, wx, wy)` → `speedAlong(current_twist, heading)`

       * `v‖ > 0.05`면 `dist / v‖`
       * 그 외는 `-1.0f` (unknown)
     * `note = "agent footprint overlap; ..."`
     * 결과 벡터에 push.

   **(2) truncated_path “튜브” 포함 여부** (옵션)

   * `agent_path_hit_enable_ == true`면 `pathTubeCoversPoint()`:

     * `truncated_path`를 `agent_path_hit_stride_m_` 거리 기준으로 간격 샘플.
     * 각 포즈에서 footprint를 **소확장(dilate_m)**만 적용해 월드 폴리곤 생성.
     * 점 `(wx,wy)`가 포함되면 **HIT**:

       * `machine_id`, `type_id`, `(wx,wy)`, `ttc_first=-1.0f`,
       * `note = "agent truncated_path overlap"`
       * push.

3. 누적된 HIT들이 반환됩니다. (여러 에이전트가 커버하면 **복수** 가능)

> 즉, **ID**와 **TTC**(가능하면)는 **오로지 `/multi_agent_infos`**의 구조화된 데이터에서 유도되며,
> **agent_mask costmap만으로는 알 수 없습니다.** (mask에는 ID/속도/방향이 없음)

---

# D) `agentCellBlockedNear(mx,my, thr, manhattan_buf)` 내부 동작(보조 힌트)

1. `costmap_`와 `agent_mask_`를 **락 분리**로 안전하게 로컬 복사.
2. **master index → world (wx,wy)** 변환.
3. **world → agent_mask index (ax,ay)** 변환 불가면 false.
4. `(ax,ay)` 주변을 **맨해튼 거리 ≤ `manhattan_buf`**로 스캔:

   * 이웃 중 하나라도 **agent_mask cost ≥ thr**이면 **true**.
5. 그렇지 않으면 **false**.

> 결론: **“이 근방에 에이전트 마스크가 칠해졌는지”**만 알려주는 **존재 힌트**.
> 이 값이 **true**여도 **곧바로 에이전트 확정 X** → 반드시 **`whoCoversPoint()` 재시도**로 귀속을 마쳐야 함.

---

# E) Replan/억제와 충돌 통지

* **충돌 통지**: `publishAgentCollisionList(hits)`

  * `machine_id[]`, `type_id[]`, `x[]`, `y[]`, `ttc_first[]`, `note[]`를 `/path_agent_collision_info`로 발행.
* **Agent-block hold**: `last_agent_block_time_`를 갱신 → **`agent_block_hold_sec_` 동안** `validatePathCallback()` 처음에서 **리턴**해 리플랜 남발 억제.
* **일반 장애물 리플랜**: `triggerReplan(reason)`

  * `cooldown_sec_` 내 재중복 방지.
  * (옵션) `publish_false_pulse_`면 일정 시간 후 false 펄스 발행.

---

# F) 파라미터가 판정에 미치는 영향 (핵심만)

* **`cost_threshold_`**: master에서 “막힘”으로 보는 최소 cost.
* **`agent_cost_threshold_`**: agent_mask에서 “에이전트 마스크 존재”로 보는 최소 cost.
* **`agent_mask_manhattan_buffer_`**: mask와 master 좌표 정렬/해상도 차, dilation/forward smear를 흡수하는 **보조 반경**(픽셀 단위).
* **`agent_match_dilate_m_`**: `whoCoversPoint()`에서 footprint를 **소확장**해 기하 매칭의 **관용도**를 높임.
* **`agent_path_hit_*`**: truncated_path 튜브 매칭의 stride/dilate/limit.
* **`obstacle_persistence_sec_` + `consecutive_threshold_`**: **일반 장애물**로 확정하기까지의 **시간·공간적 성숙도**.

---

# G) “agent_mask만으로 충분하지 않나?”에 대한 답

* **차단(avoid) 관점**만 보면 agent_mask(=254)로도 “막혔다”는 사실은 충분히 알 수 있어요.
  하지만 본 노드는:

  1. **에이전트와 일반 장애물의 구분**(Downstream 정책 차별 적용 가능),
  2. **ID/Type**,
  3. (가능하다면) **TTC**까지 포함한 정보를 `/path_agent_collision_info`로 내보내요.
* 이 **추가 의미론**은 **`/multi_agent_infos`** 기반의 `whoCoversPoint()` 없이는 만들 수 없고,
  **agent_mask만으론 불가능**합니다. (mask에는 ID·속도·방향 정보가 없음)

---

# H) 한 화면 요약 (의사코드)

```cpp
for each path sample (points or footprint cells):
  if master indicates blocked:
    // 1) 정밀 귀속 (기하 우선)
    hits = whoCoversPoint(center) or neighbors
    if !hits.empty():
       publishAgentCollisionList(hits)
       last_agent_block_time_ = now
       return

    // 2) 보조 힌트 (mask가 근방에 있나?)
    if compare_agent_mask_ &&
       agentCellBlockedNear(mx,my, agent_thr, manhattan_buf):
       hits = whoCoversPoint(center) or neighbors   // 재시도
       if !hits.empty():
          publishAgentCollisionList(hits)
          last_agent_block_time_ = now
          return

    // 3) 에이전트 미귀속 → 일반 장애물 누적
    if persistent_mature:
       consecutive++
       if consecutive >= consecutive_threshold_:
          triggerReplan("blocked ...")
          return
  else:
    consecutive = 0
```

---

필요하면 이 흐름을 “**마스크 우선 게이트 → 기하 귀속**” 방식으로 바꾸는 최적화 패턴도 제안할 수 있어. 하지만 **ID/TTC를 포함한 충돌 메시지**를 계속 쓰려면, **최종 판정은 언제나 `whoCoversPoint()`**가 맡아야 한다는 점은 그대로 유지돼야 해.
###############################################  
###############################################  
###############################################  
###############################################  
###############################################  
맞아요. 정확히 그 순서예요. 지금 코드 흐름을 짧게 정리하면:

1. **먼저 `whoCoversPoint()` 시도**

   * 막힌 셀의 월드좌표 `(wx, wy)` 기준으로
   * 중심 → 8방 이웃 순서로 `whoCoversPoint()`로 **기하학적 귀속(정밀 판정)**을 합니다.
   * 한 건이라도 HIT 나면 **여기서 바로 에이전트 확정**(ID/TTC 포함) → 메시지 발행 후 반환.

2. **못 찾았으면 보조 힌트: `agentCellBlockedNear()`**

   * `compare_agent_mask_`가 켜져 있고, 에이전트 마스크가 **근방(맨해튼 ≤ buffer)**에 보이면 `true`.

3. **힌트가 있으면 `whoCoversPoint()`를 다시 한 번**

   * 같은 중심→이웃 절차로 **다시 정밀 귀속 재시도**.
   * 이번에 HIT 나면 에이전트 확정, 아니면 **일반 장애물**로 넘어갑니다.

짧은 의사코드로 보면:

```cpp
if (blocked_cell) {
  // 1) 기하 우선
  hits = whoCoversPoint(center) || neighbors;
  if (!hits.empty()) { publishAgent(hits); return; }

  // 2) 마스크 힌트
  if (compare_agent_mask_ &&
      agentCellBlockedNear(mx,my, agent_thr, manhattan_buf)) {

    // 3) 힌트가 있을 때만 재시도
    hits = whoCoversPoint(center) || neighbors;
    if (!hits.empty()) { publishAgent(hits); return; }
  }

  // 4) 여전히 미귀속 → 일반 장애물 누적 처리
  accumulate_obstacle_and_maybe_replan();
}
```

핵심 포인트:

* **최종 에이전트 판정은 항상 `whoCoversPoint()`**(ID/TTC 계산 가능).
* `agentCellBlockedNear()`는 **재시도를 할 가치가 있는지 알려주는 힌트**일 뿐, 단독으로 “에이전트 확정”을 하지 않습니다.

그래서 질문 그대로 표현하면:
**“맞다 — 처음엔 `whoCoversPoint()`만 하고, 실패하면 `agentCellBlockedNear()`로 힌트를 본 뒤, 힌트가 있을 때만 `whoCoversPoint()`를 다시 한다.”**
########################  
###########################  
##########################  
알겠습니다. `agent_layer`와 동일한 로직을 `path_validator_node`에 적용하여, 토픽 대신 YAML에서 footprint를 읽어오도록 수정해 보겠습니다.

`path_validator_node`는 `agent_layer`와 달리 **일반 ROS 2 노드**이므로 `name_ + "."` 접두사를 사용할 필요가 없습니다. 이 점을 반영하여 코드를 수정합니다.

-----

### 1\. `path_validator_node.hpp` 변경 사항

`agent_layer`와 마찬가지로, `std::map`을 include하고, footprint/radius 데이터를 저장할 struct와 map, 그리고 헬퍼 함수 2개를 `private:` 섹션에 선언합니다.

```cpp
/* ****************************************
 * [BEFORE] path_validator_node.hpp - includes
 * ****************************************
 */
#include <chrono>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
```

```cpp
/* ****************************************
 * [AFTER] path_validator_node.hpp - includes
 * ****************************************
 */
#include <chrono>
#include <algorithm>

// [NEW] Added for std::map
#include <map>

#include "rclcpp/rclcpp.hpp"
```

-----

```cpp
/* ****************************************
 * [BEFORE] path_validator_node.hpp - private members
 * ****************************************
 */
  // === Agent 충돌 식별 ===
  struct AgentHit {
    uint16_t machine_id{0};
// ...
    std::string note;
  };

  // wx, wy를 커버하는 agent의 footprint 또는 truncated_path 튜브를 찾아 리턴
  std::vector<AgentHit> whoCoversPoint(double wx, double wy) const;
```

```cpp
/* ****************************************
 * [AFTER] path_validator_node.hpp - private members
 * ****************************************
 */
  // === Agent 충돌 식별 ===
  struct AgentHit {
    uint16_t machine_id{0};
// ...
    std::string note;
  };

  // [NEW] Map to store footprint data from YAML
  struct AgentFootprintData
  {
    // existing code uses Point32, so we store Point32
    std::vector<geometry_msgs::msg::Point32> points;
    double radius{0.0};
    bool use_radius{true};
  };
  // Map from machine_id to its footprint/radius data
  std::map<uint16_t, AgentFootprintData> agent_footprints_;

  // [NEW] Helper to get footprint for a given agent
  std::vector<geometry_msgs::msg::Point32> 
  getFootprintForAgent(const multi_agent_msgs::msg::MultiAgentInfo & a) const;

  // [NEW] Helper to convert nav2_costmap_2d::makeFootprint... results
  static std::vector<geometry_msgs::msg::Point32> toPoint32(
      const std::vector<geometry_msgs::msg::Point>& points);


  // wx, wy를 커버하는 agent의 footprint 또는 truncated_path 튜브를 찾아 리턴
  std::vector<AgentHit> whoCoversPoint(double wx, double wy) const;
```

-----

### 2\. `path_validator_node.cpp` 변경 사항

#### Includes 추가

`onInitialize`가 아닌 `PathValidatorNode` 생성자에서 `makeFootprint...` 헬퍼를 사용하기 위해 `#include <nav2_costmap_2d/footprint.hpp>`를 추가합니다.

```cpp
/* ****************************************
 * [BEFORE] path_validator_node.cpp - includes
 * ****************************************
 */
#include "replan_monitor/path_validator_node.hpp"
using std::placeholders::_1;

namespace replan_monitor
{
```

```cpp
/* ****************************************
 * [AFTER] path_validator_node.cpp - includes
 * ****************************************
 */
#include "replan_monitor/path_validator_node.hpp"

// [NEW] For makeFootprintFromString and makeFootprintFromRadius
#include <nav2_costmap_2d/footprint.hpp>

using std::placeholders::_1;

namespace replan_monitor
{
```

#### 헬퍼 함수 구현 추가

`.hpp`에서 선언한 헬퍼 함수 2개의 구현을 `.cpp` 파일 상단 (namespace 내부)에 추가합니다.

```cpp
/* ****************************************
 * [BEFORE] path_validator_node.cpp - namespace start
 * ****************************************
 */
namespace replan_monitor
{

PathValidatorNode::PathValidatorNode()
: Node("path_validator_node")
{
```

```cpp
/* ****************************************
 * [AFTER] path_validator_node.cpp - namespace start
 * ****************************************
 */
namespace replan_monitor
{

// [NEW] Implementation of static helper (from agent_layer)
std::vector<geometry_msgs::msg::Point32> PathValidatorNode::toPoint32(
    const std::vector<geometry_msgs::msg::Point>& points)
{
    std::vector<geometry_msgs::msg::Point32> points32;
    points32.reserve(points.size());
    for (const auto& p : points) {
        geometry_msgs::msg::Point32 p32;
        p32.x = static_cast<float>(p.x);
        p32.y = static_cast<float>(p.y);
        p32.z = 0.0f;
        points32.push_back(p32);
    }
    return points32;
}

// [NEW] Implementation of helper to get footprint from loaded map
std::vector<geometry_msgs::msg::Point32> 
PathValidatorNode::getFootprintForAgent(const multi_agent_msgs::msg::MultiAgentInfo & a) const
{
    auto it = agent_footprints_.find(a.machine_id);
    if (it == agent_footprints_.end()) {
        RCLCPP_WARN_ONCE(get_logger(), 
          "No footprint data found in YAML for machine_id %u. Cannot check agent collision.",
          a.machine_id);
        return {}; // Return empty vector
    }

    const auto& data = it->second;

    if (data.use_radius) {
        std::vector<geometry_msgs::msg::Point> points = 
            nav2_costmap_2d::makeFootprintFromRadius(data.radius);
        return toPoint32(points);
    } else {
        return data.points; // Return pre-loaded Point32 vector
    }
}


PathValidatorNode::PathValidatorNode()
: Node("path_validator_node")
{
```

#### `PathValidatorNode()` 생성자 수정

`agent_footprints_` 맵을 채우는 YAML 파라미터 로딩 로직을 추가합니다. (`this->get_parameter(...)` 호출이 끝나는 부분)

```cpp
/* ****************************************
 * [BEFORE] path_validator_node.cpp - Constructor end
 * ****************************************
 */
  agent_path_hit_enable_      = this->get_parameter("agent_path_hit_enable").as_bool();
  agent_path_hit_stride_m_    = this->get_parameter("agent_path_hit_stride_m").as_double();
  agent_path_hit_dilate_m_    = this->get_parameter("agent_path_hit_dilate_m").as_double();
  agent_path_hit_max_poses_   = this->get_parameter("agent_path_hit_max_poses").as_int();

  // ===== TF =====
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
```

```cpp
/* ****************************************
 * [AFTER] path_validator_node.cpp - Constructor end
 * ****************************************
 */
  agent_path_hit_enable_      = this->get_parameter("agent_path_hit_enable").as_bool();
  agent_path_hit_stride_m_    = this->get_parameter("agent_path_hit_stride_m").as_double();
  agent_path_hit_dilate_m_    = this->get_parameter("agent_path_hit_dilate_m").as_double();
  agent_path_hit_max_poses_   = this->get_parameter("agent_path_hit_max_poses").as_int();

  // [NEW] Add declaration for the robot list
  this->declare_parameter<std::vector<std::string>>("robot_ids", std::vector<std::string>({}));

  // [NEW] Loop 1: Declare all sub-parameters for each robot_id
  // (We get robot_ids first to declare, this is a bit redundant but safe)
  std::vector<std::string> robot_ids_to_declare;
  try {
    robot_ids_to_declare = this->get_parameter("robot_ids").as_string_array();
  } catch (...) {
    RCLCPP_WARN(get_logger(), "No 'robot_ids' list found in YAML, will not load any agent footprints.");
  }
  
  for (const auto & id_str : robot_ids_to_declare) {
    // This is a regular node, no 'name_' prefix.
    this->declare_parameter(id_str + ".machine_id", rclcpp::ParameterValue(0));
    this->declare_parameter(id_str + ".robot_radius", rclcpp::ParameterValue(0.0));
    this->declare_parameter(id_str + ".footprint", rclcpp::ParameterValue(std::string("[]")));
  }

  // [NEW] Loop 2: Get parameters and populate the map
  agent_footprints_.clear();
  std::vector<std::string> robot_ids;
  this->get_parameter("robot_ids", robot_ids); // Get the list again (now that it's declared)
  
  for (const auto & id_str : robot_ids) {
    
    int machine_id_int = 0;
    this->get_parameter(id_str + ".machine_id", machine_id_int);
    if (machine_id_int == 0) continue; 

    uint16_t machine_id = static_cast<uint16_t>(machine_id_int);
    
    AgentFootprintData data;
    std::string footprint_str;
    this->get_parameter(id_str + ".footprint", footprint_str);
    this->get_parameter(id_str + ".robot_radius", data.radius);

    std::vector<geometry_msgs::msg::Point> footprint_points;
    if (nav2_costmap_2d::makeFootprintFromString(footprint_str, footprint_points) &&
        footprint_points.size() >= 3)
    {
      data.points = toPoint32(footprint_points); // Convert Point to Point32
      data.use_radius = false;
    } else {
      data.use_radius = true;
    }

    agent_footprints_[machine_id] = data;
    
    RCLCPP_INFO(get_logger(), 
      "Loaded footprint for machine_id %u: use_radius=%s, points=%zu",
      machine_id, (data.use_radius ? "true" : "false"), data.points.size());
  }

  // ===== TF =====
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
```

#### `pathTubeCoversPoint()` 수정

`a.footprint`를 사용하는 대신 `getFootprintForAgent(a)` 헬퍼를 호출하도록 변경합니다.

```cpp
/* ****************************************
 * [BEFORE] path_validator_node.cpp - pathTubeCoversPoint()
 * ****************************************
 */
    // footprint를 얇게 등방성 확장한 로컬 폴리곤
    const auto & fp = a.footprint.polygon.points;
    if (fp.size() < 3) continue;

    // 로컬 dilate
```

```cpp
/* ****************************************
 * [AFTER] path_validator_node.cpp - pathTubeCoversPoint()
 * ****************************************
 */
    // [CHANGED] footprint를 얇게 등방성 확장한 로컬 폴리곤
    const auto fp = getFootprintForAgent(a);
    if (fp.size() < 3) continue;

    // 로컬 dilate
```

#### `whoCoversPoint()` 수정

`a.footprint`를 사용하는 대신 `getFootprintForAgent(a)` 헬퍼를 호출하도록 변경합니다.

```cpp
/* ****************************************
 * [BEFORE] path_validator_node.cpp - whoCoversPoint()
 * ****************************************
 */
  for (const auto & a : last_agents_->agents) {
    const auto & fp = a.footprint.polygon.points;
    if (fp.size() < 3) continue;

    // 1) 현재 위치 footprint(소확장) 커버?
```

```cpp
/* ****************************************
 * [AFTER] path_validator_node.cpp - whoCoversPoint()
 * ****************************************
 */
  for (const auto & a : last_agents_->agents) {
    // [CHANGED] Get footprint from YAML map
    const auto fp = getFootprintForAgent(a);
    if (fp.size() < 3) continue;

    // 1) 현재 위치 footprint(소확장) 커버?
```
