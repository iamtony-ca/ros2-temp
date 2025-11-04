좋아! 요청한 대로 AgentLayer를 **“내부맵(오프스크린 버퍼) + 매 사이클 클리어 + Max-merge(updateWithMax)”** 방식으로 전면 교체한 **완전체(hpp/cpp)** 코드를 준비했어.
핵심 아이디어는:

* **internal_grid_**: master(전역 costmap)와 동일한 크기의 내부 코스트맵 버퍼
* **updateCosts()**: 시작할 때 내부맵을 **FREE_SPACE로 전부 클리어**, 그 후 **에이전트 정보만** 내부맵에 래스터라이즈
* 마지막 단계에서 **Max-merge**: `if (internal > master) master = internal` 형태로 **지정된 인덱스 범위만** 업데이트

아래 코드는 ROS 2 Jazzy + Nav2 기준이며, 이전 대화에서 사용하던 파라미터들을 그대로 지원(이름 동일)하면서 내부 버퍼-클리어-병합 구조만 바꾼 버전이야.
(플러그인 XML은 기존과 동일하게 `multi_agent_nav2/AgentLayer` 클래스를 가리키면 됨.)

---

# agent_layer.hpp

```cpp
#pragma once

#include <mutex>
#include <vector>
#include <string>
#include <utility>
#include <algorithm>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <multi_agent_msgs/msg/multi_agent_info_array.hpp>
#include <multi_agent_msgs/msg/agent_status.hpp>

namespace multi_agent_nav2
{

class AgentLayer : public nav2_costmap_2d::Layer
{
public:
  AgentLayer();

  // lifecycle
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override { current_ = true; }

  // costmap callbacks
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;

  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;

  void matchSize() override;  // ★ master 크기 변경 시 내부맵도 동기화
  bool isClearable() override { return true; }

private:
  // node
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_shared_;

  // I/O
  rclcpp::Subscription<multi_agent_msgs::msg::MultiAgentInfoArray>::SharedPtr sub_;

  // last data
  std::mutex data_mtx_;
  multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr last_infos_;
  rclcpp::Time last_stamp_;

  // parameters
  bool        enabled_{true};
  std::string topic_{"/multi_agent_infos"};
  uint16_t    self_machine_id_{0};
  std::string self_type_id_{};
  bool        use_path_header_frame_{true};
  double      roi_range_m_{12.0};
  int         freshness_timeout_ms_{800};
  int         max_poses_{40};
  bool        qos_reliable_{true};

  // cost policy
  unsigned char lethal_cost_{254};
  unsigned char moving_cost_{180};
  unsigned char waiting_cost_{200};
  int         manual_cost_bias_{30};

  // geometry
  // [변경점] 등방성 dilation 과 전방(+x) smear 분리
  double      dilation_m_{0.05};
  double      forward_smear_m_{0.25};
  double      sigma_k_{2.0};     // pos_std_m 가중치

  // path 페인팅 옵션(이동중일 때만)
  bool        paint_path_{false};
  double      path_stride_m_{0.30};
  double      path_dilation_m_{0.02};
  unsigned char path_cost_{160};

  // bounds cache for this cycle
  double touch_min_x_{0.0}, touch_min_y_{0.0}, touch_max_x_{0.0}, touch_max_y_{0.0};
  bool   touched_{false};

  // [핵심] 내부 costmap 버퍼
  std::unique_ptr<nav2_costmap_2d::Costmap2D> internal_grid_;
  void clearInternal(); // FREE_SPACE로 클리어

  // helpers
  void infosCallback(const multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr msg);
  bool stale(const rclcpp::Time & stamp) const;
  bool isSelf(const multi_agent_msgs::msg::MultiAgentInfo & a) const;

  unsigned char computeCost(const multi_agent_msgs::msg::MultiAgentInfo & a) const;
  double computeDilation(const multi_agent_msgs::msg::MultiAgentInfo & a) const;

  static inline bool isMovingPhase(uint8_t phase);

  // rasterizers
  void rasterizeAgentIntoInternal(const multi_agent_msgs::msg::MultiAgentInfo & a);
  void fillFootprintAt(const geometry_msgs::msg::PolygonStamped & fp,
                       const geometry_msgs::msg::Pose & pose,
                       double extra_dilation_m,
                       double forward_len_m, // 전방(+x)로만 스미어 길이
                       nav2_costmap_2d::Costmap2D * grid,
                       unsigned char cost);

  // directional footprint dilation in local frame
  static std::vector<geometry_msgs::msg::Point>
  dilateFootprintDirectional(const std::vector<geometry_msgs::msg::Point32> & in,
                             double iso_dilate_m, double forward_len_m);

  // util
  static bool pointInPolygon(const std::vector<geometry_msgs::msg::Point> & poly,
                             double x, double y);
};

inline bool AgentLayer::isMovingPhase(uint8_t phase)
{
  using S = multi_agent_msgs::msg::AgentStatus;
  return phase == S::STATUS_MOVING || phase == S::STATUS_PATH_SEARCHING;
}

} // namespace multi_agent_nav2
```

---

# agent_layer.cpp

```cpp
#include "multi_agent_nav2/agent_layer.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>

namespace multi_agent_nav2
{

AgentLayer::AgentLayer() {}

void AgentLayer::onInitialize()
{
  node_shared_ = node_.lock();
  if (!node_shared_) {
    throw std::runtime_error("AgentLayer: failed to lock lifecycle node");
  }

  // ---- declare ----
  node_shared_->declare_parameter("enabled", rclcpp::ParameterValue(true));
  node_shared_->declare_parameter("topic", rclcpp::ParameterValue(std::string("/multi_agent_infos")));
  node_shared_->declare_parameter("self_machine_id", rclcpp::ParameterValue(0));
  node_shared_->declare_parameter("self_type_id", rclcpp::ParameterValue(std::string("")));
  node_shared_->declare_parameter("use_path_header_frame", rclcpp::ParameterValue(true));
  node_shared_->declare_parameter("roi_range_m", rclcpp::ParameterValue(12.0));
  node_shared_->declare_parameter("freshness_timeout_ms", rclcpp::ParameterValue(800));
  node_shared_->declare_parameter("max_poses", rclcpp::ParameterValue(40));
  node_shared_->declare_parameter("qos_reliable", rclcpp::ParameterValue(true));

  node_shared_->declare_parameter("lethal_cost", rclcpp::ParameterValue(254));
  node_shared_->declare_parameter("moving_cost", rclcpp::ParameterValue(180));
  node_shared_->declare_parameter("waiting_cost", rclcpp::ParameterValue(200));
  node_shared_->declare_parameter("manual_cost_bias", rclcpp::ParameterValue(30));

  node_shared_->declare_parameter("dilation_m", rclcpp::ParameterValue(0.05));
  node_shared_->declare_parameter("forward_smear_m", rclcpp::ParameterValue(0.25));
  node_shared_->declare_parameter("sigma_k", rclcpp::ParameterValue(2.0));

  node_shared_->declare_parameter("paint_path", rclcpp::ParameterValue(false));
  node_shared_->declare_parameter("path_stride_m", rclcpp::ParameterValue(0.30));
  node_shared_->declare_parameter("path_dilation_m", rclcpp::ParameterValue(0.02));
  node_shared_->declare_parameter("path_cost", rclcpp::ParameterValue(160));

  // ---- get ----
  node_shared_->get_parameter("enabled", enabled_);
  node_shared_->get_parameter("topic", topic_);
  {
    int tmp = 0;
    node_shared_->get_parameter("self_machine_id", tmp);
    self_machine_id_ = static_cast<uint16_t>(tmp);
  }
  node_shared_->get_parameter("self_type_id", self_type_id_);
  node_shared_->get_parameter("use_path_header_frame", use_path_header_frame_);
  node_shared_->get_parameter("roi_range_m", roi_range_m_);
  node_shared_->get_parameter("freshness_timeout_ms", freshness_timeout_ms_);
  node_shared_->get_parameter("max_poses", max_poses_);
  node_shared_->get_parameter("qos_reliable", qos_reliable_);

  {
    int v=254; node_shared_->get_parameter("lethal_cost", v);
    lethal_cost_ = static_cast<unsigned char>(std::clamp(v, 0, 254));
  }
  {
    int v=180; node_shared_->get_parameter("moving_cost", v);
    moving_cost_ = static_cast<unsigned char>(std::clamp(v, 0, 254));
  }
  {
    int v=200; node_shared_->get_parameter("waiting_cost", v);
    waiting_cost_ = static_cast<unsigned char>(std::clamp(v, 0, 254));
  }
  node_shared_->get_parameter("manual_cost_bias", manual_cost_bias_);

  node_shared_->get_parameter("dilation_m", dilation_m_);
  node_shared_->get_parameter("forward_smear_m", forward_smear_m_);
  node_shared_->get_parameter("sigma_k", sigma_k_);

  node_shared_->get_parameter("paint_path", paint_path_);
  node_shared_->get_parameter("path_stride_m", path_stride_m_);
  node_shared_->get_parameter("path_dilation_m", path_dilation_m_);
  {
    int v=160; node_shared_->get_parameter("path_cost", v);
    path_cost_ = static_cast<unsigned char>(std::clamp(v, 0, 254));
  }

  current_ = true;
  matchSize();

  if (enabled_) activate();
}

void AgentLayer::matchSize()
{
  // master 크기와 동일하게 내부 버퍼 준비
  auto & master = layered_costmap_->getCostmap();
  if (!internal_grid_ ||
      internal_grid_->getSizeInCellsX() != master.getSizeInCellsX() ||
      internal_grid_->getSizeInCellsY() != master.getSizeInCellsY() ||
      internal_grid_->getResolution()   != master.getResolution() ||
      internal_grid_->getOriginX()      != master.getOriginX() ||
      internal_grid_->getOriginY()      != master.getOriginY())
  {
    internal_grid_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
        master.getSizeInCellsX(),
        master.getSizeInCellsY(),
        master.getResolution(),
        master.getOriginX(),
        master.getOriginY(),
        nav2_costmap_2d::FREE_SPACE);
  }
}

void AgentLayer::activate()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  if (qos_reliable_) qos.reliable(); else qos.best_effort();

  sub_ = node_shared_->create_subscription<multi_agent_msgs::msg::MultiAgentInfoArray>(
      topic_, qos, std::bind(&AgentLayer::infosCallback, this, std::placeholders::_1));
}

void AgentLayer::deactivate()
{
  sub_.reset();
}

void AgentLayer::infosCallback(
  const multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(data_mtx_);
  last_infos_ = msg;
  last_stamp_ = msg->header.stamp;
}

bool AgentLayer::stale(const rclcpp::Time & stamp) const
{
  return (node_shared_->now() - stamp) >
         rclcpp::Duration::from_nanoseconds(
           static_cast<int64_t>(freshness_timeout_ms_) * 1000000LL);
}

bool AgentLayer::isSelf(const multi_agent_msgs::msg::MultiAgentInfo & a) const
{
  return (a.machine_id == self_machine_id_) && (a.type_id == self_type_id_);
}

unsigned char AgentLayer::computeCost(const multi_agent_msgs::msg::MultiAgentInfo & a) const
{
  using S = multi_agent_msgs::msg::AgentStatus;
  const bool moving = (a.status.phase == S::STATUS_MOVING) ||
                      (a.status.phase == S::STATUS_PATH_SEARCHING);
  unsigned char base = moving ? moving_cost_ : waiting_cost_;
  if (a.mode == "manual") {
    int c = static_cast<int>(base) + manual_cost_bias_;
    return static_cast<unsigned char>(std::clamp(c, 0, 254));
  }
  return base;
}

double AgentLayer::computeDilation(const multi_agent_msgs::msg::MultiAgentInfo & a) const
{
  double r = dilation_m_;
  if (a.pos_std_m >= 0.0) r += sigma_k_ * a.pos_std_m;
  return r;
}

void AgentLayer::clearInternal()
{
  if (!internal_grid_) return;
  unsigned char * map = internal_grid_->getCharMap();
  const size_t N = static_cast<size_t>(internal_grid_->getSizeInCellsX()) *
                   static_cast<size_t>(internal_grid_->getSizeInCellsY());
  std::memset(map, nav2_costmap_2d::FREE_SPACE, N * sizeof(unsigned char));
}

void AgentLayer::updateBounds(double robot_x, double robot_y, double /*robot_yaw*/,
                              double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_) return;

  touched_ = false;
  touch_min_x_ =  1e9; touch_min_y_ =  1e9;
  touch_max_x_ = -1e9; touch_max_y_ = -1e9;

  std::vector<multi_agent_msgs::msg::MultiAgentInfo> infos;
  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    if (!last_infos_ || stale(last_stamp_)) return;
    infos.assign(last_infos_->agents.begin(), last_infos_->agents.end());
  }

  const std::string & global_frame = layered_costmap_->getGlobalFrameID();

  for (const auto & a : infos) {
    if (isSelf(a)) continue;

    // ROI by distance from our robot
    const double dx = a.current_pose.pose.position.x - robot_x;
    const double dy = a.current_pose.pose.position.y - robot_y;
    if (std::hypot(dx, dy) > roi_range_m_) continue;

    // frame check (optional)
    if (use_path_header_frame_) {
      if (!a.truncated_path.header.frame_id.empty() &&
          a.truncated_path.header.frame_id != global_frame) {
        continue;
      }
      if (!last_infos_->header.frame_id.empty() &&
          last_infos_->header.frame_id != global_frame) {
        continue;
      }
    }

    // 현재 위치
    {
      const auto & p = a.current_pose.pose.position;
      touch_min_x_ = std::min(touch_min_x_, p.x);
      touch_min_y_ = std::min(touch_min_y_, p.y);
      touch_max_x_ = std::max(touch_max_x_, p.x);
      touch_max_y_ = std::max(touch_max_y_, p.y);
      touched_ = true;
    }

    // 옵션: path bounds (페인트 옵션을 켠 경우에만)
    if (paint_path_ && isMovingPhase(a.status.phase)) {
      const int limit = std::min<int>(a.truncated_path.poses.size(), max_poses_);
      for (int i = 0; i < limit; ++i) {
        const auto & p = a.truncated_path.poses[i].pose.position;
        touch_min_x_ = std::min(touch_min_x_, p.x);
        touch_min_y_ = std::min(touch_min_y_, p.y);
        touch_max_x_ = std::max(touch_max_x_, p.x);
        touch_max_y_ = std::max(touch_max_y_, p.y);
        touched_ = true;
      }
    }
  }

  if (touched_) {
    *min_x = std::min(*min_x, touch_min_x_);
    *min_y = std::min(*min_y, touch_min_y_);
    *max_x = std::max(*max_x, touch_max_x_);
    *max_y = std::max(*max_y, touch_max_y_);
  }
}

// === 로컬 footprint 등방성+전방(+x) 보정 ===
std::vector<geometry_msgs::msg::Point>
AgentLayer::dilateFootprintDirectional(const std::vector<geometry_msgs::msg::Point32> & in,
                                       double iso_dilate_m, double forward_len_m)
{
  std::vector<geometry_msgs::msg::Point> out; out.reserve(in.size());
  if (in.empty()) return out;

  double cx=0, cy=0;
  for (auto & p : in) { cx += p.x; cy += p.y; }
  cx /= static_cast<double>(in.size());
  cy /= static_cast<double>(in.size());

  for (auto & p : in) {
    double vx = p.x - cx, vy = p.y - cy;
    double n = std::hypot(vx, vy); if (n < 1e-6) n = 1.0;

    double x_local = p.x + iso_dilate_m * (vx / n);
    double y_local = p.y + iso_dilate_m * (vy / n);

    if (forward_len_m > 1e-6 && (p.x - cx) >= 0.0) {
      x_local += forward_len_m; // 전방(+x)만
    }

    geometry_msgs::msg::Point q;
    q.x = x_local; q.y = y_local; q.z = 0.0;
    out.push_back(q);
  }
  return out;
}

bool AgentLayer::pointInPolygon(const std::vector<geometry_msgs::msg::Point> & poly,
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

void AgentLayer::fillFootprintAt(const geometry_msgs::msg::PolygonStamped & fp,
                                 const geometry_msgs::msg::Pose & pose,
                                 double extra_dilation_m,
                                 double forward_len_m,
                                 nav2_costmap_2d::Costmap2D * grid,
                                 unsigned char cost)
{
  // 1) 로컬 다각형 변형
  auto poly = dilateFootprintDirectional(fp.polygon.points, extra_dilation_m, forward_len_m);

  // 2) 로컬→월드
  const double yaw = tf2::getYaw(pose.orientation);
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (auto & p : poly) {
    const double x = p.x, y = p.y;
    p.x = pose.position.x + c * x - s * y;
    p.y = pose.position.y + s * x + c * y;
  }

  // 3) bbox
  double minx=1e9, miny=1e9, maxx=-1e9, maxy=-1e9;
  for (auto & p : poly) {
    minx = std::min(minx, p.x);
    miny = std::min(miny, p.y);
    maxx = std::max(maxx, p.x);
    maxy = std::max(maxy, p.y);
  }

  // 4) bbox → map index
  int min_i, min_j, max_i, max_j;
  grid->worldToMapEnforceBounds(minx, miny, min_i, min_j);
  grid->worldToMapEnforceBounds(maxx, maxy, max_i, max_j);

  // 5) 래스터 채우기 → 내부맵에 setCost(max-merge는 updateCosts에서 수행)
  for (int j = min_j; j <= max_j; ++j) {
    for (int i = min_i; i <= max_i; ++i) {
      double wx, wy; grid->mapToWorld(i, j, wx, wy);

      bool inside = false;
      const size_t n = poly.size();
      for (size_t k=0, h=n-1; k<n; h=k++) {
        const double xi = poly[k].x, yi = poly[k].y;
        const double xh = poly[h].x, yh = poly[h].y;
        const bool hit = ((yi > wy) != (yh > wy)) &&
                         (wx < (xh - xi) * (wy - yi) / std::max(1e-12, (yh - yi)) + xi);
        if (hit) inside = !inside;
      }

      if (inside) {
        const unsigned char old = grid->getCost(i, j);
        if (cost > old) {
          grid->setCost(static_cast<unsigned int>(i),
                        static_cast<unsigned int>(j), cost);
        }
      }
    }
  }

  // 6) bounds bookkeeping
  if (!touched_) {
    touch_min_x_ = minx; touch_min_y_ = miny;
    touch_max_x_ = maxx; touch_max_y_ = maxy;
    touched_ = true;
  } else {
    touch_min_x_ = std::min(touch_min_x_, minx);
    touch_min_y_ = std::min(touch_min_y_, miny);
    touch_max_x_ = std::max(touch_max_x_, maxx);
    touch_max_y_ = std::max(touch_max_y_, maxy);
  }
}

void AgentLayer::rasterizeAgentIntoInternal(const multi_agent_msgs::msg::MultiAgentInfo & a)
{
  if (!internal_grid_) return;

  const unsigned char cost_now = computeCost(a);
  const double iso_extra = computeDilation(a);
  const double forward_len = isMovingPhase(a.status.phase) ? forward_smear_m_ : 0.0;

  // 1) 현재 위치 footprint (전방 스미어는 이동중일 때만)
  fillFootprintAt(a.footprint, a.current_pose.pose, iso_extra, forward_len,
                  internal_grid_.get(), cost_now);

  // 2) 옵션: 경로 찍기 (이동 중일 때만, 얇게, 스미어X)
  if (paint_path_ && isMovingPhase(a.status.phase)) {
    const int limit = std::min<int>(a.truncated_path.poses.size(), max_poses_);
    if (limit > 0 && path_stride_m_ > 1e-3) {
      double acc = 0.0;
      auto prev = a.truncated_path.poses[0].pose.position;
      // 첫 포즈는 생략하고 stride로 샘플
      for (int i = 1; i < limit; ++i) {
        const auto & cur = a.truncated_path.poses[i].pose.position;
        acc += std::hypot(cur.x - prev.x, cur.y - prev.y);
        if (acc >= path_stride_m_) {
          geometry_msgs::msg::Pose pose_i = a.truncated_path.poses[i].pose;
          // 경로 상은 등방성 소량만, 전방 스미어 0
          fillFootprintAt(a.footprint, pose_i, path_dilation_m_, 0.0,
                          internal_grid_.get(), path_cost_);
          acc = 0.0;
        }
        prev = cur;
      }
    }
  }
}

void AgentLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                             int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) return;

  // 내부 버퍼 사이즈 동기화 (안전)
  matchSize();
  if (!internal_grid_) return;

  // 내부맵 클리어
  clearInternal();

  // 최신 데이터 스냅샷
  std::vector<multi_agent_msgs::msg::MultiAgentInfo> infos;
  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    if (!last_infos_ || stale(last_stamp_)) return;
    infos.assign(last_infos_->agents.begin(), last_infos_->agents.end());
  }

  // 내부맵에 에이전트만 래스터라이즈
  for (const auto & a : infos) {
    if (isSelf(a)) continue;

    // ROI는 updateBounds에서 이미 좁혀졌지만, 여기서도 가볍게 거리 조건 체크
    // (master의 원점/ROI 계산은 updateBounds 참고)
    rasterizeAgentIntoInternal(a);
  }

  // === Max-merge (updateWithMax) ===
  // master의 제공한 업데이트 윈도우 [min_i..max_i, min_j..max_j] 범위에서만 비교/갱신
  min_i = std::max(min_i, 0);
  min_j = std::max(min_j, 0);
  max_i = std::min(max_i, static_cast<int>(master_grid.getSizeInCellsX()) - 1);
  max_j = std::min(max_j, static_cast<int>(master_grid.getSizeInCellsY()) - 1);

  for (int j = min_j; j <= max_j; ++j) {
    for (int i = min_i; i <= max_i; ++i) {
      const unsigned char a = internal_grid_->getCost(i, j);
      if (a == nav2_costmap_2d::FREE_SPACE) continue; // 내부맵이 빈 곳이면 skip
      const unsigned char m = master_grid.getCost(i, j);
      if (a > m) {
        master_grid.setCost(static_cast<unsigned int>(i),
                            static_cast<unsigned int>(j), a);
      }
    }
  }
}

} // namespace multi_agent_nav2

// pluginlib export
PLUGINLIB_EXPORT_CLASS(multi_agent_nav2::AgentLayer, nav2_costmap_2d::Layer)
```

---

## 메모 & 튜닝 포인트

* 내부 버퍼 클리어는 **매 `updateCosts()`** 호출마다 수행 → 누적(고스트) 코스트가 남지 않음
* **Max-merge**는 master가 가진 장애물/인플레이션보다 **강한(더 큰) 코스트만** 덮어씀
* `forward_smear_m`는 **이동 중일 때만** 전방(+x)으로 footprint를 연장. 정지 상태에서는 등방성(`dilation_m`)만 적용
* `paint_path=true`로 켜면, 이동 중일 때 **truncated_path**를 얇게 칠함(스미어X, 소량 dilation).
  너무 과도하면 corridor가 과하게 막힐 수 있으니 `path_stride_m(0.2~0.5)`, `path_dilation_m(0.01~0.05)`, `path_cost(~160)` 정도부터 시작
* 프레임 검사: `use_path_header_frame=true`면 `last_infos_`와 `truncated_path.header.frame_id`가 `layered_costmap_->getGlobalFrameID()`와 같아야 반영함

---

필요하면 이 버전에 맞춘 **nav2_params.yaml** 샘플도 다시 정리해줄게.  
#################################  
###################################  
####################################  
좋아! 아래는 **global_costmap**에 AgentLayer를 붙여서 쓰는 **Nav2 `nav2_params.yaml` 샘플**이야.
Jazzy 기준이며, 최신 Nav2 스타일(plugins 배열 + 각 플러그인 블록의 `plugin:` 키에 C++ 타입 표기 `::`)을 사용했어.
주요 파라미터는 너랑 맞춘 AgentLayer 코드의 이름과 1:1로 대응돼.

---

# 기본형: 하나의 글로벌 코스트맵(마스터) + AgentLayer 포함

```yaml
# nav2_params.yaml (발췌)
global_costmap:
  global_frame: map
  robot_base_frame: base_link
  use_sim_time: false

  # 주기
  update_frequency: 5.0
  publish_frequency: 2.0

  # 맵/윈도 설정
  rolling_window: false         # 글로벌은 보통 false(정적 지도 기준)
  width: 100.0                  # 필요 크기
  height: 100.0
  resolution: 0.05
  track_unknown_space: true
  always_send_full_costmap: true

  # 레이어 적용 순서 중요!
  plugins: ["static_layer", "obstacle_layer", "agent_layer", "inflation_layer"]

  static_layer:
    plugin: "nav2_costmap_2d::StaticLayer"
    map_subscribe_transient_local: true
    subscribe_to_updates: true
    enabled: true

  obstacle_layer:
    plugin: "nav2_costmap_2d::ObstacleLayer"
    enabled: true
    observation_sources: scan
    scan:
      topic: /scan
      max_obstacle_height: 2.0
      clearing: true
      marking: true
      data_type: LaserScan
      expected_update_rate: 0.0
      observation_persistence: 0.0
      inf_is_valid: true
    footprint_clearing_enabled: true

  # === 여기부터 사용자 커스텀 AgentLayer ===
  agent_layer:
    plugin: "multi_agent_nav2::AgentLayer"
    enabled: true

    # 구독 토픽 (다중로봇 상태)
    topic: "/multi_agent_infos"

    # 자기 자신 필터링
    self_machine_id: 0          # 내 머신 ID
    self_type_id: ""            # 내 타입 ID (문자열로 맞춤)

    # 프레임/ROI/신선도
    use_path_header_frame: true
    roi_range_m: 12.0
    freshness_timeout_ms: 800
    max_poses: 40
    qos_reliable: true

    # 코스트 정책
    lethal_cost: 254
    moving_cost: 180
    waiting_cost: 200
    manual_cost_bias: 30

    # 지오메트리(등방성 팽창 + 전방 스미어)
    dilation_m: 0.05            # 모든 방향 기본 여유
    forward_smear_m: 0.25       # 이동중 전방(+x) 여유
    sigma_k: 2.0                # pos_std_m 가중

    # (옵션) 이동중 경로도 얇게 칠하기
    paint_path: false           # 필요 시 true
    path_stride_m: 0.30
    path_dilation_m: 0.02
    path_cost: 160

  inflation_layer:
    plugin: "nav2_costmap_2d::InflationLayer"
    enabled: true
    cost_scaling_factor: 3.0
    inflation_radius: 0.60
```

### 포인트

* `plugins` 순서: `static → obstacle → agent → inflation`.
  AgentLayer가 그려준 값도 최종적으로 inflation 영향을 받게 하려면 **inflation을 마지막**에 두면 좋아.
* AgentLayer는 내부 버퍼를 매 사이클 **클리어 → 에이전트만 페인팅 → master와 Max-merge** 하므로, 고스트가 남지 않아.
* `paint_path: true` 로 바꾸면 **이동중 에이전트의 truncated_path**를 얇게 칠해서 선점로(협소 구간) 회피를 유도할 수 있어. 처음엔 `false`로 두고 필요 시 켜봐.

---

# (선택) Agent 전용 마스크 코스트맵 서버용 블록

`/path_validator_node`가 **에이전트만 반영된 마스크(costmap_raw)** 를 구독해야 한다면,
아예 **AgentLayer만 로드한 별도의 글로벌 코스트맵 서버**를 런치에서 하나 더 띄우는 게 가장 깔끔해.
아래 블록은 그런 **두 번째 CostmapServer**를 위해 준비된 파라미터 섹션이야. (런치에서 해당 섹션을 이 서버에 바인딩해야 함)

```yaml
# (선택) agent 전용 마스크용 글로벌 코스트맵
global_costmap_agent_only:
  global_frame: map
  robot_base_frame: base_link
  use_sim_time: false
  update_frequency: 5.0
  publish_frequency: 2.0

  rolling_window: false
  width: 100.0
  height: 100.0
  resolution: 0.05
  track_unknown_space: true
  always_send_full_costmap: true

  # 오직 agent_layer만
  plugins: ["agent_layer"]

  agent_layer:
    plugin: "multi_agent_nav2::AgentLayer"
    enabled: true
    topic: "/multi_agent_infos"
    self_machine_id: 0
    self_type_id: ""
    use_path_header_frame: true
    roi_range_m: 12.0
    freshness_timeout_ms: 800
    max_poses: 40
    qos_reliable: true

    lethal_cost: 254
    moving_cost: 180
    waiting_cost: 200
    manual_cost_bias: 30

    dilation_m: 0.05
    forward_smear_m: 0.25
    sigma_k: 2.0

    paint_path: false
    path_stride_m: 0.30
    path_dilation_m: 0.02
    path_cost: 160
```

* 이 두 번째 서버가 퍼블리시하는 토픽을 런치에서 `remap`해서 **`/agent_layer/costmap_raw`**로 내보내면,
  `path_validator_node`의 `agent_mask_topic`(기본 `/agent_layer/costmap_raw`)와 그대로 맞물린다.
* 런치 예시(개념):

  ```
  Node(
    package='nav2_costmap_2d',
    executable='costmap_2d_server',
    name='global_costmap_agent_only',
    namespace='',
    parameters=[ nav2_params.yaml, {'use_sim_time': False} ],
    remappings=[ ('/global_costmap/costmap_raw', '/agent_layer/costmap_raw') ]
  )
  ```

  (실제 네 프로젝트 런치 구조에 맞게 조정해줘)

---

## 튜닝 가이드(요약)

* **agent_layer.dilation_m**: 정지 로봇 발자국 주변 여유. 0.03~0.08부터.
* **agent_layer.forward_smear_m**: **이동중인 로봇 전방(+x)**만 늘여서 여유 확보. 0.2~0.4 권장.
* **agent_layer.paint_path**: 협소 구간 선점 회피가 필요할 때만 `true`. 너무 크게 잡으면 불필요한 차단 발생.
* **path_stride_m / path_dilation_m / path_cost**: 경로 “선”의 두께와 강도.
  먼저 `0.3 / 0.02 / 160`에서 시작 → 필요 시 stride↓(더 조밀), dilation↑(두꺼움), cost↑(우선순위 강화).
* **plugins 순서**: AgentLayer 출력까지 포함해 마지막에 **inflation**을 적용하려면
  `["static", "obstacle", "agent", "inflation"]` 권장.

필요하면 위 샘플을 네 환경(맵 크기, 해상도, 프레임, 센서 토픽)으로 살짝만 바꿔서 바로 쓸 수 있어.

##################  
####################  
좋아! 아래는 요청하신 **AgentLayer 전체 코드(hpp, cpp)**입니다.
핵심은 내부 버퍼(`internal_grid_`)에 에이전트만 렌더링하고, 머지할 때 **unknown(255)은 덮어쓰고**, 나머지는 **Max-merge**로 결합하도록 고쳤습니다. 또한 `overwrite_unknown` 파라미터로 unknown 덮어쓰기 여부를 제어할 수 있게 했습니다.

---

### `agent_layer.hpp`

```cpp
#pragma once

#include <mutex>
#include <vector>
#include <string>
#include <utility>
#include <algorithm>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <multi_agent_msgs/msg/multi_agent_info_array.hpp>
#include <multi_agent_msgs/msg/agent_status.hpp>

namespace multi_agent_nav2
{

class AgentLayer : public nav2_costmap_2d::Layer
{
public:
  AgentLayer();

  // lifecycle
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override { current_ = true; }

  // costmap callbacks
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;

  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;

  void matchSize() override;  // master 크기 변경 시 내부맵 동기화
  bool isClearable() override { return true; }

private:
  // node
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_shared_;

  // I/O
  rclcpp::Subscription<multi_agent_msgs::msg::MultiAgentInfoArray>::SharedPtr sub_;

  // last data
  std::mutex data_mtx_;
  multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr last_infos_;
  rclcpp::Time last_stamp_;

  // parameters
  bool        enabled_{true};
  std::string topic_{"/multi_agent_infos"};
  uint16_t    self_machine_id_{0};
  std::string self_type_id_{};
  bool        use_path_header_frame_{true};
  double      roi_range_m_{12.0};
  int         freshness_timeout_ms_{800};
  int         max_poses_{40};
  bool        qos_reliable_{true};

  // cost policy
  unsigned char lethal_cost_{254};
  unsigned char moving_cost_{180};
  unsigned char waiting_cost_{200};
  int           manual_cost_bias_{30};

  // geometry (등방성 dilation + 전방(+x) smear 분리)
  double      dilation_m_{0.05};
  double      forward_smear_m_{0.25};
  double      sigma_k_{2.0};     // pos_std_m 가중치

  // path 페인팅 옵션(이동중일 때만)
  bool        paint_path_{false};
  double      path_stride_m_{0.30};
  double      path_dilation_m_{0.02};
  unsigned char path_cost_{160};

  // 병합 정책
  bool        overwrite_unknown_{true};  // master가 255면 agent로 덮어쓰기

  // bounds cache for this cycle
  double touch_min_x_{0.0}, touch_min_y_{0.0}, touch_max_x_{0.0}, touch_max_y_{0.0};
  bool   touched_{false};

  // 내부 costmap 버퍼
  std::unique_ptr<nav2_costmap_2d::Costmap2D> internal_grid_;
  void clearInternal(); // FREE_SPACE로 클리어

  // helpers
  void infosCallback(const multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr msg);
  bool stale(const rclcpp::Time & stamp) const;
  bool isSelf(const multi_agent_msgs::msg::MultiAgentInfo & a) const;

  unsigned char computeCost(const multi_agent_msgs::msg::MultiAgentInfo & a) const;
  double computeDilation(const multi_agent_msgs::msg::MultiAgentInfo & a) const;

  static inline bool isMovingPhase(uint8_t phase);

  // rasterizers
  void rasterizeAgentIntoInternal(const multi_agent_msgs::msg::MultiAgentInfo & a);
  void fillFootprintAt(const geometry_msgs::msg::PolygonStamped & fp,
                       const geometry_msgs::msg::Pose & pose,
                       double extra_dilation_m,
                       double forward_len_m, // 전방(+x)로만 스미어 길이
                       nav2_costmap_2d::Costmap2D * grid,
                       unsigned char cost);

  // directional footprint dilation in local frame
  static std::vector<geometry_msgs::msg::Point>
  dilateFootprintDirectional(const std::vector<geometry_msgs::msg::Point32> & in,
                             double iso_dilate_m, double forward_len_m);

  // util
  static bool pointInPolygon(const std::vector<geometry_msgs::msg::Point> & poly,
                             double x, double y);
};

inline bool AgentLayer::isMovingPhase(uint8_t phase)
{
  using S = multi_agent_msgs::msg::AgentStatus;
  return phase == S::STATUS_MOVING || phase == S::STATUS_PATH_SEARCHING;
}

} // namespace multi_agent_nav2
```

---

### `agent_layer.cpp`

```cpp
#include "multi_agent_nav2/agent_layer.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <cstring>  // std::memset

namespace multi_agent_nav2
{

AgentLayer::AgentLayer() {}

void AgentLayer::onInitialize()
{
  node_shared_ = node_.lock();
  if (!node_shared_) {
    throw std::runtime_error("AgentLayer: failed to lock lifecycle node");
  }

  // ---- declare ----
  node_shared_->declare_parameter("enabled", rclcpp::ParameterValue(true));
  node_shared_->declare_parameter("topic", rclcpp::ParameterValue(std::string("/multi_agent_infos")));
  node_shared_->declare_parameter("self_machine_id", rclcpp::ParameterValue(0));
  node_shared_->declare_parameter("self_type_id", rclcpp::ParameterValue(std::string("")));
  node_shared_->declare_parameter("use_path_header_frame", rclcpp::ParameterValue(true));
  node_shared_->declare_parameter("roi_range_m", rclcpp::ParameterValue(12.0));
  node_shared_->declare_parameter("freshness_timeout_ms", rclcpp::ParameterValue(800));
  node_shared_->declare_parameter("max_poses", rclcpp::ParameterValue(40));
  node_shared_->declare_parameter("qos_reliable", rclcpp::ParameterValue(true));

  node_shared_->declare_parameter("lethal_cost", rclcpp::ParameterValue(254));
  node_shared_->declare_parameter("moving_cost", rclcpp::ParameterValue(180));
  node_shared_->declare_parameter("waiting_cost", rclcpp::ParameterValue(200));
  node_shared_->declare_parameter("manual_cost_bias", rclcpp::ParameterValue(30));

  node_shared_->declare_parameter("dilation_m", rclcpp::ParameterValue(0.05));
  node_shared_->declare_parameter("forward_smear_m", rclcpp::ParameterValue(0.25));
  node_shared_->declare_parameter("sigma_k", rclcpp::ParameterValue(2.0));

  node_shared_->declare_parameter("paint_path", rclcpp::ParameterValue(false));
  node_shared_->declare_parameter("path_stride_m", rclcpp::ParameterValue(0.30));
  node_shared_->declare_parameter("path_dilation_m", rclcpp::ParameterValue(0.02));
  node_shared_->declare_parameter("path_cost", rclcpp::ParameterValue(160));

  node_shared_->declare_parameter("overwrite_unknown", rclcpp::ParameterValue(true));

  // ---- get ----
  node_shared_->get_parameter("enabled", enabled_);
  node_shared_->get_parameter("topic", topic_);
  {
    int tmp = 0;
    node_shared_->get_parameter("self_machine_id", tmp);
    self_machine_id_ = static_cast<uint16_t>(tmp);
  }
  node_shared_->get_parameter("self_type_id", self_type_id_);
  node_shared_->get_parameter("use_path_header_frame", use_path_header_frame_);
  node_shared_->get_parameter("roi_range_m", roi_range_m_);
  node_shared_->get_parameter("freshness_timeout_ms", freshness_timeout_ms_);
  node_shared_->get_parameter("max_poses", max_poses_);
  node_shared_->get_parameter("qos_reliable", qos_reliable_);

  {
    int v=254; node_shared_->get_parameter("lethal_cost", v);
    lethal_cost_ = static_cast<unsigned char>(std::clamp(v, 0, 254));
  }
  {
    int v=180; node_shared_->get_parameter("moving_cost", v);
    moving_cost_ = static_cast<unsigned char>(std::clamp(v, 0, 254));
  }
  {
    int v=200; node_shared_->get_parameter("waiting_cost", v);
    waiting_cost_ = static_cast<unsigned char>(std::clamp(v, 0, 254));
  }
  node_shared_->get_parameter("manual_cost_bias", manual_cost_bias_);

  node_shared_->get_parameter("dilation_m", dilation_m_);
  node_shared_->get_parameter("forward_smear_m", forward_smear_m_);
  node_shared_->get_parameter("sigma_k", sigma_k_);

  node_shared_->get_parameter("paint_path", paint_path_);
  node_shared_->get_parameter("path_stride_m", path_stride_m_);
  node_shared_->get_parameter("path_dilation_m", path_dilation_m_);
  {
    int v=160; node_shared_->get_parameter("path_cost", v);
    path_cost_ = static_cast<unsigned char>(std::clamp(v, 0, 254));
  }

  node_shared_->get_parameter("overwrite_unknown", overwrite_unknown_);

  current_ = true;
  matchSize();

  if (enabled_) activate();
}

void AgentLayer::matchSize()
{
  // master 크기와 동일하게 내부 버퍼 준비
  auto & master = layered_costmap_->getCostmap();
  if (!internal_grid_ ||
      internal_grid_->getSizeInCellsX() != master.getSizeInCellsX() ||
      internal_grid_->getSizeInCellsY() != master.getSizeInCellsY() ||
      internal_grid_->getResolution()   != master.getResolution() ||
      internal_grid_->getOriginX()      != master.getOriginX() ||
      internal_grid_->getOriginY()      != master.getOriginY())
  {
    internal_grid_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
        master.getSizeInCellsX(),
        master.getSizeInCellsY(),
        master.getResolution(),
        master.getOriginX(),
        master.getOriginY(),
        nav2_costmap_2d::FREE_SPACE);
  }
}

void AgentLayer::activate()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  if (qos_reliable_) qos.reliable(); else qos.best_effort();

  sub_ = node_shared_->create_subscription<multi_agent_msgs::msg::MultiAgentInfoArray>(
      topic_, qos, std::bind(&AgentLayer::infosCallback, this, std::placeholders::_1));
}

void AgentLayer::deactivate()
{
  sub_.reset();
}

void AgentLayer::infosCallback(
  const multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(data_mtx_);
  last_infos_ = msg;
  last_stamp_ = msg->header.stamp;
}

bool AgentLayer::stale(const rclcpp::Time & stamp) const
{
  return (node_shared_->now() - stamp) >
         rclcpp::Duration::from_nanoseconds(
           static_cast<int64_t>(freshness_timeout_ms_) * 1000000LL);
}

bool AgentLayer::isSelf(const multi_agent_msgs::msg::MultiAgentInfo & a) const
{
  return (a.machine_id == self_machine_id_) && (a.type_id == self_type_id_);
}

unsigned char AgentLayer::computeCost(const multi_agent_msgs::msg::MultiAgentInfo & a) const
{
  using S = multi_agent_msgs::msg::AgentStatus;
  const bool moving = (a.status.phase == S::STATUS_MOVING) ||
                      (a.status.phase == S::STATUS_PATH_SEARCHING);
  unsigned char base = moving ? moving_cost_ : waiting_cost_;
  if (a.mode == "manual") {
    int c = static_cast<int>(base) + manual_cost_bias_;
    return static_cast<unsigned char>(std::clamp(c, 0, 254));
  }
  return base;
}

double AgentLayer::computeDilation(const multi_agent_msgs::msg::MultiAgentInfo & a) const
{
  double r = dilation_m_;
  if (a.pos_std_m >= 0.0) r += sigma_k_ * a.pos_std_m;
  return r;
}

void AgentLayer::clearInternal()
{
  if (!internal_grid_) return;
  unsigned char * map = internal_grid_->getCharMap();
  const size_t N = static_cast<size_t>(internal_grid_->getSizeInCellsX()) *
                   static_cast<size_t>(internal_grid_->getSizeInCellsY());
  std::memset(map, nav2_costmap_2d::FREE_SPACE, N * sizeof(unsigned char));
}

void AgentLayer::updateBounds(double robot_x, double robot_y, double /*robot_yaw*/,
                              double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_) return;

  touched_ = false;
  touch_min_x_ =  1e9; touch_min_y_ =  1e9;
  touch_max_x_ = -1e9; touch_max_y_ = -1e9;

  std::vector<multi_agent_msgs::msg::MultiAgentInfo> infos;
  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    if (!last_infos_ || stale(last_stamp_)) return;
    infos.assign(last_infos_->agents.begin(), last_infos_->agents.end());
  }

  const std::string & global_frame = layered_costmap_->getGlobalFrameID();

  for (const auto & a : infos) {
    if (isSelf(a)) continue;

    // ROI by distance from our robot
    const double dx = a.current_pose.pose.position.x - robot_x;
    const double dy = a.current_pose.pose.position.y - robot_y;
    if (std::hypot(dx, dy) > roi_range_m_) continue;

    // frame check (optional)
    if (use_path_header_frame_) {
      if (!a.truncated_path.header.frame_id.empty() &&
          a.truncated_path.header.frame_id != global_frame) {
        continue;
      }
      if (!last_infos_->header.frame_id.empty() &&
          last_infos_->header.frame_id != global_frame) {
        continue;
      }
    }

    // 현재 위치
    {
      const auto & p = a.current_pose.pose.position;
      touch_min_x_ = std::min(touch_min_x_, p.x);
      touch_min_y_ = std::min(touch_min_y_, p.y);
      touch_max_x_ = std::max(touch_max_x_, p.x);
      touch_max_y_ = std::max(touch_max_y_, p.y);
      touched_ = true;
    }

    // 옵션: path bounds (페인트 옵션을 켠 경우에만)
    if (paint_path_ && isMovingPhase(a.status.phase)) {
      const int limit = std::min<int>(a.truncated_path.poses.size(), max_poses_);
      for (int i = 0; i < limit; ++i) {
        const auto & p = a.truncated_path.poses[i].pose.position;
        touch_min_x_ = std::min(touch_min_x_, p.x);
        touch_min_y_ = std::min(touch_min_y_, p.y);
        touch_max_x_ = std::max(touch_max_x_, p.x);
        touch_max_y_ = std::max(touch_max_y_, p.y);
        touched_ = true;
      }
    }
  }

  if (touched_) {
    *min_x = std::min(*min_x, touch_min_x_);
    *min_y = std::min(*min_y, touch_min_y_);
    *max_x = std::max(*max_x, touch_max_x_);
    *max_y = std::max(*max_y, touch_max_y_);
  }
}

// === 로컬 footprint 등방성+전방(+x) 보정 ===
std::vector<geometry_msgs::msg::Point>
AgentLayer::dilateFootprintDirectional(const std::vector<geometry_msgs::msg::Point32> & in,
                                       double iso_dilate_m, double forward_len_m)
{
  std::vector<geometry_msgs::msg::Point> out; out.reserve(in.size());
  if (in.empty()) return out;

  double cx=0, cy=0;
  for (auto & p : in) { cx += p.x; cy += p.y; }
  cx /= static_cast<double>(in.size());
  cy /= static_cast<double>(in.size());

  for (auto & p : in) {
    double vx = p.x - cx, vy = p.y - cy;
    double n = std::hypot(vx, vy); if (n < 1e-6) n = 1.0;

    double x_local = p.x + iso_dilate_m * (vx / n);
    double y_local = p.y + iso_dilate_m * (vy / n);

    if (forward_len_m > 1e-6 && (p.x - cx) >= 0.0) {
      x_local += forward_len_m; // 전방(+x)만
    }

    geometry_msgs::msg::Point q;
    q.x = x_local; q.y = y_local; q.z = 0.0;
    out.push_back(q);
  }
  return out;
}

bool AgentLayer::pointInPolygon(const std::vector<geometry_msgs::msg::Point> & poly,
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

void AgentLayer::fillFootprintAt(const geometry_msgs::msg::PolygonStamped & fp,
                                 const geometry_msgs::msg::Pose & pose,
                                 double extra_dilation_m,
                                 double forward_len_m,
                                 nav2_costmap_2d::Costmap2D * grid,
                                 unsigned char cost)
{
  // 1) 로컬 다각형 변형
  auto poly = dilateFootprintDirectional(fp.polygon.points, extra_dilation_m, forward_len_m);

  // 2) 로컬→월드
  const double yaw = tf2::getYaw(pose.orientation);
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (auto & p : poly) {
    const double x = p.x, y = p.y;
    p.x = pose.position.x + c * x - s * y;
    p.y = pose.position.y + s * x + c * y;
  }

  // 3) bbox
  double minx=1e9, miny=1e9, maxx=-1e9, maxy=-1e9;
  for (auto & p : poly) {
    minx = std::min(minx, p.x);
    miny = std::min(miny, p.y);
    maxx = std::max(maxx, p.x);
    maxy = std::max(maxy, p.y);
  }

  // 4) bbox → map index
  int min_i, min_j, max_i, max_j;
  grid->worldToMapEnforceBounds(minx, miny, min_i, min_j);
  grid->worldToMapEnforceBounds(maxx, maxy, max_i, max_j);

  // 5) 내부맵에 setCost (Max-merge는 updateCosts에서 수행)
  for (int j = min_j; j <= max_j; ++j) {
    for (int i = min_i; i <= max_i; ++i) {
      double wx, wy; grid->mapToWorld(i, j, wx, wy);

      bool inside = false;
      const size_t n = poly.size();
      for (size_t k=0, h=n-1; k<n; h=k++) {
        const double xi = poly[k].x, yi = poly[k].y;
        const double xh = poly[h].x, yh = poly[h].y;
        const bool hit = ((yi > wy) != (yh > wy)) &&
                         (wx < (xh - xi) * (wy - yi) / std::max(1e-12, (yh - yi)) + xi);
        if (hit) inside = !inside;
      }

      if (inside) {
        const unsigned char old = grid->getCost(i, j);
        if (cost > old) {
          grid->setCost(static_cast<unsigned int>(i),
                        static_cast<unsigned int>(j), cost);
        }
      }
    }
  }

  // 6) bounds bookkeeping
  if (!touched_) {
    touch_min_x_ = minx; touch_min_y_ = miny;
    touch_max_x_ = maxx; touch_max_y_ = maxy;
    touched_ = true;
  } else {
    touch_min_x_ = std::min(touch_min_x_, minx);
    touch_min_y_ = std::min(touch_min_y_, miny);
    touch_max_x_ = std::max(touch_max_x_, maxx);
    touch_max_y_ = std::max(touch_max_y_, maxy);
  }
}

void AgentLayer::rasterizeAgentIntoInternal(const multi_agent_msgs::msg::MultiAgentInfo & a)
{
  if (!internal_grid_) return;

  const unsigned char cost_now = computeCost(a);
  const double iso_extra = computeDilation(a);
  const double forward_len = isMovingPhase(a.status.phase) ? forward_smear_m_ : 0.0;

  // 1) 현재 위치 footprint (전방 스미어는 이동중일 때만)
  fillFootprintAt(a.footprint, a.current_pose.pose, iso_extra, forward_len,
                  internal_grid_.get(), cost_now);

  // 2) 옵션: truncated path 찍기 (이동 중, 얇게, 스미어X)
  if (paint_path_ && isMovingPhase(a.status.phase)) {
    const int limit = std::min<int>(a.truncated_path.poses.size(), max_poses_);
    if (limit > 0 && path_stride_m_ > 1e-3) {
      double acc = 0.0;
      auto prev = a.truncated_path.poses[0].pose.position;
      for (int i = 1; i < limit; ++i) {
        const auto & cur = a.truncated_path.poses[i].pose.position;
        acc += std::hypot(cur.x - prev.x, cur.y - prev.y);
        if (acc >= path_stride_m_) {
          geometry_msgs::msg::Pose pose_i = a.truncated_path.poses[i].pose;
          fillFootprintAt(a.footprint, pose_i, path_dilation_m_, 0.0,
                          internal_grid_.get(), path_cost_);
          acc = 0.0;
        }
        prev = cur;
      }
    }
  }
}

void AgentLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                             int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) return;

  // 내부 버퍼 사이즈 동기화 (안전)
  matchSize();
  if (!internal_grid_) return;

  // 내부맵 클리어
  clearInternal();

  // 최신 데이터 스냅샷
  std::vector<multi_agent_msgs::msg::MultiAgentInfo> infos;
  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    if (!last_infos_ || stale(last_stamp_)) return;
    infos.assign(last_infos_->agents.begin(), last_infos_->agents.end());
  }

  // 내부맵에 에이전트만 래스터라이즈
  for (const auto & a : infos) {
    if (isSelf(a)) continue;
    rasterizeAgentIntoInternal(a);
  }

  // === 병합: unknown 덮어쓰기 + Max-merge ===
  min_i = std::max(min_i, 0);
  min_j = std::max(min_j, 0);
  max_i = std::min(max_i, static_cast<int>(master_grid.getSizeInCellsX()) - 1);
  max_j = std::min(max_j, static_cast<int>(master_grid.getSizeInCellsY()) - 1);

  for (int j = min_j; j <= max_j; ++j) {
    for (int i = min_i; i <= max_i; ++i) {
      const unsigned char a = internal_grid_->getCost(i, j);
      if (a == nav2_costmap_2d::FREE_SPACE) continue; // 내부맵이 빈 곳이면 skip

      const unsigned char m = master_grid.getCost(i, j);

      if (m == nav2_costmap_2d::NO_INFORMATION) {
        // unknown은 덮어써서 보이게 (옵션)
        if (overwrite_unknown_) {
          master_grid.setCost(static_cast<unsigned int>(i),
                              static_cast<unsigned int>(j), a);
        }
        // overwrite_unknown_이 false면 unknown은 유지
      } else if (a > m) {
        // 일반 셀은 Max-merge
        master_grid.setCost(static_cast<unsigned int>(i),
                            static_cast<unsigned int>(j), a);
      }
    }
  }
}

} // namespace multi_agent_nav2

// pluginlib export
PLUGINLIB_EXPORT_CLASS(multi_agent_nav2::AgentLayer, nav2_costmap_2d::Layer)
```

---

### 주요 포인트 요약

* 내부 버퍼(`internal_grid_`)에만 에이전트 도형/경로를 래스터라이즈 → `updateCosts()`에서 마스터와 병합.
* 병합 정책:

  * 마스터가 **NO_INFORMATION(255)** 이면 `overwrite_unknown`이 `true`일 때 **무조건 덮어쓰기**.
  * 그 외에는 **Max-merge**로 `a > m`일 때만 갱신.
* 전방 스미어는 로컬 좌표계 **+x 방향**으로만 적용(이동 중일 때만).
* `paint_path`가 `true`일 때만 truncated path를 얇고 성긴 스트라이드로 찍음(과도한 차단 방지).

원하시면 `yaml` 샘플도 이 코드에 맞춰 다시 드릴게요.
