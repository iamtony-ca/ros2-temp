좋아요—요청하신 “전방(+x)으로만 스미어” 적용과 `computeDilation()`에서 등방성만 반환하도록 바꾼 **완전한 hpp/cpp**를 드립니다.
변경/추가된 핵심 지점엔 `// [CHANGED]`, `// [NEW]` 주석을 달아두었습니다.

---

### `include/multi_agent_nav2/agent_layer.hpp`

```cpp
#pragma once

#include <mutex>
#include <vector>
#include <string>
#include <utility>
#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <multi_agent_msgs/msg/multi_agent_info_array.hpp>
#include <multi_agent_msgs/msg/agent_layer_meta_array.hpp>
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

  bool isClearable() override { return true; }

private:
  // node handle
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_shared_;

  // I/O
  rclcpp::Subscription<multi_agent_msgs::msg::MultiAgentInfoArray>::SharedPtr sub_;
  rclcpp::Publisher<multi_agent_msgs::msg::AgentLayerMetaArray>::SharedPtr meta_pub_;

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
  double      time_decay_sec_{1.0};
  unsigned char lethal_cost_{254};
  unsigned char moving_cost_{180};
  unsigned char waiting_cost_{200};
  int         manual_cost_bias_{30};

  // [CHANGED] 등방성 팽창과 전방 스미어를 분리
  double      dilation_m_{0.05};          // 등방성(모든 방향) 기본 여유
  double      forward_smear_m_{0.25};     // 이동 중일 때만 전방(+x)으로 늘릴 길이
  double      sigma_k_{2.0};              // pos_std_m 가중

  bool        publish_meta_{true};
  int         meta_stride_{3};
  int         freshness_timeout_ms_{800};
  int         max_poses_{40};
  bool        qos_reliable_{true};

  // bounds cache for this cycle
  double touch_min_x_{0.0}, touch_min_y_{0.0}, touch_max_x_{0.0}, touch_max_y_{0.0};
  bool   touched_{false};

  // helpers
  void infosCallback(const multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr msg);
  bool isSelf(const multi_agent_msgs::msg::MultiAgentInfo & a) const;
  bool stale(const rclcpp::Time & stamp) const;

  // [CHANGED] forward_len 인자를 추가
  void rasterizeAgentPath(const multi_agent_msgs::msg::MultiAgentInfo & a,
                          nav2_costmap_2d::Costmap2D * grid,
                          std::vector<std::pair<unsigned int,unsigned int>> & meta_hits);

  // [CHANGED] 전방 스미어 길이를 인자로 전달
  void fillFootprintAt(const geometry_msgs::msg::PolygonStamped & fp,
                       const geometry_msgs::msg::Pose & pose,
                       double extra_dilation_m,
                       double forward_len_m, // [NEW]
                       nav2_costmap_2d::Costmap2D * grid,
                       unsigned char cost,
                       std::vector<std::pair<unsigned int,unsigned int>> * meta_hits = nullptr);

  static bool pointInPolygon(const std::vector<geometry_msgs::msg::Point> & poly,
                             double x, double y);

  // [CHANGED] 등방성 팽창만 반환 (전방 스미어 제외)
  double computeDilation(const multi_agent_msgs::msg::MultiAgentInfo & a) const;

  unsigned char computeCost(const multi_agent_msgs::msg::MultiAgentInfo & a) const;

  // [NEW] 이동상태 판별 헬퍼
  static inline bool isMovingPhase(uint8_t phase)
  {
    using S = multi_agent_msgs::msg::AgentStatus;
    return phase == S::STATUS_MOVING || phase == S::STATUS_PATH_SEARCHING;
  }
};

} // namespace multi_agent_nav2
```

---

### `src/agent_layer.cpp`

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

  // Declare parameters (defaults)
  node_shared_->declare_parameter("enabled", rclcpp::ParameterValue(true));
  node_shared_->declare_parameter("topic", rclcpp::ParameterValue(std::string("/multi_agent_infos")));
  node_shared_->declare_parameter("self_machine_id", rclcpp::ParameterValue(0));
  node_shared_->declare_parameter("self_type_id", rclcpp::ParameterValue(std::string("")));
  node_shared_->declare_parameter("use_path_header_frame", rclcpp::ParameterValue(true));
  node_shared_->declare_parameter("roi_range_m", rclcpp::ParameterValue(12.0));
  node_shared_->declare_parameter("time_decay_sec", rclcpp::ParameterValue(1.0));
  node_shared_->declare_parameter("lethal_cost", rclcpp::ParameterValue(254));
  node_shared_->declare_parameter("moving_cost", rclcpp::ParameterValue(180));
  node_shared_->declare_parameter("waiting_cost", rclcpp::ParameterValue(200));
  node_shared_->declare_parameter("manual_cost_bias", rclcpp::ParameterValue(30));

  // [CHANGED] 전방 스미어와 등방성 팽창을 분리
  node_shared_->declare_parameter("dilation_m", rclcpp::ParameterValue(0.05));
  node_shared_->declare_parameter("forward_smear_m", rclcpp::ParameterValue(0.25));
  node_shared_->declare_parameter("sigma_k", rclcpp::ParameterValue(2.0));

  node_shared_->declare_parameter("publish_meta", rclcpp::ParameterValue(true));
  node_shared_->declare_parameter("meta_stride", rclcpp::ParameterValue(3));
  node_shared_->declare_parameter("freshness_timeout_ms", rclcpp::ParameterValue(800));
  node_shared_->declare_parameter("max_poses", rclcpp::ParameterValue(40));
  node_shared_->declare_parameter("qos_reliable", rclcpp::ParameterValue(true));

  // Get parameters
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
  node_shared_->get_parameter("time_decay_sec", time_decay_sec_);
  {
    int tmp = 254; node_shared_->get_parameter("lethal_cost", tmp);
    lethal_cost_ = static_cast<unsigned char>(std::clamp(tmp, 0, 254));
  }
  {
    int tmp = 180; node_shared_->get_parameter("moving_cost", tmp);
    moving_cost_ = static_cast<unsigned char>(std::clamp(tmp, 0, 254));
  }
  {
    int tmp = 200; node_shared_->get_parameter("waiting_cost", tmp);
    waiting_cost_ = static_cast<unsigned char>(std::clamp(tmp, 0, 254));
  }
  node_shared_->get_parameter("manual_cost_bias", manual_cost_bias_);

  // [CHANGED] 등방성만 dilation_m, 전방 스미어는 별도
  node_shared_->get_parameter("dilation_m", dilation_m_);
  node_shared_->get_parameter("forward_smear_m", forward_smear_m_);
  node_shared_->get_parameter("sigma_k", sigma_k_);

  node_shared_->get_parameter("publish_meta", publish_meta_);
  node_shared_->get_parameter("meta_stride", meta_stride_);
  node_shared_->get_parameter("freshness_timeout_ms", freshness_timeout_ms_);
  node_shared_->get_parameter("max_poses", max_poses_);
  node_shared_->get_parameter("qos_reliable", qos_reliable_);

  current_ = true;
  matchSize();

  if (enabled_) activate();
}

void AgentLayer::activate()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  if (qos_reliable_) qos.reliable(); else qos.best_effort();

  sub_ = node_shared_->create_subscription<multi_agent_msgs::msg::MultiAgentInfoArray>(
      topic_, qos, std::bind(&AgentLayer::infosCallback, this, std::placeholders::_1));

  if (publish_meta_) {
    meta_pub_ = node_shared_->create_publisher<multi_agent_msgs::msg::AgentLayerMetaArray>(
        "agent_layer_meta", rclcpp::QoS(1).reliable().transient_local());
  }
}

void AgentLayer::deactivate()
{
  sub_.reset();
  meta_pub_.reset();
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
  const uint8_t p = a.status.phase;
  const bool is_moving =
      (p == S::STATUS_MOVING) || (p == S::STATUS_PATH_SEARCHING);

  unsigned char base = is_moving ? moving_cost_ : waiting_cost_;

  if (a.mode == "manual") {
    int c = static_cast<int>(base) + manual_cost_bias_;
    return static_cast<unsigned char>(std::clamp(c, 0, 254));
  }
  return base;
}

// [CHANGED] 등방성(모든 방향) 팽창만 반환. 전방 스미어는 여기서 제외!
double AgentLayer::computeDilation(const multi_agent_msgs::msg::MultiAgentInfo & a) const
{
  double r = dilation_m_;
  if (a.pos_std_m >= 0.0) r += sigma_k_ * a.pos_std_m;
  return r;
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
    if (use_path_header_frame_ && a.truncated_path.header.frame_id != global_frame) {
      continue;
    }

    // 현재 위치 + 트렁케이트 경로를 모두 bounds에 반영
    {
      const auto & p = a.current_pose.pose.position;
      touch_min_x_ = std::min(touch_min_x_, p.x);
      touch_min_y_ = std::min(touch_min_y_, p.y);
      touch_max_x_ = std::max(touch_max_x_, p.x);
      touch_max_y_ = std::max(touch_max_y_, p.y);
      touched_ = true;
    }

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

  if (touched_) {
    *min_x = std::min(*min_x, touch_min_x_);
    *min_y = std::min(*min_y, touch_min_y_);
    *max_x = std::max(*max_x, touch_max_x_);
    *max_y = std::max(*max_y, touch_max_y_);
  }
}

// === 내부 헬퍼: 로컬 프레임에서 등방성 + 전방(+x) 스미어 적용 ===
// [NEW]
static inline std::vector<geometry_msgs::msg::Point>
dilateFootprintDirectional(const std::vector<geometry_msgs::msg::Point32> & in,
                           double iso_dilate_m,
                           double forward_len_m)
{
  std::vector<geometry_msgs::msg::Point> out; out.reserve(in.size());
  if (in.empty()) return out;

  // 로컬 폴리곤의 중심
  double cx=0, cy=0;
  for (auto & p : in) { cx += p.x; cy += p.y; }
  cx /= static_cast<double>(in.size());
  cy /= static_cast<double>(in.size());

  for (auto & p : in) {
    // 1) 등방성(모든 방향) 기본 여유
    double vx = p.x - cx, vy = p.y - cy;
    double n = std::hypot(vx, vy); if (n < 1e-6) n = 1.0;

    double x_local = p.x + iso_dilate_m * (vx / n);
    double y_local = p.y + iso_dilate_m * (vy / n);

    // 2) 전방(+x) 여유: 중심 기준 전방측 점들만 +x로 평행 이동
    if (forward_len_m > 1e-6 && (p.x - cx) >= 0.0) {
      x_local += forward_len_m;
    }

    geometry_msgs::msg::Point q;
    q.x = x_local; q.y = y_local; q.z = 0.0;
    out.push_back(q);
  }
  return out;
}

// [CHANGED] forward_len_m을 인자로 받아 전방(+x)으로만 스미어 적용
void AgentLayer::fillFootprintAt(const geometry_msgs::msg::PolygonStamped & fp,
                                 const geometry_msgs::msg::Pose & pose,
                                 double extra_dilation_m,
                                 double forward_len_m,   // [NEW]
                                 nav2_costmap_2d::Costmap2D * grid,
                                 unsigned char cost,
                                 std::vector<std::pair<unsigned int,unsigned int>> * meta_hits)
{
  // 1) 로컬 폴리곤을 등방성 + 전방(+x)으로만 늘리기
  auto poly = dilateFootprintDirectional(fp.polygon.points, extra_dilation_m, forward_len_m);

  // 2) 로컬→월드 변환
  const double yaw = tf2::getYaw(pose.orientation);
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (auto & p : poly) {
    const double x = p.x, y = p.y;
    p.x = pose.position.x + c * x - s * y;
    p.y = pose.position.y + s * x + c * y;
  }

  // 3) bbox in world
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

  // 5) 래스터 채우기 (Max-merge)
  for (int j = min_j; j <= max_j; ++j) {
    for (int i = min_i; i <= max_i; ++i) {
      double wx, wy; grid->mapToWorld(i, j, wx, wy);

      // ray-casting (point-in-polygon)
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
        if (meta_hits) meta_hits->emplace_back(
            static_cast<unsigned int>(i), static_cast<unsigned int>(j));
      }
    }
  }

  // 6) bounds bookkeeping for updateBounds
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

// [CHANGED] 이동 중이면 forward_smear_m_ 사용, 아니면 0.0
void AgentLayer::rasterizeAgentPath(
  const multi_agent_msgs::msg::MultiAgentInfo & a,
  nav2_costmap_2d::Costmap2D * grid,
  std::vector<std::pair<unsigned int,unsigned int>> & meta_hits)
{
  // 코스트 & 등방성 팽창
  const unsigned char cost_now = computeCost(a);
  const double iso_extra = computeDilation(a);

  // 이동 여부에 따라 전방 스미어 적용
  const double forward_len = isMovingPhase(a.status.phase) ? forward_smear_m_ : 0.0;

  // 1) 에이전트 현재 footprint 찍기 (전방 스미어 조건부 적용)
  fillFootprintAt(a.footprint, a.current_pose.pose, iso_extra, forward_len,
                  grid, cost_now, &meta_hits);

  // 2) (선택) truncated_path의 각 pose에서도 footprint를 얇게/간격 띄워서 찍고 싶다면
  //    아래 루프를 활성화하세요. 지금은 과도한 차단을 피하기 위해 "현재 위치만" 반영.
  //
  // const int limit = std::min<int>(a.truncated_path.poses.size(), max_poses_);
  // for (int i = 0; i < limit; ++i) {
  //   const auto & ps = a.truncated_path.poses[i].pose;
  //   // 경로상의 footprint는 등방성만 소량(예: iso_extra*0.5), 전방 스미어는 0.0로 권장
  //   fillFootprintAt(a.footprint, ps, iso_extra * 0.5, 0.0, grid, cost_now, &meta_hits);
  // }
}

void AgentLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                             int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  if (!enabled_) return;

  std::vector<multi_agent_msgs::msg::MultiAgentInfo> infos;
  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    if (!last_infos_ || stale(last_stamp_)) return;
    infos.assign(last_infos_->agents.begin(), last_infos_->agents.end());
  }

  std::vector<std::pair<unsigned int,unsigned int>> meta_hits;
  meta_hits.reserve(256);

  for (const auto & a : infos) {
    if (isSelf(a)) continue;
    rasterizeAgentPath(a, &master_grid, meta_hits);
  }

  if (publish_meta_ && meta_pub_) {
    multi_agent_msgs::msg::AgentLayerMetaArray arr;
    arr.header.frame_id = layered_costmap_->getGlobalFrameID();
    arr.header.stamp = node_shared_->now();

    for (size_t k = 0; k < meta_hits.size(); k += std::max(1, meta_stride_)) {
      auto [mx, my] = meta_hits[k];
      double wx, wy; master_grid.mapToWorld(mx, my, wx, wy);
      multi_agent_msgs::msg::AgentLayerCellMeta cm;
      cm.header = arr.header;

      cm.machine_id = 0;
      cm.phase = 0;
      cm.mode = "";
      cm.reroute = false;
      cm.re_path_search = false;
      cm.transferring = false;
      cm.area_id = 0;

      cm.position.x = wx; cm.position.y = wy; cm.position.z = 0.0;
      cm.mx = mx; cm.my = my;

      cm.t_first_hit = -1.0f;
      cm.sigma = 0.0f;

      arr.cells.emplace_back(std::move(cm));
    }
    meta_pub_->publish(std::move(arr));
  }
}

} // namespace multi_agent_nav2

// pluginlib export
PLUGINLIB_EXPORT_CLASS(multi_agent_nav2::AgentLayer, nav2_costmap_2d::Layer)
```

---

## 적용/튜닝 팁 요약

* `forward_smear_m`: **이동 중일 때만** 전방(+x)으로 늘어납니다. 과도하면 통로가 막히니 0.1~0.4m 권장.
* `dilation_m` + `sigma_k`: 항상 확보할 **등방성** 여유. 잡음이 크면 `sigma_k`로 보강.
* 정지 로봇은 footprint만 치사/고코스트로 유지되고, **콘 모양 확장 없음**.
* 경로까지 두껍게 칠해지는 걸 원치 않으면, 위 코드처럼 **현재 위치만** 찍고 경로 루프는 비활성화 유지하세요.
  필요 시 경로 루프를 켜되 `forward_len=0.0`, `extra_dilation`을 작게(예: `iso_extra*0.5`) 주면 “얇은 띠”로 표현 가능합니다.

필요하면 이 버전에 맞춘 `nav2_params.yaml` 샘플도 정리해 드릴게요.
###################  
####################  
######################  
좋지요! 아래는 TurtleBot3 시뮬( `tb3_simulation_launch.py` ) 기준으로 바로 넣어 쓸 수 있는 **`nav2_params.yaml` 샘플**입니다.
핵심은 `global_costmap`(필수) + `local_costmap`(선택) 둘 다에 `agent_layer`를 플러그인으로 등록하고, 레이어별 파라미터 블록을 분리해 두는 것입니다.

---

# 샘플 `nav2_params.yaml`

```yaml
# =====================================================================
# Nav2 core params (생략 가능: 기존 파일에 그대로 두고 costmap 부분만 병합해도 됨)
# =====================================================================
amcl:
  ros__parameters:
    use_sim_time: true

bt_navigator:
  ros__parameters:
    use_sim_time: true

controller_server:
  ros__parameters:
    use_sim_time: true

planner_server:
  ros__parameters:
    use_sim_time: true

smoother_server:
  ros__parameters:
    use_sim_time: true

# =====================================================================
# GLOBAL COSTMAP
# =====================================================================
global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: true
      footprint: "[[0.22, 0.22], [0.22, -0.22], [-0.22, -0.22], [-0.22, 0.22]]"
      global_frame: map
      robot_base_frame: base_link
      rolling_window: false
      track_unknown_space: true
      resolution: 0.05
      width: 50.0
      height: 50.0

      # 순서 중요: static -> obstacle -> agent -> inflation
      plugins: ["static_layer", "obstacle_layer", "agent_layer", "inflation_layer"]

      static_layer:
        plugin: nav2_costmap_2d::StaticLayer
        map_subscribe_transient_local: true

      obstacle_layer:
        plugin: nav2_costmap_2d::ObstacleLayer
        observation_sources: scan
        scan:
          topic: /scan
          clearing: true
          marking: true
          data_type: LaserScan
          expected_update_rate: 0.0
          obstacle_range: 2.5
          raytrace_range: 3.0

      # ================================
      # AgentLayer (여기가 우리의 플러그인)
      # ================================
      agent_layer:
        plugin: multi_agent_nav2::AgentLayer

        # 기본 동작 스위치
        enabled: true
        qos_reliable: true

        # 입력 토픽 및 좌표계 체크
        topic: /multi_agent_infos
        use_path_header_frame: true     # path.header.frame_id가 global_frame과 같을 때만 반영
        roi_range_m: 12.0               # 우리 로봇 기준 반경 ROI (성능/노이즈 필터링용)

        # 자기 식별(내 로봇은 무시)
        self_machine_id: 0              # 내 robot의 machine_id
        self_type_id: ""                # 내 robot의 type_id (문자열)

        # 신선도/수신 실패 대비
        freshness_timeout_ms: 800       # 최근 수신 기준 밀리초
        max_poses: 40                   # 각 agent truncated_path에서 사용할 최대 포즈 수

        # 비용/도형 관련
        lethal_cost: 254                # 정지/치명적 상황의 코스트
        waiting_cost: 200               # 대기/작업 등의 기본 코스트
        moving_cost: 180                # 이동중 코스트
        manual_cost_bias: 30            # mode=="manual" 시 가산치(클램프됨)

        # 팽창(등방성)과 전방 스미어(이동중에만 +x방향)
        dilation_m: 0.05                # 항상 적용되는 등방성 여유(모든 방향)
        sigma_k: 2.0                    # pos_std_m * sigma_k 만큼 추가 여유
        forward_smear_m: 0.25           # 이동중일 때만 전방(+x)으로 늘리는 길이

        # 메타 퍼블리시(디버그/시각화용)
        publish_meta: true
        meta_stride: 3

      inflation_layer:
        plugin: nav2_costmap_2d::InflationLayer
        cost_scaling_factor: 3.0
        inflation_radius: 0.55   # TB3 환경에 맞게 필요 시 조정

# =====================================================================
# LOCAL COSTMAP  (선택) - 로컬에서도 agent를 보고 싶다면 켜기
# =====================================================================
local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: true
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 6.0
      height: 6.0
      resolution: 0.05

      # 순서 중요: obstacle -> agent -> inflation
      plugins: ["obstacle_layer", "agent_layer", "inflation_layer"]

      obstacle_layer:
        plugin: nav2_costmap_2d::ObstacleLayer
        observation_sources: scan
        scan:
          topic: /scan
          clearing: true
          marking: true
          data_type: LaserScan
          expected_update_rate: 0.0
          obstacle_range: 2.5
          raytrace_range: 3.0

      # Local도 동일 파라미터를 가질 수 있지만 상황에 맞게 더 보수/보급적으로 변경 가능
      agent_layer:
        plugin: multi_agent_nav2::AgentLayer
        enabled: true
        qos_reliable: true
        topic: /multi_agent_infos
        use_path_header_frame: true
        roi_range_m: 8.0
        self_machine_id: 0
        self_type_id: ""
        freshness_timeout_ms: 800
        max_poses: 25
        lethal_cost: 254
        waiting_cost: 200
        moving_cost: 190
        manual_cost_bias: 30
        dilation_m: 0.07
        sigma_k: 2.0
        forward_smear_m: 0.20
        publish_meta: false     # 로컬은 OFF로도 충분
        meta_stride: 3

      inflation_layer:
        plugin: nav2_costmap_2d::InflationLayer
        cost_scaling_factor: 3.0
        inflation_radius: 0.50
```

---

# 파라미터 상세 & 튜닝 가이드

### 공통/입출력

* **`enabled`** (bool): 레이어 활성/비활성 스위치.
* **`plugin`** (string): pluginlib 등록 타입. 반드시 `multi_agent_nav2::AgentLayer`.
* **`topic`** (string): 다른 로봇 상태를 담은 `MultiAgentInfoArray` 토픽 이름.
* **`qos_reliable`** (bool): `true`면 Reliable QoS(유실 적고 지연↑), `false`면 BestEffort(유실 가능, 지연↓).

### 좌표/프레임 & ROI

* **`use_path_header_frame`** (bool): `true`면 `truncated_path.header.frame_id`가 `global_frame`과 같을 때만 반영합니다. 프레임 불일치에 의한 오표시 방지.
* **`roi_range_m`** (double): 우리 로봇 기준의 반경 ROI. 멀리 떨어진 agent는 무시하여 성능/잡음 개선.

  * 혼잡 환경: 8~12 m
  * 넓은 맵/적은 로봇: 15~25 m

### 자기 식별(자기 로봇은 무시)

* **`self_machine_id`** (uint16), **`self_type_id`** (string): `MultiAgentInfo`의 `(machine_id,type_id)`가 여기와 같으면 **무시**합니다. (중복 반영 방지)

### 데이터 신선도 & 경로 길이

* **`freshness_timeout_ms`** (int): 이 시간 이상 지난 메시지는 **무시**. 통신 끊김시 고정 장애물처럼 남는 문제를 예방.
* **`max_poses`** (int): 각 agent의 `truncated_path.poses` 중 최대 몇 개를 사용할지. 값이 크면 계산량↑.

  * 전역맵: 25~60
  * 로컬맵: 15~30

### 비용(코스트) 정책

* **`lethal_cost`** (0~254): 치명적/절대 회피해야 하는 상황에 부여할 비용. 기본 254.
* **`waiting_cost`** (0~254): 정지/작업 상태 등에서의 기본 비용. 보통 200 근처.
* **`moving_cost`** (0~254): 이동 중인 로봇의 비용. 보통 170~200 사이(너무 높으면 전역경로가 돌아가다 막힘).
* **`manual_cost_bias`** (int): `mode=="manual"`일 때 가산치. 결과는 0~254로 클램프.

  * 수동 주행 로봇을 더 피하고 싶다면 30~60.

> 정책 추천
>
> * **정지 로봇**: footprint만 **lethal**(254)로 두는 것이 명확합니다.
> * **이동 로봇**: footprint는 `moving_cost`(180±)로 **높게** 주되, 필요 시 인플레이션 레이어가 “사이즈 여유”를 제공하도록 합니다.
> * 전역 플래너가 관통한다면: `moving_cost`/`waiting_cost`를 올리거나, 인플레이션 반경을 키워 주세요.

### 형태(도형) 파라미터

* **`dilation_m`** (double): **등방성(모든 방향) 기본 여유**. footprint 외곽을 균일하게 키웁니다.

  * 측정 잡음/동적 오차가 있다면 0.03~0.10 m 권장.
* **`sigma_k`** (double): `pos_std_m`(메시지 내 위치 표준편차) × `sigma_k` 만큼 **추가 여유**. 추정치가 불안정한 로봇일수록 자연스럽게 더 크게 잡힙니다.
* **`forward_smear_m`** (double): **이동 중일 때만** 로컬 프레임의 +x(전방)으로 footprint를 **길게** 늘립니다.

  * 추천: 0.1~0.4 m. 너무 크면 통로를 막아 Planner가 경로를 찾지 못할 수 있습니다.
  * **정지 상태**에선 0으로 자동 적용(코드상).

> 동작 요약
>
> * **정지 로봇**: 등방성 + `lethal_cost`; **전방 스미어 없음** → 콘 모양 확장 방지.
> * **이동 로봇**: 등방성 + **전방(+x) 스미어** + `moving_cost`.

### 메타(디버그)

* **`publish_meta`** (bool): 셀 타격 메타 정보를 퍼블리시.
* **`meta_stride`** (int): 퍼블리시 샘플링 간격(계산/대역 감소용).

---

## 튜닝 순서 제안

1. **보이기부터**

   * `agent_layer.enabled=true`로 켭니다.
   * `waiting_cost=254`, `moving_cost=180`, `dilation_m=0.05`, `forward_smear_m=0.2`로 시작.
   * RViz에서 `Global Costmap`의 Layer 체크박스 ON.

2. **Planner가 관통하면**

   * `moving_cost`/`waiting_cost` ↑ (예: 200/254),
   * 또는 `inflation_layer.inflation_radius` ↑(0.45→0.60m),
   * 또는 `forward_smear_m` ↑(0.2→0.3m) — 단, 과도하면 막힘.

3. **경로가 너무 우회/막힘**

   * `forward_smear_m` ↓, `moving_cost` ↓, 인플레이션 반경 ↓.
   * `roi_range_m`를 줄여 불필요한 원거리 agent 영향 제거.

4. **측위 잡음/깜빡임**

   * `sigma_k` ↑ 또는 `dilation_m` ↑.
   * `freshness_timeout_ms` 재검토(값이 너무 짧아 깜빡일 수 있음).

5. **성능 이슈**

   * `max_poses` ↓, `roi_range_m` ↓, `publish_meta=false`.

---

필요하면 이 YAML을 현재 쓰시는 파일에 **global/local costmap 블록만 병합**해서 써도 됩니다.
혹시 TB3가 아닌 실제 로봇이라 Lidar FoV나 footprint가 다르면, `footprint`, `inflation_radius`, `obstacle_range`를 먼저 맞춰 주세요.

#########################   
##########################  
##########################  

좋은 포인트예요. “예전엔 보였는데 지금은 아예 안 보인다”라면, 아래 **두 군데**가 유력한 범인입니다. 둘 다 지금 올려주신 코드에 그대로 있습니다.

---

# 1) Max-merge 조건이 `NO_INFORMATION(255)`에 져서 아예 안 찍힘

지금 `fillFootprintAt()`에서 이렇게 병합하고 있죠:

```cpp
const unsigned char old = grid->getCost(i, j);
if (cost > old) {
  grid->setCost(i, j, cost);
}
```

Nav2의 기본 cost 값은:

* `FREE_SPACE = 0`
* `LETHAL_OBSTACLE = 254`
* `NO_INFORMATION = 255`

초기 맵 영역이 `NO_INFORMATION(255)`인 셀이라면, `moving_cost=180`, `waiting_cost=200` 같은 “정상적인” 값은 **절대 255보다 클 수 없어서** 위 조건이 성립하지 않습니다. 즉, **단 한 셀도 쓰지 못해** 레이어가 통째로 안 보입니다.

### 수정 (추천)

`NO_INFORMATION`이면 비교 기준을 0으로 간주해서 “비어있는 셀”로 취급하고, 그 외엔 Max-merge:

```cpp
const unsigned char old_raw = grid->getCost(i, j);
const unsigned char old = (old_raw == nav2_costmap_2d::NO_INFORMATION) ? 0 : old_raw;

if (cost > old) {
  grid->setCost((unsigned int)i, (unsigned int)j, cost);
}
```

이 2줄만 바꿔도, **UNKNOWN 영역에도 정상적으로 코스트가 찍혀서** RViz에 보입니다.
(참고: 전역맵이 StaticLayer로 이미 채워진 곳이라면 `old`가 0~252대라 그대로도 보였을 거라, “예전엔 보였는데 지금은 안 보임” 현상이 재현됩니다.)

---

# 2) `updateBounds()`가 프레임 불일치로 **통째로 `continue;`** 해서 bounds를 전혀 안 만짐

지금 `updateBounds()` 흐름이 이렇습니다:

```cpp
// ROI 통과 후...
// (문제) 여기서 frame 불일치면 continue로 에이전트 전체 skip
if (use_path_header_frame_ && a.truncated_path.header.frame_id != global_frame) {
  continue;
}

// 그 다음에 '현재 위치 + truncated path'를 bounds에 반영
{ /* current_pose -> bounds */ }
for (...) { /* path poses -> bounds */ }
```

즉, **경로 헤더 frame이 `map`과 다르면 현재 위치조차 bounds에 포함 안 되고** `touched_`가 false로 남습니다. 그러면 이 레이어가 “이번 사이클에 업데이트할 영역 없다”고 선언하는 것과 같아서, 상위가 우리 레이어의 `updateCosts()`를 안 부르거나, 업데이트 윈도우가 0으로 잡혀 시각적으로 아무것도 안 보일 수 있습니다. (이전 버전에서 보였던 건 이 `continue`가 없었기 때문일 가능성 큽니다.)

### 수정 (추천)

* **현재 위치는 항상 bounds에 포함**하세요.
* **경로 포즈만** 프레임 불일치 시 건너뜁니다.

```cpp
// 1) 현재 위치는 항상 bounds에 포함
{
  const auto & p = a.current_pose.pose.position;
  touch_min_x_ = std::min(touch_min_x_, p.x);
  touch_min_y_ = std::min(touch_min_y_, p.y);
  touch_max_x_ = std::max(touch_max_x_, p.x);
  touch_max_y_ = std::max(touch_max_y_, p.y);
  touched_ = true;
}

// 2) truncated_path는 frame 일치할 때만 bounds에 반영
if (!use_path_header_frame_ || a.truncated_path.header.frame_id == global_frame) {
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
```

> 참고: `rasterizeAgentPath()`는 이미 “현재 위치는 무조건 찍고(path는 주석처리)”라서, **bounds만 제대로 만져주면** 상위에서 정상적으로 갱신 창을 열고 우리가 찍은 값이 보이게 됩니다.

---

## 체크리스트 (빠르게 확인)

1. **merge 로직 수정했는지?**

   * `old_raw==255`일 때 0으로 바꿔 비교했는지.

2. **bounds 로직 수정했는지?**

   * `current_pose`는 프레임 상관없이 bounds에 포함시키고,
   * `truncated_path`만 프레임 일치 시 포함했는지.

3. **여전히 안 보이면**

   * `lethal_cost`만 254, 나머지는 180~210이면 잘 보입니다.
   * `roi_range_m`가 너무 작아 에이전트까지 못 미치진 않는지.
   * `freshness_timeout_ms`보다 오래된 메시지(stale) 아닌지.
   * `use_path_header_frame=true`인데 pub되는 `truncated_path.header.frame_id`가 `map`인지 확인. (`ros2 topic echo`로 frame 확인)

---

## 패치 요약

* `fillFootprintAt()` 안의 병합부만 이렇게 교체:

```cpp
const unsigned char old_raw = grid->getCost(i, j);
const unsigned char old = (old_raw == nav2_costmap_2d::NO_INFORMATION) ? 0 : old_raw;
if (cost > old) {
  grid->setCost((unsigned int)i, (unsigned int)j, cost);
}
```

* `updateBounds()`에서 **현재 위치는 항상 bounds 포함**, 경로는 프레임 일치시에만 포함(위 코드블록 그대로).

이 두 군데만 바꾸면, 이전과 동일한 입력으로도 다시 RViz에 다른 로봇이 **안정적으로** 보일 거예요.
#######################################  
#########################################  
########################################3  
좋아요—공유해준 메시지/플러그인/노드들을 전부 훑어보고, 시스템 전반 구조와 각 컴포넌트의 상호작용, 잠재 버그/설계 이슈, 즉시 적용 가능한 패치 제안까지 정리했어. (ROS 2 Jazzy / Nav2 / C++17 & Python 기준)

# 전체 구조 한눈에 보기

```
[MultiAgentInfoArray (map 프레임)]
   ├─ 발행: 여러분의 상위 상태 추정/통신 노드(가정)
   │   • agents[i]: 현재 pose, twist, footprint, truncated_path(프레임=map), 상태/모드 등
   │
   ├─ 구독: AgentLayer (Nav2 costmap layer plugin)
   │   • 타 에이전트 footprint(+방향성 팽창) 및 truncated_path 튜브를 costmap에 rasterize
   │   • /agent_layer/costmap_raw (agent mask) 퍼블리시
   │
   ├─ 구독: PathValidatorNode
   │   • 현재 글로벌 costmap + agent mask 대조로 경로 충돌성 검사
   │   • MultiAgentInfoArray 기반으로 '어떤 에이전트가 이 셀을 유발하는가' 식별
   │   • /path_agent_collision_info 퍼블리시
   │
   └─ 구독: FleetDecisionNode
       • /path_agent_collision_info 받아 후보(Candidate) 산출
       • YIELD/SLOWDOWN/STOP/RUN/REPLAN/REROUTE 상태 결정 & 펄스 토픽 발행
```

# 메시지 정의 요약

* `MultiAgentInfoArray` / `MultiAgentInfo`:

  * `current_pose`, `current_twist`, `footprint(PolygonStamped)`, `truncated_path(Path)`(frame 일치 요구), 다양한 운용 플래그/품질 메타 포함.
* `AgentStatus`:

  * 상위 phase enum.
* `PathAgentCollisionInfo`:

  * 경로상의 최초 충돌 지점들에 대한 “에이전트 유래” 메타(희소 리스트).

# 모듈별 정밀 리뷰

## 1) AgentLayer (Nav2 costmap Layer plugin)

### 설계 포인트

* **등방성 팽창(dilation)** 과 **전방 스미어(forward smear)** 를 분리.
* 이동 상태(`STATUS_MOVING` or `STATUS_PATH_SEARCHING`)에만 전방 스미어 적용.
* 현재 위치는 강하게 찍고, `truncated_path`는 **얇게(iso_extra*0.5) & 전방 스미어 미적용**으로 포착.
* `roi_range_m_`, `freshness_timeout_ms_`, `max_poses_` 등 방어적 파라미터 존재.

### 좋은 점

* 프레임 일치 검사(`use_path_header_frame_`)로 좌표계 혼선 방지.
* `publish_meta_`로 히트셀 샘플을 진단용(디버그/분석)으로 내보낼 수 있음.
* 스레드 안전: 마지막 메시지 스냅샷을 락으로 보호한 뒤 복사해 사용.

### 주의/이슈

1. **RViz 형상 이상(“소시지/반원 덩어리” 효과)**
   현재 `truncated_path`의 **모든 포즈**에서 footprint를 찍고 있어, 포즈 간 간격이 촘촘하면 길쭉하고 굵은 튜브가 생깁니다. 과도한 차단·시각적 왜곡의 원인.

2. **path 프레임 검사 지점**
   `updateBounds()`에서 `use_path_header_frame_`를 `truncated_path.header.frame_id`로 체크하지만, `updateCosts()`에서는 프레임 불일치에 대한 early return이 없음. (실제 rasterize는 pose 자체 좌표를 그대로 월드로 간주하므로, 배열 프레임이 틀리면 위치가 엉킬 수 있음)

3. **전방 스미어 방향 정의**
   `dilateFootprintDirectional()`이 **로컬 +x** 방향을 “전방”으로 가정 → pose yaw로 월드변환하므로 로봇 heading에 정합됨. 다만 폴리곤 중심 기준으로 **cx보다 x≥cx**인 꼭짓점만 밀기 때문에, **뾰족한 앞/뒤 비대칭 footprint**가 아닌 **정사각형**에서는 앞 절반 꼭짓점만 이동 → 모서리 기준으로 살짝 비틀어진 외곽선이 만들어질 수 있음(시각적으로 약간 어색할 수 있음).

4. **코스트 병합 정책**
   Max-merge로 충분히 보수적. 단, 다른 레이어와 병합시 과도한 “lethal 벽”이 생길 수도 있어, 이동 중/정지 중 코스트 단계와 forward smear 길이를 상황별로 더 분리하면 tuning이 쉬워짐.

### 권장 수정 (패치 제안)

* **경로 포즈 stride 적용** (거리 기반 간격 샘플링 + 포즈 수 상한):

  ```cpp
  // rasterizeAgentPath() 내 루프 교체: 거리 누적 스텝
  const double stride_m = std::max(0.1, 0.5 * grid->getResolution()); // 파라미터화 권장
  double acc = 0.0;
  geometry_msgs::msg::Point last = a.truncated_path.poses.front().pose.position;

  const int limit = std::min<int>(a.truncated_path.poses.size(), max_poses_);
  for (int i = 0; i < limit; ++i) {
    const auto & ps = a.truncated_path.poses[i].pose;
    if (i > 0) {
      acc += std::hypot(ps.position.x - last.x, ps.position.y - last.y);
      if (acc < stride_m) continue;
      acc = 0.0;
      last = ps.position;
    }
    // 경로 footprint는 더 얇고 낮은 코스트로
    constexpr double kPathDilateScale = 0.4;
    const double path_iso = iso_extra * kPathDilateScale;
    const unsigned char path_cost = std::min<unsigned char>(computeCost(a), 170); // moving<waiting보다 낮게도 가능
    fillFootprintAt(a.footprint, ps, path_iso, /*forward_len=*/0.0, grid, path_cost, &meta_hits);
  }
  ```
* **프레임 일치 강제**: `updateCosts()`에도 frame mismatch 시 건너뛰기.
* **전방 스미어 파라미터화**: `forward_smear_m_moving`, `forward_smear_m_waiting=0` 처럼 두 단계로 노출.
* **디버그 토픽**: 실제 래스터라이즈한 world bbox/stride 적용 결과를 Marker로 시각화(옵션).

---

## 2) PathValidatorNode (C++)

### 설계 포인트

* `/global_costmap/costmap_raw` + (옵션) `/agent_layer/costmap_raw`(agent mask)을 수신.
* ROI/시야각 기반 장애물 DB 구축 → 일정 시간 지속된 셀만 “성숙” 장애물로 간주.
* 경로 검증 모드 두 가지:

  * **포인트 기반**(`validateWithPoints`) — 빠름.
  * **footprint 기반**(`validateWithFootprint`) — 정밀.
* **우선순위**: 충돌 셀 발견 시 **항상 에이전트 유래 여부를 먼저 판단**(`whoCoversPoint`) → 맞으면 `PathAgentCollisionInfo` 퍼블리시 & 에이전트 홀드 타이머 가동.
* `whoCoversPoint`는

  * (1) **에이전트 현재 footprint**(소확장) 포함 여부,
  * (2) **truncated_path 튜브** 포함 여부(간격 stride/dilate)
    순서로 검사.

### 좋은 점

* 에이전트 충돌이면 일반 장애물 replan을 억제하는 **홀드** 로직이 있어 진동(replanning flapping)을 완화.
* footprint/튜브 매칭이 **프레임 일치**를 기본 전제로 둠(`MultiAgentInfoArray.header.frame_id` vs `global_frame_`)—스펙과 일치.

### 주의/이슈

1. **스레싱 방지(쿨다운, 홀드)**는 잘 들어가 있지만, `compare_agent_mask_`가 켜져 있을 때 **agent mask 근처** 셀을 에이전트 충돌로 오판정할 수도 있음. 다만 보조 조건으로만 쓰고, 결국 `whoCoversPoint`에 다시 의존하므로 안전.
2. `agent_path_hit_max_poses`가 크면 연산량↑. stride를 충분히 크게 유지해야 함.
3. `agent_cost_threshold_`와 AgentLayer의 코스트 등급이 괴리되면 false negative/positive가 생길 수 있음(튜닝 포인트).

### 권장 수정/팁

* `whoCoversPoint()`에서 `truncated_path.header.frame_id`가 `global_frame_`와 다를 때 **해당 에이전트 path 튜브 검사를 skip**하도록 안전장치 추가(현재는 Array header만 점검).
* 튜브 매칭 stride/dilate를 **AgentLayer의 path stride 및 iso 스케일과 대응**시키면 일관성↑.
* `/path_agent_collision_info`에 **우리(my_id)도 포함**되어 들어올 수 없도록(지금도 제외하지만) 방어 코드 유지.

---

## 3) FleetDecisionNode (Python)

### 설계 포인트

* 이벤트 드리븐 상태기계:

  * 입력: `/path_agent_collision_info`, `/multi_agent_infos`, `/replan_flag`.
  * 출력: 상태 스트링 `/decision_state`, 그리고 one-shot 제어 펄스 `/cmd/run|slowdown|yield|stop`, 요청 펄스 `/request_replan|reroute`.
* 후보(Candidate) 스코어 = **충돌 심각도(severity)** × **양보 우선(yprio)**(kappa 가중).

  * 심각도: 1/T, 1/d, 상대 heading, closing 속도 반영.
  * yprio: 상대 mode(수동), right-of-way(occupancy+ID), 상대가 reroute 중인지, PATH_SEARCHING인지 등.
* 진입 규칙: `T_eff`와 `yprio`로 SLOWDOWN/YIELD/… 판정, resume 히스테리시스와 timeout/idle-resume 탑재.

### **심각한 버그 (필수 수정)**

* **토픽 타입 불일치**
  Python에서 `from replan_monitor_msgs.msg import PathAgentCollisionInfo`로 import하고 구독하지만, **퍼블리셔는 C++ 쪽에서 `multi_agent_msgs::msg::PathAgentCollisionInfo`** 를 발행하고 있어요.
  ▶︎ **수정**:

  ```py
  # 잘못된 라인
  # from replan_monitor_msgs.msg import PathAgentCollisionInfo

  # 교체
  from multi_agent_msgs.msg import PathAgentCollisionInfo
  ```

  이거 하나로 “충돌 이벤트가 안 들어온다/상태가 안 바뀐다” 류의 증상 대부분이 해결될 가능성이 큽니다.

### 논리 이슈/개선 포인트

1. **상호 대칭으로 인한 동시 YIELD / 동시 RUN 가능성**

   * 이미 `id_bonus()`와 `right_of_way_score()`에서 **ID 비대칭(작은 ID 우선)** 을 주긴 하지만, 현재 가중/임계치 조합에 따라 **동시에 같은 결론**이 날 수 있음.
   * 해결책(권장):

     * **결정 타이브레이커를 명시적으로 적용**: 동일 조건일 때 `my_id < other_id`이면 RUN 우선, 반대는 YIELD 등 **하드 룰**을 추가.
     * **미세 offset을 ID로 섞기**: `T_yield`, `Y_th`에 `±ε*(my_id-other_id)` 같은 비대칭 오프셋을 주어 동률 붕괴.
     * **의사소통 기반 의도교환(optional)**: `/agent_intent` 같은 토픽으로 현재 의도(RUN/YIELD)를 퍼블리시하고, 상대 의도를 반영해 상충 방지.
     * **중앙 심판/락(optional)**: 교차 구간 `area_id` 기준 “토큰(권리증)”을 선점하는 방식.

2. **`right_of_way_score()`와 `id_bonus()`의 중복 효과**

   * 둘 다 ID에 의한 우선권 가산이 들어감(ROW에서 `<my_id` +0.5, id_bonus에서 `my_id>other_id` +1.0). 의도보다 **ID 영향력이 과대**일 수 있어 튜닝 필요.

3. **`combine_ttc()`의 대체 TTC 가중**

   * v_closing < 0.05이면 `T_alt = inf`로 처리돼서, 정지/역방향 에이전트에 좀 둔감. 실제로는 lateral 접근/회전 등도 위험할 수 있으니, **lateral velocity/회전**을 섞을 여지 있음(차후 개선).

4. **reroute 판정 트리거**

   * `decide_with_primary()`에서 “상대가 reroute고 yprio 높음 → REROUTE” 규칙은 시스템마다 공격적일 수 있음. 실사용에서는 `REROUTE` 트리거를 좀 더 보수적으로(예: T_eff< T_yield*0.8 + yprio>Y_th) 추천.

5. **프레임 일치/정합 검사**

   * `build_candidates()` → `match_agent_at_point()`는 전부 map 좌표 가정. 상위 스펙에 의존하고 있지만, 방어적으로 `MultiAgentInfoArray.header.frame_id == global_frame` 미일치 시 candidates 비우는 것도 방법(안전성↑).

---

# “RViz 형상이 이상하다” 원인 분석 & 해결 요약

* **원인**: AgentLayer가 `truncated_path`의 **모든 포즈**에 대해 footprint를 페인트 → 촘촘한 포즈라면 과도한 겹침/두꺼운 튜브. 게다가 iso 확장도 더해져 **반원/소시지**처럼 보임.
* **해결**:

  1. **거리 기반 stride**(예: 0.2~0.4 m)로 path 포즈 샘플링.
  2. path 포즈에서는 **전방 스미어 0** & **등방 확장 스케일 0.3~0.5**로 축소.
  3. (옵션) path 코스트는 현재 위치보다 **낮게**(예: waiting 200, moving 180라면 path는 150~170).
  4. (옵션) `max_poses_`를 줄이거나, ROI 범위 내 포즈만 고려.
  5. (옵션) path를 **선분 스윕**으로 rasterize(두 pose 사이 선분을 footprint로 Minkowski-sum 근사)하면 시각적으로 매끈하고 차단도 덜 과격.

---

# 즉시 적용 체크리스트

* [ ] **FleetDecisionNode**: `PathAgentCollisionInfo` **import 패키지 교체** (반드시!)
* [ ] **AgentLayer**: `truncated_path` stride 적용 + path 코스트/팽창 축소.
* [ ] **AgentLayer**: `updateCosts()`에서도 프레임 불일치 시 skip.
* [ ] **PathValidatorNode**: `whoCoversPoint()`에서 path header frame 체크 추가.
* [ ] **동시 의사결정 방지**: FleetDecisionNode에 **명시적 타이브레이커**(ID 기반 ε-오프셋 또는 하드 룰) 추가.
* [ ] **코스트/임계 동조화**: `agent_cost_threshold_`(PathValidator) ↔ AgentLayer의 moving/waiting/path 코스트 레벨 재튜닝.
* [ ] **RViz 진단**: AgentLayer meta/marker로 stride 적용 전후 bbox를 시각화해 개입 효과 확인.

---

# 추천 파라미터 시작값(현장 튜닝 가이드)

* AgentLayer:

  * `dilation_m`: 0.05 ~ 0.1
  * `forward_smear_m`: 0.15(이동) / 0.0(정지)
  * `max_poses`: 120
  * **추가** `path_stride_m`: 0.25
  * **추가** `path_iso_scale`: 0.4
  * **추가** `path_cost`: 160
* PathValidatorNode:

  * `agent_path_hit_stride_m`: 0.30
  * `agent_path_hit_dilate_m`: 0.02 ~ 0.04
  * `agent_cost_threshold`: AgentLayer path_cost보다 약간 낮거나 동일(예: 160)
* FleetDecisionNode:

  * `T_yield`: 2.5 (기본)
  * `T_slow`: 6.0 (기본)
  * `yield_priority_thresh(Y_th)`: 0.8 (기본)
  * **추가** `tie_break_eps`: 0.1 → `T_yield' = T_yield ± eps*(my_id<other_id ? +1 : -1)`

---

# 마무리

구조 자체는 잘 짜여있고(AgentLayer → PathValidator → FleetDecision의 파이프라인), **한 가지 큰 타입 불일치 버그**와 **AgentLayer의 path 래스터 전략**이 현재 관측된 문제의 핵심이야. 위의 간단 패치들만 반영해도:

* **충돌 이벤트 흐름**이 정상화되고,
* RViz 시각화/차단이 **과도하게 두꺼운 튜브**가 아닌, **의도한 “얇은 미리보기 튜브”**로 개선될 거야,
* 양쪽 로봇이 **동시에 YIELD/RUN**하는 대칭 문제도 **결정 타이브레이커**로 실질적으로 사라질 것.

원하면, 내가 제안한 C++/Python 패치 조각을 여러분 레포 포맷에 맞춰 **완전한 diff**로 정리해줄게.
###########################  
##############################  
################################  
멋져. 아래는 **AgentLayer 전 파라미터 레퍼런스 + 튜닝 가이드**(ROS 2 Jazzy / Nav2 / C++17 기준)야. 각 항목은 “무엇”, “왜”, “권장값/범위”, “주의점” 순서로 정리했어. 마지막에 **샘플 YAML**과 **상황별 프리셋**도 붙였어.

---

# 파라미터 레퍼런스 (설명 + 튜닝 가이드)

## 기본/입출력

* `enabled` (bool, default: `true`)

  * **무엇**: 레이어 on/off.
  * **권장**: 실환경 `true`. 문제 디버깅 시 일시적으로 `false`.

* `topic` (string, default: `"/multi_agent_infos"`)

  * **무엇**: `MultiAgentInfoArray` 구독 토픽.
  * **주의**: PathValidatorNode, 발행 노드와 동일하게.

* `self_machine_id` (int, default: `0`), `self_type_id` (string, default: `""`)

  * **무엇**: **자기 로봇** 식별. AgentLayer는 “타 로봇”만 칠함.
  * **권장**: 반드시 실제 값으로 설정(안 그러면 자기 자신도 마킹될 수 있음).

* `use_path_header_frame` (bool, default: `true`)

  * **무엇**: `truncated_path.header.frame_id`가 costmap 글로벌 프레임과 **일치할 때만** 반영.
  * **권장**: `true`. 프레임 혼선 방지. 프레임 변환을 외부에서 보장할 수 없으면 `true` 유지.

* `qos_reliable` (bool, default: `true`)

  * **무엇**: 구독 QoS 신뢰성 모드.
  * **권장**: 유선/신뢰 네트워크 `reliable`; 무선·패킷 드롭 환경은 `false`(best_effort)로 바꿔 드랍 복원성 확보.

## ROI/신선도

* `roi_range_m` (double, default: `12.0`)

  * **무엇**: 자기 로봇 기준, 이 반경 내 타 로봇만 래스터.
  * **권장**: 실내 8~15 m. 넓게 잡으면 CPU 증가 + 불필요 차단.
  * **팁**: 글로벌 costmap 해상도·속도 범위를 고려해 “정지~1~2초 내 상호작용 거리” 정도로.

* `freshness_timeout_ms` (int, default: `800`)

  * **무엇**: 최신 `MultiAgentInfoArray` 타임아웃. 지나면 업데이트 스킵.
  * **권장**: 센서/통신 주기(예: 10 Hz)에 2~3배 → 300~800 ms.
  * **주의**: 너무 크면 stale 데이터로 “유령 로봇” 잔상.

* `time_decay_sec` (double, default: `1.0`)

  * **무엇**: 설계 흔적. 현재 래스터에는 직접 사용 안 함(미래 감쇠 로직용).
  * **가이드**: 무시해도 무방. 추후 도입 시 “유효시간 내 새 데이터만 칠하기”에 사용.

## 코스트(상태/모드 반영)

* `lethal_cost` (uchar, default: `254`)

  * **무엇**: 상한 참조. 현재 코드에서 직접 사용 X(코멘트에 “치사 덮기” 언급만).
  * **가이드**: 현재는 의미 없음. `moving_cost/waiting_cost` 조합으로 충분.

* `moving_cost` (uchar, default: `180`), `waiting_cost` (uchar, default: `200`)

  * **무엇**: 타 로봇이 **이동 중**/**정지·대기**일 때 칠하는 기본 코스트.
  * **권장**: 정지는 이동보다 **조금 더 높게**(예: 180/200, 또는 170/200).
  * **주의**: PathValidator의 `agent_cost_threshold`와 **레벨을 맞춰야** false pos/neg 줄어듦.

* `manual_cost_bias` (int, default: `30`)

  * **무엇**: 상대 모드가 `"manual"`이면 코스트를 추가 가중.
  * **권장**: 20~40. 현장 오퍼레이터가 수동 주행 시 “더 강하게 피하기”.

## 형상 확장(등방성/전방 스미어)

* `dilation_m` (double, default: `0.05`)

  * **무엇**: **등방성**(모든 방향) 기본 여유(footprint 외연 확장).
  * **권장**: 0.03~0.10 m. 좁은 통로면 0.03~0.05, 난잡 환경은 0.07~0.1.
  * **주의**: 과대하면 “두꺼운 벽”처럼 보이고, 경로 차단 과격.

* `sigma_k` (double, default: `2.0`)

  * **무엇**: `pos_std_m`(위치 표준편차)에 대한 추가 확장 계수. 실제 확장 = `dilation_m + sigma_k * pos_std_m`.
  * **권장**: 1.5~2.5. 센서 품질 낮거나 멀티로봇 위치 불확실 크면 2.0↑.
  * **주의**: `pos_std_m < 0`이면 미적용.

* `forward_smear_m` (double, default: `0.25`)

  * **무엇**: **이동 상태**에서만 로컬 +x(heading 전방)로 추가 길이. 급접근/헤드온 완충.
  * **권장**: 0.10~0.35 m. 속도 빠르고 제동 여유 적을수록 증가.
  * **주의**: 정지 상태에는 코드가 자동으로 `0.0`. 전방 스미어는 **현재 위치만** 적용(경로 포즈엔 적용 안 함).

## 경로 래스터·메타

* `max_poses` (int, default: `40`)

  * **무엇**: `truncated_path`에서 고려할 최대 포즈 수.
  * **권장**: 40~120. 포즈가 매우 촘촘하면 stride 없이 전부 찍을 때 “두꺼운 튜브”가 될 수 있음(지금 코드가 그 상태).
  * **팁**: **거리 기반 stride**를 코드에 도입하면 `max_poses`는 넉넉히 두어도 안전.

* `publish_meta` (bool, default: `true`)

  * **무엇**: 내부 메타 `AgentLayerMetaArray` 퍼블리시(디버깅/시각화용).
  * **권장**: 개발·튜닝 동안 `true`. 운영에선 트래픽 줄이려면 `false`.

* `meta_stride` (int, default: `3`)

  * **무엇**: 메타 샘플링 stride. 값이 클수록 적게 발행.
  * **권장**: 2~5.

---

# 파라미터 상호작용 & 주의점

1. **PathValidatorNode와 코스트 정합**

   * PathValidator의 `agent_cost_threshold`는 AgentLayer의 `moving_cost/waiting_cost` 이상(또는 근처)으로 맞춰야 함.
   * 예) `moving=180, waiting=200`이면 `threshold≈180~200` 권장.

2. **RViz에서 ‘두꺼운 튜브/반원’처럼 보임**

   * 원인: `truncated_path` 모든 포즈에 footprint를 동일 코스트로 촘촘히 칠함.
   * 해결: **거리 stride 도입(예: 0.25 m)** + 경로 포즈에는 **소확장(iso*0.4)**, **전방 스미어 0** + **낮은 코스트(예: 160)**.

3. **프레임 일치**

   * `use_path_header_frame=true`일 때 `truncated_path.header.frame_id`가 global frame과 다르면 스킵.
   * **updateCosts()에도 동일 체크를 넣는 것**을 권장(안전성↑).

4. **불확실도 반영**

   * `sigma_k * pos_std_m`가 들어가므로, 멀티로봇 위치합치가 불안정할수록 외연이 커진다 → 미리 `pos_std_m` 스케일/단위 확인.

---

# 상황별 튜닝 레시피

### A. 협소 통로가 많고 저속(0.5 m/s 이하)

* `dilation_m=0.04~0.06`, `sigma_k=2.0`, `forward_smear_m=0.10~0.20`
* `moving_cost=170~180`, `waiting_cost=200~210`, `manual_cost_bias=30`
* `roi_range_m=8~10`, `freshness_timeout_ms=600`
* (코드 패치 시) `path_stride_m≈0.25`, `path_cost≈160`, `path_iso_scale≈0.4`

### B. 개방 공간, 중속(0.8~1.2 m/s)

* `dilation_m=0.05~0.08`, `sigma_k=2.0`, `forward_smear_m=0.20~0.30`
* `moving_cost=180`, `waiting_cost=200`, `manual_cost_bias=30`
* `roi_range_m=12~15`, `freshness_timeout_ms=800`
* (패치) `path_stride_m≈0.30`, `path_cost≈160~170`, `path_iso_scale≈0.4`

### C. 위치 추정 품질이 들쭉날쭉

* `sigma_k=2.0~2.5` 상향, `dilation_m`은 0.05 유지
* `freshness_timeout_ms`를 500~600으로 낮춰 stale 억제
* `forward_smear_m`은 과도 확대 금지(“벽 두꺼워짐” → 차단 과격)

---

# 예시 YAML (Nav2 Costmap2D 레이어 설정)

```yaml
# nav2_params.yaml (일부)
global_costmap:
  plugins:
    - {name: agent_layer, type: "multi_agent_nav2::AgentLayer"}

  agent_layer:
    enabled: true
    topic: "/multi_agent_infos"
    self_machine_id: 2
    self_type_id: "amr_a"
    use_path_header_frame: true

    # ROI/신선도
    roi_range_m: 12.0
    freshness_timeout_ms: 800

    # 코스트
    moving_cost: 180
    waiting_cost: 200
    manual_cost_bias: 30

    # 형상 확장
    dilation_m: 0.05
    sigma_k: 2.0
    forward_smear_m: 0.25

    # 메타/성능
    publish_meta: true
    meta_stride: 3
    max_poses: 80
    qos_reliable: true
```

> (코드 패치로 stride/경로 코스트를 넣는다면)

```yaml
    path_stride_m: 0.25        # 신규: 경로 포즈 샘플 간격
    path_iso_scale: 0.4        # 신규: 경로 포즈에서 등방 확장 축소율
    path_cost: 160             # 신규: 경로 포즈 코스트(현재 위치보다 낮게)
```

---

# 디버깅 체크리스트

* **프레임**: `MultiAgentInfoArray.header.frame_id == costmap global frame`? `truncated_path.header.frame_id`도 같은가?
* **자기식별**: `self_machine_id/self_type_id` 정확?
* **메시지 신선도**: RViz에 `/multi_agent_infos` stamp 갱신 주기 확인.
* **코스트 정합**: PathValidator `agent_cost_threshold`와 AgentLayer `moving/waiting` 레벨 매칭.
* **시각화**: `publish_meta=true`로 메타 샘플을 RViz Marker로 찍어 경계/튜브 상태 확인.

---

필요하면 위 YAML을 기준으로 **개방 공간/협소 통로** 각각에 맞춘 두 개의 프로파일을 만들어서 런타임에 전환할 수 있게 해줄 수도 있어. 또, 너가 원하면 내가 제안했던 **stride/경로 코스트 옵션**을 실제 C++ 코드 diff 형태로 바로 만들어 줄게.
##############################  
3################################  
##################################  
좋아, 이번엔 **PathValidatorNode 전 파라미터 레퍼런스 + 튜닝 가이드**를 깔끔하게 정리해줄게. (ROS 2 Jazzy / Nav2 / C++17 / 다로봇 환경 기준)

---

# 1) 좌표/프레임

* `global_frame` (str, `"map"`)
  경로/코스트맵/에이전트 비교의 기준 프레임. 에이전트 토픽의 `header.frame_id`도 동일해야 함.
  **가이드:** Nav2 글로벌 코스트맵 프레임과 동일하게.

* `base_frame` (str, `"base_link"`)
  TF에서 자기 위치를 얻을 때 사용.
  **가이드:** 로봇 베이스 프레임과 정확히 일치.

---

# 2) 리플랜 트리거/디바운스

* `cooldown_sec` (double, `1.0`)
  리플랜 연속 트리거 쿨다운.
  **튜닝:** 0.5~2.0s. 너무 작으면 플러딩, 너무 크면 반응 지연.

* `consecutive_threshold` (int, `3`)
  연속 “차단” 샘플 수가 임계 이상일 때 리플랜.
  **튜닝:** 2~5. 통로 울퉁불퉁/노이즈 크면 증가.

* `publish_false_pulse` (bool, `true`), `flag_pulse_ms` (int, `120`)
  `/replan_flag`를 true로 발행 후 자동 false 펄스.
  **가이드:** Nav2 BT 노드가 Edge-trigger라면 켜두는게 편함.

---

# 3) 탐색 거리/속도 기반 창

* `max_speed` (double, `0.5`)
  Lookahead(= `max(max_speed*lookahead_time_sec, min_lookahead_m)`) 계산에 사용.
  **튜닝:** 실제 최고 속도의 0.8~1.0배.

* `lookahead_time_sec` (double, `15.0`), `min_lookahead_m` (double, `2.0`)
  전방 검사 거리 결정.
  **튜닝:** 저속 8~12s, 중속 12~18s. 통로 짧으면 `min_lookahead_m` 2~4m.

* `path_check_distance_m` (double, `6.0`)
  실제 경로 검사 상한(위의 lookahead보다 더 보수적인 캡).
  **튜닝:** 실 주행 환경에서 “실제로 문제되는 거리”로. 개방공간 8~12m, 협소 4~6m.

---

# 4) 코스트맵 임계/커널

* `cost_threshold` (double, `200.0`)
  차단으로 보는 코스트 상한(= 임계 이상이면 obstacle).
  **중요:** **AgentLayer의 `moving_cost/waiting_cost`와 정합**. 예: 180/200이면 190~200 권장.

* `ignore_unknown` (bool, `true`)
  `NO_INFORMATION` 무시 여부.
  **튜닝:** 실내 SLAM이면 `true`, 외부/맵 구멍 많으면 `false` 고려.

* `kernel_half_size` (int, `1`)
  `isBlockedCellKernel()`에서 (2K+1)^2 커널로 주변까지 차단 체크.
  **튜닝:** 0~2. 통로 삐걱/노이즈 크면 1~2로 완충.

---

# 5) 내부 장애물 DB(성숙도)

* `obstacle_persistence_sec` (double, `0.5`)
  같은 셀을 이 시간 이상 연속 관측해야 “성숙 장애물”로 인정.
  **튜닝:** 0.3~1.0s. 플리커링 노이즈가 많을수록 증가.

* `db_update_frequency` (double, `5.0`)
  내부 DB 스캔 주기(Hz).
  **튜닝:** 3~10. CPU 여유 없으면 3~5Hz.

* `obstacle_prune_timeout_sec` (double, `3.0`)
  오래 안 보이면 DB에서 삭제.
  **튜닝:** 2~5s. 느린 업데이트 환경은 늘림.

* `db_stride` (int, `2`)
  ROI 샘플링 셀 간격.
  **튜닝:** 1(세밀)~3(가벼움). 고해상도 맵이면 2~3.

* `cone_angle_deg` (double, `100.0`)
  전방 시야 각(헤딩 중심). 원뿔 밖은 스킵.
  **튜닝:** 좁은 통로 90~120°, 크로스/합류 많으면 120~160°.

---

# 6) 검증 방식(점/발자국)

* `use_footprint_check` (bool, `false`)
  경로의 각 포즈를 **로봇 footprint**로 채워 차단 확인(더 정확, 더 비쌈).
  **튜닝:** 협소/정밀 상황에서 `true`. CPU 타이트할 때 `false`.

* `footprint_step_m` (double, `0.15`)
  Footprint 방식에서 경로 샘플 간격.
  **튜닝:** 0.10~0.25m. 작을수록 정확↑/부하↑.

* `footprint` (str, `"[]"`), `robot_radius` (double, `0.1`)
  Nav2 footprint 형식 문자열. 유효하면 폴리곤, 아니면 원(`robot_radius`).
  **가이드:** Nav2 costmap과 **동일 값** 사용.

---

# 7) 에이전트 마스크/충돌 메시지

* `compare_agent_mask` (bool, `true`)
  AgentLayer가 만든 마스크 코스트맵으로 보조 교차 판단.
  **튜닝:** 동적 장애물 대부분이 “다른 로봇”이면 `true`가 유리.

* `agent_mask_topic` (str, `"/agent_layer/costmap_raw"`)
  에이전트 마스크 코스트맵 토픽.

* `agent_cost_threshold` (double, `254.0`)
  에이전트 마스크에서 “로봇”으로 취급할 임계.
  **중요:** AgentLayer의 `moving_cost/waiting_cost` 이상으로.
  예) 180/200이면 200~254.

* `agent_mask_manhattan_buffer` (int, `1`)
  맨해튼 반경 탐색(±1이면 다이아몬드 반경 1).
  **튜닝:** 0~2. 경계 픽셀 깨짐 보정.

* `publish_agent_collision` (bool, `true`), `agent_collision_topic` (str, `"/path_agent_collision_info"`)
  교차로봇 충돌 정보를 퍼블리시(FleetDecisionNode 입력).

---

# 8) 에이전트 토픽/신선도/매칭

* `agents_topic` (str, `"/multi_agent_infos"`)
  에이전트 배열 입력.

* `agents_freshness_timeout_ms` (int, `800`)
  신선도 체크. 초과 시 매칭 스킵.
  **튜닝:** 발행 주파수의 2~3배.

* `agent_match_dilate_m` (double, `0.05`)
  에이전트 footprint 소확장(포인트 포함 판정 여유).
  **튜닝:** 0.03~0.08m.

---

# 9) 에이전트 경로 튜브 매칭 (truncated_path)

* `agent_path_hit_enable` (bool, `true`)
  에이전트 `truncated_path`를 얇은 튜브로 취급해 커버여부 검사.

* `agent_path_hit_stride_m` (double, `0.35`)
  경로 포즈 거리 샘플 간격.
  **튜닝:** 0.25~0.5m. 촘촘하면 과검출/부하↑.

* `agent_path_hit_dilate_m` (double, `0.02`)
  튜브 두께(footprint 등방확장량).
  **튜닝:** 0.02~0.05m. RViz에서 “길쭉한 벽” 느낌 나면 줄여라.

* `agent_path_hit_max_poses` (int, `500` 로드 후 내부 200 사용)
  최대 포즈 수.
  **튜닝:** 200~600. stride가 있으니 200~300이면 충분한 경우가 많음.

---

# 10) 상태 연동/홀드

* `agent_block_hold_sec` (double, `2.0`)
  **에이전트 충돌 확정 후** 잠깐 리플랜 억제(Decision/Fleet과의 상호작용에서 플리커 억제).
  **튜닝:** 1.0~3.0s.

* `agent_block_max_wait_sec` (double, `8.0`)
  (현재 코어 로직에서 직접 사용 빈도 낮음) 상한 대기 시간 컨셉.

* `agents_freshness_timeout_ms`와 `agent_block_hold_sec`의 균형이 중요:
  신선도 타임아웃이 **너무 크고**, 홀드가 길면 “유령 경로 튜브”에 오래 묶일 수 있음.

---

# 11) 런타임/상태

* 내부 `is_robot_in_driving_state_`는 `/robot_status`가 `"DRIVING"`/`"PLANNING"`일 때에만 검사 수행.
  **가이드:** 상태 문자열 발행 체계를 일관되게 유지(대소문자 포함).

---

## 상호작용 포인트 (AgentLayer와 맞물림)

1. **코스트 임계 정합**

   * PathValidator `cost_threshold` vs AgentLayer `moving_cost/waiting_cost` vs PathValidator `agent_cost_threshold`
   * 예: AL(180/200) → PV(cost_threshold=190~200, agent_cost_threshold=200~254)

2. **프레임 정합**

   * PathValidator `global_frame` == AgentLayer / MultiAgentInfoArray `header.frame_id`.

3. **튜브 과비대화 방지**

   * `agent_path_hit_stride_m` 충분히 크게(≥0.25), `agent_path_hit_dilate_m` 너무 키우지 않기(0.02~0.03 우선).

---

# 상황별 튜닝 레시피

### A) 협소 통로, 저속(~0.5 m/s)

* `max_speed=0.5`, `lookahead_time_sec=10.0`, `min_lookahead_m=2.0`, `path_check_distance_m=5~6`
* `cost_threshold=195~205`, `kernel_half_size=1~2`, `obstacle_persistence_sec=0.6~0.8`
* `use_footprint_check=true`, `footprint_step_m=0.12~0.18`
* `agent_path_hit_stride_m=0.30~0.40`, `agent_path_hit_dilate_m=0.02~0.03`

### B) 개방 공간, 중속(0.8~1.2 m/s)

* `max_speed=1.0`, `lookahead_time_sec=14~18`, `min_lookahead_m=2.0`, `path_check_distance_m=8~12`
* `cost_threshold=190~200`, `kernel_half_size=1`, `obstacle_persistence_sec=0.4~0.6`
* `use_footprint_check=false`(부하 줄이기)
* `agent_path_hit_stride_m=0.35~0.50`, `agent_path_hit_dilate_m=0.02~0.04`

### C) 센서 노이즈/맵 플리커 심함

* `ignore_unknown=false` 고려, `kernel_half_size=2`, `obstacle_persistence_sec=0.8~1.0`
* `db_stride=2~3`, `cone_angle_deg=120~140`
* `consecutive_threshold=4~5` 로 과민 억제

---

# 예시 YAML 스니펫

```yaml
replan_monitor:
  ros__parameters:
    global_frame: map
    base_frame: base_link

    # Trigger
    cooldown_sec: 1.0
    consecutive_threshold: 3

    # Lookahead
    max_speed: 0.8
    lookahead_time_sec: 12.0
    min_lookahead_m: 2.0
    path_check_distance_m: 8.0

    # Cost thresholds
    cost_threshold: 195.0
    ignore_unknown: true
    kernel_half_size: 1

    # Obstacle DB
    db_update_frequency: 5.0
    obstacle_persistence_sec: 0.6
    obstacle_prune_timeout_sec: 3.0
    db_stride: 2
    cone_angle_deg: 120.0

    # Replan flag pulse
    publish_false_pulse: true
    flag_pulse_ms: 120

    # Footprint check
    use_footprint_check: false
    footprint_step_m: 0.15
    footprint: "[]"
    robot_radius: 0.30

    # Agent mask
    compare_agent_mask: true
    agent_mask_topic: /agent_layer/costmap_raw
    agent_cost_threshold: 200.0
    agent_mask_manhattan_buffer: 1

    # Agent collisions out
    publish_agent_collision: true
    agent_collision_topic: /path_agent_collision_info

    # Agents input
    agents_topic: /multi_agent_infos
    agents_freshness_timeout_ms: 800
    agent_match_dilate_m: 0.05

    # Agent hold
    agent_block_hold_sec: 2.0
    agent_block_max_wait_sec: 8.0

    # Path-tube matching
    agent_path_hit_enable: true
    agent_path_hit_stride_m: 0.35
    agent_path_hit_dilate_m: 0.02
    agent_path_hit_max_poses: 300
```

---

## 디버깅 체크리스트

* **프레임:** `/multi_agent_infos.header.frame_id == global_frame`?
* **코스트 정합:** `cost_threshold` vs AgentLayer `moving/waiting` vs `agent_cost_threshold`.
* **신선도:** `agents_freshness_timeout_ms`와 실제 발행 주기 일치?
* **튜브 과비대화:** stride/dilate 조합이 과한지 RViz에서 확인.
* **홀드 로직:** 에이전트 이벤트 후 `agent_block_hold_sec` 동안 리플랜 억제되는지 로그로 검증.
* **Footprint 모드 부하:** CPU 프로파일링으로 `use_footprint_check` 영향 확인.

---

원하면 네 환경(속도, 공간 제약, AgentLayer 설정)을 기준으로 **두세 개 프리셋(YAML)**을 더 구체적으로 만들어 줄게.
##################################  
####################################3  
###################################  
좋아! `fleet_decision_node`의 **전 파라미터 레퍼런스 + 튜닝 가이드**를 한 번에 정리해줄게. (다중 로봇/교차 지점, PathValidator/AgentLayer와 연동 가정)

---

# 1) ID / 프레임

* `global_frame` (str, `"map"`)
  내부 계산·로그 기준 프레임. `MultiAgentInfoArray.header.frame_id`와 일치해야 함.
  **가이드:** Nav2 글로벌 코스트맵 프레임과 동일하게 유지.

* `my_machine_id` (int, `1`)
  본 로봇 ID. ID 비교(ROW/우선순위, 동점자 타이브레이커)에 사용.
  **가이드:** 플릿 전체에서 유일 값.

---

# 2) TTC 결합(시간 여유 추정)

* `w1_ttc` (double, `1.0`) : PathValidator가 준 **직접 TTC** 가중치
* `w2_alt` (double, `0.8`) : 상대 속도·거리로 만든 **대체 TTC** 가중치
* `T_min`   (double, `0.5`) : 1/T 특이점 방지용 하한(작을수록 민감)
* `d_min`   (double, `0.2`) : 1/d 하한(근접 민감도 상한)

**튜닝 팁**

* 센서/추정이 안정적 → `w1_ttc`↑ (직접 TTC 신뢰).
* 상대 속도 추정이 신뢰도 높음 → `w2_alt`↑.
* 과민 반응이면 `T_min`, `d_min`을 **살짝 키움**(0.6~0.8 / 0.25~0.35).

---

# 3) 위험도(Severity) 가중치

* `a1_invT` (double, `1.2`) : 1/T (가까운 충돌 시각)
* `a2_invd` (double, `0.2`) : 1/d (가까운 거리)
* `a3_heading` (double, `0.5`) : 상대 진행방향(정면>교차>동방향) 보너스
* `a4_vclosing` (double, `0.3`) : 폐쇄 속도(상대 접근 속도)

**튜닝 팁**

* 정면 충돌 억제 강하게 → `a3_heading`↑.
* 저속 근접에서 과민 → `a2_invd`↓.
* 속도 추정 노이즈 크면 `a4_vclosing`↓.

---

# 4) 양보(Yield) 우선순위 가중치

* `b1_mode` (double, `0.7`) : 상대가 `mode=="manual"`일 때 가중(사람 수동 운전 양보)
* `b2_rowgap` (double, `0.9`) : **right_of_way_score** 차이(상대 ROW 우세)
* `b3_reroute` (double, `0.8`) : 상대가 `reroute` 중이면 양보
* `b4_pathsearch` (double, `0.4`) : 상대가 `STATUS_PATH_SEARCHING`이면 양보
* `b5_occupancy` (double, `0.5`) : 상대가 영역/차선 점유 플래그면 양보
* `b6_id` (double, `0.2`) : ID 타이브레이커(작은 ID 우선)
* `kappa` (double, `0.6`) : 최종 `score = severity * (1 + kappa * yprio)`에서 yprio 영향도

**튜닝 팁**

* “수동/특수 로봇에 항상 양보” → `b1_mode`↑, `kappa`↑.
* “우선도로/차로 규칙 강화” → `b2_rowgap`↑, `b5_occupancy`↑.
* 교착 시 reroute 우대 → `b3_reroute`↑.

---

# 5) 진입 임계(Enter thresholds)

* `T_slow` (double, `6.0`) : **SLOWDOWN** 진입 임계(T_eff < T_slow)
* `T_yield` (double, `2.5`) : **YIELD** 진입 임계(T_eff < T_yield)
* `yield_priority_thresh` (double, `0.8`) : yprio ≥ 이 값이면 **YIELD 강제**

**튜닝 팁**

* 너무 자주 YIELD로 떨어지면 `T_yield`↓ 혹은 `yield_priority_thresh`↑.
* 부드러운 감속 선호 → `T_slow`를 `T_yield` 대비 충분히 크게(예: 5~8s).

---

# 6) 복귀 히스테리시스(Resume)

* `T_resume_slow` (double, `6.5`)
* `T_resume_yield` (double, `3.5`)
* `T_resume_stop`  (double, `5.0`)
  → 현재 상태(SLOW/YIELD/STOP)별 **안전 여유**(T_eff) 기준.
* `Y_exit` (double, `0.5`) : yprio가 이 값 **미만**이어야 복귀 가능.
* `K_slow_clean` (int, `2`), `K_yield_clean` (int, `3`), `K_stop_clean` (int, `3`)
  → 안전 기준 연속 만족 횟수. 충족 시 RUN으로 복귀.

**튜닝 팁**

* 출렁임(진입↔복귀 반복) 있으면: `T_resume_*` **살짝↑** 또는 `K_*_clean` **1~2 증가**.
* 복귀가 느리면: `T_resume_*`↓ 또는 `Y_exit`↑.

---

# 7) Idle/Timeout 복귀

* `resume_idle_sec` (double, `1.5`)
  최근 에이전트 이벤트가 **없었던** 시간(충돌 메시지 무소식)≥ 이면 **SLOW/YIELD**는 RUN 복귀.
* `resume_timeout_slow` (double, `6.0`)
* `resume_timeout_yield` (double, `10.0`)
* `resume_timeout_stop` (double, `15.0`)
  → 상태 유지 **하드 타임아웃**. 초과 시 RUN 강제.

**튜닝 팁**

* 이벤트 드롭/유실 환경이면 타임아웃을 **짧게** 가져가서 정체 방지.
* 교차로 대기 많은 환경이면 YIELD 타임아웃을 STOP보다 **짧게** 설정해 순환 유도.

---

# 8) 디바운스/무시 윈도우

* `agent_event_silence_sec` (double, `1.0`)
  같은 에이전트로부터의 연속 이벤트 무시 시간(스팸 억제).
* `replan_ignore_sec_after_agent` (double, `0.5`)
  **에이전트 이벤트 직후** 외부 `/replan_flag`를 무시할 윈도우.
* `run_pulse_silence_sec` (double, `0.5`)
  `/cmd/run` 펄스 과다 발행 방지.

**튜닝 팁**

* PathValidator가 자주 치는 환경 → `agent_event_silence_sec` 0.8~1.2s.
* 상위 플래너(글로벌)와 충돌 리플랜 충돌 시 → `replan_ignore_sec_after_agent` 0.3~0.8s.

---

# 9) 토픽(입출력)

입력

* `topic_collision` (str, `"/path_agent_collision_info"`) : PathValidator 충돌 후보
* `topic_agents` (str, `"/multi_agent_infos"`) : 최신 에이전트 상태
* `topic_replan_flag` (str, `"/replan_flag"`) : 외부 리플랜 트리거

상태/요청 출력

* `topic_decision_state` (str, `"/decision_state"`) : 문자열 상태(RUN/SLOWDOWN/YIELD/STOP/REPLAN/REROUTE)
* `topic_request_replan` (str, `"/request_replan"`) : Bool 1회 펄스
* `topic_request_reroute` (str, `"/request_reroute"`) : Bool 1회 펄스

실행 커맨드(펄스)

* `topic_cmd_run` (str, `"/cmd/run"`)
* `topic_cmd_slowdown` (str, `"/cmd/slowdown"`)
* `topic_cmd_yield` (str, `"/cmd/yield"`)
* `topic_cmd_stop` (str, `"/cmd/stop"`)

**가이드**

* 상위 BT/컨트롤러가 **엣지 트리거**라면 펄스 방식이 맞음.
* Topic QoS는 기본 keep last 10로 충분. 중요 신호는 상위에서 Latched/Transient 처리.

---

# 10) 내부 규칙·상호작용 요점

* 의사결정 규칙(간략):

  1. `T_eff < T_yield` **또는** `yprio ≥ yield_priority_thresh` → **YIELD**
  2. `T_eff < T_slow` → **SLOWDOWN**
  3. 상대가 reroute이고 `yprio` 높음 → **REROUTE**
  4. 기타 → **RUN**
* 복귀는 `_maybe_resume_on_clean()`가 **T_resume_*** + `Y_exit` + `K_*_clean`으로 히스테리시스.

**AgentLayer/PathValidator와의 정합**

* PathValidator가 `PathAgentCollisionInfo`를 **간헐적으로** 내보내므로, 여기서 **디바운스**와 **히스테리시스**로 안정화.
* AgentLayer의 코스트 설정이 높아질수록 PathValidator의 이벤트 빈도↑ → 여기선 `T_slow/T_yield`를 **조금 올려** 불필요한 YIELD 진입 억제 가능.

---

## 상황별 튜닝 레시피

### A) 협소 교차·저속(0.4~0.6 m/s) – 보수적·안전 우선

* TTC 결합: `w1_ttc=1.0, w2_alt=0.8, T_min=0.6, d_min=0.25`
* Severity: `a1=1.3, a2=0.25, a3=0.6, a4=0.25`
* Yield: `b1=0.8, b2=1.0, b3=0.7, b4=0.4, b5=0.6, b6=0.2, kappa=0.7`
* Enter: `T_yield=3.0, T_slow=7.0, yield_priority_thresh=0.8`
* Resume: `T_resume_yield=4.0, T_resume_slow=7.5, Y_exit=0.5, K_yield_clean=3`
* Idle/Timeout: `resume_idle_sec=1.5, resume_timeout_yield=10`

### B) 개방 공간·중속(0.8~1.2 m/s) – 효율·흐름 우선

* TTC 결합: `w1_ttc=0.9, w2_alt=1.0, T_min=0.5, d_min=0.2`
* Severity: `a1=1.1, a2=0.15, a3=0.4, a4=0.35`
* Yield: `b1=0.5, b2=0.7, b3=0.6, b4=0.3, b5=0.4, b6=0.2, kappa=0.5`
* Enter: `T_yield=2.2~2.5, T_slow=5.5~6.5, yield_priority_thresh=0.9`
* Resume: `T_resume_yield=3.0, T_resume_slow=6.5, Y_exit=0.55, K_*_clean=2`
* Idle/Timeout: `resume_idle_sec=1.0, resume_timeout_yield=8`

### C) 이벤트 스팸/노이즈 심함 – 안정성 강화

* 디바운스: `agent_event_silence_sec=1.2`, `run_pulse_silence_sec=0.7`
* 히스테리시스: `K_yield_clean=4`, `K_slow_clean=3`, `Y_exit=0.45`
* 타임아웃: `resume_timeout_yield=9~12` (환경에 맞춰)

---

## 권장 YAML 스니펫 (균형형)

```yaml
fleet_decision_node:
  ros__parameters:
    global_frame: map
    my_machine_id: 3

    # TTC combine
    w1_ttc: 1.0
    w2_alt: 0.8
    T_min: 0.6
    d_min: 0.25

    # Severity
    a1_invT: 1.2
    a2_invd: 0.2
    a3_heading: 0.5
    a4_vclosing: 0.3

    # Yield priority
    b1_mode: 0.7
    b2_rowgap: 0.9
    b3_reroute: 0.8
    b4_pathsearch: 0.4
    b5_occupancy: 0.5
    b6_id: 0.2
    kappa: 0.6

    # Enter thresholds
    T_slow: 6.0
    T_yield: 2.5
    yield_priority_thresh: 0.8

    # Resume hysteresis
    T_resume_slow: 6.5
    T_resume_yield: 3.5
    T_resume_stop: 5.0
    Y_exit: 0.5
    K_slow_clean: 2
    K_yield_clean: 3
    K_stop_clean: 3

    # Idle/Timeout resume
    resume_idle_sec: 1.5
    resume_timeout_slow: 6.0
    resume_timeout_yield: 10.0
    resume_timeout_stop: 15.0

    # Debounce / ignore windows
    agent_event_silence_sec: 1.0
    replan_ignore_sec_after_agent: 0.5
    run_pulse_silence_sec: 0.5

    # Topics
    topic_collision: /path_agent_collision_info
    topic_agents: /multi_agent_infos
    topic_replan_flag: /replan_flag
    topic_decision_state: /decision_state
    topic_request_replan: /request_replan
    topic_request_reroute: /request_reroute
    topic_cmd_run: /cmd/run
    topic_cmd_slowdown: /cmd/slowdown
    topic_cmd_yield: /cmd/yield
    topic_cmd_stop: /cmd/stop
```

---

## 디버깅·검증 체크리스트

* **프레임/ID:** `global_frame` 일치, `my_machine_id` 정확.
* **충돌 이벤트 수신률:** PathValidator의 `/path_agent_collision_info`가 **지나치게 빈번**하면 이 노드에서는 **YIELD 과다** → PathValidator의 `agent_path_hit_stride/dilate` 먼저 조정.
* **출렁임:** 상태가 자주 바뀌면 `T_resume_*`↑ `K_*_clean`↑, 혹은 `yield_priority_thresh`↑.
* **양보 불균형:** 특정 로봇만 계속 YIELD → `b2_rowgap`, `b6_id`, `kappa` 재조정(ROW/ID 영향점).
* **REPLAN/REROUTE 경쟁:** `replan_ignore_sec_after_agent`로 충돌 직후 외부 리플랜 억제.

---

원하면 너희 **AgentLayer/PathValidator 현재 세팅**을 알려줘. 그 조합에 맞춰 `T_*`, `Y_*`, `kappa`, `b*`를 더 정밀하게 **두세 가지 운영 프로파일**로 뽑아줄게 (보수형/균형형/공격형).
###################  
####################  
#####################  
좋아, “어디를 어떻게 바꾸면 되는지” **딱 필요한 만큼** 정확히 정리해 줄게.
핵심은 ① 에이전트 마스크 조회를 **월드좌표 변환 기반**으로 바꾸는 것, ② 히트 지점의 **이웃 셀도 함께 확인**해서 경계 누락을 줄이는 것, ③ 파라미터를 **해상도 기준**으로 재설정하는 것 — 이 3가지야.

---

# 1) `agentCellBlockedNear()` 교체 (가장 중요)

현재는 master 인덱스를 agent_mask 인덱스로 **그냥 재사용**해서 맵 기준이 어긋나면 놓쳐요.
아래처럼 **master(mx,my) → (wx,wy) → agent(ax,ay)** 변환을 거치도록 **함수 전체를 교체**하세요.

```cpp
// PathValidatorNode::agentCellBlockedNear() 교체 버전
inline bool PathValidatorNode::agentCellBlockedNear(
    unsigned int mx, unsigned int my,
    unsigned char thr, int manhattan_buf) const
{
  // 1) 로컬 복사 (락 순서 교착 회피)
  std::shared_ptr<nav2_costmap_2d::Costmap2D> master, agent;
  {
    std::lock_guard<std::mutex> lk(costmap_mutex_);
    if (!costmap_) return false;
    master = costmap_;
  }
  {
    std::lock_guard<std::mutex> lk(agent_mask_mutex_);
    if (!agent_mask_) return false;
    agent = agent_mask_;
  }

  // 2) master index -> world
  double wx, wy;
  master->mapToWorld(mx, my, wx, wy);

  // 3) world -> agent_mask index
  unsigned int ax, ay;
  if (!agent->worldToMap(wx, wy, ax, ay)) {
    return false;
  }

  // 4) 맨해튼 버퍼 내에서 검사
  const int sx = static_cast<int>(agent->getSizeInCellsX());
  const int sy = static_cast<int>(agent->getSizeInCellsY());
  const int ix = static_cast<int>(ax);
  const int iy = static_cast<int>(ay);

  for (int dx = -manhattan_buf; dx <= manhattan_buf; ++dx) {
    for (int dy = -manhattan_buf; dy <= manhattan_buf; ++dy) {
      if (std::abs(dx) + std::abs(dy) > manhattan_buf) continue;
      const int x = ix + dx;
      const int y = iy + dy;
      if (x < 0 || y < 0 || x >= sx || y >= sy) continue;

      const unsigned char a = agent->getCost(static_cast<unsigned int>(x),
                                             static_cast<unsigned int>(y));
      if (a >= thr) return true;
    }
  }
  return false;
}
```

* 이 수정만으로 `agent_mask_manhattan_buffer`를 **1~3** 정도의 정상값으로 내려도 잘 잡힙니다.
* 더 이상 200 같은 **비정상 큰 버퍼**는 필요 없어요.

---

# 2) 히트 셀의 **이웃도 함께 검사** (경계 픽셀 보강) — 선택적이지만 추천

`validateWithFootprint()`와 `validateWithPoints()`에서 `blocked_here == true`일 때,
바로 `whoCoversPoint(wx,wy)`만 보지 말고 **주변 1셀**의 월드 좌표도 같이 확인하면 경계 누락이 크게 줄어요.

아래 보강 블록을 두 곳 모두에 같은 방식으로 넣으면 됩니다.

```cpp
// blocked_here가 true인 지점에서 wx, wy를 구한 다음:
bool agent_hit = false;

// 중심점 우선
auto hits = whoCoversPoint(wx, wy);
if (!hits.empty()) {
  publishAgentCollisionList(hits);
  last_agent_block_time_ = this->now();
  return;
}

// 이웃 1셀까지 확장 확인 (선택, 추천)
{
  std::shared_ptr<nav2_costmap_2d::Costmap2D> cm;
  { std::lock_guard<std::mutex> lk(costmap_mutex_); cm = costmap_; }
  if (cm) {
    const int OFFS[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
    for (auto &o : OFFS) {
      double wxx, wyy;
      cm->mapToWorld(static_cast<int>(hit_mx)+o[0], static_cast<int>(hit_my)+o[1], wxx, wyy);
      auto h2 = whoCoversPoint(wxx, wyy);
      if (!h2.empty()) {
        publishAgentCollisionList(h2);
        last_agent_block_time_ = this->now();
        return;
      }
    }
  }
}
```

> 구현 위치:
>
> * `validateWithFootprint()`에서 `if (blocked_here) { ... }` 내부
> * `validateWithPoints()`에서 `if (blocked_cell) { ... }` 내부

---

# 3) 파라미터 **정상화(해상도 기준)**

큰 값(2.0 / 200)에 의존하지 않도록, 아래처럼 **맵 해상도(res)** 기준으로 잡아주세요.
(일반적으로 res=0.05m 가정 시 권장값 예시 포함)

* `agent_mask_manhattan_buffer`: **1~3**
  (위 1)수정 반영이 전제. 해상도 5cm면 1~2로도 충분)
* `agent_match_dilate_m`: **max(2*res, 0.12)** ⇒ res=0.05 → **0.12~0.15**
  (footprint 포함 판정의 경계손실 방지)
* `agent_path_hit_dilate_m`: **max(1*res, 0.05)** ⇒ res=0.05 → **0.05~0.08**
  (튜브가 셀보다 얇지 않게)
* `agent_path_hit_stride_m`: **0.25~0.40** (현 0.35는 OK)

추가로, 의도에 따라 코스트 조정:

* AgentLayer가 지금 `moving_cost=254`, `waiting_cost=254`로 **항상 치사 코스트**예요.
  PathValidator의 `cost_threshold=254`와 같아서 민감하게 막힙니다.
  “가까울 때만 막히게” 하려면:

  * AgentLayer: `moving_cost=180~200`, `waiting_cost=150~180`
  * PathValidator: `agent_cost_threshold`를 **moving_cost와 동일** 또는 약간 낮게

---

# 4) (옵션) `whoCoversPoint()` 내부의 최소 팽창 하한

파라미터 실수에 대비해, 내부에서 **최소 팽창 하한**을 한 줄 넣어두면 더 안전합니다.
해상도를 함수 안에서 모르면 보수적으로 5cm 하한만 둬도 좋아요.

```cpp
double eff_match = std::max(agent_match_dilate_m_, 0.05);        // 5cm 하한
double eff_path  = std::max(agent_path_hit_dilate_m_, 0.05);

// 이후 poly_local 확장 시 eff_match / eff_path 사용
```

---

# 최종 체크리스트

1. **함수 교체**: `agentCellBlockedNear()`를 월드좌표 변환 버전으로 바꾼다.
2. **이웃 보강**: `validateWithFootprint()` / `validateWithPoints()`의 agent 확인 지점에 **이웃 1셀 검사** 블록 추가.
3. **파라미터 정상화**: 위 권장 범위로 조정 (특히 `agent_mask_manhattan_buffer`, `agent_match_dilate_m`, `agent_path_hit_dilate_m`).
4. (선택) **최소 팽창 하한** 한 줄 추가해 실수 방어.
5. (선택) **AgentLayer 코스트 정책**을 의도에 맞춰 조정.

이렇게만 적용하면, **작은(정상) 파라미터 값**에서도 agent 충돌이 **안정적으로 agent로 분류**되고 더 이상 200/2.0 같은 비정상값에 의존하지 않아도 됩니다.
#################  
####################  
#####################  
좋아. 두 군데 모두 **해당 블록만** 바꿔 끼울 수 있게 “붙여 넣기용” 코드를 딱 줘볼게. 핵심은

* 중심 셀 외에 **이웃 1셀(8방향)** 의 월드좌표도 함께 `whoCoversPoint()`에 물어보는 **보강 검사**를 넣는 것,
* (옵션) Agent mask로 잡힌 경우에도 **같은 보강 검사**를 돌려서 경계 누락을 줄이는 것.

아래 코드를 그대로 교체하면 돼.

---

## 1) `validateWithFootprint()` 안의 `if (blocked_here) { ... }` 교체

**기존 블록 전체**를 아래로 교체하세요.

```cpp
if (blocked_here) {
  // --- costmap 포인터를 잠깐 복사 (락 최소화) ---
  std::shared_ptr<nav2_costmap_2d::Costmap2D> cm;
  { std::lock_guard<std::mutex> lock(costmap_mutex_); cm = costmap_; }
  if (!cm) return;

  // --- 중심 셀(히트셀) 기준 월드좌표 ---
  double wx, wy;
  cm->mapToWorld(static_cast<int>(hit_mx), static_cast<int>(hit_my), wx, wy);

  // --- 중심+이웃 1셀까지 에이전트 히트 검사 람다 ---
  auto agent_hit_around = [&](double cx, double cy,
                              unsigned int mx_c, unsigned int my_c) -> std::vector<AgentHit> {
    // 1) 중심 먼저
    auto hits = whoCoversPoint(cx, cy);
    if (!hits.empty()) return hits;

    // 2) 8방향 이웃
    static const int OFFS[8][2] = {
      { 1, 0},{-1, 0},{ 0, 1},{ 0,-1},
      { 1, 1},{ 1,-1},{-1, 1},{-1,-1}
    };
    for (auto &o : OFFS) {
      double wxx, wyy;
      cm->mapToWorld(static_cast<int>(mx_c)+o[0], static_cast<int>(my_c)+o[1], wxx, wyy);
      auto h2 = whoCoversPoint(wxx, wyy);
      if (!h2.empty()) return h2;
    }
    return {};
  };

  // 1) 일반 코스트로 막혔을 때: 중심+이웃 검사
  {
    auto hits = agent_hit_around(wx, wy, hit_mx, hit_my);
    if (!hits.empty()) {
      publishAgentCollisionList(hits);
      last_agent_block_time_ = this->now();
      return;
    }
  }

  // 2) (보조) agent mask가 있으면 동일한 보강 검사 재시도
  if (compare_agent_mask_) {
    const bool agent_mark = agentCellBlockedNear(
        hit_mx, hit_my,
        static_cast<unsigned char>(agent_cost_threshold_),
        agent_mask_manhattan_buffer_);
    if (agent_mark) {
      auto hits2 = agent_hit_around(wx, wy, hit_mx, hit_my);
      if (!hits2.empty()) {
        publishAgentCollisionList(hits2);
        last_agent_block_time_ = this->now();
        return;
      }
    }
  }

  // 3) 여기까지 에이전트가 아니면 일반 장애물로 누적 처리
  consecutive++;
  if (consecutive >= consecutive_threshold_) {
    triggerReplan("blocked (footprint) streak threshold reached");
    return;
  }
} else {
  consecutive = 0;
}
```

---

## 2) `validateWithPoints()` 안의 `if (blocked_cell) { ... }` 교체

**기존 블록 전체**를 아래로 교체하세요.

```cpp
if (blocked_cell) {
  // --- costmap 포인터 복사 ---
  std::shared_ptr<nav2_costmap_2d::Costmap2D> cm;
  { std::lock_guard<std::mutex> lock(costmap_mutex_); cm = costmap_; }
  if (!cm) return;

  // --- 중심 셀 월드좌표 ---
  double wx, wy;
  cm->mapToWorld(static_cast<int>(mx), static_cast<int>(my), wx, wy);

  // --- 중심+이웃 1셀까지 에이전트 히트 검사 람다 ---
  auto agent_hit_around = [&](double cx, double cy,
                              unsigned int mx_c, unsigned int my_c) -> std::vector<AgentHit> {
    // 1) 중심
    auto hits = whoCoversPoint(cx, cy);
    if (!hits.empty()) return hits;

    // 2) 8방향 이웃
    static const int OFFS[8][2] = {
      { 1, 0},{-1, 0},{ 0, 1},{ 0,-1},
      { 1, 1},{ 1,-1},{-1, 1},{-1,-1}
    };
    for (auto &o : OFFS) {
      double wxx, wyy;
      cm->mapToWorld(static_cast<int>(mx_c)+o[0], static_cast<int>(my_c)+o[1], wxx, wyy);
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
      return; // 에이전트 충돌 확정
    }
  }

  // 2) (보조) agent mask 기준으로도 보강 검사
  if (compare_agent_mask_) {
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

  // 3) 에이전트 히트가 아니면 이후의 일반 장애물 로직으로 진행
}
```

> 위 “보강 검사”는 **중심점만 보던 기존 로직** 대비, 실제 겹침 위치가 **셀 경계/꼭짓점**에 걸린 경우도 안정적으로 agent로 분류해 줍니다.
> (참고) `agentCellBlockedNear()`는 이전 답변처럼 **월드→에이전트 맵 변환** 버전으로 교체해 두면 `agent_mask_manhattan_buffer`를 1~3의 정상값으로 낮춰도 잘 동작합니다.

---

필요한 수정은 여기까지예요.
그대로 붙여 넣고 빌드하면, 작은 파라미터에서도 agent 충돌이 **일반 장애물로 오판되는 문제**가 현저히 줄어들 거예요.
