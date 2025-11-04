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
