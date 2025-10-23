좋아—최신 스펙(“agent” 용어, `nav_msgs/Path`로 바뀐 `truncated_path`)을 반영해서 **데이터 플로우**와 **전체 커스텀 메시지 세트**를 한 번에 정리해줄게. (ROS 2 Jazzy 기준)

---

# 전체 데이터 플로우(상→하: 의사결정 / 하→상: 데이터)

```
[External Fleet / Other Agents]
        │  (원자료: pose, mode, waypoints, area, etc.)
        ▼
┌───────────────────────────┐
│ Aggregator (배합/정규화)  │
│ - 입력 표준화/클리닝       │
│ - ROI/주파수 제한         │
│ - MultiAgentInfoArray pub │
└─────────────┬─────────────┘
              │  multi_agent_infos (MultiAgentInfoArray)
              ▼
┌───────────────────────────┐
│ AgentLayer (Costmap plugin)│
│ - swept-footprint 래스터   │
│ - status/mode 가중/감쇠    │
│ - agent_layer_meta pub     │
└─────────────┬─────────────┘
              │  agent_layer_meta (AgentLayerMetaArray)
              │
              │      ┌──────────────────────────────────────────┐
              │      │ Nav2: Global/Local Costmap/Planner/CTL  │
              │      │ - AgentLayer를 포함한 costmap 사용      │
              │      └──────────────────────────────────────────┘
              │
              ▼
┌───────────────────────────┐
│ PathValidator             │
│ - global costmap로 경로   │
│   차단 감지               │
│ - agent_layer_meta 대조   │
│   → ENV vs AGENT 분류     │
│ - 이벤트 발행             │
│   (ENV: /replan_flag)     │
│   (AGENT: PathAgent…Event)│
└─────────────┬─────────────┘
              │  /path_agent_conflict_event, /replan_flag
              ▼
┌───────────────────────────┐
│ Conflict Decider          │  ← 최상위 정책/FSM
│ - 시나리오 분류/우선권     │
│ - area(협소/교차) 처리     │
│ - 행동결정: WAIT/REPLAN…   │
│ - Mask pub (keepout/speed) │
└─────────────┬─────────────┘
              │  /decision_cmd, /planner/keepout_mask, /controller/speed_limit_mask
              ▼
      Nav2 Planner/Controller 가 즉시 반영
```

* **중요**: `nav_msgs/Path` 타입의 `truncated_path`는 **`header.stamp`가 존재**하므로, 소비 측(AgentLayer/Decider)은 신선도 체크(`now - stamp`)와 frame 일관성(`frame_id == map`)을 확인해서 사용하면 돼.

---

# 커스텀 메시지 스펙 (최종)

## 1) 상태/공유 계층

### `msg/AgentStatus.msg`

```text
# 상위 상태(요구 테이블 그대로 매핑)
uint8 STATUS_INIT                         = 0
uint8 STATUS_PATH_SEARCHING               = 1
uint8 STATUS_WAITING_FOR_OBS              = 2
uint8 STATUS_MOVING                       = 3
uint8 STATUS_ARRIVED                      = 4
uint8 STATUS_MARKING                      = 5
uint8 STATUS_UNLOADING                    = 6
uint8 STATUS_UNLOADED                     = 7
uint8 STATUS_LOADING                      = 8
uint8 STATUS_LOADED                       = 9
uint8 STATUS_AUTORECOVERY                 = 10
uint8 STATUS_RECOVERING                   = 11
uint8 STATUS_MANUAL_RUNNING               = 12
uint8 STATUS_MANUAL_COMPLETE              = 13
uint8 STATUS_ERROR                        = 14
uint8 STATUS_PAUSE                        = 15
uint8 STATUS_WAITING_FOR_SAFETY           = 16
uint8 STATUS_WAITING_FOR_FLOWCONTROL      = 17
uint8 STATUS_WAITING_FOR_ROS_STATUS       = 18
uint8 STATUS_CHARGING                     = 19
uint8 STATUS_CHARGE_DONE                  = 20
uint8 STATUS_UNKNOWN                      = 21

uint8 phase
```

### `msg/MultiAgentInfo.msg`

```text
# frame_id: 보통 "map"
std_msgs/Header header

# --- 식별자 ---
uint16 machine_id
string type_id

# --- 현재 상태 ---
geometry_msgs/PoseStamped current_pose
# 현재 명령 속도(cmd_vel). 기본 프레임: base_link
geometry_msgs/Twist       current_twist

# 발자국(다각형)
geometry_msgs/PolygonStamped footprint

# 상위 상태
multi_agent_msgs/AgentStatus status

bool   reroute

# 웨이포인트(0~255)
uint8  current_waypoint
uint8  next_waypoint

# 0~5 m 프리뷰 (타임스탬프/프레임 포함)
# truncated_path.header.frame_id 는 MultiAgentInfo.header.frame_id 와 동일해야 함
nav_msgs/Path truncated_path

# --- 상호 관계/교차 인지 ---
uint16[] cross_agent_ids

# --- 운용 컨텍스트 ---
string mode

# --- 에어리어(협소/교차 섹션 등) ---
uint16 area_id
bool   occupancy

# --- 작업/리플랜 상태 플래그 ---
bool   transferring
bool   re_path_search

# --- 품질/불확실도 ---
float32 pos_std_m      # XY 위치 표준편차 [m], unknown이면 -1
float32 yaw_std_rad    # 요 표준편차 [rad], unknown이면 -1
```

### `msg/MultiAgentInfoArray.msg`

```text
std_msgs/Header header             # stamp: 생성 기준 시각, frame_id: map
multi_agent_msgs/MultiAgentInfo[] agents
```

> `pos_std_m/yaw_std_rad`는 Cartographer만 사용할 때 잔차 기반 추정(불가 시 -1)으로 운용.

---

## 2) 코스트맵 메타 계층 (AgentLayer 출력)

### `msg/AgentLayerCellMeta.msg`

```text
std_msgs/Header header              # frame_id: map

uint16 machine_id
uint8  phase                        # AgentStatus.phase 값
string mode                         # "auto"/"manual"/...
bool   reroute
bool   re_path_search
bool   transferring

uint16 area_id                      # 없으면 0

# 위치 (world 또는 grid index; 둘 다 있으면 world 우선)
geometry_msgs/Point position        # 월드 좌표[m] (셀 센터)
uint32 mx
uint32 my

# 시간/불확실도
float32 t_first_hit                 # [s] 최초 점유/교차 예상 시각(모르면 음수)
float32 sigma                       # [m] 불확실도 등가 반경
```

### `msg/AgentLayerMetaArray.msg`

```text
std_msgs/Header header              # frame_id: map
multi_agent_msgs/AgentLayerCellMeta[] cells
```

> **희소 발행** 권장: 의미 있는 셀(충돌 가능/LETHAL 근처/area 경계 등)만 포함.

---

## 3) 의사결정 이벤트/명령 계층

### `msg/PathAgentConflictEvent.msg`

```text
std_msgs/Header header                     # stamp: 판정 시각, frame_id: map
string path_id

# 관련 agent들
uint16[] machine_ids
uint8[]  phases
bool     is_manual_any
bool     is_reroute_any
bool     is_re_path_search_any
bool     transferring_any

# 협소/교차 area
uint16   shared_area_id                    # 동일 area면 값, 아니면 0
bool     area_occupancy_any

# 수치 요약
float32  ttc_first
float32  conflict_length_m
float32  min_distance_m
float32  confidence

geometry_msgs/PoseStamped first_conflict_pose
string   source_layer                      # 예: "global_costmap/agent_layer"
float32  meta_staleness_ms
string   note
```

### `msg/PathAgentConflictState.msg`

```text
std_msgs/Header header
bool active
multi_agent_msgs/PathAgentConflictEvent last_event
```

### `msg/DecisionCmd.msg`

```text
uint8 CMD_GO=0
uint8 CMD_WAIT=1
uint8 CMD_SLOWDOWN=2
uint8 CMD_REPLAN_SOFT=3
uint8 CMD_REPLAN_WITH_KEEP_OUT=4

std_msgs/Header header
uint8  cmd
string reason
```

### `msg/AreaPolygon.msg`

```text
std_msgs/Header header          # frame_id: map
uint16 area_id
geometry_msgs/Polygon polygon
```

### `msg/KeepoutMask.msg`

```text
std_msgs/Header header          # frame_id: map
geometry_msgs/Polygon[] polygons
float32 dilation_m
float32 ttl_sec
string  source
```

### `msg/SpeedLimitMask.msg`

```text
std_msgs/Header header          # frame_id: map (또는 local)
geometry_msgs/Polygon[] regions
float32 v_max
float32 ttl_sec
string  source
```

### `srv/ApplyKeepoutMask.srv` (선택)

```text
multi_agent_msgs/KeepoutMask request
---
bool accepted
string message
```

---

# 권장 토픽 & QoS

* `multi_agent_infos` (`MultiAgentInfoArray`): **reliable + volatile**, 5–10 Hz
* `agent_layer_meta` (`AgentLayerMetaArray`): **reliable + transient_local + KeepLast(1)**
* `path_agent_conflict_event`: **reliable + volatile + KeepLast(10)**
* `path_agent_conflict_state`: **reliable + transient_local + KeepLast(1)**
* `decision_cmd`: **reliable + volatile + KeepLast(10)**
* `planner/keepout_mask`, `controller/speed_limit_mask`: **reliable + transient_local + KeepLast(1)**

---

## `truncated_path`(nav_msgs/Path) 사용 주의

* `Path.header.stamp`는 **해당 프리뷰의 생성 시각**임. 소비자는 `now - stamp > τ_stale`이면 낡은 데이터로 간주(예: τ=0.5–1.0 s).
* `Path.header.frame_id`는 `MultiAgentInfo.header.frame_id`와 같아야 하며, 일반적으로 `map`.
* 포즈 해상도: 0.1–0.2 m 간격(또는 약 0.2 s) 권장—과도한 점수는 비용.

---

필요하면 이 구조로 바로 빌드 가능한 **패키지 스켈레톤**(폴더+빈 .msg/.srv)과 **C++ 퍼블/서브 예제**도 만들어줄게.
#############################################################3  
#############################################################3  
#############################################################3  
좋아. **agent_layer (costmap_2d plugin)**를 “바로 붙여서 돌릴 수 있는” 수준으로 설계+코드 스켈레톤까지 한 번에 정리해줄게.
전제: ROS 2 Jazzy / C++17 / Nav2 / Ubuntu 24.04 / 컨테이너 환경.
입력: `multi_agent_infos (MultiAgentInfoArray)` — **내 자기 자신 데이터는 `machine_id` + `type_id`로 제외**.

아래 제공물:

1. 구조/동작 개요
2. 파라미터 스펙
3. C++17 코드 스켈레톤(hpp/cpp)
4. plugin.xml / CMakeLists.txt / package.xml 수정
5. Nav2 params.yaml 예시(레이어 순서 포함)
6. 성능/정확도 팁 & 테스트 체크리스트

---

# 1) 구조/동작 개요

* 입력: `multi_agent_msgs/MultiAgentInfoArray` (QoS: reliable+volatile 권장)
* 레이어 책임:

  * **다른 에이전트만** 반영(자기 자신 제외: `self_machine_id` + `self_type_id` 일치 시 스킵)
  * 각 에이전트의 `footprint(PolygonStamped)`를 **truncated_path(nav_msgs/Path)** 위에 래스터라이즈하여 **swept-occupancy** 코스트 생성
  * 상태/모드/불확실도에 따른 **가중치/팽창/시간감쇠** 적용
  * (옵션) `agent_layer_meta (AgentLayerMetaArray)` **희소 메타** 퍼블리시(의사결정·디버그용)
* 레이어 순서:
  `static/voxel/nvblox  →  (기타)  →  agent_layer  →  inflation`  (항상 inflation이 마지막)

---

# 2) 파라미터 스펙 (예시)

```yaml
global_costmap:
  plugins:
    - {name: agent_layer, type: multi_agent_nav2::AgentLayer}
  agent_layer:
    enabled: true
    topic: "/multi_agent_infos"
    self_machine_id: 1001              # 내 에이전트 ID
    self_type_id: "CUSTOM_AMR_A"       # 내 타입 ID
    use_path_header_frame: true        # truncated_path.header.frame_id 신뢰
    roi_range_m: 12.0                  # 내 주변 ROI (성능)
    time_decay_sec: 1.0                # 코스트 감쇠 시간
    lethal_cost: 254
    moving_cost: 180
    waiting_cost: 200
    manual_cost_bias: 30               # mode=="manual"이면 cost 가산
    dilation_m: 0.05                   # footprint 외곽 추가 팽창
    forward_smear_m: 0.25              # 진행방향 비대칭 스미어
    sigma_k: 2.0                        # r_safe = r_geom + k*pos_std_m
    publish_meta: true
    meta_stride: 3                     # 메타 샘플링(셀 stride)
    freshness_timeout_ms: 800          # truncated_path/pose 신선도 체크
    max_poses: 40                      # truncated_path 상 최대 샘플(성능 캡)
    qos_reliable: true
```

> 코스트 함수 대략:
> `base = (status == MOVING ? moving_cost : waiting_cost)`
> `cost = clamp(base + (mode==manual? manual_cost_bias:0), 0..254)`

---

# 3) C++17 코드 스켈레톤

### 파일: `include/multi_agent_nav2/agent_layer.hpp`

```cpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav_msgs/msg/path.hpp>
#include <multi_agent_msgs/msg/multi_agent_info_array.hpp>
#include <multi_agent_msgs/msg/agent_layer_meta_array.hpp>

namespace multi_agent_nav2
{

class AgentLayer : public nav2_costmap_2d::Layer
{
public:
  AgentLayer();
  void onInitialize() override;
  void reset() override { current_ = true; }
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;
  bool isClearable() override { return true; }
  void deactivate() override;
  void activate() override;

private:
  // ---- data / params ----
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Subscription<multi_agent_msgs::msg::MultiAgentInfoArray>::SharedPtr sub_;
  rclcpp::Publisher<multi_agent_msgs::msg::AgentLayerMetaArray>::SharedPtr meta_pub_;

  std::mutex data_mtx_;
  multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr last_infos_;
  rclcpp::Time last_stamp_;

  bool enabled_{true};
  std::string topic_;
  uint16_t self_machine_id_{0};
  std::string self_type_id_;
  bool use_path_header_frame_{true};
  double roi_range_m_{12.0};
  double time_decay_sec_{1.0};
  unsigned char lethal_cost_{254};
  unsigned char moving_cost_{180};
  unsigned char waiting_cost_{200};
  int manual_cost_bias_{30};
  double dilation_m_{0.05};
  double forward_smear_m_{0.25};
  double sigma_k_{2.0};
  bool publish_meta_{true};
  int meta_stride_{3};
  int freshness_timeout_ms_{800};
  int max_poses_{40};
  bool qos_reliable_{true};

  // cached bounds of this cycle
  double touch_min_x_, touch_min_y_, touch_max_x_, touch_max_y_;
  bool touched_{false};

  // ---- helpers ----
  void infosCallback(const multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr msg);
  bool isSelf(const multi_agent_msgs::msg::MultiAgentInfo & a) const;
  bool stale(const rclcpp::Time & stamp) const;

  void rasterizeAgentPath(const multi_agent_msgs::msg::MultiAgentInfo & a,
                          nav2_costmap_2d::Costmap2D * grid,
                          std::vector<std::pair<unsigned int,unsigned int>> & meta_hits);

  void fillFootprintAt(const geometry_msgs::msg::PolygonStamped & fp,
                       const geometry_msgs::msg::Pose & pose,
                       double extra_dilation_m,
                       nav2_costmap_2d::Costmap2D * grid,
                       unsigned char cost,
                       std::vector<std::pair<unsigned int,unsigned int>> * meta_hits = nullptr);

  static bool pointInPolygon(const std::vector<geometry_msgs::msg::Point> & poly,
                             double x, double y);

  unsigned char computeCost(const multi_agent_msgs::msg::MultiAgentInfo & a) const;
  double computeDilation(const multi_agent_msgs::msg::MultiAgentInfo & a) const;
};

} // namespace multi_agent_nav2
```

### 파일: `src/agent_layer.cpp`

```cpp
#include "multi_agent_nav2/agent_layer.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>

PLUGINLIB_EXPORT_CLASS(multi_agent_nav2::AgentLayer, nav2_costmap_2d::Layer)

namespace multi_agent_nav2
{

AgentLayer::AgentLayer() {}

void AgentLayer::onInitialize()
{
  node_ = node_.reset() ? node_ : node_; // no-op to appease linters
  node_ = node_.get() ? node_ : rclcpp_lifecycle::LifecycleNode::SharedPtr(getNode());
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("/multi_agent_infos")));
  declareParameter("self_machine_id", rclcpp::ParameterValue(0));
  declareParameter("self_type_id", rclcpp::ParameterValue(std::string("")));
  declareParameter("use_path_header_frame", rclcpp::ParameterValue(true));
  declareParameter("roi_range_m", rclcpp::ParameterValue(12.0));
  declareParameter("time_decay_sec", rclcpp::ParameterValue(1.0));
  declareParameter("lethal_cost", rclcpp::ParameterValue(254));
  declareParameter("moving_cost", rclcpp::ParameterValue(180));
  declareParameter("waiting_cost", rclcpp::ParameterValue(200));
  declareParameter("manual_cost_bias", rclcpp::ParameterValue(30));
  declareParameter("dilation_m", rclcpp::ParameterValue(0.05));
  declareParameter("forward_smear_m", rclcpp::ParameterValue(0.25));
  declareParameter("sigma_k", rclcpp::ParameterValue(2.0));
  declareParameter("publish_meta", rclcpp::ParameterValue(true));
  declareParameter("meta_stride", rclcpp::ParameterValue(3));
  declareParameter("freshness_timeout_ms", rclcpp::ParameterValue(800));
  declareParameter("max_poses", rclcpp::ParameterValue(40));
  declareParameter("qos_reliable", rclcpp::ParameterValue(true));

  enabled_ = getParameter("enabled").as_bool();
  topic_ = getParameter("topic").as_string();
  self_machine_id_ = static_cast<uint16_t>(getParameter("self_machine_id").as_int());
  self_type_id_ = getParameter("self_type_id").as_string();
  use_path_header_frame_ = getParameter("use_path_header_frame").as_bool();
  roi_range_m_ = getParameter("roi_range_m").as_double();
  time_decay_sec_ = getParameter("time_decay_sec").as_double();
  lethal_cost_ = static_cast<unsigned char>(getParameter("lethal_cost").as_int());
  moving_cost_ = static_cast<unsigned char>(getParameter("moving_cost").as_int());
  waiting_cost_ = static_cast<unsigned char>(getParameter("waiting_cost").as_int());
  manual_cost_bias_ = getParameter("manual_cost_bias").as_int();
  dilation_m_ = getParameter("dilation_m").as_double();
  forward_smear_m_ = getParameter("forward_smear_m").as_double();
  sigma_k_ = getParameter("sigma_k").as_double();
  publish_meta_ = getParameter("publish_meta").as_bool();
  meta_stride_ = getParameter("meta_stride").as_int();
  freshness_timeout_ms_ = getParameter("freshness_timeout_ms").as_int();
  max_poses_ = getParameter("max_poses").as_int();
  qos_reliable_ = getParameter("qos_reliable").as_bool();

  current_ = true;
  enabled_ ? activate() : deactivate();
  default_value_ = nav2_costmap_2d::NO_INFORMATION;
  matchSize();
}

void AgentLayer::activate()
{
  auto qos = qos_reliable_ ? rclcpp::QoS(rclcpp::SensorDataQoS().reliable())
                           : rclcpp::QoS(rclcpp::SensorDataQoS().best_effort());

  sub_ = node_->create_subscription<multi_agent_msgs::msg::MultiAgentInfoArray>(
      topic_, qos,
      std::bind(&AgentLayer::infosCallback, this, std::placeholders::_1));

  if (publish_meta_) {
    meta_pub_ = node_->create_publisher<multi_agent_msgs::msg::AgentLayerMetaArray>(
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
  return (node_->now() - stamp) > rclcpp::Duration::from_nanoseconds(
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
  bool is_moving = (p == S::STATUS_MOVING) || (p == S::STATUS_PATH_SEARCHING);
  unsigned char base = is_moving ? moving_cost_ : waiting_cost_;
  if (a.mode == "manual") {
    int c = static_cast<int>(base) + manual_cost_bias_;
    return static_cast<unsigned char>(std::min(254, std::max(0, c)));
  }
  return base;
}

double AgentLayer::computeDilation(const multi_agent_msgs::msg::MultiAgentInfo & a) const
{
  double r = dilation_m_;
  if (a.pos_std_m >= 0.0) r += sigma_k_ * a.pos_std_m;
  if (forward_smear_m_ > 0.0 && a.status.phase == multi_agent_msgs::msg::AgentStatus::STATUS_MOVING) {
    r += forward_smear_m_;
  }
  return r;
}

void AgentLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
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

  // ROI 필터
  for (auto & a : infos) {
    if (isSelf(a)) continue;

    // 거리 ROI
    double dx = a.current_pose.pose.position.x - robot_x;
    double dy = a.current_pose.pose.position.y - robot_y;
    if (std::hypot(dx, dy) > roi_range_m_) continue;

    // truncated_path 신선도/프레임 체크
    if (a.truncated_path.header.frame_id != layered_costmap_->getGlobalFrameID() && use_path_header_frame_) {
      continue; // 프레임 불일치(옵션)
    }

    // 바운드 확장: truncated_path 범위를 대략 포함
    // (정밀한 바운드는 updateCosts에서 실제로 셀을 칠할 때 갱신)
    for (size_t i = 0; i < a.truncated_path.poses.size() && static_cast<int>(i) < max_poses_; ++i) {
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

void AgentLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                             int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) return;

  std::vector<multi_agent_msgs::msg::MultiAgentInfo> infos;
  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    if (!last_infos_ || stale(last_stamp_)) return;
    infos.assign(last_infos_->agents.begin(), last_infos_->agents.end());
  }

  std::vector<std::pair<unsigned int,unsigned int>> meta_hits;
  for (auto & a : infos) {
    if (isSelf(a)) continue;

    // ROI 거리
    double rx, ry;
    layered_costmap_->getCostmap()->mapToWorld((min_i+max_i)/2, (min_j+max_j)/2, rx, ry); // rough
    double dx = a.current_pose.pose.position.x - rx;
    double dy = a.current_pose.pose.position.y - ry;
    if (std::hypot(dx, dy) > (roi_range_m_ + 2.0)) continue;

    rasterizeAgentPath(a, &master_grid, meta_hits);
  }

  // 희소 메타 퍼블리시
  if (publish_meta_ && meta_pub_) {
    multi_agent_msgs::msg::AgentLayerMetaArray arr;
    arr.header.frame_id = layered_costmap_->getGlobalFrameID();
    arr.header.stamp = node_->now();
    arr.cells.reserve(meta_hits.size()/meta_stride_ + 1);
    for (size_t k = 0; k < meta_hits.size(); k += std::max(1, meta_stride_)) {
      auto [mx, my] = meta_hits[k];
      double wx, wy; master_grid.mapToWorld(mx, my, wx, wy);
      multi_agent_msgs::msg::AgentLayerCellMeta cm;
      cm.header = arr.header;
      // 주의: 아래 메타는 단순화. 실제로는 셀마다 유래 agent를 넣으려면 rasterize 시점에 also capture 필요.
      cm.machine_id = 0;  // 간략화: 필요시 확장
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

void AgentLayer::rasterizeAgentPath(
  const multi_agent_msgs::msg::MultiAgentInfo & a,
  nav2_costmap_2d::Costmap2D * grid,
  std::vector<std::pair<unsigned int,unsigned int>> & meta_hits)
{
  if (a.truncated_path.poses.empty()) return;

  const unsigned char cost = computeCost(a);
  const double extra = computeDilation(a);

  const auto & fp = a.footprint;
  const int limit = std::min<int>(a.truncated_path.poses.size(), max_poses_);

  for (int i = 0; i < limit; ++i) {
    const auto & ps = a.truncated_path.poses[i].pose;
    fillFootprintAt(fp, ps, extra, grid, cost, &meta_hits);
  }
}

static inline std::vector<geometry_msgs::msg::Point>
dilatePolygon(const std::vector<geometry_msgs::msg::Point32> & in, double d)
{
  // 간단 근사: 각 점을 바깥쪽으로 d만큼 평행이동(정확하진 않으나 소형 팽창에 충분)
  std::vector<geometry_msgs::msg::Point> out; out.reserve(in.size());
  // 무게중심
  double cx=0, cy=0;
  for (auto & p : in) { cx += p.x; cy += p.y; }
  cx /= std::max<size_t>(1, in.size()); cy /= std::max<size_t>(1, in.size());
  for (auto & p : in) {
    double vx = p.x - cx, vy = p.y - cy;
    double n = std::hypot(vx, vy); if (n < 1e-6) n = 1.0;
    geometry_msgs::msg::Point q;
    q.x = p.x + d * (vx / n);
    q.y = p.y + d * (vy / n);
    q.z = 0.0;
    out.push_back(q);
  }
  return out;
}

void AgentLayer::fillFootprintAt(const geometry_msgs::msg::PolygonStamped & fp,
                                 const geometry_msgs::msg::Pose & pose,
                                 double extra_dilation_m,
                                 nav2_costmap_2d::Costmap2D * grid,
                                 unsigned char cost,
                                 std::vector<std::pair<unsigned int,unsigned int>> * meta_hits)
{
  // 1) 발자국 local polygon을 dilation
  auto poly = dilatePolygon(fp.polygon.points, extra_dilation_m);

  // 2) world 변환(translation + yaw)
  const double yaw = tf2::getYaw(pose.orientation);
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (auto & p : poly) {
    double x = p.x, y = p.y;
    p.x = pose.position.x + c * x - s * y;
    p.y = pose.position.y + s * x + c * y;
  }

  // 3) bbox → 셀 루프
  double minx=1e9, miny=1e9, maxx=-1e9, maxy=-1e9;
  for (auto & p : poly) { minx = std::min(minx, p.x); miny = std::min(miny, p.y);
                          maxx = std::max(maxx, p.x); maxy = std::max(maxy, p.y); }

  unsigned int min_i, min_j, max_i, max_j;
  if (!grid->worldToMapEnforceBounds(minx, miny, min_i, min_j)) return;
  if (!grid->worldToMapEnforceBounds(maxx, maxy, max_i, max_j)) return;

  // 4) 포인트-인-폴리곤(raycast)로 내부 셀 채우기
  for (unsigned int j = min_j; j <= max_j; ++j) {
    for (unsigned int i = min_i; i <= max_i; ++i) {
      double wx, wy; grid->mapToWorld(i, j, wx, wy);
      if (pointInPolygon(poly, wx, wy)) {
        grid->setCost(i, j, cost);
        if (meta_hits) meta_hits->emplace_back(i, j);
      }
    }
  }

  // updateBounds용 터치 박스 갱신(안전)
  touch_min_x_ = std::min(touch_min_x_, minx);
  touch_min_y_ = std::min(touch_min_y_, miny);
  touch_max_x_ = std::max(touch_max_x_, maxx);
  touch_max_y_ = std::max(touch_max_y_, maxy);
  touched_ = true;
}

bool AgentLayer::pointInPolygon(const std::vector<geometry_msgs::msg::Point> & poly,
                                double x, double y)
{
  // ray casting
  bool inside = false;
  size_t n = poly.size();
  for (size_t i=0, j=n-1; i<n; j=i++) {
    const double xi = poly[i].x, yi = poly[i].y;
    const double xj = poly[j].x, yj = poly[j].y;
    const bool intersect =
      ((yi > y) != (yj > y)) &&
      (x < (xj - xi) * (y - yi) / std::max(1e-12, (yj - yi)) + xi);
    if (intersect) inside = !inside;
  }
  return inside;
}

} // namespace multi_agent_nav2
```

> 주석: 위 메타 퍼블리시 부분은 **간략 스켈레톤**이라 셀별 `machine_id` 등 상세를 싣지 않았어. 실제로는 `rasterizeAgentPath()`에서 현재 에이전트의 id/phase 등을 **로컬 벡터에 모아** 함께 내보내면 된다(메모리/대역 고려해 stride 사용).

---

# 4) plugin.xml / CMake / package

### `plugin.xml`

```xml
<library path="lib/libmulti_agent_nav2">
  <class name="multi_agent_nav2::AgentLayer"
         type="multi_agent_nav2::AgentLayer"
         base_class_type="nav2_costmap_2d::Layer">
    <description>Costmap layer that paints other agents from MultiAgentInfoArray.</description>
  </class>
</library>
```

### `CMakeLists.txt` (요약)

```cmake
cmake_minimum_required(VERSION 3.16)
project(multi_agent_nav2)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(multi_agent_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/agent_layer.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp rclcpp_lifecycle pluginlib nav2_costmap_2d nav_msgs geometry_msgs multi_agent_msgs
)
pluginlib_export_plugin_description_file(nav2_costmap_2d plugin.xml)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})

ament_package()
```

### `package.xml` (요약)

```xml
<package format="3">
  <name>multi_agent_nav2</name>
  <version>0.1.0</version>
  <description>AgentLayer for Nav2 costmap_2d</description>
  <maintainer email="you@example.com">You</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>pluginlib</depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>nav2_costmap_2d</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>multi_agent_msgs</depend>

  <export>
    <nav2_costmap_2d plugin="${prefix}/share/${PROJECT_NAME}/plugin.xml"/>
  </export>
</package>
```

---

# 5) Nav2 params.yaml 예시

```yaml
nav2_controller:
  ros__parameters:
    use_sim_time: false

nav2_planner:
  ros__parameters:
    planner_plugin_ids: ["GridBased"]
    planner_plugin_types: ["nav2_smac_planner/SmacPlanner2D"]

global_costmap:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    use_sim_time: false
    publish_frequency: 5.0
    update_frequency: 5.0
    rolling_window: true
    width: 50.0
    height: 50.0
    resolution: 0.05
    plugins: ["static_layer", "obstacle_layer", "agent_layer", "inflation"]
    static_layer:
      plugin: "nav2_costmap_2d::StaticLayer"
    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
    agent_layer:
      plugin: "multi_agent_nav2::AgentLayer"
      enabled: true
      topic: "/multi_agent_infos"
      self_machine_id: 1001
      self_type_id: "CUSTOM_AMR_A"
      roi_range_m: 12.0
      time_decay_sec: 1.0
      lethal_cost: 254
      moving_cost: 180
      waiting_cost: 200
      manual_cost_bias: 30
      dilation_m: 0.05
      forward_smear_m: 0.25
      sigma_k: 2.0
      publish_meta: true
      meta_stride: 3
      freshness_timeout_ms: 800
      max_poses: 40
      qos_reliable: true
    inflation:
      plugin: "nav2_costmap_2d::InflationLayer"
      inflation_radius: 0.8
      cost_scaling_factor: 2.5

local_costmap:
  ros__parameters:
    # 동일한 agent_layer 설정 복제 가능(필요 시 파라미터 틀만 조정)
    plugins: ["voxel_layer", "agent_layer", "inflation"]
```

---

# 6) 성능/정확도 팁 & 테스트

* **성능**

  * ROI(`roi_range_m`), `max_poses`, `meta_stride`로 셀 루프량을 제어
  * `updateBounds`에서 bbox를 보수적으로만 확장 → `updateCosts`에서 실제 칠하기
  * 컨테이너에서는 `RMW_FASTRTPS`보다 `CycloneDDS`가 멀티 토픽에 안정적인 경우 많음
* **정확도**

  * footprint는 **CCW 다각형** 권장, 원형은 20~24각 근사
  * `pos_std_m`, `yaw_std_rad` 있으면 `sigma_k` 가중으로 r_safe 확대
  * `manual` 모드 + 협소구간은 비용 상향(Decider에서 WAIT 우선하도록)
* **테스트**

  * RViz: `Costmap` + `agent_layer_meta` 시각화(메타는 MarkerArray로도 추가 가능)
  * 케이스: Head-on / Crossing / Following / Blocking(협소) 4종, 자기 자신 필터 확인
  * `freshness_timeout_ms`에 따른 오래된 truncated_path 무시 확인

---

원하면 위 스켈레톤을 **완전 컴파일 가능한 샘플 패키지 형태**로 더 다듬어서(네임스페이스, CMake install 경로 등 포함) 붙여줄 수 있어. 또, `meta_pub_`에 **셀별 machine_id/phase**를 제대로 실어 보내는 구현(벡터 수집 방식)도 확장해줄게.
