알겠습니다. `local_costmap`("odom" 프레임)에서 `agent_layer`를 사용하기 위해, `tf_` 버퍼를 사용하여 "map" 프레임의 에이전트 데이터를 "odom" 프레임으로 변환하는 코드를 추가하겠습니다.

`nav2_costmap_2d::Layer` 기본 클래스는 `tf_`라는 이름의 `tf2_ros::Buffer`를 이미 상속해 주므로, 우리는 새 버퍼를 만들 필요 없이 `tf_`를 사용하기만 하면 됩니다.

또한, 이전에 발견했던 **파라미터 로딩 버그(Loop 2)도 함께 수정**했습니다.

-----

### 1\. `agent_layer.hpp` 변경 사항

`private:` 섹션에 TF 관련 헤더를 인클루드하고, TF 변환을 수행할 헬퍼 함수를 하나 선언합니다.

```cpp
/* ****************************************
 * [BEFORE] agent_layer.hpp - includes
 * ****************************************
 */
#include <multi_agent_msgs/msg/multi_agent_info_array.hpp>
#include <multi_agent_msgs/msg/agent_layer_meta_array.hpp>
#include <multi_agent_msgs/msg/agent_status.hpp>

namespace multi_agent_nav2
{
```

```cpp
/* ****************************************
 * [AFTER] agent_layer.hpp - includes
 * ****************************************
 */
#include <multi_agent_msgs/msg/multi_agent_info_array.hpp>
#include <multi_agent_msgs/msg/agent_layer_meta_array.hpp>
#include <multi_agent_msgs/msg/agent_status.hpp>

// [NEW] Add TF2 headers for transformation
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace multi_agent_nav2
{
```

-----

```cpp
/* ****************************************
 * [BEFORE] agent_layer.hpp - private helpers
 * ****************************************
 */
  // [NEW] Helper to convert nav2_costmap_2d::makeFootprint... results
  static std::vector<geometry_msgs::msg::Point32> toPoint32(
      const std::vector<geometry_msgs::msg::Point>& points);

  // helpers
  void infosCallback(const multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr msg);
```

```cpp
/* ****************************************
 * [AFTER] agent_layer.hpp - private helpers
 * ****************************************
 */
  // [NEW] Helper to convert nav2_costmap_2d::makeFootprint... results
  static std::vector<geometry_msgs::msg::Point32> toPoint32(
      const std::vector<geometry_msgs::msg::Point>& points);

  // [NEW] Helper for transforming agent data to the costmap's frame
  bool transformAgentInfo(
      const multi_agent_msgs::msg::MultiAgentInfo & agent_in_map,
      multi_agent_msgs::msg::MultiAgentInfo & agent_in_costmap_frame,
      const std::string & costmap_frame) const;

  // helpers
  void infosCallback(const multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr msg);
```

-----

### 2\. `agent_layer.cpp` 변경 사항

#### Includes 추가

`tf2_ros/buffer.h` 헤더를 `.cpp` 파일에도 추가합니다.

```cpp
/* ****************************************
 * [BEFORE] agent_layer.cpp - includes
 * ****************************************
 */
#include <geometry_msgs/msg/point32.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <multi_agent_msgs/msg/agent_status.hpp>
#include <multi_agent_msgs/msg/agent_layer_cell_meta.hpp>
```

```cpp
/* ****************************************
 * [AFTER] agent_layer.cpp - includes
 * ****************************************
 */
#include <geometry_msgs/msg/point32.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <multi_agent_msgs/msg/agent_status.hpp>
#include <multi_agent_msgs/msg/agent_layer_cell_meta.hpp>

// [NEW] Include TF2 buffer (although tf_ is inherited, explicit include is safer)
#include "tf2_ros/buffer.h"
```

#### TF 변환 헬퍼 함수 구현

`AgentLayer::AgentLayer()` 생성자 이전에 헬퍼 함수 구현을 추가합니다.

```cpp
/* ****************************************
 * [BEFORE] agent_layer.cpp - before constructor
 * ****************************************
 */
    fp_stamped.header.stamp = a.current_pose.header.stamp;
    return fp_stamped;
}



AgentLayer::AgentLayer() {}
```

```cpp
/* ****************************************
 * [AFTER] agent_layer.cpp - before constructor
 * ****************************************
 */
    fp_stamped.header.stamp = a.current_pose.header.stamp;
    return fp_stamped;
}


// [NEW] Implementation for the TF helper function
bool AgentLayer::transformAgentInfo(
    const multi_agent_msgs::msg::MultiAgentInfo & agent_in_map,
    multi_agent_msgs::msg::MultiAgentInfo & agent_in_costmap_frame,
    const std::string & costmap_frame) const
{
  // We assume agent_in_map data is in the frame_id of last_infos_->header
  const std::string& map_frame = last_infos_->header.frame_id;
  const rclcpp::Time& map_stamp = last_infos_->header.stamp; // Use the stamp from the array

  if (map_frame.empty()) {
    RCLCPP_WARN_ONCE(logger_, "MultiAgentInfoArray message has empty frame_id. Cannot transform.");
    return false;
  }

  // If frames are already the same, just copy
  if (map_frame == costmap_frame) {
    agent_in_costmap_frame = agent_in_map;
    return true;
  }

  // Copy non-transformable data (status, id, etc.)
  agent_in_costmap_frame = agent_in_map; 
  agent_in_costmap_frame.truncated_path.poses.clear(); // Clear poses to re-fill

  try {
    // 1. Transform current_pose
    geometry_msgs::msg::PoseStamped pose_to_transform = agent_in_map.current_pose;
    pose_to_transform.header.frame_id = map_frame; // Ensure correct source frame
    pose_to_transform.header.stamp = map_stamp;     // Use array stamp for TF lookup

    geometry_msgs::msg::PoseStamped transformed_pose;
    // tf_ is the inherited buffer from nav2_costmap_2d::Layer
    tf_->transform(pose_to_transform, transformed_pose, costmap_frame);
    agent_in_costmap_frame.current_pose = transformed_pose;

    // 2. Transform truncated_path
    for (const auto& pose_stamped_in_map : agent_in_map.truncated_path.poses) {
      pose_to_transform.pose = pose_stamped_in_map.pose; // Reuse pose_to_transform object

      geometry_msgs::msg::PoseStamped pose_in_costmap_frame;
      tf_->transform(pose_to_transform, pose_in_costmap_frame, costmap_frame);
      
      agent_in_costmap_frame.truncated_path.poses.push_back(pose_in_costmap_frame);
    }
    
    // Update header of the path
    agent_in_costmap_frame.truncated_path.header.frame_id = costmap_frame;
    agent_in_costmap_frame.truncated_path.header.stamp = transformed_pose.header.stamp;
    return true;

  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node_shared_->get_clock(), 2000,
      "Failed to transform agent %u from '%s' to '%s': %s",
      agent_in_map.machine_id, map_frame.c_str(), costmap_frame.c_str(), ex.what());
    return false;
  }
  return false;
}


AgentLayer::AgentLayer() {}
```

#### `onInitialize()` 수정

`[WARN]` 메시지의 원인이었던 \*\*파라미터 로딩 버그(Loop 2)\*\*를 수정합니다.

```cpp
/* ****************************************
 * [BEFORE] agent_layer.cpp - onInitialize() Loop 2
 * ****************************************
 */
  // [NEW] Loop 2: Get parameters and populate the map
  agent_footprints_.clear();
  for (const auto & id_str : robot_ids) {
    std::string id_ns = name_ + "." + id_str;
    
    int machine_id_int = 0;
    node_shared_->get_parameter(id_ns + ".machine_id", machine_id_int);
    if (machine_id_int == 0) continue; // Skip invalid ID

    uint16_t machine_id = static_cast<uint16_t>(machine_id_int);
    
    AgentFootprintData data;
    std::string footprint_str;
    node_shared_->get_parameter(id_ns + ".footprint", footprint_str);
    node_shared_->get_parameter(id_ns + ".robot_radius", data.radius);
```

```cpp
/* ****************************************
 * [AFTER] agent_layer.cpp - onInitialize() Loop 2
 * ****************************************
 */
  // [NEW] Loop 2: Get parameters and populate the map
  agent_footprints_.clear();
  for (const auto & id_str : robot_ids) {
    // [FIX] 'id_ns' 제거. 'get_parameter' 헬퍼를 사용합니다.
    // std::string id_ns = name_ + "." + id_str; 
    
    int machine_id_int = 0;
    // [FIX] Layer의 'get_parameter' 헬퍼는 'name_'을 자동으로 붙여줍니다.
    get_parameter(id_str + ".machine_id", machine_id_int);
    if (machine_id_int == 0) continue; // Skip invalid ID

    uint16_t machine_id = static_cast<uint16_t>(machine_id_int);
    
    AgentFootprintData data;
    std::string footprint_str;
    get_parameter(id_str + ".footprint", footprint_str); // [FIX]
    get_parameter(id_str + ".robot_radius", data.radius); // [FIX]
```

#### `updateBounds()` 수정

TF 변환 로직을 `for` 루프에 적용합니다.

```cpp
/* ****************************************
 * [BEFORE] agent_layer.cpp - updateBounds() loop
 * ****************************************
 */
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
```

```cpp
/* ****************************************
 * [AFTER] agent_layer.cpp - updateBounds() loop
 * ****************************************
 */
  // [CHANGED] costmap_frame은 "map" 또는 "odom"이 될 수 있습니다.
  const std::string & costmap_frame = layered_costmap_->getGlobalFrameID();

  for (const auto & a_map : infos) { // "map" 프레임 기준 원본 데이터
    if (isSelf(a_map)) continue;

    // [NEW] Transform agent info from "map" to costmap frame (e.g., "odom")
    multi_agent_msgs::msg::MultiAgentInfo a; // 변환된 데이터가 저장될 변수
    if (!transformAgentInfo(a_map, a, costmap_frame)) {
      continue; // TF 변환 실패 시 이 에이전트 무시
    }

    // [CHANGED] ROI 검사를 변환된 'a'의 좌표로 수행
    const double dx = a.current_pose.pose.position.x - robot_x;
    const double dy = a.current_pose.pose.position.y - robot_y;
    if (std::hypot(dx, dy) > roi_range_m_) continue;

    // [REMOVED] TF 변환을 거쳤으므로 이 프레임 체크는 더 이상 유효하지 않거나 불필요합니다.
    // if (use_path_header_frame_ && a.truncated_path.header.frame_id != global_frame) {
    //   continue;
    // }

    // [CHANGED] 변환된 'a'의 좌표를 사용
    {
      const auto & p = a.current_pose.pose.position;
```

#### `updateCosts()` 수정

`updateBounds`와 동일한 TF 변환 로직을 `for` 루프에 적용합니다.

```cpp
/* ****************************************
 * [BEFORE] agent_layer.cpp - updateCosts() loop
 * ****************************************
 */
  // [NEW] Get the cached robot position from updateBounds
  const double robot_x = cached_robot_x_;
  const double robot_y = cached_robot_y_;


  for (const auto & a : infos) {
    if (isSelf(a)) continue;

// [NEW] CRITICAL FIX: Apply the same ROI check that was in updateBounds
    const double dx = a.current_pose.pose.position.x - robot_x;
    const double dy = a.current_pose.pose.position.y - robot_y;
    if (std::hypot(dx, dy) > roi_range_m_) continue;

    rasterizeAgentPath(a, &master_grid, meta_hits);
  }
```

```cpp
/* ****************************************
 * [AFTER] agent_layer.cpp - updateCosts() loop
 * ****************************************
 */
  // [NEW] Get the cached robot position from updateBounds
  const double robot_x = cached_robot_x_;
  const double robot_y = cached_robot_y_;

  // [NEW] Get the costmap frame ID ("map" 또는 "odom")
  const std::string & costmap_frame = layered_costmap_->getGlobalFrameID();

  for (const auto & a_map : infos) { // "map" 프레임 기준 원본 데이터
    if (isSelf(a_map)) continue;

    // [NEW] Transform agent info from "map" to costmap frame (e.g., "odom")
    multi_agent_msgs::msg::MultiAgentInfo a; // 변환된 데이터가 저장될 변수
    if (!transformAgentInfo(a_map, a, costmap_frame)) {
      continue; // TF 변환 실패 시 이 에이전트 무시
    }

    // [CHANGED] ROI check using the *transformed* pose 'a'
    const double dx = a.current_pose.pose.position.x - robot_x;
    const double dy = a.current_pose.pose.position.y - robot_y;
    if (std::hypot(dx, dy) > roi_range_m_) continue;

    rasterizeAgentPath(a, &master_grid, meta_hits); // 변환된 'a'를 전달
  }
```
################################################ 
#####################  
#######################  
네, 사용자님의 생각이 정확합니다. `global_costmap`("map" 프레임)에서는 정상인데 `local_costmap`("odom" 프레임)에서만 문제가 발생한다면, 이는 99.9% TF 변환 문제입니다.

사용자님의 가설대로, `/multi_agent_infos` 토픽의 "map" 프레임 데이터를 "odom" 프레임으로 변환하지 않고 `local_costmap`에 그리고 있기 때문입니다.

제가 이전에 `local_costmap`을 지원하도록 제안했던 `transformAgentInfo` 함수에 **버그가 있었습니다.** 죄송합니다.

`truncated_path`의 포즈들을 변환할 때, TF 변환에 필요한 `header` 정보를 올바르게 설정하지 않고 재사용하여 변환이 실패했거나 잘못된 값으로 변환되었습니다.

-----

### 🐞 `agent_layer.cpp`의 버그 수정

`agent_layer.cpp` 파일에서 `transformAgentInfo` 함수 내부의 `for` 루프만 수정하면 됩니다.

#### `agent_layer.cpp (transformAgentInfo)` 수정

```cpp
/* ****************************************
 * [BEFORE] agent_layer.cpp - transformAgentInfo()
 * ****************************************
 */
    // 2. Transform truncated_path
    for (const auto& pose_stamped_in_map : agent_in_map.truncated_path.poses) {
      pose_to_transform.pose = pose_stamped_in_map.pose; // Reuse pose_to_transform object

      geometry_msgs::msg::PoseStamped pose_in_costmap_frame;
      tf_->transform(pose_to_transform, pose_in_costmap_frame, costmap_frame);
      
      agent_in_costmap_frame.truncated_path.poses.push_back(pose_in_costmap_frame);
    }
```

```cpp
/* ****************************************
 * [AFTER] agent_layer.cpp - transformAgentInfo()
 * ****************************************
 */
    // 2. Transform truncated_path
    for (const auto& pose_stamped_in_map : agent_in_map.truncated_path.poses) {
      
      // [FIX] Create a NEW PoseStamped for each path point
      geometry_msgs::msg::PoseStamped path_pose_to_transform;
      // [FIX] Explicitly set the header for *this* point
      path_pose_to_transform.header.frame_id = map_frame;
      path_pose_to_transform.header.stamp = map_stamp;
      // [FIX] Copy the pose from the path
      path_pose_to_transform.pose = pose_stamped_in_map.pose; 

      geometry_msgs::msg::PoseStamped pose_in_costmap_frame;
      // [FIX] Transform the new, correct object
      tf_->transform(path_pose_to_transform, pose_in_costmap_frame, costmap_frame);
      
      agent_in_costmap_frame.truncated_path.poses.push_back(pose_in_costmap_frame);
    }
```

### 🔧 수정된 이유

  * **Before (버그):** `pose_to_transform` 변수를 재사용했습니다. 이 변수는 `current_pose`의 `PoseStamped` 정보가 담겨있었는데, 루프 내에서 `.pose` 필드만 덮어썼습니다. 이 방식은 `header` 정보가 일관되지 않거나, `tf_->transform` 함수가 예기치 않게 동작할 수 있는 위험이 있었습니다.
  * **After (수정):** `for` 루프 안에서 **매번 새로운 `path_pose_to_transform` 객체를 생성**합니다. 그리고 `map_frame`과 `map_stamp`를 사용하여 `header`를 명시적으로 올바르게 설정한 뒤, `pose` 정보만 복사하여 변환합니다.

이 코드는 `local_costmap`("odom")과 `global_costmap`("map") 모두에서 TF 변환을 올바르게 처리하여 프레임 불일치 문제를 해결합니다.
##############################  
################################  
################################  
네, `agent_layer`에서 다른 로봇의 상태(`AgentStatus`)별로 코스트 팽창(dilation) 사이즈를 다르게 적용하는 방법을 알려드리겠습니다.

### 🛠️ 변경해야 할 코드 부분

`agent_layer.cpp`의 **`computeDilation` 함수**를 수정해야 합니다.

현재 이 함수는 `dilation_m_` 파라미터 하나만을 반환하고 있지만, 여기서 `a.status.phase`를 확인하여 상태별로 다른 값을 반환하도록 로직을 확장하면 됩니다.

-----

### 📄 코드 템플릿 (computeDilation 수정)

다음은 `computeDilation` 함수를 수정하는 템플릿입니다.

```cpp
// [CHANGED] 상태별 Dilation 적용
double AgentLayer::computeDilation(const multi_agent_msgs::msg::MultiAgentInfo & a) const
{
  using S = multi_agent_msgs::msg::AgentStatus;
  const uint8_t phase = a.status.phase;

  // 1. 기본값 설정 (파라미터 dilation_m_)
  double r = dilation_m_;

  // 2. 상태별 조건부 Dilation 적용
  switch (phase) {
    // Case A: 팽창을 크게 하고 싶은 상태들 (예: 에러, 수동 주행, 충전 중 등)
    // 다른 로봇이 멀리서부터 피해가도록 함
    case S::STATUS_ERROR:
    case S::STATUS_MANUAL_RUNNING:
    case S::STATUS_CHARGING:
    case S::STATUS_AUTORECOVERY:
      r = std::max(r, 0.3); // 예: 최소 0.3m 보장 (수치는 직접 수정하세요)
      break;

    // Case B: 팽창을 작게 하고 싶은 상태들 (예: 좁은 곳 작업 중, 정밀 도킹 등)
    // 다른 로봇이 가까이 지나갈 수 있도록 함
    case S::STATUS_LOADING:
    case S::STATUS_UNLOADING:
    case S::STATUS_MARKING: 
      r = std::max(r, 0.02); // 예: 아주 작게 (0.02m)
      break;

    // Case C: 일반 주행/대기 상태 (기본값 사용)
    case S::STATUS_MOVING:
    case S::STATUS_WAITING_FOR_OBS:
    default:
      // r = dilation_m_; // 이미 설정됨
      break;
  }

  // 3. (옵션) 위치 불확실성 반영
  // if (a.pos_std_m >= 0.0) r += sigma_k_ * a.pos_std_m;

  return r;
}
```

### 📝 적용 방법

1.  `agent_layer.cpp` 파일을 엽니다.
2.  기존의 `computeDilation` 함수를 찾습니다.
3.  위의 템플릿 코드로 내용을 교체합니다.
4.  `case` 문 내부의 상태 목록과 적용할 `r` 값(반지름)을 사용자님의 의도에 맞게 수정합니다.

이 변경을 통해 `rasterizeAgentPath` 함수가 호출될 때마다 에이전트의 현재 상태에 맞는 팽창 크기가 `fillFootprintAt` 함수로 전달되어 코스트맵에 반영됩니다.  
###################################  
####################################  
######################################  
`AgentLayer`는 **Nav2 Costmap 2D**의 플러그인으로서, 외부에서 수신된 다른 로봇(Agent)들의 정보를 바탕으로 코스트맵에 장애물을 표시하는 역할을 합니다.

요청하신 대로 주요 함수와 파라미터에 대해 상세히 설명해 드리겠습니다.

---

### 1. 주요 함수 (Key Functions)

이 클래스의 핵심 로직을 담당하는 함수들입니다.

#### **`onInitialize()`**
* **역할:** 플러그인이 로드될 때 **최초 1회 실행**되는 초기화 함수입니다.
* **주요 동작:**
    1.  ROS 2 노드 핸들을 가져옵니다.
    2.  **파라미터 선언 및 로딩:** YAML 파일에 정의된 모든 설정을 읽어옵니다.
    3.  **Footprint 로딩:** `robot_ids` 리스트를 순회하며 각 로봇 ID에 해당하는 `footprint` 또는 `radius` 정보를 읽어 `agent_footprints_` 맵에 저장합니다.
    4.  **ROS 통신 설정:** `/multi_agent_infos` 토픽 구독(Subscriber)과 메타 데이터 발행(Publisher)을 설정합니다.

#### **`updateBounds(robot_x, robot_y, ...)`**
* **역할:** 코스트맵 갱신 주기마다 호출되며, **"이번 주기에 갱신해야 할 영역(Bounding Box)"**을 계산합니다.
* **주요 동작:**
    1.  **로봇 위치 캐싱:** `updateCosts`에서 사용하기 위해 현재 로봇의 위치(`cached_robot_x/y`)를 저장합니다.
    2.  **TF 변환:** 수신된 에이전트 정보를 현재 코스트맵의 프레임(Global은 "map", Local은 "odom")으로 변환합니다.
    3.  **ROI 필터링:** 로봇과 에이전트 간의 거리가 `roi_range_m_` 이내인지 확인합니다.
    4.  **영역 확장:** 유효한 에이전트의 현재 위치와 예측 경로(`truncated_path`)를 감싸는 사각형 영역만큼 `min_x`, `min_y`, `max_x`, `max_y`를 확장합니다.

#### **`updateCosts(master_grid, ...)`**
* **역할:** `updateBounds`에서 계산된 영역 내의 셀들에 **실제 비용(Cost)을 칠하는(Writing)** 함수입니다.
* **주요 동작:**
    1.  `updateBounds`와 동일하게 **TF 변환** 및 **ROI 필터링**을 수행합니다. (이중 체크를 통해 성능 최적화 및 버그 방지)
    2.  각 에이전트에 대해 **`rasterizeAgentPath`**를 호출하여 장애물을 그립니다.
    3.  디버깅용 메타 데이터(`/agent_layer_meta`)를 발행합니다.

#### **`rasterizeAgentPath(agent, grid, ...)`**
* **역할:** 단일 에이전트를 코스트맵에 그리기 위한 준비를 합니다.
* **주요 동작:**
    1.  `getFootprintForAgent`를 통해 해당 에이전트의 모양(Footprint)을 가져옵니다.
    2.  `computeCost`와 `computeDilation`을 호출하여 적용할 비용과 팽창 크기를 결정합니다.
    3.  이동 중(`isMovingPhase`)이라면 전방 스미어(`forward_smear_m_`) 값을 설정합니다.
    4.  **`fillFootprintAt`**을 호출하여 실제 맵에 값을 씁니다.

#### **`fillFootprintAt(footprint, pose, ...)`**
* **역할:** 주어진 위치와 모양대로 맵에 색칠을 하는 **Low-level 그리기 함수**입니다.
* **주요 동작:**
    1.  **Dilation & Smear:** 기본 풋프린트 다각형을 등방성 팽창시키거나 전방으로 늘립니다.
    2.  **Transform:** 로봇 중심 좌표계의 풋프린트를 월드 좌표계로 변환합니다.
    3.  **Rasterization:** 다각형 내부의 모든 셀을 찾아내어 `master_grid`에 지정된 `cost`를 입력합니다. (기존 값보다 클 경우에만 덮어쓰는 Max-Merge 방식 사용)

#### **`transformAgentInfo(...)`**
* **역할:** 데이터의 좌표계("map")와 코스트맵의 좌표계("odom" 등)가 다를 때, TF를 사용하여 위치와 경로를 변환해 주는 헬퍼 함수입니다.

---

### 2. 파라미터 (Parameters)

YAML 파일(`nav2_params.yaml`)에서 설정할 수 있는 변수들입니다.

#### **기본 설정**
* `enabled` (bool): 레이어 활성화 여부 (Default: `true`).
* `topic` (string): 에이전트 정보를 수신할 토픽 이름 (Default: `"/multi_agent_infos"`).
* `qos_reliable` (bool): 토픽 구독 시 Reliable QoS 사용 여부. False면 Best Effort 사용 (Default: `true`).

#### **필터링 및 성능**
* `self_machine_id` (int): 내 로봇의 ID. (자신은 장애물로 그리지 않기 위해 사용).
* `roi_range_m` (double): 내 로봇을 중심으로 이 반경(미터) 내에 있는 에이전트만 처리합니다. (Default: `12.0`).
* `time_decay_sec` (double): 데이터가 이 시간보다 오래되면 무시합니다. (현재 코드에서는 `freshness_timeout_ms`가 더 직접적으로 쓰임).
* `freshness_timeout_ms` (int): 수신된 메시지가 이 시간(ms)보다 오래되면 맵에 그리지 않습니다 (Default: `800`).
* `max_poses` (int): 예측 경로(`truncated_path`) 중 몇 번째 포즈까지만 그릴지 제한합니다 (Default: `40`).

#### **비용(Cost) 관련**
* `lethal_cost` (int): 충돌 시 부여할 치명적 비용 (Default: `254`).
* `moving_cost` (int): 이동 중인 에이전트에게 부여할 비용 (Default: `254`).
* `waiting_cost` (int): 정지 중인 에이전트에게 부여할 비용 (Default: `200`).
* `manual_cost_bias` (int): 수동 조작 모드인 에이전트에게 추가할 비용 가중치 (Default: `30`).

#### **형상 및 팽창 (Geometry & Dilation)**
* `dilation_m` (double): 에이전트 주변에 적용할 기본 안전 여유 거리 (등방성 팽창) (Default: `0.05`).
* `forward_smear_m` (double): 이동 중일 때 진행 방향(전방)으로 풋프린트를 늘릴 길이 (Default: `0.005`).
* `sigma_k` (double): 위치 불확실성(`pos_std_m`)을 팽창에 반영할 때의 계수 (현재 코드는 주석 처리됨).

#### **개별 로봇 Footprint 설정 (`robot_ids` 리스트)**
* `robot_ids` (string list): 관리할 로봇들의 ID 목록 (예: `["robot_001", "robot_002"]`).
* 각 ID 아래:
    * `machine_id` (int): 해당 로봇의 정수형 ID 식별자.
    * `footprint` (string): `[[x1,y1], [x2,y2], ...]` 형태의 좌표 문자열.
    * `robot_radius` (double): 풋프린트가 비어있을 경우 사용할 원형 반지름.

#### **디버깅**
* `publish_meta` (bool): 디버깅용 메타 데이터 토픽 발행 여부.
* `meta_stride` (int): 메타 데이터를 발행할 때 몇 개의 셀마다 하나씩 발행할지 (데이터 양 조절).
