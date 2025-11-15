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
