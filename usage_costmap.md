네, `costmap_2d_publisher.cpp` 소스 코드를 제공해주셔서 감사합니다. 이 코드는 두 토픽의 차이점을 명확하게 설명하는 결정적인 증거를 담고 있습니다. 코드를 기반으로 두 토픽의 정체, 차이점, 그리고 어떤 것을 사용해야 하는지 명확히 설명해 드리겠습니다.

다른 코드는 필요 없습니다. 이 파일 하나로 모든 것이 설명됩니다.

### 코드 분석을 통한 `/costmap`과 `/costmap_raw`의 진실

두 토픽은 모두 동일한 원본 데이터, 즉 내부적으로 관리되는 `Costmap2D` 객체(`costmap_` 멤버 변수)로부터 생성됩니다. 하지만 발행(publish) 직전에 **서로 다른 가공 과정**을 거칩니다.

#### 1\. `/costmap` 토픽 (`nav_msgs::msg::OccupancyGrid` 타입)

이 토픽은 `prepareGrid()` 함수에서 생성되며, 가장 중요한 부분은 `cost_translation_table_`을 사용한 데이터 변환 과정입니다.

**`Costmap2DPublisher` 생성자**를 보면 `cost_translation_table_`이 초기화됩니다.

```cpp
// ... in constructor
if (cost_translation_table_ == NULL) {
    cost_translation_table_ = new char[256];

    // special values:
    cost_translation_table_[0] = 0;      // NO obstacle
    cost_translation_table_[253] = 99;   // INSCRIBED obstacle
    cost_translation_table_[254] = 100;  // LETHAL obstacle
    cost_translation_table_[255] = -1;   // UNKNOWN

    // regular cost values scale the range 1 to 252 ... to fit into 1 to 98
    for (int i = 1; i < 253; i++) {
        cost_translation_table_[i] = static_cast<char>(1 + (97 * (i - 1)) / 251);
    }
}
```

  * **핵심**: 이 코드는 Nav2 내부에서 사용하는 **0\~255 범위의 Cost 값**을 `OccupancyGrid` 표준 메시지에서 사용하는 **-1, 0\~100 범위로 변환(mapping)하는 규칙 테이블**입니다.
  * **데이터 손실**: 특히 `1`부터 `252`까지의 세밀한 Inflation cost 값들을 `1`부터 `98`까지의 범위로 압축합니다. 이 과정에서 **데이터의 정밀도 손실이 발생**합니다.

그리고 **`prepareGrid()` 함수**에서 이 변환 테이블을 사용합니다.

```cpp
// ... in prepareGrid()
unsigned char * data = costmap_->getCharMap();
for (unsigned int i = 0; i < grid_->data.size(); i++) {
    grid_->data[i] = cost_translation_table_[data[i]];
}
```

  * **증거**: 원본 `costmap` 데이터(`data[i]`)를 `cost_translation_table_`을 이용해 변환한 뒤 메시지에 담고 있습니다.

**결론**: `/costmap` 토픽은 RViz와 같은 표준 `OccupancyGrid` 시각화 도구와의 호환성을 위해 **원본 데이터를 가공하고 압축한, 정보 손실이 발생한 버전**입니다.

-----

#### 2\. `/costmap_raw` 토픽 (`nav2_msgs::msg::Costmap` 타입)

이 토픽은 `prepareCostmap()` 함수에서 생성됩니다. 데이터 처리 과정을 주목해 주십시오.

```cpp
// ... in prepareCostmap()
unsigned char * data = costmap_->getCharMap();
memcpy(costmap_raw_->data.data(), data, costmap_raw_->data.size());
```

  * **핵심**: `memcpy` 함수를 사용합니다. 이는 메모리를 그대로 복사하는 함수로, **데이터 변환이나 가공을 전혀 하지 않습니다.**
  * **데이터 보존**: Nav2 내부의 `Costmap2D`가 가진 0\~255 범위의 원본 cost 값을 **아무런 손실 없이 그대로** 메시지에 담습니다.

**결론**: `/costmap_raw` 토픽은 Nav2가 내부적으로 사용하는 **순수한(raw) costmap 데이터를 손실 없이 그대로 발행**한 버전입니다. 다른 알고리즘이나 노드가 Planner와 동일한 정보를 기반으로 계산을 수행할 수 있도록 하기 위함입니다.

### 최종 비교 및 사용 가이드

| 항목 | `/costmap` | `/costmap_raw` |
| :--- | :--- | :--- |
| **토픽 이름** | `(global/local)_costmap/costmap` | `(global/local)_costmap/costmap_raw` |
| **메시지 타입**| `nav_msgs::msg::OccupancyGrid` | `nav2_msgs::msg::Costmap` |
| **데이터 범위**| -1 (Unknown), 0 (Free), 1-100 (Occupied) | 0-255 (Nav2 내부 Cost 값) |
| **데이터 변환**| **변환 O (Lossy / 손실 압축)** | **변환 X (Lossless / 무손실)** |
| **주요 목적**| **시각화 (RViz)**, 표준 `OccupancyGrid` 호환 | **알고리즘 연산**, 정밀한 의사결정 |
| **데이터 정밀도**| 낮음 (Inflation 단계가 압축됨) | **높음 (원본 데이터)** |

### 결론: 어떤 것을 사용해야 하는가?

**상황에 따라 명확하게 구분해서 사용해야 합니다.**

  * **`/costmap`을 사용해야 할 때:**

      * 단순히 RViz에서 코스트맵을 **시각적으로 확인**하고 싶을 때.
      * `OccupancyGrid` 메시지 타입만 지원하는 외부 툴과 연동할 때.
      * Inflation의 세밀한 값은 필요 없고, "여기가 장애물인가 아닌가" 정도의 거친 정보만 필요할 때.

  * **`/costmap_raw`를 사용해야 할 때:**

      * **Nav2의 의사결정에 직접 관여하는 로직을 작성할 때 (가장 중요).**
      * 질문자님의 `ReplanMonitorNode`, `IsGoalsOccupiedCondition` 노드처럼 경로/목표 지점의 cost를 직접 읽어와 임계값과 비교하는 경우.
      * **Planner가 보는 것과 100% 동일한 데이터**를 기반으로 판단해야 할 때.
      * Inflation cost 값의 미세한 차이를 구분해야 하는 정밀한 로직을 구현할 때.

따라서, 질문자님께서 작성하신 두 노드(`ReplanMonitorNode`, `IsGoalsOccupiedCondition`)의 목적을 고려할 때, Planner와 동일한 기준으로 "위험도"를 판단해야 하므로 **데이터 손실이 없는 `/costmap_raw` 토픽을 구독하는 것이 항상 더 안전하고 올바른 선택**입니다.



네, `costmap_2d_publisher.cpp` 코드 분석을 통해 얻은 새로운 정보를 바탕으로 두 노드의 코드를 다시 검토하고 최종 결론을 내려 드리겠습니다.

### 핵심 원칙 재확인

결정적인 사실은 이것입니다:

  * **`/costmap` (OccupancyGrid)**: RViz 시각화를 위해 **데이터가 가공/압축된(손실된) 버전**.
  * **`/costmap_raw` (Costmap)**: Planner와 같은 알고리즘이 사용하기 위한 **원본 데이터(무손실) 버전**.

따라서, **Cost 값을 읽어와 어떤 로직에 따라 의사결정을 내리는 모든 노드는 반드시 `_raw` 토픽을 사용해야 합니다.**

### 1\. `ReplanMonitorNode` 재검토

  * **현재 코드의 문제점**:

    ```cpp
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/global_costmap/costmap", rclcpp::QoS(10), std::bind(&ReplanMonitorNode::costmapCallback, this, std::placeholders::_1), sub_options);
    ```

    이 노드는 현재 `/global_costmap/costmap` 토픽을 구독하고 있습니다. 즉, **시각화용으로 가공된, 정보 손실이 발생한 데이터를 사용**하고 있습니다.

  * **로직과의 불일치**:

    ```cpp
    if (cost >= cost_threshold_) { // cost_threshold_ = 20
    ```

    코드 내에서 cost 값이 20 이상인지를 확인하여 경로가 막혔다고 판단합니다. 하지만 `/costmap` 토픽의 데이터는 1\~252 범위의 Inflation Cost가 1\~98 범위로 압축되어 있습니다. 이로 인해 다음과 같은 심각한 문제가 발생합니다.

    1.  **정확성 저하**: Planner가 위험하다고 판단하는 세밀한 Cost 값의 차이를 이 노드는 알 수 없습니다.
    2.  **오작동 가능성**: `cost_threshold_` 값 20이 원래 Costmap에서 어떤 값에 해당하는지 정확히 알 수 없으므로, 너무 민감하게 또는 너무 둔감하게 Re-planning을 유발할 수 있습니다.

  * **올바른 수정 방향**:
    이 노드의 목적은 Planner와 동일한 기준으로 경로의 위험성을 판단하는 것입니다. 따라서 **반드시 `/global_costmap/costmap_raw` 토픽을 구독하도록 코드를 수정해야 합니다.**

    **수정 예시:**

    ```cpp
    // 1. 메시지 타입을 nav2_msgs::msg::Costmap 으로 변경해야 합니다.
    #include "nav2_msgs/msg/costmap.hpp" // 헤더 파일에 추가

    // 2. 콜백 함수와 구독자 멤버 변수 타입 변경
    // private:
    //  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg); ->
    void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
    //  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_; ->
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
    //  nav_msgs::msg::OccupancyGrid current_costmap_; ->
    //  nav2_costmap_2d::Costmap2D current_costmap_; // Costmap2D 객체로 직접 관리하는 것이 더 효율적입니다.

    // 3. 구독 부분 수정
    costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>( // 타입 변경
      "/global_costmap/costmap_raw", rclcpp::QoS(10), ...); // 토픽 이름 변경
    ```

### 2\. `IsGoalsOccupiedCondition` 재검토

  * **현재 코드의 장점과 단점**:

    ```cpp
    // 기본 토픽 이름
    costmap_topic = "local_costmap/costmap_raw";

    // 구독
    costmap_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
      costmap_topic, qos, ...);
    ```

    이 노드는 기본적으로 `_raw` 토픽을 사용하고 `nav2_msgs::msg::Costmap` 메시지를 구독하므로, **데이터를 손실 없이 올바르게 받고 있습니다.** 이는 매우 잘 된 부분입니다. `LETHAL_OBSTACLE`(값 253)과 같이 특정 Cost 값을 정확하게 비교해야 하는 로직에 완벽하게 부합합니다.

    **하지만, 기본 토픽 이름이 `local_costmap/costmap_raw`인 것은 대부분의 경우에 부적절합니다.**

      * \*\*목표 지점(Goal)\*\*은 보통 로봇에서 멀리 떨어져 있어 `local_costmap`의 범위를 벗어납니다.
      * 목표 지점이 물리적으로 불가능한 위치(예: 지도 상의 벽 내부)인지를 검사하려면 **전체 지도를 담고 있는 `global_costmap`을 봐야 합니다.**

  * **올바른 사용법**:
    이 노드의 코드는 수정할 필요가 없습니다. `costmap_topic`을 외부에서 주입받을 수 있도록 설계되었기 때문입니다.
    대신, 이 BT 노드를 사용하는 **Behavior Tree XML 파일에서 토픽 이름을 명시적으로 `/global_costmap/costmap_raw`로 지정**해주어야 합니다.

    **BT XML 사용 예시:**

    ```xml
    <IsGoalsOccupiedCondition
        goals="{goals}"
        occupied_goals="{occupied_goals}"
        unoccupied_goals="{unoccupied_goals}"
        costmap_topic="/global_costmap/costmap_raw" />
    ```

### 최종 결론 및 권장사항

| 노드 이름 | 현재 구현 | 문제점 | **올바른 방향** |
| :--- | :--- | :--- | :--- |
| **`ReplanMonitorNode`** | `/global_costmap/costmap` (`OccupancyGrid`) 구독 | **잘못됨.** 시각화용 데이터를 사용해 Planner와 다른 기준으로 판단함. | **코드를 수정**하여 `/global_costmap/costmap_raw` (`Costmap`)를 구독해야 함. |
| **`IsGoalsOccupiedCondition`** | `.../costmap_raw` (`Costmap`) 구독 | **올바름.** 무손실 데이터를 사용. (단, 기본 토픽 이름이 부적절) | 코드는 그대로 두고, **BT XML에서 토픽을 `/global_costmap/costmap_raw`로 지정**하여 사용해야 함. |

요약하자면, 의사결정을 위한 로직에는 항상 **원본 데이터를 담고 있는 `_raw` 토픽을 사용하는 것이 철칙**입니다. `ReplanMonitorNode`는 이 원칙을 위배했으므로 반드시 수정되어야 하고, `IsGoalsOccupiedCondition`은 원칙을 잘 따랐으나 실제 사용 시 적절한 토픽을 지정해주는 주의가 필요합니다.




네, `costmap_raw` 토픽을 사용하도록 `ReplanMonitorNode`의 전체 코드를 수정하여 제공해 드리겠습니다.

수정된 코드는 Planner가 사용하는 원본 데이터를 직접 사용하므로 훨씬 더 정확하고 안정적으로 동작할 것입니다. 코드의 주요 변경 사항은 다음과 같습니다.

### 주요 변경 사항 요약

1.  **토픽 및 메시지 타입 변경**: `/global_costmap/costmap` (`nav_msgs::msg::OccupancyGrid`) 대신 `/global_costmap/costmap_raw` (`nav2_msgs::msg::Costmap`) 토픽을 구독합니다.
2.  **내부 데이터 형식 변경**: 수신한 `Costmap` 메시지를 `nav2_costmap_2d::Costmap2D` 객체로 변환하여 내부적으로 관리합니다. 이를 통해 `worldToMap`, `getCost`와 같은 편리하고 안전한 유틸리티 함수를 사용할 수 있습니다.
3.  **Costmap 콜백 로직 수정**: 메시지를 `Costmap2D` 객체로 역직렬화(deserialize)하는 로직을 구현합니다.
4.  **Cost 계산 로직 수정**: 수동으로 월드 좌표를 맵 좌표로 변환하는 대신, `Costmap2D` 객체의 내장 함수를 사용하여 더 안정적이고 명확하게 Cost를 가져옵니다.

-----

### 수정된 코드 (`replan_monitor_node.hpp`)

```cpp
#ifndef REPLAN_MONITOR_NODE_HPP
#define REPLAN_MONITOR_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include <unordered_map>
#include <mutex>
#include <cmath>
#include <limits>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ADDED: nav2_costmap_2d와 nav2_msgs::msg::Costmap 헤더 포함
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_msgs/msg/costmap.hpp"

class ReplanMonitorNode : public rclcpp::Node {
public:
    ReplanMonitorNode();

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    // CHANGED: 콜백 함수의 파라미터 타입 변경
    void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
    void evaluateReplanCondition();
    bool getCurrentPoseFromTF(geometry_msgs::msg::Pose &pose_out);

    // CHANGED: 구독자 타입 변경
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr replan_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path current_path_;
    // CHANGED: Costmap을 nav2_costmap_2d::Costmap2D 객체로 관리
    std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
    std::string costmap_frame_;

    rclcpp::Time last_replan_time_;

    std::unordered_map<int, rclcpp::Time> obstacle_seen_time_;
    std::unordered_map<int, double> obstacle_distance_history_;
    std::mutex data_mutex_;
    size_t closest_index = 0;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string target_frame_ = "base_link";
    std::string source_frame_ = "map";

    // Parameters
    double cooldown_sec_ = 1.0;
    size_t blocked_threshold_ = 2;
    double passed_pose_ignore_dist_ = 0.9;
    double obstacle_duration_threshold_sec_ = 0.5;
    double approach_threshold_dist_ = 0.1;
    double max_speed_ = 0.5;
    double lookahead_time_sec_ = 15;
    double goal_ignore_radius_ = 0.2;
    // NOTE: 이 값은 이제 0~255 스케일의 raw cost와 비교됩니다.
    double cost_threshold_ = 20; // e.g., nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE (253) 보다 낮은 값
    double immediate_block_dist_ = 0.7;
    bool immediate_replan = false;
};

#endif // REPLAN_MONITOR_NODE_HPP
```

### 수정된 코드 (`replan_monitor_node.cpp`)

```cpp
#include "replan_monitor/replan_monitor_node.hpp"
#include "nav2_costmap_2d/cost_values.hpp" // ADDED: LETHAL_OBSTACLE 등 cost 값 상수를 사용하기 위함

ReplanMonitorNode::ReplanMonitorNode()
: Node("replan_monitor_node") {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::SubscriptionOptions sub_options;
    auto callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options.callback_group = callback_group;

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/plan", rclcpp::QoS(10), std::bind(&ReplanMonitorNode::pathCallback, this, std::placeholders::_1), sub_options);

    // CHANGED: 구독 토픽과 메시지 타입 변경
    costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
        "/global_costmap/costmap_raw", rclcpp::QoS(rclcpp::SystemDefaultsQoS()).transient_local().reliable(), 
        std::bind(&ReplanMonitorNode::costmapCallback, this, std::placeholders::_1), sub_options);

    replan_pub_ = this->create_publisher<std_msgs::msg::Bool>("/replan_flag", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&ReplanMonitorNode::evaluateReplanCondition, this));

    last_replan_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "ReplanMonitorNode initialized");
}

void ReplanMonitorNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_path_ = *msg;
    closest_index = 0;
    obstacle_seen_time_.clear();
    obstacle_distance_history_.clear();
}

// CHANGED: 콜백 함수 전체 로직 변경
void ReplanMonitorNode::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    // Costmap 메시지를 nav2_costmap_2d::Costmap2D 객체로 변환
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>(
        msg->metadata.size_x, msg->metadata.size_y,
        msg->metadata.resolution, msg->metadata.origin.position.x,
        msg->metadata.origin.position.y);
    
    // memcpy를 사용하여 원본 cost 데이터를 복사
    unsigned char* char_map = costmap_->getCharMap();
    memcpy(char_map, &msg->data[0], msg->data.size() * sizeof(unsigned char));
    costmap_frame_ = msg->header.frame_id;
}

bool ReplanMonitorNode::getCurrentPoseFromTF(geometry_msgs::msg::Pose &pose_out) {
    try {
        geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
            source_frame_, target_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));

        pose_out.position.x = tf.transform.translation.x;
        pose_out.position.y = tf.transform.translation.y;
        pose_out.position.z = tf.transform.translation.z;
        pose_out.orientation = tf.transform.rotation;
        return true;
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
        return false;
    }
}

void ReplanMonitorNode::evaluateReplanCondition() {
    std::unique_lock<std::mutex> lock(data_mutex_);

    // 로컬 스코프에 costmap 포인터를 복사하여 안전하게 사용
    auto costmap = costmap_;
    if (current_path_.poses.empty() || !costmap) {
        return;
    }

    geometry_msgs::msg::Pose current_pose;
    if (!getCurrentPoseFromTF(current_pose)) {
        return;
    }

    lock.unlock(); // TF 조회 후 뮤텍스 락 해제 (긴 계산 동안 다른 콜백 방해하지 않도록)

    immediate_replan = false;
    std_msgs::msg::Bool flag_msg;
    flag_msg.data = false;

    // ... (가장 가까운 경로점 찾는 로직은 동일)
    size_t closest_index_start = (closest_index > 5) ? closest_index - 5 : 0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = closest_index_start; i < current_path_.poses.size(); ++i) {
        const auto &p = current_path_.poses[i].pose;
        double dx = p.position.x - current_pose.position.x;
        double dy = p.position.y - current_pose.position.y;
        double dist = std::hypot(dx, dy);
        if (dist < min_dist) {
            min_dist = dist;
            this->closest_index = i; // Use member variable
        }
    }
    
    rclcpp::Time now = this->now();
    double lookahead_distance = max_speed_ * lookahead_time_sec_;
    const auto &goal_pose = current_path_.poses.back().pose;
    if (!std::isfinite(goal_pose.position.x) || !std::isfinite(goal_pose.position.y)) return;
    size_t blocked = 0;

    for (size_t i = this->closest_index; i < current_path_.poses.size(); ++i) {
        const auto &pose = current_path_.poses[i].pose;
        double dist = std::hypot(pose.position.x - current_pose.position.x, pose.position.y - current_pose.position.y);
        if (dist < passed_pose_ignore_dist_ || dist > lookahead_distance) continue;

        double goal_dist = std::hypot(pose.position.x - goal_pose.position.x, pose.position.y - goal_pose.position.y);
        if (goal_dist < goal_ignore_radius_) continue;

        // CHANGED: Costmap2D 유틸리티를 사용하여 cost 값 획득
        unsigned int mx, my;
        if (!costmap->worldToMap(pose.position.x, pose.position.y, mx, my)) {
            continue; // 경로가 costmap 밖에 있는 경우 무시
        }
        
        unsigned char cost = costmap->getCost(mx, my);
        int index = costmap->getIndex(mx, my); // 고유 식별을 위해 맵 인덱스 사용

        if (cost >= cost_threshold_) {
            if (dist < immediate_block_dist_) {
                immediate_replan = true;
            }
            // ... (나머지 장애물 지속 시간 및 접근 감지 로직은 동일)
            auto it = obstacle_seen_time_.find(index);
            if (it == obstacle_seen_time_.end()) {
                obstacle_seen_time_[index] = now;
                obstacle_distance_history_[index] = dist;
                continue;
            }

            rclcpp::Duration duration = now - it->second;
            bool is_approaching = false;
            if (obstacle_distance_history_.count(index)) {
                double prev_dist = obstacle_distance_history_[index];
                if (prev_dist - dist > approach_threshold_dist_) is_approaching = true;
                obstacle_distance_history_[index] = dist;
            }
            
            if (duration.seconds() >= obstacle_duration_threshold_sec_ && is_approaching) {
                blocked++;
            }
        } else {
            obstacle_seen_time_.erase(index);
            obstacle_distance_history_.erase(index);
        }

        if (immediate_replan) {
            flag_msg.data = true;
            break; // 즉시 replan이므로 더 이상 검사할 필요 없음
        }
    }

    // ... (replan 발행 로직은 거의 동일, last_replan_time_ 접근 시 뮤텍스 필요)
    lock.lock(); // 멤버 변수 접근 전 다시 락
    if (flag_msg.data || (blocked >= blocked_threshold_ && (now - last_replan_time_).seconds() > cooldown_sec_)) {
        flag_msg.data = true;
        last_replan_time_ = now;
        replan_pub_->publish(flag_msg);
        if (immediate_replan) {
            RCLCPP_WARN(this->get_logger(), "Triggering replan: immediate block detected.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Triggering replan: %zu path points consistently blocked.", blocked);
        }
    }
}
```

### `cost_threshold_` 파라미터에 대한 중요 정보

이제 코드는 0\~255 범위의 원본 cost 값을 사용합니다. 따라서 `cost_threshold_` 파라미터의 의미가 달라졌습니다.

  * `0`: `FREE_SPACE`
  * `1` \~ `252`: `INSCRIBED_INFLATED_OBSTACLE` (Inflation 영역, 숫자가 클수록 장애물에 가까움)
  * `253`: `LETHAL_OBSTACLE` (실제 장애물)
  * `254`: `NO_INFORMATION` (알 수 없는 영역)

기존의 `cost_threshold_ = 20`은 inflation 영역의 낮은 cost 구역을 의미합니다. 이는 여전히 유효할 수 있지만, 이제는 `nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE` (253)과 같은 명시적인 값을 기준으로 파라미터를 설정하는 것을 고려할 수 있습니다. 예를 들어, 로봇의 내접 반경(inscribed radius)이 닿는 영역부터 replan을 고려하고 싶다면 `cost_threshold_`를 `253`에 가깝게 설정할 수 있습니다.

### 빌드 설정 업데이트

이 코드를 빌드하려면 `nav2_msgs` 패키지에 대한 의존성을 추가해야 합니다.

**`package.xml`**

```xml
<depend>nav2_msgs</depend>
```

**`CMakeLists.txt`**

```cmake
find_package(nav2_msgs REQUIRED)
# ...
ament_target_dependencies(replan_monitor_node
  # ... other dependencies
  nav2_msgs
)
```

이제 이 코드는 Nav2 Planner와 동일한 데이터를 사용하여 훨씬 더 정확하게 Re-planning 시점을 결정할 수 있습니다.
