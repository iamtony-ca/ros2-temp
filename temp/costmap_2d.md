ROS 2 Nav2에서 `global_costmap`의 `update_frequency`와 `publish_frequency`를 다르게 설정하는 것은 CPU 및 네트워크 리소스 최적화를 위한 아주 전형적이고 중요한 패턴입니다. 

제공해주신 `costmap_2d_ros.cpp` 코드를 바탕으로 이 두 파라미터가 어떻게 다르게 동작하는지 정확히 분석해 드리겠습니다.

---

### 1. `update_frequency` (5.0 Hz) : 내부 상태 갱신 주기

`update_frequency`는 **Costmap이 센서 데이터, TF(로봇 위치), 그리고 각 레이어(Static, Obstacle, Inflation 등)의 플러그인 데이터를 수집하여 메모리 상의 내부 그리드(Grid) 맵을 재계산하는 주기**를 의미합니다.

코드에서 이 파라미터는 `map_update_frequency_` 멤버 변수로 저장되며, Costmap의 핵심 구동 스레드인 `mapUpdateLoop`의 루프 주기를 결정합니다.

**관련 코드 포인트:**
```cpp
// on_activate() 함수 내부: 업데이트 스레드 생성
map_update_thread_ = std::make_unique<std::thread>(
  std::bind(&Costmap2DROS::mapUpdateLoop, this, map_update_frequency_));
```

```cpp
// mapUpdateLoop() 함수 내부:
void Costmap2DROS::mapUpdateLoop(double frequency)
{
  // ... (생략) ...
  rclcpp::WallRate r(frequency);    // update_frequency(5.0Hz)에 맞춰 루프 주기 설정

  while (rclcpp::ok() && !map_update_thread_shutdown_) {
    // ...
      updateMap(); // <- 실제 레이어들의 데이터를 모아 맵을 업데이트 (내부 연산)
    // ...
    r.sleep(); // 5.0Hz(약 200ms) 주기를 맞추기 위해 대기
  }
}
```
* **의미:** Nav2의 글로벌 플래너(예: NavFn, Smac)가 경로를 계획할 때 읽어가는 메모리 상의 Costmap은 이 주기로 갱신됩니다. 5.0Hz 설정은 로봇이 1초에 5번 새로운 장애물 정보를 내부 맵에 반영한다는 뜻입니다.

### 2. `publish_frequency` (2.0 Hz) : 외부 토픽 발행 주기

`publish_frequency`는 **업데이트된 내부 Costmap 데이터를 ROS 2 토픽(`~/costmap`, `~/costmap_updates`)으로 Serialize하여 퍼블리시하는 주기**를 의미합니다. 주로 RViz에서 시각화를 하거나, 외부 노드에서 Costmap 데이터를 구독해야 할 때 사용됩니다.

코드에서 이 값은 `publish_cycle_` (주기 시간)로 변환되어 저장되며, **별도의 스레드가 아닌 `mapUpdateLoop` 스레드 내부에서 조건문으로 동작**합니다.

**관련 코드 포인트:**
```cpp
// getParameters() 내부: frequency를 duration으로 변환
if (map_publish_frequency_ > 0) {
  publish_cycle_ = rclcpp::Duration::from_seconds(1 / map_publish_frequency_);
}
```

```cpp
// mapUpdateLoop() 함수 내부: updateMap() 직후에 실행됨
      updateMap(); // 5.0Hz로 매번 실행됨
      
      // ... (바운딩 박스 업데이트 생략) ...

      auto current_time = now();
      // 마지막 퍼블리시 시간으로부터 publish_cycle_(0.5초, 즉 2.0Hz)이 지났는지 확인
      if ((last_publish_ + publish_cycle_ < current_time) || 
          (current_time < last_publish_))      
      {
        RCLCPP_DEBUG(get_logger(), "Publish costmap at %s", name_.c_str());
        costmap_publisher_->publishCostmap(); // <- 조건이 맞을 때만 토픽 발행

        for (auto & layer_pub : layer_publishers_) {
          layer_pub->publishCostmap();
        }

        last_publish_ = current_time; // 발행 시간 갱신
      }
```
* **의미:** 내부 맵은 5번(5.0Hz) 업데이트 되지만, 그 업데이트된 결과를 ROS 2 네트워크를 통해 쏘는 것은 2번(2.0Hz)만 하겠다는 뜻입니다.

---

### 💡 실무 적용 포인트 (왜 이렇게 설정하는가?)

1.  **네트워크 및 CPU 최적화:** * 글로벌 코스트맵은 로봇의 전체 환경을 담고 있어 데이터(OccupancyGrid) 크기가 매우 큽니다.
    * 이 거대한 데이터를 5.0Hz로 매번 퍼블리시하면 ROS 2 미들웨어(DDS) 네트워크 트래픽이 폭주하고 CPU 점유율이 급증할 수 있습니다.
2.  **데이터의 목적이 다름:**
    * **내부 연산용 (`update_frequency`):** 경로 계획 알고리즘(Planner)은 토픽을 구독하지 않고 `Costmap2D` C++ 객체의 포인터를 직접 참조하여 메모리에서 빠르게 데이터를 읽습니다. 따라서 `updateMap()`은 자주 실행되어야 안전합니다.
    * **시각화용 (`publish_frequency`):** 토픽 발행의 주된 목적은 개발자의 RViz 모니터링입니다. 사람이 눈으로 확인하는 데에는 2.0Hz(0.5초에 한 번 갱신)면 충분합니다.

**결론적으로:** 작성하신 파라미터 세팅(Update 5.0, Publish 2.0)은 **"경로 계획을 위해 맵은 초당 5번 빠르게 갱신하되, RViz용 시각화 토픽은 무거우니 초당 2번만 쏴서 리소스를 아끼겠다"**는 아주 올바르고 효율적인 코드 설계입니다.


##
##
##
`always_send_full_costmap` 파라미터는 **ROS 2 네트워크 대역폭(Bandwidth) 최적화**와 직결되는 핵심 파라미터입니다. 

로봇이 주행할 때 맵 전체가 변하는 것이 아니라 센서에 감지된 로봇 주변(일부 영역)만 변하기 때문에, 맵 데이터를 퍼블리시하는 방식을 결정하는 역할을 합니다.

제공해주신 `costmap_2d_publisher.cpp`의 `publishCostmap()` 함수를 중심으로 이 파라미터가 어떻게 작동하는지 상세히 분석해 드리겠습니다.

---

### 1. 핵심 로직 분석: `publishCostmap()`

코드에서 토픽을 발행하는 분기점은 `publishCostmap()` 함수 내의 `if - else if` 문에 있습니다.

```cpp
void Costmap2DPublisher::publishCostmap()
{
  float resolution = costmap_->getResolution();
  
  // 조건 1: always_send_full_costmap_가 true이거나, 맵의 메타데이터(크기, 해상도, 원점)가 변경되었을 때
  if (always_send_full_costmap_ || grid_resolution_ != resolution ||
    grid_width_ != costmap_->getSizeInCellsX() ||
    grid_height_ != costmap_->getSizeInCellsY() ||
    saved_origin_x_ != costmap_->getOriginX() ||
    saved_origin_y_ != costmap_->getOriginY())
  {
    // ... [전체 맵 퍼블리시 로직] ...
    if (costmap_pub_->get_subscription_count() > 0) {
      prepareGrid();
      costmap_pub_->publish(std::move(grid_)); // ~/costmap 발행
    }
    // ...
  } 
  // 조건 2: 맵 메타데이터는 그대로고, 업데이트된 바운딩 박스(x0, xn)가 존재할 때
  else if (x0_ < xn_) {
    // ... [부분 업데이트 퍼블리시 로직] ...
    if (costmap_update_pub_->get_subscription_count() > 0) {
      costmap_update_pub_->publish(createGridUpdateMsg()); // ~/costmap_updates 발행
    }
    // ...
  }
  // ...
}
```

### 2. 파라미터 설정에 따른 구체적 동작 차이

#### A. `always_send_full_costmap = true` 인 경우
첫 번째 `if` 조건이 무조건 참(True)이 됩니다. 
* **호출 함수:** `prepareGrid()` 와 `prepareCostmap()`이 호출됩니다.
* **작동 방식:** ```cpp
  unsigned char * data = costmap_->getCharMap();
  for (unsigned int i = 0; i < grid_->data.size(); i++) {
    grid_->data[i] = cost_translation_table_[data[i]];
  }
  ```
  내부 Costmap 배열 전체를 복사하여 `nav_msgs::msg::OccupancyGrid` 메시지를 만듭니다.
* **발행 토픽:** `~/costmap` (기본 맵) 및 `~/costmap_raw`
* **결과:** 이전에 설명한 `publish_frequency`(예: 2.0Hz)마다 **맵 전체 데이터를 네트워크로 전송**합니다.

#### B. `always_send_full_costmap = false` 인 경우 (기본값)
첫 번째 `if` 조건은 맵의 크기나 원점이 바뀌었을 때(예: SlamToolbox 등에 의해 맵이 확장되었을 때)만 참이 됩니다. 일반적인 주행 중에는 `else if` 문으로 빠지게 됩니다.
* **호출 함수:** `createGridUpdateMsg()` 와 `createCostmapUpdateMsg()`가 호출됩니다.
* **작동 방식:**
  ```cpp
  update->x = x0_;
  update->y = y0_;
  update->width = xn_ - x0_;
  update->height = yn_ - y0_;
  
  std::uint32_t i = 0;
  for (std::uint32_t y = y0_; y < yn_; y++) {
    for (std::uint32_t x = x0_; x < xn_; x++) {
      update->data[i++] = cost_translation_table_[costmap_->getCost(x, y)];
    }
  }
  ```
  Costmap 내부에서 변경이 발생한 **최소 바운딩 박스(Bounding Box: `x0_` ~ `xn_`, `y0_` ~ `yn_`) 영역의 데이터만 추출**합니다.
* **발행 토픽:** `~/costmap_updates` 및 `~/costmap_raw_updates` (`map_msgs::msg::OccupancyGridUpdate` 타입)
* **결과:** 첫 1회(또는 맵 크기 변경 시)에만 전체 맵(`~/costmap`)을 쏘고, 이후부터는 바뀐 영역의 조각 맵(`~/costmap_updates`)만 쏩니다. RViz나 다른 노드는 이 두 토픽을 조합해서 전체 맵을 화면에 그립니다.

---

### 💡 실무 관점에서의 요약 (DDS 및 리소스 관리)

모바일 매니퓰레이터나 다중 로봇 시스템(Multi-robot control) 환경에서는 네트워크 트래픽 관리가 매우 중요합니다.

* **왜 `false` (부분 업데이트)를 써야 하는가?**
  글로벌 코스트맵이 4000x4000 셀 크기라고 가정하면, `OccupancyGrid` 메시지 하나가 약 16MB입니다. 이를 2Hz로 퍼블리시하면 초당 32MB의 트래픽이 발생하며, DDS 통신에 엄청난 병목(Overhead)을 유발합니다. `false`로 두면 변경된 수십~수백 픽셀만 전송하므로 트래픽이 극적으로 줄어듭니다.
* **그럼 `true`는 언제 쓰는가?**
  디버깅 목적이거나, 구독하는 외부 노드(또는 자체 제작한 Custom Node)가 `OccupancyGridUpdate` 타입을 지원하지 않고 오직 `OccupancyGrid`만 받아서 처리할 수 있도록 짜여진 경우, 강제로 전체 맵을 쏘게 만들기 위해 사용합니다. (Nav2의 로컬 코스트맵은 기본적으로 범위가 작기 때문에 트래픽 부담이 적어 이 파라미터의 영향이 상대적으로 적습니다).
