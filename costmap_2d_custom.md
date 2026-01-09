네, Nav2(Jazzy) 전문가로서 답변드립니다.

결론부터 말씀드리면 **가능합니다.** 심지어 매우 일반적인 요구사항입니다.
`global_costmap`은 본질적으로 `nav2_costmap_2d::Costmap2DROS`라는 C++ 객체가 생성하는 데이터일 뿐이므로, 이를 **별도의 노드로 띄우거나 네임스페이스를 분리하여 실행**하면 원하시는 대로 2개의 서로 다른 Global Costmap을 운용할 수 있습니다.

사용자분의 목적(기존 Nav2용 vs 커스텀 노드용)에 맞춰 가장 깔끔한 두 가지 방법을 제안해 드립니다.

---

### 방법 1: `nav2_costmap_2d` 노드를 별도로 하나 더 띄우기 (추천)

가장 표준적인 방법입니다. 기존의 `planner_server`가 관리하는 Global Costmap은 그대로 두고, **Costmap 기능만 수행하는 독립적인 노드**를 하나 더 실행시키는 방식입니다.

이 방식은 기존 Nav2 시스템을 건드리지 않고, 완전히 독립된 파라미터(`params.yaml`)를 가질 수 있어 관리가 쉽습니다.

#### 1. Launch 파일 작성 (Python)

기존 Nav2 런치 파일 외에, 커스텀 코스트맵을 위한 런치 파일을 추가로 만듭니다. 여기서 핵심은 **Remapping**과 **Namespace**입니다.

```python
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 파라미터 파일 경로 설정
    params_file = 'path/to/your/custom_costmap_params.yaml'

    # 커스텀 Costmap 노드
    # nav2_costmap_2d 패키지의 실행파일을 직접 사용합니다.
    custom_costmap_node = LifecycleNode(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap_custom_node',
        namespace='custom_space',  # 네임스페이스로 분리
        output='screen',
        parameters=[params_file],
        # 여기서 토픽 이름을 원하는 대로 바꿉니다.
        # 네임스페이스가 있으므로 실제로는 /custom_space/global_costmap/costmap 등이 됨
        # 만약 네임스페이스 없이 이름만 바꾸고 싶다면 아래처럼 remapping 사용
        remappings=[
            ('/global_costmap/costmap', '/global_costmap/costmap_custom'),
            ('/global_costmap/costmap_raw', '/global_costmap/costmap_raw_custom'),
            ('/global_costmap/costmap_updates', '/global_costmap/costmap_updates_custom'),
            # footprint도 필요하다면 분리
            ('/global_costmap/published_footprint', '/global_costmap/published_footprint_custom'), 
        ]
    )

    # Lifecycle Manager (Jazzy에서는 필수)
    # 이 노드를 활성화(Active) 상태로 만들어줘야 토픽이 나옵니다.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap_custom',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': ['global_costmap_custom_node']} # 위에서 정한 노드 이름
        ]
    )

    return LaunchDescription([
        custom_costmap_node,
        lifecycle_manager
    ])

```

#### 2. Params 파일 (`custom_costmap_params.yaml`)

기존 `nav2_params.yaml`의 global_costmap 섹션을 복사해서 입맛에 맞게 수정합니다.

```yaml
global_costmap_custom_node:
  ros__parameters:
    # 주의: 여기서 global_costmap 키를 그대로 사용합니다. 
    # (코드 내부적으로 이 이름을 참조하는 경우가 많음)
    global_costmap:
      ros__parameters:
        update_frequency: 5.0
        publish_frequency: 2.0
        global_frame: map
        robot_base_frame: base_link
        use_sim_time: False
        robot_radius: 0.22
        resolution: 0.05
        track_unknown_space: true
        plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
        # ... 필요한 레이어 설정 ...

```

---

### 방법 2: 커스텀 노드 내부에서 `Costmap2DROS` 객체 생성 (C++ 개발자용)

만약 "내가 만든 노드"가 단순히 토픽을 subscribe 하는 것이 아니라, **직접 C++ 레벨에서 Costmap 데이터를 활용**해야 한다면(예: 충돌 체크, 독자적인 경로 계획 등), 굳이 토픽 통신을 거칠 필요 없이 **내 노드 안에 Costmap 객체를 심는 것**이 성능상 훨씬 유리합니다.

사용자분이 C++ 개발자시므로 이 방법이 더 적합할 수도 있습니다.

#### C++ 코드 예시

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

class MyCustomNode : public rclcpp::Node
{
public:
  MyCustomNode() : Node("my_custom_node")
  {
    // Costmap2DROS 객체 생성
    // 내부적으로 파라미터를 읽어들이고, 레이어를 구성하며, 
    // 설정에 따라 /global_costmap/costmap_custom 토픽도 발행할 수 있습니다.
    
    // 'global_costmap'이라는 이름으로 서브 노드 개념을 생성
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
        "global_costmap_custom"); 

    // 중요: 별도의 스레드에서 Costmap을 돌려야 메인 로직이 안 멈춥니다.
    costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);
  }

  void on_configure()
  {
      // Lifecycle 관리 (on_configure 단계에서 수행)
      costmap_ros_->on_configure(this->get_node_options().state());
  }
  
  void on_activate()
  {
      costmap_ros_->on_activate(this->get_node_options().state());
  }

private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
};

```

이 경우 `params.yaml`에서 이 노드의 네임스페이스 아래에 `global_costmap_custom` 항목을 만들어두면, 해당 설정에 따라 코스트맵이 생성되고 토픽 이름은 `~global_costmap_custom/costmap` 형태가 됩니다.

---

### 전문가의 조언 (Tips)

1. **리소스 관리**: Global Costmap은 메모리(Map 크기에 비례)와 CPU(레이어 업데이트 연산)를 많이 사용합니다.
* 두 개의 Global Costmap이 완전히 동일한 `static_layer`(지도)를 사용한다면, **메모리 낭비**가 심할 수 있습니다.
* 커스텀 노드의 목적이 단순히 "장애물만 다르게 보고 싶다"라면, **Layer Plugin**을 새로 만들거나 Filter를 쓰는 것이 더 가볍습니다. 하지만 "완전히 다른 해상도"나 "다른 Update Frequency"가 필요하다면 위에서 제안한 **방법 1(별도 노드 실행)**이 정답입니다.


2. **TF 부하**: 두 개의 코스트맵 모두 `tf`를 리스닝하며 로봇 위치를 계산합니다. `tf` 트리가 복잡하다면 CPU 부하가 늘어날 수 있습니다.
3. **Jazzy 호환성**: Jazzy 버전에서도 `nav2_costmap_2d` 패키지 구조는 크게 변하지 않았으므로 위 코드는 그대로 동작합니다. 단, Lifecycle 관리가 엄격하므로 `Lifecycle Manager`를 잊지 마세요.

**요약하자면,** `nav2_costmap_2d` 실행 파일을 `remappings`와 함께 별도의 `LifecycleNode`로 띄우는 **방법 1**을 적용하시면 원하시는 `global_costmap/costmap_custom` 토픽을 깔끔하게 얻으실 수 있습니다.
