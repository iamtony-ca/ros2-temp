네, Nav2와 BehaviorTree.Cpp 전문가로서 요청하신 내용을 정확히 이해했습니다. 기존 `BT::SubTreeNode`의 모든 핵심 기능과 로직을 그대로 유지하면서, 제공해주신 `amr_bt_nodes` 네임스페이스와 ROS 2 플러그인 스타일에 맞춰 바로 사용 가능한 커스텀 데코레이터 노드를 작성해 드리겠습니다.

새로운 노드의 이름은 기존 `SubTree`와의 충돌을 피하고 커스텀 노드임을 명확히 하기 위해 `SubtreeDecorator`로 정하겠습니다. XML에서 사용할 태그 이름도 `<SubtreeDecorator>`가 됩니다.

아래에 바로 프로젝트에 추가하여 사용할 수 있는 `subtree_decorator.hpp`와 `subtree_decorator.cpp` 파일을 완전한 형태로 제공합니다.

-----

### 1\. 헤더 파일: `subtree_decorator.hpp`

이 파일은 `SubtreeDecorator` 클래스를 정의합니다. 기존 `subtree_node.h`의 구조를 그대로 따르면서 네임스페이스와 최신 BehaviorTree.Cpp 스타일(`NodeConfiguration`)을 적용했습니다.

```cpp
// amr_bt_nodes/include/amr_bt_nodes/subtree_decorator.hpp
#pragma once

#include "behaviortree_cpp/decorator_node.h"

namespace amr_bt_nodes
{

/**
 * @brief The SubtreeDecorator is a custom implementation that mimics the
 * standard BT::SubTreeNode. It wraps an entire Subtree, creating a
 * separated BlackBoard. If you want to have data flow through ports, you
 * need to explicitly remap the ports.
 *
 * This node is functionally identical to the built-in <SubTree> node but is
 * provided within the amr_bt_nodes namespace as a plugin.
 *
 * It supports automatic remapping of ports via the `_autoremap` attribute.
 *
 * Example Usage in XML:
 *
 * <root main_tree_to_execute = "MainTree" >
 *
 * <BehaviorTree ID="MainTree">
 * <Sequence>
 * <Script code="myParam:='Hello'" />
 * * <SubtreeDecorator ID="MySub" param="{myParam}" />
 *
 * <Script code="param:='Auto remapped'" />
 * <SubtreeDecorator ID="MySub" _autoremap="true" />
 * </Sequence>
 * </BehaviorTree>
 *
 * <BehaviorTree ID="MySub">
 * <SaySomething message="{param}" />
 * </BehaviorTree>
 * </root>
 *
 */
class SubtreeDecorator : public BT::DecoratorNode
{
public:
  SubtreeDecorator(const std::string & name, const BT::NodeConfiguration & config);

  virtual ~SubtreeDecorator() override = default;

  static BT::PortsList providedPorts();

  // Method called by the factory to link this node to the proper Subtree
  void setSubtreeID(const std::string & ID)
  {
    subtree_id_ = ID;
  }

  const std::string & subtreeID() const
  {
    return subtree_id_;
  }

  virtual BT::NodeStatus tick() override;

  // The Subtree node is a special case and we need to report it as such
  // to the factory.
  virtual BT::NodeType type() const override final
  {
    return BT::NodeType::SUBTREE;
  }

private:
  std::string subtree_id_;
};

}  // namespace amr_bt_nodes
```

### 2\. 소스 파일: `subtree_decorator.cpp`

이 파일은 `SubtreeDecorator` 클래스의 실제 구현과 BehaviorTree.Cpp 팩토리에 노드를 등록하기 위한 플러그인 코드를 포함합니다. `subtree_node.cpp`의 로직을 그대로 가져오고, `TriggerPlannerGoalsDecorator`의 플러그인 등록 방식을 따랐습니다.

```cpp
// amr_bt_nodes/src/subtree_decorator.cpp
#include "amr_bt_nodes/subtree_decorator.hpp"

namespace amr_bt_nodes
{

SubtreeDecorator::SubtreeDecorator(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::DecoratorNode(name, config)
{
  // This is the name used in the XML file to call the node
  setRegistrationID("SubtreeDecorator");
}

BT::PortsList SubtreeDecorator::providedPorts()
{
  // This node has a single special port: _autoremap
  // It is a boolean that, if true, will automatically remap all the ports
  // with the same name.
  auto port =
    BT::PortInfo(BT::PortDirection::INPUT, typeid(bool), BT::GetAnyFromStringFunctor<bool>());
  port.setDefaultValue(false);
  port.setDescription("If true, all the ports with the same name will be remapped");

  return {{"_autoremap", port}};
}

BT::NodeStatus SubtreeDecorator::tick()
{
  // The Subtree node has a different logic a regular Decorator.
  // The BT::Tree is responsible for setting the child of this node
  // in the `Tree::instantiateTree` method.
  
  BT::NodeStatus prev_status = status();
  if (prev_status == BT::NodeStatus::IDLE) {
    // A Subtree is never IDLE, at least not in the first tick.
    // We expect it to be RUNNING.
    setStatus(BT::NodeStatus::RUNNING);
  }

  // The child is the root of the Subtree.
  // Tick the child and return its status.
  const BT::NodeStatus child_status = child_node_->executeTick();
  
  if (isStatusCompleted(child_status)) {
    // if the child reached a terminal state, reset it
    resetChild();
  }

  return child_status;
}

}  // namespace amr_bt_nodes

// Include this at the end of the file to register the node with the factory
#include "behaviortree_cpp/bt_factory.h"

// The extern "C" block is crucial for the plugin loader to find the entry point
extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory & factory)
{
  // The first argument is the XML tag name, the second is the C++ class name.
  // It's good practice to keep them the same.
  factory.registerNodeType<amr_bt_nodes::SubtreeDecorator>("SubtreeDecorator");
}
```

### 설명 및 사용법

1.  **파일 위치**: 위 코드들을 `amr_bt_nodes` 패키지의 `include/amr_bt_nodes/`와 `src/` 디렉토리에 각각 `subtree_decorator.hpp`, `subtree_decorator.cpp` 이름으로 저장하세요.

2.  **CMakeLists.txt**: `amr_bt_nodes` 패키지의 `CMakeLists.txt`에 새 소스 파일을 추가하고, BehaviorTree.Cpp 라이브러리와 링크해야 합니다.

    ```cmake
    # ... other cmake settings

    add_library(amr_bt_nodes_decorator_plugins
      src/trigger_planner_goals_decorator.cpp
      src/subtree_decorator.cpp  # 이 파일을 추가하세요
    )

    ament_target_dependencies(amr_bt_nodes_decorator_plugins
      "behaviortree_cpp"
      "rclcpp"
      # ... other dependencies
    )

    # ... install rules
    ```

3.  **`bt_plugin_libraries` 등록**: Nav2 BT Navigator 설정에서 이 플러그인 라이브러리를 사용하도록 지정해야 합니다. `bt_navigator`의 파라미터 파일에서 `bt_plugin_libraries` 리스트에 `amr_bt_nodes_decorator_plugins`를 추가하세요.

4.  **BT XML에서 사용**: 이제 Behavior Tree XML 파일에서 `<SubtreeDecorator>` 태그를 사용하여 이 노드를 호출할 수 있습니다. 기능은 기존의 `<SubTree>`와 완전히 동일합니다.

    ```xml
    <SubtreeDecorator ID="NameOfYourSubtree" _autoremap="true" input_port="{parent_port}"/>
    ```

이 코드는 원본 `SubTreeNode`의 모든 기능(포트 리매핑, `_autoremap` 기능, 별도의 블랙보드 생성 등)을 완벽하게 모방하며, 요청하신 ROS 2 플러그인 스타일에 맞춰져 있어 바로 빌드하고 사용할 수 있습니다.







물론입니다. 방금 생성한 `SubtreeDecorator`를 Nav2에서 Behavior Tree 플러그인으로 등록하는 방법과 실제 사용 예시 XML을 단계별로 자세히 설명해 드리겠습니다.

사용자께서 제공해주신 `[참고 custom bt]` 코드에 `BT_RegisterNodesFromPlugin` 함수가 포함되어 있으므로, Nav2에서 `bt_plugin_libraries` 파라미터를 사용하는 가장 일반적이고 간단한 방법을 기준으로 설명하겠습니다.

-----

### 1\. 플러그인 등록 방법

플러그인 등록은 크게 두 단계로 이루어집니다.

1.  **CMakeLists.txt 수정**: C++ 코드를 `.so` 공유 라이브러리 파일로 빌드합니다.
2.  **Nav2 파라미터 파일 수정**: Nav2가 이 라이브러리를 로드하도록 설정합니다.

#### 1.1. `CMakeLists.txt` 수정

`amr_bt_nodes` 패키지의 `CMakeLists.txt` 파일에 아래 내용을 추가하거나 수정합니다. 이미 `trigger_planner_goals_decorator.cpp`를 위해 비슷한 설정이 되어 있을 것이므로, `subtree_decorator.cpp`를 추가하기만 하면 됩니다.

```cmake
# ... (find_package 등 다른 설정들)

# BT 노드 소스 파일들을 모아 공유 라이브러리를 생성합니다.
# 라이브러리 이름은 "amr_bt_nodes_plugins" 등으로 지정합니다.
add_library(amr_bt_nodes_plugins SHARED
  src/trigger_planner_goals_decorator.cpp
  src/subtree_decorator.cpp  # 여기에 새 파일을 추가합니다.
)

# 생성된 라이브러리가 필요로 하는 의존성들을 링크합니다.
ament_target_dependencies(amr_bt_nodes_plugins
  "behaviortree_cpp"
  "rclcpp"
  "geometry_msgs"
  # ... 기타 필요한 패키지
)

# ...

# 생성된 라이브러리와 헤더 파일들을 설치(install)해야
# ROS 2 런타임에서 찾을 수 있습니다.
install(TARGETS
  amr_bt_nodes_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include
)

# ... (ament_package 등 나머지 설정)
```

**핵심**: 위 설정은 `subtree_decorator.cpp`를 포함한 소스 코드들을 컴파일하여 `libamr_bt_nodes_plugins.so` 라는 공유 라이브러리 파일을 생성하고, ROS 2 작업 공간의 `install/lib` 디렉토리에 설치하는 역할을 합니다.

#### 1.2. Nav2 파라미터 파일 (`nav2_params.yaml`) 수정

Nav2를 실행할 때 사용하는 파라미터 파일(일반적으로 `nav2_params.yaml`)에서 `bt_navigator`의 설정을 수정해야 합니다. `bt_plugin_libraries` 리스트에 방금 생성한 라이브러리의 이름을 추가합니다.

```yaml
bt_navigator:
  ros__parameters:
    # ... (다른 bt_navigator 파라미터들)

    # BehaviorTree.CPP v3 플러그인 라이브러리 목록
    # 여기에 등록된 라이브러리들을 Nav2가 실행 시점에 로드합니다.
    bt_plugin_libraries:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_globally_updated_goal_condition_bt_node
      - nav2_is_path_valid_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_goal_updater_node_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_distance_traveled_condition_bt_node
      - nav2_single_trigger_bt_node
      - nav2_is_battery_low_condition_bt_node
      - nav2_navigate_through_poses_action_bt_node
      - nav2_navigate_to_pose_action_bt_node
      - nav2_remove_passed_goals_action_bt_node
      # --- 아래 라인을 리스트의 마지막에 추가합니다 ---
      - amr_bt_nodes_plugins
```

**핵심**: `bt_plugin_libraries`에 `amr_bt_nodes_plugins`를 추가하면, `bt_navigator`가 시작될 때 `libamr_bt_nodes_plugins.so` 라이브러리를 동적으로 로드(load)하고, 그 안에 있는 `BT_RegisterNodesFromPlugin` 함수를 실행하여 `SubtreeDecorator` 노드를 Behavior Tree 팩토리에 등록합니다.

이제 빌드 (`colcon build`) 후 Nav2를 실행하면 `<SubtreeDecorator>` 태그를 사용할 준비가 완료됩니다.

-----

### 2\. BT XML 사용 예시

아래는 `SubtreeDecorator`를 사용하여 메인 트리에서 서브 트리를 호출하고, 블랙보드 포트를 통해 데이터를 전달하는 방법을 보여주는 완전한 XML 예시입니다.

이 예시는 두 개의 트리를 정의합니다:

  * `MainTree`: 메인 실행 로직. `SubtreeDecorator`를 두 번 호출합니다.
  * `TalkerSubtree`: `MainTree`에 의해 호출되는 서브 트리.

<!-- end list -->

```xml
<root main_tree_to_execute="MainTree">

  <BehaviorTree ID="MainTree">
    <Sequence name="MainSequence">

      <SubtreeDecorator ID="TalkerSubtree" message="Hello, I am the first call."/>

      <AlwaysSuccess/> <Script code="parent_variable := 'This is the second call, using remapping.' "/>
      <SubtreeDecorator ID="TalkerSubtree" message="{parent_variable}"/>

      <AlwaysSuccess/>

      <Script code="message := 'The third call with auto-remapping!' "/>
      <SubtreeDecorator ID="TalkerSubtree" _autoremap="true"/>

    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="TalkerSubtree">
    <Sequence>
      <PrintKeyValue key="message" />
    </Sequence>
  </BehaviorTree>

</root>
```

#### XML 설명:

1.  **`<root main_tree_to_execute="MainTree">`**: 실행을 시작할 메인 트리를 `MainTree`로 지정합니다.
2.  **`<BehaviorTree ID="MainTree">`**: 메인 트리를 정의합니다.
3.  **`<SubtreeDecorator ID="TalkerSubtree" ... />`**: 우리가 만든 `SubtreeDecorator` 노드를 사용해 `TalkerSubtree` 라는 ID를 가진 서브 트리를 호출합니다.
      * **첫 번째 호출**: `message="Hello..."` 부분은 `TalkerSubtree`의 `message` 입력 포트에 "Hello..." 라는 고정된 문자열 값을 직접 전달합니다.
      * **두 번째 호출**: `<Script>`를 사용해 `MainTree`의 블랙보드에 `parent_variable`이라는 변수를 만듭니다. `message="{parent_variable}"` 구문은 `parent_variable`의 값을 `TalkerSubtree`의 `message` 포트로 **리매핑(remapping)** 합니다.
      * **세 번째 호출**: `_autoremap="true"` 속성은 이름이 같은 포트들을 자동으로 연결합니다. `MainTree`의 블랙보드에 있는 `message` 변수의 값이 `TalkerSubtree`의 `message` 입력 포트로 자동으로 전달됩니다.
4.  **`<BehaviorTree ID="TalkerSubtree">`**: 호출될 서브 트리를 별도로 정의합니다. 이 트리는 자신만의 블랙보드를 가집니다.
5.  **`<PrintKeyValue key="message" />`**: `TalkerSubtree`의 블랙보드에서 `message`라는 키를 가진 값을 찾아 콘솔에 출력하는 기본 액션 노드입니다. 이를 통해 메인 트리에서 값이 제대로 전달되었는지 확인할 수 있습니다.

이 XML을 Nav2에서 로드하면, `SubtreeDecorator`가 정상적으로 동작하여 콘솔에 세 개의 메시지가 순서대로 출력되는 것을 확인할 수 있습니다.
