네, 알겠습니다. 요청하신 대로 `amr_bt_nodes` 네임스페이스와 `RunNTimesDecorator` 클래스 이름을 사용하고, `.hpp`와 `.cpp` 파일을 완벽하게 분리하여 플러그인 형태로 작성해 드리겠습니다.

참고용으로 주신 코드는 ROS 2 노드 핸들을 필요로 하지만, 이 데코레이터는 순수하게 BehaviorTree.CPP 프레임워크 내에서만 동작하므로 ROS 2 의존성은 추가하지 않는 것이 더 깔끔하고 재사용성이 높습니다.

-----

### 1\. 헤더 파일 (`run_n_times_decorator.hpp`)

이 파일은 클래스의 선언을 담고 있습니다. ROS 2 패키지의 `include/amr_bt_nodes/` 디렉토리에 위치시키면 됩니다.

```cpp
// Copyright 2025 YourName <youremail@example.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "behaviortree_cpp/decorator_node.h"

namespace amr_bt_nodes
{

/**
 * @brief Executes the child node for a specified number of ticks each time a new goal/execution cycle begins.
 * After N ticks, this node will continuously return SUCCESS for the remainder of the
 * current execution cycle without ticking its child.
 * The internal tick count is reset to zero when the node is halted, making it ready
 * for the next execution cycle (e.g., a new navigation goal).
 */
class RunNTimesDecorator : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for amr_bt_nodes::RunNTimesDecorator
   * @param name The name of the node
   * @param config The node configuration
   */
  RunNTimesDecorator(
    const std::string & name,
    const BT::NodeConfiguration & config);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts();

  /**
   * @brief The halt method, called when the node is preempted.
   */
  void halt() override;

private:
  /**
   * @brief The main tick behaviour
   * @return BT::NodeStatus The status of the node
   */
  BT::NodeStatus tick() override;

  unsigned int tick_count_{0};
  bool completed_for_this_goal_{false};
  int num_ticks_to_run_{1};
};

}  // namespace amr_bt_nodes
```

-----

### 2\. 소스 파일 (`run_n_times_decorator.cpp`)

이 파일은 클래스의 실제 구현과 플러그인 등록 매크로를 포함합니다. ROS 2 패키지의 `src` 디렉토리에 위치시키면 됩니다.

```cpp
// Copyright 2025 YourName <youremail@example.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "amr_bt_nodes/run_n_times_decorator.hpp"

namespace amr_bt_nodes
{

RunNTimesDecorator::RunNTimesDecorator(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::DecoratorNode(name, config)
{
}

BT::PortsList RunNTimesDecorator::providedPorts()
{
  return {
    BT::InputPort<int>("num_ticks", 1, "Number of ticks to execute the child")
  };
}

void RunNTimesDecorator::halt()
{
  // Reset the state for the next goal/execution cycle.
  tick_count_ = 0;
  completed_for_this_goal_ = false;
  DecoratorNode::halt();
}

BT::NodeStatus RunNTimesDecorator::tick()
{
  // If the required number of ticks has already been completed for this goal,
  // do not execute the child and return SUCCESS immediately.
  if (completed_for_this_goal_) {
    return BT::NodeStatus::SUCCESS;
  }

  // On the first tick of a new execution cycle, read the 'num_ticks' parameter.
  if (tick_count_ == 0) {
    if (!getInput<int>("num_ticks", num_ticks_to_run_) || num_ticks_to_run_ < 1) {
      throw BT::RuntimeError(
              "[RunNTimesDecorator] Missing or invalid required input port [num_ticks]");
    }
  }

  tick_count_++;

  // Tick the child and propagate its status.
  setStatus(BT::NodeStatus::RUNNING);
  const auto child_status = child_node_->executeTick();

  // After ticking, check if we have now completed the required number of ticks.
  if (tick_count_ >= num_ticks_to_run_) {
    completed_for_this_goal_ = true;
  }

  return child_status;
}

}  // namespace amr_bt_nodes

// BehaviorTree.CPP 플러그인 등록을 위한 외부 함수
#include "behaviortree_cpp/bt_factory.h"
extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<amr_bt_nodes::RunNTimesDecorator>("RunNTimesDecorator");
}
```

-----

### 3\. 빌드 및 사용 방법

#### CMakeLists.txt 설정

이 코드를 빌드하여 공유 라이브러리(`.so` 파일)로 만들려면 `CMakeLists.txt`에 다음 내용을 추가해야 합니다.

```cmake
# ... find_package(ament_cmake REQUIRED) 등 기본 설정

# BehaviorTree.CPP 의존성 추가
find_package(behaviortree_cpp_v3 REQUIRED)

add_library(
  amr_bt_nodes_run_n_times_decorator_bt_plugin SHARED
  src/run_n_times_decorator.cpp)

target_include_directories(
  amr_bt_nodes_run_n_times_decorator_bt_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  amr_bt_nodes_run_n_times_decorator_bt_plugin PUBLIC
  "behaviortree_cpp_v3")

install(
  TARGETS amr_bt_nodes_run_n_times_decorator_bt_plugin
  DESTINATION lib/${PROJECT_NAME})

# ...
```

#### BT XML에서 사용

빌드된 플러그인을 로드한 후, XML 파일에서 다음과 같이 노드를 사용할 수 있습니다.

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <RunNTimesDecorator num_ticks="3">
        <PlaySound file="start_moving.wav"/>
      </RunNTimesDecorator>
      <FollowPath/>
    </Sequence>
  </BehaviorTree>
</root>
```

이렇게 하면 요청하신 모든 조건을 만족하는 완전한 BT 플러그인이 완성됩니다.
