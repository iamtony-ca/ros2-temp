알겠습니다. 요청하신 대로 클래스 이름을 `SetBlackboardBoolAction`으로, 네임스페이스를 `amr_bt_nodes`로 변경하여 코드를 다시 작성해 드리겠습니다.

이렇게 네임스페이스를 별도로 관리하면, Nav2의 기본 노드들과 명확히 구분되어 프로젝트의 모듈성과 재사용성을 높일 수 있는 좋은 구조입니다.

-----

### 1\. 헤더 파일 (`set_blackboard_bool_action.hpp`)

```cpp
// Copyright 2025 Your Name
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

#ifndef AMR_BT_NODES__PLUGINS__ACTION__SET_BLACKBOARD_BOOL_ACTION_HPP_
#define AMR_BT_NODES__PLUGINS__ACTION__SET_BLACKBOARD_BOOL_ACTION_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"

namespace amr_bt_nodes
{

/**
 * @brief The SetBlackboardBoolAction is an action that sets a boolean value
 * on the blackboard. It is more type-safe than the generic SetBlackboard
 * for boolean types.
 *
 * It can take a value directly or from another blackboard entry.
 *
 * Example usage:
 *
 * <SetBlackboardBool output_key="is_battery_low" value="true" />
 *
 * or copying from another port:
 *
 * <SetBlackboardBool output_key="task_completed" value="{is_done}" />
 *
 */
class SetBlackboardBoolAction : public BT::SyncActionNode
{
public:
  /**
   * @brief A constructor for amr_bt_nodes::SetBlackboardBoolAction
   * @param name The name of the node
   * @param config The configuration of the node
   */
  SetBlackboardBoolAction(const std::string & name, const BT::NodeConfig & config);

  /**
   * @brief Provides the ports of the node
   * @return PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts();

private:
  /**
   * @brief The main execution routine.
   * @return NodeStatus SUCCESS or FAILURE
   */
  BT::NodeStatus tick() override;
};

}  // namespace amr_bt_nodes

#endif  // AMR_BT_NODES__PLUGINS__ACTION__SET_BLACKBOARD_BOOL_ACTION_HPP_
```

### 2\. 소스 파일 (`set_blackboard_bool_action.cpp`)

```cpp
// Copyright 2025 Your Name
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

#include "amr_bt_nodes/plugins/action/set_blackboard_bool_action.hpp" // 경로에 맞게 수정 필요

#include <string>

namespace amr_bt_nodes
{

SetBlackboardBoolAction::SetBlackboardBoolAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SetBlackboardBoolAction::providedPorts()
{
  return {
    BT::InputPort<bool>("value", "Boolean value to be written to the blackboard"),
    BT::InputPort<std::string>("output_key", "Name of the blackboard entry to write to")
  };
}

BT::NodeStatus SetBlackboardBoolAction::tick()
{
  std::string output_key;
  if (!getInput("output_key", output_key)) {
    throw BT::RuntimeError("missing required input port [output_key]");
  }

  bool value;
  if (!getInput("value", value)) {
    throw BT::RuntimeError("missing required input port [value]");
  }

  config().blackboard->set<bool>(output_key, value);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace amr_bt_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  // XML에서 사용할 태그 이름은 간결하게 "SetBlackboardBool"을 그대로 유지하는 것을 추천합니다.
  factory.registerNodeType<amr_bt_nodes::SetBlackboardBoolAction>("SetBlackboardBool");
}
```

### 변경 사항 요약

1.  **네임스페이스 변경**: `nav2_behavior_tree`에서 `amr_bt_nodes`로 변경되었습니다.
2.  **클래스 이름 변경**: `SetBlackboardBoolNode`에서 `SetBlackboardBoolAction`으로 변경되었습니다.
3.  **헤더 가드 및 `#include` 경로**: 네임스페이스와 클래스 이름 변경에 맞춰 수정되었습니다. 실제 프로젝트의 디렉토리 구조에 따라 `#include` 경로는 조정해야 할 수 있습니다.
4.  **노드 등록**: `BT_REGISTER_NODES` 매크로 내에서 새로운 클래스 이름(`amr_bt_nodes::SetBlackboardBoolAction`)을 사용하도록 업데이트했습니다. XML에서 사용될 태그 이름은 가독성을 위해 간결한 `"SetBlackboardBool"`을 그대로 유지했습니다.

이제 이 파일들을 `amr_bt_nodes`라는 ROS 2 패키지에 추가하고 빌드하여 Nav2 BT Navigator의 플러그인으로 로드하면 됩니다.
