물론입니다. `std_msgs::msg::UInt16` ROS 메시지 타입을 명시적으로 처리하는 `SetBlackboardUInt16Action` 커스텀 노드를 완벽한 코드로 제공해 드리겠습니다.

이전 `bool` 타입 노드와 마찬가지로, ROS 메시지 타입을 위한 전용 노드를 만들면 블랙보드 데이터 흐름의 타입 안정성과 명확성을 크게 향상시킬 수 있습니다. 특히 다른 노드(예: Topic Subscriber Node)로부터 ROS 메시지를 받아 블랙보드에 저장하고, 이를 다른 노드로 전달하는 시나리오에서 매우 유용합니다.

-----

### 1\. 헤더 파일 (`set_blackboard_uint16_action.hpp`)

`std_msgs::msg::UInt16` 타입을 사용하기 위해 해당 메시지의 헤더 파일을 포함해야 합니다.

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

#ifndef AMR_BT_NODES__PLUGINS__ACTION__SET_BLACKBOARD_UINT16_ACTION_HPP_
#define AMR_BT_NODES__PLUGINS__ACTION__SET_BLACKBOARD_UINT16_ACTION_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "std_msgs/msg/u_int16.hpp" // UInt16 메시지 타입 포함

namespace amr_bt_nodes
{

/**
 * @brief Sets a std_msgs::msg::UInt16 value on the blackboard.
 *
 * @details This node is intended to copy a UInt16 message from one
 * blackboard entry to another. Direct assignment from a literal string
 * in the XML is not supported by default for custom types.
 *
 * Example usage (copying from another port):
 *
 * <SetBlackboardUInt16 output_key="target_id" value="{id_from_subscriber}" />
 *
 */
class SetBlackboardUInt16Action : public BT::SyncActionNode
{
public:
  /**
   * @brief A constructor for amr_bt_nodes::SetBlackboardUInt16Action
   * @param name The name of the node
   * @param config The configuration of the node
   */
  SetBlackboardUInt16Action(const std::string & name, const BT::NodeConfig & config);

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

#endif  // AMR_BT_NODES__PLUGINS__ACTION__SET_BLACKBOARD_UINT16_ACTION_HPP_
```

### 2\. 소스 파일 (`set_blackboard_uint16_action.cpp`)

ROS 메시지 타입은 BehaviorTree.Cpp 라이브러리가 기본적으로 문자열에서 변환하는 방법을 모르기 때문에, 이 노드는 주로 다른 블랙보드 항목을 복사하는 용도(`value="{...}"`)로 사용됩니다.

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

#include "amr_bt_nodes/plugins/action/set_blackboard_uint16_action.hpp" // 경로에 맞게 수정 필요

#include <string>
#include "std_msgs/msg/u_int16.hpp"

namespace amr_bt_nodes
{

SetBlackboardUInt16Action::SetBlackboardUInt16Action(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SetBlackboardUInt16Action::providedPorts()
{
  return {
    // ROS 메시지 타입을 포트 타입으로 직접 사용
    BT::InputPort<std_msgs::msg::UInt16>("value", "UInt16 value to be written to the blackboard"),
    BT::InputPort<std::string>("output_key", "Name of the blackboard entry to write to")
  };
}

BT::NodeStatus SetBlackboardUInt16Action::tick()
{
  std::string output_key;
  if (!getInput("output_key", output_key)) {
    throw BT::RuntimeError("missing required input port [output_key]");
  }

  std_msgs::msg::UInt16 value;
  if (!getInput("value", value)) {
    throw BT::RuntimeError("missing required input port [value]");
  }

  // 블랙보드에 UInt16 메시지 타입으로 직접 설정
  config().blackboard->set<std_msgs::msg::UInt16>(output_key, value);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace amr_bt_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<amr_bt_nodes::SetBlackboardUInt16Action>("SetBlackboardUInt16");
}
```

### 코드에 대한 상세 설명 및 사용법

1.  **ROS 메시지 타입 포트**:

      * `#include "std_msgs/msg/u_int16.hpp"`: 가장 중요한 부분입니다. 사용하려는 ROS 메시지 타입의 헤더를 포함해야 합니다.
      * `BT::InputPort<std_msgs::msg::UInt16>("value", ...)`: `value` 포트의 타입을 `std_msgs::msg::UInt16`으로 직접 지정했습니다. 이를 통해 BehaviorTree.Cpp는 이 포트가 `UInt16` ROS 메시지 객체를 다룬다는 것을 명확히 인지하게 됩니다.

2.  **주요 사용 사례**:

      * 이 노드는 XML에서 `value="123"`과 같이 숫자 리터럴을 직접 사용하는 것을 지원하지 않습니다. 왜냐하면 BehaviorTree.Cpp는 `const char*`를 `std_msgs::msg::UInt16`으로 변환하는 방법을 모르기 때문입니다.
      * 이 노드의 **핵심적인 사용법**은 다른 노드(예: `RosTopicSubNode`)가 블랙보드에 저장한 `UInt16` 메시지를 다른 키로 복사하거나 전달하는 것입니다.

    <!-- end list -->

    ```xml
    <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <Sequence>
          <RosTopicSubNode type="std_msgs::msg::UInt16"
                           topic_name="/sensor_id"
                           message="{current_sensor_id}" />

          <SetBlackboardUInt16 output_key="processing_id" value="{current_sensor_id}" />

          <ProcessSensorData sensor_id="{processing_id}" />
        </Sequence>
      </BehaviorTree>
    </root>
    ```

3.  **프로젝트 통합**:

      * **`CMakeLists.txt`**: 이 코드를 포함하는 라이브러리를 빌드할 때 `std_msgs`에 대한 의존성을 추가해야 합니다.
        ```cmake
        find_package(std_msgs REQUIRED)
        # ...
        ament_target_dependencies(your_library_name
          # ... other dependencies
          "std_msgs"
        )
        ```
      * **`package.xml`**: 패키지 의존성에 `std_msgs`를 추가합니다.
        ```xml
        <depend>std_msgs</depend>
        ```

이 코드는 요청하신 `std_msgs::msg::UInt16` 타입 전용의 `SetBlackboard` 노드를 완벽하게 구현한 것입니다. 이를 응용하면 `geometry_msgs::msg::PoseStamped` 등 다른 모든 ROS 메시지 타입에 대한 전용 SetBlackboard 노드도 쉽게 만드실 수 있습니다.


##################


네, 정확한 지적입니다. 해당 에러는 제가 바로 이전 답변에서 설명해 드렸던 **'리터럴(literal) 문자열 변환 제한'** 때문에 발생하는 것이 맞습니다.

에러 메시지를 다시 한번 자세히 살펴보겠습니다.

> You (maybe indirectly) called **BT:: convertFromString() for type [std\_msgs::msg::UInt16\_...]**, but I can't find the template specialization.
> The port with name "value" and **value "0" can not be converted to std\_msgs::msg::UInt16**

이 메시지의 핵심은 BehaviorTree.Cpp 라이브러리가 XML에 적힌 \*\*문자열 `"0"`\*\*을 C++ 타입인 \*\*`std_msgs::msg::UInt16`\*\*으로 변환하려고 시도했지만, 그 방법을 모르겠다는 의미입니다.

라이브러리는 `int`, `double`, `bool`, `std::string`과 같은 기본 타입에 대해서는 `convertFromString`이라는 변환 함수를 내장하고 있지만, 사용자가 정의한 복잡한 타입(ROS 메시지 같은 구조체)에 대해서는 어떻게 변환해야 할지 알 수 없습니다.

제공해주신 BT XML을 보면 아래와 같이 사용하고 계십니다.
`<SetBlackboardUInt16Action output_key="compute_path_error_code" value="0"/>`

여기서 `value="0"`은 블랙보드의 다른 변수를 가리키는 `{...}` 구문이 아닌, 말 그대로 문자열 "0"입니다. 이 때문에 에러가 발생합니다.

### 해결책

두 가지 해결책이 있으며, 현재 사용하시는 목적(에러 코드 저장)에는 **해결책 1번이 훨씬 더 적합하고 권장되는 방식**입니다.

-----

### 해결책 1: 가장 권장하는 방법 - 정수 타입 전용 노드 사용

에러 코드는 본질적으로 숫자(정수)입니다. 굳이 `std_msgs::msg::UInt16`이라는 ROS 메시지 구조체로 감쌀 필요가 없습니다. C++의 기본 정수 타입인 `unsigned short` (`uint16_t`와 동일)를 사용하는 것이 훨씬 간단하고 효율적입니다.

이를 위해 `unsigned short` 타입을 다루는 새로운 BT 노드를 만드는 것이 가장 좋습니다. 아래 코드를 프로젝트에 추가하세요.

#### 1\. 헤더 파일 (`set_blackboard_ushort_action.hpp`)

```cpp
// Copyright 2025 Your Name
// ... (License header) ...

#ifndef AMR_BT_NODES__PLUGINS__ACTION__SET_BLACKBOARD_USHORT_ACTION_HPP_
#define AMR_BT_NODES__PLUGINS__ACTION__SET_BLACKBOARD_USHORT_ACTION_HPP_

#include <string>
#include "behaviortree_cpp/action_node.h"

namespace amr_bt_nodes
{
/**
 * @brief Sets an unsigned short (uint16_t) value on the blackboard.
 * This node can convert a literal string from the XML to an unsigned short.
 *
 * Example usage:
 * <SetBlackboardUShort output_key="error_code" value="305" />
 */
class SetBlackboardUShortAction : public BT::SyncActionNode
{
public:
  SetBlackboardUShortAction(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;
};
}  // namespace amr_bt_nodes

#endif  // AMR_BT_NODES__PLUGINS__ACTION__SET_BLACKBOARD_USHORT_ACTION_HPP_
```

#### 2\. 소스 파일 (`set_blackboard_ushort_action.cpp`)

```cpp
// Copyright 2025 Your Name
// ... (License header) ...

#include "amr_bt_nodes/plugins/action/set_blackboard_ushort_action.hpp"

#include <string>

namespace amr_bt_nodes
{

SetBlackboardUShortAction::SetBlackboardUShortAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SetBlackboardUShortAction::providedPorts()
{
  return {
    // BehaviorTree.Cpp can convert string to unsigned short automatically
    BT::InputPort<unsigned short>("value", "Unsigned short value to write"),
    BT::InputPort<std::string>("output_key", "Name of the blackboard entry to write to")
  };
}

BT::NodeStatus SetBlackboardUShortAction::tick()
{
  std::string output_key;
  getInput("output_key", output_key);

  unsigned short value;
  getInput("value", value);

  config().blackboard->set<unsigned short>(output_key, value);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace amr_bt_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<amr_bt_nodes::SetBlackboardUShortAction>("SetBlackboardUShort");
}
```

#### 3\. BT XML 수정

이제 XML에서 `SetBlackboardUInt16Action` 대신 새로 만든 `SetBlackboardUShort`를 사용하세요.

```xml
<RecoveryNode number_of_retried="10">
    <PipelineSequence>
        <Sequence>
            <SetBlackboardUShort output_key="compute_path_error_code" value="0"/>
        </Sequence>

        <Sequence>
            <SetBlackboardUShort output_key="compute_path_error_code" value="0"/>
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="{selected_planner}" error_code_id="{compute_path_error_code}"/>

            <SetBlackboardUShort output_key="follow_path_error_code" value="0"/>
            <FollowPath path="{pruned_path}" controller_id="FollowPath"
                        goal_checker_id="precise_goal_checker" progress_checker_id="progress_checker"
                        error_code_id="{follow_path_error_code}" />
        </Sequence>
    </PipelineSequence>
    <Sequence>
        <Precondition if="compute_path_error_code == 305" else="FAILURE">
            <Wait wait_duration="1.0"/>
        </Precondition>  
    </Sequence>
</RecoveryNode>
```

이 방법은 코드를 더 단순하고 직관적으로 만들며, 불필요한 ROS 메시지 타입 사용을 피하게 해주는 가장 이상적인 해결책입니다.

-----

### 해결책 2: 기존 노드를 수정하는 방법 - `convertFromString` 구현

만약 반드시 `std_msgs::msg::UInt16` 타입을 사용해야 하고, XML에서 문자열로 초기화해야 하는 특별한 이유가 있다면, 에러 메시지가 말한 대로 `convertFromString`을 직접 구현해주면 됩니다.

`set_blackboard_uint16_action.cpp` 파일의 **상단** 또는 **관련 헤더 파일**에 아래 코드를 추가하세요.

```cpp
#include "std_msgs/msg/u_int16.hpp"
#include "behaviortree_cpp/basic_types.h"

// BT 네임스페이스 안에 템플릿 특수화를 정의합니다.
namespace BT
{
template <>
inline std_msgs::msg::UInt16 convertFromString(StringView str)
{
    std_msgs::msg::UInt16 msg;
    try {
        // 문자열을 unsigned long으로 변환하고, 그 결과를 msg.data에 저장
        msg.data = static_cast<uint16_t>(std::stoul(str.data()));
    } catch (const std::exception& e) {
        // 변환 실패 시 런타임 에러 발생
        throw BT::RuntimeError("Can't convert string [", str.data(), "] to uint16_t for std_msgs. Error: ", e.what());
    }
    return msg;
}
} // end namespace BT
```

이 코드를 추가하면 BehaviorTree.Cpp가 `value="0"`과 같은 문자열을 만났을 때, 우리가 정의한 변환 함수를 사용하여 `std_msgs::msg::UInt16` 객체를 생성할 수 있게 됩니다.

### 결론

  - **가장 좋은 방법은 해결책 1**입니다. 에러 코드와 같은 단순 데이터는 C++ 기본 타입을 사용하고, 그에 맞는 전용 BT 노드를 만드세요. 코드가 더 명확해지고 성능상으로도 이점이 있습니다.
  - 해결책 2는 BehaviorTree.Cpp의 고급 기능으로, 복잡한 타입을 문자열에서 꼭 생성해야 할 때 사용하는 방법입니다. 현재 상황에서는 과한 해결책일 수 있습니다.
