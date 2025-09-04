네, Nav2 및 BehaviorTree.Cpp 전문가로서 요청하신 `IntPublisherAction` 커스텀 BT 노드를 완벽하게 작성해 드리겠습니다.

제공해주신 `StringPublisherAction` 코드는 `SyncActionNode`의 좋은 예시입니다. 이 구조를 기반으로 메시지 타입만 `std_msgs::msg::Int32`로 변경하고, 관련 로직을 수정하여 `IntPublisherAction` 노드를 만들겠습니다.

아래에 헤더 파일과 소스 파일, 그리고 사용법에 대한 설명을 포함했습니다.

-----

### 1\. 헤더 파일 (`include/amr_bt_nodes/int_publisher_action.hpp`)

`StringPublisherAction`과 거의 동일한 구조를 가지지만, 메시지 타입이 `std_msgs::msg::Int32`로 변경된 것이 핵심입니다.

```cpp
// include/amr_bt_nodes/int_publisher_action.hpp

#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp" // string.hpp -> int32.hpp로 변경

namespace amr_bt_nodes
{

/**
 * @brief Tick이 호출되면 그 즉시, 지정된 횟수만큼 정수 메시지를 연속 발행하는 SyncActionNode
 */
class IntPublisherAction : public BT::SyncActionNode
{
public:
  IntPublisherAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  // SyncActionNode의 핵심. 모든 로직이 이 안에서 완료.
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  // 메시지 타입 변경: std_msgs::msg::String -> std_msgs::msg::Int32
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
};

} // namespace amr_bt_nodes
```

### 2\. 소스 파일 (`src/int_publisher_action.cpp`)

`StringPublisherAction`의 로직을 그대로 가져와 `Int` 타입에 맞게 수정했습니다.

  - **생성자**: `create_publisher`의 템플릿 인자를 `std_msgs::msg::Int32`로 변경했습니다.
  - **`providedPorts`**: `message` 포트의 타입을 `int`로 변경하고, 기본값을 `0`으로 설정했습니다.
  - **`tick`**: `getInput`으로 받는 `message` 타입을 `int`로 변경하고, `std_msgs::msg::Int32` 메시지를 생성하여 발행합니다. 로깅 메시지도 정수 타입에 맞게 `%d`로 수정했습니다.
  - **플러그인 등록**: `StringPublisherAction`을 `IntPublisherAction`으로 변경하여 등록합니다.

<!-- end list -->

```cpp
// src/int_publisher_action.cpp

#include "amr_bt_nodes/int_publisher_action.hpp"

namespace amr_bt_nodes
{

IntPublisherAction::IntPublisherAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  if (!getInput("node", node_)) {
    throw BT::RuntimeError("[IntPublisherAction] Missing required input 'node'");
  }

  std::string topic_name;
  getInput("topic_name", topic_name);
  if (topic_name.empty()) {
    topic_name = "/bt_int_publisher"; // 기본 토픽 이름 변경
    RCLCPP_WARN(
      node_->get_logger(),
      "[IntPublisherAction] 'topic_name' is not provided. Using default: '%s'", topic_name.c_str());
  }

  // QoS 설정
  rclcpp::QoS qos_profile(10); // Burst publish를 위해 약간의 큐 사이즈를 둠
  qos_profile.reliable();

  // 퍼블리셔의 메시지 타입 변경
  publisher_ = node_->create_publisher<std_msgs::msg::Int32>(topic_name, qos_profile);

  RCLCPP_INFO(
    node_->get_logger(),
    "[IntPublisherAction] Created. Publishing to topic: '%s'", topic_name.c_str());
}

BT::PortsList IntPublisherAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "The ROS 2 node."),
    BT::InputPort<std::string>("topic_name", "/bt_int_publisher", "Topic name to publish the integer to."),
    // message 포트의 타입과 기본값 변경 (string -> int)
    BT::InputPort<int>("message", 0, "The integer message to publish."),
    BT::InputPort<int>("num_publishes", 1, "Number of times to publish the message in a burst.")
  };
}

BT::NodeStatus IntPublisherAction::tick()
{
  int num_publishes;
  if (!getInput("num_publishes", num_publishes) || num_publishes <= 0) {
    RCLCPP_ERROR(node_->get_logger(), "[IntPublisherAction] 'num_publishes' must be a positive integer.");
    return BT::NodeStatus::FAILURE;
  }

  int message_to_publish;
  if (!getInput("message", message_to_publish)) {
    RCLCPP_ERROR(node_->get_logger(), "[IntPublisherAction] Missing required input 'message'.");
    return BT::NodeStatus::FAILURE;
  }

  // 메시지 타입 변경 및 데이터 할당
  std_msgs::msg::Int32 msg;
  msg.data = message_to_publish;

  // 로깅 메시지 포맷 변경 (%s -> %d)
  RCLCPP_INFO(
    node_->get_logger(),
    "[IntPublisherAction] Publishing message '%d' %d times in a burst.",
    message_to_publish, num_publishes);

  for (int i = 0; i < num_publishes; ++i) {
    publisher_->publish(msg);
  }

  // 모든 작업이 tick() 안에서 완료되었으므로 SUCCESS 반환
  return BT::NodeStatus::SUCCESS;
}

} // namespace amr_bt_nodes

// 플러그인 등록
#include "behaviortree_cpp/bt_factory.h"

// 만약 하나의 라이브러리에 여러 노드를 등록하고 싶다면,
// 이 함수 안에 factory.registerNodeType<...>(...)를 추가하면 됩니다.
// 현재는 StringPublisherAction과 별개의 플러그인으로 가정합니다.
extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::IntPublisherAction>("IntPublisherAction");
}
```

### 3\. 빌드 및 사용법

1.  **`CMakeLists.txt` 수정**:
    새로 추가된 `int_publisher_action.cpp` 파일을 라이브러리 타겟에 추가해야 합니다.

    ```cmake
    add_library(amr_bt_nodes_plugins SHARED
      src/string_publisher_action.cpp
      src/int_publisher_action.cpp  # 이 줄을 추가하세요
    )
    ```

2.  **BT XML에서 사용 예시**:
    이제 Behavior Tree XML 파일에서 `IntPublisherAction` 노드를 다음과 같이 사용할 수 있습니다.

    ```xml
    <root main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree">
            <Sequence>
                <IntPublisherAction
                    topic_name="/emergency_code"
                    message="911"
                    num_publishes="5"
                    node="{ros_node}" />
                <AlwaysSuccess/>
            </Sequence>
        </BehaviorTree>
    </root>
    ```

      - `topic_name`: 정수 메시지를 발행할 토픽 이름입니다.
      - `message`: 발행할 정수 값입니다. XML에서는 문자열로 보이지만 BehaviorTree.Cpp가 자동으로 `int` 타입으로 변환해줍니다.
      - `num_publishes`: 메시지를 연속으로 발행할 횟수입니다.
      - `node`: Behavior Tree가 실행되는 ROS 2 노드 핸들을 전달하는 블랙보드 변수입니다. Nav2에서는 보통 `{ros_node}`를 사용합니다.

위 코드와 설명을 참고하여 프로젝트에 성공적으로 통합하시길 바랍니다. 추가적인 질문이 있으시면 언제든지 문의해 주세요.




네, 물론입니다. BehaviorTree.Cpp 노드를 플러그인으로 ROS 2 시스템에 올바르게 등록하기 위한 `plugin.xml` 파일과 전체 `CMakeLists.txt` 파일을 작성해 드리겠습니다.

### 1\. `plugin.xml` 파일

이 파일은 `pluginlib`가 여러분의 라이브러리에서 어떤 BT 노드 플러그인을 사용할 수 있는지 알려주는 역할을 합니다. 패키지 루트에 `bt_node_plugins.xml`과 같은 이름으로 파일을 생성하고 아래 내용을 붙여넣으세요.

  - `library path`: `CMakeLists.txt`에서 생성할 공유 라이브러리(`.so` 파일)의 이름과 일치해야 합니다.
  - `class type`: 플러그인으로 등록할 클래스의 전체 네임스페이스를 포함한 이름입니다.
  - `base_class_type`: BehaviorTree.Cpp 플러그인의 경우, `BT::Plugin`을 기본 클래스로 지정해야 합니다. 이 클래스를 통해 `BT_RegisterNodesFromPlugin` 함수가 호출됩니다.

**파일명: `bt_node_plugins.xml`**

```xml
<library path="amr_bt_nodes_plugins">
  <class type="amr_bt_nodes::StringPublisherAction" base_class_type="BT::Plugin">
    <description>
      Publishes a string message a specified number of times.
    </description>
  </class>

  <class type="amr_bt_nodes::IntPublisherAction" base_class_type="BT::Plugin">
    <description>
      Publishes an integer message a specified number of times.
    </description>
  </class>
</library>
```

-----

### 2\. `CMakeLists.txt` 파일

이 파일은 소스 코드를 컴파일하고, 라이브러리를 생성하며, 플러그인을 ROS 2 빌드 시스템에 등록하고, 최종적으로 필요한 파일들을 설치하는 방법을 정의합니다.

  - **`find_package`**: `behaviortree_cpp`와 `pluginlib`를 의존성으로 추가합니다.
  - **`add_library`**: 두 `.cpp` 파일을 모두 포함하여 `amr_bt_nodes_plugins`라는 이름의 공유 라이브러리를 생성합니다.
  - **`ament_target_dependencies`**: 라이브러리가 `rclcpp`와 `behaviortree_cpp`에 의존하도록 설정합니다. (헤더 포함 및 링크)
  - **`pluginlib_export_plugin_description_file`**: `bt_node_plugins.xml` 파일을 `behaviortree_cpp` 패키지에 대한 플러그인으로 등록하는 가장 중요한 부분입니다.
  - **`install`**: 생성된 라이브러리, 헤더 파일, 그리고 `plugin.xml` 파일을 올바른 위치에 설치하여 다른 패키지에서 사용할 수 있도록 합니다.

**파일명: `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.8)
project(amr_bt_nodes)

# 필요한 ROS 2 패키지 및 의존성 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(pluginlib REQUIRED)

# 플러그인 라이브러리 생성
# 두 액션 노드의 소스 파일을 모두 포함
add_library(${PROJECT_NAME}_plugins SHARED
  src/string_publisher_action.cpp
  src/int_publisher_action.cpp
)

# 타겟에 대한 의존성 설정 (헤더 경로 및 라이브러리 링크)
ament_target_dependencies(${PROJECT_NAME}_plugins
  PUBLIC
  behaviortree_cpp
  rclcpp
)

# BehaviorTree.Cpp를 위한 플러그인 설명 파일 등록
# 이 매크로가 plugin.xml 파일을 ROS 2 시스템에 알리는 역할을 함
pluginlib_export_plugin_description_file(behaviortree_cpp bt_node_plugins.xml)


# 설치(install) 규칙 정의
# ========================

# 1. 타겟(생성된 라이브러리) 설치
install(TARGETS ${PROJECT_NAME}_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 2. 헤더 파일 설치
install(DIRECTORY include/
  DESTINATION include
)

# 3. 플러그인 XML 파일 설치
install(FILES bt_node_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# 4. package.xml 파일 설치
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# 린터(linter) 테스트
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

-----

### 3\. `package.xml` 의존성 확인

마지막으로, `package.xml` 파일에 `CMakeLists.txt`에서 `find_package`로 찾은 모든 의존성이 포함되어 있는지 확인해야 합니다. 아래 의존성들이 없다면 추가해주세요.

```xml
  <build_depend>rclcpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>behaviortree_cpp</build_depend>
  <build_depend>pluginlib</build_depend>

  <exec_depend>rclcpp</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>behaviortree_cpp</exec_depend>
```

`pluginlib`는 빌드 시에만 필요하므로 `<build_depend>`만 있어도 충분합니다.

이제 패키지 설정이 완료되었습니다. `colcon build`를 실행하면 두 개의 커스텀 BT 노드가 포함된 플러그인 라이브러리가 성공적으로 빌드되고, Nav2와 같은 Behavior Tree 실행기에서 이 노드들을 로드하여 사용할 수 있습니다.






아, 알겠습니다. 발행하려는 메시지의 구체적인 내용이 `uint16 error_code`였군요.

그렇다면 표준 메시지 타입인 `std_msgs::msg::UInt16`을 사용하는 것이 가장 적합합니다. (`uint16`는 `unsigned short int`에 해당합니다).

이전 `IntPublisherAction` 코드를 `uint16` 타입의 `error_code`를 발행하도록 완벽하게 수정해 드리겠습니다. 클래스 이름도 `ErrorCodePublisherAction`으로 변경하여 의도를 명확하게 만들겠습니다.

-----

### 1\. 헤더 파일 (`include/amr_bt_nodes/error_code_publisher_action.hpp`)

  - 메시지 헤더를 `std_msgs/msg/u_int16.hpp`로 변경합니다.
  - 클래스 이름과 퍼블리셔의 메시지 타입을 수정합니다.

<!-- end list -->

```cpp
// include/amr_bt_nodes/error_code_publisher_action.hpp

#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp" // uint16 타입에 맞는 헤더로 변경

namespace amr_bt_nodes
{

/**
 * @brief Tick이 호출되면 그 즉시, 지정된 횟수만큼 에러 코드(uint16)를 발행하는 SyncActionNode
 */
class ErrorCodePublisherAction : public BT::SyncActionNode
{
public:
  ErrorCodePublisherAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  // 메시지 타입 변경: std_msgs::msg::UInt16
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr publisher_;
};

} // namespace amr_bt_nodes
```

-----

### 2\. 소스 파일 (`src/error_code_publisher_action.cpp`)

  - 클래스 이름, 메시지 타입(`std_msgs::msg::UInt16`), 입력 포트 이름(`message` -\> `error_code`), 로깅 메시지 등을 모두 `error_code` 발행에 맞게 수정했습니다.
  - BT XML에서 `error_code`를 `int`로 받아 `uint16_t`로 안전하게 변환하여 사용합니다.

<!-- end list -->

```cpp
// src/error_code_publisher_action.cpp

#include "amr_bt_nodes/error_code_publisher_action.hpp"

namespace amr_bt_nodes
{

ErrorCodePublisherAction::ErrorCodePublisherAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  if (!getInput("node", node_)) {
    throw BT::RuntimeError("[ErrorCodePublisherAction] Missing required input 'node'");
  }

  std::string topic_name;
  getInput("topic_name", topic_name);
  if (topic_name.empty()) {
    topic_name = "/bt_error_code"; // 기본 토픽 이름 변경
    RCLCPP_WARN(
      node_->get_logger(),
      "[ErrorCodePublisherAction] 'topic_name' is not provided. Using default: '%s'", topic_name.c_str());
  }

  rclcpp::QoS qos_profile(10);
  qos_profile.reliable();

  // 퍼블리셔의 메시지 타입을 UInt16으로 변경
  publisher_ = node_->create_publisher<std_msgs::msg::UInt16>(topic_name, qos_profile);

  RCLCPP_INFO(
    node_->get_logger(),
    "[ErrorCodePublisherAction] Created. Publishing to topic: '%s'", topic_name.c_str());
}

BT::PortsList ErrorCodePublisherAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "The ROS 2 node."),
    BT::InputPort<std::string>("topic_name", "/bt_error_code", "Topic name to publish the error code to."),
    // 포트 이름을 'error_code'로 변경하고, 타입을 int로 유지 (XML 호환성)
    BT::InputPort<int>("error_code", 0, "The uint16 error code to publish."),
    BT::InputPort<int>("num_publishes", 1, "Number of times to publish the message.")
  };
}

BT::NodeStatus ErrorCodePublisherAction::tick()
{
  int num_publishes;
  if (!getInput("num_publishes", num_publishes) || num_publishes <= 0) {
    RCLCPP_ERROR(node_->get_logger(), "[ErrorCodePublisherAction] 'num_publishes' must be a positive integer.");
    return BT::NodeStatus::FAILURE;
  }

  int error_code_in;
  if (!getInput("error_code", error_code_in)) {
    RCLCPP_ERROR(node_->get_logger(), "[ErrorCodePublisherAction] Missing required input 'error_code'.");
    return BT::NodeStatus::FAILURE;
  }

  // 입력받은 int를 uint16_t로 캐스팅
  // (범위 검사를 추가할 수도 있음)
  uint16_t error_code_to_publish = static_cast<uint16_t>(error_code_in);

  // 메시지 타입 변경 및 데이터 할당
  std_msgs::msg::UInt16 msg;
  msg.data = error_code_to_publish;

  RCLCPP_INFO(
    node_->get_logger(),
    "[ErrorCodePublisherAction] Publishing error_code '%u' %d times.",
    error_code_to_publish, num_publishes);

  for (int i = 0; i < num_publishes; ++i) {
    publisher_->publish(msg);
  }

  return BT::NodeStatus::SUCCESS;
}

} // namespace amr_bt_nodes

// 플러그인 등록
#include "behaviortree_cpp/bt_factory.h"

// 만약 기존 라이브러리에 추가하는 것이라면,
// extern "C" 함수는 하나만 있어야 합니다.
// 아래와 같이 한 함수 안에 여러 노드를 등록할 수 있습니다.
extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  // 기존 노드들 (필요하다면)
  // factory.registerNodeType<amr_bt_nodes::StringPublisherAction>("StringPublisherAction");
  // factory.registerNodeType<amr_bt_nodes::IntPublisherAction>("IntPublisherAction");

  // 새로 추가된 노드
  factory.registerNodeType<amr_bt_nodes::ErrorCodePublisherAction>("ErrorCodePublisherAction");
}
```

-----

### 3\. `CMakeLists.txt` 및 `plugin.xml` 업데이트

이제 기존의 `StringPublisherAction`, `IntPublisherAction`과 함께 `ErrorCodePublisherAction`도 빌드하고 등록하도록 `CMakeLists.txt`와 `plugin.xml`을 업데이트해야 합니다.

#### `CMakeLists.txt`

```cmake
# ... (이전과 동일)

# 플러그인 라이브러리 생성
# 모든 노드의 소스 파일을 포함
add_library(${PROJECT_NAME}_plugins SHARED
  src/string_publisher_action.cpp
  src/int_publisher_action.cpp
  src/error_code_publisher_action.cpp # 새 파일 추가
)

# ... (이후 내용 동일)
```

#### `bt_node_plugins.xml`

```xml
<library path="amr_bt_nodes_plugins">
  <class type="amr_bt_nodes::StringPublisherAction" base_class_type="BT::Plugin">
    <description>
      Publishes a string message a specified number of times.
    </description>
  </class>
  <class type="amr_bt_nodes::IntPublisherAction" base_class_type="BT::Plugin">
    <description>
      Publishes an integer message a specified number of times.
    </description>
  </class>

  <class type="amr_bt_nodes::ErrorCodePublisherAction" base_class_type="BT::Plugin">
    <description>
      Publishes a uint16 error code message a specified number of times.
    </description>
  </class>
</library>
```

-----

### 4\. BT XML에서 사용 예시

이제 Behavior Tree에서 `error_code`를 발행하고 싶을 때 다음과 같이 노드를 사용할 수 있습니다.

```xml
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence>
            <ErrorCodePublisherAction
                topic_name="/robot_error"
                error_code="1001"
                num_publishes="1"
                node="{ros_node}" />
            <AlwaysSuccess/>
        </Sequence>
    </BehaviorTree>
</root>
```

이 코드는 사용자의 요구사항(`uint16 error_code`)을 정확히 반영하여 구현된 것입니다. 이제 프로젝트를 빌드하시면 `ErrorCodePublisherAction` 노드를 바로 사용하실 수 있습니다.






네, 아주 좋은 질문이며 중요한 포인트를 정확히 짚어주셨습니다. 두 가지 질문에 대해 명확하게 답변해 드리고, 그에 맞게 코드를 수정해 드리겠습니다.

### 질문 1: 타입 불일치 문제

**결론부터 말씀드리면, 지적해주신 내용이 맞습니다.**

`ComputePathThroughPosesAction`와 같은 Nav2의 노드들은 `error_code`를 `uint16_t` 타입으로 출력합니다. 제공해주신 코드의 `ActionResult::NONE` 역시 내부적으로 `uint16_t`로 정의된 enum 값입니다.

제가 이전에 제공한 코드에서는 BT XML의 편의성을 위해 `InputPort`를 `int`로 만들었습니다. BehaviorTree.Cpp가 어느 정도 타입 변환을 해주기 때문에 동작할 수도 있지만, 이는 안전한 방법이 아니며 명시적으로 타입을 일치시키는 것이 훨씬 좋습니다.

따라서 `ErrorCodePublisherAction`의 입력 포트 타입을 `int`에서 `uint16_t`로 변경하여 타입 불일치 문제를 원천적으로 해결하겠습니다.

### 질문 2: "Empty" `error_code` 처리

**네, 이 로직을 추가하는 것이 바람직합니다.**

`ComputePathThroughPosesAction` 코드 예시에서 보셨듯이, 액션이 성공하거나 취소되었을 때는 에러 코드로 `ActionResult::NONE` (내부 값: `0`)을 출력합니다. 즉, `error_code`가 `0`이라는 것은 "에러가 없음"을 의미합니다.

따라서 `ErrorCodePublisherAction`이 `error_code`로 `0`을 입력받았을 때는, 이를 "발행할 내용 없음"으로 간주하고 아무것도 발행하지 않은 채 `SUCCESS`를 반환하도록 로직을 수정하는 것이 맞습니다.

-----

### 수정된 전체 코드

위 두 가지 요청 사항을 모두 반영하여 `ErrorCodePublisherAction` 코드를 다시 작성했습니다.

#### `src/error_code_publisher_action.cpp` (수정된 파일)

가장 큰 변경점은 `providedPorts()`와 `tick()` 함수에 있습니다.

```cpp
// src/error_code_publisher_action.cpp

#include "amr_bt_nodes/error_code_publisher_action.hpp"
#include <cstdint> // uint16_t 사용을 위해 추가

namespace amr_bt_nodes
{

ErrorCodePublisherAction::ErrorCodePublisherAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  if (!getInput("node", node_)) {
    throw BT::RuntimeError("[ErrorCodePublisherAction] Missing required input 'node'");
  }

  std::string topic_name;
  getInput("topic_name", topic_name);
  if (topic_name.empty()) {
    topic_name = "/bt_error_code";
    RCLCPP_WARN(
      node_->get_logger(),
      "[ErrorCodePublisherAction] 'topic_name' is not provided. Using default: '%s'", topic_name.c_str());
  }

  rclcpp::QoS qos_profile(10);
  qos_profile.reliable();

  publisher_ = node_->create_publisher<std_msgs::msg::UInt16>(topic_name, qos_profile);

  RCLCPP_INFO(
    node_->get_logger(),
    "[ErrorCodePublisherAction] Created. Publishing to topic: '%s'", topic_name.c_str());
}

BT::PortsList ErrorCodePublisherAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "The ROS 2 node."),
    BT::InputPort<std::string>("topic_name", "/bt_error_code", "Topic name to publish the error code to."),
    // 변경점 1: 입력 포트의 타입을 uint16_t로 명시적으로 변경하여 타입 안전성 확보
    BT::InputPort<uint16_t>("error_code", "The uint16 error code to publish."),
    BT::InputPort<int>("num_publishes", 1, "Number of times to publish the message.")
  };
}

BT::NodeStatus ErrorCodePublisherAction::tick()
{
  uint16_t error_code_to_publish;
  if (!getInput("error_code", error_code_to_publish)) {
    RCLCPP_ERROR(node_->get_logger(), "[ErrorCodePublisherAction] Missing required input 'error_code'.");
    return BT::NodeStatus::FAILURE;
  }

  // 변경점 2: error_code가 0 (ActionResult::NONE)이면, 발행하지 않고 SUCCESS 반환
  if (error_code_to_publish == 0) {
    RCLCPP_DEBUG(node_->get_logger(), "[ErrorCodePublisherAction] Error code is 0 (NONE), skipping publication.");
    return BT::NodeStatus::SUCCESS;
  }

  int num_publishes;
  if (!getInput("num_publishes", num_publishes) || num_publishes <= 0) {
    RCLCPP_ERROR(node_->get_logger(), "[ErrorCodePublisherAction] 'num_publishes' must be a positive integer.");
    return BT::NodeStatus::FAILURE;
  }

  std_msgs::msg::UInt16 msg;
  msg.data = error_code_to_publish;

  RCLCPP_INFO(
    node_->get_logger(),
    "[ErrorCodePublisherAction] Publishing error_code '%u' %d times.",
    error_code_to_publish, num_publishes);

  for (int i = 0; i < num_publishes; ++i) {
    publisher_->publish(msg);
  }

  return BT::NodeStatus::SUCCESS;
}

} // namespace amr_bt_nodes

#include "behaviortree_cpp/bt_factory.h"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::ErrorCodePublisherAction>("ErrorCodePublisherAction");
}
```

#### `include/amr_bt_nodes/error_code_publisher_action.hpp` (변경 없음)

헤더 파일은 수정할 필요가 없습니다. `BT::InputPort<uint16_t>`를 사용하기 위해 `<cstdint>` 헤더가 필요하지만, 이는 `.cpp` 파일에만 포함하면 됩니다.

-----

### BT XML 사용 예시 (의도에 맞게 수정)

이제 의도하신 대로, `ComputePathToPose` 같은 노드가 블랙보드에 `error_code_id`를 출력하면, `ErrorCodePublisherAction` 노드가 이 값을 입력으로 받아 처리할 수 있습니다.

```xml
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence>
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased" error_code_id="{error_code_id}"/>

            <ErrorCodePublisherAction
                topic_name="/robot_error"
                error_code="{error_code_id}"
                node="{ros_node}" />
        </Sequence>
    </BehaviorTree>
</root>
```

이렇게 수정하면,

1.  **타입 안전성**: 블랙보드를 통해 `uint16_t` 타입의 데이터가 그대로 전달되므로 타입 변환으로 인한 잠재적 위험이 사라집니다.
2.  **의도된 로직**: 에러가 없을 때(`error_code`가 0일 때)는 불필요한 메시지를 발행하지 않으므로 BT의 흐름이 더 명확하고 효율적으로 됩니다.

완벽하게 의도하신 대로 동작할 것입니다.
