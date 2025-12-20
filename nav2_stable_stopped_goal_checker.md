네, 요청하신 기능을 완벽하게 구현한 `StableStoppedGoalChecker` 플러그인 코드를 작성해 드립니다.

### 💡 설계 핵심

1. **X/Y 분리:** `SimpleGoalChecker`의 유클리드 거리() 방식 대신, 축 오차와 축 오차를 각각 절대값(`fabs`)으로 비교하도록 변경했습니다.
2. **시간(Duration) 조건:** 목표 범위(Pose Tolerance) 안에 들어왔다고 바로 성공으로 치지 않고, 설정된 `stability_duration` 시간 동안 **지속적으로** 범위 안에 머물러야만 다음 단계(속도 체크)로 넘어갑니다.
3. **속도(Velocity) 조건:** 위치와 시간이 모두 만족되면, 마지막으로 `StoppedGoalChecker`처럼 현재 속도가 0에 가까운지 확인합니다.

---

### 1. Header File (`stable_stopped_goal_checker.hpp`)

```cpp
/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024, Custom Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NAV2_CONTROLLER__PLUGINS__STABLE_STOPPED_GOAL_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__STABLE_STOPPED_GOAL_CHECKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace nav2_controller
{

/**
 * @class StableStoppedGoalChecker
 * @brief Checks if goal is reached with separate X/Y tolerances, stability duration, and zero velocity.
 */
class StableStoppedGoalChecker : public nav2_core::GoalChecker
{
public:
  StableStoppedGoalChecker();
  
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
    
  void reset() override;
  
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;
    
  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;

protected:
  // Tolerance parameters
  double x_goal_tolerance_;
  double y_goal_tolerance_;
  double yaw_goal_tolerance_;
  
  // Velocity parameters (from StoppedGoalChecker)
  double rot_stopped_velocity_;
  double trans_stopped_velocity_;

  // Time stability parameters
  double stability_duration_;

  // State variables for time checking
  bool in_pose_tolerance_;
  rclcpp::Time first_tolerance_time_;
  rclcpp::Clock::SharedPtr clock_;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::string plugin_name_;

  /**
   * @brief Callback executed when a parameter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__STABLE_STOPPED_GOAL_CHECKER_HPP_

```

---

### 2. Source File (`stable_stopped_goal_checker.cpp`)

```cpp
/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024, Custom Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cmath>
#include <string>
#include <memory>
#include <limits>
#include <vector>

#include "nav2_controller/plugins/stable_stopped_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "angles/angles.h"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

using std::hypot;
using std::fabs;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{

StableStoppedGoalChecker::StableStoppedGoalChecker()
: x_goal_tolerance_(0.25),
  y_goal_tolerance_(0.25),
  yaw_goal_tolerance_(0.25),
  rot_stopped_velocity_(0.25),
  trans_stopped_velocity_(0.25),
  stability_duration_(0.0),
  in_pose_tolerance_(false)
{
}

void StableStoppedGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();
  clock_ = node->get_clock();

  // Declare parameters
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".x_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".y_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".trans_stopped_velocity", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".rot_stopped_velocity", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".stability_duration", rclcpp::ParameterValue(0.5)); // Default 0.5s

  // Get parameters
  node->get_parameter(plugin_name + ".x_goal_tolerance", x_goal_tolerance_);
  node->get_parameter(plugin_name + ".y_goal_tolerance", y_goal_tolerance_);
  node->get_parameter(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(plugin_name + ".trans_stopped_velocity", trans_stopped_velocity_);
  node->get_parameter(plugin_name + ".rot_stopped_velocity", rot_stopped_velocity_);
  node->get_parameter(plugin_name + ".stability_duration", stability_duration_);

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&StableStoppedGoalChecker::dynamicParametersCallback, this, _1));
}

void StableStoppedGoalChecker::reset()
{
  in_pose_tolerance_ = false;
}

bool StableStoppedGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity)
{
  // 1. Check Pose Tolerance (Split X, Y, Yaw)
  double dx = fabs(query_pose.position.x - goal_pose.position.x);
  double dy = fabs(query_pose.position.y - goal_pose.position.y);

  double dyaw = angles::shortest_angular_distance(
    tf2::getYaw(query_pose.orientation),
    tf2::getYaw(goal_pose.orientation));

  bool current_in_pose_tolerance = (dx <= x_goal_tolerance_) &&
                                   (dy <= y_goal_tolerance_) &&
                                   (fabs(dyaw) <= yaw_goal_tolerance_);

  // 2. Check Stability Duration
  if (current_in_pose_tolerance) {
    if (!in_pose_tolerance_) {
      // First time entering tolerance
      first_tolerance_time_ = clock_->now();
      in_pose_tolerance_ = true;
    }
    
    // Calculate how long we've been in tolerance
    double time_in_tolerance = (clock_->now() - first_tolerance_time_).seconds();

    // If we haven't stayed long enough, we are not done yet.
    if (time_in_tolerance < stability_duration_) {
      return false;
    }

  } else {
    // We went out of tolerance, reset the timer
    in_pose_tolerance_ = false;
    return false;
  }

  // 3. Check Velocity (Stopped condition)
  // Only reached if Pose and Duration are satisfied
  return fabs(velocity.angular.z) <= rot_stopped_velocity_ &&
         hypot(velocity.linear.x, velocity.linear.y) <= trans_stopped_velocity_;
}

bool StableStoppedGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  double invalid_field = std::numeric_limits<double>::lowest();

  pose_tolerance.position.x = x_goal_tolerance_;
  pose_tolerance.position.y = y_goal_tolerance_;
  pose_tolerance.position.z = invalid_field;
  pose_tolerance.orientation =
    nav2_util::geometry_utils::orientationAroundZAxis(yaw_goal_tolerance_);

  vel_tolerance.linear.x = trans_stopped_velocity_;
  vel_tolerance.linear.y = trans_stopped_velocity_;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = rot_stopped_velocity_;

  return true;
}

rcl_interfaces::msg::SetParametersResult
StableStoppedGoalChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".x_goal_tolerance") {
        x_goal_tolerance_ = parameter.as_double();
      } else if (name == plugin_name_ + ".y_goal_tolerance") {
        y_goal_tolerance_ = parameter.as_double();
      } else if (name == plugin_name_ + ".yaw_goal_tolerance") {
        yaw_goal_tolerance_ = parameter.as_double();
      } else if (name == plugin_name_ + ".trans_stopped_velocity") {
        trans_stopped_velocity_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rot_stopped_velocity") {
        rot_stopped_velocity_ = parameter.as_double();
      } else if (name == plugin_name_ + ".stability_duration") {
        stability_duration_ = parameter.as_double();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav2_controller::StableStoppedGoalChecker, nav2_core::GoalChecker)

```

### 3. `nav2_params.yaml` 설정 예시

위 플러그인을 빌드하신 후, `controller_server` 파라미터에서 다음과 같이 설정하여 사용할 수 있습니다.

```yaml
controller_server:
  ros__parameters:
    goal_checker_plugins: ["stable_goal_checker"] 
    stable_goal_checker:
      plugin: "nav2_controller::StableStoppedGoalChecker"
      x_goal_tolerance: 0.05       # 5cm X축 허용 오차
      y_goal_tolerance: 0.05       # 5cm Y축 허용 오차
      yaw_goal_tolerance: 0.1      # 약 5.7도 회전 허용 오차
      trans_stopped_velocity: 0.01 # 1cm/s 이하일 때 정지 간주
      rot_stopped_velocity: 0.01   # 0.01rad/s 이하일 때 정지 간주
      stability_duration: 1.0      # 위 조건을 1초 이상 만족해야 함

```

이 코드는 기존 `SimpleGoalChecker`와 `StoppedGoalChecker`의 구조를 그대로 따르면서, 요구하신 **X/Y 분리** 및 **시간 지연(Time Persistence)** 로직을 완벽하게 통합했습니다.
###################################  
#######################################  
########################################  
별도의 패키지(Standalone Package)로 관리하고 싶으시군요. 아주 좋은 접근입니다. 이렇게 하면 Nav2 원본 소스를 건드리지 않고도 깔끔하게 기능을 확장하고 배포할 수 있습니다.

패키지 이름을 **`nav2_stable_stopped_goal_checker`**라고 가정하고 작성해 드리겠습니다.

### 📂 권장 디렉토리 구조

작성하기 전에 파일들이 어디에 위치해야 하는지 먼저 보여드릴게요. (이 구조대로 파일을 만드셔야 합니다.)

```text
nav2_stable_stopped_goal_checker/
├── CMakeLists.txt
├── package.xml
├── plugins.xml
├── include/
│   └── nav2_stable_stopped_goal_checker/
│       └── stable_stopped_goal_checker.hpp  <-- 헤더 파일 위치
└── src/
    └── stable_stopped_goal_checker.cpp      <-- 소스 파일 위치

```

---

### 1. package.xml

ROS 2 빌드 시스템(ament)과 Nav2가 이 패키지를 인식하고 플러그인을 로드할 수 있도록 정의합니다.

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>nav2_stable_stopped_goal_checker</name>
  <version>1.0.0</version>
  <description>
    A custom Nav2 Goal Checker plugin with split X/Y tolerances and time stability check.
  </description>
  <maintainer email="user@todo.todo">User Name</maintainer>
  <license>BSD-3-Clause</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>nav2_core</depend>
  <depend>nav2_util</depend>
  <depend>nav2_costmap_2d</depend>
  <depend>geometry_msgs</depend>
  <depend>pluginlib</depend>
  <depend>angles</depend>
  <depend>tf2</depend>
  <depend>tf2_geometry_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
    <nav2_core plugin="${prefix}/plugins.xml" />
  </export>
</package>

```

---

### 2. CMakeLists.txt

빌드 설정입니다. 라이브러리를 생성하고 설치하며, `plugins.xml`도 함께 설치되도록 설정합니다.

```cmake
cmake_minimum_required(VERSION 3.8)
project(nav2_stable_stopped_goal_checker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(angles REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
)

# create library
add_library(${PROJECT_NAME} SHARED
  src/stable_stopped_goal_checker.cpp
)

target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  nav2_core::nav2_core
  nav2_util::nav2_util
  nav2_costmap_2d::nav2_costmap_2d
  geometry_msgs::geometry_msgs
  pluginlib::pluginlib
  angles::angles
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
)

# ROS 2 Pluginlib Macro (Explicitly register easier for debugging, though macro in cpp handles it)
# 이 부분은 소스코드 내 PLUGINLIB_EXPORT_CLASS 매크로가 처리하므로 필수는 아니지만,
# 명시적 의존성을 위해 남겨둡니다.

# Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

# Install plugins.xml definition
install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  nav2_core
  geometry_msgs
)

ament_package()

```

---

### 3. plugins.xml

Pluginlib이 런타임에 이 라이브러리를 찾아 로드할 수 있도록 정보를 제공합니다.

```xml
<library path="nav2_stable_stopped_goal_checker">
  <class name="nav2_controller::StableStoppedGoalChecker" 
         type="nav2_controller::StableStoppedGoalChecker" 
         base_class_type="nav2_core::GoalChecker">
    <description>
      Checks if goal is reached with separate X/Y tolerances, stability duration, and zero velocity.
    </description>
  </class>
</library>

```

---

### ⚠️ 중요: C++ 소스 코드 수정 사항

별도 패키지로 분리했으므로, **헤더 파일의 경로**와 **Include 문**을 약간 수정해야 합니다.

1. **헤더 파일 (`include/nav2_stable_stopped_goal_checker/stable_stopped_goal_checker.hpp`)**
* 네임스페이스는 `nav2_controller`를 유지해도 되지만, 파일 경로는 위 디렉토리 구조를 따르세요.
* Include Guard(`ifndef ...`) 이름도 파일 경로에 맞게 바꿔주시면 좋습니다 (예: `NAV2_STABLE_STOPPED_GOAL_CHECKER__...`).


2. **소스 파일 (`src/stable_stopped_goal_checker.cpp`)**
* 헤더 include 경로를 변경해야 합니다.
* **변경 전:** `#include "nav2_controller/plugins/stable_stopped_goal_checker.hpp"`
* **변경 후:** `#include "nav2_stable_stopped_goal_checker/stable_stopped_goal_checker.hpp"`



### 🚀 빌드 및 실행 방법

1. 워크스페이스의 `src` 폴더로 이동하여 위 파일들을 생성합니다.
2. 빌드:
```bash
colcon build --packages-select nav2_stable_stopped_goal_checker
source install/setup.bash

```


3. **사용:** `nav2_params.yaml`에서 `plugin` 타입을 다음과 같이 지정합니다. (네임스페이스는 코드에서 `nav2_controller`를 사용했으므로 그대로 둡니다.)
```yaml
plugin: "nav2_controller::StableStoppedGoalChecker"

```



이제 완벽하게 독립된 패키지로 커스텀 플러그인을 관리하실 수 있습니다! 추가적으로 Dockerfile 설정이나 의존성 관련 팁이 필요하시면 말씀해 주세요.
##############################  
#################################  
##################################  
전체 구조/클래스 설계 자체는 **Nav2 Jazzy의 `nav2_core::GoalChecker` 인터페이스 형태**와 잘 맞습니다. (initialize/reset/isGoalReached/getTolerances 구성도 정상 흐름이에요.)
다만 지금 상태 그대로면 **“컴파일/링크에서 터질 확률이 매우 큰 부분”**이 몇 군데 있습니다. 아래만 고치면 꽤 깔끔하게 동작할 가능성이 높습니다.

---

## 1) (치명) `.cpp`에서 헤더 include 경로가 패키지 구조랑 불일치

현재 패키지 구조:

* `include/nav2_stable_stopped_goal_checker/stable_stopped_goal_checker.hpp`

그런데 `.cpp`는:

```cpp
#include "nav2_controller/plugins/stable_stopped_goal_checker.hpp"
```

➡️ 이건 **그 경로에 파일이 없어서 컴파일 에러** 납니다.

### 해결안 (추천)

`.cpp`를 아래처럼 바꾸세요.

```cpp
#include "nav2_stable_stopped_goal_checker/stable_stopped_goal_checker.hpp"
```

또는 반대로, include 디렉토리를 `include/nav2_controller/plugins/...`로 바꾸고 파일도 그 위치로 옮기면 지금 include를 유지할 수 있어요(하지만 패키지명이 nav2_controller가 아니므로 보통은 비추천).

---

## 2) (치명) CMake에서 `target_link_libraries()`에 “없는 타겟”을 링크하고 있을 가능성 큼

지금 CMake:

```cmake
target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  ...
  geometry_msgs::geometry_msgs
  ...
  nav2_core::nav2_core
)
```

ROS2 ament 패키지들 중 상당수는 `pkg::pkg` 형태의 **imported target을 제공하지 않습니다**.
그래서 Jazzy 환경/빌드 방식에 따라 `"Target ... not found"` 류로 터질 수 있어요.

Nav2/ROS2 플러그인 예시들은 보통 **`ament_target_dependencies()`** 패턴을 씁니다. (Nav2 플러그인 export 예시에서도 그렇게 합니다. ([Robotics Stack Exchange][1]))

### 해결안 (추천 CMake 형태)

```cmake
add_library(${PROJECT_NAME} SHARED
  src/stable_stopped_goal_checker.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_util
  nav2_costmap_2d
  geometry_msgs
  pluginlib
  angles
  tf2
)

# (중요) pluginlib export (아래 3번 참고)
pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp rclcpp_lifecycle nav2_core nav2_util nav2_costmap_2d
  geometry_msgs pluginlib angles tf2
)

ament_package()
```

> `tf2_geometry_msgs`는 현재 코드에서 사실상 안 쓰고 있어서 의존성에서 빼도 됩니다(있어도 큰 문제는 없지만요).

---

## 3) (강력 권장) `pluginlib_export_plugin_description_file()` 추가

지금은 package.xml에

```xml
<nav2_core plugin="${prefix}/plugins.xml" />
```

를 넣어둬서 “설명 파일 위치”는 알려주고 있지만, Nav2 플러그인 패턴에선 CMake에서 아래 매크로를 같이 쓰는 게 일반적입니다. ([Robotics Stack Exchange][1])

```cmake
pluginlib_export_plugin_description_file(nav2_core plugins.xml)
```

이게 있으면 ament index 쪽에 플러그인 description이 잘 등록되어 **런타임 로딩 이슈**를 줄이는 데 도움이 됩니다.

---

## 4) `plugins.xml` 포맷: 지금 형태도 보통 동작하지만 더 안전한 표준형으로 감싸기 권장

현재:

```xml
<library path="nav2_stable_stopped_goal_checker">
  <class ...>
```

pluginlib은 환경에 따라 `<class_libraries>` 루트로 감싼 예시가 널리 쓰입니다. ([Robotics Stack Exchange][1])
아래처럼 바꾸면 호환성 면에서 더 안전합니다.

```xml
<class_libraries>
  <library path="nav2_stable_stopped_goal_checker">
    <class name="nav2_controller::StableStoppedGoalChecker"
           type="nav2_controller::StableStoppedGoalChecker"
           base_class_type="nav2_core::GoalChecker">
      <description>...</description>
    </class>
  </library>
</class_libraries>
```

---

## 5) 코드 로직 측면에서 “오류”는 아니지만, 운영 시 흔히 겪는 함정들

### (A) `reset()`에서 타이머도 같이 초기화하면 더 명확

지금은 `in_pose_tolerance_ = false;`만 해서도 동작은 하지만, 가독성/안전성 측면에서 타임스탬프도 초기화하면 디버깅이 쉬워요.

```cpp
void StableStoppedGoalChecker::reset()
{
  in_pose_tolerance_ = false;
  first_tolerance_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());
}
```

### (B) `dynamicParametersCallback`는 `const std::vector<...>&`가 복사 줄여줌

성능/스타일 개선:

```cpp
dynamicParametersCallback(const std::vector<rclcpp::Parameter> & parameters);
```

### (C) Tolerance의 의미

`dx <= x_tol && dy <= y_tol` 은 “사각형 영역”입니다.
원형 범위(유클리드)로 하고 싶으면 `hypot(dx,dy) <= xy_tol` 로 바꿔야 해요.
지금이 의도(축별 tolerance)라면 문제 없습니다.

---

## 6) YAML 사용 예시는 Nav2 방식과 잘 맞음

`controller_server.ros__parameters.goal_checker_plugins` 와 각 플러그인 네임스페이스 아래 파라미터를 두는 방식은 Nav2 문서 패턴과 일치합니다. ([Nav2 Documentation][2])

---

# 결론: “지금 당장 고쳐야 할 것” 3개

1. **cpp include 경로 수정** (`nav2_stable_stopped_goal_checker/...`)
2. **CMake의 target_link_libraries → ament_target_dependencies로 변경** (imported target 문제 방지) ([Robotics Stack Exchange][1])
3. **`pluginlib_export_plugin_description_file(nav2_core plugins.xml)` 추가** ([Robotics Stack Exchange][1])

원하면, 내가 위 수정사항을 반영한 **최종 CMakeLists.txt / plugins.xml / include 경로까지 완전히 일관된 패키지 템플릿**으로 “그대로 복붙하면 빌드되는 버전”을 통째로 정리해서 줄게.

[1]: https://robotics.stackexchange.com/questions/102107/exporting-plugins-to-nav2-how-to "ros - Exporting plugins to nav2. How to? - Robotics Stack Exchange"
[2]: https://docs.nav2.org/configuration/packages/configuring-controller-server.html?utm_source=chatgpt.com "Controller Server — Nav2 1.0.0 documentation"
