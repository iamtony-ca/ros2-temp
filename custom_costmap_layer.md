네, 물론입니다. `PluginContainerLayer`가 자체적으로 조합한 costmap을 ROS 2 토픽으로 발행하도록 수정한 전체 코드를 제공해 드리겠습니다.

아래 두 파일을 `nav2_costmap_2d` 패키지의 소스 디렉토리(`src/` 또는 `src/costmap_plugins/`)에 덮어쓰거나, 새로운 이름으로 저장하여 `CMakeLists.txt`에 추가한 뒤 `colcon`으로 빌드하면 됩니다.

수정된 부분은 `// MODIFIED:` 또는 `// ADDED:` 주석으로 표시해 두었습니다.

-----

## **`plugin_container_layer.hpp` (수정된 헤더 파일)**

헤더 파일에는 `OccupancyGrid` 메시지 타입과 `Publisher` 멤버 변수를 추가했습니다.

```cpp
// Copyright (c) 2024 Polymath Robotics, Inc.
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

#ifndef NAV2_COSTMAP_2D__PLUGIN_CONTAINER_LAYER_HPP_
#define NAV2_COSTMAP_2D__PLUGIN_CONTAINER_LAYER_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "pluginlib/class_loader.hpp"

// ADDED: Include the OccupancyGrid message header
#include "nav_msgs/msg/occupancy_grid.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using rcl_interfaces::msg::ParameterType;
namespace nav2_costmap_2d
{
/**
 * @class PluginContainerLayer
 * @brief Holds a list of plugins and applies them only to the specific layer
 */
class PluginContainerLayer : public CostmapLayer
{
public:
  /**
   * @brief Initialization process of layer on startup
   */
  virtual void onInitialize();
  /**
   * @brief Update the bounds of the master costmap by this layer's update
   *dimensions
   * @param robot_x X pose of robot
   * @param robot_y Y pose of robot
   * @param robot_yaw Robot orientation
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  virtual void updateBounds(
    double robot_x,
    double robot_y,
    double robot_yaw,
    double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  /**
   * @brief Update the costs in the master costmap in the window
   * @param master_grid The master costmap grid to update
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i,
    int min_j,
    int max_i,
    int max_j);
  virtual void onFootprintChanged();
  /** @brief Update the footprint to match size of the parent costmap. */
  virtual void matchSize();
  /**
   * @brief Deactivate the layer
   */
  virtual void deactivate();
  /**
   * @brief Activate the layer
   */
  virtual void activate();
  /**
   * @brief Reset this costmap
   */
  virtual void reset();
  /**
   * @brief If clearing operations should be processed on this layer or not
   */
  virtual bool isClearable();
  /**
   * @brief Clear an area in the constituent costmaps with the given dimension
   * if invert, then clear everything except these dimensions
   */
  void clearArea(int start_x, int start_y, int end_x, int end_y, bool invert) override;

  void addPlugin(std::shared_ptr<Layer> plugin, std::string layer_name);
  pluginlib::ClassLoader<Layer> plugin_loader_{"nav2_costmap_2d", "nav2_costmap_2d::Layer"};
  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

private:
  /// @brief Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    dyn_params_handler_;

  nav2_costmap_2d::CombinationMethod combination_method_;
  std::vector<std::shared_ptr<Layer>> plugins_;
  std::vector<std::string> plugin_names_;
  std::vector<std::string> plugin_types_;

  // ADDED: Publisher for this container's combined costmap
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr container_costmap_pub_;
};
}  // namespace nav2_costmap_2d
#endif  // NAV2_COSTMAP_2D__PLUGIN_CONTAINER_LAYER_HPP_

```

-----

## **`plugin_container_layer.cpp` (수정된 소스 파일)**

소스 파일에는 Publisher를 초기화하고, `updateCosts` 함수 내에서 조합된 costmap 데이터를 메시지에 담아 발행하는 코드를 추가했습니다.

```cpp
// Copyright (c) 2024 Polymath Robotics, Inc.
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

#include "nav2_costmap_2d/plugin_container_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "pluginlib/class_list_macros.hpp"

// ADDED: For memcpy
#include <cstring>

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::PluginContainerLayer, nav2_costmap_2d::Layer)

using std::vector;

namespace nav2_costmap_2d
{

void PluginContainerLayer::onInitialize()
{
  auto node = node_.lock();

  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(node, name_ + "." + "enabled",
      rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node, name_ + "." + "plugins",
      rclcpp::ParameterValue(std::vector<std::string>{}));
  nav2_util::declare_parameter_if_not_declared(node, name_ + "." + "combination_method",
      rclcpp::ParameterValue(1));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "plugins", plugin_names_);

  int combination_method_param{};
  node->get_parameter(name_ + "." + "combination_method", combination_method_param);
  combination_method_ = combination_method_from_int(combination_method_param);

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &PluginContainerLayer::dynamicParametersCallback,
      this,
      std::placeholders::_1));

  plugin_types_.resize(plugin_names_.size());

  for (unsigned int i = 0; i < plugin_names_.size(); ++i) {
    plugin_types_[i] = nav2_util::get_plugin_type_param(node, name_ + "." + plugin_names_[i]);
    std::shared_ptr<Layer> plugin = plugin_loader_.createSharedInstance(plugin_types_[i]);
    addPlugin(plugin, plugin_names_[i]);
  }

  default_value_ = nav2_costmap_2d::NO_INFORMATION;

  PluginContainerLayer::matchSize();
  
  // ADDED: Initialize the publisher for this container's costmap.
  // The topic will be e.g. /global_costmap/container_A/costmap_raw
  container_costmap_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    name_ + "/costmap_raw", rclcpp::SystemDefaultsQoS());

  current_ = true;
}

void PluginContainerLayer::addPlugin(std::shared_ptr<Layer> plugin, std::string layer_name)
{
  plugins_.push_back(plugin);
  auto node = node_.lock();
  plugin->initialize(layered_costmap_, name_ + "." + layer_name, tf_, node, callback_group_);
}

void PluginContainerLayer::updateBounds(
  double robot_x,
  double robot_y,
  double robot_yaw,
  double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end();
    ++plugin)
  {
    (*plugin)->updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
  }
}

void PluginContainerLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i,
  int min_j,
  int max_i,
  int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) {
    return;
  }

  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end();
    ++plugin)
  {
    (*plugin)->updateCosts(*this, min_i, min_j, max_i, max_j);
  }

  // ADDED: Publish the container's internally combined costmap before updating the master grid.
  if (container_costmap_pub_->get_subscription_count() > 0) {
    auto grid = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    
    grid->header.frame_id = layered_costmap_->getGlobalFrameID();
    grid->header.stamp = node_.lock()->now();
    grid->info.resolution = resolution_;
    grid->info.width = size_x_;
    grid->info.height = size_y_;
    
    // Set origin
    grid->info.origin.position.x = origin_x_;
    grid->info.origin.position.y = origin_y_;
    grid->info.origin.position.z = 0.0;
    grid->info.origin.orientation.w = 1.0;

    grid->data.resize(size_x_ * size_y_);

    // Copy the costmap data from unsigned char* to std::vector<int8_t>
    memcpy(grid->data.data(), costmap_, size_x_ * size_y_ * sizeof(unsigned char));
    
    container_costmap_pub_->publish(std::move(grid));
  }

  switch (combination_method_) {
    case CombinationMethod::Overwrite:
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case CombinationMethod::Max:
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    case CombinationMethod::MaxWithoutUnknownOverwrite:
      updateWithMaxWithoutUnknownOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:  // Nothing
      break;
  }

  current_ = true;
}

void PluginContainerLayer::activate()
{
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end();
    ++plugin)
  {
    (*plugin)->activate();
  }
}

void PluginContainerLayer::deactivate()
{
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end();
    ++plugin)
  {
    (*plugin)->deactivate();
  }
}

void PluginContainerLayer::reset()
{
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end();
    ++plugin)
  {
    (*plugin)->reset();
  }
  resetMaps();
  current_ = false;
}

void PluginContainerLayer::onFootprintChanged()
{
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end();
    ++plugin)
  {
    (*plugin)->onFootprintChanged();
  }
}

void PluginContainerLayer::matchSize()
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  Costmap2D * master = layered_costmap_->getCostmap();
  resizeMap(
    master->getSizeInCellsX(), master->getSizeInCellsY(),
    master->getResolution(), master->getOriginX(), master->getOriginY());

  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end();
    ++plugin)
  {
    (*plugin)->matchSize();
  }
}

bool PluginContainerLayer::isClearable()
{
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end();
    ++plugin)
  {
    if((*plugin)->isClearable()) {
      return true;
    }
  }
  return false;
}

void PluginContainerLayer::clearArea(int start_x, int start_y, int end_x, int end_y, bool invert)
{
  CostmapLayer::clearArea(start_x, start_y, end_x, end_y, invert);
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end();
    ++plugin)
  {
    auto costmap_layer = std::dynamic_pointer_cast<nav2_costmap_2d::CostmapLayer>(*plugin);
    if ((*plugin)->isClearable() && costmap_layer != nullptr) {
      costmap_layer->clearArea(start_x, start_y, end_x, end_y, invert);
    }
  }
}

rcl_interfaces::msg::SetParametersResult PluginContainerLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == name_ + "." + "combination_method") {
        combination_method_ = combination_method_from_int(parameter.as_int());
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == name_ + "." + "enabled" && enabled_ != parameter.as_bool()) {
        enabled_ = parameter.as_bool();
        current_ = false;
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_costmap_2d
```


###############################  





네, 물론입니다. `amr_costmap_2d` 네임스페이스를 사용하고, 바로 빌드할 수 있는 완전한 ROS 2 패키지 형태로 제공해 드리겠습니다.

아래 설명에 따라 디렉토리를 만들고 각 파일을 해당 위치에 복사한 뒤, ROS 2 워크스페이스에서 빌드하면 `amr_costmap_2d::PluginContainerLayer`를 바로 사용할 수 있습니다.

-----

### **1. 패키지 구조**

ROS 2 워크스페이스(`your_ros2_ws/src`) 안에 다음과 같은 구조로 디렉토리와 파일을 생성하세요.

```
your_ros2_ws/src/
└── amr_costmap_2d/
    ├── include/
    │   └── amr_costmap_2d/
    │       └── plugin_container_layer.hpp
    ├── src/
    │   └── plugin_container_layer.cpp
    ├── amr_costmap_plugins.xml
    ├── CMakeLists.txt
    └── package.xml
```

-----

### **2. `package.xml`**

패키지의 정보와 의존성을 정의합니다.

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>amr_costmap_2d</name>
  <version>1.0.0</version>
  <description>A custom costmap_2d layer that contains other plugins and publishes its own map.</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>nav2_costmap_2d</depend>
  <depend>pluginlib</depend>
  <depend>nav_msgs</depend>
  <depend>nav2_util</depend>
  <depend>rcl_interfaces</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
    <nav2_costmap_2d plugin="${prefix}/amr_costmap_plugins.xml" />
  </export>
</package>
```

-----

### **3. `CMakeLists.txt`**

패키지를 빌드하는 방법을 정의합니다.

```cmake
cmake_minimum_required(VERSION 3.8)
project(amr_costmap_2d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(amr_costmap_2d_layer SHARED
  src/plugin_container_layer.cpp
)

target_include_directories(amr_costmap_2d_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(amr_costmap_2d_layer PUBLIC
  "rclcpp"
  "nav2_costmap_2d"
  "pluginlib"
  "nav_msgs"
)

pluginlib_export_plugin_description_file(nav2_costmap_2d amr_costmap_plugins.xml)

install(
  TARGETS amr_costmap_2d_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES amr_costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

-----

### **4. `amr_costmap_plugins.xml`**

`pluginlib`이 이 패키지의 플러그인을 찾을 수 있도록 정보를 제공하는 파일입니다.

```xml
<library path="amr_costmap_2d/libamr_costmap_2d_layer">
  <class name="amr_costmap_2d/PluginContainerLayer"
         type="amr_costmap_2d::PluginContainerLayer"
         base_class_type="nav2_costmap_2d::Layer">
    <description>
      A layer that contains other plugins and publishes its own combined costmap.
    </description>
  </class>
</library>
```

-----

### **5. `include/amr_costmap_2d/plugin_container_layer.hpp`**

**Namespace가 `amr_costmap_2d`로 변경된** 헤더 파일입니다.

```cpp
// Copyright (c) 2024 Polymath Robotics, Inc.
// MODIFIED by Gemini for amr_costmap_2d package

#ifndef AMR_COSTMAP_2D__PLUGIN_CONTAINER_LAYER_HPP_
#define AMR_COSTMAP_2D__PLUGIN_CONTAINER_LAYER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pluginlib/class_loader.hpp"

namespace amr_costmap_2d
{
class PluginContainerLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  PluginContainerLayer()
  : plugin_loader_("nav2_costmap_2d", "nav2_costmap_2d::Layer") {}

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i,
    int max_j);
  virtual void onFootprintChanged();
  virtual void matchSize();
  virtual void deactivate();
  virtual void activate();
  virtual void reset();
  virtual bool isClearable();
  void clearArea(int start_x, int start_y, int end_x, int end_y, bool invert) override;
  void addPlugin(std::shared_ptr<nav2_costmap_2d::Layer> plugin, std::string layer_name);

  pluginlib::ClassLoader<nav2_costmap_2d::Layer> plugin_loader_;
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

private:
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  nav2_costmap_2d::CombinationMethod combination_method_;
  std::vector<std::shared_ptr<nav2_costmap_2d::Layer>> plugins_;
  std::vector<std::string> plugin_names_;
  std::vector<std::string> plugin_types_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr container_costmap_pub_;
};
}  // namespace amr_costmap_2d
#endif  // AMR_COSTMAP_2D__PLUGIN_CONTAINER_LAYER_HPP_
```

-----

### **6. `src/plugin_container_layer.cpp`**

**Namespace가 `amr_costmap_2d`로 변경된** 소스 파일입니다.

```cpp
// Copyright (c) 2024 Polymath Robotics, Inc.
// MODIFIED by Gemini for amr_costmap_2d package

#include "amr_costmap_2d/plugin_container_layer.hpp"

#include <cstring>
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(amr_costmap_2d::PluginContainerLayer, nav2_costmap_2d::Layer)

namespace amr_costmap_2d
{

void PluginContainerLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + "." + "enabled", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + "." + "plugins", rclcpp::ParameterValue(std::vector<std::string>{}));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + "." + "combination_method", rclcpp::ParameterValue(1));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "plugins", plugin_names_);

  int combination_method_param{};
  node->get_parameter(name_ + "." + "combination_method", combination_method_param);
  combination_method_ = nav2_costmap_2d::combination_method_from_int(combination_method_param);

  dyn_params_handler_ =
    node->add_on_set_parameters_callback(
    std::bind(&PluginContainerLayer::dynamicParametersCallback, this, std::placeholders::_1));

  plugin_types_.resize(plugin_names_.size());
  for (unsigned int i = 0; i < plugin_names_.size(); ++i) {
    plugin_types_[i] = nav2_util::get_plugin_type_param(node, name_ + "." + plugin_names_[i]);
    std::shared_ptr<nav2_costmap_2d::Layer> plugin =
      plugin_loader_.createSharedInstance(plugin_types_[i]);
    addPlugin(plugin, plugin_names_[i]);
  }

  default_value_ = nav2_costmap_2d::NO_INFORMATION;
  PluginContainerLayer::matchSize();
  container_costmap_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    name_ + "/costmap_raw", rclcpp::SystemDefaultsQoS());
  current_ = true;
}

void PluginContainerLayer::addPlugin(
  std::shared_ptr<nav2_costmap_2d::Layer> plugin, std::string layer_name)
{
  plugins_.push_back(plugin);
  auto node = node_.lock();
  plugin->initialize(layered_costmap_, name_ + "." + layer_name, tf_, node, callback_group_);
}

void PluginContainerLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  for (auto & plugin : plugins_) {
    plugin->updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
  }
}

void PluginContainerLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) {
    return;
  }

  for (auto & plugin : plugins_) {
    plugin->updateCosts(*this, min_i, min_j, max_i, max_j);
  }

  if (container_costmap_pub_->get_subscription_count() > 0) {
    auto grid = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    grid->header.frame_id = layered_costmap_->getGlobalFrameID();
    grid->header.stamp = node_.lock()->now();
    grid->info.resolution = resolution_;
    grid->info.width = size_x_;
    grid->info.height = size_y_;
    grid->info.origin.position.x = origin_x_;
    grid->info.origin.position.y = origin_y_;
    grid->info.origin.position.z = 0.0;
    grid->info.origin.orientation.w = 1.0;
    grid->data.resize(size_x_ * size_y_);
    memcpy(grid->data.data(), costmap_, size_x_ * size_y_ * sizeof(unsigned char));
    container_costmap_pub_->publish(std::move(grid));
  }

  switch (combination_method_) {
    case nav2_costmap_2d::CombinationMethod::Overwrite:
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case nav2_costmap_2d::CombinationMethod::Max:
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    case nav2_costmap_2d::CombinationMethod::MaxWithoutUnknownOverwrite:
      updateWithMaxWithoutUnknownOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:
      break;
  }
  current_ = true;
}

// ... (이하 나머지 함수들은 원본과 동일)

void PluginContainerLayer::activate()
{
  for (auto & plugin : plugins_) {
    plugin->activate();
  }
}

void PluginContainerLayer::deactivate()
{
  for (auto & plugin : plugins_) {
    plugin->deactivate();
  }
}

void PluginContainerLayer::reset()
{
  for (auto & plugin : plugins_) {
    plugin->reset();
  }
  resetMaps();
  current_ = false;
}

void PluginContainerLayer::onFootprintChanged()
{
  for (auto & plugin : plugins_) {
    plugin->onFootprintChanged();
  }
}

void PluginContainerLayer::matchSize()
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  nav2_costmap_2d::Costmap2D * master = layered_costmap_->getCostmap();
  resizeMap(
    master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
    master->getOriginX(), master->getOriginY());
  for (auto & plugin : plugins_) {
    plugin->matchSize();
  }
}

bool PluginContainerLayer::isClearable()
{
  for (auto & plugin : plugins_) {
    if (plugin->isClearable()) {
      return true;
    }
  }
  return false;
}

void PluginContainerLayer::clearArea(int start_x, int start_y, int end_x, int end_y, bool invert)
{
  CostmapLayer::clearArea(start_x, start_y, end_x, end_y, invert);
  for (auto & plugin : plugins_) {
    auto costmap_layer =
      std::dynamic_pointer_cast<nav2_costmap_2d::CostmapLayer>(plugin);
    if (plugin->isClearable() && costmap_layer != nullptr) {
      costmap_layer->clearArea(start_x, start_y, end_x, end_y, invert);
    }
  }
}

rcl_interfaces::msg::SetParametersResult PluginContainerLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
      if (param_name == name_ + "." + "combination_method") {
        combination_method_ = nav2_costmap_2d::combination_method_from_int(parameter.as_int());
      }
    } else if (param_type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
      if (param_name == name_ + "." + "enabled" && enabled_ != parameter.as_bool()) {
        enabled_ = parameter.as_bool();
        current_ = false;
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace amr_costmap_2d
```

-----

### **7. 빌드 및 사용 방법**

**빌드:**

1.  ROS 2 워크스페이스의 `src` 폴더로 이동합니다.
2.  위 구조에 맞게 `amr_costmap_2d` 패키지를 생성합니다.
3.  워크스페이스의 루트로 돌아가서 빌드합니다.

<!-- end list -->

```bash
# ROS 2 환경을 소싱합니다.
source /opt/ros/jazzy/setup.bash

# 워크스페이스 루트로 이동
cd ~/your_ros2_ws

# amr_costmap_2d 패키지만 선택하여 빌드
colcon build --packages-select amr_costmap_2d
```

**사용법 (`nav2_params.yaml`):**

이제 Nav2 설정 파일에서 이 플러그인을 일반 레이어처럼 사용할 수 있습니다. `plugin:` 필드에 `amr_costmap_plugins.xml`에 정의한 `name`을 사용합니다.

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "wanted_layers_container", "unwanted_layer"]
      # ... other global costmap parameters

      # 포함하고 싶은 레이어들을 담는 컨테이너
      wanted_layers_container:
        plugin: "amr_costmap_2d/PluginContainerLayer"
        plugins: ["obstacle_layer", "inflation_layer"] # 예시: 이 컨테이너는 obstacle과 inflation을 포함
      
      # 제외하고 싶은 레이어 (그냥 독립적으로 둠)
      unwanted_layer:
        plugin: "nav2_costmap_2d::VoxelLayer" # 예시
        # ... voxel layer parameters

      # 컨테이너에 포함된 레이어들의 상세 설정
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        # ... obstacle layer parameters

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        # ... inflation layer parameters
      
      # ...
```

위와 같이 설정하고 Nav2를 실행하면, `wanted_layers_container`가 발행하는 토픽을 구독할 수 있습니다.

  * **토픽 이름:** `/global_costmap/wanted_layers_container/costmap_raw`

이 토픽을 구독하면 `obstacle_layer`와 `inflation_layer`가 합쳐진 costmap을 얻을 수 있습니다.
