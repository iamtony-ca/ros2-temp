네, **Jetson Orin AGX의 성능을 극대화하면서 딜레이를 잡는 "VoxelGrid + LaserScan" 완벽 조합**의 Launch 코드를 작성해 드립니다.

이 코드는 **Composable Node(컴포저블 노드)** 방식을 사용합니다.
데이터가 노드 사이를 이동할 때 복사(Copy)하지 않고 메모리 주소만 넘겨주는 **Zero-Copy** 방식이라, 일반 노드보다 훨씬 빠르고 CPU 부하가 적습니다.

### 📋 사전 준비 (패키지 설치)

실행하기 전에 필요한 패키지가 설치되어 있는지 터미널에서 확인해주세요.

```bash
sudo apt update
sudo apt install ros-humble-pcl-ros ros-humble-pointcloud-to-laserscan
```

-----

### 🚀 `voxel_to_scan.launch.py` (완성본)

이 파일을 `launch` 폴더에 저장하고 실행하시면 됩니다.

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    
    # ========================================================================
    # 1. 파라미터 설정 (필요시 터미널에서 변경 가능하도록 Argument로 뺌)
    # ========================================================================
    
    # VoxelGrid: 점을 얼마나 뭉칠 것인가? (0.05 = 5cm, 0.1 = 10cm)
    # 이 값이 클수록 데이터가 가벼워지고 딜레이가 줄어듭니다.
    leaf_size = LaunchConfiguration('leaf_size', default='0.05')
    
    # 높이 필터링: 바닥과 천장 제거 (단위: m)
    min_height = LaunchConfiguration('min_height', default='0.1')
    max_height = LaunchConfiguration('max_height', default='1.7')
    
    # ZED 데이터 토픽 (입력)
    input_topic = LaunchConfiguration('input_topic', default='/zed_multi/zed_front/point_cloud/cloud_registered')
    
    # 출력될 TF 프레임 (X축이 정면인 프레임 사용 권장)
    target_frame = LaunchConfiguration('target_frame', default='zed_front_left_camera_frame')

    return LaunchDescription([
        
        DeclareLaunchArgument('leaf_size', default_value='0.05', description='Voxel Leaf Size'),
        DeclareLaunchArgument('min_height', default_value='0.1', description='Min height of obstacle'),
        DeclareLaunchArgument('max_height', default_value='1.7', description='Max height of obstacle'),

        # ========================================================================
        # 2. Composable Node Container (Zero-Copy를 위한 컨테이너)
        # ========================================================================
        ComposableNodeContainer(
            name='perception_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt', # Multi-Threaded 컨테이너 사용 (성능 향상)
            composable_node_descriptions=[
                
                # ----------------------------------------------------------------
                # Node A: VoxelGrid Filter (데이터 다이어트)
                # 역할: 수백만 개의 점을 5cm 단위로 뭉쳐서 수만 개로 줄임
                # ----------------------------------------------------------------
                ComposableNode(
                    package='pcl_ros',
                    plugin='pcl::VoxelGrid',
                    name='voxel_grid_filter',
                    parameters=[{
                        'leaf_size': leaf_size, 
                        'filter_field_name': 'z', # Z축(높이) 기준으로도 1차 필터링 수행
                        'filter_limit_min': min_height,
                        'filter_limit_max': max_height,
                        'filter_limit_negative': False # 범위 밖의 점은 버림
                    }],
                    remappings=[
                        ('input', input_topic),
                        ('output', '/zed_multi/zed_front/point_cloud/downsampled') # 중간 토픽
                    ]
                ),

                # ----------------------------------------------------------------
                # Node B: PointCloud to LaserScan (2D 변환)
                # 역할: 가벼워진 데이터를 받아서 2D LaserScan으로 납작하게 만듦
                # ----------------------------------------------------------------
                ComposableNode(
                    package='pointcloud_to_laserscan',
                    plugin='pointcloud_to_laserscan::PointCloudToLaserScanNode',
                    name='pointcloud_to_laserscan',
                    parameters=[{
                        'target_frame': target_frame,
                        'transform_tolerance': 0.05, # TF 지연 허용치
                        
                        # 높이 필터링 (VoxelGrid에서 이미 했지만, 안전장치로 한 번 더)
                        'min_height': min_height,
                        'max_height': max_height,
                        
                        # 스캔 범위 설정
                        'angle_min': -1.57, # -90도
                        'angle_max': 1.57,  # +90도
                        'angle_increment': 0.0087, # 0.5도 간격 (너무 조밀하지 않게)
                        'scan_time': 0.033, # 30Hz 기준
                        'range_min': 0.45,
                        'range_max': 10.0,
                        'use_inf': True
                    }],
                    remappings=[
                        # VoxelGrid가 뱉어낸 'downsampled' 데이터를 입력으로 받음
                        ('cloud_in', '/zed_multi/zed_front/point_cloud/downsampled'),
                        ('scan', '/scan_front') # 최종 결과 토픽
                    ]
                )
            ],
            output='screen',
        )
    ])
```

-----

### 💡 핵심 포인트 설명 (튜닝 가이드)

1.  **`component_container_mt` 사용:**

      * 일반 `component_container` 대신 Multi-Threaded(`mt`) 버전을 사용하여, VoxelGrid 처리와 LaserScan 변환을 병렬로 처리합니다. Orin AGX의 멀티코어를 잘 활용합니다.

2.  **`leaf_size` (가장 중요):**

      * 기본값 `0.05` (5cm)로 설정했습니다.
      * 만약 여전히 느리다면 터미널에서 실행할 때 `0.1` (10cm)로 늘려보세요.
      * 명령어 예시: `ros2 launch my_pkg voxel_to_scan.launch.py leaf_size:=0.1`

3.  **2중 높이 필터링:**

      * `VoxelGrid`에서 1차로 높이(0.1\~1.7m)를 잘라내고, `LaserScan` 노드에서 2차로 확인합니다.
      * 이렇게 하면 바닥(Ground)과 천장 데이터를 **변환 단계에 진입하기도 전에 버리기 때문에** 속도가 훨씬 빨라집니다.

이 코드를 실행하면 RViz2에서 `/zed_multi/zed_front/point_cloud/downsampled` 토픽을 확인해 보세요. 점들이 듬성듬성하지만 물체 형상은 유지된 채로 매우 빠르게 갱신되는 것을 볼 수 있을 것입니다.
#####################  
########################  
###########################  
아래 코드는 **ROS2 Humble + PCL 기반의 VoxelGrid 필터 노드**를 완전히 구현한 예시예요.
`PointCloud2` 입력 토픽을 받아서 **0.05 m voxel**로 downsample 해서 다른 토픽으로 publish 합니다.

구성:

1. `voxel_filter_node.cpp` (노드 소스)
2. `CMakeLists.txt`
3. `package.xml`

---

## 1. 소스 코드: `src/voxel_filter_node.cpp`

```cpp
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>

class VoxelFilterNode : public rclcpp::Node
{
public:
  VoxelFilterNode()
  : Node("voxel_filter_node")
  {
    // Parameters
    this->declare_parameter<std::string>("input_topic", "/points_raw");
    this->declare_parameter<std::string>("output_topic", "/points_voxel");
    this->declare_parameter<double>("voxel_leaf_size", 0.05);

    this->get_parameter("input_topic", input_topic_);
    this->get_parameter("output_topic", output_topic_);
    this->get_parameter("voxel_leaf_size", voxel_leaf_size_);

    RCLCPP_INFO(get_logger(), "VoxelFilterNode started.");
    RCLCPP_INFO(get_logger(), "  input_topic : %s", input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  output_topic: %s", output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  voxel size  : %.3f m", voxel_leaf_size_);

    // QoS: 센서 데이터용 QoS 사용 (드롭 허용)
    auto qos = rclcpp::SensorDataQoS();

    // Publisher / Subscriber
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      qos,
      std::bind(&VoxelFilterNode::pointCloudCallback, this, std::placeholders::_1)
    );
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!msg) {
      RCLCPP_WARN(get_logger(), "Received null PointCloud2 message.");
      return;
    }

    // ROS2 PointCloud2 -> PCL PointCloud2
    pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr pcl_cloud_filtered(new pcl::PCLPointCloud2());

    pcl_conversions::toPCL(*msg, *pcl_cloud);

    // VoxelGrid 필터 설정
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
    voxel_filter.setInputCloud(pcl_cloud);
    voxel_filter.setLeafSize(
      static_cast<float>(voxel_leaf_size_),
      static_cast<float>(voxel_leaf_size_),
      static_cast<float>(voxel_leaf_size_)
    );

    voxel_filter.filter(*pcl_cloud_filtered);

    // PCL -> ROS2 PointCloud2
    sensor_msgs::msg::PointCloud2 output;
    pcl_conversions::fromPCL(*pcl_cloud_filtered, output);

    // 입력 cloud의 header 유지 (frame_id, stamp 등)
    output.header = msg->header;

    pub_->publish(output);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  std::string input_topic_;
  std::string output_topic_;
  double voxel_leaf_size_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoxelFilterNode>());
  rclcpp::shutdown();
  return 0;
}
```

---

## 2. CMake 설정: `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.5)
project(voxel_filter)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# For faster builds
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall -Wextra")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(voxel_filter_node src/voxel_filter_node.cpp)

ament_target_dependencies(voxel_filter_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(voxel_filter_node
  ${PCL_LIBRARIES}
)

install(TARGETS
  voxel_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

---

## 3. 패키지 메타 정보: `package.xml`

```xml
<?xml version="1.0"?>
<package format="3">
  <name>voxel_filter</name>
  <version>0.0.1</version>
  <description>VoxelGrid filter node for PointCloud2 in ROS2 Humble</description>

  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>pcl_conversions</depend>

  <!-- 시스템 의존성 (Ubuntu 기준) : libpcl-dev -->
  <exec_depend>libpcl-dev</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## 4. 빌드 & 실행 방법

워크스페이스 예시: `~/ros2_ws/src/voxel_filter`

```bash
cd ~/ros2_ws
colcon build --packages-select voxel_filter
source install/setup.bash
```

기본 파라미터(입력: `/points_raw`, 출력: `/points_voxel`, voxel size: 0.05 m)로 실행:

```bash
ros2 run voxel_filter voxel_filter_node
```

다른 토픽/voxel size를 쓰고 싶으면:

```bash
ros2 run voxel_filter voxel_filter_node \
  --ros-args \
  -p input_topic:=/lidar/points \
  -p output_topic:=/lidar/points_voxel \
  -p voxel_leaf_size:=0.05
```

---

원하는 구조가 Python 버전이거나, TF/frame 변경, dynamic parameter callback까지 넣고 싶으면 말해줘. 그 버전으로도 바로 만들어 줄게!
