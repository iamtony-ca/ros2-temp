좋아. 아래는 **ROS2(rclpy) + OpenCV StereoSGBM**로 **rectified left/right 이미지**를 받아서 **disparity(시차) + depth(미터)** 를 **10Hz 이상**으로 퍼블리시하는 **Python 노드 예시**야.

* 입력: `left/image_rect`, `right/image_rect`, `left/camera_info`, `right/camera_info`
* 출력:

  * `~/disparity` : `stereo_msgs/DisparityImage` (권장)
  * `~/depth` : `sensor_msgs/Image` (`32FC1`, meters, invalid=NaN)
* 설계: **이미지 pair는 계속 최신으로 저장**, **timer에서 지정한 output_hz(기본 10Hz)** 로만 SGBM 계산 → CPU 부하/지터 관리가 쉬움

---

## `stereo_sgbm_depth_node.py`

```python
#!/usr/bin/env python3
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge

import message_filters


def _as_3x4(P):
    return np.array(P, dtype=np.float64).reshape(3, 4)


class StereoSGBMDepthNode(Node):
    """
    Subscribes to rectified stereo images + camera_info and publishes:
      - stereo_msgs/DisparityImage (disparity in pixels)
      - sensor_msgs/Image depth (32FC1 meters, NaN for invalid)

    Notes:
      - Assumes input images are already rectified (image_rect).
      - Baseline is estimated from right CameraInfo P matrix:
            P_right[0,3] = -fx * B   =>   B = -P_right[0,3] / fx
    """

    def __init__(self):
        super().__init__('stereo_sgbm_depth_node')

        # ---- Parameters (topics) ----
        self.declare_parameter('left_image_topic',  '/left/image_rect')
        self.declare_parameter('right_image_topic', '/right/image_rect')
        self.declare_parameter('left_info_topic',   '/left/camera_info')
        self.declare_parameter('right_info_topic',  '/right/camera_info')

        # ---- Parameters (output & runtime) ----
        self.declare_parameter('output_hz', 10.0)
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('sync_slop_sec', 0.03)   # ~33ms: good for 30fps
        self.declare_parameter('scale', 1.0)            # <1.0 => downscale for speed (e.g., 0.8, 0.5)
        self.declare_parameter('opencv_num_threads', 0) # 0: OpenCV default

        # ---- StereoSGBM parameters ----
        self.declare_parameter('min_disparity', 0)
        self.declare_parameter('num_disparities', 128)  # MUST be multiple of 16
        self.declare_parameter('block_size', 5)         # odd number: 3..11 typical
        self.declare_parameter('uniqueness_ratio', 10)
        self.declare_parameter('speckle_window_size', 100)
        self.declare_parameter('speckle_range', 2)
        self.declare_parameter('disp12_max_diff', 1)
        self.declare_parameter('pre_filter_cap', 31)
        self.declare_parameter('mode', 'SGBM_3WAY')     # 'SGBM', 'SGBM_3WAY', etc.

        # Optional: disable depth for very small disparity
        self.declare_parameter('min_valid_disparity', 1.0)

        # Setup OpenCV threads
        ocv_threads = int(self.get_parameter('opencv_num_threads').value)
        if ocv_threads > 0:
            cv2.setNumThreads(ocv_threads)

        self.bridge = CvBridge()

        # CameraInfo cache
        self.left_info: CameraInfo | None = None
        self.right_info: CameraInfo | None = None

        # Latest synced images (stored as ROS messages)
        self.latest_left_msg: Image | None = None
        self.latest_right_msg: Image | None = None
        self.latest_stamp = None

        # Publishers
        self.pub_disp = self.create_publisher(DisparityImage, '~/disparity', 10)
        self.pub_depth = self.create_publisher(Image, '~/depth', 10)

        # CameraInfo subscriptions (cache)
        left_info_topic = self.get_parameter('left_info_topic').value
        right_info_topic = self.get_parameter('right_info_topic').value
        self.create_subscription(CameraInfo, left_info_topic, self._left_info_cb, 10)
        self.create_subscription(CameraInfo, right_info_topic, self._right_info_cb, 10)

        # Image subscriptions with approximate sync
        left_img_topic = self.get_parameter('left_image_topic').value
        right_img_topic = self.get_parameter('right_image_topic').value

        qsize = int(self.get_parameter('queue_size').value)
        slop = float(self.get_parameter('sync_slop_sec').value)

        left_sub = message_filters.Subscriber(self, Image, left_img_topic)
        right_sub = message_filters.Subscriber(self, Image, right_img_topic)

        sync = message_filters.ApproximateTimeSynchronizer(
            [left_sub, right_sub],
            queue_size=qsize,
            slop=slop
        )
        sync.registerCallback(self._stereo_cb)

        # Timer for fixed-rate processing
        out_hz = float(self.get_parameter('output_hz').value)
        out_hz = max(1.0, out_hz)
        self.timer = self.create_timer(1.0 / out_hz, self._process_once)

        # Build SGBM object
        self.sgbm = self._create_sgbm()

        self.get_logger().info(
            f"StereoSGBMDepthNode started.\n"
            f"  left_image_topic:  {left_img_topic}\n"
            f"  right_image_topic: {right_img_topic}\n"
            f"  left_info_topic:   {left_info_topic}\n"
            f"  right_info_topic:  {right_info_topic}\n"
            f"  output_hz:         {out_hz}\n"
            f"  scale:             {float(self.get_parameter('scale').value)}"
        )

    # ----------------- Callbacks -----------------
    def _left_info_cb(self, msg: CameraInfo):
        self.left_info = msg

    def _right_info_cb(self, msg: CameraInfo):
        self.right_info = msg

    def _stereo_cb(self, left_msg: Image, right_msg: Image):
        # store latest only (processing happens in timer)
        self.latest_left_msg = left_msg
        self.latest_right_msg = right_msg
        self.latest_stamp = left_msg.header.stamp

    # ----------------- Core processing -----------------
    def _create_sgbm(self):
        min_disp = int(self.get_parameter('min_disparity').value)
        num_disp = int(self.get_parameter('num_disparities').value)
        block_size = int(self.get_parameter('block_size').value)

        # Ensure valid constraints
        if num_disp <= 0:
            num_disp = 128
        # StereoSGBM requires numDisparities multiple of 16
        num_disp = int(math.ceil(num_disp / 16) * 16)

        if block_size < 3:
            block_size = 3
        if block_size % 2 == 0:
            block_size += 1

        # P1/P2 rule of thumb
        cn = 1  # we'll feed grayscale
        P1 = 8 * cn * block_size * block_size
        P2 = 32 * cn * block_size * block_size

        uniqueness = int(self.get_parameter('uniqueness_ratio').value)
        speckle_ws = int(self.get_parameter('speckle_window_size').value)
        speckle_rng = int(self.get_parameter('speckle_range').value)
        disp12 = int(self.get_parameter('disp12_max_diff').value)
        pre_cap = int(self.get_parameter('pre_filter_cap').value)

        mode_str = str(self.get_parameter('mode').value).upper()
        if mode_str == 'SGBM_3WAY':
            mode = cv2.STEREO_SGBM_MODE_SGBM_3WAY
        elif mode_str == 'HH':
            mode = cv2.STEREO_SGBM_MODE_HH
        elif mode_str == 'SGBM':
            mode = cv2.STEREO_SGBM_MODE_SGBM
        else:
            mode = cv2.STEREO_SGBM_MODE_SGBM_3WAY

        sgbm = cv2.StereoSGBM_create(
            minDisparity=min_disp,
            numDisparities=num_disp,
            blockSize=block_size,
            P1=P1,
            P2=P2,
            disp12MaxDiff=disp12,
            preFilterCap=pre_cap,
            uniquenessRatio=uniqueness,
            speckleWindowSize=speckle_ws,
            speckleRange=speckle_rng,
            mode=mode
        )

        self.get_logger().info(
            f"SGBM configured: minDisp={min_disp}, numDisp={num_disp}, blockSize={block_size}, mode={mode_str}"
        )
        return sgbm

    def _process_once(self):
        # Need images + camera infos
        if self.latest_left_msg is None or self.latest_right_msg is None:
            return
        if self.left_info is None or self.right_info is None:
            return

        # Convert ROS Image -> OpenCV
        try:
            left_cv = self.bridge.imgmsg_to_cv2(self.latest_left_msg, desired_encoding='passthrough')
            right_cv = self.bridge.imgmsg_to_cv2(self.latest_right_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        # Ensure grayscale uint8
        left_gray = self._to_gray_u8(left_cv)
        right_gray = self._to_gray_u8(right_cv)

        # Optional downscale for speed
        scale = float(self.get_parameter('scale').value)
        scale = float(np.clip(scale, 0.1, 1.0))
        if scale != 1.0:
            left_gray = cv2.resize(left_gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            right_gray = cv2.resize(right_gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

        # Get intrinsics & baseline
        fx, baseline = self._get_fx_and_baseline(self.left_info, self.right_info)
        if fx is None or baseline is None or baseline <= 0.0:
            self.get_logger().warn("Failed to compute fx/baseline from CameraInfo. Check right P matrix.")
            return

        fx_s = fx * scale  # because disparity computed on scaled images

        # Compute disparity (OpenCV returns fixed-point: disp * 16)
        disp_raw = self.sgbm.compute(left_gray, right_gray)  # int16
        disp = disp_raw.astype(np.float32) / 16.0

        # Mask invalid / small disparity
        min_valid_disp = float(self.get_parameter('min_valid_disparity').value)
        disp_invalid = disp <= max(0.0, min_valid_disp)
        disp[disp_invalid] = np.nan

        # Depth in meters: Z = fx * B / disparity
        depth = (fx_s * baseline) / disp  # NaN stays NaN

        # Publish disparity as stereo_msgs/DisparityImage
        disp_msg = DisparityImage()
        disp_msg.header = self.latest_left_msg.header
        disp_msg.header.frame_id = self.latest_left_msg.header.frame_id  # keep left frame

        disp_img = self.bridge.cv2_to_imgmsg(disp, encoding='32FC1')
        disp_img.header = disp_msg.header
        disp_msg.image = disp_img

        # Fill metadata
        disp_msg.f = float(fx_s)
        disp_msg.t = float(baseline)
        # min/max disparity (informative)
        min_disp = int(self.get_parameter('min_disparity').value)
        num_disp = int(self.get_parameter('num_disparities').value)
        num_disp = int(math.ceil(num_disp / 16) * 16)
        disp_msg.min_disparity = float(min_disp)
        disp_msg.max_disparity = float(min_disp + num_disp)
        disp_msg.delta_d = 1.0 / 16.0

        self.pub_disp.publish(disp_msg)

        # Publish depth as sensor_msgs/Image (32FC1, meters)
        depth_msg = self.bridge.cv2_to_imgmsg(depth.astype(np.float32), encoding='32FC1')
        depth_msg.header = self.latest_left_msg.header
        self.pub_depth.publish(depth_msg)

    def _to_gray_u8(self, img):
        # img may be mono8, bgr8, rgb8, etc.
        if img.ndim == 2:
            gray = img
        else:
            # Assume BGR by default (common in ROS/OpenCV)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if gray.dtype == np.uint8:
            return gray

        # If 16-bit or float, normalize to 8-bit for SGBM
        g = gray.astype(np.float32)
        g = g - np.nanmin(g)
        denom = np.nanmax(g)
        if denom <= 1e-6:
            return np.zeros_like(gray, dtype=np.uint8)
        g = (g / denom) * 255.0
        return g.astype(np.uint8)

    def _get_fx_and_baseline(self, left_info: CameraInfo, right_info: CameraInfo):
        # fx from left P (or K) — use P[0]
        P_left = _as_3x4(left_info.p)
        P_right = _as_3x4(right_info.p)

        fx = float(P_left[0, 0])
        if fx <= 0.0:
            return None, None

        # Baseline from right projection matrix:
        # P_right[0,3] = Tx = -fx * B  =>  B = -Tx / fx
        Tx = float(P_right[0, 3])
        baseline = -Tx / fx

        return fx, baseline


def main():
    rclpy.init()
    node = StereoSGBMDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 실행 예시

토픽 이름이 다르면 파라미터로 바꿔서 실행하면 돼.

```bash
ros2 run <your_pkg> stereo_sgbm_depth_node \
  --ros-args \
  -p left_image_topic:=/stereo/left/image_rect \
  -p right_image_topic:=/stereo/right/image_rect \
  -p left_info_topic:=/stereo/left/camera_info \
  -p right_info_topic:=/stereo/right/camera_info \
  -p output_hz:=10.0 \
  -p num_disparities:=128 \
  -p block_size:=5 \
  -p scale:=1.0
```

### 10Hz가 빡빡하면(CPU 여유 없으면) 이렇게:

* **scale=0.8** 또는 **0.5** 로 내리면 성능이 확 좋아져요.

```bash
-p scale:=0.5
```

---

## 바로 튜닝 추천 (SVGA / depth 10Hz 목표)

* **1차 목표(10Hz 달성)**: `scale=0.5` + `num_disparities=96~128` + `block_size=5`
* **품질 올리기**: `scale=0.8` 또는 `1.0`으로 올리되 `num_disparities`는 필요한 만큼만 유지
* “가까운 물체 위주”면 `num_disparities`를 줄이는 게 가장 큰 속도 이득이에요.

---

원하면 다음도 바로 붙여줄게:

1. **PointCloud2** 생성(깊이 + 카메라 모델)
2. (선택) **WLS 후처리** 옵션까지 포함한 버전(품질↑, CPU↑)
3. RViz2에서 depth 확인용 **컬러화(Colormap) 이미지 토픽** 추가

지금 카메라 토픽 프레임 이름(예: `left_camera_frame`)이랑, baseline이 대략 몇 cm인지(알면) 알려주면 파라미터도 더 딱 맞게 잡아줄 수 있어요.
###########################  
#############################  
##############################  
좋아—이미지 rect + camera_info를 받아서 **SGBM disparity + depth + PointCloud2**까지 퍼블리시하는 **ROS2 Humble/Jazzy 호환 C++ 패키지(소스/CMakelists/package.xml/launch까지)**를 한 번에 제공할게요. (처리는 10Hz 타이머로 고정하고, 입력 30Hz는 최신 프레임만 사용하도록 설계합니다.)


아래는 **ROS2 Humble/Jazzy**에서 그대로 빌드/실행 가능한 **완전한 C++ 패키지 예시**입니다.

* 입력(이미 rectified 된 것 사용):

  * `/left/image_rect` (`sensor_msgs/Image`)
  * `/right/image_rect`
  * `/left/camera_info` (`sensor_msgs/CameraInfo`)
  * `/right/camera_info`
* 출력:

  * `~/disparity` (`stereo_msgs/DisparityImage`, 32FC1 disparity in pixels)
  * `~/depth` (`sensor_msgs/Image`, 32FC1 depth in meters, invalid=NaN)
  * `~/points` (`sensor_msgs/PointCloud2`, XYZ in meters; invalid은 NaN)
* 성능/구조:

  * 입력은 30Hz로 들어와도 **최신 pair만 저장**
  * **10Hz 타이머**에서만 SGBM + depth + pointcloud 생성 (CPU 부하 안정)
  * optional `scale`로 다운스케일해서 10Hz 확보 가능

> ⚠️ 전제: `image_rect`가 “정확히 정렬된 rectified stereo”여야 합니다. (그게 이미 되고 있다고 하셨으니 OK)

---

# 1) 패키지 구조

패키지명 예: `stereo_sgbm_depth`

```
stereo_sgbm_depth/
  CMakeLists.txt
  package.xml
  include/stereo_sgbm_depth/stereo_sgbm_depth_node.hpp
  src/stereo_sgbm_depth_node.cpp
  src/main.cpp
  launch/stereo_sgbm_depth.launch.py
```

---

# 2) `package.xml`

```xml
<?xml version="1.0"?>
<package format="3">
  <name>stereo_sgbm_depth</name>
  <version>0.0.1</version>
  <description>StereoSGBM-based disparity/depth/PointCloud2 node for ROS2 Humble/Jazzy (CPU)</description>
  <maintainer email="you@example.com">you</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>stereo_msgs</depend>
  <depend>image_transport</depend>
  <depend>cv_bridge</depend>
  <depend>message_filters</depend>

  <depend>camera_info_manager</depend>

  <build_depend>opencv</build_depend>
  <exec_depend>opencv</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

# 3) `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.8)
project(stereo_sgbm_depth)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_library(stereo_sgbm_depth_node_lib
  src/stereo_sgbm_depth_node.cpp
)
ament_target_dependencies(stereo_sgbm_depth_node_lib
  rclcpp sensor_msgs stereo_msgs image_transport cv_bridge message_filters
)
target_link_libraries(stereo_sgbm_depth_node_lib ${OpenCV_LIBS})

add_executable(stereo_sgbm_depth_node
  src/main.cpp
)
target_link_libraries(stereo_sgbm_depth_node stereo_sgbm_depth_node_lib)
ament_target_dependencies(stereo_sgbm_depth_node rclcpp)

install(TARGETS
  stereo_sgbm_depth_node
  stereo_sgbm_depth_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

---

# 4) 헤더: `include/stereo_sgbm_depth/stereo_sgbm_depth_node.hpp`

```cpp
#pragma once

#include <mutex>
#include <optional>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

namespace stereo_sgbm_depth
{

class StereoSGBMDepthNode : public rclcpp::Node
{
public:
  explicit StereoSGBMDepthNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using ImageMsg = sensor_msgs::msg::Image;
  using CameraInfoMsg = sensor_msgs::msg::CameraInfo;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  // Callbacks
  void cameraInfoLeftCb(const CameraInfoMsg::SharedPtr msg);
  void cameraInfoRightCb(const CameraInfoMsg::SharedPtr msg);
  void stereoCb(const ImageMsg::ConstSharedPtr & left, const ImageMsg::ConstSharedPtr & right);

  void onTimer();

  // Helpers
  cv::Mat toGrayU8(const ImageMsg::ConstSharedPtr & msg) const;
  bool computeFxAndBaseline(double & fx, double & baseline) const;

  void rebuildSGBM();

  // Latest data cache
  mutable std::mutex mtx_;
  ImageMsg::ConstSharedPtr latest_left_;
  ImageMsg::ConstSharedPtr latest_right_;
  rclcpp::Time latest_stamp_{0, 0, RCL_ROS_TIME};
  CameraInfoMsg::SharedPtr left_info_;
  CameraInfoMsg::SharedPtr right_info_;

  // Subscribers
  rclcpp::Subscription<CameraInfoMsg>::SharedPtr sub_left_info_;
  rclcpp::Subscription<CameraInfoMsg>::SharedPtr sub_right_info_;

  std::shared_ptr<message_filters::Subscriber<ImageMsg>> sub_left_img_;
  std::shared_ptr<message_filters::Subscriber<ImageMsg>> sub_right_img_;
  std::shared_ptr<Synchronizer> sync_;

  // Publishers
  rclcpp::Publisher<stereo_msgs::msg::DisparityImage>::SharedPtr pub_disparity_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_points_;

  rclcpp::TimerBase::SharedPtr timer_;

  // OpenCV
  cv::Ptr<cv::StereoSGBM> sgbm_;

  // Parameters (topics)
  std::string left_image_topic_;
  std::string right_image_topic_;
  std::string left_info_topic_;
  std::string right_info_topic_;

  // Parameters (runtime)
  double output_hz_{10.0};
  int queue_size_{10};
  double sync_slop_sec_{0.03};
  double scale_{1.0};

  // SGBM params
  int min_disparity_{0};
  int num_disparities_{128};  // multiple of 16
  int block_size_{5};         // odd
  int uniqueness_ratio_{10};
  int speckle_window_size_{100};
  int speckle_range_{2};
  int disp12_max_diff_{1};
  int pre_filter_cap_{31};
  std::string mode_{"SGBM_3WAY"};
  double min_valid_disparity_{1.0};

  // PointCloud settings
  bool publish_pointcloud_{true};
};

}  // namespace stereo_sgbm_depth
```

---

# 5) 구현: `src/stereo_sgbm_depth_node.cpp`

```cpp
#include "stereo_sgbm_depth/stereo_sgbm_depth_node.hpp"

#include <cmath>
#include <limits>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace stereo_sgbm_depth
{

static inline int roundUpToMultiple(int v, int m)
{
  return ((v + m - 1) / m) * m;
}

StereoSGBMDepthNode::StereoSGBMDepthNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("stereo_sgbm_depth_node", options)
{
  // ---- Declare parameters ----
  this->declare_parameter<std::string>("left_image_topic", "/left/image_rect");
  this->declare_parameter<std::string>("right_image_topic", "/right/image_rect");
  this->declare_parameter<std::string>("left_info_topic", "/left/camera_info");
  this->declare_parameter<std::string>("right_info_topic", "/right/camera_info");

  this->declare_parameter<double>("output_hz", 10.0);
  this->declare_parameter<int>("queue_size", 10);
  this->declare_parameter<double>("sync_slop_sec", 0.03);
  this->declare_parameter<double>("scale", 1.0);

  this->declare_parameter<int>("min_disparity", 0);
  this->declare_parameter<int>("num_disparities", 128);
  this->declare_parameter<int>("block_size", 5);
  this->declare_parameter<int>("uniqueness_ratio", 10);
  this->declare_parameter<int>("speckle_window_size", 100);
  this->declare_parameter<int>("speckle_range", 2);
  this->declare_parameter<int>("disp12_max_diff", 1);
  this->declare_parameter<int>("pre_filter_cap", 31);
  this->declare_parameter<std::string>("mode", "SGBM_3WAY");
  this->declare_parameter<double>("min_valid_disparity", 1.0);

  this->declare_parameter<bool>("publish_pointcloud", true);

  // ---- Get parameters ----
  left_image_topic_ = this->get_parameter("left_image_topic").as_string();
  right_image_topic_ = this->get_parameter("right_image_topic").as_string();
  left_info_topic_ = this->get_parameter("left_info_topic").as_string();
  right_info_topic_ = this->get_parameter("right_info_topic").as_string();

  output_hz_ = this->get_parameter("output_hz").as_double();
  queue_size_ = this->get_parameter("queue_size").as_int();
  sync_slop_sec_ = this->get_parameter("sync_slop_sec").as_double();
  scale_ = this->get_parameter("scale").as_double();

  min_disparity_ = this->get_parameter("min_disparity").as_int();
  num_disparities_ = this->get_parameter("num_disparities").as_int();
  block_size_ = this->get_parameter("block_size").as_int();
  uniqueness_ratio_ = this->get_parameter("uniqueness_ratio").as_int();
  speckle_window_size_ = this->get_parameter("speckle_window_size").as_int();
  speckle_range_ = this->get_parameter("speckle_range").as_int();
  disp12_max_diff_ = this->get_parameter("disp12_max_diff").as_int();
  pre_filter_cap_ = this->get_parameter("pre_filter_cap").as_int();
  mode_ = this->get_parameter("mode").as_string();
  min_valid_disparity_ = this->get_parameter("min_valid_disparity").as_double();

  publish_pointcloud_ = this->get_parameter("publish_pointcloud").as_bool();

  // ---- Publishers ----
  pub_disparity_ = this->create_publisher<stereo_msgs::msg::DisparityImage>("~/disparity", 10);
  pub_depth_ = this->create_publisher<sensor_msgs::msg::Image>("~/depth", 10);
  pub_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/points", 10);

  // ---- CameraInfo subscribers (cache) ----
  sub_left_info_ = this->create_subscription<CameraInfoMsg>(
    left_info_topic_, 10,
    std::bind(&StereoSGBMDepthNode::cameraInfoLeftCb, this, std::placeholders::_1));

  sub_right_info_ = this->create_subscription<CameraInfoMsg>(
    right_info_topic_, 10,
    std::bind(&StereoSGBMDepthNode::cameraInfoRightCb, this, std::placeholders::_1));

  // ---- Image subscribers + sync ----
  sub_left_img_ = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, left_image_topic_);
  sub_right_img_ = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, right_image_topic_);

  sync_ = std::make_shared<Synchronizer>(SyncPolicy(queue_size_), *sub_left_img_, *sub_right_img_);
  sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_sec_));
  sync_->registerCallback(std::bind(&StereoSGBMDepthNode::stereoCb, this,
                                    std::placeholders::_1, std::placeholders::_2));

  // ---- SGBM init ----
  rebuildSGBM();

  // ---- Timer ----
  if (output_hz_ < 1.0) output_hz_ = 1.0;
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / output_hz_),
    std::bind(&StereoSGBMDepthNode::onTimer, this));

  RCLCPP_INFO(this->get_logger(),
              "StereoSGBMDepthNode started.\n"
              "  left_image_topic:  %s\n"
              "  right_image_topic: %s\n"
              "  left_info_topic:   %s\n"
              "  right_info_topic:  %s\n"
              "  output_hz:         %.2f\n"
              "  scale:             %.2f\n"
              "  publish_pointcloud:%s",
              left_image_topic_.c_str(), right_image_topic_.c_str(),
              left_info_topic_.c_str(), right_info_topic_.c_str(),
              output_hz_, scale_, publish_pointcloud_ ? "true" : "false");
}

void StereoSGBMDepthNode::cameraInfoLeftCb(const CameraInfoMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  left_info_ = msg;
}

void StereoSGBMDepthNode::cameraInfoRightCb(const CameraInfoMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  right_info_ = msg;
}

void StereoSGBMDepthNode::stereoCb(const ImageMsg::ConstSharedPtr & left,
                                  const ImageMsg::ConstSharedPtr & right)
{
  std::lock_guard<std::mutex> lock(mtx_);
  latest_left_ = left;
  latest_right_ = right;
  latest_stamp_ = rclcpp::Time(left->header.stamp);
}

void StereoSGBMDepthNode::rebuildSGBM()
{
  // sanitize
  num_disparities_ = std::max(16, num_disparities_);
  num_disparities_ = roundUpToMultiple(num_disparities_, 16);

  block_size_ = std::max(3, block_size_);
  if ((block_size_ % 2) == 0) block_size_ += 1;

  // rule of thumb for P1/P2
  const int cn = 1;  // grayscale
  const int P1 = 8 * cn * block_size_ * block_size_;
  const int P2 = 32 * cn * block_size_ * block_size_;

  int mode = cv::StereoSGBM::MODE_SGBM_3WAY;
  std::string m = mode_;
  for (auto & c : m) c = static_cast<char>(::toupper(c));
  if (m == "SGBM") {
    mode = cv::StereoSGBM::MODE_SGBM;
  } else if (m == "HH") {
    mode = cv::StereoSGBM::MODE_HH;
  } else {
    mode = cv::StereoSGBM::MODE_SGBM_3WAY;
  }

  sgbm_ = cv::StereoSGBM::create(
    min_disparity_,
    num_disparities_,
    block_size_,
    P1,
    P2,
    disp12_max_diff_,
    pre_filter_cap_,
    uniqueness_ratio_,
    speckle_window_size_,
    speckle_range_,
    mode);

  RCLCPP_INFO(this->get_logger(),
              "SGBM configured: minDisp=%d numDisp=%d blockSize=%d mode=%s",
              min_disparity_, num_disparities_, block_size_, mode_.c_str());
}

cv::Mat StereoSGBMDepthNode::toGrayU8(const ImageMsg::ConstSharedPtr & msg) const
{
  // Convert ROS image -> cv::Mat (no copy if possible)
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
  } catch (const cv_bridge::Exception & e) {
    throw std::runtime_error(std::string("cv_bridge toCvShare failed: ") + e.what());
  }

  cv::Mat img = cv_ptr->image;
  cv::Mat gray;

  if (img.channels() == 1) {
    gray = img;
  } else {
    // most common in ROS is BGR8
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
  }

  if (gray.type() == CV_8UC1) {
    return gray;
  }

  // normalize other types to 8U
  cv::Mat gray_f;
  gray.convertTo(gray_f, CV_32F);
  double minv, maxv;
  cv::minMaxLoc(gray_f, &minv, &maxv);
  if (maxv - minv < 1e-6) {
    return cv::Mat(gray.size(), CV_8UC1, cv::Scalar(0));
  }
  cv::Mat out;
  gray_f = (gray_f - static_cast<float>(minv)) * (255.0f / static_cast<float>(maxv - minv));
  gray_f.convertTo(out, CV_8U);
  return out;
}

bool StereoSGBMDepthNode::computeFxAndBaseline(double & fx, double & baseline) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!left_info_ || !right_info_) return false;

  // fx from left P (preferred in rectified stereo)
  const auto & Pl = left_info_->p;
  const auto & Pr = right_info_->p;

  fx = Pl[0];  // P(0,0)
  if (fx <= 0.0) return false;

  // Baseline from right projection matrix:
  // Pr[3] = P(0,3) = Tx = -fx * B  =>  B = -Tx / fx
  const double Tx = Pr[3];
  baseline = -Tx / fx;

  return (baseline > 0.0);
}

void StereoSGBMDepthNode::onTimer()
{
  ImageMsg::ConstSharedPtr left;
  ImageMsg::ConstSharedPtr right;
  CameraInfoMsg::SharedPtr left_info;
  CameraInfoMsg::SharedPtr right_info;

  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!latest_left_ || !latest_right_ || !left_info_ || !right_info_) return;
    left = latest_left_;
    right = latest_right_;
    left_info = left_info_;
    right_info = right_info_;
  }

  // Convert to grayscale
  cv::Mat left_gray, right_gray;
  try {
    left_gray = toGrayU8(left);
    right_gray = toGrayU8(right);
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "Image conversion failed: %s", e.what());
    return;
  }

  // Downscale for speed if needed
  double scale = std::clamp(scale_, 0.1, 1.0);
  if (scale != 1.0) {
    cv::resize(left_gray, left_gray, cv::Size(), scale, scale, cv::INTER_AREA);
    cv::resize(right_gray, right_gray, cv::Size(), scale, scale, cv::INTER_AREA);
  }

  // fx & baseline
  double fx = 0.0, baseline = 0.0;
  if (!computeFxAndBaseline(fx, baseline)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Failed to compute fx/baseline from CameraInfo P matrices.");
    return;
  }
  const double fx_s = fx * scale;

  // Compute disparity (OpenCV fixed-point disp*16 in int16)
  cv::Mat disp16;
  sgbm_->compute(left_gray, right_gray, disp16);

  cv::Mat disp;
  disp16.convertTo(disp, CV_32F, 1.0 / 16.0);

  // Invalid / small disparity -> NaN
  const float min_valid = static_cast<float>(std::max(0.0, min_valid_disparity_));
  cv::Mat depth(disp.size(), CV_32F);

  const float nanf = std::numeric_limits<float>::quiet_NaN();

  for (int y = 0; y < disp.rows; ++y) {
    const float * dptr = disp.ptr<float>(y);
    float * zptr = depth.ptr<float>(y);
    for (int x = 0; x < disp.cols; ++x) {
      const float d = dptr[x];
      if (!std::isfinite(d) || d <= min_valid) {
        zptr[x] = nanf;
      } else {
        zptr[x] = static_cast<float>((fx_s * baseline) / static_cast<double>(d));
      }
    }
  }

  // Publish DisparityImage
  stereo_msgs::msg::DisparityImage disp_msg;
  disp_msg.header = left->header;
  disp_msg.header.frame_id = left->header.frame_id;

  cv_bridge::CvImage disp_bridge;
  disp_bridge.header = disp_msg.header;
  disp_bridge.encoding = "32FC1";
  disp_bridge.image = disp;

  disp_msg.image = *disp_bridge.toImageMsg();
  disp_msg.f = static_cast<float>(fx_s);
  disp_msg.t = static_cast<float>(baseline);
  disp_msg.min_disparity = static_cast<float>(min_disparity_);
  disp_msg.max_disparity = static_cast<float>(min_disparity_ + num_disparities_);
  disp_msg.delta_d = 1.0f / 16.0f;

  pub_disparity_->publish(disp_msg);

  // Publish depth image
  cv_bridge::CvImage depth_bridge;
  depth_bridge.header = left->header;
  depth_bridge.encoding = "32FC1";
  depth_bridge.image = depth;

  pub_depth_->publish(*depth_bridge.toImageMsg());

  // Publish PointCloud2 (XYZ)
  if (publish_pointcloud_) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = left->header;
    cloud.header.frame_id = left->header.frame_id;

    cloud.height = static_cast<uint32_t>(depth.rows);
    cloud.width = static_cast<uint32_t>(depth.cols);
    cloud.is_dense = false;

    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(static_cast<size_t>(cloud.height) * static_cast<size_t>(cloud.width));

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    // Use left CameraInfo intrinsics (K) for back-projection
    const double cx = left_info->k[2] * scale;
    const double cy = left_info->k[5] * scale;
    const double fy = left_info->k[4] * scale;

    for (int v = 0; v < depth.rows; ++v) {
      const float * zptr = depth.ptr<float>(v);
      for (int u = 0; u < depth.cols; ++u, ++iter_x, ++iter_y, ++iter_z) {
        const float Z = zptr[u];
        if (!std::isfinite(Z) || Z <= 0.0f) {
          *iter_x = nanf;
          *iter_y = nanf;
          *iter_z = nanf;
          continue;
        }

        // pinhole backprojection:
        // X = (u - cx) * Z / fx,  Y = (v - cy) * Z / fy
        const float X = static_cast<float>((static_cast<double>(u) - cx) * static_cast<double>(Z) / fx_s);
        const float Y = static_cast<float>((static_cast<double>(v) - cy) * static_cast<double>(Z) / fy);
        *iter_x = X;
        *iter_y = Y;
        *iter_z = Z;
      }
    }

    pub_points_->publish(cloud);
  }
}

}  // namespace stereo_sgbm_depth
```

---

# 6) `src/main.cpp`

```cpp
#include "stereo_sgbm_depth/stereo_sgbm_depth_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<stereo_sgbm_depth::StereoSGBMDepthNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

---

# 7) Launch: `launch/stereo_sgbm_depth.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stereo_sgbm_depth',
            executable='stereo_sgbm_depth_node',
            name='stereo_sgbm_depth_node',
            output='screen',
            parameters=[{
                'left_image_topic': '/left/image_rect',
                'right_image_topic': '/right/image_rect',
                'left_info_topic': '/left/camera_info',
                'right_info_topic': '/right/camera_info',

                'output_hz': 10.0,
                'scale': 1.0,              # 0.5~1.0
                'num_disparities': 128,     # multiple of 16
                'block_size': 5,
                'min_valid_disparity': 1.0,
                'mode': 'SGBM_3WAY',

                'publish_pointcloud': True,
            }],
        )
    ])
```

---

# 8) 빌드 & 실행

```bash
# workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# (여기에 stereo_sgbm_depth 폴더를 넣고)

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

ros2 launch stereo_sgbm_depth stereo_sgbm_depth.launch.py
```

토픽 리맵/파라미터 오버라이드 예:

```bash
ros2 launch stereo_sgbm_depth stereo_sgbm_depth.launch.py \
  left_image_topic:=/stereo/left/image_rect \
  right_image_topic:=/stereo/right/image_rect \
  left_info_topic:=/stereo/left/camera_info \
  right_info_topic:=/stereo/right/camera_info
```

---

# 9) 10Hz 이상을 “확실히” 만드는 실전 설정

SVGA(800×600)에서 CPU만으로 10Hz가 애매하면 아래 순서로 조정하면 됩니다.

1. `scale:=0.5` (가장 강력)
2. `num_disparities:=96` 또는 `64` (근거리 위주면 크게 줄여도 됨)
3. `block_size:=5` 유지 (7로 올리면 보통 느려짐)
4. `mode:=SGBM_3WAY` 유지

---

원하면 다음도 바로 확장해줄게요(실제 로봇에서 자주 필요):

* PointCloud에 **RGB 색깔 포함(XYZRGB)**
* **ROI**만 depth 계산해서 더 빠르게 (예: 로봇 전방 하단 영역만)
* **WLS 후처리** 옵션(품질↑, CPU↑)
* `QoS`를 센서용으로 조정(`SensorDataQoS`)해서 지연/버퍼 최적화

지금 `left/right image_rect`의 encoding이 보통 `mono8`인지 `bgr8`인지(또는 `rgb8`)만 알려주면, PointCloud에 색 넣는 버disp_msg.header.################
############################  
################################  
#################################  
좋아. **WLS 후처리(=opencv_contrib의 ximgproc)** 를 “옵션”으로 붙이려면 핵심이 2개야:

1. **left disparity + right disparity 둘 다 필요** (WLS가 L/R consistency를 보기 때문)
2. `opencv_contrib`의 **`cv::ximgproc`** 를 써야 함

아래는 네가 올린 C++ 노드에 **WLS on/off 파라미터**를 추가한 “완전 동작 버전” 변경사항이야.

---

## 0) (필수) opencv_contrib 설치 확인

Ubuntu에서 보통 아래로 해결돼:

```bash
sudo apt update
sudo apt install -y libopencv-contrib-dev
```

---

## 1) `CMakeLists.txt` 수정 (ximgproc 컴포넌트 추가)

기존 `find_package(OpenCV REQUIRED)`를 아래처럼 바꾸는 걸 추천해.

```cmake
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d ximgproc)
```

그리고 link는 그대로 `${OpenCV_LIBS}`면 충분한 경우가 많아.

> 만약 환경에 따라 COMPONENTS가 문제를 일으키면, `find_package(OpenCV REQUIRED)`로 두고 빌드가 ximgproc를 찾는지 확인하는 방식도 가능하지만, “명확히” 하려면 위처럼 쓰는 게 좋아.

---

## 2) 헤더 수정: `stereo_sgbm_depth_node.hpp`

### 추가 include

```cpp
#include <opencv2/ximgproc/disparity_filter.hpp>
```

### 클래스 멤버 추가 (WLS 관련)

`private:` 아래에 추가:

```cpp
  // WLS
  bool enable_wls_{false};
  double wls_lambda_{8000.0};     // typical: 8000~20000
  double wls_sigma_{1.5};         // typical: 0.8~2.0
  int wls_lrc_thresh_{24};        // typical: 5~30
  int wls_depth_disc_radius_{3};  // typical: 1~5

  cv::Ptr<cv::StereoMatcher> right_matcher_;
  cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter_;
```

---

## 3) 구현 수정: `stereo_sgbm_depth_node.cpp`

### (A) 파라미터 선언/로드 추가

생성자에서 declare:

```cpp
  this->declare_parameter<bool>("enable_wls", false);
  this->declare_parameter<double>("wls_lambda", 8000.0);
  this->declare_parameter<double>("wls_sigma", 1.5);
  this->declare_parameter<int>("wls_lrc_thresh", 24);
  this->declare_parameter<int>("wls_depth_disc_radius", 3);
```

get_parameter 로드:

```cpp
  enable_wls_ = this->get_parameter("enable_wls").as_bool();
  wls_lambda_ = this->get_parameter("wls_lambda").as_double();
  wls_sigma_ = this->get_parameter("wls_sigma").as_double();
  wls_lrc_thresh_ = this->get_parameter("wls_lrc_thresh").as_int();
  wls_depth_disc_radius_ = this->get_parameter("wls_depth_disc_radius").as_int();
```

로그에도 표시하고 싶으면:

```cpp
  RCLCPP_INFO(this->get_logger(),
              "WLS: enable=%s lambda=%.1f sigma=%.2f lrc=%d depth_disc_radius=%d",
              enable_wls_ ? "true" : "false",
              wls_lambda_, wls_sigma_, wls_lrc_thresh_, wls_depth_disc_radius_);
```

---

### (B) `rebuildSGBM()`에서 right matcher / wls filter 생성

`rebuildSGBM()` 마지막에 아래를 추가해줘.

```cpp
  // WLS helpers (need left matcher)
  // right matcher computes disparity for the right view
  right_matcher_ = cv::ximgproc::createRightMatcher(sgbm_);

  // WLS filter guided by left image
  wls_filter_ = cv::ximgproc::createDisparityWLSFilter(sgbm_);
  wls_filter_->setLambda(wls_lambda_);
  wls_filter_->setSigmaColor(wls_sigma_);
  wls_filter_->setLRCthresh(wls_lrc_thresh_);
  wls_filter_->setDepthDiscontinuityRadius(wls_depth_disc_radius_);
```

---

### (C) `onTimer()`에서 disparity 계산부를 “WLS 옵션 분기”로 변경

기존에는:

```cpp
  cv::Mat disp16;
  sgbm_->compute(left_gray, right_gray, disp16);

  cv::Mat disp;
  disp16.convertTo(disp, CV_32F, 1.0 / 16.0);
```

이 부분을 아래처럼 교체:

```cpp
  cv::Mat disp_left16;
  sgbm_->compute(left_gray, right_gray, disp_left16);   // CV_16S (disp*16)

  cv::Mat disp16_to_use = disp_left16;

  if (enable_wls_) {
    // Need right disparity too
    cv::Mat disp_right16;
    right_matcher_->compute(right_gray, left_gray, disp_right16); // CV_16S

    // Filtered output (CV_16S)
    cv::Mat disp_filtered16;
    // guidance image must be 8-bit 1ch or 3ch; we have left_gray CV_8U
    wls_filter_->setLambda(wls_lambda_);
    wls_filter_->setSigmaColor(wls_sigma_);
    wls_filter_->setLRCthresh(wls_lrc_thresh_);
    wls_filter_->setDepthDiscontinuityRadius(wls_depth_disc_radius_);

    wls_filter_->filter(disp_left16, left_gray, disp_filtered16, disp_right16);

    disp16_to_use = disp_filtered16;
  }

  cv::Mat disp;
  disp16_to_use.convertTo(disp, CV_32F, 1.0 / 16.0);
```

이 뒤의 depth/pointcloud 계산은 **disp를 그대로 사용**하면 돼. (WLS가 켜져 있으면 더 깔끔한 disparity가 들어오게 됨)

---

## 4) launch에 파라미터 추가

`launch/stereo_sgbm_depth.launch.py`에 옵션 넣어줘.

```python
'enable_wls': True,
'wls_lambda': 8000.0,
'wls_sigma': 1.5,
'wls_lrc_thresh': 24,
'wls_depth_disc_radius': 3,
```

---

## 5) 성능 팁 (SVGA에서 10Hz 유지)

WLS는 품질은 좋아지지만 CPU를 더 먹어. 10Hz가 흔들리면 아래 순서로 조절하면 안정적이야.

* `scale:=0.5` (가장 강력)
* `num_disparities:=96` (또는 64)
* `wls_lambda`는 크게 올릴수록 “매끈”, 대신 경계가 뭉개질 수 있음
* `wls_sigma`는 색상(가이던스) 기반 smoothing 강도 느낌이라 1.0~2.0 사이에서 조절

---

## 6) 실행 예시

```bash
ros2 run stereo_sgbm_depth stereo_sgbm_depth_node --ros-args \
  -p left_image_topic:=/stereo/left/image_rect \
  -p right_image_topic:=/stereo/right/image_rect \
  -p left_info_topic:=/stereo/left/camera_info \
  -p right_info_topic:=/stereo/right/camera_info \
  -p output_hz:=10.0 \
  -p scale:=0.5 \
  -p enable_wls:=true \
  -p wls_lambda:=8000.0 \
  -p wls_sigma:=1.5
```

---

원하면 내가 위 변경이 반영된 **최종 완성본 전체 파일(헤더/CPP/CMake/package/launch)** 을 “그대로 복붙해서 빌드되는 형태”로 한 번에 다시 정리해서 줄게.
(지금은 네가 기존 코드를 이미 빌드 성공했다고 해서, **필요한 변경점만 정확히 추가**하는 형태로 적어줬어.)
