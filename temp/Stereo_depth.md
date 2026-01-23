좋아. 아래는 ROS2(rclpy) + OpenCV StereoSGBM로 rectified left/right 이미지를 받아서 disparity(시차) + depth(미터) 를 10Hz 이상으로 퍼블리시하는 Python 노드 예시야.

입력: left/image_rect, right/image_rect, left/camera_info, right/camera_info

출력:

~/disparity : stereo_msgs/DisparityImage (권장)

~/depth : sensor_msgs/Image (32FC1, meters, invalid=NaN)


설계: 이미지 pair는 계속 최신으로 저장, timer에서 지정한 output_hz(기본 10Hz) 로만 SGBM 계산 → CPU 부하/지터 관리가 쉬움



---

stereo_sgbm_depth_node.py

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


---

실행 예시

토픽 이름이 다르면 파라미터로 바꿔서 실행하면 돼.

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

10Hz가 빡빡하면(CPU 여유 없으면) 이렇게:

scale=0.8 또는 0.5 로 내리면 성능이 확 좋아져요.


-p scale:=0.5


---

바로 튜닝 추천 (SVGA / depth 10Hz 목표)

1차 목표(10Hz 달성): scale=0.5 + num_disparities=96~128 + block_size=5

품질 올리기: scale=0.8 또는 1.0으로 올리되 num_disparities는 필요한 만큼만 유지

“가까운 물체 위주”면 num_disparities를 줄이는 게 가장 큰 속도 이득이에요.



---

원하면 다음도 바로 붙여줄게:

1. PointCloud2 생성(깊이 + 카메라 모델)


2. (선택) WLS 후처리 옵션까지 포함한 버전(품질↑, CPU↑)


3. RViz2에서 depth 확인용 컬러화(Colormap) 이미지 토픽 추가



지금 카메라 토픽 프레임 이름(예: left_camera_frame)이랑, baseline이 대략 몇 cm인지(알면) 알려주면 파라미터도 더 딱 맞게 잡아줄 수 있어요.
