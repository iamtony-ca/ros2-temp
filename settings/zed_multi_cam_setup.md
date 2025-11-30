안녕하세요. ROS 2 전문가로서 답변 드립니다.

ZED ROS 2 Wrapper(`zed-ros2-wrapper`)는 매우 강력한 기능을 제공하지만, 토픽과 데이터 타입이 다양해서 처음 설정할 때 혼동하기 쉽습니다. 질문하신 **Depth 데이터의 타입**과 **RGB 데이터**, 그리고 **토픽명**에 대해 명확하게 정리해 드리겠습니다.

결론부터 말씀드리면, **Depth 데이터는 두 가지 형태(Image와 PointCloud2) 모두 제공**되며, 용도에 따라 선택해서 사용하시면 됩니다.

---

### 1. 데이터 타입 (Data Types)

#### **A. Depth Data**
ZED Wrapper는 Depth 정보를 두 가지 방식으로 발행(Publish)합니다.

1.  **Depth Map (2D 이미지 형태)**
    * **타입:** `sensor_msgs/msg/Image`
    * **설명:** 각 픽셀의 값이 거리(미터 단위, `32FC1` - 32-bit Float)를 나타내는 이미지입니다.
    * **용도:** OpenCV로 처리하거나, 연산량을 줄여야 할 때, 혹은 2D 기반의 장애물 회피에 주로 사용됩니다.
2.  **Point Cloud (3D 점군 형태)**
    * **타입:** `sensor_msgs/msg/PointCloud2`
    * **설명:** X, Y, Z 좌표와 색상(RGB) 정보가 포함된 3D 데이터입니다.
    * **용도:** Nav2의 3D Voxel Layer, 3D 매핑(OctoMap, nvblox), 3D Object Detection에 사용됩니다. 질문하신 내용처럼 **Point Cloud 형태로도 데이터가 나옵니다.**

#### **B. RGB Data**
* **타입:** `sensor_msgs/msg/Image`
* **설명:** 일반적인 컬러 이미지입니다. 보통 `bgra8` 또는 `bgr8` 인코딩을 사용합니다.
* **참고:** `sensor_msgs/msg/CompressedImage` 타입으로도 압축된 토픽이 별도로 발행됩니다.

---

### 2. 표준 토픽명 (Topic Names)

기본 설정(Default) 기준으로, 네임스페이스가 `zed`이고 노드 이름이 `zed_node`일 때의 토픽명입니다. (launch 파일 설정에 따라 접두어 `/zed/zed_node/`는 바뀔 수 있습니다.)

| 데이터 종류 | 토픽명 (Topic Name) | 메시지 타입 (Type) | 비고 |
| :--- | :--- | :--- | :--- |
| **RGB (Raw)** | `~/rgb/image_raw_color` | `sensor_msgs/msg/Image` | 렌즈 왜곡 보정 전 |
| **RGB (Rectified)** | `~/rgb/image_rect_color` | `sensor_msgs/msg/Image` | **(권장)** 왜곡 보정됨. VLA나 CV 모델에 주로 사용 |
| **Depth (Map)** | `~/depth/depth_registered` | `sensor_msgs/msg/Image` | RGB 이미지와 1:1 매칭(Align)된 Depth 맵 |
| **Point Cloud** | `~/point_cloud/cloud_registered` | `sensor_msgs/msg/PointCloud2` | RGB 색상이 입혀진(Registered) 3D 포인트 클라우드 |
| **Confidence** | `~/confidence/confidence_map` | `sensor_msgs/msg/Image` | 각 픽셀의 신뢰도 맵 |

> **전문가 팁:** "Registered"라는 단어는 Depth 데이터가 RGB 카메라의 시점과 픽셀 단위로 정렬(Align)되었다는 뜻입니다. 따라서 `image_rect_color`의 (u, v) 좌표는 `depth_registered`의 (u, v)와 물리적으로 같은 지점을 가리킵니다.

---

### 3. 실무적인 조언 (Expert Advice)

**1. Nav2 통합 시 주의사항 (QoS 설정)**
ZED ROS 2 Wrapper는 기본적으로 센서 데이터에 대해 **QoS Reliability를 `BEST_EFFORT`**로 설정하여 퍼블리시하는 경우가 많습니다.
반면, Nav2나 다른 구독 노드가 `RELIABLE`로 설정되어 있으면 데이터가 연결되지 않을 수 있습니다.
* **해결:** 구독하는 쪽(Nav2 params 등)에서 QoS를 `Best Effort`로 맞추거나, `ros2 topic info -v /토픽명`으로 현재 QoS 설정을 반드시 확인하세요.

**2. 대역폭(Bandwidth) 관리**
`PointCloud2` 데이터는 용량이 매우 큽니다. 만약 네트워크(Wi-Fi)를 통해 다른 PC로 데이터를 전송해야 한다면, `PointCloud2` 대신 `Depth Image`를 보내서 수신 측에서 `depth_image_proc` 패키지를 이용해 PointCloud로 변환하는 것이 대역폭 절약에 유리할 수 있습니다. 로컬(Jetson 등)에서 다 처리한다면 그대로 `PointCloud2`를 쓰셔도 무방합니다.

---

**Next Step:**
현재 사용 중인 `common.yaml` 설정 파일에서 `depth.quality` 파라미터가 어떻게 설정되어 있는지 확인해보시겠습니까? (PERFORMANCE, QUALITY, ULTRA 등에 따라 CPU/GPU 부하와 정밀도가 크게 달라집니다.)  
########################################################################################################################  
########################################################################################################################  
########################################################################################################################  

명확한 목적(Nav2, QR 인식)이 있으시군요. 그렇다면 **가장 적합하고 표준적인 토픽**을 딱 집어서 추천해 드리겠습니다.

ZED 노드의 기본 네임스페이스가 `/zed/zed_node`라고 가정했을 때, 아래 두 토픽을 Subscribe 하시면 됩니다.

### 1\. 요약: 이것만 연결하시면 됩니다

| 목적 | 데이터 종류 | 토픽명 (Topic Name) | 메시지 타입 (Message Type) |
| :--- | :--- | :--- | :--- |
| **Nav2 주행** | **3D Point Cloud** | `/zed/zed_node/point_cloud/cloud_registered` | `sensor_msgs/msg/PointCloud2` |
| **QR 코드 인식** | **Rectified RGB** | `/zed/zed_node/rgb/image_rect_color` | `sensor_msgs/msg/Image` |

-----

### 2\. 상세 설명 및 전문가 팁

#### **A. Nav2 연동용: PointCloud2**

  * **선택 이유:** `cloud_registered`는 RGB 카메라 시점과 정렬된 3D 점군 데이터입니다. Nav2의 Costmap 2D(혹은 3D)에서 장애물 레이어(Obstacle Layer)나 복셀 레이어(Voxel Layer)가 이 데이터를 받아 로봇 주변의 장애물을 마킹합니다.
  * **Nav2 파라미터 설정 예시 (`nav2_params.yaml`):**
    Nav2 설정 파일에서 `observation_sources`를 설정할 때 아래와 같이 입력합니다.
    ```yaml
    local_costmap:
      local_costmap:
        ros__parameters:
          # ... (기타 설정)
          voxel_layer:
            plugin: "nav2_costmap_2d::VoxelLayer"
            enabled: True
            observation_sources: pointcloud_sensor
            pointcloud_sensor:
              topic: /zed/zed_node/point_cloud/cloud_registered
              max_obstacle_height: 2.0
              clearing: True
              marking: True
              data_type: "PointCloud2"
              raytrace_max_range: 3.0
              raytrace_min_range: 0.0
              obstacle_max_range: 2.5
              obstacle_min_range: 0.0
    ```
    > **주의:** ZED의 PointCloud는 데이터 양이 많아 CPU 부하가 클 수 있습니다. `common.yaml` 설정에서 `point_cloud_freq`를 15Hz 이하로 낮추거나 `point_cloud_downsample_factor`를 조절하는 것을 권장합니다.

#### **B. QR 마커 인식용: RGB Image**

  * **선택 이유:** 반드시 `image_rect_color` (Rectified)를 사용해야 합니다.
      * **Raw Image:** 렌즈 왜곡(지구본처럼 둥글게 보이는 현상)이 있어 QR 코드가 평면으로 인식되지 않거나, 인식되더라도 PnP(Perspective-n-Point) 알고리즘으로 거리를 계산할 때 오차가 큽니다.
      * **Rectified Image:** 왜곡이 펴진 이미지이므로 QR 라이브러리(zbar, OpenCV 등)의 인식률이 훨씬 높고 좌표 계산이 정확합니다.
  * **구현 팁:** Python이나 C++ 코드에서 `cv_bridge`를 사용하여 ROS 메시지를 OpenCV 포맷(`bgr8`)으로 변환한 후 QR 디코딩 함수에 넣으시면 됩니다.

-----

### 3\. 꼭 확인해야 할 체크포인트

1.  **네임스페이스 확인:** 실행 중인 터미널에서 `ros2 topic list`를 쳤을 때, 제가 알려드린 `/zed/zed_node/...` 앞에 다른 접두어가 붙어있는지 확인하세요. (예: `/my_robot/zed/zed_node/...`)
2.  **QoS (Quality of Service) 설정:**
      * ZED는 기본적으로 `BEST_EFFORT` (전송 보장 안 함, 속도 우선)로 데이터를 쏠 확률이 높습니다.
      * Nav2나 작성하시는 QR 노드에서 `RELIABLE` (전송 보장)로 받으려고 하면 데이터가 안 들어옵니다.
      * **Nav2:** 위 yaml 설정에는 없지만, `qos_reliability: best_effort`를 추가해야 할 수도 있습니다.
      * **QR 노드:** 코드 작성 시 Subscription QoS를 `rclpy.qos.qos_profile_sensor_data` (이게 Best Effort임)로 설정하세요.

-----

**Next Step:**
"QR 코드의 3D 위치(x, y, z)"까지 알아내서 로봇이 그쪽으로 이동하게 만들고 싶으신가요? 그렇다면 **RGB 이미지 픽셀 좌표(u, v)를 PointCloud 데이터와 매칭하여 실제 좌표를 뽑아내는 방법**을 알려드릴까요?

########################################################################################################################  
########################################################################################################################  
########################################################################################################################  

제트슨 Orin AGX와 쿼드 캡처 카드를 사용하여 4개의 ZED X를 구동하는 하이엔드 시스템을 구축하고 계시는군요.

현재 상황을 분석해보면, \*\*"이미 AMR 시스템이 TF 트리와 로봇 모델(URDF)을 꽉 잡고 있으니, ZED 노드는 조용히 센서 데이터(RGB, Depth)만 뱉어라"\*\*는 요구사항입니다.

기존 `zed_multi_camera.launch.py`는 `robot_state_publisher`를 실행하고 첫 번째 카메라의 TF를 송출하려고 시도하기 때문에 충돌이 발생합니다. 이를 제거하고 **순수하게 센서 노드 4개만 띄우는** 완벽한 수정본을 작성해 드립니다.

### 수정된 Launch 파일: `four_zedx_no_tf.launch.py`

이 코드를 `zed_multi_camera` 패키지나 별도의 커스텀 패키지의 `launch` 폴더에 저장해서 사용하세요.

```python
# Copyright 2025 Stereolabs
# Custom Launch file for 4x ZED X setup on AMR with pre-existing TF/URDF
# Author: Gemini (Based on User Request)

import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    LogInfo
)
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution
)
from launch_ros.actions import ComposableNodeContainer

def parse_array_param(param):
    str_param = param.replace('[', '')
    str_param = str_param.replace(']', '')
    str_param = str_param.replace(' ', '')
    arr = str_param.split(',')
    return arr

def launch_setup(context, *args, **kwargs):
    actions = []

    # 1. Configuration Parsing
    # 기본적으로 4개의 카메라 이름을 설정합니다 (필요시 launch argument로 변경 가능)
    # Default names: zed_front, zed_rear, zed_left, zed_right
    names = LaunchConfiguration('cam_names')
    models = LaunchConfiguration('cam_models')
    ids = LaunchConfiguration('cam_ids') # GMSL Port ID (0, 1, 2, 3)

    names_arr = parse_array_param(names.perform(context))
    models_arr = parse_array_param(models.perform(context))
    ids_arr = parse_array_param(ids.perform(context))

    num_cams = len(names_arr)

    # Validation
    if num_cams != len(models_arr):
        return [LogInfo(msg=TextSubstitution(text='Error: cam_models size mismatch.'))]
    if num_cams != len(ids_arr):
        return [LogInfo(msg=TextSubstitution(text='Error: cam_ids size mismatch.'))]

    # 2. Container Setup
    # 4개의 ZED 노드를 하나의 컨테이너에서 실행하여 리소스 효율성을 높입니다.
    # Orin AGX는 강력하므로 isolated 모드를 사용합니다.
    namespace_val = 'zed_multi'
    container_name = 'zed_multi_container'
    container_exec = 'component_container_isolated'

    info = f'* Starting Container: /{namespace_val}/{container_name}'
    actions.append(LogInfo(msg=TextSubstitution(text=info)))

    zed_container = ComposableNodeContainer(
        name=container_name,
        namespace=namespace_val,
        package='rclcpp_components',
        executable=container_exec,
        arguments=['--use_multi_threaded_executor', '--ros-args', '--log-level', 'info'],
        output='screen',
    )
    actions.append(zed_container)

    # 3. Spawn ZED Nodes (Loop)
    # 핵심 변경 사항: 모든 TF 및 URDF 발행 옵션을 'false'로 강제합니다.
    cam_idx = 0
    
    # zed_wrapper 패키지 경로
    zed_wrapper_pkg = get_package_share_directory('zed_wrapper')

    for name in names_arr:
        model = models_arr[cam_idx]
        cam_id = ids_arr[cam_idx] # GMSL 포트 번호 사용 권장 (0~3)

        info = f'* Starting ZED Node: {name} (Model: {model}, ID: {cam_id}) - TF/URDF DISABLED'
        actions.append(LogInfo(msg=TextSubstitution(text=info)))

        # 각 카메라 노드 실행
        # zed_camera.launch.py를 재사용하되, 인자를 통해 기능을 제한합니다.
        zed_node_launch = IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource([
                zed_wrapper_pkg, '/launch/zed_camera.launch.py'
            ]),
            launch_arguments={
                'camera_name': name,
                'camera_model': model,
                'camera_id': cam_id,
                'node_name': 'zed_node', # 네임스페이스 아래에 생성됨 예: /zed_front/zed_node
                'container_name': container_name,
                'namespace': name, # 중요: 각 카메라는 자신의 이름을 네임스페이스로 가짐
                
                # --- [CRITICAL MODIFICATION] ---
                # AMR의 Cartographer 및 Robot State Publisher와 충돌 방지
                'publish_urdf': 'false',      # URDF 발행 금지 (이미 AMR URDF에 포함됨)
                'publish_tf': 'false',        # odom -> camera_link TF 금지
                'publish_map_tf': 'false',    # map -> odom TF 금지
                'publish_imu_tf': 'false',    # IMU TF 금지
                'enable_gnss': 'false',       # GNSS 비활성화
                'publish_svo_clock': 'false',
                # -------------------------------
            }.items()
        )
        actions.append(zed_node_launch)
        cam_idx += 1

    return actions

def generate_launch_description():
    return LaunchDescription(
        [
            # 기본값 설정: 4개의 ZED X 카메라, GMSL 포트 0,1,2,3 순서
            DeclareLaunchArgument(
                'cam_names',
                default_value='[zed_front,zed_rear,zed_left,zed_right]',
                description='Names of the 4 cameras'),
            DeclareLaunchArgument(
                'cam_models',
                default_value='[zedx,zedx,zedx,zedx]',
                description='Models of the 4 cameras'),
            DeclareLaunchArgument(
                'cam_ids',
                default_value='[0,1,2,3]',
                description='GMSL Port IDs for the cameras'),
            OpaqueFunction(function=launch_setup)
        ]
    )
```

-----

### 수정 포인트 및 전문가 코멘트

1.  **URDF/TF 완전 차단 (`CRITICAL MODIFICATION` 부분)**

      * 기존 `zed_multi_camera.launch.py`에 있던 `robot_state_publisher` 노드 실행 부분을 **삭제**했습니다.
      * `zed_camera.launch.py`를 호출할 때 `publish_urdf`, `publish_tf`, `publish_map_tf`, `publish_imu_tf`를 모두 `'false'`로 강제 할당했습니다.
      * 이로써 `/tf` 토픽에 ZED 관련 프레임이 중복 발행되어 로봇의 TF 트리가 튀거나(jerk) 경고가 뜨는 현상을 막았습니다. (단, AMR URDF 상의 `camera_link` 이름과 ZED 노드가 사용하는 프레임 이름이 일치해야 합니다.)

2.  **Container 최적화**

      * `component_container_isolated`를 사용했습니다. ZED X 4대는 데이터 양이 엄청나기 때문에, 각 노드가 독립적인 프로세스처럼 동작하면서도 컴포넌트 컨테이너의 이점(토픽 통신 효율)을 가질 수 있도록 설정했습니다.

3.  **Launch Argument 간소화**

      * Serial Number 대신 \*\*`cam_ids` (GMSL 포트 번호 0, 1, 2, 3)\*\*를 기본으로 사용하도록 변경했습니다. 쿼드 캡처 카드를 쓸 때는 시리얼 번호보다 포트 번호로 매핑하는 것이 물리적 위치(앞/뒤/좌/우)를 고정하는 데 훨씬 유리합니다.
      * 기본값으로 `zed_front`, `zed_rear`, `zed_left`, `zed_right`를 넣어두었으니, 별도 인자 없이 바로 실행하면 됩니다.

### 실행 방법

터미널에서 아래와 같이 실행하시면 됩니다. (파일명을 `four_zedx.launch.py`로 저장했다고 가정)

```bash
ros2 launch zed_multi_camera four_zedx.launch.py
```

### 추가 확인 사항 (매우 중요)

**AMR의 URDF**에 정의된 카메라 링크 이름과 ZED 노드가 사용하는 프레임 이름이 맞아야 합니다.
ZED ROS2 Wrapper는 기본적으로 다음과 같은 프레임 이름을 사용합니다:

  * `<camera_name>_camera_center`
  * `<camera_name>_left_camera_frame`
  * `<camera_name>_left_camera_optical_frame` (여기가 실제 데이터가 나오는 좌표계)

만약 AMR URDF에서 링크 이름을 다르게 정의했다면(예: `front_cam_link`), `common.yaml` 파일이나 launch file의 param override를 통해 `pos_tracking.base_frame`이나 `general.camera_link` 파라미터를 맞춰주셔야 Nav2에서 TF 에러(`Transform error: "zed_front_left_camera_optical_frame" passed to lookupTransform...`)가 발생하지 않습니다.

########################################################################################################################  
########################################################################################################################  
########################################################################################################################  

제공해주신 TF 트리 이미지와 현재 상황을 완벽하게 이해했습니다.

**상황 분석:**

1.  **TF 트리:** `base_link` -\> `front_mount` -\> `zed_front_camera_center` ... 순으로 이미 TF 트리가 잘 잡혀 있습니다.
2.  **요구 사항:** ZED 노드는 **TF(`tf`)와 로봇 모델(`robot_description`)을 절대 퍼블리시하면 안 됩니다.** 오직 센서 데이터(Image, Depth/PointCloud)만 뱉어내고, 그 데이터의 `header.frame_id`가 기존 TF 트리의 링크 이름과 일치하기만 하면 됩니다.

기존 `zed_multi_camera.launch.py`는 `robot_state_publisher`를 포함하고 있어 사용자 환경과 충돌합니다. 이를 제거하고, 순수하게 **"센서 드라이버 4개만 켜는"** 깔끔한 Launch 파일을 작성해 드립니다.

### 수정된 Launch 파일: `zed_quad_cam_sensor_only.launch.py`

이 파일을 `zed_wrapper` 패키지나 별도의 워크스페이스 내 launch 폴더에 저장하여 사용하세요.

```python
# Copyright 2025 Stereolabs
# Custom Launch file for 4x ZED X setup on AMR with pre-existing TF/URDF
# Purpose: Launch drivers ONLY, without publishing TF or URDF.

import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    LogInfo
)
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution
)
from launch_ros.actions import ComposableNodeContainer

def parse_array_param(param):
    str_param = param.replace('[', '')
    str_param = str_param.replace(']', '')
    str_param = str_param.replace(' ', '')
    arr = str_param.split(',')
    return arr

def launch_setup(context, *args, **kwargs):
    actions = []

    # 1. 설정값 파싱
    # 기본적으로 4개의 카메라 이름을 설정 (이미지상의 네이밍 규칙 준수)
    # TF 트리에 있는 이름인 'zed_front', 'zed_rear' 등을 사용해야 합니다.
    names = LaunchConfiguration('cam_names')
    models = LaunchConfiguration('cam_models')
    ids = LaunchConfiguration('cam_ids') # GMSL Port ID (0, 1, 2, 3)

    names_arr = parse_array_param(names.perform(context))
    models_arr = parse_array_param(models.perform(context))
    ids_arr = parse_array_param(ids.perform(context))

    num_cams = len(names_arr)

    # 2. 유효성 검사
    if num_cams != len(models_arr):
        return [LogInfo(msg=TextSubstitution(text='Error: cam_models size mismatch.'))]
    if num_cams != len(ids_arr):
        return [LogInfo(msg=TextSubstitution(text='Error: cam_ids size mismatch.'))]

    # 3. 컨테이너 설정 (Jetson Orin AGX의 성능을 위해 Isolated 모드 사용)
    namespace_val = 'zed_multi'
    container_name = 'zed_multi_container'
    container_exec = 'component_container_isolated'

    info = f'* Starting Container: /{namespace_val}/{container_name}'
    actions.append(LogInfo(msg=TextSubstitution(text=info)))

    zed_container = ComposableNodeContainer(
        name=container_name,
        namespace=namespace_val,
        package='rclcpp_components',
        executable=container_exec,
        arguments=['--use_multi_threaded_executor', '--ros-args', '--log-level', 'info'],
        output='screen',
    )
    actions.append(zed_container)

    # 4. ZED 노드 실행 (Loop)
    zed_wrapper_pkg = get_package_share_directory('zed_wrapper')

    for idx, name in enumerate(names_arr):
        model = models_arr[idx]
        cam_id = ids_arr[idx]

        info = f'* Starting ZED Node: {name} (Model: {model}, ID: {cam_id}) - SENSOR ONLY MODE'
        actions.append(LogInfo(msg=TextSubstitution(text=info)))

        # zed_camera.launch.py를 호출하되, TF/URDF 관련 기능을 모두 끕니다.
        zed_node_launch = IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource([
                zed_wrapper_pkg, '/launch/zed_camera.launch.py'
            ]),
            launch_arguments={
                'camera_name': name,       # 예: zed_front (이 이름이 토픽명과 frame_id의 prefix가 됨)
                'camera_model': model,     # 예: zedx
                'camera_id': cam_id,       # 예: 0
                'node_name': 'zed_node',
                'container_name': container_name,
                'namespace': name,         # 네임스페이스 분리 (/zed_front/zed_node/...)
                
                # --- [핵심 수정 사항: TF 및 URDF 퍼블리시 차단] ---
                'publish_urdf': 'false',      # 로봇 모델 퍼블리시 안 함 (이미 AMR에 있음)
                'publish_tf': 'false',        # odom -> base_link TF 안 함
                'publish_map_tf': 'false',    # map -> odom TF 안 함
                'publish_imu_tf': 'false',    # imu TF 안 함
                'enable_gnss': 'false',       # GNSS 사용 안 함
                'publish_svo_clock': 'false',
                # -----------------------------------------------
            }.items()
        )
        actions.append(zed_node_launch)

    return actions

def generate_launch_description():
    return LaunchDescription(
        [
            # TF 트리 이미지에 맞춰 이름 기본값 설정 (zed_front, zed_rear, + 추가될 2개)
            # 추가될 카메라 이름은 'zed_left', 'zed_right'라고 가정했습니다. 필요시 수정하세요.
            DeclareLaunchArgument(
                'cam_names',
                default_value='[zed_front,zed_rear,zed_left,zed_right]',
                description='Names of the 4 cameras (Must match URDF prefixes)'),
            DeclareLaunchArgument(
                'cam_models',
                default_value='[zedx,zedx,zedx,zedx]',
                description='Models of the 4 cameras'),
            DeclareLaunchArgument(
                'cam_ids',
                default_value='[0,1,2,3]',
                description='GMSL Port IDs for the cameras'),
            OpaqueFunction(function=launch_setup)
        ]
    )
```

### 

### 적용 시 주의사항 (Expert Check)

1.  **이름 일치 (가장 중요):**

      * 제공해주신 이미지에서 `zed_front_camera_center`와 같은 링크 이름이 보입니다.
      * ZED Wrapper는 `camera_name` 파라미터를 기반으로 `[camera_name]_camera_center` 등의 프레임 ID를 생성하여 메시지 헤더에 찍습니다.
      * 따라서 Launch 파일의 `cam_names` 리스트(`zed_front`, `zed_rear` 등)가 **AMR URDF에 정의된 이름의 접두사(prefix)와 정확히 일치**해야 합니다.
      * 만약 URDF에는 `front_cam_link`라고 되어 있는데 여기서 `zed_front`라고 실행하면, 센서 데이터는 `zed_front_...` 프레임으로 나오지만 TF 트리에는 그 프레임이 없어서 `Transform Error`가 발생합니다.

2.  **새로 추가할 카메라 2개:**

      * 아직 URDF에 입력하지 않은 2개의 카메라도 `zed_left`, `zed_right` (혹은 사용하실 이름)와 같은 패턴으로 URDF를 작성하신 후, 이 Launch 파일의 `cam_names` 목록과 맞춰주시면 됩니다.

3.  **실행 명령어:**

    ```bash
    ros2 launch [패키지명] zed_quad_cam_sensor_only.launch.py
    ```

    실행 후 `ros2 topic list`를 확인하면 `/zed_front/zed_node/point_cloud/cloud_registered` 등의 토픽이 보이고, `ros2 run tf2_tools view_frames`로 확인했을 때 TF 트리가 꼬이지 않고 기존 AMR 트리와 깔끔하게 연결되어 있어야(센서 데이터의 frame\_id가 트리에 존재해야) 성공입니다.

########################################################################################################################  
########################################################################################################################  
########################################################################################################################  

제트슨 Orin AGX에서 4대의 ZED X를 구동하는 환경이라면 \*\*"리소스 최적화"\*\*와 \*\*"TF 충돌 방지"\*\*가 핵심입니다.

요청하신 대로 **AMR이 메인 Odometry와 TF를 담당**하고, ZED는 오직 **Visual Sensor(RGB + PointCloud)** 역할만 수행하도록 설정을 완벽하게 다이어트했습니다.

아래 두 파일을 복사해서 덮어쓰시면 됩니다.

-----

### 1\. `common_stereo.yaml` (핵심 수정됨)

**주요 변경 사항:**

  * **Positional Tracking (`pos_tracking`):** `false`로 변경. (AMR이 위치를 잡으므로 ZED의 VO 연산을 꺼서 GPU 자원을 대폭 아낌)
  * **Sensors (`imu`):** `false`로 변경. (AMR의 IMU와 충돌 방지 및 대역폭 절약)
  * **Depth Mode:** `PERFORMANCE`로 변경. (4대 동시 구동 시 `NEURAL` 모드는 딜레이가 발생할 수 있음. Nav2용 장애물 감지에는 PERFORMANCE로도 충분함)
  * **TF Publishing:** 모든 TF 발행 옵션 `false`.

<!-- end list -->

```yaml
# config/common_stereo.yaml
# Common parameters to Stereolabs ZED Stereo cameras
# MODIFIED FOR: 4-Camera Sensor-Only Setup (No TF, No Odometry, No IMU)

---
/**:
    ros__parameters:
        use_sim_time: false

        simulation:
            sim_enabled: false
            sim_address: '127.0.0.1'
            sim_port: 30000

        svo:
            use_svo_timestamps: true
            publish_svo_clock: false
            svo_loop: false
            svo_realtime: false
            play_from_frame: 0
            replay_rate: 1.0

        general:
            camera_timeout_sec: 5
            camera_max_reconnect: 5
            camera_flip: false
            self_calib: true
            serial_number: 0
            pub_resolution: 'CUSTOM' 
            pub_downscale_factor: 2.0 # 성능을 위해 다운스케일 유지 (Pointcloud 부하 감소)
            pub_frame_rate: 15.0 # [중요] 4대 구동 시 15Hz 권장. Nav2에는 충분함.
            enable_image_validity_check: 1
            gpu_id: -1
            optional_opencv_calibration_file: ''
            async_image_retrieval: false
            publish_status: true

        video:
            saturation: 4
            sharpness: 4
            gamma: 8
            auto_exposure_gain: true
            exposure: 80
            gain: 80
            auto_whitebalance: true
            whitebalance_temperature: 42
            enable_24bit_output: false
            publish_rgb: true # [QR 인식용] True 유지
            publish_left_right: false
            publish_raw: false
            publish_gray: false
            publish_stereo: false

        sensors:
            # [수정됨] AMR에서 IMU를 처리하므로 ZED IMU는 끕니다.
            publish_imu_tf: false 
            sensors_image_sync: false
            sensors_pub_rate: 100.
            publish_imu: false 
            publish_imu_raw: false
            publish_cam_imu_transf: false
            publish_mag: false
            publish_baro: false
            publish_temp: false

        region_of_interest:
            automatic_roi: false
            depth_far_threshold_meters: 2.5
            image_height_ratio_cutoff: 0.5
            apply_to_depth: true
            apply_to_positional_tracking: false # Tracking 꺼짐
            apply_to_object_detection: false
            apply_to_body_tracking: false
            apply_to_spatial_mapping: false
            publish_roi_mask: false

        depth:
            # [수정됨] 4대 동시 구동 부하를 줄이기 위해 NEURAL -> PERFORMANCE 권장
            # Nav2 장애물 인식용으로는 PERFORMANCE도 충분히 정확합니다.
            depth_mode: 'PERFORMANCE' 
            depth_stabilization: 0 # Tracking을 끄면 0 권장
            openni_depth_mode: false
            point_cloud_freq: 10.0 # [Nav2용] 10Hz 적절함
            point_cloud_res: 'COMPACT' # 대역폭 절약
            depth_confidence: 95
            depth_texture_conf: 100
            remove_saturated_areas: true
            publish_depth_map: true
            publish_depth_info: false
            publish_point_cloud: true # [Nav2용] True 유지 (cloud_registered)
            publish_depth_confidence: false
            publish_disparity: false

        pos_tracking:
            # [핵심 수정] AMR Cartographer가 위치를 잡으므로 ZED Odometry는 끕니다.
            # 이를 통해 GPU 자원을 획기적으로 절약할 수 있습니다.
            pos_tracking_enabled: false 
            
            pos_tracking_mode: 'GEN_2'
            imu_fusion: false
            publish_tf: false # Launch 파일에서 끄지만, 여기서도 확실히 끔
            publish_map_tf: false # AMR이 map->odom 담당
            map_frame: 'map'
            odometry_frame: 'odom'
            area_memory: false
            save_area_memory_on_closing: false
            reset_odom_with_loop_closure: false
            publish_3d_landmarks: false
            publish_lm_skip_frame: 5
            depth_min_range: 0.0
            set_as_static: false
            set_gravity_as_origin: false
            floor_alignment: false
            initial_base_pose: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            path_pub_rate: 2.0
            path_max_count: -1
            two_d_mode: false
            fixed_z_value: 0.0
            transform_time_offset: 0.0
            reset_pose_with_svo_loop: false
            publish_odom_pose: false
            publish_pose_cov: false
            publish_cam_path: false

        gnss_fusion:
            gnss_fusion_enabled: false # 사용 안 함

        mapping:
            mapping_enabled: false # 사용 안 함
            resolution: 0.05
            max_mapping_range: 5.0
            fused_pointcloud_freq: 1.0
            clicked_point_topic: '/clicked_point'
            pd_max_distance_threshold: 0.15
            pd_normal_similarity_threshold: 15.0
            publish_det_plane: false

        object_detection:
            od_enabled: false # 사용 안 함
            enable_tracking: false
            detection_model: 'MULTI_CLASS_BOX_FAST'
            max_range: 20.0
            filtering_mode: 'NMS3D'
            prediction_timeout: 2.0
            allow_reduced_precision_inference: false

        body_tracking:
            bt_enabled: false # 사용 안 함
            model: 'HUMAN_BODY_MEDIUM'
            body_format: 'BODY_38'
            allow_reduced_precision_inference: false
            max_range: 15.0
            body_kp_selection: 'FULL'
            enable_body_fitting: false
            enable_tracking: false
            prediction_timeout_s: 0.5
            confidence_threshold: 50.0
            minimum_keypoints_threshold: 5

        stream_server:
            stream_enabled: false

        advanced:
            thread_sched_policy: 'SCHED_BATCH'
            thread_grab_priority: 50
            thread_sensor_priority: 70
            thread_pointcloud_priority: 60

        debug:
            sdk_verbose: 1
            sdk_verbose_log_file: ''
            use_pub_timestamps: false
            debug_common: false
            debug_sim: false
            debug_video_depth: false
            debug_camera_controls: false
            debug_point_cloud: false
            debug_positional_tracking: false
            debug_gnss: false
            debug_sensors: false
            debug_mapping: false
            debug_terrain_mapping: false
            debug_object_detection: false
            debug_body_tracking: false
            debug_roi: false
            debug_streaming: false
            debug_advanced: false
            debug_nitros: false
            disable_nitros: false
```

-----

### 2\. `zedx.yaml` (최적화됨)

**주요 변경 사항:**

  * **Frame Rate:** `grab_frame_rate`를 15로 낮추는 것을 고려할 수 있으나, 부드러운 Auto Exposure 제어를 위해 `30`을 유지했습니다. (common.yaml에서 pub rate가 15이므로 네트워크 부하는 없음)
  * **Min Depth:** Nav2용 장애물 감지를 위해 최소 거리를 0.3m(30cm) 정도로 조정하는 것이 일반적이나, 기본값(0.01)을 두되 노이즈가 심하면 Nav2 config에서 `min_obstacle_height` 등으로 제어하세요.

<!-- end list -->

```yaml
# config/zedx.yaml
# Parameters for Stereolabs ZED X camera
---
/**:
    ros__parameters:
        general:
          camera_model: 'zedx'
          camera_name: 'zedx' # overwritten by launch file
          grab_resolution: 'HD1200' # ZED X Native resolution
          grab_frame_rate: 30 # SDK 내부 처리 속도. 4대 동시 구동 시 GPU 부하가 심하면 15로 낮추세요.

        video:
          exposure_time: 16000
          auto_exposure_time_range_min: 28
          auto_exposure_time_range_max: 30000
          exposure_compensation: 50
          analog_gain: 1255
          auto_analog_gain_range_min: 1000
          auto_analog_gain_range_max: 16000
          digital_gain: 1
          auto_digital_gain_range_min: 1
          auto_digital_gain_range_max: 256
          denoising: 50

        depth:
          min_depth: 0.2 # [권장] 너무 가까운 거리는 노이즈가 심하므로 20~30cm로 설정 추천
          max_depth: 10.0 # [권장] 실내 AMR 주행이라면 10m 이상은 불필요한 연산일 수 있음.
```

### 

### 전문가 코멘트 (적용 후 확인 사항)

1.  **Depth Mode:** `common_stereo.yaml`에서 `depth_mode`를 `PERFORMANCE`로 설정했습니다. 만약 4대를 켰는데도 Orin AGX의 GPU 사용량(`jtop`으로 확인)이 70% 미만이라면, `NEURAL`이나 `NEURAL_LIGHT`로 올려서 Depth 품질을 높이셔도 됩니다. 하지만 `PERFORMANCE`가 Nav2 주행에는 가장 빠르고 안정적입니다.
2.  **IMU & TF:** 이 설정 파일들은 **ZED가 스스로 위치 추정(Odometry)을 하지 않도록** 설정되었습니다 (`pos_tracking_enabled: false`). 따라서 `/tf` 토픽 충돌 문제는 원천적으로 차단됩니다.
3.  **데이터 확인:** 설정 적용 후 `ros2 topic hz /zed_front/zed_node/point_cloud/cloud_registered`를 입력했을 때 약 10Hz 근처가 나오는지 확인하십시오.

########################################################################################################################  
########################################################################################################################  
########################################################################################################################  

네, 매우 중요한 포인트입니다. 천장에 매달린 조명이나 문틀처럼 로봇이 지나갈 수 있는 공간인데도, 장애물로 인식해서 회피하려고 하면 주행 효율이 떨어지니까요.

결론부터 말씀드리면, **ZED ROS 2 Wrapper(`common.yaml`) 자체에는 "Height(Z축 높이)"로 데이터를 자르는 기능이 없습니다.** 카메라는 자신을 기준으로 앞쪽(Z축) 거리만 알 뿐, 바닥에서 자신이 얼마나 떠 있는지(TF 정보)를 이용해 데이터를 잘라내지는 않기 때문입니다.

대신, 이 기능은 **Nav2 (Navigation Stack)** 설정에서 처리하는 것이 \*\*표준(Standard)\*\*이자 정석입니다.

-----

### 해결 방법: Nav2 Costmap 파라미터 수정

Nav2는 들어오는 PointCloud 데이터를 로봇의 `base_link`나 `map` 좌표계로 변환한 뒤, 설정된 높이 범위 밖의 점들은 **장애물 지도(Costmap)에 반영하지 않고 무시**하는 기능을 가지고 있습니다.

`nav2_params.yaml` 파일(보통 `nav2_bringup`이나 커스텀 패키지의 `params` 폴더에 있음)을 열어서 `local_costmap`과 `global_costmap` 설정을 다음과 같이 수정하세요.

#### **수정해야 할 파일: `nav2_params.yaml`**

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      # ... (기타 설정)
      plugins: ["voxel_layer", "inflation_layer"] # 또는 obstacle_layer
      
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0  # [중요] 이 높이보다 높은 데이터는 무시 (Costmap에 안 찍힘)
        unknown_threshold: 15
        mark_threshold: 0
        observation_sources: zed_source
        
        zed_source:
          topic: /zed_front/zed_node/point_cloud/cloud_registered # 토픽명 확인!
          max_obstacle_height: 1.8  # [핵심 1] 로봇 높이 (m) - 이보다 높은 점은 무시함
          min_obstacle_height: 0.1  # [핵심 2] 바닥 노이즈 제거 (m) - 바닥이나 카펫을 장애물로 인식 안 하게 함
          obstacle_max_range: 2.5   # 센서 인식 거리 (m)
          obstacle_min_range: 0.0
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          clearing: True
          marking: True
          data_type: "PointCloud2"
```

### 파라미터 상세 설명 (Expert Insight)

1.  **`min_obstacle_height: 0.1` (최소 높이)**

      * **용도:** 로봇의 바퀴가 닿는 바닥면(0.0m)이나 약간의 카펫, 전선 몰딩 등을 장애물로 인식하지 않게 합니다.
      * **추천:** `0.0`으로 하면 바닥의 미세한 오차가 전부 장애물로 찍혀서 로봇이 못 움직일 수 있습니다. 보통 `0.05` \~ `0.1` (5\~10cm) 정도로 설정합니다.

2.  **`max_obstacle_height: 1.8` (최대 높이)**

      * **용도:** 질문하신 내용입니다. 이 값보다 \*\*Z축 좌표가 높은 점들(Point)\*\*은 Costmap에 마킹(Marking)되지 않고 **투명 취급**됩니다.
      * **설정값:** `로봇의 실제 높이 + 약간의 여유(5cm)` 정도로 설정하면 됩니다. 예를 들어 로봇 키가 1.5m라면 1.6m 정도로 설정하세요.

### (심화) 만약 토픽 자체를 필터링하고 싶다면?

Nav2와 상관없이, 다른 노드(예: 별도의 충돌 방지 알고리즘)에서도 높이가 잘린 데이터가 필요하다면 `pcl_ros` 패키지의 **CropBox 필터**를 사용해야 합니다. 하지만 이는 CPU를 추가로 소모하므로 **Nav2 주행이 목적이라면 위의 방법(Costmap 파라미터 수정)만 적용하는 것이 가장 효율적입니다.**

-----

**Next Step:**
현재 로봇(`base_link`)의 바닥에서 ZED 카메라 렌즈까지의 높이가 대략 몇 cm 정도 되시나요? 그 높이에 따라 `min_obstacle_height`를 얼마나 공격적으로 잡을 수 있을지 판단할 수 있습니다. (카메라가 낮게 달릴수록 바닥 노이즈가 많이 들어옵니다.)


```yaml

local_costmap:
  local_costmap:
    ros__parameters:
      # ...
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        observation_sources: pointcloud_sensor
        pointcloud_sensor:
          topic: /zed_front/zed_node/point_cloud/cloud_registered
          data_type: "PointCloud2"
          
          # [핵심 설정] 로봇 바닥(0.0) ~ 로봇 높이(예: 0.5m) 사이만 장애물로 인정
          min_obstacle_height: 0.05  # 바닥 노이즈 제거를 위해 5cm 이상부터 인식
          max_obstacle_height: 0.60  # 로봇 키가 50cm라면, 여유있게 60cm 이상은 무시
          
          # [중요] 레이트레이싱(장애물 삭제) 범위는 인식 범위보다 약간 높게 잡는게 좋음
          raytrace_min_range: 0.0
          raytrace_max_range: 5.0
          obstacle_max_range: 4.5
          obstacle_min_range: 0.0



region_of_interest:
            # ...
            # 1.0이면 전체 이미지 사용, 0.5면 이미지의 아래쪽 절반만 사용 (하늘/천장 제거)
            image_height_ratio_cutoff: 0.6 # 예: 아래쪽 60%만 사용하고 위 40%는 버림
            
            apply_to_depth: true # Depth 데이터에도 적용
            apply_to_positional_tracking: false
            # ...          

```
방법 2. ZED Wrapper 설정으로 자르기 (대역폭 절약용, 약식)
ZED Wrapper(common.yaml)에는 "지면으로부터 몇 미터"라는 3D 필터는 없지만, 이미지의 윗부분을 날려버리는 2D ROI 설정은 있습니다. 만약 카메라가 바닥과 평행하게 설치되어 있다면, 이미지 윗부분을 자르는 것이 곧 높은 물체를 무시하는 효과를 냅니다.

common_stereo.yaml 파일을 엽니다.

```YAML
        region_of_interest:
            # ...
            # 1.0이면 전체 이미지 사용, 0.5면 이미지의 아래쪽 절반만 사용 (하늘/천장 제거)
            image_height_ratio_cutoff: 0.6 # 예: 아래쪽 60%만 사용하고 위 40%는 버림
            
            apply_to_depth: true # Depth 데이터에도 적용
            apply_to_positional_tracking: false
            # ...
```

설정 팁: 로봇의 카메라 화면을 보면서, 천장이나 너무 높은 물체가 안 보일 때까지 0.6, 0.7 등으로 값을 조절해보세요.

장점: 데이터 생성 단계에서 잘라버리므로 CPU/GPU 부하가 줄고 전송 데이터 양도 줍니다.

단점: 카메라가 위쪽으로 기울어져 있거나(Tilt), 경사로를 올라갈 때는 앞에 있는 낮은 물체도 '이미지 상 위쪽'에 있어서 잘려나갈 위험이 있습니다.



########################################################################################################################  
########################################################################################################################  
########################################################################################################################  
