네, **Computer Vision** 및 **ROS 2 Jazzy** 전문가로서 답변드립니다.

Stereo RGB 카메라를 Nav2 Costmap과 연동하는 것은 매우 표준적이지만, 2D LiDAR와 달리 **"Depth 생성 (Disparity Map)  3D PointCloud 변환"**이라는 전처리 과정이 필수적입니다.

ROS 2 Jazzy 환경에서 이를 구현하기 위한 아키텍처와 구체적인 설정 방법을 정리해 드립니다.

---

### 1. 전체 파이프라인 (Data Flow)

Nav2는 기본적으로 RGB 이미지를 직접 해석하지 않습니다. 따라서 아래와 같은 데이터 변환 과정이 필요합니다.

1. **Raw Images:** 좌/우 카메라 이미지 (`sensor_msgs/Image`) + 카메라 정보 (`sensor_msgs/CameraInfo`)
2. **Rectification & Stereo Matching:** 렌즈 왜곡 보정 및 시차(Disparity) 계산
3. **3D Projection:** 시차를 3D 좌표로 변환  **`sensor_msgs/PointCloud2`**
4. **Nav2 Costmap:** PointCloud2 데이터를 Voxel Layer 또는 Obstacle Layer에 입력

---

### 2. 구현 방법 A: 표준 패키지 사용 (`stereo_image_proc`)

가장 정석적이고 별도의 GPU 의존성이 낮은 방법은 ROS 2의 표준 패키지인 `stereo_image_proc`을 사용하는 것입니다.

#### 1) 패키지 설치 (Jazzy)

```bash
sudo apt update
sudo apt install ros-jazzy-stereo-image-proc

```

#### 2) `stereo_image_proc` 실행

이 노드는 좌/우 카메라의 `image_raw`와 `camera_info`를 구독하여 `points2` (PointCloud2) 토픽을 발행합니다.

**Launch 파일 예시 (Composability 활용):**
최신 ROS 2 스타일인 `ComposableNode`를 사용하여 오버헤드를 줄이는 것을 권장합니다.

```python
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='stereo_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::DisparityNode',
                    name='disparity_node',
                    remappings=[
                        ('left/image_rect', '/camera/left/image_rect'),
                        ('left/camera_info', '/camera/left/camera_info'),
                        ('right/image_rect', '/camera/right/image_rect'),
                        ('right/camera_info', '/camera/right/camera_info')
                    ]
                ),
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::PointCloudNode',
                    name='point_cloud_node',
                    remappings=[
                        ('left/camera_info', '/camera/left/camera_info'),
                        ('right/camera_info', '/camera/right/camera_info'),
                        ('left/image_rect_color', '/camera/left/image_rect'), # 혹은 color 사용
                        ('disparity', '/disparity'),
                        ('points2', '/camera/depth/points') # Nav2로 보낼 최종 토픽
                    ]
                )
            ],
            output='screen',
        )
    ])

```

* **주의:** 카메라 드라이버가 이미 Rectified Image(`image_rect`)를 주지 않는다면, `stereo_image_proc` 전체를 실행하여 Rectification 과정도 포함해야 합니다.

---

### 3. 구현 방법 B: NVIDIA GPU 가속 (Isaac ROS & nvblox)

사용자분의 관심사(nvblox, Isaac Sim)를 고려할 때, 만약 **Jetson Orin**이나 **NVIDIA GPU**가 있는 환경이라면 `stereo_image_proc` 대신 **Isaac ROS**를 강력히 추천합니다. CPU 기반 매칭은 노이즈가 심하고 연산량이 많습니다.

* **`isaac_ros_stereo_depth`**: GPU 가속을 통해 고품질의 Disparity 및 PointCloud 생성.
* **`nvblox`**: PointCloud를 거치지 않고 직접 3D reconstruction을 수행하여 Nav2용 2D Costmap 또는 3D Grid를 바로 생성 가능. (이것이 가장 모던한 접근입니다.)

---

### 4. Nav2 Costmap 설정 (`nav2_params.yaml`)

생성된 `PointCloud2` 데이터를 Nav2가 인식하도록 설정합니다. Mobile Manipulator라면 3D 장애물을 인식해야 하므로 `VoxelLayer`를 사용하는 것이 좋습니다.

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3.0
      height: 3.0
      resolution: 0.05
      
      plugins: ["voxel_layer", "inflation_layer"]

      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        publish_voxel_map: true
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: stereo_cam
        
        # 핵심 설정 부분
        stereo_cam:
          topic: /camera/depth/points  # 위에서 생성한 PointCloud 토픽
          sensor_frame: camera_optical_frame # PointCloud의 Frame ID (TF 필수!)
          max_obstacle_height: 2.0
          min_obstacle_height: 0.1     # 바닥 노이즈 제거
          clearing: true               # 빈 공간 인식 시 장애물 제거
          marking: true                # 장애물 추가
          data_type: "PointCloud2"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

```

### 5. 핵심 고려 사항 (Troubleshooting)

1. **TF (Transform) 연결:**
* 반드시 `base_link`  `camera_link`  `camera_optical_frame`으로 이어지는 TF 트리가 `robot_state_publisher` 등을 통해 발행되고 있어야 합니다.
* 카메라 좌표계는 보통 Z축이 전방(Optical)이므로, URDF 설정 시 주의해야 합니다.


2. **Textureless Region (흰 벽 문제):**
*
* Stereo 카메라는 특징점(Texture)이 없는 흰 벽이나 유리창에서는 깊이 추정에 실패합니다. 이 경우 PointCloud가 생성되지 않아 Costmap에 장애물이 표시되지 않을 수 있습니다.
* **해결:** Active Stereo(패턴 프로젝터)가 있는 카메라를 쓰거나, `filters` 패키지를 써서 노이즈를 걸러내야 합니다.


3. **리소스 관리:**
* `PointCloud2`는 데이터 양이 방대합니다. `VoxelLayer`의 `raytrace_max_range`를 너무 길게 잡으면 CPU 부하가 급증합니다. 로컬 회피용으로는 3~4m 정도로 제한하는 것이 효율적입니다.



### 요약

1. **Stereo Matcher** (`stereo_image_proc` 또는 `isaac_ros`)를 실행하여 `/points2` 토픽을 생성한다.
2. Nav2의 **local_costmap** params에 `VoxelLayer`를 추가하고 해당 토픽을 구독시킨다.
3. **TF**가 올바른지 확인한다.

**다음 단계로 무엇을 도와드릴까요?**
`stereo_image_proc`을 테스트할 수 있는 간단한 docker-compose 예시를 만들어 드릴까요, 아니면 Isaac ROS/nvblox를 활용한 심화 설정에 대해 더 설명해 드릴까요?
#########################  
##############################  
#################################  
네, **학습과 연습**이 목적이시라면 그 의도에 맞춰서 정확히 해결해 드려야죠! ZED X의 훌륭한 하드웨어를 두고 왜 소프트웨어(stereo_image_proc)로 고생하시나 했지만, **ROS 2의 내부 파이프라인을 이해하는 데에는 이만한 연습이 없습니다.**

말씀하신 `Assertion failed` 에러는 **"왼쪽 눈과 오른쪽 눈의 데이터가 같은 좌표계 이름(Frame ID)을 달고 있어야 하는데, 서로 다르다"**는 이유로 발생합니다.

`stereo_image_proc`을 속여서 작동시키기 위해, **오른쪽 카메라 데이터의 포장지(Header frame_id)만 왼쪽 것과 똑같이 바꿔치기하는 "중계 노드(Shim Node)"**를 하나 만들어서 해결하겠습니다.

---

### 1단계: 왼쪽 카메라의 정확한 Frame ID 확인

먼저, 우리가 "정답"으로 삼을 왼쪽 카메라의 `frame_id`를 알아내야 합니다. 터미널에 다음을 입력하세요.

```bash
ros2 topic echo /zed_multi/zed_right/left/color/rect/camera_info --field header.frame_id --once

```

출력 결과가 예를 들어 `zed_right_left_camera_optical_frame`라고 나왔다고 가정하겠습니다. (이 값을 아래 코드에 넣어야 합니다.)

---

### 2단계: Frame ID 해결사 노드 만들기 (Python)

이 파이썬 스크립트는 오른쪽 카메라의 이미지와 정보를 받아서, `frame_id`만 왼쪽 것과 똑같이 바꾼 뒤 다시 발행(Publish)합니다.

`fix_frame_id.py`라는 이름으로 파일을 만들고 아래 내용을 붙여넣으세요.
(**주의:** `TARGET_FRAME_ID` 변수에는 1단계에서 확인한 값을 넣으세요!)

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

class FrameIdFixer(Node):
    def __init__(self):
        super().__init__('frame_id_fixer')

        # === [설정] 여기에 1단계에서 확인한 왼쪽 카메라의 Frame ID를 넣으세요 ===
        self.TARGET_FRAME_ID = 'zed_right_left_camera_optical_frame' 
        # ===============================================================

        # 구독 (원래 오른쪽 데이터)
        self.sub_info = self.create_subscription(
            CameraInfo, 
            '/zed_multi/zed_right/right/color/rect/camera_info', 
            self.info_callback, 10)
        self.sub_img = self.create_subscription(
            Image, 
            '/zed_multi/zed_right/right/color/rect/image', 
            self.img_callback, 10)

        # 발행 (수정된 오른쪽 데이터)
        self.pub_info = self.create_publisher(CameraInfo, '/fixed/right/camera_info', 10)
        self.pub_img = self.create_publisher(Image, '/fixed/right/image', 10)

        self.get_logger().info('Frame ID Fixer Node Started...')

    def info_callback(self, msg):
        msg.header.frame_id = self.TARGET_FRAME_ID # Frame ID 강제 변경
        self.pub_info.publish(msg)

    def img_callback(self, msg):
        msg.header.frame_id = self.TARGET_FRAME_ID # Frame ID 강제 변경
        self.pub_img.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FrameIdFixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

---

### 3단계: Launch 파일 수정

이제 `stereo_image_proc`이 원본 오른쪽 데이터 대신, 우리가 만든 **`/fixed/...`** 토픽을 듣도록 `remappings`를 수정합니다.

```python
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        # 1. 우리가 만든 Frame ID 고치는 노드 실행 (Python 스크립트 실행)
        Node(
            package='YOUR_PACKAGE_NAME', # 파이썬 스크립트가 있는 패키지명 (없으면 단순히 python3 실행으로 대체 가능)
            executable='fix_frame_id',   # setup.py에 등록된 entry point
            name='frame_fixer',
            output='screen'
        ),

        # 2. Stereo Image Proc 실행
        ComposableNodeContainer(
            name='stereo_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::DisparityNode',
                    name='disparity_node',
                    remappings=[
                        # 왼쪽은 원래 데이터 그대로 사용
                        ('left/image_rect', '/zed_multi/zed_right/left/color/rect/image'),
                        ('left/camera_info', '/zed_multi/zed_right/left/color/rect/camera_info'),
                        
                        # [중요] 오른쪽은 우리가 수정한 '/fixed' 토픽을 사용
                        ('right/image_rect', '/fixed/right/image'),
                        ('right/camera_info', '/fixed/right/camera_info')
                    ]
                ),
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::PointCloudNode',
                    name='point_cloud_node',
                    remappings=[
                        # PointCloudNode도 동일하게 매핑
                        ('left/camera_info', '/zed_multi/zed_right/left/color/rect/camera_info'),
                        ('right/camera_info', '/fixed/right/camera_info'),
                        ('left/image_rect_color', '/zed_multi/zed_right/left/color/rect/image'),
                        ('disparity', '/disparity'),
                        ('points2', '/camera/depth/points')
                    ]
                )
            ],
            output='screen',
        )
    ])

```

---

### 실행 순서

1. **ZED Driver 실행:** 카메라 데이터를 뿜어냅니다.
2. **Fixer 노드 실행:** `python3 fix_frame_id.py` (또는 ros2 run)
* 이제 `/fixed/right/image` 같은 토픽이 나오는지 확인하세요.


3. **Launch 파일 실행:** 이제 `stereo_image_proc`이 에러 없이 돌아갈 것입니다.

이 방식을 사용하면, 물리적으로는 다른 위치에 있는 두 카메라지만 소프트웨어적으로는 "같은 평면상에 정렬된(Rectified) 이미지"라고 선언해 줌으로써 `stereo_image_proc`의 엄격한 검사를 통과하게 됩니다.

성공하면 RViz2를 켜서 `/camera/depth/points` 토픽을 확인해 보세요! PointCloud가 보일 겁니다.
#################################  
#################################  
#################################  
제공해주신 코드는 ROS 2 `stereo_image_proc` 패키지의 표준 launch 파일입니다. 이것은 매우 범용적으로 작성되어 있어서, 님의 구체적인 ZED 카메라 토픽 구조(`/zed_multi/...`)에 맞추려면 수정이 꽤 필요합니다.

요청하신 대로 **1) 파라미터 상세 설명**과 **2) ZED 카메라 및 Frame ID Fixer에 맞게 수정한 코드**를 정리해 드립니다.

---

### 1. 주요 파라미터 설명 (Computer Vision 관점)

이 파라미터들은 **"왼쪽 이미지의 어떤 픽셀이 오른쪽 이미지의 어디랑 짝궁인지(Matching)"** 찾는 알고리즘의 성능을 조절합니다.

#### **A. 핵심 알고리즘 설정**

* **`stereo_algorithm`**: (중요) 매칭 방식 결정
* `0` (Block Matching): 빠르지만 품질이 낮음. 점들이 듬성듬성 생김.
* `1` (SGM - Semi-Global Matching): 느리지만 훨씬 부드럽고 꽉 찬 PointCloud를 생성함. **(추천)**


* **`approximate_sync`**:
* `True`: 좌/우 이미지의 시간이 미세하게 달라도 짝을 맞춰줌. (네트워크 지연 등이 있을 때 필수)
* `False`: 시간이 완벽하게 일치해야 함. (ZED 같은 하드웨어 싱크 카메라는 False가 정석이나, 토픽 발행 딜레이가 있다면 True로 켜야 함)



#### **B. 매칭 품질 & 범위 설정**

* **`disparity_range`**: (가장 중요) 탐색 범위
* 값이 클수록 **더 가까운 물체**까지 감지할 수 있지만 연산량이 늘어남.
* 반드시 **16의 배수**여야 함 (예: 32, 64, 128). 보통 64나 128 사용.


* **`min_disparity`**:
* 탐색을 시작할 최소 시차. 보통 `0`으로 둡니다. 이 값을 키우면 먼 거리는 무시하고 가까운 것만 봅니다.


* **`correlation_window_size`**: (블록 크기)
* 비교할 사각형의 크기(픽셀).
* **값이 작으면(5~9):** 디테일이 살지만 노이즈가 많음.
* **값이 크면(15~21):** 부드러워지지만 작은 물체가 뭉개짐. **(보통 15 추천)**



#### **C. 노이즈 필터링 (품질 개선)**

* **`uniqueness_ratio`**:
* "확실한 짝궁"만 남기는 기준. 값이 클수록 엄격하게 걸러내어 PointCloud에 구멍이 많이 뚫리지만 정확도는 올라감. **(보통 10~15)**


* **`texture_threshold`**:
* 흰 벽처럼 무늬가 없는 곳을 걸러냄. 값을 높이면 벽면 노이즈가 사라짐.


* **`speckle_size` / `speckle_range**`:
* 허공에 둥둥 떠다니는 작은 점(Speckle)들을 지워버리는 필터.



---

### 2. 수정된 Launch 파일 (복사해서 사용하세요)

이 코드는 님의 **ZED 토픽**과, 앞서 만든 **Fixer 노드(오른쪽 카메라 수정)**를 반영하여 바로 작동하도록 개조했습니다.

**변경점:**

1. 복잡한 Namespace 인자 제거  님의 실제 토픽 이름(`zed_multi/...`)으로 **하드코딩**.
2. 불필요한 `image_proc`(색보정/Rectify) 과정 비활성화 (ZED가 이미 해주므로 CPU 낭비임).
3. 파라미터를 SGM(고품질) 모드로 기본 설정.

`run_zed_stereo_proc.launch.py` 이름으로 저장하세요.

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # === [설정 1] 토픽 이름 정의 (사용자 환경) ===
    # 왼쪽: ZED 원본 데이터 사용
    left_image_topic = '/zed_multi/zed_right/left/color/rect/image'
    left_info_topic  = '/zed_multi/zed_right/left/color/rect/camera_info'
    
    # 오른쪽: 우리가 만든 "Fixer 노드"가 뱉어내는 데이터 사용 (frame_id 수정본)
    right_image_topic = '/fixed/right/image'
    right_info_topic  = '/fixed/right/camera_info'

    # 결과물: Nav2가 구독할 토픽 이름
    output_cloud_topic = '/camera/depth/points'


    return LaunchDescription([
        # 파라미터 선언 (터미널에서 변경 가능)
        DeclareLaunchArgument('approximate_sync', default_value='True', description='Use approximate sync'),
        DeclareLaunchArgument('stereo_algorithm', default_value='1', description='0=BlockMatching, 1=SGBM(Better)'),
        DeclareLaunchArgument('disparity_range', default_value='128', description='Search range (16 multiple)'),

        # Stereo Image Proc 컨테이너 실행
        ComposableNodeContainer(
            name='stereo_proc_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            composable_node_descriptions=[
                
                # 1. Disparity Node (시차 계산)
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::DisparityNode',
                    name='disparity_node',
                    parameters=[{
                        'approximate_sync': LaunchConfiguration('approximate_sync'),
                        'stereo_algorithm': LaunchConfiguration('stereo_algorithm'),
                        'disparity_range': LaunchConfiguration('disparity_range'),
                        'correlation_window_size': 15,
                        'uniqueness_ratio': 10.0,
                        'texture_threshold': 10,
                        'speckle_size': 100,
                        'speckle_range': 4,
                        'min_disparity': 0,
                    }],
                    remappings=[
                        # [입력]
                        ('left/image_rect', left_image_topic),
                        ('left/camera_info', left_info_topic),
                        ('right/image_rect', right_image_topic),
                        ('right/camera_info', right_info_topic)
                        # [출력] -> /disparity (기본값 유지)
                    ]
                ),

                # 2. PointCloud Node (3D 변환)
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::PointCloudNode',
                    name='point_cloud_node',
                    parameters=[{
                        'approximate_sync': LaunchConfiguration('approximate_sync'),
                        'use_color': True, # 컬러 입히기
                        'avoid_point_cloud_padding': False,
                    }],
                    remappings=[
                        # [입력]
                        ('left/image_rect_color', left_image_topic), # 컬러 입힐 때 사용
                        ('left/camera_info', left_info_topic),
                        ('right/camera_info', right_info_topic),
                        ('disparity', '/disparity'), # 위 노드가 만든 토픽 받기
                        
                        # [출력] -> Nav2가 쓸 토픽
                        ('points2', output_cloud_topic)
                    ]
                )
            ]
        )
    ])

```

### 3. 실행 방법 (Step-by-Step)

1. **ZED 카메라 실행:** (기존 방식대로)
2. **Fixer 노드 실행:** (아까 만든 파이썬 파일)
```bash
python3 fix_frame_id.py

```


3. **이 Launch 파일 실행:**
```bash
ros2 launch <패키지명> run_zed_stereo_proc.launch.py

```



### 전문가 팁: 튜닝 가이드

* 실행 후 **Nav2 Costmap에 장애물이 잘 안 뜬다면?**
* `min_disparity`나 `disparity_range` 문제일 수 있습니다.
* 특히 **가까운 물체가 안 보이면** `min_disparity`는 0으로 두시고, 카메라 초점 거리상 너무 가까운 건(20cm 이내) 물리적으로 안 보일 수 있음을 감안해야 합니다.


* **Costmap에 점들이 너무 지저분하다면?**
* launch 파일에서 `speckle_size`를 `100`  `500` 정도로 올려보세요. 허공의 노이즈가 확 줄어듭니다.
############################
##############################
################################
**아주 훌륭한 선택입니다.**

`stereo_image_proc` 자체에는 "Decimation(다운샘플링)"이라는 정수형 파라미터가 직접적으로 존재하지 않습니다. 대신 ROS 2의 표준 방식인 **`image_proc::CropDecimateNode`**를 영상 처리 **앞단**에 붙여서 해결합니다.

이 방법은 단순히 점의 개수만 줄이는 것이 아니라, **SGBM(매칭 연산)을 수행할 픽셀 수 자체를 줄여주므로**, CPU 부하가 획기적으로 줄어들고 딜레이가 사라집니다.

---

### 1. 변경된 데이터 파이프라인

이미지를 먼저 줄이고(Resize), 그 작은 이미지를 가지고 Depth를 계산하는 방식입니다.

1. **입력 (High Res):** ZED(Left) / Fixer(Right)  예를 들어 1280x720
2. **Decimator:** 1/2 또는 1/4로 축소  640x360 (데이터량 1/4배)
3. **Stereo Proc:** 작은 이미지로 연산 수행  **초고속 연산**
4. **결과:** 가볍고 빠른 PointCloud  `pointcloud_to_laserscan` 딜레이 해소

---

### 2. 수정된 Launch 파일 코드

아래 코드는 **Decimator Node 2개(좌/우)**를 추가하고, 파이프라인을 연결한 최종 버전입니다.

**주요 변경점:**

* `decimation_factor` 변수 추가: `2`로 설정하면 가로/세로가 절반이 되어 **데이터는 1/4**로 줄어듭니다. `4`로 설정하면 **데이터는 1/16**로 줄어듭니다.

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # === [설정] 토픽 이름 정의 ===
    # 1. 원본 소스 (ZED & Fixer)
    raw_left_image  = '/zed_multi/zed_right/left/color/rect/image'
    raw_left_info   = '/zed_multi/zed_right/left/color/rect/camera_info'
    raw_right_image = '/fixed/right/image'       # Fixer 노드 출력
    raw_right_info  = '/fixed/right/camera_info' # Fixer 노드 출력

    # 2. 다운샘플링된 내부 토픽 (중간 단계)
    down_left_image  = '/down/left/image_rect'
    down_left_info   = '/down/left/camera_info'
    down_right_image = '/down/right/image_rect'
    down_right_info  = '/down/right/camera_info'

    # 3. 최종 출력
    output_cloud_topic = '/camera/depth/points'

    return LaunchDescription([
        # === [Argument] 다운샘플링 배율 설정 ===
        # 2 -> 해상도 1/2 (데이터 1/4), 4 -> 해상도 1/4 (데이터 1/16)
        DeclareLaunchArgument('decimation_factor', default_value='2', description='Decimation factor (2 or 4)'),
        
        DeclareLaunchArgument('approximate_sync', default_value='True'),
        DeclareLaunchArgument('stereo_algorithm', default_value='1'), # SGBM
        DeclareLaunchArgument('disparity_range', default_value='128'), 

        ComposableNodeContainer(
            name='stereo_proc_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            composable_node_descriptions=[
                
                # =========================================
                # [Step 1] 왼쪽 이미지 줄이기 (Decimator)
                # =========================================
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::CropDecimateNode',
                    name='decimator_left',
                    parameters=[{
                        'decimation_x': LaunchConfiguration('decimation_factor'),
                        'decimation_y': LaunchConfiguration('decimation_factor'),
                        'width': 0,  # 0이면 전체 크기에서 decimation 수행
                        'height': 0,
                    }],
                    remappings=[
                        ('in/image_raw', raw_left_image),
                        ('in/camera_info', raw_left_info),
                        ('out/image_raw', down_left_image),     # 줄어든 이미지 나감
                        ('out/camera_info', down_left_info)     # 줄어든 Info 나감 (자동 보정됨)
                    ]
                ),

                # =========================================
                # [Step 2] 오른쪽 이미지 줄이기 (Decimator)
                # =========================================
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::CropDecimateNode',
                    name='decimator_right',
                    parameters=[{
                        'decimation_x': LaunchConfiguration('decimation_factor'),
                        'decimation_y': LaunchConfiguration('decimation_factor'),
                        'width': 0,
                        'height': 0,
                    }],
                    remappings=[
                        ('in/image_raw', raw_right_image),
                        ('in/camera_info', raw_right_info),
                        ('out/image_raw', down_right_image),
                        ('out/camera_info', down_right_info)
                    ]
                ),

                # =========================================
                # [Step 3] Stereo Proc (줄어든 이미지로 수행)
                # =========================================
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::DisparityNode',
                    name='disparity_node',
                    parameters=[{
                        'approximate_sync': LaunchConfiguration('approximate_sync'),
                        'stereo_algorithm': LaunchConfiguration('stereo_algorithm'),
                        'disparity_range': LaunchConfiguration('disparity_range'),
                        'correlation_window_size': 15,
                        'uniqueness_ratio': 15.0,
                        'speckle_size': 500,  # 노이즈 제거
                        'speckle_range': 4,
                        'min_disparity': 1,
                    }],
                    remappings=[
                        # 입력으로 다운샘플링된 토픽을 받습니다.
                        ('left/image_rect', down_left_image),
                        ('left/camera_info', down_left_info),
                        ('right/image_rect', down_right_image),
                        ('right/camera_info', down_right_info)
                    ]
                ),

                # [Step 4] PointCloud 생성
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::PointCloudNode',
                    name='point_cloud_node',
                    parameters=[{
                        'approximate_sync': LaunchConfiguration('approximate_sync'),
                        'use_color': True,
                    }],
                    remappings=[
                        ('left/image_rect_color', down_left_image),
                        ('left/camera_info', down_left_info),
                        ('right/camera_info', down_right_info),
                        ('disparity', '/disparity'),
                        ('points2', output_cloud_topic)
                    ]
                )
            ]
        )
    ])

```

---

### 3. 적용 및 기대 효과

1. **실행:** 위 코드로 launch 파일을 교체하고 실행하세요.
* 기본값인 `decimation_factor=2`가 적용됩니다.
* 더 가볍게 하고 싶으면: `ros2 launch ... decimation_factor:=4`


2. **`image_proc` 패키지 설치 확인:**
* 만약 실행 시 `CropDecimateNode`를 못 찾는다는 에러가 뜨면 설치가 필요합니다.
* `sudo apt install ros-jazzy-image-proc`


3. **기대 효과:**
* **입력 해상도:** 1280x720  640x360 (Factor 2)
* **PointCloud 개수:** 약 92만 개  약 23만 개 (75% 감소)
* **ROS 2 Delay:** 1.0s  **0.2s 이하**로 급격히 감소 예상.
* **장애물 인식:** 해상도가 줄어도 벽, 사람, 기둥 같은 장애물 인식에는 전~혀 지장이 없습니다.



이제 `ros2 topic hz /camera/depth/points`로 속도가 얼마나 빨라졌는지, 그리고 `pointcloud_to_laserscan`의 딜레이가 줄었는지 확인해 보세요!
