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
##############################  
################################  
#################################  
**정말 좋은 접근입니다.** 엔지니어로서 성능 병목(Bottleneck)의 원인을 정확히 파악하셨습니다.

기존 코드는 유효하지 않은 점(NaN)이나 필요 없는 영역(천장 등)까지 포함하여 `width * height` 크기의 거대한 메모리를 무조건 할당합니다. 이것이 직렬화/역직렬화 과정에서 큰 딜레이를 유발합니다.

요청하신 대로 `stereo_image_proc`의 소스 코드를 수정하여, **Point Cloud 생성 단계에서 즉시 필터링**을 수행하도록 만들겠습니다. 이렇게 하면 데이터 크기가 1/10 ~ 1/100로 줄어들어 딜레이가 획기적으로 개선됩니다.

### 수정 전략 (Key Changes)

1. **ROI Filter (Height):** `z` 값(높이)이 설정된 범위를 벗어나면 아예 메모리에 담지 않습니다.
2. **Downsampling (Step):** Voxel Filter 대신, 훨씬 가볍고 빠른 **Step Downsampling**을 적용합니다. (모든 픽셀을 보는 대신 2칸, 4칸씩 건너뛰며 샘플링). PCL 라이브러리 의존성 없이 즉시 적용 가능합니다.
3. **Unorganized Cloud 변환:** 기존의 `Image-like (Grid)` 구조를 버리고, 유효한 점들만 일렬로 나열하는 `List` 구조로 변경하여 페이로드를 최소화합니다.

---

### 수정된 소스 코드 (`point_cloud_node.cpp`)

아래 코드를 복사하여 덮어쓰시면 됩니다. 주석 **`// [MODIFIED]`** 로 표시된 부분이 변경된 핵심 로직입니다.

```cpp
// [MODIFIED] 필요한 헤더 추가
#include <vector>
#include <cstring> 

// ... (기존 헤더들은 그대로 유지)
#include <limits>
#include <memory>
#include <string>

#include "image_geometry/stereo_camera_model.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include "rcutils/logging_macros.h"

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
// [MODIFIED] Iterator는 이제 쓰지 않고 직접 메모리를 조작합니다 (속도 최적화)
// #include <sensor_msgs/point_cloud2_iterator.hpp> 
#include <stereo_msgs/msg/disparity_image.hpp>

namespace stereo_image_proc
{

class PointCloudNode : public rclcpp::Node
{
public:
  explicit PointCloudNode(const rclcpp::NodeOptions & options);

private:
  // Subscriptions
  image_transport::SubscriberFilter sub_l_image_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> sub_l_info_, sub_r_info_;
  message_filters::Subscriber<stereo_msgs::msg::DisparityImage> sub_disparity_;
  using ExactPolicy = message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo,
    sensor_msgs::msg::CameraInfo,
    stereo_msgs::msg::DisparityImage>;
  using ApproximatePolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo,
    sensor_msgs::msg::CameraInfo,
    stereo_msgs::msg::DisparityImage>;
  using ExactSync = message_filters::Synchronizer<ExactPolicy>;
  using ApproximateSync = message_filters::Synchronizer<ApproximatePolicy>;
  std::shared_ptr<ExactSync> exact_sync_;
  std::shared_ptr<ApproximateSync> approximate_sync_;

  // Publications
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_points2_;

  // Processing state
  image_geometry::StereoCameraModel model_;
  cv::Mat_<cv::Vec3f> points_mat_; 

  // [MODIFIED] 파라미터 저장을 위한 변수
  double filter_min_height_;
  double filter_max_height_;
  int downsample_step_;

  void connectCb();

  void imageCb(
    const sensor_msgs::msg::Image::ConstSharedPtr & l_image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & l_info_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & r_info_msg,
    const stereo_msgs::msg::DisparityImage::ConstSharedPtr & disp_msg);
};

PointCloudNode::PointCloudNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("point_cloud_node", options)
{
  using namespace std::placeholders;

  int queue_size = this->declare_parameter("queue_size", 5);
  bool approx = this->declare_parameter("approximate_sync", false);
  this->declare_parameter("use_system_default_qos", false);
  
  // [MODIFIED] 필터 파라미터 선언
  // min/max height: 로봇 바닥 및 천장 제거용 (단위: meter)
  filter_min_height_ = this->declare_parameter("filter_min_height", -100.0); 
  filter_max_height_ = this->declare_parameter("filter_max_height", 100.0);
  
  // downsample_step: 1이면 모든 픽셀, 2면 1/4, 4면 1/16 (Voxel Filter 대체 효과)
  downsample_step_ = this->declare_parameter("downsample_step", 1);
  if (downsample_step_ < 1) downsample_step_ = 1;

  this->declare_parameter("use_color", true);
  // Padding 관련 파라미터는 Unorganized Cloud에서는 큰 의미 없으나 호환성 위해 유지
  this->declare_parameter("avoid_point_cloud_padding", false);

  if (approx) {
    approximate_sync_.reset(
      new ApproximateSync(
        ApproximatePolicy(queue_size),
        sub_l_image_, sub_l_info_,
        sub_r_info_, sub_disparity_));
    approximate_sync_->registerCallback(
      std::bind(&PointCloudNode::imageCb, this, _1, _2, _3, _4));
  } else {
    exact_sync_.reset(
      new ExactSync(
        ExactPolicy(queue_size),
        sub_l_image_, sub_l_info_,
        sub_r_info_, sub_disparity_));
    exact_sync_->registerCallback(
      std::bind(&PointCloudNode::imageCb, this, _1, _2, _3, _4));
  }

  rclcpp::PublisherOptions pub_opts;
  pub_opts.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  pub_points2_ = create_publisher<sensor_msgs::msg::PointCloud2>("points2", 1, pub_opts);

  connectCb();
}

void PointCloudNode::connectCb()
{
  image_transport::TransportHints hints(this, "raw");
  const bool use_system_default_qos = this->get_parameter("use_system_default_qos").as_bool();
  rclcpp::QoS image_sub_qos = rclcpp::SensorDataQoS();
  if (use_system_default_qos) {
    image_sub_qos = rclcpp::SystemDefaultsQoS();
  }
  const auto image_sub_rmw_qos = image_sub_qos.get_rmw_qos_profile();
  auto sub_opts = rclcpp::SubscriptionOptions();
  sub_opts.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  
  sub_l_image_.subscribe(
    this, "left/image_rect_color", hints.getTransport(), image_sub_rmw_qos, sub_opts);
  sub_l_info_.subscribe(this, "left/camera_info", image_sub_rmw_qos, sub_opts);
  sub_r_info_.subscribe(this, "right/camera_info", image_sub_rmw_qos, sub_opts);
  sub_disparity_.subscribe(this, "disparity", image_sub_rmw_qos, sub_opts);
}

inline bool isValidPoint(const cv::Vec3f & pt)
{
  return pt[2] != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(pt[2]);
}

void PointCloudNode::imageCb(
  const sensor_msgs::msg::Image::ConstSharedPtr & l_image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & l_info_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & r_info_msg,
  const stereo_msgs::msg::DisparityImage::ConstSharedPtr & disp_msg)
{
  if (pub_points2_->get_subscription_count() == 0u) {
    return;
  }

  // 1. 파라미터 실시간 업데이트 (런타임 튜닝 가능하도록)
  this->get_parameter("filter_min_height", filter_min_height_);
  this->get_parameter("filter_max_height", filter_max_height_);
  this->get_parameter("downsample_step", downsample_step_);
  bool use_color = this->get_parameter("use_color").as_bool();

  model_.fromCameraInfo(l_info_msg, r_info_msg);

  // 2. Disparity -> 3D Projection
  const sensor_msgs::msg::Image & dimage = disp_msg->image;
  float * data = reinterpret_cast<float *>(const_cast<uint8_t *>(&dimage.data[0]));
  const cv::Mat_<float> dmat(dimage.height, dimage.width, data, dimage.step);
  model_.projectDisparityImageTo3d(dmat, points_mat_, true);
  cv::Mat_<cv::Vec3f> mat = points_mat_;

  // 3. [핵심] 유효한 포인트만 골라내기 (ROI Filtering & Downsampling)
  // std::vector를 버퍼로 사용하여 유효한 데이터만 쌓습니다.
  
  // 포인트 구조체 정의 (메모리 정렬)
  struct PointXYZ { float x, y, z; };
  struct PointXYZRGB { float x, y, z; float rgb; };

  std::vector<uint8_t> buffer;
  // 최대 예상 크기로 예약 (재할당 방지)
  size_t est_points = (mat.rows * mat.cols) / (downsample_step_ * downsample_step_);
  if(use_color) buffer.reserve(est_points * sizeof(PointXYZRGB));
  else          buffer.reserve(est_points * sizeof(PointXYZ));

  // Color 처리 준비
  namespace enc = sensor_msgs::image_encodings;
  const std::string & encoding = l_image_msg->encoding;
  cv::Mat_<cv::Vec3b> color_mat;
  bool has_color = false;
  
  if (use_color && (encoding == enc::RGB8 || encoding == enc::BGR8)) {
     color_mat = cv::Mat_<cv::Vec3b>(l_image_msg->height, l_image_msg->width, 
                                     (cv::Vec3b *)(&l_image_msg->data[0]), l_image_msg->step);
     has_color = true;
  }

  // Loop 수행 (Step 만큼 건너뛰며 순회)
  int valid_count = 0;
  for (int v = 0; v < mat.rows; v += downsample_step_) {
    for (int u = 0; u < mat.cols; u += downsample_step_) {
      
      cv::Vec3f pt = mat(v, u);

      // [Filter 1] 유효성 검사 (NaN, Inf 제거)
      if (!isValidPoint(pt)) continue;

      // [Filter 2] ROI Height 필터 (천장/바닥 제거)
      // Camera 좌표계에서 pt[1]은 아래쪽(Y), pt[2]는 앞쪽(Z), pt[0]은 오른쪽(X)입니다.
      // 보통 '높이'는 World 좌표계의 Z이지만, 카메라 좌표계 기준으로는 Y축이 높이와 관련됩니다.
      // 다만, 사용자의 TF 설정에 따라 다를 수 있으므로 여기서는
      // "Nav2에서 사용하는 높이"가 아니라 "Camera Frame 기준의 좌표"임을 유의해야 합니다.
      // 일단 일반적인 Nav2 연동을 위해, 변환된 PointCloud의 특정 축을 검사합니다.
      // (Optical Frame: Z=Forward, Y=Down, X=Right)
      // 사용자가 원하는 "높이" 필터링이 Optical Frame의 Y축(위아래)인지, Z축(거리)인지에 따라 다릅니다.
      // 보통 '천장'을 자르려면 Optical Frame의 Y축(pt[1])을 검사해야 합니다. (Y가 음수면 위쪽)
      
      // 여기서는 사용자가 직관적으로 이해하는 "높이(Height)" 필터링을 위해
      // pt[1] (Camera Y축, 아래가 양수) 값을 검사합니다.
      // 예: 로봇 위 2m 천장 -> Y = -2.0 미만인 점 제거
      
      // *주의*: 만약 Optical Frame이 아니라 Base Link 기준 높이를 원한다면 TF 변환이 필요하지만,
      // 여기서는 성능을 위해 카메라 좌표계 기준으로 필터링합니다.
      // (보통 카메라가 정면을 보면 Y축이 높이입니다.)
      
      // 여기서는 일반적인 거리(Range) 필터가 아니라 사용자가 요청한 'Height' 필터를
      // Camera Frame Y축 기준으로 적용합니다. (Y값은 아래로 갈수록 커짐)
      // 천장(위쪽)은 Y값이 작고(음수), 바닥(아래쪽)은 Y값이 큽니다.
      if (pt[1] < -filter_max_height_ || pt[1] > -filter_min_height_) continue;
      // 참고: 위 조건은 카메라가 정면을 보고 있을 때 기준입니다. 
      // 만약 헷갈린다면 일단 주석처리 하거나, Range(Z축) 필터로 바꾸셔도 됩니다.
      // if (pt[2] > 5.0) continue; // 예: 5m 이상 거리는 무시

      // 버퍼에 데이터 밀어넣기
      if (use_color && has_color) {
        PointXYZRGB p;
        p.x = pt[0]; p.y = pt[1]; p.z = pt[2];
        
        // Color Packing
        const cv::Vec3b & c = color_mat(v, u);
        uint32_t rgb = (static_cast<uint32_t>(c[0]) << 16 | 
                        static_cast<uint32_t>(c[1]) << 8 | 
                        static_cast<uint32_t>(c[2]));
        // Encoding check for BGR/RGB swap is omitted for speed; assuming RGB8 or BGR8 match
        if (encoding == enc::BGR8) {
            rgb = (static_cast<uint32_t>(c[2]) << 16 | 
                   static_cast<uint32_t>(c[1]) << 8 | 
                   static_cast<uint32_t>(c[0]));
        }
        
        p.rgb = *reinterpret_cast<float*>(&rgb);
        
        // Vector에 byte 단위로 추가
        const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&p);
        buffer.insert(buffer.end(), ptr, ptr + sizeof(PointXYZRGB));
      } else {
        PointXYZ p;
        p.x = pt[0]; p.y = pt[1]; p.z = pt[2];
        const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&p);
        buffer.insert(buffer.end(), ptr, ptr + sizeof(PointXYZ));
      }
      valid_count++;
    }
  }

  // 4. PointCloud2 메시지 생성
  auto points_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  points_msg->header = disp_msg->header;
  
  // [중요] Unorganized Cloud (1D 배열 형태)로 설정
  points_msg->height = 1;
  points_msg->width = valid_count;
  
  points_msg->is_bigendian = false;
  points_msg->is_dense = true; // NaN 없음

  sensor_msgs::PointCloud2Modifier pcd_modifier(*points_msg);
  if (use_color) {
    pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    points_msg->point_step = 16; // 4 floats * 4 bytes
  } else {
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
    points_msg->point_step = 12; // 3 floats * 4 bytes
  }

  points_msg->row_step = points_msg->point_step * points_msg->width;
  
  // 5. 데이터 복사 (이미 필터링된 버퍼를 그대로 덮어씀)
  points_msg->data = std::move(buffer);

  pub_points2_->publish(*points_msg);
}

}  // namespace stereo_image_proc

RCLCPP_COMPONENTS_REGISTER_NODE(stereo_image_proc::PointCloudNode)

```

---

### 코드 적용 후 사용 방법

1. **빌드:**
위 코드로 파일을 교체한 후, 해당 패키지(`stereo_image_proc`)를 다시 빌드(colcon build)하세요.
2. **Launch 파일 수정:**
Launch 파일에서 파라미터를 추가하여 제어할 수 있습니다.
```python
ComposableNode(
    package='stereo_image_proc',
    plugin='stereo_image_proc::PointCloudNode',
    name='point_cloud_node',
    parameters=[{
        'use_color': True,
        # [새로 추가된 기능]
        'downsample_step': 4,       # 16개의 픽셀 중 1개만 선택 (데이터량 94% 감소)
        'filter_min_height': 0.1,   # 바닥(0.1m) 아래 제거 (카메라 좌표계 Y축 기준 주의)
        'filter_max_height': 2.0,   # 천장(2.0m) 위 제거
    }],
    # ... remappings ...
)

```



### 왜 이 방식이 빠른가요?

1. **Unorganized Cloud:** 기존에는 공기(NaN)까지 포함해서 `1280*720`개를 다 보냈지만, 이제는 `유효한 점`만 보냅니다. (데이터 크기 급감)
2. **Early Rejection:** `imageCb` 내부 루프에서 `isValid`와 `ROI` 체크를 먼저 수행하므로, 불필요한 데이터를 메모리에 쓰는 과정 자체가 사라졌습니다.
3. **Step Downsampling:** 복잡한 연산 없이 `i += step` 방식으로 건너뛰기 때문에 CPU 부하가 거의 없습니다.

이제 `pointcloud_to_laserscan`에 도달하는 데이터는 이미 **"다이어트가 완료된 핵심 데이터"**뿐이므로, 딜레이가 0.6초에서 0.05초 수준으로 떨어질 것입니다.
########################################  
###########################################  
#############################################  
네, 컴퓨터 비전 전문가로서 이 코드(Stereo Image Proc의 `DisparityNode`)에 등장하는 파라미터들을 **직관적**으로 이해하실 수 있도록 설명해 드리겠습니다.

이 노드의 핵심 목표는 **왼쪽/오른쪽 카메라 이미지의 차이(Disparity)를 계산하여 깊이(Depth)를 추정**하는 것입니다. 파라미터들은 크게 **1. ROS 2 통신 설정**과 **2. 스테레오 매칭 알고리즘 튜닝** 두 가지로 나뉩니다.

---

### 1. ROS 2 노드 및 통신 설정 (데이터 동기화)

이 부분은 두 카메라(좌/우)의 이미지가 들어올 때 어떻게 짝을 맞출지 결정합니다.

* **`queue_size` (기본값: 5)**
* **의미:** 메시지 수신 버퍼 크기입니다.
* **직관:** "택배 상자 쌓아두는 공간"입니다. 네트워크가 불안정하거나 처리 속도가 느려질 때, 몇 장의 이미지를 버리지 않고 대기시킬지 결정합니다. 너무 크면 지연(Latency)이 생길 수 있습니다.


* **`approximate_sync` (기본값: false)**
* **의미:** 대략적인 시간 동기화 사용 여부입니다.
* **직관:** 하드웨어 트리거가 되어 있어 좌/우 카메라 셔터가 정확히 동시에 터진다면 `false`(Exact Time)를 씁니다. 만약 USB 카메라 두 대를 따로 써서 시간이 미세하게 안 맞다면 `true`로 설정해서 "비슷한 시간대"의 이미지를 짝지어줍니다.


* **`approximate_sync_tolerance_seconds`**
* **의미:** `approximate_sync`가 켜졌을 때 허용할 시간 오차입니다. 이 시간보다 차이가 크면 짝이 아니라고 보고 버립니다.



---

### 2. 스테레오 매칭 알고리즘 파라미터 (핵심 비전 로직)

여기가 가장 중요한 부분입니다. OpenCV의 `StereoBM` (Block Matching)과 `StereoSGBM` (Semi-Global Block Matching) 알고리즘을 제어합니다.

#### **A. 알고리즘 선택 및 탐색 범위**

* **`stereo_algorithm` (0: BM, 1: SGBM)**
* **BM (Block Matching):** 속도가 매우 빠르지만 정확도가 다소 떨어집니다.
* **SGBM (Semi-Global BM):** 계산량이 많아 무겁지만, 훨씬 부드럽고 빈 공간(Hole)이 적은 고품질의 깊이 지도를 만듭니다.


* **`min_disparity`**
* **의미:** 탐색을 시작할 최소 시차(Disparity) 값입니다.
* **직관:** **"얼마나 멀리 있는 물체까지 볼 것인가?"**와 관련됩니다. 보통 0으로 두지만, 카메라 캘리브레이션에 따라 이미지를 이동시켜야 한다면 값을 조정합니다.


* **`disparity_range` (매우 중요)**
* **의미:** 탐색할 시차의 범위(개수)입니다. (반드시 16의 배수여야 함)
* **직관:** **"측정 가능한 가장 가까운 거리"**를 결정합니다.
* 값이 **크면**: 아주 가까운 물체의 깊이도 계산할 수 있지만, 계산량이 엄청나게 늘어납니다.
* 값이 **작으면**: 계산은 빠르지만, 너무 가까이 있는 물체는 깊이를 못 잡고 무시합니다.





#### **B. 매칭 프로세스 (유사도 검사)**

* **`correlation_window_size` (SAD Window Size)**
* **의미:** 픽셀을 비교할 때 주변 몇 픽셀을 묶어서 볼지 결정합니다. (홀수여야 함, 예: 5, 7, 15...)
* **직관:** **"돋보기의 크기"**입니다.
* **작으면**: 디테일(작은 물체)을 잘 잡지만, 노이즈(자글자글함)가 심해집니다.
* **크면**: 노이즈가 줄고 부드러워지지만, 물체 경계가 뭉개지고 작은 물체는 사라집니다.





#### **C. 필터링 및 노이즈 제거 (품질 향상)**

* **`prefilter_size` & `prefilter_cap` (BM 전용)**
* **직관:** 매칭을 하기 전에 조명 변화나 밝기 차이를 줄여주는 **"선글라스"** 역할입니다. 이미지의 밝기를 정규화해서 그림자나 조명 차이 때문에 매칭 실패하는 것을 막아줍니다.


* **`texture_threshold`**
* **의미:** 텍스처(무늬)가 없는 영역을 걸러내는 임계값입니다.
* **직관:** **"흰 벽이나 파란 하늘은 깊이를 계산하지 마라."** 아무 무늬가 없는 곳은 매칭이 불가능하므로, 엉뚱한 값을 내느니 차라리 계산을 포기(검은색 처리)하게 만듭니다.


* **`uniqueness_ratio`**
* **의미:** 최적의 매칭 값과 두 번째로 좋은 매칭 값의 차이 비율입니다.
* **직관:** **"확신이 없으면 정답이라고 하지 마."**
* 이 값이 높으면: 아주 확실한 매칭만 남기고, 조금이라도 헷갈리면 버립니다. (데이터가 듬성듬성해짐)
* 이 값이 낮으면: 꽉 찬 깊이 지도를 얻지만, 틀린 값(Outlier)이 많아집니다.




* **`speckle_size` & `speckle_range**`
* **의미:** 작은 노이즈 덩어리(Speckle) 제거.
* **직관:** **"허공에 떠 있는 먼지 같은 점들은 지워라."**
* `speckle_size`: 이 픽셀 수보다 작은 덩어리는 노이즈로 간주하고 지웁니다.
* `speckle_range`: 덩어리를 판단할 때 깊이 차이가 얼마나 나면 다른 덩어리로 볼지 결정합니다.





#### **D. SGBM 전용 파라미터 (스무딩 및 최적화)**

SGBM은 픽셀 단위 매칭뿐만 아니라 전체적인 "평탄함(Smoothness)"을 고려합니다.

* **`P1`**
* **직관:** **"깊이가 1칸 정도 살짝 바뀌는 건 괜찮아."** (비스듬한 벽면 등). 이 페널티 값이 작으면 경사가 부드럽게 이어집니다.


* **`P2`**
* **직관:** **"깊이가 갑자기 팍 바뀌는 건(물체 경계선) 좀 더 신중해야 해."**
* `P2`는 항상 `P1`보다 커야 합니다. 이 값이 커질수록 표면이 아주 매끄러워지지만 디테일이 사라질 수 있습니다. (보통 P1 = 8*채널*win^2, P2 = 32*채널*win^2 공식 사용)




* **`disp12_max_diff`**
* **직관:** **"왼쪽->오른쪽 검사와 오른쪽->왼쪽 검사가 일치해야 한다."** (Left-Right Consistency Check). 두 방향 검사 결과가 이 값 이상 차이 나면 가짜 매칭으로 보고 버립니다.


* **`sgbm_mode`**
* **직관:** SGBM 알고리즘의 변종 선택입니다. (일반 SGBM, 3-way, HH 등). HH(Hirschmuller) 모드는 더 정확하지만 느립니다.



---

### 요약: 튜닝 가이드

1. **깊이 지도가 너무 듬성듬성하다면?**  `uniqueness_ratio`를 낮추거나 `texture_threshold`를 낮추세요.
2. **가까운 물체가 안 보인다면?**  `disparity_range`를 늘리세요 (단, 연산량 증가 주의).
3. **결과가 너무 자글자글하다면?**  `correlation_window_size`를 키우거나 `P1`, `P2` (SGBM일 때) 값을 높이세요.
4. **흰 벽에서 이상한 값이 튄다면?**  `texture_threshold`를 높이세요.

이 파라미터들은 환경(실내/실외, 조명, 물체 거리)에 따라 최적값이 완전히 달라지므로, **rqt_reconfigure** 같은 툴을 켜두고 실시간으로 슬라이더를 움직여보며 감을 잡는 것이 가장 좋습니다.

**혹시 현재 사용 중인 카메라 환경(예: 실내 로봇 팔, 실외 자율주행 등)을 알려주시면, 그에 맞는 추천 파라미터 값을 제안해 드릴 수 있습니다.**
