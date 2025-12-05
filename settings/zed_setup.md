지금 목표를 한 줄로 정리하면:

> Jetson Orin AGX(JetPack 6.2.1) + ZED Link Quad + ZED X 4대 + ROS2 Humble 에서 **ZED ROS2 wrapper로 RGB / Depth만 pub 하고**, IMU/odom/SLAM 관련 TF/robot_state_publisher는 전부 끄기

아래 튜토리얼은 **풀 스택 기준(드라이버 설치 ~ 런치파일)** 로 적을게.
이미 끝난 단계가 있으면 그 부분은 스킵해도 돼.

---

## 0. 전체 구조 / 전제

* Jetson Orin AGX + JetPack 6.2.1 (Ubuntu 22.04 + ROS2 Humble 가정)
* ZED Link Quad capture card + ZED X 4대 (GMSL2)
* `zed-ros2-wrapper` v5.1 계열 (Humble 지원)([Stereolabs][1])
* 하고 싶은 것:

  * ✅ 각 카메라별 RGB / Depth 토픽만 사용 (`~/rgb/...`, `~/depth/...` 계열)([Stereolabs][1])
  * ❌ ZED IMU / odom / pose / map-odom / odom-base_link TF, GNSS, mapping, object detection 등은 전부 끔([Stereolabs][1])
  * SLAM 노드는 따로 있고, 그 쪽에서 odom/TF/robot_state_publisher를 담당

---

## 1. Jetson Orin AGX 기본 세팅 (요약)

JetPack 6.2.1 이미 올라가 있다고 가정.

```bash
# 전형적인 성능 모드
sudo nvpmodel -m 0
sudo jetson_clocks
```

---

## 2. ZED Link Quad + ZED SDK 설치

### 2-1. ZED Link Quad 드라이버 설치

Stereolabs 문서 기준으로, Orin AGX + ZED Link Quad는 **전용 ZED Link 드라이버**를 먼저 설치해야 한다.([Stereolabs][1])

1. Jetson의 L4T/JetPack 버전에 맞는 ZED Link 드라이버 `.deb` 를 Stereolabs 사이트에서 다운로드
2. 설치:

```bash
sudo dpkg -i zed-link-driver_<버전>_amd64.deb
sudo apt-get -f install
```

3. 재부팅 후, 링크 인식 확인:

```bash
dmesg | grep -i zed
# 혹은 GMSL 링크 상태 체크 (문서에 있는 예시 스크립트)
```

공식 튜토리얼(“ZED Link Quad on AGX Orin”)에서 위 순서를 그대로 따라가면 됨.

### 2-2. ZED SDK 설치 (JetPack 6.x 대응 버전)

1. Stereolabs 다운로드 페이지에서 Jetson용 ZED SDK (JetPack 6.x용) `.run` 파일 받기([Stereolabs][2])
2. 실행:

```bash
chmod +x ZED_SDK_Ubuntu22_JP6.x_vX.Y.run
sudo ./ZED_SDK_Ubuntu22_JP6.x_vX.Y.run
```

3. 설치가 끝나면 테스트:

```bash
ZED_Explorer --all      # 연결된 모든 ZED X 카메라, 시리얼 확인
ZED_Diagnostic          # 드라이버/SDK 상태 점검
```

이때 `--all` 출력에 4개의 ZED X와 각 시리얼 번호가 보이면 OK.([Stereolabs][3])

---

## 3. ROS2 Humble 설치 (apt 기반, 요약)

JetPack 6.2.1 은 Ubuntu 22.04 이므로 공식 apt 설치 사용 가능.

```bash
sudo apt update && sudo apt install -y \
  ros-humble-desktop \
  python3-colcon-common-extensions \
  python3-rosdep

sudo rosdep init || true
rosdep update

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 4. zed-ros2-wrapper 소스 설치 및 빌드

### 4-1. 워크스페이스 생성

```bash
mkdir -p ~/zed_ws/src
cd ~/zed_ws/src
git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
cd ~/zed_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4-2. JetPack 6.x 빌드 이슈 해결용 패키지 설치

Stereolabs README에 따르면 JetPack 6.x에서 CUDA 경로 관련 에러가 나면 `nvidia-jetpack` 패키지를 추가로 설치하라고 되어 있음.([GitHub][4])

```bash
sudo apt install -y nvidia-jetpack nvidia-jetpack-dev
```

### 4-3. colcon 빌드

```bash
cd ~/zed_ws
colcon build --cmake-args "-DCMAKE_BUILD_TYPE=Release"
echo "source ~/zed_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 5. ZED 설정 파일 구성 (IMU/odom/TF 끄기)

ZED ROS2 wrapper는 `common_stereo.yaml` + 카메라별 `zedx.yaml` 로 설정한다.([Stereolabs][1])

### 5-1. 공통 설정 파일 복사

```bash
cd ~/zed_ws/src/zed-ros2-wrapper/zed_wrapper/config

cp common_stereo.yaml common_stereo_quad.yaml
cp zedx.yaml zedx_front_left.yaml
cp zedx.yaml zedx_front_right.yaml
cp zedx.yaml zedx_rear_left.yaml
cp zedx.yaml zedx_rear_right.yaml
```

이제 `common_stereo_quad.yaml` 에서 **IMU/odom/TF/SLAM 관련 기능을 전부 OFF** 한다.

### 5-2. 공통 YAML에서 켜고/끄는 핵심 파라미터

`common_stereo_quad.yaml` 안에 아래 블럭들을 추가/수정 (이미 있으면 값만 수정):

```yaml
# --- Depth 관련 (RGB + Depth만 쓰고 싶다면) ---
depth:
  depth_mode: "NEURAL"                  # 품질/속도 적당한 모드
  publish_depth_map: true               # /depth/depth_registered
  publish_depth_info: true              # /depth/depth_info
  publish_point_cloud: false            # 필요 없으면 false로 GPU/대역폭 절약
  publish_disparity: false
  publish_depth_confidence: false

video:
  publish_rgb: true                     # /rgb/color/rect/image
  publish_left: false
  publish_right: false
  publish_stereo: false
  publish_raw: false
  publish_gray: false

# --- 포지셔널 트래킹 / odom / TF 전부 OFF ---
pos_tracking:
  pos_tracking_enabled: false           # SLAM이 따로 있으니 비활성화
  imu_fusion: false
  publish_tf: false                     # odom -> base_link 등 TF 끔
  publish_map_tf: false                 # map -> odom TF 끔
  publish_odom_pose: false              # /odom, /pose 등 끔
  publish_cam_path: false              # /path_* 끔

# --- GNSS / mapping / object / body tracking 전부 OFF ---
gnss_fusion:
  gnss_fusion_enabled: false

mapping:
  mapping_enabled: false

object_detection:
  od_enabled: false

body_tracking:
  bt_enabled: false

# --- 센서(IMU 등) 전부 OFF ---
sensors:
  publish_imu_tf: false                 # imu_link -> camera TF 끔
  publish_imu: false                    # /imu/data 끔
  publish_imu_raw: false                # /imu/data_raw 끔
  publish_cam_imu_transf: false         # /left_cam_imu_transform 끔
  publish_mag: false
  publish_baro: false
  publish_temp: false

# --- 디버그/로그 정도는 필요에 따라 ---
debug:
  sdk_verbose: 1
  debug_common: false
```

위 파라미터 이름은 공식 문서 `Configuration parameters` 섹션 기준이다.([Stereolabs][1])

> 💡 이렇게 하면 ZED 노드는 **RGB / Depth 관련 토픽만 pub** 하고, IMU/odom/TFLocalization/Mapping/Object Detection 등은 아예 돌지 않는다.

---

## 6. 카메라별 YAML(zedx_*.yaml)에서 이름/시리얼/프레임 설정

각 카메라별 YAML(`zedx_front_left.yaml` 등)에서 최소 다음 필드들을 설정하면 좋다:

```yaml
# 예: zedx_front_left.yaml

camera_model: "zedx"
camera_name: "zedx_fl"          # namespace로도 같이 쓰임

general:
  serial_number: 12345678       # ZED_Explorer --all 로 확인한 시리얼
  gpu_id: 0                     # 모두 같은 GPU를 써도 됨 (Orin AGX)

# 해상도/프레임레이트
camera:
  grab_resolution: "HD720"
  grab_frame_rate: 30
```

나머지 3개의 카메라도 각각:

* `camera_name`: `zedx_fr`, `zedx_rl`, `zedx_rr`
* `serial_number`: 각자의 시리얼 번호로 설정

---

## 7. 4대 카메라를 동시에 띄우는 Launch 파일

여기서 **핵심**은:

* Stereolabs 기본 `zed_camera.launch.py` 를 쓰면 robot_state_publisher/URDF까지 같이 올라간다.
* 질문 조건상 `robot_state_publisher` 도 ZED wrapper 쪽에서 안 올리는 게 깔끔하므로,
  **우리가 직접 `zed_wrapper_node`만 띄우는 Launch 파일을 만드는 방식**으로 가는 게 안전.

### 7-1. 간단한 multi-camera 런치 패키지 만들기

새 패키지 (예: `zed_multi_launch`) 생성:

```bash
cd ~/zed_ws/src
ros2 pkg create zed_multi_launch --build-type ament_python --dependencies rclpy launch launch_ros
```

`zed_multi_launch/zed_multi_launch/zedx_quad.launch.py` 파일 생성:

```python
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    zed_wrapper_share = get_package_share_directory('zed_wrapper')

    common_cfg = os.path.join(zed_wrapper_share, 'config', 'common_stereo_quad.yaml')

    def make_zed_node(ns: str, camera_cfg: str):
        return Node(
            package='zed_wrapper',
            executable='zed_wrapper_node',
            namespace=ns,
            name='zed_node',
            output='screen',
            parameters=[
                common_cfg,
                camera_cfg,
            ],
        )

    fl_cfg = os.path.join(zed_wrapper_share, 'config', 'zedx_front_left.yaml')
    fr_cfg = os.path.join(zed_wrapper_share, 'config', 'zedx_front_right.yaml')
    rl_cfg = os.path.join(zed_wrapper_share, 'config', 'zedx_rear_left.yaml')
    rr_cfg = os.path.join(zed_wrapper_share, 'config', 'zedx_rear_right.yaml')

    return LaunchDescription([
        make_zed_node('zedx_fl', fl_cfg),
        make_zed_node('zedx_fr', fr_cfg),
        make_zed_node('zedx_rl', rl_cfg),
        make_zed_node('zedx_rr', rr_cfg),
    ])
```

> 여기서는 **robot_state_publisher를 전혀 띄우지 않았다**는 점이 중요함.
> `zed_wrapper_node` 자체가 카메라 내부 frame(예: `zedx_fl_camera_link`, `zedx_fl_left_camera_optical`)은 TF로 내보내지만, map/odom/base_link 계열은 우리가 위에서 끈 상태라 SLAM이랑 충돌 안 나.

### 7-2. 빌드 & 실행

```bash
cd ~/zed_ws
colcon build --packages-select zed_multi_launch
source install/setup.bash

ros2 launch zed_multi_launch zedx_quad.launch.py
```

---

## 8. 토픽/TF 확인 – 정말 IMU/odom/TF가 안 나오는지 체크

다른 터미널에서:

```bash
source ~/zed_ws/install/setup.bash

ros2 topic list | grep zedx_fl
```

정상이라면 대략 이런 느낌의 토픽만 있어야 함:

* `/zedx_fl/zed_node/rgb/color/rect/image`
* `/zedx_fl/zed_node/rgb/color/rect/camera_info`
* `/zedx_fl/zed_node/depth/depth_registered`
* `/zedx_fl/zed_node/depth/depth_info`

그리고 아래와 같은 것들은 **안 나와야** 함:

* `/zedx_fl/zed_node/imu/data`, `/imu/data_raw` 등 (우리가 sensors.*를 false 로 함)
* `/zedx_fl/zed_node/odom`, `/pose`, `/path_*`
* `/zedx_fl/zed_node/body_trk/*`, `/obj_det/*`, `/mapping/*` 등

TF도 확인:

```bash
ros2 run tf2_tools view_frames
# 혹은
ros2 topic echo /tf --once
ros2 topic echo /tf_static --once
```

여기서:

* `zedx_fl_camera_link` → `zedx_fl_left_camera_optical` 같은 **카메라 내부 TF**는 존재 (static)
* `map -> odom`, `odom -> base_link` 같은 TF는 없어야 함 (우리가 `pos_tracking_enabled=false`, `publish_tf=false`).

외부 SLAM/robot_state_publisher 노드가 **로봇 전체 TF**(예: `map -> odom -> base_link -> ...`)를 관리하고,
카메라 위치는 너가 따로 `static_transform_publisher`나 로봇 URDF에 붙여서 관리하면 된다.

---

## 9. SLAM 노드와의 연동 팁

1. **카메라 ↔ 로봇 베이스 TF**

   * SLAM이 사용하는 카메라 프레임이 무엇인지 확인 (`zedx_fl_left_camera_optical` 같은 것)([Stereolabs][1])
   * 로봇의 `base_link` 기준으로 카메라 위치를 알면, 예를 들어:

     ```bash
     # 예시: base_link 기준 x=0.2, y=0.0, z=0.8, yaw=0(rad)
     ros2 run tf2_ros static_transform_publisher \
       0.2 0.0 0.8 0 0 0 \
       base_link zedx_fl_left_camera_optical
     ```

2. **SLAM 노드에서 depth/RGB 구독**

   * 예: `/zedx_fl/zed_node/rgb/color/rect/image`, `/zedx_fl/zed_node/depth/depth_registered` 사용.([Stereolabs][1])

3. **여러 카메라를 SLAM에 쓰는 경우**

   * 한 SLAM에 여러 카메라를 직접 넣거나, 카메라마다 별 SLAM 노드를 두고 Fusion 하는 구조 등은 SLAM쪽 설정에 따라 달라짐.
   * 어쨌든 ZED 쪽에서는 오도메트리 관련 건 다 꺼져 있으니 TF 충돌은 안 난다.

---

## 10. 성능 관련 한 줄 요약

* Orin AGX + ZED X 4대면 GPU 로드는 꽤 나올 수 있음.
* 필요 없다면:

  * point cloud 비활성화(위에서 false로 설정)
  * 해상도 `HD720`, FPS 15~30 사이로 조절
  * `depth_mode: NEURAL_LIGHT`로 바꿔서 부하를 줄이는 것도 고려

---

## 정리

1. ZED Link Quad 드라이버 + ZED SDK 설치
2. ROS2 Humble + `zed-ros2-wrapper` 빌드 (JetPack 6.x용 의존성 설치 포함)
3. `common_stereo_quad.yaml` 에서 **pos_tracking, sensors, mapping, object, body, gnss 다 OFF**
4. 각 `zedx_*.yaml` 에서 `camera_name`, `serial_number`, 해상도, FPS 설정
5. `zed_wrapper_node` 만 4개 띄우는 커스텀 Launch (`robot_state_publisher` 없음)
6. `ros2 topic list` / `ros2 topic echo` 로 RGB/Depth만 나오는지, IMU/odom/TF 안 나오는지 확인
7. 외부 SLAM + robot_state_publisher가 로봇 전체 TF 담당

---

원하면 다음 단계로:

* 실제 `common_stereo_quad.yaml`/`zedx_*.yaml` 전체 내용을 한 번에 써 줄 수도 있고
* 네가 쓰고 있는 SLAM 패키지(예: Nav2+Cartographer, Isaac ROS VSLAM 등)에 맞춰서 **카메라 프레임/토픽 매칭**까지 구체적으로 정리해 줄게.

[1]: https://www.stereolabs.com/docs/ros2/zed-node/ "ROS 2 - ZED Stereo Node - Stereolabs"
[2]: https://www.stereolabs.com/docs/development/zed-sdk/jetson?utm_source=chatgpt.com "How to Install ZED SDK on NVIDIA® Jetson"
[3]: https://www.stereolabs.com/docs/ros2/multi_camera/ "Multi-camera setup - Stereolabs"
[4]: https://github.com/stereolabs/zed-ros2-wrapper "GitHub - stereolabs/zed-ros2-wrapper: ROS 2 wrapper for the ZED SDK"
##########################  
############################  
##############################  
`common_stereo.yaml` 파일을 찾으셨군요\! 바로 이 부분이 \*\*"토픽이 리스트에 보이지 않던 원인"\*\*입니다. ZED SDK는 네트워크 대역폭과 CPU 자원을 아끼기 위해, 기본적으로 필수적인 토픽 외에는 모두 `false`로 꺼두기 때문입니다.

각 파라미터가 무엇을 의미하는지, 그리고 \*\*작성자님의 목적(Left/Right 개별 이미지 + Gray 이미지)\*\*에 맞게 어떻게 수정해야 하는지 설명해 드릴게요.

-----

### 1\. 파라미터 상세 설명

| 파라미터 | 의미 및 설명 | 추천 설정 |
| :--- | :--- | :--- |
| **`enable_24bit_output`** | **이미지 데이터의 크기 결정**<br>- `false`: **32-bit BGRA** (투명도 채널 포함). 데이터가 큼.<br>- `true`: **24-bit BGR** (투명도 제거). 데이터가 작음.<br>로봇 비전에서는 투명도(Alpha) 채널이 필요 없으므로 **true**로 설정하여 대역폭을 25% 절약하는 것이 좋습니다. | **true** (강력 추천) |
| **`publish_rgb`** | **메인 컬러 이미지 발행 여부**<br>주로 **Left 카메라의 보정된(Rectified) 컬러 이미지**를 발행합니다. 우리가 흔히 보는 메인 화면입니다. | **true** |
| **`publish_left_right`** | **Left / Right 개별 이미지 발행 여부**<br>- 이게 `true`여야 **Right 카메라의 컬러 이미지**(`.../right/image_rect_color`)가 발행됩니다.<br>- 작성자님이 찾던 \*\*"오른쪽, 왼쪽 이미지 각각 필요해"\*\*를 해결해 주는 옵션입니다. | **true** (필수) |
| **`publish_raw`** | **보정되지 않은(Distorted) 원본 이미지**<br>렌즈 왜곡이 그대로 있는 이미지입니다. 직접 캘리브레이션을 하거나 특수한 VSLAM을 돌리는 게 아니라면 보통 필요 없습니다. | **false** |
| **`publish_gray`** | **Grayscale(흑백) 이미지 발행 여부**<br>- 컬러 이미지를 흑백으로 변환하여 발행합니다.<br>- 작성자님이 찾던 \*\*"grey 이미지도 받고 싶어"\*\*를 해결해 주는 옵션입니다. | **true** (필수) |
| **`publish_stereo`** | **Stereo Pair 이미지**<br>Left와 Right 이미지를 하나로 합쳐서(Side-by-Side) 보냅니다. VR 헤드셋 등에는 유용하지만, 로봇 개발에서는 이미지를 다시 잘라내야 해서 잘 안 씁니다. | **false** |

-----

### 2\. 작성자님의 상황에 맞는 추천 설정

작성자님의 요구사항은 \*\*"Left/Right 각각의 이미지"\*\*와 \*\*"Grayscale 이미지"\*\*를 얻는 것입니다. 또한 ZED X를 사용 중이므로 데이터 전송량이 많을 텐데, 이를 효율적으로 관리해야 합니다.

아래와 같이 `common_stereo.yaml`을 수정하세요.

```yaml
        video:
            saturation: 4 
            sharpness: 4 
            gamma: 8 
            auto_exposure_gain: true 
            exposure: 80 
            gain: 80 
            auto_whitebalance: true 
            whitebalance_temperature: 42 
            
            # [핵심 변경 1] 대역폭 절약을 위해 24bit 사용 (투명도 채널 제거)
            enable_24bit_output: true 

            # 기본 RGB (Left)
            publish_rgb: true 

            # [핵심 변경 2] Right 카메라 이미지 토픽 활성화
            # 이걸 켜면 /zed_multi/.../right/image_rect_color 가 나옵니다.
            publish_left_right: true 

            # 왜곡된 원본은 필요 없으므로 끔
            publish_raw: false 

            # [핵심 변경 3] Grayscale 이미지 토픽 활성화
            # 이걸 켜면 /zed_multi/.../left/image_rect_gray 등이 나옵니다.
            publish_gray: true 

            # Stereo 합친 이미지는 필요 없으므로 끔
            publish_stereo: false 
```

### 3\. 적용 후 확인 방법

위와 같이 수정하고 저장한 뒤, ZED 노드를 재시작(`ctrl+c` -\> `ros2 launch...`) 해주세요.
그다음 터미널에서 토픽 리스트를 확인하면 원하시던 토픽들이 보일 것입니다.

```bash
ros2 topic list
```

**예상되는 추가 토픽들:**

  * `/zed_multi/zed_right/left/image_rect_color` (Left 컬러)
  * `/zed_multi/zed_right/right/image_rect_color` (Right 컬러)
  * `/zed_multi/zed_right/left/image_rect_gray` (Left 흑백)
  * `/zed_multi/zed_right/right/image_rect_gray` (Right 흑백)

### ⚠️ 주의사항 (Expert Tip)

`publish_left_right: true`와 `publish_gray: true`를 모두 켜면, 카메라가 뱉어내는 데이터 양이 **2배\~3배**로 늘어납니다.
ZED X는 해상도가 높기 때문에(FHD or 1200p), 모든 이미지를 다 켜면 **IPC의 CPU 부하가 급증**하거나 \*\*네트워크 지연(Lag)\*\*이 발생할 수 있습니다.

만약 로봇이 버벅거린다면?

1.  **해상도(Resolution) 낮추기:** `common.yaml` 다른 부분에 있는 해상도 설정을 한 단계 낮춥니다.
2.  **FPS 낮추기:** 프레임 레이트를 30fps에서 15fps 정도로 낮춥니다.
3.  **필요한 것만 켜기:** 만약 알고리즘이 Gray 이미지만 쓴다면, `publish_rgb`와 `publish_left_right`를 끄고 Gray만 켤 수도 있습니다.
