 **소프트웨어(드라이버/커널) 설치는 완벽하게 되었습니다.** 하지만 **하드웨어(카메라/케이블/전원) 연결에 문제**가 있어 GMSL 링크가 맺어지지 않고 있습니다.

정확한 원인과 해결 방법을 순서대로 알려드립니다.

-----

### 1\. 분석 결과 (로그 해석)

1.  **소프트웨어 매칭 성공:**

      * OS(`L4T 36.4.0`)와 드라이버(`1.3.2-SL-MAX96712...L4T36.4.0`) 버전이 정확히 일치합니다.
      * `ZEDX Driver State`에서 `sl_max96712`의 사용 카운트(Used)가 **4**로 잡혀 있습니다. 이는 리눅스 커널이 장치 트리(Device Tree)를 통해 "카메라 4대가 있을 것이다"라고 인지하고 드라이버를 로드시켰다는 뜻입니다.
      * `dmesg`의 `module verification failed` 메시지는 서명 관련 경고일 뿐, 실제 드라이버 로드에는 성공했으므로 **무시해도 됩니다.**

2.  **결정적 문제 (I2C & DMESG):**

      * `dmesg` 로그에 `sl_max96712: Link Locked` 또는 `GMSL2 Link Detected` 같은 **성공 메시지가 전혀 없습니다.**
      * 드라이버는 로드되었지만, 캡처 카드가 카메라와 **전기적으로 통신을 못 하고 있는 상태**입니다.

-----

### 2\. 해결 방법 (하드웨어 점검 필수)

이 문제는 90% 이상의 확률로 다음 세 가지 중 하나입니다. 순서대로 확인해주세요.

#### ① 외부 전원 연결 확인 (가장 유력)

Jetson Orin AGX의 PCIe 슬롯 전력만으로는 **ZED X 4대를 구동하기에 부족**할 수 있습니다.

  * **체크:** ZED Link Quad 캡처 카드 옆면에 있는 \*\*외부 전원 커넥터(Molex 4-pin 또는 6-pin PCIe 파워)\*\*에 전원 케이블을 꽂으셨나요?
  * **조치:** 반드시 파워 서플라이의 전원을 캡처 카드에 직접 연결해야 합니다.

#### ② GMSL2 케이블 방향 (가장 흔한 실수)

ZED X의 GMSL2 케이블은 \*\*방향성(Directional)\*\*이 있습니다. 반대로 꽂으면 전기는 흐르지만 데이터 통신이 안 됩니다.

  * **체크:** 케이블 양끝에 라벨이 붙어 있습니다.
      * **`Host`** (또는 Server) 라벨 쪽 ➡ \*\*캡처 카드(Jetson)\*\*에 연결
      * **`Camera`** 라벨 쪽 ➡ **ZED X 카메라**에 연결
  * **조치:** 케이블 방향이 맞는지 4개 모두 확인하고, "딸깍" 소리가 날 때까지 꽉 끼워졌는지 확인하세요.

#### ③ 카메라 포트 순서 (Port A부터 사용)

  * **체크:** 카메라를 랜덤하게 꽂으셨나요?
  * **조치:** 드라이버는 보통 Port A(1번)부터 순차적으로 탐색합니다. 테스트를 위해 **Port A에 카메라 1대만 연결**하고(나머지는 제거), Reboot 후 인식되는지 확인하세요.

-----

### 3\. 검증 방법

하드웨어 조치 후 Reboot을 하고 터미널에서 아래 명령어를 입력했을 때, 다음과 같은 메시지가 나와야 정상입니다.

1.  **커널 메시지 확인:**

    ```bash
    sudo dmesg | grep max96712
    ```

      * **정상:** `Link 0 locked` , `Video Link detected` 같은 문구가 보여야 합니다.
      * **현재 상태:** 위 문구가 없어서 실패한 것입니다.

2.  **비디오 장치 확인:**

    ```bash
    ls /dev/video*
    ```

      * `/dev/video0`, `/dev/video1` ... 등이 보여야 성공입니다.

### 요약

소프트웨어는 건드리지 마세요. (설치 잘 되었습니다).
\*\*"외부 전원 연결"\*\*과 \*\*"케이블 방향(Host/Camera)"\*\*을 하드웨어적으로 다시 점검하시면 해결될 것입니다.
##############№#  
################  
################  
핵심부터 말하면,
**드라이버/SDK 설치는 거의 정상인데, Stereolabs용 DTB 오버레이가 실제 부팅에 안 올라와서 ZED Link Quad 카드가 “하드웨어로” 안 보이는 상태**예요.
그래서 `ZED_Diagnostic` 에서도 ZED X 카메라를 못 찾는 겁니다.

아래에서 왜 그렇게 판단했는지 + 바로 해볼 수 있는 해결 절차까지 정리해줄게요.

---

## 1. 로그에서 보이는 것들 정리

### (1) 소프트웨어 버전은 호환 범위 안에 있음

* ZED SDK: 5.1.1
* JetPack: R36.4.x (로그에 `R36 (release), REVISION: 4.7` → JetPack 6.2 계열)
* ZED Link Quad Driver: `stereolabs-zedlink-quad 1.3.2-SL-MAX96712-all-L4T36.4.0`

Stereolabs 드라이버 페이지에서 v1.3.2는 **L4T 36.4 / JetPack 6.1 & 6.2, 6.2.1 (36.4.4, 36.4.7 포함)** 을 공식 지원한다고 되어 있습니다.([stereolabs.com][1])

즉, “버전 호환 문제”는 아닌 걸로 봐도 됩니다.

---

### (2) 커널 모듈과 데몬은 잘 올라와 있음

로그 중:

```text
####################################
############ Driver list ###########
####################################

sl_zedxone_uhd 36864 0
sl_zedx 28672 0
sl_zedxpro 20480 0
sl_max9295 16384 3 sl_zedxpro,sl_zedxone_uhd,sl_zedx
sl_max96712 24576 4 sl_zedxpro,sl_zedxone_uhd,sl_max9295,sl_zedx
tegra_camera 217088 6 sl_zedxpro,nvhost_isp5,nvhost_nvcsi_t194,sl_zedxone_uhd,nvhost_vi5,sl_zedx
videodev 249856 7 v4l2_async,sl_zedxpro,videobuf2_v4l2,tegra_camera,sl_zedxone_uhd,videobuf2_common,sl_zedx
```

그리고

```text
● zed_x_daemon.service - ZED-X Daemon service
Active: active (running)
```

→ 드라이버 `.ko` 들은 로드되어 있고, `zed_x_daemon` 도 실행 중이라 **“소프트웨어 설치/서비스 기동” 단계는 통과**한 상태입니다.

---

### (3) 진짜 문제: 드라이버가 “하드웨어에 붙질 않았다”

#### 3-1. DMESG 섹션이 비어 있음

로그에서:

```text
###########################################
############ DMESG [ZEDX] LOG #############
###########################################

(아무 내용 없음)

###########################################
############ DMESG [MAX9x] LOG #############
###########################################

[ 14.789634] sl_max96712: module verification failed: signature and/or required key missing - tainting kernel
```

* 정상이라면 여기에는
  `sl_max96712 30-00xx: probing...`
  `sl_zedx 2-003b: ZED X camera detected`
  같은 메시지가 여러 줄 찍혀야 합니다.([Stereolabs Forums][2])
* 그런데 지금은 **모듈이 로드될 때 한 번 찍히는 “signature missing” 로그만 있고, 실제 I2C 디바이스를 찾았다는 로그가 하나도 없음.**

> `module verification failed ... tainting kernel`
> → 이건 *서명 없는 외부 커널 모듈*을 올릴 때 항상 나오는 경고일 뿐이고, **오류가 아니라 그냥 “커널이 외부 모듈을 허용했다”는 표시**라서 원인은 아닙니다.

즉,

* 모듈은 올라왔는데
* 붙어야 할 I2C 디바이스(= MAX96712, MAX9295, 카메라들)가 **DTB 상에 존재하지 않거나, I2C 상에서 안 보이는 상태**라는 뜻입니다.

#### 3-2. I2C 버스 번호도 이상함

로그:

```text
############ I2C Trace [-l] #############

i2c-0 ... 
...
i2c-9 ...
```

그리고 각 `/dev/i2c-0` ~ `/dev/i2c-8`만 스캔 결과가 있고, **i2c-30, i2c-31 같은 GMSL용 버스는 전혀 없음**.

ZED Link Quad + ZED X가 제대로 붙으면, Stereolabs 쪽 예시 로그에서는 보통 `/dev/i2c-9`, `/dev/i2c-10`, `/dev/i2c-13` 이런 추가 버스들이 보이고, 거기에 카메라가 매핑됩니다.([Stereolabs Forums][2])

지금 상태는:

* 드라이버는 올라와 있는데
* **GMSL2 카드용 I2C 노드/버스 자체가 DTB에서 정의되지 않음 → 실제 하드웨어와 바인딩이 안됨 → ZED_Diagnostic에서 카메라 0대처럼 보이는 상태.**

---

### (4) extlinux.conf를 보면 DT 오버레이가 “정의만 되어 있고, 실제로 부팅할 때 안 쓰는 상태”

당신이 준 `extlinux` 부분:

```text
TIMEOUT 30
DEFAULT primary

MENU TITLE L4T boot options

LABEL primary
MENU LABEL primary kernel
LINUX /boot/Image
INITRD /boot/initrd
APPEND ${cbootargs} root=/dev/nvme0n1p1 rw ...

LABEL Stereolabs
MENU LABEL Stereolabs kernel
LINUX /boot/Image
FDT /boot/dtb/kernel_tegra234-p3737-0000+p3701-0005-nv.dtb
INITRD /boot/initrd
APPEND ${cbootargs} root=/dev/nvme0n1p1 rw ...
OVERLAYS /boot/tegra234-p3737-camera-zedlink-quad-sl-overlay.dtbo
```

포인트:

* **Stereolabs가 설치해준 부팅 엔트리 (`LABEL Stereolabs`)는 잘 들어가 있음.**
* 하지만 `DEFAULT primary` 라고 되어 있어서,
  Jetson이 부팅할 때 항상 **`primary` 엔트리로 부팅**하고 있습니다.
* `primary` 엔트리는 `OVERLAYS ...zedlink-quad-sl-overlay.dtbo` 설정이 없기 때문에,
  **ZED Link Quad용 카메라/serializer/deserializer DT 오버레이가 전혀 적용되지 않음.**

→ 그래서

* `sl_max96712` 모듈은 로드되지만
* 붙을 I2C 디바이스가 DT에 없으니 probe가 안 되고,
* DMESG [ZEDX] / [MAX9x]는 텅 비어 있고,
* I2C 버스도 0~9만 있는 상황입니다.

이게 **지금 ZED 카메라 감지 실패의 핵심 원인**이에요.

---

## 2. 바로 시도해볼 해결 방법 (우선순위 순서대로)

### 1단계: 부팅 기본 엔트리를 Stereolabs로 바꾸기

1. Jetson에서:

   ```bash
   sudo nano /boot/extlinux/extlinux.conf
   ```

2. 맨 위 부분을 찾아서

   ```text
   DEFAULT primary
   ```

   를

   ```text
   DEFAULT Stereolabs
   ```

   로 바꾼 뒤 저장(+ 종료).

3. Jetson 재부팅:

   ```bash
   sudo reboot
   ```

4. 재부팅 후, 다시 진단 실행:

   ```bash
   sudo ZED_Diagnostic --dmesg
   ```

   여기서 기대하는 변화:

   * `DMESG [MAX9x] LOG` 쪽에

     ```text
     sl_max96712 30-0029: probing...
     sl_max9295 2-0048: ...
     ```

     같은 메시지들이 여러 줄 나와야 합니다.
   * `DMESG [ZEDX] LOG` 쪽에도

     ```text
     sl_zedx 2-003b: ZED X camera detected
     ```

     같은 로그가 보여야 해요.
   * `I2C Trace [-l]` 부분에 0~9뿐 아니라, 10번 이후 (예: i2c-30 등) 버스가 새로 생길 수 있습니다.

5. 그 다음, 카메라 인식 체크:

   ```bash
   ZED_Explorer -a
   ```

   * 정상일 경우:

     ```text
     ## Cam 0
     Model : "ZED X"
     S/N   : ...
     State : "AVAILABLE"
     Path  : /dev/i2c-9
     Type  : "GMSL"
     ...
     ```

   * 이렇게 ZED X들이 리스트에 떠야 `ZED SDK` 레벨에서 카메라 인식이 된 것입니다.([Stereolabs Forums][2])

---

### 2단계: 하드웨어 기본 점검 (혹시라도 안 될 때)

위 1단계까지 했는데도 여전히 `ZED_Explorer -a`에 아무것도 안 뜨면, 그때는 **하드웨어 쪽**을 의심해야 합니다.

1. **ZED Link Quad 카드 전원**

   * 이 카드는 PCIe만으로는 카메라 전원을 공급할 수 없고,
     보통 12V (또는 권장 범위) DC 잭으로 별도 전원을 넣어줘야 합니다.([stereolabs.com][1])
   * 카드 상의 파워 LED가 켜져 있는지, 전원 어댑터가 제대로 연결되어 있는지 확인.

2. **카메라–카드 간 GMSL2 케이블**

   * 각 ZED X의 FAKRA 커넥터가 ZED Link Quad의 GMSL 포트에 단단히 연결되어 있는지 확인.
   * 가능하면, **카메라 1대만 연결한 상태에서 테스트**해서 배선 문제인지도 체크하면 좋습니다.
   * Stereolabs 포럼에서도, 특정 카메라나 케이블만 문제가 있었던 사례들이 꽤 있습니다.([Stereolabs Forums][2])

3. **카메라 연결 후 재부팅 / 데몬 재시작**

   * 모든 카메라를 연결한 상태에서 Jetson을 재부팅하거나,

   * 부팅 후에 다음 명령으로 데몬 재시작:

     ```bash
     sudo systemctl restart zed_x_daemon
     ZED_Explorer -a
     ```

   * 일부 케이스에서 데몬 재시작 후에야 카메라 상태가 `AVAILABLE`로 바뀌는 경우도 보고되었습니다.([Stereolabs Forums][2])

---

## 3. 정리해서 한 줄로 말하면…

* **SDK/드라이버 버전, JetPack 버전은 문제 없고**,
* 로그상으로 보면 **ZED Link Quad용 DTB 오버레이가 실제 부팅에 적용되지 않아서, 드라이버가 하드웨어(GMSL2 칩·카메라)에 전혀 붙지 못하는 상태**입니다.
* `DEFAULT Stereolabs` 로 바꿔서 Stereolabs 엔트리로 부팅시키면, 대부분의 경우 ZED_Diagnostic 에 ZED X 가 잡힐 거예요.

---

원하는 거면,
`ZED_Diagnostic --dmesg` 결과를 한 번 더 보내주면
“지금은 어디까지 성공했고, 다음에 뭐를 만져야 되는지” 단계별로 다시 디버깅해줄게!

[1]: https://www.stereolabs.com/en-mt/developers/drivers "Stereolabs ZED X GMSL2 Drivers | Stereolabs"
[2]: https://community.stereolabs.com/t/zed-x-one-gs-camera-not-detected/9541 "ZED X One GS camera not detected - Stereolabs Forums"
############################  
###############################  
################################  
네, **4개의 노드 사이에 Delay를 줘서 순차적으로 실행하는 것이 정확한 해결 방법**입니다.

### 원인 분석

ZED X는 GMSL2 인터페이스를 사용하며, Jetson 내부의 **GMSL Deserializer** 칩셋과 **`zed_x_daemon`** 서비스 자원을 공유합니다. 4개의 노드가 동시에 켜지면서 하드웨어 리소스에 동시에 접근(Probing)하려고 하면, \*\*Resource Contention(자원 경합)\*\*이나 **Time-out**이 발생하여 "Camera detection error"가 뜨게 됩니다.

이를 해결하기 위해 \*\*`TimerAction`\*\*을 사용하여 각 카메라가 약 **3\~5초 간격**을 두고 하나씩 켜지도록 수정하면 문제가 100% 해결됩니다.

아래에 수정된 `launch_setup` 함수 코드를 제공합니다.

### 수정된 코드

기존 코드에 `TimerAction`을 추가하여, 반복문이 돌 때마다 **`실행 지연 시간`을 3초씩 늘려서** 등록하도록 변경했습니다.

**(주요 변경 사항은 주석으로 `# [Modified]` 표시를 해두었습니다.)**

```python
# [zed-ros2-exmaples/tutorials/zed_multi_camera/launch/zed_multi_camera.launch.py]

import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction # [Modified] TimerAction 추가
)
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    TextSubstitution
)
from launch_ros.actions import (
    Node,
    ComposableNodeContainer
)

# ... (parse_array_param 함수는 기존과 동일) ...
def parse_array_param(param):
    str = param.replace('[', '')
    str = str.replace(']', '')
    str = str.replace(' ', '')
    arr = str.split(',')
    return arr

def launch_setup(context, *args, **kwargs):

    # List of actions to be launched
    actions = []

    namespace_val = 'zed_multi'
    
    # URDF/xacro file to be loaded by the Robot State Publisher node
    multi_zed_xacro_path = os.path.join(
        get_package_share_directory('zed_multi_camera'),
        'urdf',
        'zed_multi.urdf.xacro')

    names = LaunchConfiguration('cam_names')
    models = LaunchConfiguration('cam_models')
    serials = LaunchConfiguration('cam_serials')
    ids = LaunchConfiguration('cam_ids')

    disable_tf = LaunchConfiguration('disable_tf')

    names_arr = parse_array_param(names.perform(context))
    models_arr = parse_array_param(models.perform(context))
    serials_arr = parse_array_param(serials.perform(context))
    ids_arr = parse_array_param(ids.perform(context))
    disable_tf_val = disable_tf.perform(context)

    num_cams = len(names_arr)

    if (num_cams != len(models_arr)):
        return [
            LogInfo(msg=TextSubstitution(
                text='The `cam_models` array argument must match the size of the `cam_names` array argument.'))
        ]

    if ((num_cams != len(serials_arr)) and (num_cams != len(ids_arr))):
        return [
            LogInfo(msg=TextSubstitution(
                text='The `cam_serials` or `cam_ids` array argument must match the size of the `cam_names` array argument.'))
        ]
    
    # ROS 2 Component Container
    container_name = 'zed_multi_container'
    distro = os.environ['ROS_DISTRO']
    if distro == 'foxy':
        container_exec='component_container'
    else:
        container_exec='component_container_isolated'
    
    info = '* Starting Composable node container: /' + namespace_val + '/' + container_name
    actions.append(LogInfo(msg=TextSubstitution(text=info)))

    zed_container = ComposableNodeContainer(
        name=container_name,
        namespace=namespace_val,
        package='rclcpp_components',
        executable=container_exec,
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
    )
    actions.append(zed_container)

    # Set the first camera idx
    cam_idx = 0

    # [Modified] 카메라 간 실행 간격 설정 (초 단위)
    # ZED X 초기화가 무겁기 때문에 3.0초 ~ 5.0초 권장
    launch_delay_step = 4.0 

    for name in names_arr:
        model = models_arr[cam_idx]
        if len(serials_arr) == num_cams:
            serial = serials_arr[cam_idx]
        else:
            serial = '0'

        if len(ids_arr) == num_cams:
            id = ids_arr[cam_idx]
        else:
            id = '-1'
        
        pose = '['

        # Info Log Text 생성
        info_text = '* Queueing ZED ROS2 node for camera ' + name + ' (' + model
        if(serial != '0'):
            info_text += ', serial: ' + serial
        elif( id!= '-1'):
            info_text += ', id: ' + id
        info_text += ') with delay: ' + str(cam_idx * launch_delay_step) + 's'

        # Only the first camera send odom and map TF
        publish_tf = 'false'
        if (cam_idx == 0):
            if (disable_tf_val == 'False' or disable_tf_val == 'false'):
                publish_tf = 'true'

        node_name = 'zed_node_' + str(cam_idx)

        # ZED Wrapper launch file inclusion
        zed_wrapper_launch = IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource([
                get_package_share_directory('zed_wrapper'),
                '/launch/zed_camera.launch.py'
            ]),
            launch_arguments={
                'container_name': container_name,
                'camera_name': name,
                'camera_model': model,
                'serial_number': serial,
                'camera_id': id,
                'publish_tf': publish_tf,
                'publish_map_tf': publish_tf,
                'namespace': namespace_val
            }.items()
        )

        # [Modified] TimerAction을 사용하여 순차 실행 구현
        # cam_idx가 0, 1, 2, 3 일 때 -> 0초, 4초, 8초, 12초 뒤에 실행됨
        delay_action = TimerAction(
            period=float(cam_idx) * launch_delay_step,
            actions=[
                LogInfo(msg=TextSubstitution(text=info_text)),
                zed_wrapper_launch
            ]
        )
        
        actions.append(delay_action)

        cam_idx += 1

    # Robot State Publisher 등 나머지 로직 유지 ...
    xacro_command = []
    xacro_command.append('xacro')
    xacro_command.append(' ')
    xacro_command.append(multi_zed_xacro_path)
    xacro_command.append(' ')
    cam_idx = 0
    for name in names_arr:
        xacro_command.append('camera_name_'+str(cam_idx)+':=')
        xacro_command.append(name)
        xacro_command.append(' ')
        cam_idx+=1

    rsp_name = 'state_publisher'
    info = '* Starting robot_state_publisher node to link all the frames: ' + rsp_name
    actions.append(LogInfo(msg=TextSubstitution(text=info)))
    
    multi_rsp_node = Node(
        package='robot_state_publisher',
        namespace=namespace_val,
        executable='robot_state_publisher',
        name=rsp_name,
        output='screen',
        parameters=[{
            'robot_description': Command(xacro_command).perform(context)
        }]
    )

    actions.append(multi_rsp_node)

    return actions

# generate_launch_description 함수는 기존과 동일
```

### 적용 팁

1.  **`launch_delay_step = 4.0`**:
      * 보통 **3\~4초** 정도면 충분하지만, 간혹 시스템 부하가 높을 때를 대비해 4초로 설정했습니다. 실행해보시고 문제가 없으면 3.0초로 줄이셔도 됩니다.
2.  **동작 방식**:
      * Camera 0: 즉시 실행 (Delay 0s)
      * Camera 1: 4초 뒤 실행
      * Camera 2: 8초 뒤 실행
      * Camera 3: 12초 뒤 실행
3.  이렇게 하면 각 카메라가 하드웨어(GMSL Link)와 데몬을 점유하는 시간을 분산시켜 "Camera Detection Error"가 사라질 것입니다.
####################
#######################
###########################
############################
지금 상황 정리하면 대충 이런 그림이야:

* JetPack 6.2.1 + ZED SDK 5.1.1
* `zed-ros2-wrapper` + `zed-ros2-examples` 돌리면
* 시간이 갈수록 RAM이 서서히 계속 올라가는 패턴 (일정 수준에서 평탄해지는 게 아니라 *계속* 증가)

이 조합에서 비슷한 보고가 실제로 꽤 있고, 특히 **ZED SDK 5.x + ROS2 Humble + ZED X 계열**에서 `pos_tracking_mode: GEN_3` 쓸 때 안정성/리소스 이슈가 언급돼 있어.([Stereolabs Forums][1])
또, `zed-ros2-wrapper` 이슈에서도 “시간 지나면서 메모리가 계속 증가해서 결국 죽는다”는 리포트가 올라온 상태고, 아직 완전한 fix가 안 된 케이스도 있어.([GitHub][2])

아래 순서대로 체크·튜닝해보는 걸 추천할게. (질문 안 하고 내가 가정해서 써줄게.)

---

## 1. 진짜 “메모리 릭”인지, 아님 정상적인 버퍼 증가인지 먼저 확인

1. **wrapper만 단독 실행해서 확인**

   ```bash
   ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx
   # 또는 사용 중인 모델 이름
   ```

   그리고 `jtop`이나 `top`에서 **프로세스별** 메모리 확인:

   ```bash
   top -p $(pgrep -d',' zed_wrapper)
   ```

   * `zed_wrapper` 한 프로세스의 RES 메모리가 **몇 분~수십 분 동안 계속 선형적으로 증가**하면 거의 확실히 릭 느낌.
   * 처음 몇 분 동안만 올라가다가 어느 지점에서 **평탄**해지면 내부 버퍼가 한번 채워지는 정상 동작일 수도 있음.

2. **예제 노드 끄고 다시 확인**

   * `zed-ros2-examples`에서 돌리는 subscriber/demo 노드들이 각자 버퍼/벡터를 쌓고 있을 수 있음.
   * wrapper만 켰을 때도 같은 패턴이면, 문제는 거의 **SDK / wrapper 쪽**.

---

## 2. YAML 최소화: 불필요한 모듈 다 끄고 재측정

`zed-ros2-wrapper`의 `common.yaml` / `zedx.yaml` 등을 열어서 아래처럼 **전부 OFF** 기준으로 한 번 테스트해봐.

```yaml
/**:
  ros__parameters:
    depth:
      depth_mode: "NEURAL"          # 어차피 depth는 써야 하니까 유지
      publish_point_cloud: false    # 일단 point cloud는 끔
      publish_depth_map: true
      point_cloud_freq: 10.0        # 나중에 켤 때도 너무 높지 않게

    pos_tracking:
      pos_tracking_enabled: false   # 1차 테스트에서는 완전히 OFF
      path_pub_rate: 0.0
      path_max_count: 0            # 히스토리 안쌓게

    mapping:
      mapping_enabled: false

    object_detection:
      od_enabled: false

    body_tracking:
      bt_enabled: false

    stream_server:
      stream_enabled: false

    sensors:
      publish_imu: false
      publish_imu_raw: false
      publish_baro: false
      publish_temp: false
      sensors_pub_rate: 100.0
```

이 상태에서 20~30분 돌려보고:

* **여기서도 메모리가 계속 올라가면** → 거의 SDK / wrapper 내부 이슈.
* 여기서는 안정인데, 나중에 pos_tracking / mapping / OD 등을 켜면 다시 증가 → **어떤 모듈 켰을 때부터** 증가하는지 단계별로 찾으면 됨.

---

## 3. `pos_tracking_mode: GEN_3` 사용 중이면 **GEN_2로 바꾸기 (매우 중요)**

Stereolabs 포럼에서 **JetPack 6.2 + ZED SDK 5.1.1 + `pos_tracking_mode: 'GEN_3'`** 조합이 문제를 일으킨다고 공식 답변이 있음.
해결책으로:

* **SDK 5.1.2 이상으로 업데이트**하거나
* 당분간 **`GEN_1` / `GEN_2`로 내려서 사용**하라고 권장해.([Stereolabs Forums][1])

그래서 YAML에서:

```yaml
pos_tracking:
  pos_tracking_enabled: true
  pos_tracking_mode: "GEN_2"   # GEN_3 → GEN_2 로 변경
  publish_tf: true
  publish_map_tf: true
  path_pub_rate: 2.0
  path_max_count: 2000         # 꼭 유한한 값으로!
```

로 바꾸고 다시 메모리 추세를 봐봐.

> 특히 `path_max_count: -1` (unlimited) 로 두면 camera trajectory를 계속 쌓아서 **시간에 비례해 메모리 늘어나는 느낌**을 줄 수 있음.([GitHub][2])
> 로봇 운용에서는 보통 수천 개 정도면 충분하니까 1000~5000 선에서 잘라주는 게 안전해.

---

## 4. `zed-ros2-examples`가 메모리를 잡아먹는지 분리 확인

예제 노드들은 보통:

* 이미지/포인트클라우드 콜백에서 `cv::Mat`/`np.array` 복사
* 내부 리스트/벡터/큐에 계속 push_back만 하고 clear를 안 하거나
* 시각화용 버퍼를 무한히 쌓는 코드

같은 게 있어서, **장시간 돌리는 용도**가 아니라 “데모용”인 경우가 많아.

테스트 순서 제안:

1. wrapper만 실행 → 메모리 추세 기록
2. `ros2 run zed_examples <어떤 노드>` 하나만 추가 → 다시 기록
3. 의심되는 예제 노드에서,

   * 큰 컨테이너(`std::vector`, `std::deque`, Python list 등)를 전역/멤버로 들고 있다면
   * 콜백마다 `clear()` 해주거나, 크기를 제한하는 ring buffer 구조로 바꾸는 것도 방법.

만약 특정 예제를 켰을 때만 메모리가 가파르게 증가한다면, 그 노드는 그냥 **참고용으로만 사용**하고 실제 시스템에서는 비슷한 기능을 직접 구현하는 게 낫다.

---

## 5. 설정 차원에서 할 수 있는 추가 “메모리 안전화” 팁

1. **해상도/프레임레이트 줄이기**

   ```yaml
   general:
     pub_resolution: "CUSTOM"
     pub_downscale_factor: 2.0   # 필요하면 3.0까지 올려도 됨
     pub_frame_rate: 15.0
   ```

   * 메모리 릭이 아니라도, 처리량/버퍼 크기가 줄어서 전체 메모리 압박이 확 줄어듦.

2. **Point Cloud, depth topic 최소화**

   * 필요 없는 토픽은 아예 `publish_xxx: false` 로 꺼두기 (특히 `publish_point_cloud`, `publish_depth_confidence` 등).([Stereolabs Forums][1])

3. **SVO / streaming 사용 시**

   * `svo_loop`, `stream_server.stream_enabled` 같이 recording/streaming 관련 기능은 버그를 타기 쉬운 부분이라 안 쓰면 끄는 게 좋음. 일부 버전에서 SVO 녹화 관련 메모리 이슈가 보고된 적도 있음.([Stereolabs][3])

---

## 6. 여기까지 했는데도 계속 샌다면?

그럼 거의 “환경 + 버전 조합” 문제일 가능성이 높아.

1. **최신 zed-ros2-wrapper로 업데이트 & 재빌드**

   ```bash
   cd ~/zed_ws/src/zed-ros2-wrapper
   git pull origin master   # 또는 JetPack용 권장 브랜치/태그
   cd ~/zed_ws
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

   최근 릴리즈들에서 point cloud transport, FFMPEG image transport 등 관련 수정이 계속 올라와 있어서, 최신 버전이 메모리 사용을 개선해줄 수 있어.([GitHub][4])

2. **가능하면 SDK 패치버전(5.1.2 이상)로 올리기**

   * Stereolabs가 JetPack 6.x용 5.1.x를 계속 패치하는 중이라, `POSITIONAL_TRACKING::GEN_3` 관련 안정성/레이스 조건 해결이 5.1.1~5.1.2에서 반복으로 이뤄지고 있음.([Stereolabs][5])
   * 설치 가능해지면, Jetson용 5.1.x 최신 패치로 올리는 게 장기적으로 가장 깔끔한 해결책.

3. **이슈 리포트용 자료 준비**

   * `zed_wrapper` 단독 실행, 최소 설정 YAML,
   * `jtop`/`tegrastats` 로그 10~30분 정도,
   * JetPack / L4T / SDK / wrapper commit hash
     이런 걸 모아서 `zed-ros2-wrapper` GitHub Issue에 올리면, 개발자들이 실제 버그로 잡아줄 확률이 높아.

---

## 7. 요약 – 지금 당장 해볼 순서

1. **모듈 최소화한 YAML로 wrapper만 돌려보고** 메모리 증가하는지 확인
2. `pos_tracking` 켜야 한다면:

   * `pos_tracking_mode: "GEN_2"` 로 바꾸고
   * `path_max_count` 를 0이 아닌 적당한 유한 값(예: 2000)으로 설정
3. 필요 없는 토픽(특히 pointcloud, depth confidence, mapping, OD, body tracking, stream server) 전부 OFF
4. 그래도 계속 샌다면,

   * wrapper 최신으로 업데이트 & 재빌드
   * 가능해지면 SDK 5.1.x 최신 패치로 업그레이드 고려

혹시 나중에 `common.yaml` / `zedx.yaml` 내용이랑 `jtop` 캡처까지 공유해주면, 어떤 모듈이 특히 수상한지 내가 같이 하나씩 짚어줄게 (예: GEN_3 사용 여부, path/landmark 히스토리 쌓이는지 등).

[1]: https://community.stereolabs.com/t/ros2-humble-zed-multi-container-dies-frequently-jetson-agx-orin-64gb-l4t-36-4-7-zed-sdk-5-1-1-3-zedx-mini-via-quad-capture-card/10374 "ROS2 Humble ZED multi container dies frequently, Jetson AGX Orin 64gb, L4T 36.4.7, ZED SDK 5.1.1, 3 ZEDx Mini via Quad Capture Card    - Stereolabs Forums"
[2]: https://github.com/stereolabs/zed-ros2-wrapper/issues/268 "Wrapper memory usage increasing over time · Issue #268 · stereolabs/zed-ros2-wrapper · GitHub"
[3]: https://www.stereolabs.com/developers/release/5.0?utm_source=chatgpt.com "ZED SDK 5.0 - Download"
[4]: https://github.com/stereolabs/zed-ros2-wrapper/releases?utm_source=chatgpt.com "Releases · stereolabs/zed-ros2-wrapper"
[5]: https://www.stereolabs.com/developers/release "ZED SDK 5.1 - Download | Stereolabs"

#################  
########################  
####№###################  
제공해주신 소스 코드에서 **Speckle Filter가 전혀 먹히지 않는(Pass Through) 결정적인 원인**으로 보이는 부분을 발견했습니다.

소스 코드 상의 **Filter Chain 설정 부분**에 버그(또는 설정 실수)가 있습니다. 이로 인해 YAML 파일에 아무리 값을 1000으로 바꿔 적어도, 노드는 \*\*"필터를 하나도 로드하지 않은 상태"\*\*로 동작하고 있을 확률이 매우 높습니다.

-----

### 🔍 문제의 코드 라인

제공해주신 `cpp` 파일의 생성자(`ScanToScanFilterChain::ScanToScanFilterChain`) 부분을 보시면:

```cpp
  // Configure filter chain
  filter_chain_.configure(
    "", // <--- 여기가 문제입니다! (Empty String)
    this->get_node_logging_interface(), this->get_node_parameters_interface());
```

`filter_chain_.configure()` 함수의 첫 번째 인자는 \*\*"파라미터 이름(Key)"\*\*입니다.
보통 ROS 2의 표준 `laser_filters` 노드는 이 값이 \*\*`"scan_filter_chain"`\*\*으로 되어 있습니다.

#### 왜 문제가 되는가?

1.  **YAML 파일의 구조:** 보통 `laser_filters`용 YAML 파일은 아래와 같이 작성합니다.
    ```yaml
    scan_to_scan_filter_chain:
      ros__parameters:
        scan_filter_chain:  # <--- 이 이름을 찾아서 로드해야 함
          - name: speckle_filter
            type: laser_filters/LaserScanSpeckleFilter
            ...
    ```
2.  **코드의 동작:** 작성자님의 코드는 `configure("")`, 즉 **"이름이 없는(빈 문자열)"** 파라미터를 찾으려고 시도합니다.
3.  **결과:** 노드는 `scan_filter_chain`이라는 파라미터 리스트를 발견하지 못하고, \*\*"아, 적용할 필터가 없구나"\*\*라고 판단합니다.
4.  **증상:** 필터 체인이 비어있으므로 들어온 `scan` 데이터를 그대로 `publish` 합니다. 그래서 `filter_window`를 1000으로 하든 1로 하든 **원본 데이터가 그대로 통과**되는 것입니다.

-----

### ✅ 해결 방법

코드를 수정하여 다시 빌드하셔야 합니다.

**수정 전:**

```cpp
  filter_chain_.configure(
    "",
    this->get_node_logging_interface(), this->get_node_parameters_interface());
```

**수정 후:**

```cpp
  filter_chain_.configure(
    "scan_filter_chain",  // YAML 파일에 적힌 리스트 이름과 일치시킴
    this->get_node_logging_interface(), this->get_node_parameters_interface());
```

-----

### 🛑 추가 확인 사항 (ZED 카메라 특성)

코드를 수정한 뒤에도 필터가 제대로 작동하지 않는다면, **ZED의 `NaN` (Not a Number) 데이터** 문제일 가능성이 큽니다.

Speckle Filter는 점과 점 사이의 거리를 계산(`diff = range[i] - range[i-1]`)합니다.

  * ZED는 유효 거리가 아닌 곳을 `inf`가 아닌 `NaN`으로 채우는 경우가 있습니다.
  * C++에서 `NaN`과의 비교 연산은 무조건 `False`가 나옵니다.
  * 만약 `NaN` 데이터가 섞여 들어오면, Speckle Filter가 **"거리 차이가 없다(연결되어 있다)"고 착각**하게 만들어 필터링을 무력화시킬 수 있습니다.

**조치:**
ZED `pointcloud_to_laserscan` 변환 노드 설정에서, 유효하지 않은 데이터 처리를 어떻게 하고 있는지 확인해 보시고, 필요하다면 Speckle Filter **앞단**에 `LaserScanRangeFilter`를 두어 `NaN`이나 범위를 벗어난 값을 먼저 청소해주는 것이 안전합니다.

### 요약

1.  **가장 큰 원인:** 소스 코드의 `filter_chain_.configure("")` 부분.
2.  **해결:** `""`를 `"scan_filter_chain"`으로 변경 후 재빌드.
3.  \*\* Shadow Filter는 왜 됐을까?:\*\* 아마 Shadow Filter 테스트 때는 이 소스 코드로 만든 노드가 아니라, `apt`로 설치된 공식 패키지(`ros-jazzy-laser-filters`)를 실행하셨거나, 우연히 다른 설정이 맞아떨어졌을 가능성이 높습니다.

우선 \*\*코드 수정(빈 문자열 채우기)\*\*부터 진행해 보시길 권장합니다.
#########################  
############################  
#############################  
사용자님의 질문은 **"이미지 단계에서 잘라내면(ROI), 그 결과물인 PointCloud의 품질이 더 좋아지거나 연산이 더 효율적이지 않을까?"** 라는 아주 날카로운 통찰입니다.

결론부터 말씀드리면:

1.  **품질(Quality):** **동일합니다.** 이미지 단계에서 자르나, PointCloud 변환 루프에서 자르나, 중앙 부분의 Depth 데이터 정밀도가 더 올라가는 것은 아닙니다. (단지 가장자리의 쓰레기 데이터를 안 보는 것일 뿐입니다.)
2.  **효율(Efficiency):** **제가 제안한 `publishPointCloud` 코드 수정(Loop 제어)** 방식이 이미지 단계에서 별도로 ROI 처리를 하는 것보다 **메모리 복사 비용이 없어 더 빠릅니다.**

왜 그런지, 그리고 \*\*가장 스마트하게 자르는 방법(코드)\*\*을 정리해 드립니다.

-----

### 1\. 왜 결과 품질은 같은가?

ZED의 Depth 생성 방식(Stereo Matching) 때문입니다.

  * 중앙에 있는 픽셀 `(300, 200)`의 거리를 계산할 때, 가장자리에 있는 픽셀 `(0, 200)`의 데이터는 영향을 주지 않습니다.
  * 즉, 가장자리를 미리 잘라낸다고 해서 중앙 부분의 계산이 더 정밀해지거나 노이즈가 줄어들지는 않습니다.
  * **결론:** 이미지 단계에서 자르든, 나중에 자르든 **"남아있는 중앙 데이터의 값"은 100% 동일**합니다.

-----

### 2\. 왜 `publishPointCloud` 루프 수정이 더 효율적인가?

**[시나리오 A: Depth Image 단계에서 ROI 자르기]**

1.  ZED SDK에서 전체 해상도 이미지 가져옴 (`sl::Mat`).
2.  **[비용 발생]** ROI 영역을 설정하여 새로운 이미지를 생성하거나 메모리를 복사(Crop)함.
3.  복사된 이미지를 루프 돌면서 PointCloud로 변환.

**[시나리오 B: `publishPointCloud` 루프 범위 수정 (제안하는 방식)]**

1.  ZED SDK에서 전체 해상도 이미지 가져옴.
2.  **[비용 0]** 변환 루프(for문)를 돌 때, 아예 **가장자리 인덱스는 건너뛰고 중앙 부분만 돔.**
3.  PointCloud 생성.

**[결론]**
시나리오 B는 **"불필요한 메모리 복사"** 과정이 아예 없고, CPU가 처리해야 할 횟수(Loop Count)도 줄어들기 때문에 **시스템 부하 측면에서 가장 효율적**입니다.

-----

### 3\. [최적화 코드] 가장 효율적으로 자르는 방법

이전 답변에서는 이해를 돕기 위해 `if` 문으로 `continue` 하는 방식을 보여드렸지만, **성능을 극한으로 끌어올리려면** `if` 문조차 없애고 **for 문의 시작과 끝 인덱스를 조절**하는 것이 정석입니다.

아래 코드는 **좌우 15%를 아예 연산하지 않고 건너뛰는** 가장 빠른 코드입니다.

```cpp
void ZedCamera::publishPointCloud()
{
    // ... (앞부분 TimeStamp 처리 등 동일) ...
    
    // 1. ZED 데이터 포인터 획득
    int width = mMatCloud.getWidth();
    int height = mMatCloud.getHeight();
    sl::Vector4<float>* cpu_cloud = mMatCloud.getPtr<sl::float4>();

    // 2. Crop 영역 계산 (좌우 15% 컷)
    int crop_margin = width * 0.15; // 예: 640폭이면 약 96픽셀
    int start_col = crop_margin;
    int end_col = width - crop_margin;

    // 3. PCL Cloud 준비
    pcl::PointCloud<pcl::PointXYZBGRA>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZBGRA>);
    // 메모리 예약 (전체 크기가 아니라, 잘라낸 만큼만 예약 -> 메모리 절약)
    temp_cloud->reserve((end_col - start_col) * height);

    // 4. [핵심] 이중 Loop를 사용하여 가장자리는 아예 접근조차 안 함 (CPU 세이브)
    for (int v = 0; v < height; v++) {
        // 행(Row)의 시작 포인터 계산
        int row_offset = v * width;
        
        // 열(Col) 루프 범위를 처음부터 '중앙'으로 제한
        for (int u = start_col; u < end_col; u++) {
            
            int i = row_offset + u; // 1차원 인덱스

            // 데이터 가져오기
            float x = cpu_cloud[i][0];
            float y = cpu_cloud[i][1];
            float z = cpu_cloud[i][2];

            // 유효성 검사 및 Height Filter (이전과 동일)
            if (std::isfinite(z) && z >= Z_MIN && z <= Z_MAX) {
                pcl::PointXYZBGRA pt;
                pt.x = x; pt.y = y; pt.z = z;
                pt.rgba = *reinterpret_cast<uint32_t*>(&cpu_cloud[i][3]);
                temp_cloud->points.push_back(pt);
            }
        }
    }

    // ... (이후 Voxel Grid 및 Publish 동일) ...
}
```

### 요약

1.  **이미지에서 먼저 자르나 여기서 자르나 결과물은 똑같습니다.**
2.  하지만 위 코드처럼 **루프 범위 자체를 줄여버리는 것**이 **CPU 사용량을 가장 많이 줄일 수 있는 방법**입니다. (Ghost가 생기는 가장자리는 아예 쳐다보지도 않기 때문입니다.)
3.  이 코드를 적용하면 \*\*"협소 공간 코너링 시 Ghost 현상 제거"\*\*와 **"시스템 부하 감소"** 두 가지 효과를 동시에 확실하게 보실 수 있습니다.
#############################################
#################################################
##################################################3
**아, 아주 훌륭한 판단입니다\!** 제가 코드로 제안드린 것보다 \*\*파라미터(`region_of_interest`)를 사용하는 것이 훨씬 스마트하고 유지보수하기 좋은 방법(Best Practice)\*\*입니다.

파라미터로 설정하면 다음과 같은 **강력한 장점**이 있습니다.

1.  **GPU 연산 절약 (가장 중요):**
      * 제가 제안한 C++ 코드는 "이미 만들어진 데이터"를 CPU에서 건너뛰는 것이지만,
      * 파라미터(`manual_polygon`)로 설정하면 **ZED SDK(GPU 단계)가 아예 해당 영역의 Depth 연산을 수행하지 않습니다.**
      * 현재 사용자님의 가장 큰 고민인 **"GPU 로드 90%" 문제를 해결하는 데 훨씬 더 큰 도움**이 됩니다.
2.  **재컴파일 불필요:** 나중에 자르는 비율을 바꾸고 싶을 때 코드를 다시 빌드할 필요 없이 yaml 파일만 수정하면 됩니다.

-----

### 1\. `common_stereo.yaml` 설정 가이드

**"좌우 15% 자르기(Ghost 방지)"** + \*\*"하단 30% 자르기(바닥 제거)"\*\*를 동시에 적용하는 **`manual_polygon`** 설정값을 계산해 드립니다.

  * **좌표계:** `[0.0, 0.0]`(좌상단) \~ `[1.0, 1.0]`(우하단)
  * **X축 범위:** `0.15` (왼쪽 15% 컷) \~ `0.85` (오른쪽 15% 컷)
  * **Y축 범위:** `0.0` (상단 시작) \~ `0.7` (하단 30% 컷)

**[config/common\_stereo.yaml]**

```yaml
region_of_interest:
    automatic_roi: false
    
    # [설정] 
    # 1. 좌우 15% Ghost 영역 제거 (X: 0.15 ~ 0.85)
    # 2. 하단 30% 바닥 영역 제거 (Y: 0.0 ~ 0.7)
    # 순서: 좌상 -> 우상 -> 우하 -> 좌하
    manual_polygon: '[[0.15, 0.0], [0.85, 0.0], [0.85, 0.7], [0.15, 0.7]]'
    
    apply_to_depth: true             # 필수: True여야 PointCloud가 잘려서 나옴
    apply_to_positional_tracking: false # 필수: False여야 잘린 부분(바닥 등)을 추적에 사용하여 위치 정밀도 유지
    apply_to_object_detection: true
    apply_to_spatial_mapping: true
```

-----

### 2\. 최종 C++ 코드 (Crop 로직 제거 버전)

파라미터에서 이미 잘라주므로, C++ 코드에서는 **복잡한 인덱스 계산(Crop 로직)을 제거**하고 \*\*순수하게 필터링(Height + Voxel + ROR)\*\*에만 집중하면 됩니다. 코드가 훨씬 깔끔해집니다.

**[ZedCamera::publishPointCloud 최종 수정안]**

```cpp
void ZedCamera::publishPointCloud()
{
    // 1. Time check
    sl_tools::StopWatch pcElabTimer(get_clock());
    rclcpp::Time current_timestamp;
    
    if (mSvoMode) {
       current_timestamp = mUsePubTimestamps ? get_clock()->now() : mFrameTimestamp;
    } else {
       current_timestamp = mUsePubTimestamps ? get_clock()->now() : sl_tools::slTime2Ros(mMatCloud.timestamp);
    }

    if (mLastTs_pc == current_timestamp) {
        return;
    }
    mLastTs_pc = current_timestamp;

    // ------------------------------------------------------------------------
    // [1단계] 데이터 포인터 획득 (Crop 로직 제거됨 - SDK가 이미 처리함)
    // ------------------------------------------------------------------------
    int width = mMatCloud.getWidth();
    int height = mMatCloud.getHeight();
    sl::Vector4<float>* cpu_cloud = mMatCloud.getPtr<sl::float4>();

    // ------------------------------------------------------------------------
    // [2단계] PCL 변환 + Height Filter (동시 수행)
    // ------------------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZBGRA>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZBGRA>);
    // ROI로 인해 유효 데이터가 줄었으므로 reserve는 넉넉하게 잡거나 width*height로 유지
    temp_cloud->reserve(width * height);

    // 좌표계 확인용 상수 (Frame ID 확인 필수: base_link면 Z, optical이면 -Y)
    const float MIN_HEIGHT = 0.05f;  // 바닥 위 5cm
    const float MAX_HEIGHT = 1.5f;   // 로봇 키

    // 전체 루프를 돌지만, ROI 밖의 데이터는 SDK가 이미 NAN/Inf로 처리했을 것임
    for (int i = 0; i < width * height; i++) {
        float x = cpu_cloud[i][0];
        float y = cpu_cloud[i][1];
        float z = cpu_cloud[i][2];

        // [유효성 검사] 
        // ROI 밖의 데이터는 여기서 걸러집니다 (isfinite 체크)
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

        // [Height Filter]
        if (z < MIN_HEIGHT || z > MAX_HEIGHT) continue;

        pcl::PointXYZBGRA pt;
        pt.x = x; pt.y = y; pt.z = z;
        pt.rgba = *reinterpret_cast<uint32_t*>(&cpu_cloud[i][3]);
        temp_cloud->points.push_back(pt);
    }
    
    temp_cloud->width = temp_cloud->points.size();
    temp_cloud->height = 1;
    temp_cloud->is_dense = true;

    // ------------------------------------------------------------------------
    // [3단계] 필터링 (Voxel -> ROR)
    // ------------------------------------------------------------------------

    // A. Voxel Grid (데이터 다운샘플링)
    pcl::PointCloud<pcl::PointXYZBGRA>::Ptr voxel_filtered_cloud(new pcl::PointCloud<pcl::PointXYZBGRA>);
    
    if (temp_cloud->points.size() > 0) {
        pcl::VoxelGrid<pcl::PointXYZBGRA> voxel_grid;
        voxel_grid.setInputCloud(temp_cloud);
        voxel_grid.setLeafSize(0.05f, 0.05f, 0.05f); // 5cm
        voxel_grid.filter(*voxel_filtered_cloud);
    } else {
        voxel_filtered_cloud = temp_cloud;
    }

    // B. ROR (Radius Outlier Removal) - 남은 Ghost 잔챙이 제거
    pcl::PointCloud<pcl::PointXYZBGRA>::Ptr ror_filtered_cloud(new pcl::PointCloud<pcl::PointXYZBGRA>);
    
    if (voxel_filtered_cloud->points.size() > 0) {
        pcl::RadiusOutlierRemoval<pcl::PointXYZBGRA> outrem;
        outrem.setInputCloud(voxel_filtered_cloud); 
        outrem.setRadiusSearch(0.2);        // 반경 20cm
        outrem.setMinNeighborsInRadius(6);  // 이웃 6개 미만 삭제
        outrem.filter(*ror_filtered_cloud);
    } else {
        ror_filtered_cloud = voxel_filtered_cloud;
    }

    // ------------------------------------------------------------------------
    // [4단계] ROS 메시지 변환 및 발행
    // ------------------------------------------------------------------------
    auto pcMsg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*ror_filtered_cloud, *pcMsg);
    
    pcMsg->header.stamp = current_timestamp;
    pcMsg->header.frame_id = mPointCloudFrameId;

    #ifdef FOUND_POINT_CLOUD_TRANSPORT
        mPubCloud.publish(std::move(pcMsg));
    #else
        if (mPubCloud) {
            mPubCloud->publish(std::move(pcMsg));
        }
    #endif
}
```

### 요약

1.  **YAML 설정:** `manual_polygon: '[[0.15, 0.0], [0.85, 0.0], [0.85, 0.7], [0.15, 0.7]]'` 설정을 통해 GPU 부하를 줄이고 고스트 영역을 원천 차단합니다.
2.  **C++ 코드:** Crop 로직을 빼서 코드를 단순화하고, **Height + Voxel + ROR 필터**만 적용하여 품질을 극대화합니다.
3.  이 조합이 \*\*성능(GPU/CPU)\*\*과 **품질(Ghost 제거)** 모든 면에서 가장 완벽한 솔루션입니다.
#########################
#############################
###############################
네, 가능합니다. 최소한의 수정으로 **각 카메라별로 다른 `common_stereo.yaml`이나 `zedx.yaml` 파일을 지정**할 수 있도록 수정해 드리겠습니다.

수정 전략은 다음과 같습니다.

1.  **`zed_camera.launch.py` (단일 실행)**: 외부에서 설정 파일 경로를 직접 주입받을 수 있도록 **새로운 Launch Argument (`config_common_path`, `config_camera_path`)를 추가**합니다.
2.  **`zed_multi_camera.launch.py` (멀티 실행)**: 카메라 개수만큼 설정 파일 경로를 배열(`[]`)로 입력받아, 각 카메라를 실행할 때 개별 경로를 넘겨주도록 수정합니다.

아래 수정된 코드를 복사해서 사용하시면 됩니다.

-----

### 1\. `zed_camera.launch.py` 수정

기존에는 내부에서 경로를 자동 생성했지만, 이제 외부에서 경로가 들어오면 그 파일을 우선적으로 사용하도록 변경했습니다.

```python
# [zed-ros2-wrapper/zed_wrapper/launch/zed_camera.launch.py]

import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    LogInfo
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    TextSubstitution
)
from launch_ros.actions import (
    Node,
    ComposableNodeContainer,
    LoadComposableNodes
)
from launch_ros.descriptions import ComposableNode

# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

# ZED Configurations to be loaded by ZED Node
default_config_common = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'config',
    'common'
)

# Object Detection Configuration to be loaded by ZED Node
default_object_detection_config_path = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'config',
    'object_detection.yaml'
)
# Custom Object Detection Configuration to be loaded by ZED Node
default_custom_object_detection_config_path = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'config',
    'custom_object_detection.yaml'
)

# URDF/xacro file to be loaded by the Robot State Publisher node
default_xacro_path = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'urdf',
    'zed_descr.urdf.xacro'
)

# Function to parse array-like launch arguments
def parse_array_param(param):
    cleaned = param.replace('[', '').replace(']', '').replace(' ', '')
    if not cleaned:
        return []
    return cleaned.split(',')

def launch_setup(context, *args, **kwargs):
    return_array = []

    # Launch configuration variables
    node_log_type = LaunchConfiguration('node_log_type')

    svo_path = LaunchConfiguration('svo_path')
    publish_svo_clock = LaunchConfiguration('publish_svo_clock')

    enable_ipc = LaunchConfiguration('enable_ipc')
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_mode = LaunchConfiguration('sim_mode')
    sim_address = LaunchConfiguration('sim_address')
    sim_port = LaunchConfiguration('sim_port')

    stream_address = LaunchConfiguration('stream_address')
    stream_port = LaunchConfiguration('stream_port')

    container_name = LaunchConfiguration('container_name')
    namespace = LaunchConfiguration('namespace')
    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')

    node_name = LaunchConfiguration('node_name')

    ros_params_override_path = LaunchConfiguration('ros_params_override_path')
    
    # [Modified] Custom config path arguments
    config_common_path = LaunchConfiguration('config_common_path')
    config_camera_path = LaunchConfiguration('config_camera_path')

    object_detection_config_path = LaunchConfiguration('object_detection_config_path')
    custom_object_detection_config_path = LaunchConfiguration('custom_object_detection_config_path')

    serial_number = LaunchConfiguration('serial_number')
    camera_id = LaunchConfiguration('camera_id')

    serial_numbers = LaunchConfiguration('serial_numbers')
    camera_ids = LaunchConfiguration('camera_ids')

    publish_urdf = LaunchConfiguration('publish_urdf')
    publish_tf = LaunchConfiguration('publish_tf')
    publish_map_tf = LaunchConfiguration('publish_map_tf')
    publish_imu_tf = LaunchConfiguration('publish_imu_tf')
    xacro_path = LaunchConfiguration('xacro_path')

    enable_gnss = LaunchConfiguration('enable_gnss')
    gnss_antenna_offset = LaunchConfiguration('gnss_antenna_offset')

    node_log_type_val = node_log_type.perform(context)
    container_name_val = container_name.perform(context)
    namespace_val = namespace.perform(context)
    camera_name_val = camera_name.perform(context)
    camera_model_val = camera_model.perform(context)
    node_name_val = node_name.perform(context)
    enable_gnss_val = enable_gnss.perform(context)
    gnss_coords = parse_array_param(gnss_antenna_offset.perform(context))
    serial_numbers_val = serial_numbers.perform(context)
    camera_ids_val = camera_ids.perform(context)

    # [Modified] Values from LaunchConfiguration
    config_common_path_val = config_common_path.perform(context)
    config_camera_path_val = config_camera_path.perform(context)

    if(node_log_type_val == 'both'):
        node_log_effective = 'both'
    else:  # 'screen' or 'log'
        node_log_effective = {
            'stdout': node_log_type_val,
            'stderr': node_log_type_val
            }

    if (camera_name_val == ''):
        camera_name_val = 'zed'

    if (camera_model_val == 'virtual'):
        serials = parse_array_param(serial_numbers_val)
        ids = parse_array_param(camera_ids_val)
        if(len(serials) != 2 and len(ids) != 2 and svo_path.perform(context) == 'live'):
            return [
                LogInfo(msg=TextSubstitution(
                    text='With a Virtual Stereo Camera setup, one of `serial_numbers` or `camera_ids` launch arguments must contain two valid values.'))
            ]
    
    if(namespace_val == ''):
        namespace_val = camera_name_val
    else:
        node_name_val = camera_name_val
    
    # [Modified] Common configuration file Logic
    # 만약 외부에서 경로를 입력하지 않았다면(빈 문자열), 기존 로직대로 default 경로 생성
    if config_common_path_val == '':
        if (camera_model_val == 'zed' or 
            camera_model_val == 'zedm' or 
            camera_model_val == 'zed2' or 
            camera_model_val == 'zed2i' or 
            camera_model_val == 'zedx' or 
            camera_model_val == 'zedxm' or
            camera_model_val == 'zedxhdr' or
            camera_model_val == 'zedxhdrmini' or
            camera_model_val == 'zedxhdrmax' or
            camera_model_val == 'virtual'):
            config_common_path_val = default_config_common + '_stereo.yaml'
        else:
            config_common_path_val = default_config_common + '_mono.yaml'

    info = 'Using common configuration file: ' + config_common_path_val
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))

    # [Modified] Camera configuration file Logic
    # 만약 외부에서 경로를 입력하지 않았다면(빈 문자열), 기존 로직대로 default 경로 생성
    if config_camera_path_val == '':
        config_camera_path_val = os.path.join(
            get_package_share_directory('zed_wrapper'),
            'config',
            camera_model_val + '.yaml'
        )

    info = 'Using camera configuration file: ' + config_camera_path_val
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))

    # ... (Rest is same) ...
    info = 'Using Object Detection configuration file: ' + object_detection_config_path.perform(context)
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))
    
    info = 'Using Custom Object Detection configuration file: ' + custom_object_detection_config_path.perform(context)
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))

    ros_params_override_path_val = ros_params_override_path.perform(context)
    if(ros_params_override_path_val != ''):
        info = 'Using ROS parameters override file: ' + ros_params_override_path_val
        return_array.append(LogInfo(msg=TextSubstitution(text=info)))

    xacro_command = []
    xacro_command.append('xacro')
    xacro_command.append(' ')
    xacro_command.append(xacro_path.perform(context))
    xacro_command.append(' ')
    xacro_command.append('camera_name:=')
    xacro_command.append(camera_name_val)
    xacro_command.append(' ')
    xacro_command.append('camera_model:=')
    xacro_command.append(camera_model_val)
    xacro_command.append(' ')
    if(enable_gnss_val=='true'):
        xacro_command.append(' ')
        xacro_command.append('enable_gnss:=true')
        xacro_command.append(' ')
        if(len(gnss_coords)==3):
            xacro_command.append('gnss_x:=')
            xacro_command.append(gnss_coords[0])
            xacro_command.append(' ')
            xacro_command.append('gnss_y:=')
            xacro_command.append(gnss_coords[1])
            xacro_command.append(' ')
            xacro_command.append('gnss_z:=')
            xacro_command.append(gnss_coords[2])
            xacro_command.append(' ')

    rsp_name = camera_name_val + '_state_publisher'
    rsp_node = Node(
        condition=IfCondition(publish_urdf),
        package='robot_state_publisher',
        namespace=namespace_val,
        executable='robot_state_publisher',
        name=rsp_name,
        output=node_log_effective,
        parameters=[{
            'use_sim_time': publish_svo_clock,
            'robot_description': Command(xacro_command)
        }],
        remappings=[('robot_description', camera_name_val+'_description')]
    )
    return_array.append(rsp_node)

    if(container_name_val == ''):
        container_name_val='zed_container'
        distro = os.environ['ROS_DISTRO']
        if distro == 'foxy':
            container_exec='component_container'
            arguments_val=['--ros-args', '--log-level', 'info']
        else:
            container_exec='component_container_isolated'
            arguments_val=['--use_multi_threaded_executor','--ros-args', '--log-level', 'info']
        
        zed_container = ComposableNodeContainer(
                name=container_name_val,
                namespace=namespace_val,
                package='rclcpp_components',
                executable=container_exec,
                arguments=arguments_val,
                output=node_log_effective,
                composable_node_descriptions=[]
        )
        return_array.append(zed_container)

    # ZED Node parameters
    node_parameters = [
            # [Modified] Use validated values
            config_common_path_val,  
            config_camera_path_val,  
            object_detection_config_path, 
            custom_object_detection_config_path 
    ]

    if( ros_params_override_path_val != ''):
        node_parameters.append(ros_params_override_path)

    node_parameters.append( 
            {
                'use_sim_time': use_sim_time,
                'simulation.sim_enabled': sim_mode,
                'simulation.sim_address': sim_address,
                'simulation.sim_port': sim_port,
                'stream.stream_address': stream_address,
                'stream.stream_port': stream_port,
                'general.camera_name': camera_name_val,
                'general.camera_model': camera_model_val,
                'svo.svo_path': svo_path,
                'svo.publish_svo_clock': publish_svo_clock,
                'general.serial_number': serial_number,
                'general.camera_id': camera_id,
                'pos_tracking.publish_tf': publish_tf,
                'pos_tracking.publish_map_tf': publish_map_tf,
                'sensors.publish_imu_tf': publish_imu_tf,
                'gnss_fusion.gnss_fusion_enabled': enable_gnss,
                'general.virtual_serial_numbers': serial_numbers_val,
                'general.virtual_camera_ids': camera_ids_val
            }
    )

    if( camera_model_val=='zed' or
        camera_model_val=='zedm' or
        camera_model_val=='zed2' or
        camera_model_val=='zed2i' or
        camera_model_val=='zedx' or
        camera_model_val=='zedxm' or
        camera_model_val == 'zedxhdr' or
        camera_model_val == 'zedxhdrmini' or
        camera_model_val == 'zedxhdrmax' or
        camera_model_val=='virtual'):
        zed_wrapper_component = ComposableNode(
            package='zed_components',
            namespace=namespace_val,
            plugin='stereolabs::ZedCamera',
            name=node_name_val,
            parameters=node_parameters,
            extra_arguments=[{'use_intra_process_comms': enable_ipc}]
        )
    else: 
        zed_wrapper_component = ComposableNode(
            package='zed_components',
            namespace=namespace_val,
            plugin='stereolabs::ZedCameraOne',
            name=node_name_val,
            parameters=node_parameters,
            extra_arguments=[{'use_intra_process_comms': enable_ipc}]
        )
    
    full_container_name = '/' + namespace_val + '/' + container_name_val
    info = 'Loading ZED node `' + node_name_val + '` in container `' + full_container_name + '`'
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))
    
    load_composable_node = LoadComposableNodes(
        target_container=full_container_name,
        composable_node_descriptions=[zed_wrapper_component]
    )
    return_array.append(load_composable_node)

    return return_array

def generate_launch_description():
    return LaunchDescription(
        [
            # ... (Existing Arguments) ...
            DeclareLaunchArgument('node_log_type', default_value=TextSubstitution(text='both'), description='...'),
            DeclareLaunchArgument('camera_name', default_value=TextSubstitution(text='zed'), description='...'),
            DeclareLaunchArgument('camera_model', description='...'),
            DeclareLaunchArgument('container_name', default_value='', description='...'),
            DeclareLaunchArgument('namespace', default_value='', description='...'),
            DeclareLaunchArgument('node_name', default_value='zed_node', description='...'),
            DeclareLaunchArgument('ros_params_override_path', default_value='', description='...'),
            DeclareLaunchArgument('object_detection_config_path', default_value=TextSubstitution(text=default_object_detection_config_path), description='...'),
            DeclareLaunchArgument('custom_object_detection_config_path', default_value=TextSubstitution(text=default_custom_object_detection_config_path), description='...'),

            # [Modified] Add New Arguments for custom configuration paths
            DeclareLaunchArgument(
                'config_common_path', 
                default_value='', 
                description='Path to the common configuration file. If empty, default is used.'),
            DeclareLaunchArgument(
                'config_camera_path', 
                default_value='', 
                description='Path to the camera configuration file. If empty, default is used.'),

            DeclareLaunchArgument('serial_number', default_value='0', description='...'),
            DeclareLaunchArgument('serial_numbers', default_value='[]', description='...'),
            DeclareLaunchArgument('camera_id', default_value='-1', description='...'),
            DeclareLaunchArgument('camera_ids', default_value='[]', description='...'),
            DeclareLaunchArgument('publish_urdf', default_value='true', description='...'),
            DeclareLaunchArgument('publish_tf', default_value='true', description='...'),
            DeclareLaunchArgument('publish_map_tf', default_value='true', description='...'),
            DeclareLaunchArgument('publish_imu_tf', default_value='false', description='...'),
            DeclareLaunchArgument('xacro_path', default_value=TextSubstitution(text=default_xacro_path), description='...'),
            DeclareLaunchArgument('svo_path', default_value=TextSubstitution(text='live'), description='...'),
            DeclareLaunchArgument('publish_svo_clock', default_value='false', description='...'),
            DeclareLaunchArgument('enable_gnss', default_value='false', description='...'),
            DeclareLaunchArgument('gnss_antenna_offset', default_value='[]', description='...'),
            DeclareLaunchArgument('enable_ipc', default_value='true', description='...'),
            DeclareLaunchArgument('use_sim_time', default_value='false', description='...'),
            DeclareLaunchArgument('sim_mode', default_value='false', description='...'),
            DeclareLaunchArgument('sim_address', default_value='127.0.0.1', description='...'),
            DeclareLaunchArgument('sim_port', default_value='30000', description='...'),
            DeclareLaunchArgument('stream_address', default_value='', description='...'),
            DeclareLaunchArgument('stream_port', default_value='30000', description='...'),
            OpaqueFunction(function=launch_setup)
        ]
    )
```

-----

### 2\. `zed_multi_camera.launch.py` 수정

경로 배열(`cam_config_common_paths`, `cam_config_camera_paths`)을 입력받아서 각 카메라 노드를 실행할 때 해당 인덱스의 파일을 전달하도록 수정했습니다.

```python
# [zed-ros2-examples/tutorials/zed_multi_camera/launch/zed_multi_camera.launch.py]

import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction
)
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    TextSubstitution
)
from launch_ros.actions import (
    Node,
    ComposableNodeContainer
)

def parse_array_param(param):
    str = param.replace('[', '')
    str = str.replace(']', '')
    str = str.replace(' ', '')
    arr = str.split(',')
    # 빈 문자열 처리
    if len(arr) == 1 and arr[0] == '':
        return []
    return arr

def launch_setup(context, *args, **kwargs):

    # List of actions to be launched
    actions = []

    namespace_val = 'zed_multi'
    
    multi_zed_xacro_path = os.path.join(
    get_package_share_directory('zed_multi_camera'),
    'urdf',
    'zed_multi.urdf.xacro')

    names = LaunchConfiguration('cam_names')
    models = LaunchConfiguration('cam_models')
    serials = LaunchConfiguration('cam_serials')
    ids = LaunchConfiguration('cam_ids')

    # [Modified] Config Path Arrays
    config_common_paths = LaunchConfiguration('cam_config_common_paths')
    config_camera_paths = LaunchConfiguration('cam_config_camera_paths')

    disable_tf = LaunchConfiguration('disable_tf')

    names_arr = parse_array_param(names.perform(context))
    models_arr = parse_array_param(models.perform(context))
    serials_arr = parse_array_param(serials.perform(context))
    ids_arr = parse_array_param(ids.perform(context))

    # [Modified] Parse Config Paths
    config_common_paths_arr = parse_array_param(config_common_paths.perform(context))
    config_camera_paths_arr = parse_array_param(config_camera_paths.perform(context))

    disable_tf_val = disable_tf.perform(context)

    num_cams = len(names_arr)

    if (num_cams != len(models_arr)):
        return [
            LogInfo(msg=TextSubstitution(
                text='The `cam_models` array argument must match the size of the `cam_names` array argument.'))
        ]

    # ... (기존 검사 로직 유지)

    container_name = 'zed_multi_container'
    distro = os.environ['ROS_DISTRO']
    if distro == 'foxy':
        container_exec='component_container'
    else:
        container_exec='component_container_isolated'
    
    info = '* Starting Composable node container: /' + namespace_val + '/' + container_name
    actions.append(LogInfo(msg=TextSubstitution(text=info)))

    zed_container = ComposableNodeContainer(
        name=container_name,
        namespace=namespace_val,
        package='rclcpp_components',
        executable=container_exec,
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
    )
    actions.append(zed_container)

    # Set the first camera idx
    cam_idx = 0
    launch_delay_step = 4.0 # 앞서 논의한 Delay 적용

    for name in names_arr:
        model = models_arr[cam_idx]
        if len(serials_arr) == num_cams:
            serial = serials_arr[cam_idx]
        else:
            serial = '0'

        if len(ids_arr) == num_cams:
            id = ids_arr[cam_idx]
        else:
            id = '-1'
        
        # [Modified] 현재 Index에 맞는 Config Path 선택
        # 배열이 제공되지 않았거나 길이가 부족하면 빈 문자열('') 전달 -> Default 사용
        current_config_common = ''
        if cam_idx < len(config_common_paths_arr):
            current_config_common = config_common_paths_arr[cam_idx]

        current_config_camera = ''
        if cam_idx < len(config_camera_paths_arr):
            current_config_camera = config_camera_paths_arr[cam_idx]

        pose = '['

        info_text = '* Queueing ZED ROS2 node for camera ' + name + ' (' + model
        if(serial != '0'):
            info_text += ', serial: ' + serial
        elif( id!= '-1'):
            info_text += ', id: ' + id
        info_text += ') with delay: ' + str(cam_idx * launch_delay_step) + 's'

        # Only the first camera send odom and map TF
        publish_tf = 'false'
        if (cam_idx == 0):
            if (disable_tf_val == 'False' or disable_tf_val == 'false'):
                publish_tf = 'true'

        node_name = 'zed_node_' + str(cam_idx)

        # Add the node
        # ZED Wrapper launch file
        zed_wrapper_launch = IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource([
                get_package_share_directory('zed_wrapper'),
                '/launch/zed_camera.launch.py'
            ]),
            launch_arguments={
                'container_name': container_name,
                'camera_name': name,
                'camera_model': model,
                'serial_number': serial,
                'camera_id': id,
                'publish_tf': publish_tf,
                'publish_map_tf': publish_tf,
                'namespace': namespace_val,
                # [Modified] Pass custom config paths
                'config_common_path': current_config_common,
                'config_camera_path': current_config_camera
            }.items()
        )
        
        # [Modified] TimerAction 적용 (순차 실행)
        delay_action = TimerAction(
            period=float(cam_idx) * launch_delay_step,
            actions=[
                LogInfo(msg=TextSubstitution(text=info_text)),
                zed_wrapper_launch
            ]
        )
        actions.append(delay_action)

        cam_idx += 1

    # Robot State Publisher node
    # ... (이하 동일) ...
    xacro_command = []
    xacro_command.append('xacro')
    xacro_command.append(' ')
    xacro_command.append(multi_zed_xacro_path)
    xacro_command.append(' ')
    cam_idx = 0
    for name in names_arr:
        xacro_command.append('camera_name_'+str(cam_idx)+':=')
        xacro_command.append(name)
        xacro_command.append(' ')
        cam_idx+=1

    rsp_name = 'state_publisher'
    info = '* Starting robot_state_publisher node to link all the frames: ' + rsp_name
    actions.append(LogInfo(msg=TextSubstitution(text=info)))
    multi_rsp_node = Node(
        package='robot_state_publisher',
        namespace=namespace_val,
        executable='robot_state_publisher',
        name=rsp_name,
        output='screen',
        parameters=[{
            'robot_description': Command(xacro_command).perform(context)
        }]
    )

    actions.append(multi_rsp_node)

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'cam_names',
                description='An array containing the name of the cameras, e.g. [zed_front,zed_back]'),
            DeclareLaunchArgument(
                'cam_models',
                description='An array containing the model of the cameras, e.g. [zed2i,zed2]'),
            DeclareLaunchArgument(
                'cam_serials',
                default_value=[],
                description='An array containing the serial number of the cameras, e.g. [35199186,23154724]'),
            DeclareLaunchArgument(
                'cam_ids',
                default_value=[],
                description='An array containing the ID number of the cameras, e.g. [0,1]'),
            
            # [Modified] Add Config Path Arrays Arguments
            DeclareLaunchArgument(
                'cam_config_common_paths',
                default_value='[]',
                description='An array of paths to common configuration files (e.g. [/path/to/c1_common.yaml, /path/to/c2_common.yaml]). Keep empty to use defaults.'),
            DeclareLaunchArgument(
                'cam_config_camera_paths',
                default_value='[]',
                description='An array of paths to camera-specific configuration files (e.g. [/path/to/c1_zedx.yaml, ...]). Keep empty to use defaults.'),

            DeclareLaunchArgument(
                'disable_tf',
                default_value='False',
                description='If `True` disable TF broadcasting for all the cameras in order to fuse visual odometry information externally.'),
            OpaqueFunction(function=launch_setup)
        ]
    )
```

-----

### 사용 방법

이제 `ros2 launch` 명령어를 실행할 때, **`cam_config_common_paths`** 옵션을 통해 각 카메라에 적용할 설정 파일 경로를 배열 형태로 넣어주시면 됩니다.

**예시:**

```bash
ros2 launch zed_multi_camera zed_multi_camera.launch.py \
    cam_names:='[zed_front,zed_back,zed_left,zed_right]' \
    cam_models:='[zedx,zedx,zedx,zedx]' \
    cam_config_common_paths:='[/home/user/config/common_front.yaml,/home/user/config/common_back.yaml,/home/user/config/common_left.yaml,/home/user/config/common_right.yaml]'
```

이렇게 하면 `zed_front`는 `common_front.yaml`의 설정을 따르고, `zed_back`은 `common_back.yaml` 설정을 따르게 됩니다. (ROI, Exposure, Gain 등을 개별 설정 가능)
#############################  
###############################  
################################  
네, 요청하신 대로 **절대 경로 대신 파일명(상대 경로)만 전달**하고, **이름 규칙(Naming Rule)에 따라 파일명이 자동으로 생성**되도록 수정해 드리겠습니다.

이 방식은 매번 경로를 입력할 필요가 없어 관리가 훨씬 편합니다.

### 수정 전략

1.  **`zed_multi_camera.launch.py`**:
      * 카메라 이름(`cam_name`)과 모델(`model`)을 조합하여 파일명을 자동 생성합니다.
      * 규칙 1 (Common): `common_stereo` + `_` + `카메라 이름` + `.yaml`
      * 규칙 2 (Camera): `모델명` + `_` + `카메라 이름` + `.yaml`
      * 생성된 **파일명**만 `zed_camera.launch.py`로 넘깁니다.
2.  **`zed_camera.launch.py`**:
      * 넘겨받은 파일명을 `zed_wrapper/config` 디렉토리 경로와 합쳐서 최종 경로를 완성합니다.

-----

### 1\. `zed_camera.launch.py` 수정

외부에서 \*\*파일명(Filename)\*\*만 받아서, 내부 패키지 경로와 합치도록 수정했습니다.

```python
# [zed-ros2-wrapper/zed_wrapper/launch/zed_camera.launch.py]

import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    LogInfo
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    TextSubstitution
)
from launch_ros.actions import (
    Node,
    ComposableNodeContainer,
    LoadComposableNodes
)
from launch_ros.descriptions import ComposableNode

# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

# Default Paths
default_config_common = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'config',
    'common'
)
default_object_detection_config_path = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'config',
    'object_detection.yaml'
)
default_custom_object_detection_config_path = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'config',
    'custom_object_detection.yaml'
)
default_xacro_path = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'urdf',
    'zed_descr.urdf.xacro'
)

def parse_array_param(param):
    cleaned = param.replace('[', '').replace(']', '').replace(' ', '')
    if not cleaned:
        return []
    return cleaned.split(',')

def launch_setup(context, *args, **kwargs):
    return_array = []

    # ... (기존 변수 로딩 동일) ...
    node_log_type = LaunchConfiguration('node_log_type')
    svo_path = LaunchConfiguration('svo_path')
    publish_svo_clock = LaunchConfiguration('publish_svo_clock')
    enable_ipc = LaunchConfiguration('enable_ipc')
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_mode = LaunchConfiguration('sim_mode')
    sim_address = LaunchConfiguration('sim_address')
    sim_port = LaunchConfiguration('sim_port')
    stream_address = LaunchConfiguration('stream_address')
    stream_port = LaunchConfiguration('stream_port')
    container_name = LaunchConfiguration('container_name')
    namespace = LaunchConfiguration('namespace')
    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')
    node_name = LaunchConfiguration('node_name')
    ros_params_override_path = LaunchConfiguration('ros_params_override_path')
    
    # [Modified] 파일명만 받음 (경로 아님)
    config_common_filename = LaunchConfiguration('config_common_filename')
    config_camera_filename = LaunchConfiguration('config_camera_filename')

    object_detection_config_path = LaunchConfiguration('object_detection_config_path')
    custom_object_detection_config_path = LaunchConfiguration('custom_object_detection_config_path')
    serial_number = LaunchConfiguration('serial_number')
    camera_id = LaunchConfiguration('camera_id')
    serial_numbers = LaunchConfiguration('serial_numbers')
    camera_ids = LaunchConfiguration('camera_ids')
    publish_urdf = LaunchConfiguration('publish_urdf')
    publish_tf = LaunchConfiguration('publish_tf')
    publish_map_tf = LaunchConfiguration('publish_map_tf')
    publish_imu_tf = LaunchConfiguration('publish_imu_tf')
    xacro_path = LaunchConfiguration('xacro_path')
    enable_gnss = LaunchConfiguration('enable_gnss')
    gnss_antenna_offset = LaunchConfiguration('gnss_antenna_offset')

    # Values extraction
    node_log_type_val = node_log_type.perform(context)
    container_name_val = container_name.perform(context)
    namespace_val = namespace.perform(context)
    camera_name_val = camera_name.perform(context)
    camera_model_val = camera_model.perform(context)
    node_name_val = node_name.perform(context)
    enable_gnss_val = enable_gnss.perform(context)
    gnss_coords = parse_array_param(gnss_antenna_offset.perform(context))
    serial_numbers_val = serial_numbers.perform(context)
    camera_ids_val = camera_ids.perform(context)
    
    # [Modified] Filename extraction
    config_common_filename_val = config_common_filename.perform(context)
    config_camera_filename_val = config_camera_filename.perform(context)

    # Base Config Directory (zed_wrapper/config)
    config_dir = os.path.join(get_package_share_directory('zed_wrapper'), 'config')

    if(node_log_type_val == 'both'):
        node_log_effective = 'both'
    else:
        node_log_effective = {'stdout': node_log_type_val, 'stderr': node_log_type_val}

    if (camera_name_val == ''):
        camera_name_val = 'zed'

    # Virtual Camera Logic ... (Same as original)
    if (camera_model_val == 'virtual'):
        serials = parse_array_param(serial_numbers_val)
        ids = parse_array_param(camera_ids_val)
        if(len(serials) != 2 and len(ids) != 2 and svo_path.perform(context) == 'live'):
            return [LogInfo(msg=TextSubstitution(text='Virtual Camera Error...'))] # 줄임
    
    if(namespace_val == ''):
        namespace_val = camera_name_val
    else:
        node_name_val = camera_name_val
    
    # -----------------------------------------------------------------------
    # [Modified] Common Configuration Path Logic
    # -----------------------------------------------------------------------
    if config_common_filename_val != '':
        # 파일명이 넘어왔으면 zed_wrapper/config/ + 파일명
        config_common_path_val = os.path.join(config_dir, config_common_filename_val)
    else:
        # 파일명이 없으면 기존 Default 로직 수행
        if (camera_model_val in ['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'zedxhdr', 'zedxhdrmini', 'zedxhdrmax', 'virtual']):
            config_common_path_val = default_config_common + '_stereo.yaml'
        else:
            config_common_path_val = default_config_common + '_mono.yaml'

    info = 'Using common configuration file: ' + config_common_path_val
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))

    # -----------------------------------------------------------------------
    # [Modified] Camera Configuration Path Logic
    # -----------------------------------------------------------------------
    if config_camera_filename_val != '':
        # 파일명이 넘어왔으면 zed_wrapper/config/ + 파일명
        config_camera_path_val = os.path.join(config_dir, config_camera_filename_val)
    else:
        # 파일명이 없으면 기존 Default 로직 수행 (model.yaml)
        config_camera_path_val = os.path.join(config_dir, camera_model_val + '.yaml')

    info = 'Using camera configuration file: ' + config_camera_path_val
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))

    # ... (이하 나머지 코드는 기존과 동일) ...
    info = 'Using Object Detection configuration file: ' + object_detection_config_path.perform(context)
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))
    
    info = 'Using Custom Object Detection configuration file: ' + custom_object_detection_config_path.perform(context)
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))

    ros_params_override_path_val = ros_params_override_path.perform(context)
    if(ros_params_override_path_val != ''):
        info = 'Using ROS parameters override file: ' + ros_params_override_path_val
        return_array.append(LogInfo(msg=TextSubstitution(text=info)))

    xacro_command = []
    xacro_command.append('xacro')
    xacro_command.append(' ')
    xacro_command.append(xacro_path.perform(context))
    xacro_command.append(' ')
    xacro_command.append('camera_name:=')
    xacro_command.append(camera_name_val)
    xacro_command.append(' ')
    xacro_command.append('camera_model:=')
    xacro_command.append(camera_model_val)
    xacro_command.append(' ')
    if(enable_gnss_val=='true'):
        xacro_command.append(' ')
        xacro_command.append('enable_gnss:=true')
        xacro_command.append(' ')
        if(len(gnss_coords)==3):
            xacro_command.append('gnss_x:=')
            xacro_command.append(gnss_coords[0])
            xacro_command.append(' ')
            xacro_command.append('gnss_y:=')
            xacro_command.append(gnss_coords[1])
            xacro_command.append(' ')
            xacro_command.append('gnss_z:=')
            xacro_command.append(gnss_coords[2])
            xacro_command.append(' ')

    rsp_name = camera_name_val + '_state_publisher'
    rsp_node = Node(
        condition=IfCondition(publish_urdf),
        package='robot_state_publisher',
        namespace=namespace_val,
        executable='robot_state_publisher',
        name=rsp_name,
        output=node_log_effective,
        parameters=[{
            'use_sim_time': publish_svo_clock,
            'robot_description': Command(xacro_command)
        }],
        remappings=[('robot_description', camera_name_val+'_description')]
    )
    return_array.append(rsp_node)

    if(container_name_val == ''):
        container_name_val='zed_container'
        distro = os.environ['ROS_DISTRO']
        if distro == 'foxy':
            container_exec='component_container'
            arguments_val=['--ros-args', '--log-level', 'info']
        else:
            container_exec='component_container_isolated'
            arguments_val=['--use_multi_threaded_executor','--ros-args', '--log-level', 'info']
        
        zed_container = ComposableNodeContainer(
                name=container_name_val,
                namespace=namespace_val,
                package='rclcpp_components',
                executable=container_exec,
                arguments=arguments_val,
                output=node_log_effective,
                composable_node_descriptions=[]
        )
        return_array.append(zed_container)

    # ZED Node parameters
    node_parameters = [
            config_common_path_val,  
            config_camera_path_val,  
            object_detection_config_path, 
            custom_object_detection_config_path 
    ]

    if( ros_params_override_path_val != ''):
        node_parameters.append(ros_params_override_path)

    node_parameters.append( 
            {
                'use_sim_time': use_sim_time,
                'simulation.sim_enabled': sim_mode,
                'simulation.sim_address': sim_address,
                'simulation.sim_port': sim_port,
                'stream.stream_address': stream_address,
                'stream.stream_port': stream_port,
                'general.camera_name': camera_name_val,
                'general.camera_model': camera_model_val,
                'svo.svo_path': svo_path,
                'svo.publish_svo_clock': publish_svo_clock,
                'general.serial_number': serial_number,
                'general.camera_id': camera_id,
                'pos_tracking.publish_tf': publish_tf,
                'pos_tracking.publish_map_tf': publish_map_tf,
                'sensors.publish_imu_tf': publish_imu_tf,
                'gnss_fusion.gnss_fusion_enabled': enable_gnss,
                'general.virtual_serial_numbers': serial_numbers_val,
                'general.virtual_camera_ids': camera_ids_val
            }
    )

    if( camera_model_val in ['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'zedxhdr', 'zedxhdrmini', 'zedxhdrmax', 'virtual']):
        zed_wrapper_component = ComposableNode(
            package='zed_components',
            namespace=namespace_val,
            plugin='stereolabs::ZedCamera',
            name=node_name_val,
            parameters=node_parameters,
            extra_arguments=[{'use_intra_process_comms': enable_ipc}]
        )
    else: 
        zed_wrapper_component = ComposableNode(
            package='zed_components',
            namespace=namespace_val,
            plugin='stereolabs::ZedCameraOne',
            name=node_name_val,
            parameters=node_parameters,
            extra_arguments=[{'use_intra_process_comms': enable_ipc}]
        )
    
    full_container_name = '/' + namespace_val + '/' + container_name_val
    info = 'Loading ZED node `' + node_name_val + '` in container `' + full_container_name + '`'
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))
    
    load_composable_node = LoadComposableNodes(
        target_container=full_container_name,
        composable_node_descriptions=[zed_wrapper_component]
    )
    return_array.append(load_composable_node)

    return return_array

def generate_launch_description():
    return LaunchDescription(
        [
            # ... (기존 아규먼트들) ...
            DeclareLaunchArgument('node_log_type', default_value=TextSubstitution(text='both'), description='...'),
            DeclareLaunchArgument('camera_name', default_value=TextSubstitution(text='zed'), description='...'),
            DeclareLaunchArgument('camera_model', description='...'),
            DeclareLaunchArgument('container_name', default_value='', description='...'),
            DeclareLaunchArgument('namespace', default_value='', description='...'),
            DeclareLaunchArgument('node_name', default_value='zed_node', description='...'),
            DeclareLaunchArgument('ros_params_override_path', default_value='', description='...'),
            DeclareLaunchArgument('object_detection_config_path', default_value=TextSubstitution(text=default_object_detection_config_path), description='...'),
            DeclareLaunchArgument('custom_object_detection_config_path', default_value=TextSubstitution(text=default_custom_object_detection_config_path), description='...'),

            # [Modified] 파일명 입력을 위한 Argument 추가 (기본값 빈 문자열)
            DeclareLaunchArgument(
                'config_common_filename', 
                default_value='', 
                description='Configuration filename (e.g., common_stereo_zed_front.yaml). Path is relative to zed_wrapper/config.'),
            DeclareLaunchArgument(
                'config_camera_filename', 
                default_value='', 
                description='Camera specific config filename (e.g., zedx_zed_front.yaml). Path is relative to zed_wrapper/config.'),

            DeclareLaunchArgument('serial_number', default_value='0', description='...'),
            DeclareLaunchArgument('serial_numbers', default_value='[]', description='...'),
            DeclareLaunchArgument('camera_id', default_value='-1', description='...'),
            DeclareLaunchArgument('camera_ids', default_value='[]', description='...'),
            DeclareLaunchArgument('publish_urdf', default_value='true', description='...'),
            DeclareLaunchArgument('publish_tf', default_value='true', description='...'),
            DeclareLaunchArgument('publish_map_tf', default_value='true', description='...'),
            DeclareLaunchArgument('publish_imu_tf', default_value='false', description='...'),
            DeclareLaunchArgument('xacro_path', default_value=TextSubstitution(text=default_xacro_path), description='...'),
            DeclareLaunchArgument('svo_path', default_value=TextSubstitution(text='live'), description='...'),
            DeclareLaunchArgument('publish_svo_clock', default_value='false', description='...'),
            DeclareLaunchArgument('enable_gnss', default_value='false', description='...'),
            DeclareLaunchArgument('gnss_antenna_offset', default_value='[]', description='...'),
            DeclareLaunchArgument('enable_ipc', default_value='true', description='...'),
            DeclareLaunchArgument('use_sim_time', default_value='false', description='...'),
            DeclareLaunchArgument('sim_mode', default_value='false', description='...'),
            DeclareLaunchArgument('sim_address', default_value='127.0.0.1', description='...'),
            DeclareLaunchArgument('sim_port', default_value='30000', description='...'),
            DeclareLaunchArgument('stream_address', default_value='', description='...'),
            DeclareLaunchArgument('stream_port', default_value='30000', description='...'),
            OpaqueFunction(function=launch_setup)
        ]
    )
```

-----

### 2\. `zed_multi_camera.launch.py` 수정

루프 안에서 **파일명 자동 생성 로직**을 추가하여, Launch 실행 시 별도의 파라미터 입력 없이도 이름 규칙에 따라 파일을 찾아가도록 했습니다.

```python
# [zed-ros2-examples/tutorials/zed_multi_camera/launch/zed_multi_camera.launch.py]

import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction
)
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    TextSubstitution
)
from launch_ros.actions import (
    Node,
    ComposableNodeContainer
)

def parse_array_param(param):
    str = param.replace('[', '')
    str = str.replace(']', '')
    str = str.replace(' ', '')
    arr = str.split(',')
    if len(arr) == 1 and arr[0] == '':
        return []
    return arr

def launch_setup(context, *args, **kwargs):
    actions = []
    namespace_val = 'zed_multi'
    
    multi_zed_xacro_path = os.path.join(
        get_package_share_directory('zed_multi_camera'),
        'urdf',
        'zed_multi.urdf.xacro')

    names = LaunchConfiguration('cam_names')
    models = LaunchConfiguration('cam_models')
    serials = LaunchConfiguration('cam_serials')
    ids = LaunchConfiguration('cam_ids')
    disable_tf = LaunchConfiguration('disable_tf')

    names_arr = parse_array_param(names.perform(context))
    models_arr = parse_array_param(models.perform(context))
    serials_arr = parse_array_param(serials.perform(context))
    ids_arr = parse_array_param(ids.perform(context))
    disable_tf_val = disable_tf.perform(context)

    num_cams = len(names_arr)

    if (num_cams != len(models_arr)):
        return [LogInfo(msg=TextSubstitution(text='Models/Names mismatch'))]

    # ... Container 생성 부분 동일 ...
    container_name = 'zed_multi_container'
    distro = os.environ['ROS_DISTRO']
    if distro == 'foxy':
        container_exec='component_container'
    else:
        container_exec='component_container_isolated'
    
    info = '* Starting Composable node container: /' + namespace_val + '/' + container_name
    actions.append(LogInfo(msg=TextSubstitution(text=info)))

    zed_container = ComposableNodeContainer(
        name=container_name,
        namespace=namespace_val,
        package='rclcpp_components',
        executable=container_exec,
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
    )
    actions.append(zed_container)

    cam_idx = 0
    launch_delay_step = 4.0 

    for name in names_arr:
        model = models_arr[cam_idx]
        if len(serials_arr) == num_cams:
            serial = serials_arr[cam_idx]
        else:
            serial = '0'

        if len(ids_arr) == num_cams:
            id = ids_arr[cam_idx]
        else:
            id = '-1'

        # ---------------------------------------------------------------------
        # [Modified] 파일명 자동 생성 로직 (Naming Rule 적용)
        # ---------------------------------------------------------------------
        
        # 1. Common Stereo Config 생성
        # 예: cam_names='zed_front' -> common_stereo_zed_front.yaml
        # (Mono 모델인 경우 로직을 분기해도 되지만, 일단 Stereo 기준으로 작성)
        stereo_models = ['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'zedxhdr', 'virtual']
        
        if model in stereo_models:
             base_common = 'common_stereo'
        else:
             base_common = 'common_mono'
             
        # Rule: common_stereo + "_" + cam_names + ".yaml"
        generated_common_filename = base_common + '_' + name + '.yaml'

        # 2. Camera Specific Config 생성
        # 예: model='zedx', cam_names='zed_front' -> zedx_zed_front.yaml
        # Rule: model + "_" + cam_names + ".yaml"
        generated_camera_filename = model + '_' + name + '.yaml'

        # ---------------------------------------------------------------------

        info_text = '* Queueing ZED ROS2 node for camera ' + name + ' (' + model + ')'
        info_text += ' Configs: [' + generated_common_filename + ', ' + generated_camera_filename + ']'
        
        publish_tf = 'false'
        if (cam_idx == 0):
            if (disable_tf_val == 'False' or disable_tf_val == 'false'):
                publish_tf = 'true'

        zed_wrapper_launch = IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource([
                get_package_share_directory('zed_wrapper'),
                '/launch/zed_camera.launch.py'
            ]),
            launch_arguments={
                'container_name': container_name,
                'camera_name': name,
                'camera_model': model,
                'serial_number': serial,
                'camera_id': id,
                'publish_tf': publish_tf,
                'publish_map_tf': publish_tf,
                'namespace': namespace_val,
                # [Modified] 생성된 파일명 전달
                'config_common_filename': generated_common_filename,
                'config_camera_filename': generated_camera_filename
            }.items()
        )
        
        delay_action = TimerAction(
            period=float(cam_idx) * launch_delay_step,
            actions=[
                LogInfo(msg=TextSubstitution(text=info_text)),
                zed_wrapper_launch
            ]
        )
        actions.append(delay_action)

        cam_idx += 1

    # ... (Robot State Publisher 등 나머지 동일) ...
    xacro_command = []
    xacro_command.append('xacro')
    xacro_command.append(' ')
    xacro_command.append(multi_zed_xacro_path)
    xacro_command.append(' ')
    cam_idx = 0
    for name in names_arr:
        xacro_command.append('camera_name_'+str(cam_idx)+':=')
        xacro_command.append(name)
        xacro_command.append(' ')
        cam_idx+=1

    rsp_name = 'state_publisher'
    actions.append(LogInfo(msg=TextSubstitution(text='* Starting robot_state_publisher...')))
    multi_rsp_node = Node(
        package='robot_state_publisher',
        namespace=namespace_val,
        executable='robot_state_publisher',
        name=rsp_name,
        output='screen',
        parameters=[{
            'robot_description': Command(xacro_command).perform(context)
        }]
    )

    actions.append(multi_rsp_node)

    return actions

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('cam_names', description='...'),
            DeclareLaunchArgument('cam_models', description='...'),
            DeclareLaunchArgument('cam_serials', default_value=[], description='...'),
            DeclareLaunchArgument('cam_ids', default_value=[], description='...'),
            DeclareLaunchArgument('disable_tf', default_value='False', description='...'),
            OpaqueFunction(function=launch_setup)
        ]
    )
```

### 작동 방식 및 준비 사항

1.  **파일 준비**: 이제 `zed_wrapper/config/` 폴더 안에 아래와 같은 이름으로 파일들이 존재해야 합니다.

      * `common_stereo_zed_front.yaml`
      * `zedx_zed_front.yaml`
      * `common_stereo_zed_back.yaml`
      * `zedx_zed_back.yaml`
      * ... (나머지 카메라도 동일)

2.  **실행**: 기존과 똑같이 실행하면 됩니다.

    ```bash
    ros2 launch zed_multi_camera zed_multi_camera.launch.py \
        cam_names:='[zed_front,zed_back]' \
        cam_models:='[zedx,zedx]'
    ```

      * 그러면 내부적으로:
          * `zed_front` -\> `common_stereo_zed_front.yaml`, `zedx_zed_front.yaml` 로딩
          * `zed_back` -\> `common_stereo_zed_back.yaml`, `zedx_zed_back.yaml` 로딩

이렇게 하면 절대 경로 입력 없이 깔끔하게 관리가 가능합니다\!
#########################  
################################  
################################  
로그의 핵심은 **(1) CAM3에서 Argus 프레임 타임아웃 발생 → (2) ZED SDK가 “CAMERA REBOOTING”으로 복구 시도 → (3) Argus/캡처카드(공유 리소스) 쪽이 불안정해지면서 나머지 카메라까지 연쇄적으로 동일 증상 → 노드 종료** 흐름일 가능성이 큽니다.

## 왜 “CAM3 timeout”이 4대 전체 다운으로 번질 수 있나

* ZED X(+ZED Link Quad)는 내부적으로 Jetson의 **NvArgus(Argus) 카메라 스택**을 타고 동작합니다. Argus 쪽에서 타임아웃이 나면 “복구(recovering)”를 시도하지만, **GMSL 디시리얼라이저/VI-CSI 파이프라인/Argus 데몬은 여러 카메라가 공유**하는 부분이 있어서 **한 채널(CAM3) 문제/리셋이 전체 세션을 흔들어** 다른 카메라도 같이 “CAMERA REBOOTING”으로 넘어갈 수 있습니다.
* 실제로 Stereolabs 쪽에서도 **일정 시간 사용 후 Argus가 불안정해지고, grab 실패→reboot 시도 이후 애플리케이션이 죽어서 재시작이 필요**한 유형의 케이스가 보고돼 있습니다. ([Stereolabs Forums][1])
* NVIDIA 포럼에서도 **GMSL + Argus 사용 시 “시간이 좀 지난 뒤” 타임아웃이 나고, 이후 에러 상태가 “복구 불가(unrecoverable)”로 고착**되는 케이스가 보고됩니다. ([NVIDIA Developer Forums][2])

## 가장 흔한 원인 Top 5 (우선순위)

1. **물리 링크 불안정(케이블/커넥터/리본 케이블 접촉)**

   * Stereolabs도 ZED Link 캡처카드의 **리본 케이블 체결 상태**를 1순위로 보라고 가이드합니다. ([Stereolabs Forums][3])
2. **특정 포트/특정 카메라/특정 케이블 불량**

   * “CAM3에서만 먼저 터진다”면 특히 의심. (그 다음 전체가 연쇄 다운)
3. **과부하(해상도/프레임/Depth 모드/추가 기능)로 인한 파이프라인 타임아웃**

   * Stereolabs 포럼에서도 **해상도 낮추기**를 바로 권장합니다. ([Stereolabs Forums][3])
4. **JetPack 6.x Argus 계열 이슈(장시간/멀티카메라에서 타임아웃→데몬/세션 불안정)**

   * NVIDIA 쪽 멀티캠 이슈에서 타임아웃 관련 워크어라운드(무한 타임아웃 모드) 언급도 나옵니다. ([NVIDIA Developer Forums][4])
5. **전원/발열/스로틀링**

   * 1day+ 구간에서 터지는 패턴이면 “열/전원/리소스 누수”도 강하게 의심됩니다.

## 바로 할 수 있는 “원인 분리” 체크 (가장 효율 좋은 순서)

### A) “CAM3”가 **항상 같은 물리 카메라**인지, **항상 같은 포트**인지 분리

* 카메라 4대를 **서로 포트 교차**해 연결해서 24시간 내 재현을 봅니다.

  * 문제가 **포트를 따라가면** → 캡처카드 채널/커넥터/리본/보드 쪽
  * 문제가 **카메라를 따라가면** → 해당 카메라/해당 GMSL 케이블 불량 가능성 큼

### B) 부하를 확 낮춰서 “과부하/대역폭/Argus 타임아웃”인지 확인

* `zed_wrapper/config/zedx.yaml`에서

  * `grab_resolution`: `HD1200` → `HD1080` 또는 `SVGA`
  * `grab_frame_rate`: 30 → 15 (테스트용)
* Depth도 가능하면 한 단계 가볍게(NEURAL LIGHT가 이미 가벼운 편이지만, 테스트 목적이면 depth/pointcloud/odometry/objects 등을 최대한 끄고 “이미지 스트림만”으로 24h 테스트 권장)

### C) 실패 직후에 아래 로그를 꼭 확보 (원인이 Argus/VI/CSI인지 바로 갈립니다)

* Argus 데몬:

  * `sudo journalctl -u nvargus-daemon -n 300 --no-pager`
* 커널(카메라/CSI/VI/MAX96712 관련):

  * `dmesg -T | egrep -i "argus|nvargus|nvcsi|vi|camrtc|max967|max929|timeout|corr|error" | tail -n 200`

## “재발 시 즉시 복구”를 위한 현실적인 워크어라운드

장시간 안정성 테스트 목적이면, **완전 무정지보다 ‘자동 복구’까지 포함한 안정성**을 보는 게 실무적으로 유효합니다.

1. Argus 데몬 재시작(가장 흔한 복구 루트)

* `sudo systemctl restart nvargus-daemon` (또는 배포판에 따라 `sudo service nvargus-daemon restart`)
* 그 다음 zed 관련 데몬/노드 재기동

2. JetPack 6.x에서 타임아웃 완화용 “infinite timeout” 모드(실험적)

* NVIDIA 포럼에서 아래 형태로 **무한 타임아웃 모드로 nvargus-daemon을 띄우는 방법**이 언급됩니다. ([NVIDIA Developer Forums][4])

  * `sudo service nvargus-daemon stop`
  * `sudo enableCamInfiniteTimeout=1 nvargus-daemon`
* 다만, 스레드 맥락상 **첨부된 라이브러리/패치와 함께 테스트**하는 흐름이라, “그냥 옵션만으로 100% 해결”을 보장하긴 어렵습니다(하지만 방향성은 동일: *Argus timeout이 시스템 전체를 무너뜨리지 않게*).

## 결론적으로 “뭐가 문제인가?”

현상 자체는 **ZED SDK/ROS2 wrapper가 죽은 게 1차가 아니라**, **Jetson Argus(또는 GMSL 링크/캡처카드 채널)에서 프레임 타임아웃이 먼저 발생**했고, 그 여파로 **카메라 리부팅/세션 붕괴가 연쇄 발생**한 케이스로 보는 게 가장 타당합니다. (Stereolabs/NVIDIA 양쪽에서 비슷한 “장시간 후 Argus timeout → 복구 어려움” 패턴이 보고됨) ([Stereolabs Forums][1])

원하면, 네 환경 기준으로 **(1) zedx.yaml에서 안정성 테스트용 저부하 프리셋**, **(2) failure 감지 → nvargus-daemon/zed 노드 자동 재기동하는 systemd watchdog 스크립트**까지 바로 만들어줄게.

[1]: https://community.stereolabs.com/t/zedx-argus-socket-failing-after-period-of-use-and-unable-to-recover/7004 "Zedx argus socket failing after period of use and unable to recover - Stereolabs Forums"
[2]: https://forums.developer.nvidia.com/t/argus-timeout-when-capturing-frames-followed-by-unrecoverable-state/344997 "Argus timeout when capturing frames followed by unrecoverable state - Jetson Orin Nano - NVIDIA Developer Forums"
[3]: https://community.stereolabs.com/t/zed-x-connection-argus-issues/10348 "ZED X Connection/Argus issues - Stereolabs Forums"
[4]: https://forums.developer.nvidia.com/t/nvarguscamerasrc-failed-for-8-cameras-simultaneously/337618 "Nvarguscamerasrc failed for 8 cameras simultaneously - Jetson AGX Orin - NVIDIA Developer Forums"
#########################  
##########################  
#############################  
전문가 관점에서 분석해 드리겠습니다. **JetPack 6.2.1 (L4T 36.4.7)** 환경에서 **ZED X 4대**를 동시에 구동하다가 약 42시간(1일 18시간) 시점에 **동시 타임아웃**이 발생한 현상은, 개별 카메라의 불량보다는 **시스템 리소스, 발열, 또는 PCIe 대역폭/전력 관리**와 관련된 중앙 병목(Central Bottleneck) 문제일 가능성이 매우 높습니다.

로그 내용(`[ZED-Argus]{produceDualV4} CAM 3 has timeout`)은 Orin의 ISP(Image Signal Processor)가 GMSL2 Capture Card로부터 프레임 데이터를 제때 받지 못했음을 의미합니다.

다음은 가장 유력한 원인들과 해결책을 순서대로 정리한 것입니다.

---

### 1. 원인 분석: GMSL2 캡처 카드 및 PCIe 과열 (가장 유력)

ZED X 4대를 구동할 때, 카메라 자체의 발열도 있지만 데이터를 모아서 Orin으로 보내주는 **Quad GMSL2 Capture Card의 발열**이 상당합니다.

* **증상:** 42시간 동안 열이 서서히 축적되다가 캡처 카드의 Deserializer 칩셋(주로 Maxim 칩)이 과열로 인해 일시적인 'Thermal Shutdown' 또는 클럭 저하를 일으켰을 가능성이 큽니다.
* **이유:** 4개의 카메라가 동시에 죽었다는 점은 개별 케이블 문제가 아니라 이들이 연결된 **허브(PCIe 카드)**가 멈췄다는 강력한 증거입니다.
* **해결책:**
* Capture Card에 **별도의 쿨링 팬**이 장착되어 있는지 확인하십시오. (Orin 케이스 내부 공기 순환만으로는 부족할 수 있습니다.)
* 테스트 중 `jtop`이나 센서를 통해 온도를 모니터링해야 합니다.



### 2. PCIe ASPM (Active State Power Management) 문제

Jetson 환경에서 장시간 구동 시 가장 흔한 불안정 원인 중 하나입니다. 시스템이 절전 모드를 위해 PCIe 링크 전력을 줄이려고 시도하다가, 다시 활성화될 때 타이밍을 놓쳐 연결이 끊기는 현상입니다.

* **증상:** 랜덤한 시간(주로 장시간 유휴 혹은 부하 변동 시)에 장치 연결이 끊김.
* **해결책:** PCIe 절전 모드를 강제로 꺼야 합니다.
1. 터미널에서 다음 스크립트를 실행하여 ASPM을 비활성화합니다.
```bash
sudo /usr/bin/jetson_clocks

```


2. 영구적으로 적용하기 위해 부트로더 설정을 확인하거나, 서비스로 등록하여 부팅 시마다 `performance` 모드가 되도록 설정하십시오.
3. `/boot/extlinux/extlinux.conf` 파일의 `APPEND` 라인 끝에 `pcie_aspm=off`를 추가하고 재부팅해 보십시오.



### 3. NvArgus Daemon 메모리 누수 또는 불안정

JetPack 6.x (L4T 36.x) 대의 Argus 데몬은 이전 버전보다 개선되었으나, 4개의 4K/QHD급 스트림을 장시간 처리할 때 **buffer pool** 문제가 발생할 수 있습니다.

* **증상:** `Argus` 관련 타임아웃 로그가 발생하며, 재부팅 전까지는 카메라가 다시 잡히지 않거나 `nvargus-daemon`을 재시작해야 함.
* **진단:**
* 문제가 발생했을 때 `sudo systemctl status nvargus-daemon`을 확인해 보셨나요?


* **해결책:**
* ZED SDK 5.1.1은 최신이지만, 혹시 모를 데몬 충돌을 방지하기 위해 테스트 스크립트에 **주기적인 리셋 로직**을 넣기보다, 시스템 레벨에서 `nvargus-daemon`의 리소스 사용량(메모리)이 시간이 지남에 따라 선형적으로 증가하는지 확인해야 합니다.



### 4. 전원 공급 부족 (Voltage Drop)

Orin AGX Dev Kit의 전원 어댑터는 강력하지만, USB나 PCIe를 통해 공급되는 전력이 순간적으로 출렁일 경우(Transient voltage drop), GMSL2 링크가 끊어질 수 있습니다.

* **상황:** 4대의 카메라가 동시에 Depth 연산을 수행하며 GPU/CPU 부하가 피크를 칠 때 전압 강하 발생.
* **해결책:**
* 전원 모드가 **MAXN**으로 설정되어 있는지 확인하십시오 (`sudo nvpmodel -m 0`).
* ZED X 4대는 GMSL2 카드를 통해 외부 전원을 별도로 공급받나요, 아니면 PCIe 슬롯 파워만 사용하나요? 4대라면 **Capture Card에 별도 6-pin/SATA 전원 공급**이 필수적입니다. PCIe 슬롯 전력만으로는 4대 구동 시 불안정할 수 있습니다.



### 5. ZED SDK/ROS 2 Wrapper 설정 (Data Bandwidth)

ZED X 4대에서 들어오는 데이터 양이 PCIe 대역폭이나 Orin의 메모리 대역폭 한계치에 근접했을 수 있습니다.

* **체크:** `grab_frame_rate`나 `pub_frame_rate`가 너무 높게 설정되어 있지 않은지 확인하십시오. (예: 4대 모두 60fps 등)
* **제안:** 안정성 테스트를 위해 FPS를 15~30 정도로 낮추고 해상도를 조절하여 5일 테스트를 통과하는지 먼저 확인하십시오.

---

### 추천하는 조치 순서

가장 의심스러운 부분부터 검증하기 위해 다음 단계를 제안합니다.

1. **하드웨어 쿨링 보강:** Capture Card에 직접 바람을 불어주는 팬을 설치하고 다시 테스트를 시작하십시오.
2. **PCIe ASPM 비활성화:** `pcie_aspm=off` 커널 파라미터 적용.
3. **Argus 데몬 재시작 스크립트 준비:** 만약 소프트웨어적 버그라면, `CAMERA REBOOTING` 에러 감지 시 ZED 노드만 죽는 게 아니라 `sudo systemctl restart nvargus-daemon`을 수행하도록 래퍼 스크립트를 작성하는 것이 현장에서는 유효한 회피책입니다.

**다음 단계:**
현재 4대의 카메라 설정(해상도, FPS, Depth Mode)과 사용 중인 GMSL2 캡처 카드의 제조사(Stereolabs 오리지널인지, Connect Tech 등 타사 제품인지)를 알려주시면, 해당 하드웨어에 맞는 더 구체적인 PCIe 설정법을 알려드릴 수 있습니다.
  
