결론부터 말씀드리면, \*\*"4대의 카메라가 뿜어내는 데이터 폭주를 OS의 기본 네트워크 버퍼(UDP Buffer)가 감당하지 못해서 생기는 문제"\*\*입니다.

1대일 때는 기본 버퍼로 겨우 버텼지만, 4대가 되면서 패킷이 OS 레벨에서 유실(Drop)되고, FastDDS가 조각난 패킷을 재조립(Reassembly)하다가 버퍼 크기 오류를 뱉는 상황입니다.

이 문제는 \*\*Jetson AGX(송신측)\*\*와 **IPC1(수신측)** **양쪽 모두**에서 설정을 변경해야 해결됩니다.

-----

### 해결책 1: 리눅스 커널 네트워크 버퍼 늘리기 (가장 중요 ★)

기본 우분투 설정은 대용량 이미지/Pointcloud 전송에 적합하지 않습니다. 터미널을 열고 **AGX와 IPC1 양쪽 모두**에 아래 명령어를 입력하세요.

**1. 임시 적용 (재부팅 시 초기화, 테스트용)**

```bash
# 수신 버퍼 최대 크기 (약 25MB)
sudo sysctl -w net.core.rmem_max=26214400
# 송신 버퍼 최대 크기 (약 25MB)
sudo sysctl -w net.core.wmem_max=26214400
# 수신 버퍼 기본값
sudo sysctl -w net.core.rmem_default=26214400
# 송신 버퍼 기본값
sudo sysctl -w net.core.wmem_default=26214400
```

**2. 영구 적용 (설정 파일 저장)**

```bash
# /etc/sysctl.conf 파일 열기
sudo nano /etc/sysctl.conf

# 맨 아래에 다음 내용 추가
net.core.rmem_max=26214400
net.core.wmem_max=26214400
net.core.rmem_default=26214400
net.core.wmem_default=26214400
net.ipv4.udp_mem = 102400 873800 26214400

# 저장(Ctrl+O, Enter) 후 종료(Ctrl+X)
# 설정 즉시 적용
sudo sysctl -p
```

-----

### 해결책 2: FastDDS XML 설정 (UDP 모드 최적화)

유선 랜 연결이므로 이전에 도커 내부용으로 만들었던 `Shared Memory` 설정은 IPC1과의 통신에 도움이 안 됩니다. **UDP 전송을 위한 XML**을 따로 만들어야 합니다.

**`fastdds_udp.xml` 작성 (AGX, IPC1 양쪽 모두 동일하게 사용)**

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>udp_transport</transport_id>
            <type>UDPv4</type>
            <sendBufferSize>26214400</sendBufferSize>
            <receiveBufferSize>26214400</receiveBufferSize>
            <maxMessageSize>65000</maxMessageSize> 
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="udp_participant" is_default_profile="true">
        <rtps>
            <useBuiltinTransports>false</useBuiltinTransports>
            <userTransports>
                <transport_id>udp_transport</transport_id>
            </userTransports>
        </rtps>
    </participant>
</profiles>
```

**실행 시 적용:**

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/경로/fastdds_udp.xml
ros2 launch ... (또는 ros2 topic echo ...)
```

-----

### 해결책 3: 물리적 대역폭 현실 직시 (1Gbps vs 4 Cameras)

만약 위 설정을 해도 끊기거나 느리다면, 이는 **물리적 네트워크 대역폭 한계**일 가능성이 높습니다.

  * **ZED X 1대 데이터량:** HD1200 @ 30fps 기준, Raw Image + Depth + PointCloud를 합치면 **300\~500 Mbps**를 쉽게 넘습니다.
  * **4대 동시 전송:** **1.2Gbps \~ 2Gbps**가 필요합니다.
  * **일반 유선 랜:** 보통 **1Gbps**입니다.

**즉, 물리적으로 선이 감당을 못합니다.**

**조치 방법:**

1.  **`ros2 topic echo` 금지:** PointCloud2 토픽을 `echo` 하면 터미널 텍스트 변환 부하 때문에 터질 수 있습니다. `ros2 topic hz`나 `ros2 topic bw`로 확인하세요.
2.  **Rviz2 확인 시:** `Reliability` 정책을 \*\*`Best Effort`\*\*로 설정하세요. (패킷 좀 잃어버려도 되니 최신꺼만 보여줘라)
3.  **데이터 다이어트 (필수):**
      * IPC1으로 데이터를 보낼 때, `qos_profile`을 조정하거나,
      * \*\*압축된 이미지(`compressed image`)\*\*만 전송하거나,
      * ZED Wrapper 설정(`common.yaml`)에서 **FPS를 낮추거나(15fps)** 해상도를 낮춰야 합니다.

### 요약

1.  **AGX & IPC1 양쪽**에서 `sysctl`로 네트워크 버퍼를 25MB 이상으로 늘리세요. (가장 중요)
2.  **물리적 랜선이 1Gbps라면**, 4대분의 PointCloud를 Raw 데이터로 동시에 보내는 건 불가능합니다. \*\*압축(Compression)\*\*을 쓰거나 **필요한 카메라만 Subscribe** 해야 합니다.


#############################  
#############################  
#############################  
#############################  
\*\*"토픽 리스트는 보이는데 데이터를 보려고 하면 터진다"\*\*는 현상은 전형적인 **대역폭 부족(Bandwidth Limitation)** 및 **OS 버퍼 오버플로우** 증상입니다.

ROS 2의 Discovery(토픽 리스트 갱신)는 아주 작은 패킷만 주고받기 때문에 연결된 것처럼 보이지만, 실제 대용량 데이터(이미지, PointCloud)를 보내는 순간 네트워크 파이프가 막혀서 패킷이 유실되고 에러가 나는 것입니다.

질문하신 **현재 네트워크 상태를 확인하는 방법**과 **해결책**을 정리해 드립니다.

-----

### 1\. 현재 이더넷 링크 속도 확인 (Link Speed)

물리적으로 랜선이 100Mbps로 연결되었는지, 1Gbps(1000Mbps)로 연결되었는지 확인하는 명령어입니다.

**Jetson과 IPC 양쪽에서 확인하세요.**

1.  **네트워크 인터페이스 이름 찾기:**

    ```bash
    ip link
    ```

    (보통 `eth0`, `enp0s...` 등의 이름을 가집니다.)

2.  **속도 확인 (`ethtool`):**

    ```bash
    # 예: eth0이 인터페이스 이름일 경우
    sudo ethtool eth0 | grep Speed
    ```

      * **결과가 `Speed: 1000Mb/s`:** 1Gbps 연결입니다. (일반적인 최대치)
      * **결과가 `Speed: 100Mb/s`:** 케이블 불량(CAT5)이거나 스위치 문제입니다. (이 경우 무조건 끊깁니다.)
      * **결과가 `Speed: 10000Mb/s`:** 10Gbps 연결입니다. (이상적인 환경)

-----

### 2\. 실제 전송 속도 측정 (Throughput Test)

`Speed`가 1000Mb/s라고 떠도, 케이블 품질이나 시스템 부하 때문에 실제 속도는 안 나올 수 있습니다. `iperf3`로 **도로의 실제 통행량**을 측정해야 합니다.

1.  **설치 (양쪽 모두):**

    ```bash
    sudo apt update && sudo apt install iperf3
    ```

2.  **측정 (서버-클라이언트 방식):**

      * **IPC (수신측)에서 실행:**
        ```bash
        iperf3 -s
        ```
      * **Jetson (송신측)에서 실행:**
        ```bash
        # IPC의 IP주소가 192.168.1.100 이라고 가정
        iperf3 -c 192.168.1.100
        ```

3.  **결과 해석 (`Bitrate` 확인):**

      * **940 Mbits/sec 이상:** 1Gbps 네트워크가 정상 작동 중입니다.
      * **그 이하:** 네트워크 장비나 케이블 점검이 필요합니다.

-----

### 3\. "Sequence size exceeds..." 에러 해결 (필수 설정)

도커를 안 쓰시더라도, ZED 4대의 데이터를 FastDDS로 보내려면 **OS의 커널 버퍼(UDP Buffer)를 늘려주는 설정은 Host OS에 직접 반드시 적용**해야 합니다. 이 에러는 \*\*"데이터가 너무 커서 조각내서 보냈는데, 받는 쪽(IPC) 그릇이 작아서 넘쳤다"\*\*는 뜻입니다.

**Jetson과 IPC 양쪽 터미널에 아래 명령어를 입력하세요 (재부팅 후에도 유지되게 설정):**

```bash
# 설정 파일 열기
sudo nano /etc/sysctl.conf

# 문서 맨 아래에 다음 내용 복사-붙여넣기
net.core.rmem_max=26214400
net.core.wmem_max=26214400
net.core.rmem_default=26214400
net.core.wmem_default=26214400
net.ipv4.udp_mem = 102400 873800 26214400

# 저장(Ctrl+O, Enter) 후 종료(Ctrl+X)

# 설정 적용
sudo sysctl -p
```

이걸 적용하지 않으면 대역폭이 충분해도 에러가 뜹니다.

-----

### 4\. 현실적인 문제: 1Gbps로는 4대를 못 버팁니다.

만약 `iperf3` 결과가 **940Mbps(약 110MB/s)** 정도 나온다고 가정해 보겠습니다.

ZED X 1대가 쏘는 데이터 양을 대략 계산해보면:

  * **HD1200 @ 30fps** 기준 (RGB + Depth)
  * 압축 없는 Raw 데이터: **약 300\~400 Mbps** (카메라 1대당)

**결론:**

  * 카메라 2대까지는 1Gbps 라인에서 아슬아슬하게 됩니다.
  * **카메라 4대면 약 1.2Gbps \~ 1.6Gbps가 필요합니다.**
  * 물리적으로 1Gbps 랜선 하나로는 **절대로** 4대의 Raw 데이터를 동시에 전송할 수 없습니다.

**해결책 (택 1):**

1.  **압축 전송 사용 (가장 현실적):**
    IPC에서 `image_raw` 대신 **`image_raw/compressed`** 토픽을 구독하세요.
    (ZED Wrapper는 기본적으로 `image_transport`를 지원하여 압축 토픽을 발행합니다.)

2.  **10Gbps 네트워크 업그레이드:**
    Jetson AGX Orin은 10GbE 포트가 있습니다. IPC에도 10GbE 랜카드를 달고, CAT6A 이상 케이블로 연결하면 해결됩니다.

3.  **FPS 낮추기:**
    ZED 설정(`common.yaml`)에서 `grab_frame_rate`를 15로 낮추면 대역폭이 절반으로 줄어들어 1Gbps에서도 가능할 수 있습니다.
######################
##########################
##########################
#############################
**2.5Gbps 대역폭**에 **SVGA 15fps**라면 대역폭은 차고 넘치는 상황입니다. 물리적인 네트워크 환경은 아주 훌륭합니다.

그럼에도 불구하고 Depth 데이터(대용량 데이터)가 안 넘어오고 **"Sequence size exceeds remaining buffer"** 에러가 뜬다면, 원인은 **100% "FastDDS의 기본 설정(Default Configuration)이 대용량 데이터 전송용으로 잡혀있지 않아서"** 입니다.

OS 버퍼(`sysctl`)를 늘려주셨지만, **FastDDS(미들웨어)에게 "이 늘어난 버퍼를 가져다 써라"고 알려주지 않으면** FastDDS는 여전히 작은 기본 버퍼만 사용하다가 에러를 뱉습니다.

해결을 위해 다음 3단계를 **Jetson(송신)과 IPC(수신) 양쪽 모두**에 적용해 주세요.

-----

### 1단계: 누락된 Kernel 설정 추가 (중요)

사용자님이 입력하신 명령어 4줄은 버퍼의 "최대치"만 늘렸지, **"UDP 프로토콜이 사용할 전체 메모리"** 설정이 빠져 있을 수 있습니다.

아래 명령어를 양쪽 터미널에 추가로 입력하세요.

```bash
# UDP 전송을 위한 전체 메모리 할당 (Low / Pressure / High)
sudo sysctl -w net.ipv4.udp_mem="102400 873800 26214400"

# 적용 확인
sudo sysctl -p
```

-----

### 2단계: FastDDS XML 프로파일 적용 (핵심 해결책 ★)

FastDDS가 65KB 이상의 큰 데이터(이미지)를 쪼개서(Fragmentation) 보내고, 받을 때 큰 버퍼를 쓰도록 강제하는 설정 파일입니다.

**1. 파일 생성 (`~/fastdds_udp.xml`)**
Jetson과 IPC 양쪽에 동일하게 만드세요.

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>udp_transport</transport_id>
            <type>UDPv4</type>
            <sendBufferSize>25165824</sendBufferSize>
            <receiveBufferSize>25165824</receiveBufferSize>
            <maxMessageSize>65000</maxMessageSize>
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="udp_participant" is_default_profile="true">
        <rtps>
            <useBuiltinTransports>false</useBuiltinTransports>
            <userTransports>
                <transport_id>udp_transport</transport_id>
            </userTransports>
        </rtps>
    </participant>
</profiles>
```

**2. 환경변수 적용 (터미널에서 실행)**
ZED 노드를 실행하기 전(Jetson)과 데이터를 받기 전(IPC)에 이 환경변수를 선언해야 합니다.

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_udp.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

*매번 치기 귀찮으시다면 `~/.bashrc` 맨 아래에 추가해 두세요.*

-----

### 3단계: 점보 프레임 (Jumbo Frames) 활성화 (선택 사항)

2.5Gbps 네트워크 카드는 \*\*MTU(Maximum Transmission Unit)\*\*를 기본 1500에서 \*\*9000(Jumbo Frame)\*\*으로 늘릴 때 성능이 극대화됩니다. 큰 데이터를 덜 쪼개서 보내기 때문에 CPU 부하가 줄어듭니다.

**양쪽(Jetson, IPC) 터미널에서 실행:**

```bash
# 인터페이스 이름이 eth0 이라고 가정 (ip link로 확인 필요)
sudo ip link set dev eth0 mtu 9000
```

*(주의: 이더넷 스위치 허브를 거쳐간다면, 허브도 점보 프레임을 지원해야 합니다. 직결이면 상관없습니다.)*

-----

### 테스트 방법

위 설정을 다 마친 후, IPC(수신측)에서 아래 명령어로 테스트해보세요.

**잘못된 방법 (터미널 폭주):**
`ros2 topic echo /zed/zed_node/depth/depth_registered` (데이터가 너무 커서 텍스트 변환하다가 죽습니다.)

**올바른 방법:**

```bash
# 1. 대역폭 확인 (Best Effort로 받아서 끊김 없이 들어오는지 확인)
ros2 topic bw /zed/zed_node/depth/depth_registered --qos-reliability best_effort

# 2. 헤르츠 확인 (15hz가 안정적으로 나오는지)
ros2 topic hz /zed/zed_node/depth/depth_registered --qos-reliability best_effort
```

### 요약

사용자님의 문제는 \*\*"도로(2.5Gbps)는 넓은데 트럭(FastDDS 버퍼)이 작아서 짐을 못 싣는 상황"\*\*입니다.
**`fastdds_udp.xml`** 파일을 만들고 환경변수로 적용하면 100% 해결될 겁니다. 이것이 적용되면 나중에 4대를 켜도 충분히 버팁니다.
#########################  
#############################  
#############################  
지금 겪고 계신 **"Sequence size exceeds remaining buffer"** 에러는 설정한 XML이 **제대로 로드되지 않았거나**, ROS 2 데몬(Daemon)이 **과거 설정(기본값)을 기억하고 있어서** 발생하는 현상일 확률이 99%입니다.

특히, `.bashrc`를 수정하셨더라도 **이미 떠 있는 ROS 2 백그라운드 프로세스**는 이를 모릅니다.

다음 **4단계 절차**를 순서대로 하나씩 수행해서 해결해 봅시다.

-----

### 1단계: 가장 중요\! ROS 2 데몬 죽이기 (Daemon Reset)

ROS 2는 빠른 통신을 위해 백그라운드에서 `ros2 daemon`을 돌립니다. 이 녀석이 죽기 전까지는 `.bashrc`를 아무리 수정해도 \*\*옛날 설정(작은 버퍼)\*\*을 그대로 유지합니다.

**Jetson(송신)과 IPC(수신) 양쪽 터미널에서 다음 명령어를 입력하세요:**

```bash
# 데몬 정지 (필수)
ros2 daemon stop

# 확인 사살 (프로세스가 없다면 에러가 떠도 괜찮음)
pkill -f ros2
pkill -f _ros2_daemon
```

이후 터미널을 껐다가 다시 켜거나, `source ~/.bashrc`를 해주세요.

-----

### 2단계: 환경변수 적용 확인 (검증)

내가 설정한 XML 경로를 ROS 2가 진짜로 보고 있는지 확인해야 합니다.
**양쪽 터미널**에서 아래 명령어를 입력하세요.

```bash
echo $FASTRTPS_DEFAULT_PROFILES_FILE
```

  * **결과:** `/dir/fastdds_udp.xml` (사용자님이 지정한 실제 경로)가 나와야 합니다.
  * **주의:** 만약 아무것도 안 나오거나 경로가 틀렸다면 `.bashrc` 오타를 수정해야 합니다.
  * **팁:** 파일이 실제로 존재하는지도 보세요: `ls -lh /dir/fastdds_udp.xml`

-----

### 3단계: XML 파일 강화 (안전장치 추가)

기존 XML 설정에서 조금 더 강력하게 \*\*"기본 전송 방식(Builtin)을 끄고, 내가 정한 대용량 방식만 써라"\*\*고 강제하는 내용으로 수정해 드립니다.

**`fastdds_udp.xml` 내용을 아래와 같이 수정하세요 (양쪽 모두):**

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>udp_large_data</transport_id> <type>UDPv4</type>
            <sendBufferSize>26214400</sendBufferSize>
            <receiveBufferSize>26214400</receiveBufferSize>
            <maxMessageSize>65000</maxMessageSize>
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="udp_large_participant" is_default_profile="true">
        <rtps>
            <useBuiltinTransports>false</useBuiltinTransports> <userTransports>
                <transport_id>udp_large_data</transport_id> </userTransports>
            <use_simple_rtps_discovery>true</use_simple_rtps_discovery>
        </rtps>
    </participant>
</profiles>
```

-----

### 4단계: OS 커널 버퍼 확인 (재검증)

아까 `sysctl` 설정을 하셨지만, 재부팅 후 초기화되었을 수도 있으니 확인합니다.

```bash
sysctl net.core.rmem_max
```

  * **결과:** `26214400` (또는 그 이상)이 나와야 합니다.
  * **만약 `212992` 같은 작은 숫자가 나온다면:** `sudo sysctl -p`를 다시 입력하세요.

-----

### 실행 테스트

이제 다시 실행해 봅니다.

1.  **양쪽:** `ros2 daemon stop` (혹시 모르니 한번 더)
2.  **Jetson:** ZED 노드 실행 (`ros2 launch ...`)
3.  **IPC:** 대역폭 확인
    ```bash
    # echo 말고 bw로 먼저 확인하세요 (터미널 출력 부하 방지)
    ros2 topic bw /zed/zed_node/depth/depth_registered --qos-reliability best_effort
    ```

**이래도 안 된다면?**
정말 드문 경우지만, 설치된 ROS 2 패키지 중 `rmw-fastrtps-cpp`가 없거나 깨졌을 수 있습니다.
양쪽에서 아래 명령어로 설치를 확실히 해주세요.

```bash
sudo apt update
sudo apt install ros-humble-rmw-fastrtps-cpp
```

(Jetson이 Jazzy라면 `ros-jazzy-rmw-fastrtps-cpp`)

대부분 \*\*1단계(데몬 정지)\*\*와 \*\*3단계(XML Builtin 끄기)\*\*에서 해결됩니다.
#####################  
######################  
##########################  
네, 사용자님의 분석이 **정확합니다.** 의심하신 부분이 문제의 핵심 원인입니다.

사용자님은 **"로봇의 중심(base\_link)을 기준으로 Odometry를 사용"** 하고 싶은데, ZED Wrapper의 소스코드는 **"카메라 링크(zed\_camera\_link)를 강제로 Base로 지정"** 하고 있어 TF 트리 구조가 꼬이고 있습니다.

이로 인해 `zed_node`가 초기화될 때 TF 트리를 조회하다가, 자신이 예상한 프레임(`zed_camera_link`)과 실제 TF 트리의 구조가 맞지 않아 **무한 대기(Wait for transform)** 상태에 빠지고, 결과적으로 이미지조차 발행하지 않는 것입니다.

**소스코드를 수정해야 합니다.** 정확한 수정 방법을 알려드립니다.

-----

### [수정 가이드] `zed_camera_component_main.cpp` 수정

제공해주신 코드의 **732번째 라인 근처** (`getPosTrackingParams` 함수 내부)를 수정하여, 하드코딩된 값을 `common.yaml`에서 설정할 수 있도록 변경해야 합니다.

**파일 경로:** `zed-ros2-wrapper/zed_components/src/zed_camera/src/zed_camera_component_main.cpp`

#### 1\. 수정할 위치 찾기

함수: `void ZedCamera::getPosTrackingParams()` 내부

**[변경 전 - 원본 코드]**

```cpp
// ... (라인 730 근처)
  RCLCPP_INFO_STREAM(
    get_logger(), " * Positional tracking mode: "
      << sl::toString(mPosTrkMode).c_str());

  // ▼▼▼ 여기가 문제입니다. 무조건 카메라 링크를 Base로 지정하고 있습니다. ▼▼▼
  mBaseFrameId = mCameraName + "_camera_link"; 

  sl_tools::getParam(
    shared_from_this(), "pos_tracking.map_frame", mMapFrameId,
    mMapFrameId, " * Map frame id: ");
// ...
```

**[변경 후 - 수정 코드]**
`mBaseFrameId`를 하드코딩하지 않고, 파라미터 서버에서 가져오도록 수정합니다. 만약 파라미터가 없으면 기존처럼 동작하도록 기본값을 설정합니다.

```cpp
// ... (라인 730 근처)
  RCLCPP_INFO_STREAM(
    get_logger(), " * Positional tracking mode: "
      << sl::toString(mPosTrkMode).c_str());

  // ▼▼▼ [수정] yaml에서 base_frame을 읽어오도록 변경 ▼▼▼
  std::string defaultBaseFrame = mCameraName + "_camera_link"; // 기본값
  sl_tools::getParam(
    shared_from_this(), "pos_tracking.base_frame", mBaseFrameId,
    defaultBaseFrame, " * Base frame id: ");
  // ▲▲▲ [수정 끝] ▲▲▲

  sl_tools::getParam(
    shared_from_this(), "pos_tracking.map_frame", mMapFrameId,
    mMapFrameId, " * Map frame id: ");
// ...
```

-----

### 2\. `common.yaml` 설정 파일 추가

이제 소스코드가 수정되었으므로, `common.yaml` 파일에 `base_frame` 파라미터를 명시해주어야 합니다.

**파일:** `zed_ros2_wrapper/zed_wrapper/config/common.yaml` (또는 사용 중인 config 파일)

```yaml
pos_tracking:
    pos_tracking_enabled: true
    # ... 기존 설정들 ...
    
    # ▼ [추가] 이제 이 설정이 소스코드에 반영됩니다.
    base_frame: "base_link"  # 사용자님의 로봇 Base Frame 이름 (ipc에서 쏘는 이름)
    
    publish_tf: true         # ZED가 odom -> base_link를 발행하게 함
    publish_map_tf: true     # ZED가 map -> odom을 발행하게 함
```

-----

### 3\. 빌드 및 실행

소스코드를 수정했으므로, Docker 이미지를 다시 빌드하거나 워크스페이스를 재빌드해야 합니다.

1.  **재빌드:**
    ```bash
    cd ~/ros2_ws
    colcon build --symlink-install --packages-select zed_components
    source install/setup.bash
    ```
2.  **실행 및 확인:**
    이제 ZED 노드가 실행될 때 로그에 다음과 같이 떠야 합니다.
    > `* Base frame id: base_link`

-----

### 왜 이렇게 해야 해결되는가? (TF 흐름 설명)

**수정 전 (문제 상황):**

1.  ZED 노드는 `odom` -\> `zed_camera_link` 변환을 계산하려고 함.
2.  하지만 내부적으로 `getCamera2BaseTransform()` 함수(라인 1133)가 `zed_camera_link`와 `base_link` 사이의 관계를 찾으려 시도함.
3.  소스코드 상 `mBaseFrameId`가 `zed_camera_link`로 되어 있어, 자기 자신을 참조하거나 TF 트리가 꼬임.
4.  결국 "유효한 정적 변환(Static Transform)을 찾을 수 없음" 상태가 되어 무한 대기.

**수정 후 (정상 상황):**

1.  ZED 노드에게 "Base Frame은 `base_link`야"라고 알려줌.
2.  ZED 노드는 시작하자마자 TF 트리를 조회: "누가 `base_link` ↔ `zed_camera_link` 관계를 알고 있니?"
3.  \*\*IPC(다른 PC)\*\*에서 `tf_static`으로 `base_link` -\> `zed_camera_link`를 보내주고 있음.
4.  ZED 노드: "오케이, 관계 확인됨. 이제 내가 계산한 위치를 `odom` -\> `base_link`로 발행할게."
5.  **결과:** TF 트리가 `map` -\> `odom` -\> `base_link` -\> `zed_camera_link`로 완벽하게 연결됨.

이 수정을 적용하면 이미지 토픽도 정상적으로 나오고, TF 문제도 해결될 것입니다.
