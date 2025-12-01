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
