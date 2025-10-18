좋아, 지금까지 정리한 내용을 **pc1(서버+클라이언트) / pc2(클라이언트)** 기준으로 “그대로 따라 하면 되는” 형태로 step-by-step로 묶었어. (Humble↔Jazzy, Fast DDS Discovery Server, 대용량 토픽 안정화, Nitros→표준 ROS 경계 적용)

---

# 전체 목표 요약

* `serialize_message()` 전부 제거 (ROS2 기본 직렬화 사용)
* **공통 msg 버전 고정**(nvblox/isaac_ros_nvblox_msgs 등)
* **pc1**에서 **Fast DDS Discovery Server**를 항상 실행
* **pc1**의 ROS 노드 = **클라이언트(127.0.0.1:11811)**
  **pc2**의 ROS 노드 = **클라이언트(pc1_IP:11811)**
* **UDPv4만** 사용(SHM off), **QoS 다이어트**(대용량 토픽 BEST_EFFORT)
* **Nitros → 표준 ROS** 변환 후에만 inter-host 송수신

---

# 사전 준비 (두 PC 공통)

1. **공통 메시지 버전 고정**

   * `nvblox_msgs` / `isaac_ros_nvblox_msgs` / 사용 중인 커스텀 `.msg/.srv/.action`
   * 같은 Git tag/commit로 **pc1, pc2 모두 빌드/설치**
   * 확인:

     ```bash
     ros2 interface show nvblox_msgs/msg/DistanceMapSlice
     # 두 PC에서 출력이 완전히 동일해야 함
     ```

2. **직렬화 코드 제거**

   * `from rclpy.serialization import serialize_message` 등 **전부 제거**
   * 브릿지/리포블리시 등을 쓰지 말고 **직접 토픽 통신**으로 설계

3. **네트워크 기본**

   * 유선 NIC 같은 서브넷 (예: 192.168.10.0/24)
   * MTU 1500 권장(점보프레임 사용 중이면 우선 1500으로 통일해 안정성 확인)
   * 방화벽에서 **11811/UDP**(Discovery Server 포트) 허용

4. **Docker 권장 옵션**

   * 센서/지도/ESDF 스택은 가능하면:

     ```bash
     --net=host --ipc=host
     ```
   * (브리지 네트워크를 써야 한다면 반드시 Discovery Server 사용)

---

# pc1 (서버 + 클라이언트)

## 1) Discovery Server 실행(항상 켜두기)

```bash
# 포그라운드 테스트
fastdds discovery --server-id 0 --ip-address 0.0.0.0 --port 11811
# 또는 백그라운드
fastdds discovery --server-id 0 --ip-address 0.0.0.0 --port 11811 &
```

* 서비스로 관리하려면 systemd 또는 docker(—restart=always)로 등록해두면 편함.

## 2) Fast DDS 클라이언트 XML (pc1 전용)

`/opt/fastdds_client_pc1.xml` (루트/읽기전용 권장)

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">

  <participant profile_name="default_participant" is_default_profile="true">
    <rtps>
      <!-- inter-host에서 SHM은 의미 없고 충돌 소지 있어 끕니다 -->
      <useBuiltinTransports>false</useBuiltinTransports>
      <userTransports>
        <transport_id>udp_transport</transport_id>
      </userTransports>

      <builtin>
        <discovery_config>
          <discoveryProtocol>SERVER</discoveryProtocol>
          <leaseDuration>infinite</leaseDuration>
          <clientAnnouncementPeriod><sec>3</sec><nanosec>0</nanosec></clientAnnouncementPeriod>

          <!-- pc1은 로컬 서버에 붙습니다 -->
          <metatrafficUnicastLocatorList>
            <locator>
              <udpv4><address>127.0.0.1</address><port>11811</port></udpv4>
            </locator>
          </metatrafficUnicastLocatorList>
        </discovery_config>
      </builtin>

      <!-- 실제 유선 대역만 허용 -->
      <interfaceWhiteList>
        <address>192.168.10.0/24</address>
      </interfaceWhiteList>
    </rtps>
  </participant>

  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_transport</transport_id>
      <type>UDPv4</type>
      <sendBufferSize>4194304</sendBufferSize>     <!-- 4 MiB -->
      <receiveBufferSize>4194304</receiveBufferSize>
    </transport_descriptor>
  </transport_descriptors>

  <!-- 퍼블리셔 기본: 비동기/스루풋 컨트롤(폭주 방지) -->
  <publisher profile_name="default_publisher" is_default_profile="true">
    <qos>
      <publishMode><kind>ASYNCHRONOUS_PUBLISH_MODE</kind></publishMode>
      <throughputController>
        <bytesPerPeriod>1048576</bytesPerPeriod>
        <periodMillisec>10</periodMillisec>
      </throughputController>
    </qos>
  </publisher>

  <subscriber profile_name="default_subscriber" is_default_profile="true">
    <qos>
      <resourceLimitsQos>
        <max_samples>512</max_samples>
        <allocated_samples>32</allocated_samples>
      </resourceLimitsQos>
    </qos>
  </subscriber>

</profiles>
```

## 3) 환경변수 (pc1)

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE=/opt/fastdds_client_pc1.xml
```

## 4) Docker로 실행 예 (pc1)

```bash
docker run --rm -it --net=host --ipc=host \
  -e ROS_DOMAIN_ID -e RMW_IMPLEMENTATION \
  -e FASTDDS_DEFAULT_PROFILES_FILE=/opt/fastdds_client_pc1.xml \
  -v /opt/fastdds_client_pc1.xml:/opt/fastdds_client_pc1.xml:ro \
  <image_for_nav2_or_consumer>
```

---

# pc2 (클라이언트)

## 1) Fast DDS 클라이언트 XML (pc2 전용)

`/opt/fastdds_client_pc2.xml`

* **pc1의 유선 IP**(예: 192.168.10.10)를 반드시 넣기

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">

  <participant profile_name="default_participant" is_default_profile="true">
    <rtps>
      <useBuiltinTransports>false</useBuiltinTransports>
      <userTransports>
        <transport_id>udp_transport</transport_id>
      </userTransports>

      <builtin>
        <discovery_config>
          <discoveryProtocol>SERVER</discoveryProtocol>
          <leaseDuration>infinite</leaseDuration>
          <clientAnnouncementPeriod><sec>3</sec><nanosec>0</nanosec></clientAnnouncementPeriod>

          <!-- pc1 Discovery Server 주소 -->
          <metatrafficUnicastLocatorList>
            <locator>
              <udpv4><address>192.168.10.10</address><port>11811</port></udpv4>
            </locator>
          </metatrafficUnicastLocatorList>
        </discovery_config>
      </builtin>

      <interfaceWhiteList>
        <address>192.168.10.0/24</address>
      </interfaceWhiteList>
    </rtps>
  </participant>

  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_transport</transport_id>
      <type>UDPv4</type>
      <sendBufferSize>4194304</sendBufferSize>
      <receiveBufferSize>4194304</receiveBufferSize>
    </transport_descriptor>
  </transport_descriptors>

  <publisher profile_name="default_publisher" is_default_profile="true">
    <qos>
      <publishMode><kind>ASYNCHRONOUS_PUBLISH_MODE</kind></publishMode>
      <throughputController>
        <bytesPerPeriod>1048576</bytesPerPeriod>
        <periodMillisec>10</periodMillisec>
      </throughputController>
    </qos>
  </publisher>

  <subscriber profile_name="default_subscriber" is_default_profile="true">
    <qos>
      <resourceLimitsQos>
        <max_samples>512</max_samples>
        <allocated_samples>32</allocated_samples>
      </resourceLimitsQos>
    </qos>
  </subscriber>

</profiles>
```

## 2) 환경변수 (pc2)

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE=/opt/fastdds_client_pc2.xml
```

## 3) Docker로 실행 예 (pc2)

```bash
docker run --rm -it --net=host --ipc=host \
  -e ROS_DOMAIN_ID -e RMW_IMPLEMENTATION \
  -e FASTDDS_DEFAULT_PROFILES_FILE=/opt/fastdds_client_pc2.xml \
  -v /opt/fastdds_client_pc2.xml:/opt/fastdds_client_pc2.xml:ro \
  <image_for_zed_nvblox_producer>
```

---

# Isaac ROS / NVBlox 경계 설정 (중요)

1. **Nitros → 표준 ROS 메시지 변환**

   * inter-host로 나갈 토픽은 반드시 **표준 ROS 타입**으로 퍼블리시:

     * 예: `NitrosImage` → `sensor_msgs/Image`
     * `NitrosDistanceMapSlice` → `nvblox_msgs/DistanceMapSlice`
     * `NitrosPointCloud` → `sensor_msgs/PointCloud2`
   * 그래프/런처에서 변환 퍼블리셔가 **활성**인지 확인

2. **토픽 타입/이름 확인**

   ```bash
   ros2 topic info -t /nvblox/distance_slice
   # Type: nvblox_msgs/msg/DistanceMapSlice 나와야 OK
   ```

---

# QoS(대용량 토픽 안정화)

* **PointCloud2 / Image / Depth / ESDF slice**:

  * `reliability: best_effort`
  * `history: keep_last`
  * `depth: 1~5`
  * 퍼블리셔는 비동기(위 XML로 기본 설정)
* **맵/경로/TF(저대역폭 + 중요)**: RELIABLE 유지

예: `qos_overrides.yaml`

```yaml
/nvblox/distance_slice:
  reliability: best_effort
  history: keep_last
  depth: 3
/nvblox/esdf_pointcloud:
  reliability: best_effort
  history: keep_last
  depth: 1
/camera/left/image_rect_color:
  reliability: best_effort
  history: keep_last
  depth: 1
```

* 런처/노드 파라미터로 적용하거나 QoS override 기능 지원 시 사용

---

# 기동 순서(권장)

1. **pc1**: Discovery Server 시작
2. **pc1**: Nav2/소비자 측 스택 실행(환경변수/XML 설정된 상태)
3. **pc2**: ZED+Isaac ROS nvblox(생성자) 실행(환경변수/XML 설정된 상태)
4. 연결 확인:

   ```bash
   ros2 node list
   ros2 topic list
   ros2 topic info -t /nvblox/distance_slice
   ros2 topic hz /nvblox/distance_slice
   ros2 topic bw /nvblox/esdf_pointcloud
   ```

---

# 운영 팁 / 문제시 체크

* **pc2 OFF로 단독 운용**:
  pc1은 127.0.0.1만 서버로 바라보므로 지연 없이 정상 동작(구독자 없는 토픽만 생김)
* **간헐 끊김/버벅임**:
  BEST_EFFORT 적용 여부, `send/receiveBufferSize` 확대, Throughput Controller 확인
* **OOM 의심**:
  `dmesg -T | egrep -i 'oom|killed'`, Jetson이면 `tegrastats`로 피크 확인
* **RMW/설정 확인**:
  `echo $RMW_IMPLEMENTATION` / `ros2 doctor --report` / `echo $FASTDDS_DEFAULT_PROFILES_FILE`
* **방화벽/포트**:
  11811/UDP 열림, 양쪽에서 `ros2 node list`가 안정적으로 유지되는지 관찰
* **MTU**:
  1500으로 맞추고 테스트(스위치/도커 브릿지 구간 포함)

---

# (선택) 이중화 서버

* pc1에서 백업 서버 추가:

  ```bash
  fastdds discovery --server-id 1 --ip-address 0.0.0.0 --port 11812 &
  ```
* 두 XML의 `<metatrafficUnicastLocatorList>`에 **11811, 11812** 둘 다 추가 → 서버 재시작 시 재발견 속도↑

---

필요하면 네가 실제로 쓰는 **토픽 목록(이름/Hz/바이트 크기)** 알려줘. 그에 맞춰 `qos_overrides.yaml`을 더 촘촘히(ESDF slice/PointCloud2/이미지별 최적 depth 값 등) 손봐서 드릴게.
