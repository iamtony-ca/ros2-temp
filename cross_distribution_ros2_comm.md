좋아. \*\*A안(Initial Peers, 정적 피어)\*\*로
**PC1(우분투 22.04의 도커 컨테이너/Humble)** ↔ **PC2(우분투 24.04 호스트/Jazzy)** 를 유선 직결·고정 IP에서 멀티캐스트 없이 **유니캐스트로만** 안정 연결하는 **완전 실전 가이드**야.
핵심은: \*각 노드가 “내가 들을 메타트래픽 포트(로컬 바인드)”를 정하고, “상대가 듣는 주소(원격)를 initialPeersList에 적어준다”\*는 것. ([Fast DDS Documentation][1])

---

# 0) 예시 네트워크 / 용어

* PC1: `192.168.100.1` (도커 컨테이너는 `--net=host` 권장)
* PC2: `192.168.100.2`
* ROS 2 미들웨어: **Fast DDS** (`rmw_fastrtps_cpp`)
* XML 경로(예): `/etc/fastdds.xml` (아래 환경변수로 로드) ([Fast DDS Documentation][2])

---

# 1) 공통 환경 설정 (양쪽 모두)

```bash
# RMW 고정
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# 동일 도메인
export ROS_DOMAIN_ID=10
# Fast DDS XML 지정(권장)
export FASTDDS_DEFAULT_PROFILES_FILE=/etc/fastdds.xml
```

> `FASTDDS_DEFAULT_PROFILES_FILE` 로 XML을 읽는 게 공식 권장. (대안: 실행 디렉토리에 `DEFAULT_FASTDDS_PROFILES.xml`) ([Fast DDS Documentation][2])

* \*\*도커(PC1)\*\*는 멀티캐스트/UDP 이슈 피하려고 **host 네트워크 권장**:

```bash
docker run --rm -it --net=host --ipc=host \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e ROS_DOMAIN_ID=10 \
  -e FASTDDS_DEFAULT_PROFILES_FILE=/etc/fastdds.xml \
  -v /etc/fastdds.xml:/etc/fastdds.xml:ro \
  your_humble_image:latest bash
```

---

# 2) PC1용 XML 만들기 (`/etc/fastdds.xml`)

**포인트**

* `metatrafficUnicastLocatorList` = **내가(PC1) 수신할 메타트래픽 소켓** (보통 `0.0.0.0` + 고정 포트)
* `initialPeersList` = **상대(PC2)가 듣는 주소**
* 멀티캐스트는 비활성(리스트 비우기) → **정적 유니캐스트만** 사용 ([Fast DDS Documentation][3])

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <participant profile_name="ros2_default" is_default_profile="true">
      <rtps>
        <builtin>
          <discovery_config>
            <discoveryProtocol>SIMPLE</discoveryProtocol>
            <initialPeersList>
              <!-- 상대(PC2)가 '수신'할 메타트래픽 유니캐스트 주소 -->
              <locator>
                <udpv4>
                  <address>192.168.100.2</address>
                  <port>7412</port>
                </udpv4>
              </locator>
            </initialPeersList>
          </discovery_config>

          <!-- 내가(PC1) '수신'할 메타트래픽 유니캐스트 소켓(로컬 바인드) -->
          <metatrafficUnicastLocatorList>
            <locator>
              <udpv4>
                <address>0.0.0.0</address>
                <port>7411</port>
              </udpv4>
            </locator>
          </metatrafficUnicastLocatorList>

          <!-- 멀티캐스트 사용 안 함 -->
          <metatrafficMulticastLocatorList/>
        </builtin>

        <useBuiltinTransports>true</useBuiltinTransports>

        <!-- (선택) 특정 NIC만 허용하고 싶으면 allowlist -->
        <!--
        <participantPropertiesList>
          <property>
            <name>dds.transport.UDPv4.builtin.parent.allowlist</name>
            <value>192.168.100.1</value>
          </property>
        </participantPropertiesList>
        -->
      </rtps>
    </participant>
  </profiles>
</dds>
```

> `metatrafficUnicastLocatorList` 는 “**수신용 리스닝 로케이터**”다. 원격 IP를 넣는 곳이 아님. 보통 `0.0.0.0`으로 둬서 인터페이스 변화에도 안전하게 바인드. ([Fast DDS Documentation][3])

---

# 3) PC2용 XML 만들기 (`/etc/fastdds.xml`)

PC2는 서로 **포트를 다르게** 써서 충돌을 피한다(여기선 7412).

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <participant profile_name="ros2_default" is_default_profile="true">
      <rtps>
        <builtin>
          <discovery_config>
            <discoveryProtocol>SIMPLE</discoveryProtocol>
            <initialPeersList>
              <!-- 상대(PC1)가 '수신'하는 주소 -->
              <locator>
                <udpv4>
                  <address>192.168.100.1</address>
                  <port>7411</port>
                </udpv4>
              </locator>
            </initialPeersList>
          </discovery_config>

          <!-- 내가(PC2) '수신'할 메타트래픽 소켓 -->
          <metatrafficUnicastLocatorList>
            <locator>
              <udpv4>
                <address>0.0.0.0</address>
                <port>7412</port>
              </udpv4>
            </locator>
          </metatrafficUnicastLocatorList>

          <metatrafficMulticastLocatorList/>
        </builtin>
        <useBuiltinTransports>true</useBuiltinTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
```

---

# 4) 방화벽(있다면) 포트 허용

```bash
sudo ufw allow 7411:7412/udp
# 초기엔 ufw 비활성으로 검증 후, 필요한 포트만 열고 다시 활성화 권장
```

> Initial Peers는 멀티캐스트 포트(7400 등)를 안 써도 되고, **여기서 지정한 유니캐스트 포트만** 열면 됨. (Fast DDS 메타트래픽의 의미와 포트는 문서 참고) ([Fast DDS Documentation][3])

---

# 5) 동작 확인 (talker/listener)

**PC2(호스트/Jazzy)**:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=10
export FASTDDS_DEFAULT_PROFILES_FILE=/etc/fastdds.xml
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_ip
```

**PC1(컨테이너/Humble)**:

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=10
export FASTDDS_DEFAULT_PROFILES_FILE=/etc/fastdds.xml
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_ip
```

* `/chatter` 토픽이 서로 보이고 메시지가 오르면 성공.
* 이 구성은 \*\*멀티캐스트를 전혀 요구하지 않는 “정적 유니캐스트 발견”\*\*이다. ([Fast DDS Documentation][1])

---

# 6) 실패 시 체크리스트

1. **환경변수 로드 여부**

   * `echo $FASTDDS_DEFAULT_PROFILES_FILE`에 경로가 떠야 하고, 파일이 실제 있음. (Fast DDS가 이 경로의 XML을 자동 로드) ([Fast DDS Documentation][4])
   * `echo $RMW_IMPLEMENTATION` → `rmw_fastrtps_cpp` 확인.
2. **XML 포인트**

   * 원격 IP는 **반드시** `initialPeersList`에.
   * **로컬 바인드**는 `metatrafficUnicastLocatorList`(보통 `0.0.0.0` + 포트).
   * `metatrafficMulticastLocatorList`는 비워서 멀티캐스트 미사용(권장). ([Fast DDS Documentation][3])
3. **포트 충돌/차단**

   * 두 대가 **서로 다른 수신 포트**를 써야 한다(예: PC1=7411, PC2=7412).
   * UFW/방화벽에서 해당 UDP 포트를 허용.
4. **NIC 바인딩**

   * 특정 NIC만 쓰고 싶으면 allowlist 속성으로 제한(문서의 XML 설정 방법 참고). 먼저는 `0.0.0.0`으로 단순 바인드가 안전. ([Fast DDS Documentation][3])
5. **진단 팁**

   * `ros2 node list`, `ros2 topic list`로 가시성 확인.
   * `sudo tcpdump -i <nic> udp port 7411 or udp port 7412` 로 메타트래픽 유니캐스트 흐름 확인.

---

# 7) 자주 묻는 것

* **케이블이 빠지면?**
  로컬(같은 머신) 노드끼리는 계속 통신 가능. 다만 **로컬 바인드를 특정 NIC IP로 고정**해두면, NIC down 시 참가자 생성 실패 위험이 있으니 `0.0.0.0` 권장. (리스닝 로케이터 동작 상식) ([Fast DDS Documentation][3])
* **노드가 늘어나면?**
  Initial Peers는 아주 조용하고 단순하지만, 규모가 커지면 관리가 번거로울 수 있어. 그땐 **Discovery Server**나 **DDS Router**가 적합. (DS는 별도 문서) ([Fast DDS Documentation][5])

---

## 핵심만 다시

* **원격 주소**는 `initialPeersList`에, **내 수신 소켓**은 `metatrafficUnicastLocatorList`에.
* **둘의 수신 포트는 서로 다르게**, 멀티캐스트는 비워서 **유니캐스트만** 사용.
* XML은 `FASTDDS_DEFAULT_PROFILES_FILE`로 로드.
  이 구성이 Fast DDS의 정석적인 **Initial Peers(정적 발견)** 방식이다. ([Fast DDS Documentation][1])

원하면 위 예시를 네 실제 IP/파일 경로에 맞춰 **최소 설정 패키지(zip)** 형태로 정리해서 바로 쓸 수 있게 만들어줄게.

[1]: https://fast-dds.docs.eprosima.com/en/v3.2.2/fastdds/use_cases/wifi/initial_peers.html?utm_source=chatgpt.com "15.2.1. Configuring Initial Peers - 3.2.2"
[2]: https://fast-dds.docs.eprosima.com/en/latest/fastdds/ros2/ros2_configure.html?utm_source=chatgpt.com "16.1. Configuring Fast DDS in ROS 2 - 3.3.0"
[3]: https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/listening_locators.html?utm_source=chatgpt.com "6.8. Listening Locators - 3.3.0"
[4]: https://fast-dds.docs.eprosima.com/en/stable/fastdds/env_vars/env_vars.html?utm_source=chatgpt.com "11. Environment variables - 3.3.0"
[5]: https://fast-dds.docs.eprosima.com/en/latest/fastdds/ros2/discovery_server/ros2_discovery_server.html?utm_source=chatgpt.com "16.2. Use ROS 2 with Fast-DDS Discovery Server - 3.3.0"


##########################################  

좋아, \*\*B안(Discovery Server)\*\*를 PC1(우분투 22.04, 도커 컨테이너/Humble) ↔ PC2(우분투 24.04, 호스트/Jazzy) 유선 직결·고정 IP 환경에 맞춰 **완전 실전용 단계별 가이드**로 정리했어.
(핵심은 PC2에 Discovery Server를 띄우고, 양쪽 노드에서 `ROS_DISCOVERY_SERVER=<PC2_IP>:11811` 환경변수만 지정하는 것) ([ROS Docs][1])

---

# 0) 가정 & 표기

* IP 예시: **PC1=192.168.10.1**, **PC2=192.168.10.2** (같은 /24)
* ROS 2 미들웨어: **Fast DDS(rmw\_fastrtps\_cpp)** 사용
  (Discovery Server는 Fast DDS 기능이며, ROS 2는 `ROS_DISCOVERY_SERVER` 환경변수로 간단히 연동됨) ([ROS Docs][1])

---

# 1) 필수 패키지 확인

두 쪽 모두 Fast DDS RMW가 있어야 함.

**PC2 (Ubuntu 24.04, Jazzy 호스트):**

```bash
sudo apt update
sudo apt install ros-jazzy-rmw-fastrtps-cpp fastdds-tools  # fastdds CLI 포함
```

**PC1 (Ubuntu 22.04, 컨테이너 내부의 Humble):**

```bash
sudo apt update
sudo apt install ros-humble-rmw-fastrtps-cpp fastdds-tools
```

> `fastdds-tools`는 `fastdds discovery` CLI 제공. 이걸로 서버를 바로 띄울 수 있음. ([Fast DDS Documentation][2])

---

# 2) PC2에서 Discovery Server 실행

가장 간단한 방법(포트 11811 고정):

```bash
# PC2 (호스트)에서
fastdds discovery --server-id 0 --listen-address 0.0.0.0 --port 11811
```

* 이 명령 한 줄이면 **서버 모드**가 켜짐. (터미널을 열어둔 상태로 계속 실행되게) ([Fast DDS Documentation][2])
* 나중에 다중 서버를 원하면 백업 서버도 같은 방식으로 띄우고, 클라이언트에 세미콜론으로 여러 개 지정 가능(아래 4단계 참고). ([Fast DDS Documentation][3])

> 참고: ROS 공식 튜토리얼들도 같은 방식과 포트를 사용한 예시를 듦. ([ROS Docs][1])

---

# 3) 클라이언트 측(양쪽 노드) 환경변수 설정

**PC1(컨테이너)** 와 \*\*PC2(호스트)\*\*에서 **노드를 실행하는 터미널마다** 다음을 적용:

```bash
# 공통
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=10                       # 두 대 모두 동일 값
export ROS_DISCOVERY_SERVER=192.168.10.2:11811  # 서버가 돌아가는 PC2:포트
```

* 핵심은 `ROS_DISCOVERY_SERVER` 환경변수. 여기에 **서버의 IP:포트**를 지정하면 해당 터미널에서 실행되는 모든 ROS 2 프로세스가 **Discovery Server 클라이언트**로 동작한다. ([ROS Docs][1])
* 컨테이너는 **host 네트워크**를 권장(멀티캐스트 이슈를 피해가고 UDP가 깔끔해짐):

  ```bash
  docker run --rm -it --net=host --ipc=host \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e ROS_DOMAIN_ID=10 \
    -e ROS_DISCOVERY_SERVER=192.168.10.2:11811 \
    your_humble_image:latest bash
  ```

---

# 4) (선택) 고가용성/다중 서버

Discovery Server를 2대 이상 띄웠다면, 클라이언트는 세미콜론으로 나열하면 됨:

```bash
export ROS_DISCOVERY_SERVER="192.168.10.2:11811;192.168.10.3:11811"
```

* 이 목록은 **재연결 경로** 등에 영향을 주며, 이미 확립된 연결을 바로 끊지는 않음(서버 목록을 바꿔도 기존 세션은 유지됨). ([Fast DDS Documentation][3])

---

# 5) 방화벽 / 포트

* 서버 포트(예시 11811/UDP)를 열어야 함:

  ```bash
  sudo ufw allow 11811/udp
  ```
* 직결 환경에서는 보통 방화벽 이슈가 없지만, 정책이 켜져 있으면 꼭 허용.

---

# 6) 동작 테스트

**PC2(서버 측)**:

```bash
# 터미널1: 서버 그대로 유지 (2단계에서 실행 중)
# 터미널2:
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=10
export ROS_DISCOVERY_SERVER=192.168.10.2:11811
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_ds
```

**PC1(컨테이너 측)**:

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=10
export ROS_DISCOVERY_SERVER=192.168.10.2:11811
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_ds
```

* `/chatter`가 정상 송수신되면 성공. (ROS 튜토리얼 절차와 동일) ([ROS Docs][1])

---

# 7) 운영 팁(문제 예방)

1. **RMW 확인:**
   `echo $RMW_IMPLEMENTATION` → `rmw_fastrtps_cpp` 여야 함. (Cyclone 등 다른 RMW이면 Discovery Server가 적용 안 됨) ([ROS Docs][1])
2. **혼합 디스커버리 금지:**
   어떤 노드는 Discovery Server, 어떤 노드는 기본(Simple)로 섞으면 서로 **발견 자체가 안 됨**. 전부 동일하게 DS를 쓰거나, 필요 시 **ROS 2 Router**로 브리지해야 함. ([Vulcanexus Docs][4])
3. **NIC 고정이 필요하면:**
   XML 프로필로 허용 NIC를 제한(allowlist)할 수 있음. (초기에는 굳이 필요 없고, 문제 생길 때 적용 권장) ([Read the Docs][5])
4. **로그·상태 확인:**
   서버 터미널의 로그로 접속/해제 확인 가능.
5. **포트/도메인 충돌 회피:**
   같은 머신에 다른 DS를 추가로 띄우면 포트 바꿔야 함(예: 11812). `ROS_DOMAIN_ID`는 노드 간 동일 값 유지. ([Fast DDS Documentation][2])

---

# 8) systemd로 서버 상시 실행(선택)

PC2에 서비스로 등록하면 재부팅 후 자동 시작 가능:

```ini
# /etc/systemd/system/fastdds-discovery.service
[Unit]
Description=Fast DDS Discovery Server
After=network-online.target
Wants=network-online.target

[Service]
ExecStart=/usr/bin/fastdds discovery --server-id 0 --listen-address 0.0.0.0 --port 11811
Restart=on-failure
User=root

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now fastdds-discovery.service
systemctl status fastdds-discovery.service
```

---

# 9) 자주 묻는 질문(케이블 뽑힘/단절 시)

* **서버(PC2)와의 링크가 끊겨도,** 같은 호스트 내 로컬 노드 간 통신은 계속 잘됨. 원격 노드는 재연결 시 다시 발견됨.
* 서버 IP/포트가 바뀌면 클라이언트 터미널의 `ROS_DISCOVERY_SERVER`를 현재 값으로 다시 지정해야 한다. (이미 연결된 세션은 즉시 끊지 않음) ([Fast DDS Documentation][3])

---

## 요약

1. PC2에서 `fastdds discovery --port 11811`로 서버 실행 →
2. 양쪽 노드 터미널에서 `export ROS_DISCOVERY_SERVER=192.168.10.2:11811` + `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` + `ROS_DOMAIN_ID` 통일 →
3. `talker`/`listener`로 검증.
   이게 Discovery Server의 가장 간단하고 견고한 구성이고, **Humble ↔ Jazzy 혼합 환경**에서도 동일하게 동작한다(둘 다 Fast DDS 사용 시). ([ROS Docs][1])

필요하면 **다중 서버(HA)**, **특정 NIC 고정**, **Router로 다른 도메인/디스커버리 방식 브리지** 같은 고급 구성을 바로 이어서 잡아줄게.

[1]: https://docs.ros.org/en/jazzy/Tutorials/Advanced/Discovery-Server/Discovery-Server.html?utm_source=chatgpt.com "Using Fast DDS Discovery Server as ..."
[2]: https://fast-dds.docs.eprosima.com/en/latest/fastddscli/cli/cli.html?utm_source=chatgpt.com "1. CLI - 3.3.0"
[3]: https://fast-dds.docs.eprosima.com/en/v3.2.2/fastdds/discovery/discovery_server.html?utm_source=chatgpt.com "5.3.4. Discovery Server Settings - 3.2.2"
[4]: https://docs.vulcanexus.org/en/iron/rst/tutorials/cloud/change_domain_discovery_server/change_domain_discovery_server.html?utm_source=chatgpt.com "3.2. Change ROS 2 Domain to Discovery Server"
[5]: https://media.readthedocs.org/pdf/eprosima-fast-rtps/stable/eprosima-fast-rtps.pdf?utm_source=chatgpt.com "Fast DDS Documentation"

