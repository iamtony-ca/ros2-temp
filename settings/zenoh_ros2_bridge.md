제공해주신 Zenoh README 내용과 사용자분의 환경(Orin AGX/Humble, IPC/Jazzy, 동일 서브넷 유선 연결)을 바탕으로, 두 호스트 간 ROS 2 통신을 위한 Zenoh Bridge 튜토리얼을 작성해 드립니다.

사용자분의 프로필을 고려하여 **Docker를 활용한 방식**으로 구성했습니다. 이 방식은 호스트의 의존성(Rust 버전 등)을 신경 쓸 필요가 없고, Orin(arm64)과 IPC(amd64) 모두에서 동일한 이미지를 사용할 수 있어 가장 깔끔합니다.

-----

# Zenoh Bridge를 활용한 ROS 2 Humble(Orin) ↔ Jazzy(IPC) 통신 튜토리얼

이 튜토리얼의 핵심 목표는 **서로 다른 ROS 2 버전**과 **서로 다른 호스트**를 Zenoh 프로토콜을 통해 연결하는 것입니다. Zenoh Bridge는 로컬 DDS 트래픽을 Zenoh 패킷으로 변환하여 TCP를 통해 상대방에게 전달하고, 다시 DDS로 변환해주는 역할을 합니다.

### 📋 사전 준비 및 핵심 원칙

README에서 가장 강조하는 부분은 \*\*"두 호스트 간에 직접적인 DDS 통신(Multicast)이 발생하면 안 된다"\*\*는 것입니다. 만약 Zenoh도 연결되어 있고, 유선랜을 통해 DDS Multicast도 도달한다면 메시지가 무한 루프(Looping traffic)를 돌게 됩니다.

따라서 우리는 **ROS\_DOMAIN\_ID를 분리**하여 물리적으로는 연결되어 있더라도 논리적으로는 서로 다른 DDS 네트워크에 있도록 설정합니다.

  * **Orin AGX (Humble):** `ROS_DOMAIN_ID=0` (Leader/Server 역할)
  * **IPC (Jazzy):** `ROS_DOMAIN_ID=1` (Client/Connector 역할)

-----

### Step 1. Docker 이미지 준비 (양쪽 모두)

Orin AGX와 IPC 양쪽 터미널에서 아래 명령어를 실행하여 Zenoh Bridge 이미지를 다운로드합니다. README에 따르면 amd64와 arm64 모두 지원합니다.

```bash
docker pull eclipse/zenoh-bridge-ros2dds:latest
```

-----

### Step 2. Orin AGX 설정 (Listener 모드)

Orin AGX는 `ROS_DOMAIN_ID=0`을 바라보며, 외부(IPC)에서 들어오는 Zenoh 연결을 기다리는 역할을 합니다.

  * **가정:** Orin의 IP 주소가 `192.168.1.10`이라고 가정합니다.
  * **설명:**
      * `--net host`: 호스트의 네트워크 인터페이스를 그대로 사용해야 로컬 ROS 2 DDS 트래픽을 캡처할 수 있습니다.
      * `ROS_DOMAIN_ID=0`: Orin 내부의 Humble 노드들과 통신하기 위함입니다.
      * 별도의 `-e` 옵션이 없으면 기본적으로 `0.0.0.0:7447` 포트에서 연결을 대기(Listen)합니다.

**Orin 터미널 실행:**

```bash
# Orin AGX (Humble)
docker run --rm -it --net host \
  -e ROS_DOMAIN_ID=0 \
  eclipse/zenoh-bridge-ros2dds:latest
```

실행 후 로그에 `Sleeping for 1s...` 혹은 `Started Zenoh...`와 함께 IP와 포트(7447)가 보이면 정상입니다.

-----

### Step 3. IPC 설정 (Connector 모드)

IPC는 `ROS_DOMAIN_ID=1`을 바라보며, Orin AGX에게 TCP로 연결을 시도합니다.

  * **설정:**
      * `ROS_DOMAIN_ID=1`: Orin과 겹치지 않게 하여 직접적인 DDS 통신을 막습니다. (IPC 내부 Jazzy 노드들은 ID 1을 사용해야 함)
      * `-e tcp/192.168.1.10:7447`: Orin의 IP로 연결을 시도합니다. (IP는 실제 Orin IP로 변경해주세요)

**IPC 터미널 실행:**

```bash
# IPC (Jazzy) - Orin의 IP가 192.168.1.10 이라고 가정
docker run --rm -it --net host \
  -e ROS_DOMAIN_ID=1 \
  eclipse/zenoh-bridge-ros2dds:latest -e tcp/192.168.1.10:7447
```

실행 후 로그에 `Session established with ...` 메시지가 뜨면 두 브리지 간의 연결이 성공한 것입니다.

-----

### Step 4. 통신 테스트

이제 실제로 토픽이 넘어가는지 확인합니다. 중요한 점은 **각 터미널에서 테스트할 때도 `ROS_DOMAIN_ID`를 맞춰주어야 한다는 점**입니다.

#### 1\. Orin (Publisher) → IPC (Subscriber)

**Orin (새 터미널):**

```bash
export ROS_DOMAIN_ID=0
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello from Orin'"
```

**IPC (새 터미널):**

```bash
export ROS_DOMAIN_ID=1
# 토픽 리스트 확인 (잠시 후 보여야 함)
ros2 topic list
# 데이터 확인
ros2 topic echo /chatter
```

#### 2\. IPC (Publisher) → Orin (Subscriber)

반대로 IPC에서 발행하고 Orin에서 받는 것도 동일하게 작동해야 합니다.

-----

### Step 5. (심화) Namespace 설정 (추천)

README의 "Easy multi-robots via Namespace configuration" 섹션을 보면, 여러 로봇을 다룰 때 충돌을 방지하기 위해 브리지 자체에 네임스페이스를 부여하는 기능을 추천하고 있습니다.

만약 Orin AGX가 모바일 매니퓰레이터 본체라면, Orin 쪽 브리지를 실행할 때 `--ros-namespace` 옵션(또는 config)을 주면 편리합니다.

**Orin AGX 실행 시 (네임스페이스 적용 예시):**

```bash
docker run --rm -it --net host \
  -e ROS_DOMAIN_ID=0 \
  eclipse/zenoh-bridge-ros2dds:latest -n /mobile_manipulator
```

이렇게 실행하면:

1.  Orin 내부에서는 원래 토픽명(예: `/scan`)으로 통신합니다.
2.  Zenoh를 타고 IPC로 넘어갈 때는 `/mobile_manipulator/scan`으로 자동 변환되어 넘어갑니다.
3.  IPC(관제 PC)에서는 `/mobile_manipulator/scan`으로 데이터를 받게 되어, 나중에 로봇이 여러 대가 되어도 구분하기 쉽습니다.

-----

### 요약 및 주의사항

1.  **DDS 격리 필수:** Orin과 IPC는 유선으로 직결되어 있으므로, 반드시 서로 다른 `ROS_DOMAIN_ID`를 사용하여 "Raw DDS 통신"과 "Zenoh 통신"이 섞이지 않게 하십시오.
2.  **방화벽:** 혹시 연결이 안 된다면 Ubuntu `ufw` 방화벽에서 `7447` 포트(Zenoh)와 UDP 포트들(DDS용)이 열려 있는지 확인하세요. (사내망이나 폐쇄망이면 보통 꺼져 있을 확률이 높습니다)
3.  **ROS 2 버전:** Humble(DDS 기본값: CycloneDDS)과 Jazzy(DDS 기본값: eProsima Fast DDS)는 RMW 구현체가 달라도 Zenoh Bridge가 중간에서 표준 DDS 프로토콜을 처리해주므로 큰 문제 없이 통신됩니다. 다만, 혹시라도 이슈가 생기면 양쪽 모두 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`로 통일하는 것이 정신 건강에 이롭습니다.

이 튜토리얼대로 진행해 보시고, 연결 로그(`Session established`)가 뜨지 않는다면 Orin의 IP와 방화벽 상태를 먼저 점검해 주세요.
########################  
#############################  
##############################  
네, **정확합니다.**

각 호스트(Orin과 IPC)에서 **`zenoh-bridge-ros2dds` Docker 컨테이너를 딱 1개씩만 실행**해주면, 그 안에 있는 모든 ROS 2 노드들이 **양방향**으로 통신할 수 있습니다.

### 왜 1개면 충분한가요?

Zenoh Bridge는 "특정 토픽 하나"만 연결하는 파이프가 아니라, **해당 `ROS_DOMAIN_ID` 내의 모든 DDS 트래픽을 관장하는 "게이트웨이(Gateway)" 역할**을 하기 때문입니다.

작동 원리를 간단히 도식화하면 다음과 같습니다:

1.  **Orin 측 브리지**는 Orin 내부(Domain 0)의 **모든** 토픽/서비스를 감지하고, 외부(IPC)로 넘어갈 데이터가 있다면 Zenoh 패킷으로 압축해서 보냅니다.
2.  동시에, IPC에서 Zenoh 패킷이 넘어오면 즉시 풀어서 Orin 내부(Domain 0)에 **DDS로 다시 뿌려줍니다(Re-publish)**.
3.  **IPC 측 브리지**도 똑같이 작동합니다.

### 실행 요약 (다시 확인)

이미 구성하신 대로 아래 상태면 완벽합니다.

**1. Orin AGX (Host A)**

  * 실행: `zenoh-bridge` 컨테이너 1개
  * 역할: Humble 시스템(Domain 0)의 대변인
  * 설정: `ROS_DOMAIN_ID=0`, `Mode=Listen (Server)`

**2. IPC (Host B)**

  * 실행: `zenoh-bridge` 컨테이너 1개
  * 역할: Jazzy 시스템(Domain 1)의 대변인
  * 설정: `ROS_DOMAIN_ID=1`, `Mode=Connect (Client) -> Orin IP`

### 💡 전문가 팁: "정말 다 연결된 건가?" 확인하는 법

컨테이너 2개만 딱 띄워놓고, **Orin** 터미널에서 아래 명령어를 쳐보세요.

```bash
# Orin 터미널에서 실행 (IPC에 있는 노드가 보여야 함)
export ROS_DOMAIN_ID=0
ros2 node list
```

놀랍게도 IPC(Jazzy/Domain 1)에서 실행 중인 노드 이름들이 리스트에 뜰 것입니다. Zenoh Bridge가 노드 정보(Discovery info)까지 교환해주기 때문에, 마치 로컬에 있는 것처럼 `ros2 topic list`, `ros2 service list`가 모두 통합되어 보입니다.

**결론:** 토픽이 100개든 1000개든, 컨테이너는 양쪽에 하나씩만 있으면 됩니다.
#####################  
######################  
#######################  
좋은 질문입니다. \*\*"Server(Listener) 쪽은 왜 IP 옵션이 없는가?"\*\*에 대한 의문과 \*\*"역할 변경"\*\*에 대해 명확히 설명해 드릴게요.

결론부터 말씀드리면, **Server(Listener) 역할은 "누구에게 걸겠다(`-e`)"는 옵션이 없으면 자동으로 "모든 IP(`0.0.0.0`)에서 전화 올 때까지 대기"하는 모드**로 작동하기 때문입니다.

상황에 따라 역할을 바꾸는 방법(Scenario A vs B)을 정리해 드립니다.

-----

### 1\. `-e` 옵션과 `-l` 옵션의 차이

Zenoh Bridge 실행 시 두 가지 핵심 옵션이 있습니다.

  * **`-e` (Connect Endpoint):** "내가 저쪽 IP로 전화를 **걸겠다**." (Client 역할)
  * **`-l` (Listen Endpoint):** "나는 내 IP에서 전화를 **기다리겠다**." (Server 역할)
      * **생략 시 기본값:** `-l tcp/0.0.0.0:7447` (내 PC의 **모든** 랜카드/IP에서 7447 포트로 들어오는 연결을 받음)

따라서 아까 Orin에서 `-e`를 안 쓴 이유는, \*\*"Orin은 가만히 대기하고, IPC가 접속해오길 기다리기 때문"\*\*입니다.

-----

### 2\. 상황별 역할 설정 (Scenario A vs B)

두 호스트가 유선으로 직결되어 있으니, **IP가 고정되어 있거나 잘 변하지 않는 쪽**을 Server(Listener)로 두는 것이 편합니다. 성능 차이는 전혀 없습니다. 연결만 되면 양방향 통신은 똑같습니다.

두 호스트의 IP를 아래와 같이 가정하고 설정을 비교해 보겠습니다.

  * **Orin IP:** `192.168.1.10`
  * **IPC IP:** `192.168.1.20`

#### 상황 A: Orin이 대기하고(Server), IPC가 접속(Client)

*가장 추천하는 방식 (로봇 본체인 Orin이 서버 역할을 하는 것이 관리상 편할 때가 많음)*

  * **Orin (Server):** 옵션 없이 실행 (혹은 명시적으로 `-l` 사용)
    ```bash
    # 모든 인터페이스에서 대기 (기본값)
    docker run --rm -it --net host -e ROS_DOMAIN_ID=0 eclipse/zenoh-bridge-ros2dds:latest

    # [권장] 만약 특정 유선 LAN IP로만 대기하고 싶다면:
    docker run --rm -it --net host -e ROS_DOMAIN_ID=0 eclipse/zenoh-bridge-ros2dds:latest -l tcp/192.168.1.10:7447
    ```
  * **IPC (Client):** Orin의 IP를 적어줌
    ```bash
    docker run --rm -it --net host -e ROS_DOMAIN_ID=1 eclipse/zenoh-bridge-ros2dds:latest -e tcp/192.168.1.10:7447
    ```

#### 상황 B: IPC가 대기하고(Server), Orin이 접속(Client)

*만약 로봇(Orin) IP가 DHCP로 자주 바뀌고, 관제 PC(IPC) IP가 고정이라면 이 방식이 유리함*

  * **IPC (Server):** 옵션 없이 실행
    ```bash
    # IPC가 대기 모드
    docker run --rm -it --net host -e ROS_DOMAIN_ID=1 eclipse/zenoh-bridge-ros2dds:latest
    ```
  * **Orin (Client):** IPC의 IP를 적어줌
    ```bash
    # Orin이 IPC에게 접속 시도
    docker run --rm -it --net host -e ROS_DOMAIN_ID=0 eclipse/zenoh-bridge-ros2dds:latest -e tcp/192.168.1.20:7447
    ```

-----

### 3\. 개발자님을 위한 추천 설정

현재 **유선 직결** 상태라고 하셨으므로, 물리적으로 연결된 이더넷 인터페이스의 IP를 서로 확실히 알고 계실 겁니다.

보안과 명확성을 위해 **Server 역할을 하는 쪽도 `-l` (listen) 옵션을 명시적으로 적어주는 것**을 추천합니다. 그래야 실수로 와이파이 망 등 다른 네트워크로 패킷이 새는 것을 방지할 수 있습니다.

**최종 추천 커맨드 (Orin을 Server로 쓸 경우):**

```bash
# [Orin] 내 유선 IP가 192.168.1.10 이라면:
docker run --rm -it --net host -e ROS_DOMAIN_ID=0 eclipse/zenoh-bridge-ros2dds:latest -l tcp/192.168.1.10:7447

# [IPC] Orin으로 접속:
docker run --rm -it --net host -e ROS_DOMAIN_ID=1 eclipse/zenoh-bridge-ros2dds:latest -e tcp/192.168.1.10:7447
```

이렇게 하면 역할이 바뀌더라도 `-l` 과 `-e` 옵션만 서로 바꾸고 IP만 상대방 것으로 맞춰주면 됩니다.
########################  
###########################  
############################  
네, **물론 가능합니다.** Zenoh Bridge는 매우 강력한 **토픽 필터링(Allow/Deny List)** 기능을 제공합니다.

전체 대역폭을 아끼거나 보안을 위해 \*\*"내가 지정한 토픽만 보내고 받고 싶다"\*\*는 경우, **`--allow` (또는 `-a`)** 옵션을 사용하면 됩니다.

Docker 환경에서 이를 설정하는 두 가지 방법(간편한 방법 vs 관리하기 좋은 방법)을 알려드리겠습니다.

-----

### 방법 1. 커맨드 라인 인자 사용 (간편한 방법)

테스트 단계에서 몇 개의 토픽만 빠르게 지정할 때 유용합니다. `docker run` 명령어 뒤에 `-a` 옵션을 반복해서 적어주면 됩니다.

**규칙:** `-a` 옵션을 하나라도 적는 순간, **그 외의 모든 토픽은 자동으로 차단(Deny)** 됩니다.

예를 들어, Orin에서 **`/scan`** (라이다)과 **`/cmd_vel`** (속도 명령)만 통신하고 싶다면:

```bash
# Orin AGX (Server 예시)
docker run --rm -it --net host \
  -e ROS_DOMAIN_ID=0 \
  eclipse/zenoh-bridge-ros2dds:latest \
  -l tcp/192.168.1.10:7447 \
  -a /scan \
  -a /cmd_vel
```

이렇게 하면 Orin의 브리지는 오직 `/scan`과 `/cmd_vel` 데이터만 Zenoh 네트워크로 흘려보내거나 받습니다. `/tf`나 `/robot_description` 같은 다른 데이터는 무시됩니다.

-----

### 방법 2. 설정 파일(Config File) 사용 (추천)

관리해야 할 토픽이 많아지거나 정규표현식을 복잡하게 써야 한다면, JSON5 설정 파일을 만들어서 Docker에 마운트하는 것이 훨씬 깔끔합니다.

#### 1\. `config.json5` 파일 작성 (Orin/IPC 호스트에 생성)

원하는 경로(예: `~/zenoh_conf/config.json5`)에 아래 파일을 만듭니다.

```json5
{
  // Zenoh 연결 설정
  // (커맨드 라인 인자가 우선순위가 높으므로 여기선 생략해도 되지만 명시 가능)
  // listen: { endpoints: ["tcp/192.168.1.10:7447"] }, 

  ros2: {
    // 여기에 허용할 토픽 리스트 작성 (정규표현식 지원)
    allow: [
      "/cmd_vel",          // 정확히 일치하는 토픽
      "/scan",             // 정확히 일치하는 토픽
      "/robot_1/.*",       // /robot_1/ 아래의 모든 하위 토픽 허용
      "/tf",               // TF도 필요하다면 명시 필수!
      "/tf_static"
    ]
  }
}
```

#### 2\. Docker 실행 시 파일 마운트

작성한 파일을 컨테이너 내부(`/config.json5`)로 마운트하고, `-c` 옵션으로 경로를 지정해 줍니다.

```bash
# Orin AGX 실행 예시
docker run --rm -it --net host \
  -v $(pwd)/config.json5:/config.json5 \
  -e ROS_DOMAIN_ID=0 \
  eclipse/zenoh-bridge-ros2dds:latest \
  -l tcp/192.168.1.10:7447 \
  -c /config.json5
```

-----

### 💡 중요한 팁 (개발자 노트)

1.  **정규 표현식(Regex) 지원:**

      * `-a /chatter` : 정확히 `/chatter`만 허용합니다.
      * `-a /my_robot/.*` : `/my_robot/`으로 시작하는 모든 토픽을 허용합니다. (예: `/my_robot/status`, `/my_robot/pos`)
      * 만약 모든 토픽을 다 보내고 싶은데 특정 토픽만 **빼고** 싶다면, `-a` 대신 **`--deny` (`-d`)** 옵션을 쓰면 됩니다. (예: 카메라 이미지처럼 무거운 `/camera/raw_image`만 차단)

2.  **`/tf` 주의:**

      * Nav2 등을 사용하신다면 \*\*`/tf`\*\*와 **`/tf_static`** 토픽을 허용 리스트에 넣는 것을 잊지 마세요. 필터링을 걸면 이것들도 같이 차단되어 Rviz에서 로봇이 분해되어 보이거나 좌표 변환이 안 될 수 있습니다.

3.  **양쪽 설정의 대칭성:**

      * 가장 확실한 방법은 \*\*Orin과 IPC 양쪽 모두 동일한 필터 규칙(`-a` 목록)\*\*을 적용하는 것입니다.
      * 대역폭을 아끼는 것이 목적이므로, \*\*데이터를 보내는 쪽(Publisher)\*\*에서 필터링을 거는 것이 핵심입니다. (Orin이 무거운 데이터를 굳이 Zenoh 네트워크로 밀어넣지 않도록 Orin 쪽 브리지에 `-a`를 거는 것이 효율적입니다.)

어떤 방식을 선호하시나요? 토픽이 몇 개 안 된다면 **방법 1**로 바로 테스트해보시는 걸 추천합니다.
