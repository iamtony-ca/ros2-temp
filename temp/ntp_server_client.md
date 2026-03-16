네, **가능합니다.** 심지어 아주 전형적이고 권장되는 `chrony` 구성 방식입니다.

`chrony`는 기본적으로 Client 역할뿐만 아니라 Server 역할도 동시에 수행할 수 있도록 설계되어 있습니다. 질문하신 구조는 **Stratum(계층)** 구조로, Windows가 Stratum 1(또는 상위)이 되고, Ubuntu 1이 그 시간을 받아 다시 Stratum 2로서 Ubuntu 2(Stratum 3)에게 시간을 뿌려주는 형태가 됩니다.

Ubuntu 24.04 환경에서 ROS 2를 사용하시니 시간 동기화(Time Synchronization)가 TF tree나 센서 데이터 타임스탬프 정렬에 매우 중요하겠죠. 아래에 단계별 설정 방법을 정리해 드립니다.

---

### 1. 네트워크 토폴로지 가정

설명의 명확성을 위해 IP 주소를 다음과 같이 가정하겠습니다. 실제 환경에 맞춰 IP만 바꿔주세요.

* **Window1 (NTP Server)**: `192.168.1.10`
* **Ubuntu1 (Middleman)**:
* 랜카드1 (Window1 연결): `192.168.1.20`
* 랜카드2 (Ubuntu2 연결): `192.168.2.20`


* **Ubuntu2 (End Client)**: `192.168.2.30`

---

### 2. Ubuntu 1 설정 (Client & Server 겸용)

Ubuntu 1은 Window1의 시간을 받아오고(Client), 동시에 Ubuntu 2에게 시간을 제공(Server)해야 합니다.

**1) chrony 설치 (없다면)**

```bash
sudo apt update
sudo apt install chrony

```

**2) 설정 파일 수정**
`/etc/chrony/chrony.conf` 파일을 엽니다.

```bash
sudo nano /etc/chrony/chrony.conf

```

**3) 내용 수정**
기존 `pool ...` 라인들은 주석 처리(#) 하시고(폐쇄망이라면), 아래 내용을 추가합니다.

```conf
# [Client 설정] Window1을 NTP 서버로 지정
# iburst 옵션은 초기 동기화 속도를 높여줍니다.
server 192.168.1.10 iburst

# [Server 설정] Ubuntu2가 이 PC(Ubuntu1)의 시간을 가져갈 수 있도록 허용
# Ubuntu2의 IP 혹은 해당 서브넷 대역을 적어줍니다.
allow 192.168.2.0/24
# 또는 단일 IP만 허용하려면: allow 192.168.2.30

# (옵션) 네트워크 연결이 끊겨도 로컬 시계를 신뢰하여 서버 역할 유지
local stratum 10

```

**4) 서비스 재시작**

```bash
sudo systemctl restart chrony

```

---

### 3. Ubuntu 2 설정 (Client 전용)

Ubuntu 2는 Ubuntu 1만 바라보면 됩니다.

**1) 설정 파일 수정**

```bash
sudo nano /etc/chrony/chrony.conf

```

**2) 내용 수정**
마찬가지로 기존 pool을 주석 처리하고 Ubuntu 1을 바라보게 합니다.

```conf
# [Client 설정] Ubuntu1을 NTP 서버로 지정
# Ubuntu1의 랜카드2(Ubuntu2와 연결된 쪽) IP를 적습니다.
server 192.168.2.20 iburst

```

*(별도의 `allow` 설정은 필요 없습니다. 얘는 받기만 하니까요.)*

**3) 서비스 재시작**

```bash
sudo systemctl restart chrony

```

---

### 4. 검증 (Verification)

설정 직후 동기화까지 몇 초에서 몇 분 정도 걸릴 수 있습니다.

**Ubuntu 1 에서 확인:**

```bash
chronyc sources -v

```

* 결과에 Window1의 IP(`192.168.1.10`)가 보이고, 맨 앞에 `*` (현재 동기화 중) 또는 `+` 기호가 떠야 합니다.

**Ubuntu 2 에서 확인:**

```bash
chronyc sources -v

```

* 결과에 Ubuntu1의 IP(`192.168.2.20`)가 보이고, 마찬가지로 `*` 기호가 떠야 합니다.

---

### 5. 트러블슈팅 (중요 체크포인트)

ROS 2 통신 이슈를 방지하기 위해 다음 사항들을 꼭 체크하세요.

1. **방화벽 (UFW):**
Ubuntu 1은 서버 역할을 해야 하므로 NTP 포트(UDP 123)를 열어줘야 합니다.
```bash
# Ubuntu 1에서 실행
sudo ufw allow 123/udp

```


(Window1의 방화벽에서도 UDP 123 인바운드가 허용되어 있어야 Ubuntu1이 시간을 받아올 수 있습니다.)
2. **Stratum 레벨:**
Window1이 NTP 서버 설정을 제대로 했더라도, Stratum 정보가 너무 낮거나 불안정하면 Ubuntu1이 "신뢰할 수 없는 서버"로 간주하고 동기화를 거부할 수 있습니다.
* Ubuntu 1에서 `chronyc tracking`을 입력했을 때 `Leap status : Normal`이 아니라면 Window 설정을 의심해봐야 합니다.


3. **Local Stratum:**
만약 Window1과의 연결이 끊기더라도 Ubuntu1이 Ubuntu2에게 계속 시간을 뿌려주길 원한다면, 위 설정 예시의 `local stratum 10` 옵션이 필수적입니다. 이 옵션이 없으면 상위 서버(Window)와 끊어지는 순간 Ubuntu1은 "나도 시간 기준이 없어"라며 Ubuntu2의 요청을 무시하게 됩니다.

이 구성으로 진행하시면 ROS 2 멀티 로봇 환경에서 타임스탬프 꼬임 없이 깔끔하게 통신하실 수 있을 겁니다.
