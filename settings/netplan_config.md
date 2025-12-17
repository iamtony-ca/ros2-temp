제시해주신 명령어들은 크게 **커널 파라미터(`sysctl`)**와 **네트워크 인터페이스 설정(`ip link`)** 두 가지로 나뉩니다. 각각 영구 적용하는 방법이 다릅니다.

IPC(Ubuntu 24.04)와 Jetson(Ubuntu 22.04) 모두 동일한 방식으로 적용 가능합니다.

---

### 1. 커널 파라미터 영구 적용 (`sysctl` 설정)

`sysctl` 설정값은 `/etc/sysctl.d/` 디렉토리에 설정 파일을 만들어서 저장하면 재부팅 시 자동으로 로드됩니다.

터미널에서 다음 명령어를 한 번에 복사해서 붙여넣으세요. (설정 파일을 생성하는 명령어입니다)

```bash
sudo bash -c 'cat > /etc/sysctl.d/99-ros2-cyclonedds.conf <<EOF
# ROS 2 Cyclone DDS Tuning
net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
net.core.rmem_max=2147483647
net.core.rmem_default=2147483647
net.core.wmem_max=2147483647
net.core.wmem_default=2147483647
EOF'

```

**적용 확인:**
재부팅 없이 바로 적용하려면 아래 명령어를 입력합니다.

```bash
sudo sysctl -p /etc/sysctl.d/99-ros2-cyclonedds.conf

```

---

### 2. MTU 9000 영구 적용 (`ip link` 설정)

`ip link` 명령어로 설정한 MTU는 재부팅하면 초기화됩니다. Ubuntu에서는 **Netplan**을 통해 영구 설정해야 합니다.

**주의:** 작성해주신 `eno1`은 주로 PC(IPC)의 랜카드 이름입니다. **Jetson은 보통 `eth0`**일 수 있으니, 각 장비에 맞는 이름으로 설정해야 합니다.

#### 1단계: Netplan 설정 파일 찾기

```bash
ls /etc/netplan/

```

보통 `01-netcfg.yaml`, `00-installer-config.yaml`, `01-network-manager-all.yaml` 중 하나가 있습니다. 해당 파일을 편집합니다.

```bash
sudo nano /etc/netplan/00-installer-config.yaml  # 파일명은 실제 있는 것으로 수정하세요

```

#### 2단계: 내용 수정 (들여쓰기 주의!)

파일을 열어 해당 인터페이스(`eno1` 또는 `eth0`) 아래에 `mtu: 9000`을 추가합니다.

**예시 (IPC - eno1):**

```yaml
network:
  version: 2
  renderer: networkd # 또는 NetworkManager
  ethernets:
    eno1:
      dhcp4: true
      mtu: 9000   # <--- 이 줄을 추가 (들여쓰기 정확히 맞춰야 함)

```

**예시 (Jetson - eth0 인 경우):**

```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: true
      mtu: 9000   # <--- 이 줄을 추가

```

#### 3단계: 적용

저장(`Ctrl+O`, `Enter`)하고 나 온(`Ctrl+X`) 뒤, 아래 명령어로 적용합니다.

```bash
sudo netplan apply

```

적용 후 `ifconfig eno1` (또는 `eth0`)을 쳤을 때 `mtu 9000`으로 나오면 성공입니다.

---

### (보너스) `txqueuelen` 영구 적용법

아까 제가 추천해 드렸던 `txqueuelen 10000` 설정은 Netplan으로 안 되는 경우가 많습니다. 이것까지 영구 적용하려면 **udev 규칙**을 만드는 것이 가장 확실합니다.

```bash
# 99-txqueue.rules 파일 생성
sudo nano /etc/udev/rules.d/99-txqueue.rules

```

파일 안에 아래 내용을 적습니다. (`eno1`은 장비에 맞게 변경하세요)

```bash
SUBSYSTEM=="net", ACTION=="add|change", KERNEL=="eno1", ATTR{tx_queue_len}="10000"

```

*(Jetson의 경우 `KERNEL=="eth0"`로 변경)*

저장 후 재부팅하면 자동으로 적용됩니다.
##################################  
전혀 당황할 필요 없습니다! Ubuntu 버전이나 설치 방식(Server vs Desktop, Cloud-init 사용 여부)에 따라 파일 이름은 다를 수 있습니다.

Netplan은 파일 이름의 **숫자 순서대로(Lexical order)** 설정을 읽어 들입니다. (예: `50-xxx.yaml`을 먼저 읽고, `90-xxx.yaml`을 나중에 읽음). 뒤에 읽힌 파일의 설정이 앞의 설정을 덮어씁니다.

따라서, 기존 파일(`50`이나 `90`)을 건드리지 말고, **가장 숫자가 높은 새 파일을 만들어서 적용**하는 것이 가장 안전하고 깔끔한 방법입니다.

---

### 해결 방법: `99-custom-mtu.yaml` 생성

가장 마지막에 읽히도록 `99`로 시작하는 파일을 새로 만들겠습니다.

**1. 새 설정 파일 생성**
터미널에 다음 명령어를 입력하세요. (`eno1` 기준)

```bash
sudo nano /etc/netplan/99-custom-mtu.yaml

```

**2. 내용 작성**
아래 내용을 복사해서 붙여넣으세요.
*주의: `eno1` 부분은 실제 랜카드 이름에 맞춰야 합니다. (Jetson은 `eth0`일 수 있음)*

```yaml
network:
  version: 2
  ethernets:
    eno1:
      mtu: 9000
      # 주의: 기존에 DHCP를 쓰고 있었다면 true, 고정 IP라면 false 및 주소 설정 필요
      # 대부분의 공유기 환경에서는 true가 기본입니다.
      dhcp4: true 

```

**※ 중요 체크 (고정 IP 사용자라면?)**
만약 현재 IP를 수동으로 잡아서 쓰고 계시다면(`dhcp4: false`인 경우), 위 내용 대신 아래처럼 기존 IP 설정을 같이 적어줘야 안전합니다. (DHCP 사용자라면 위 코드로 충분합니다.)

```yaml
# 고정 IP 사용자용 예시 (필요시에만 사용)
network:
  version: 2
  ethernets:
    eno1:
      mtu: 9000
      dhcp4: false
      addresses:
        - 30.30.0.15/24  # 본인 IP
      routes:
        - to: default
          via: 30.30.0.1 # 게이트웨이
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]

```

**3. 저장 및 적용**

* 저장: `Ctrl + O`, `Enter`
* 종료: `Ctrl + X`
* 적용 명령어 실행:

```bash
sudo netplan apply

```

**4. 확인**
적용이 끝났으면 확인해 봅니다.

```bash
ip link show eno1 | grep mtu

```

`mtu 9000`이라고 나오면 성공적으로 영구 적용된 것입니다. 이제 재부팅 해도 9000으로 유지됩니다.
#########################  
###########################  
##############################  
네, 말씀해주신 설정(Network Settings GUI에서 입력한 내용)을 그대로 Netplan 파일로 옮기면 됩니다.

**게이트웨이(Gateway)나 DNS가 없다면 적지 않는 것이 맞습니다.** 특히 지금처럼 로봇과 PC가 1:1로 직결된 폐쇄망(Closed Network)에서는 오히려 게이트웨이를 적으면 인터넷 트래픽이 그쪽으로 길을 잘못 들어서 인터넷이 끊길 수도 있습니다.

서브넷 마스크가 `255.255.0.0`이라면 CIDR 표기법으로는 `/16`에 해당합니다.

따라서 `/etc/netplan/99-custom-mtu.yaml` 파일을 아래와 같이 작성하시면 됩니다.

### 수정된 Netplan 설정 파일 내용

* **인터페이스 이름 주의:** 아까 XML 설정에서는 `enp2s0`를 쓰셨고, 명령어 예시에서는 `eno1`이 있었습니다. 반드시 **`ip addr`로 확인된 실제 유선 랜카드 이름**으로 바꿔 넣으세요. (아래 예시는 `enp2s0`로 가정했습니다.)

```yaml
network:
  version: 2
  ethernets:
    enp2s0:              # <--- 여기에 실제 인터페이스 이름 입력 (예: eno1, eth0, enp2s0)
      mtu: 9000
      dhcp4: false
      addresses:
        - 30.30.0.15/16  # 30.30.0.15 IP에 255.255.0.0 서브넷 마스크를 의미

```

### 적용 방법

1. 파일 열기 (혹은 생성):
```bash
sudo nano /etc/netplan/99-custom-mtu.yaml

```


2. 위의 내용을 복사해서 붙여넣고, **인터페이스 이름**을 본인 환경에 맞게 수정합니다.
3. 저장 (`Ctrl + O`, `Enter`) 후 종료 (`Ctrl + X`).
4. 설정 적용:
```bash
sudo netplan apply

```



### 확인

적용 후 터미널에 아래 명령어를 입력해 보세요.

```bash
ip addr show enp2s0  # 본인 인터페이스 이름

```

* `mtu 9000` 이라고 떠야 합니다.
* `inet 30.30.0.15/16` 이라고 떠야 합니다.

이렇게 하면 재부팅 후에도 고정 IP와 MTU 9000 설정이 완벽하게 유지됩니다.
#####################  
#######################  
########################  
