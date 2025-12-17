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
