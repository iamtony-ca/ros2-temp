네트워크 설정에 대해 각 항목별로 자세히 설명드리겠습니다.

## 1. 고정 IP 할당

### 왜 필요한가?
- DHCP를 사용하면 IP가 변경될 수 있어 ROS2 통신이 끊김
- 고정 IP로 안정적인 peer-to-peer 연결 보장
- Discovery 시간 단축

### 설정 방법

#### Ubuntu/Debian (Netplan 사용)
```bash
# 네트워크 인터페이스 확인
ip addr show
# 또는
ifconfig

# Netplan 설정 파일 편집
sudo nano /etc/netplan/01-netcfg.yaml
```

**Device1 설정**:
```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:  # 또는 enp0s3 등 실제 인터페이스 이름
      dhcp4: no
      addresses:
        - 192.168.1.10/24
      # 라우터 연결이 필요한 경우만
      # gateway4: 192.168.1.1
      # nameservers:
      #   addresses: [8.8.8.8, 8.8.4.4]
```

**Device2 설정**:
```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 192.168.1.20/24
```

```bash
# 설정 적용
sudo netplan apply

# 연결 확인
ping 192.168.1.20  # Device1에서
ping 192.168.1.10  # Device2에서
```

#### 직접 IP 설정 (임시)
```bash
# Device1
sudo ip addr add 192.168.1.10/24 dev eth0
sudo ip link set eth0 up

# Device2
sudo ip addr add 192.168.1.20/24 dev eth0
sudo ip link set eth0 up
```

---

## 2. MTU 크기 최적화 (Jumbo Frame)

### MTU란?
**Maximum Transmission Unit**: 한 번에 전송할 수 있는 최대 패킷 크기

### 기본값과 최적값
- **표준 Ethernet MTU**: 1500 bytes
- **Jumbo Frame**: 9000 bytes

### 장점
- **패킷 수 감소**: 큰 데이터를 적은 패킷으로 전송
- **오버헤드 감소**: 헤더 처리 횟수 감소
- **CPU 사용률 감소**: 패킷 처리 부담 감소
- **Latency 감소**: 특히 큰 메시지(이미지, 포인트클라우드 등)에 효과적

### 설정 방법

```bash
# 현재 MTU 확인
ip link show eth0

# MTU 변경 (임시)
sudo ip link set eth0 mtu 9000

# 적용 확인
ip link show eth0 | grep mtu
```

### 영구 설정 (Netplan)
```yaml
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 192.168.1.10/24
      mtu: 9000  # 추가
```

### ⚠️ 주의사항
- **양쪽 디바이스 모두** MTU를 9000으로 설정해야 함
- 중간에 스위치가 있다면 스위치도 Jumbo Frame 지원 필요
- 지원하지 않으면 패킷 단편화(fragmentation) 발생 → 오히려 성능 저하

### MTU 테스트
```bash
# 9000 바이트 패킷 테스트 (성공해야 함)
ping -M do -s 8972 192.168.1.20
# 8972 = 9000 - 28 (IP+ICMP 헤더)

# 실패하면 MTU 값 낮추기 (8000, 7000 등)
```

---

## 3. TCP/UDP 버퍼 크기 증가

### 버퍼의 역할
- **송신 버퍼(wmem)**: 데이터를 네트워크로 보내기 전 임시 저장
- **수신 버퍼(rmem)**: 네트워크에서 받은 데이터 임시 저장
- 버퍼가 작으면 → **패킷 드롭** 발생 → 재전송 → latency 증가

### 각 파라미터 설명

```bash
# 최대 수신 버퍼 크기: 128MB
sudo sysctl -w net.core.rmem_max=134217728

# 최대 송신 버퍼 크기: 128MB
sudo sysctl -w net.core.wmem_max=134217728

# 기본 수신 버퍼 크기: 16MB
sudo sysctl -w net.core.rmem_default=16777216

# 기본 송신 버퍼 크기: 16MB
sudo sysctl -w net.core.wmem_default=16777216
```

### 왜 이렇게 큰 값?
- ROS2는 DDS를 사용하여 대량의 데이터 전송
- 센서 데이터(카메라, LiDAR)는 매우 큼
- 버스트 트래픽 처리 능력 향상

### 추가 최적화 파라미터

```bash
# UDP 수신 버퍼 증가 (ROS2는 주로 UDP 사용)
sudo sysctl -w net.core.netdev_max_backlog=5000

# TCP 버퍼 자동 조정 (필요시)
sudo sysctl -w net.ipv4.tcp_rmem="4096 87380 134217728"
sudo sysctl -w net.ipv4.tcp_wmem="4096 65536 134217728"

# 소켓 버퍼
sudo sysctl -w net.core.optmem_max=40960
```

### 영구 설정
```bash
# /etc/sysctl.conf 파일에 추가
sudo nano /etc/sysctl.conf
```

```conf
# ROS2 네트워크 최적화
net.core.rmem_max=134217728
net.core.wmem_max=134217728
net.core.rmem_default=16777216
net.core.wmem_default=16777216
net.core.netdev_max_backlog=5000
net.core.optmem_max=40960

# UDP 버퍼 최적화
net.ipv4.udp_rmem_min=16384
net.ipv4.udp_wmem_min=16384
```

```bash
# 재부팅 없이 적용
sudo sysctl -p

# 확인
sysctl net.core.rmem_max
sysctl net.core.wmem_max
```

---

## 4. 전체 최적화 스크립트

```bash
#!/bin/bash
# ros2_network_optimize.sh

INTERFACE="eth0"  # 실제 인터페이스 이름으로 변경
LOCAL_IP="192.168.1.10"  # Device1 또는 Device2에 맞게 변경

echo "ROS2 네트워크 최적화 시작..."

# IP 설정 (Netplan 사용 권장)
# sudo ip addr add ${LOCAL_IP}/24 dev ${INTERFACE}
# sudo ip link set ${INTERFACE} up

# MTU 설정
echo "MTU를 9000으로 설정..."
sudo ip link set ${INTERFACE} mtu 9000

# 버퍼 크기 설정
echo "네트워크 버퍼 크기 증가..."
sudo sysctl -w net.core.rmem_max=134217728
sudo sysctl -w net.core.wmem_max=134217728
sudo sysctl -w net.core.rmem_default=16777216
sudo sysctl -w net.core.wmem_default=16777216
sudo sysctl -w net.core.netdev_max_backlog=5000
sudo sysctl -w net.core.optmem_max=40960

echo "설정 완료!"
echo "MTU: $(ip link show ${INTERFACE} | grep mtu)"
echo "rmem_max: $(sysctl net.core.rmem_max)"
echo "wmem_max: $(sysctl net.core.wmem_max)"
```

---

## 5. 성능 확인 방법

### iperf3로 대역폭 테스트
```bash
# Device1 (서버)
sudo apt install iperf3
iperf3 -s

# Device2 (클라이언트)
iperf3 -c 192.168.1.10 -t 30
```

### ROS2 성능 측정
```bash
# Publisher 실행 (Device1)
ros2 run demo_nodes_cpp talker

# Subscriber + 성능 측정 (Device2)
ros2 topic hz /chatter
ros2 topic bw /chatter
ros2 topic delay /chatter  # 만약 타임스탬프가 있다면
```

이렇게 설정하면 유선 LAN 직접 연결에서 최적의 latency를 얻을 수 있습니다!
