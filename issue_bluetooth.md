오케이. 증상이 “apt update 이후에 블루투스 재연결이 잘 안 됨”이면 보통 아래 네 가지가 원인입니다.

1. BlueZ/커널 업데이트로 BT 스택 파라미터가 바뀌었거나(특히 ERTM)
2. BT USB 동글 펌웨어/드라이버(Realtek 계열) 로딩 이슈
3. 자동절전(autosuspend)이나 rfkill 같은 전원/블록 문제
4. (PS5=DualSense 기준) 페어링 캐시 꼬임 + 신뢰(trust) 미설정

아래 순서대로 점검/복구해 보세요. (PS5 듀얼센스 기준으로 예시를 듭니다. 다른 패드여도 대부분 동일)

---

# 0) 빠른 1분 리셋

```bash
# 블루투스 서비스 리셋
sudo systemctl restart bluetooth

# rfkill 확인 (blocked면 unblock)
rfkill list
sudo rfkill unblock bluetooth

# 어댑터 전원 토글
bluetoothctl power off
sleep 1
bluetoothctl power on
```

---

# 1) 어댑터/드라이버가 정상 인식되는지

```bash
lsusb | grep -i -e blue -e realtek -e intel -e csr
lsmod | grep -i bt
dmesg | grep -i -e bluetooth -e btusb -e firmware | tail -n 100
```

* `btusb` 모듈이 로드되고 “firmware loaded” 메시지가 보이면 정상.
* Realtek 동글(예: RTL8761B/RTL8852)이라면 `rtl_bt/..._fw.bin` 로딩 실패가 없는지 확인하세요. 실패 로그가 있으면 `linux-firmware`를 최신으로:

```bash
sudo apt update
sudo apt install --reinstall linux-firmware
sudo reboot
```

---

# 2) ERTM 비활성화 (게임패드 끊김/입력지연/재연결 실패시 효과 큼)

커널/BlueZ 업데이트로 ERTM이 다시 켜지면 듀얼센스/일부 패드가 문제를 일으키기도 합니다.

### 일회성(즉시 테스트)

```bash
echo 1 | sudo tee /sys/module/bluetooth/parameters/disable_ertm
```

### 영구 설정

`/etc/modprobe.d/bluetooth-ertm.conf` 생성:

```bash
echo "options bluetooth disable_ertm=Y" | sudo tee /etc/modprobe.d/bluetooth-ertm.conf
```

그리고 재부팅.

---

# 3) 페어링 캐시 깨끗이 비우고 재페어링 (가장 많이 해결됨)

업데이트 후에는 기존 캐시가 꼬여서 계속 “연결됨→바로 끊김”이 반복될 수 있습니다.

1. 기존 장치 제거

```bash
bluetoothctl
[bluetooth]# devices
[bluetooth]# remove <패드_MAC주소>
[bluetooth]# quit
```

2. (선택) 캐시 폴더 삭제
   어댑터 맥 주소가 `AA_BB_CC_DD_EE_FF` 라면:

```bash
sudo systemctl stop bluetooth
sudo rm -rf /var/lib/bluetooth/AA:BB:CC:DD:EE:FF/*  # 주의: 다른 페어링도 지워짐
sudo systemctl start bluetooth
```

3. 듀얼센스 페어링 진입(PS + Create 버튼 몇 초) → 다시 스캔/페어:

```bash
bluetoothctl
[bluetooth]# power on
[bluetooth]# agent on
[bluetooth]# default-agent
[bluetooth]# pairable on
[bluetooth]# discoverable on
[bluetooth]# scan on
# "Wireless Controller" 또는 "DualSense" 발견되면
[bluetooth]# pair <MAC>
[bluetooth]# trust <MAC>
[bluetooth]# connect <MAC>
```

* `trust`를 꼭 해 주세요. 업데이트 후 자동 재연결이 안 되는 이유 1순위입니다.

---

# 4) USB 자동절전(autosuspend) 해제 (연결됐다 끊김 반복 시)

특히 노트북/저전력 정책에서 BT 동글이 절전으로 떨어지면 기기가 튕깁니다.

1. 동글의 USB ID 확인:

```bash
lsusb
```

예: `0bda:8771` (Realtek 예시)

2. Udev 규칙으로 autosuspend off
   `/etc/udev/rules.d/99-bt-autosuspend.rules`:

```bash
# 예: Realtek 0bda:8771
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="0bda", ATTR{idProduct}=="8771", TEST=="power/control", RUN+="/bin/sh -c 'echo on > /sys%p/power/control'"
```

적용:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

(또는 TLP/PowerTop를 사용 중이면 해당 장치의 USB autosuspend를 “Off”로)

---

# 5) 듀얼센스 전용 체크 (hid-playstation)

Ubuntu 24.04(커널 6.8)에는 기본 포함이지만, 간혹 모듈이 빠져있거나 블랙리스트 된 경우:

```bash
lsmod | grep -i playstation
# 없으면 수동 로드
sudo modprobe hid_playstation
```

그리고 `evtest`로 입력 이벤트가 오는지 확인:

```bash
sudo apt install evtest
cat /proc/bus/input/devices | awk 'BEGIN{RS="";FS="\n"} /Wireless Controller|DualSense/{print}'
# 이벤트 노드 확인 후
sudo evtest /dev/input/eventX
```

입력이 보이면 커널/HID는 정상입니다.

---

# 6) Nav2/ROS2 쪽 입·출력 확인 (블루투스는 OK인데 /cmd_vel이 안 나갈 때)

1. `joy_linux` 노드가 패드를 잡는지:

```bash
ros2 run joy joy_node
# 또는 패키지에 따라 joy_linux 노드
ros2 topic echo /joy
```

버튼/스틱 값이 들어오면 map/teleop 변환 노드 확인.

2. 텔레옵 파이프라인 점검(예: teleop_twist_joy → /cmd_vel):

* 패러미터 파일에서 버튼-축 매핑이 업데이트 후 초기화되지 않았는지
* `use_sim_time`, QoS, 네임스페이스 충돌 확인
* `/cmd_vel`가 실제 로봇이 구독하는 토픽과 일치하는지(네임스페이스 포함)

3. 입력 디바이스 권한:

```bash
groups $USER | grep input || sudo usermod -aG input $USER
# 적용 위해 재로그인/재부팅
```

---

# 7) BlueZ/커널 버전 확인 및 롤백/고정(필요 시)

문제가 특정 업데이트(예: bluez X.Y.Z) 이후라면 일시적으로 이전 버전으로 고정하는 방법도 있습니다.

```bash
apt-cache policy bluez
uname -r
```

* 문제 재현되는 버전 메모 → Launchpad 버그/변경점 확인 후
* 임시로 이전 bluez 버전에 고정(`apt-mark hold bluez`) 하거나 HWE 커널/일반 커널 전환을 검토

---

# 8) 로그 수집(그래도 안 되면)

다시 재현하면서 로그를 모아 주세요. 보내주시면 원인 좁혀볼게요.

```bash
# 블루투스 스택 패킷 로그
sudo btmon | tee btmon.log

# 시스템 저널
journalctl -u bluetooth -b --no-pager | tee bluetooth.service.log

# 커널 메시지
dmesg | grep -i -e btusb -e bluetooth | tee dmesg.bt.log
```

---

## 요약 추천 경로

1. 서비스/전원 리셋 → 2) ERTM 비활성화 → 3) 페어링 캐시 삭제 후 `pair/trust/connect` 재실행 → 4) USB autosuspend 해제 → 5) `hid_playstation`/`evtest`로 입력 확인 → 6) ROS2 `joy`→`/cmd_vel` 파이프라인 점검.
   대부분 2)+3)로 해결됩니다.

원하시면 btmon/로그 출력 일부를 붙여 주세요. 듀얼센스가 아닌 다른 모델이면 모델명(또는 `lsusb`, `bluetoothctl info`) 기준으로 드라이버/옵션(예: Xbox=xone/xpadneo, 8BitDo=ERTM/호환 모드)을 맞춰 드릴게요.
