Seri, 증상은 “센서/케이블은 정상인데 Ubuntu PC2 쪽에서만 데이터가 늦게 도착(시간 지연·버퍼링)”입니다. 보통 아래 5가지 계층에서 생깁니다: (1) USB–Serial 드라이버의 지연 버퍼, (2) USB 전원관리(auto-suspend), (3) CPU 전력관리/IRQ 스케줄링, (4) ROS 2 QoS/큐 백로그, (5) 잘못된 시계 설정(use_sim_time 등).
바로 효과 보는 순서대로 점검·해결 방법을 드릴게요.


---

1) 가장 빠른 5분 점검 (즉시 효과)

A. 물리 포트 바꾸기 & 허브 우회

PC2에서 허브/전면 포트 말고 메인보드 후면 USB 2.0 포트에 직접 꽂아보세요.

꽂자마자 커널 로그로 칩셋을 확인:

dmesg -w | grep -i -E 'ttyUSB|ttyACM|usb serial'
lsusb -t


B. USB 전원관리(autosuspend) 끄기 (임시)

장치 경로 확인:

DEV=/dev/ttyUSB0     # 실제 포트로 수정
readlink -f /sys/class/tty/$(basename $DEV) | sed 's:/tty/ttyUSB[0-9]\+::'

위가 /sys/bus/usb/devices/1-3 같은 경로로 나오면:

sudo sh -c 'echo on > /sys/bus/usb/devices/1-3/power/control'
sudo sh -c 'echo -1 > /sys/bus/usb/devices/1-3/power/autosuspend_delay_ms'

이 상태로 등속 회전→정지 테스트를 다시 해보세요. 지연이 사라지면 원인은 전원관리입니다(아래 §4에서 영구 설정).


C. FTDI 계열이면 “latency_timer=1ms”

칩셋이 FTDI(FT232 등)인 경우에만 존재합니다:

# 존재하면 16 → 1 로 낮추기
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer 2>/dev/null
sudo sh -c 'echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer' 2>/dev/null || true

효과: 드라이버 수신 지연을 크게 줄여서 “멈췄는데 한참 뒤 0” 같은 현상이 개선됩니다.
(CH340/CP210x는 이 파일이 없을 수 있어요. 그런 경우 §B·§D가 더 중요합니다.)


D. CPU 전력관리 해제(성능 모드)

노트북/저전력 프로필이면 폴링·콜백이 뒤로 밀릴 수 있어요:

# GNOME/Ubuntu 24.04+ 권장
powerprofilesctl set performance 2>/dev/null || true

# 대안(일시 적용)
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor >/dev/null

(선택) irqbalance가 이상하게 코어를 돌리면 끄고 비교:

sudo systemctl stop irqbalance


E. ROS 2에서 “진짜 지연”인지 “큐 적체”인지 확인

주파수 확인:

ros2 topic hz /imu/data

기대치(예: 100 Hz)가 PC2에서만 낮거나 요동하면 OS/드라이버/전력관리 이슈일 확률↑.

QoS가 RELIABLE이고 subscriber가 느리면 메시지가 줄 서서 늦게 보이는 일이 생깁니다. 퍼블리셔/구독자 모두 가능하면:

reliability: BEST_EFFORT

history: KEEP_LAST, depth: 5~10 로 바꿔 보세요(아래 §3 참고).




---

2) 시계/타임스탬프 혼선 제거 (의외로 흔함)

실센서인데 use_sim_time가 켜져 있으면 time-base가 꼬여 보일 수 있어요:

ros2 param get /witmotion_imu use_sim_time
# true면
ros2 param set /witmotion_imu use_sim_time false

메시지에 header.stamp가 있다면, 수신 시각(now)와의 차이를 확인해 “전달 지연”인지 “타임스탬프 문제”인지 구분하세요.



---

3) ROS 2 QoS 권장값 (적체로 인한 ‘늦게 0됨’ 방지)

witmotion 노드가 QoS 파라미터를 노출한다면 이런 느낌으로:

# params.yaml (예시)
witmotion_imu:
  ros__parameters:
    qos_reliability: best_effort     # 또는 reliability: "best_effort"
    qos_history: keep_last
    qos_depth: 10
    qos_durability: volatile

실행 시:

ros2 run <pkg> <node> --ros-args --params-file params.yaml

RELIABLE→패킷 유실은 줄지만, 느린 구독자 때문에 버퍼가 쌓여 “과거 데이터”를 나중에 내보내는 일이 생깁니다. IMU는 보통 BEST_EFFORT가 더 자연스럽습니다.



---

4) 설정을 “영구”로 만들기

A. USB autosuspend 영구 해제(해당 장치만)

장치 VID:PID 확인:

lsusb | grep -i -E 'ch340|cp21|ftdi|silabs|prolific|usb-serial'

udev 규칙(예: FTDI 0403:6001라 가정):

printf '%s\n' \
'ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6001", ATTR{power/control}="on", ATTR{power/autosuspend}="-1"' \
| sudo tee /etc/udev/rules.d/99-imu-usb-power.rules
sudo udevadm control --reload && sudo udevadm trigger


B. FTDI만: latency_timer=1ms 영구 적용

printf '%s\n' \
'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTR{latency_timer}="1"' \
| sudo tee /etc/udev/rules.d/99-ftdi-lowlatency.rules
sudo udevadm control --reload && sudo udevadm trigger

> CH340/CP210x는 latency_timer가 없으니 §A·전력/허브/포트 변경이 핵심입니다.
가능하면 FTDI 기반 케이블로 교체하는 게 가장 쉬운 상수화 해결책인 경우가 많습니다.




---

5) 드라이버·시스템 레벨 추가 체크

보드/드라이버 확인

udevadm info -q property -n /dev/ttyUSB0 | grep -E 'ID_MODEL|ID_VENDOR|ID_USB_DRIVER'
modinfo ftdi_sio 2>/dev/null | head
modinfo ch341 2>/dev/null | head
modinfo cp210x 2>/dev/null | head

커널 메시지 오류 감시(재연 중에 열어두기)

dmesg -w | egrep -i 'usb|tty|ftdi|cp210|ch34|reset|overflow|error'

직렬 라인 옵션(흐름제어/캐노니컬 모드 금지 확인)
대개 노드가 RAW로 열지만 환경에 따라 간섭될 수 있어요.

stty -F /dev/ttyUSB0 -a
# ixon/ixoff/crtscts/icanon/echo 등이 꺼져있는지 확인

CPU/GPU 부하

top -d1

IMU 읽기/파싱 스레드가 밀릴 정도 부하가 있으면 우선 해소(프로파일러/성능 모드).



---

6) “지연인지 필터링인지” 빠른 구분법

Windows/Ubuntu PC1에서는 즉시 0으로 꺼짐 → 센서 내부 DMP/오토보정 문제 가능성 ↓

Ubuntu PC2만 늦게 0 → 드라이버/전원관리/큐 적체 같은 호스트 측 지연 가능성 ↑

위 §1 B·C·D를 적용해서 즉시 반응성이 살아나면 거의 확실히 호스트 측 원인이었습니다.



---

7) 요약·권장 순서

1. 후면 USB2.0 직접 연결 → 2) autosuspend 끄기(on, delay=-1) → 3) (FTDI면) latency_timer=1ms → 4) 성능 모드 & irqbalance 일시 중지 → 5) ROS 2 QoS를 BEST_EFFORT/작은 depth → 6) 허브/케이블 교체 또는 FTDI 케이블 전환.



위 단계만으로도 “등속 멈춤 후 값이 한참 뒤 0” 문제는 보통 해결됩니다. 실행해보시고, lsusb -t, dmesg -w 일부와 ros2 topic hz /imu/data 결과가 어떻게 바뀌는지 알려주시면(특히 PC1 vs PC2 비교) 다음 최적화 포인트까지 바로 잡아드릴게요.
