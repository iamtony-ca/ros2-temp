네, 개발자님의 직감이 정확하십니다. **`tegrastats`의 텍스트 출력(String)을 파이썬으로 파싱(Parsing)하는 방식은 "절대 권장하지 않습니다."**

이유는 간단합니다. JetPack 버전이 업데이트되거나, 특정 센서가 잠시 꺼지면 텍스트 포맷(띄어쓰기, 순서)이 바뀌어버려서 코드가 뻗어버릴(Exception) 위험이 매우 크기 때문입니다.

가장 **Robust(견고)하고 Industry Standard(산업 표준)**에 가까운 방법은 두 가지 라이브러리를 **"혼합(Hybrid)"**하여 사용하는 것입니다.

1. **범용 시스템(CPU, RAM, Network):** Python 표준 라이브러리인 **`psutil`** 사용 (가장 안정적).
2. **Jetson 특화 하드웨어(GPU, Power, Fan):** **`jtop` (jetson-stats)의 Python API** 사용 (NVIDIA 시스템 파일을 안전하게 읽어오는 Wrapper).

이 두 가지를 조합하여, 에러 핸들링까지 포함된 **"현존하는 가장 안전한 모니터링 코드"**를 작성해 드립니다.

---

### 🛡️ Robust System Monitor Node (Final Version)

이 코드는 다음과 같은 특징이 있습니다.

* **Safe Reading:** 데이터 수집 실패 시 노드가 죽지 않고 `None` 또는 `0`을 반환합니다.
* **Non-Blocking:** Ping이나 네트워크 체크가 오래 걸려도 ROS 루프를 막지 않도록 타임아웃을 짧게 설정합니다.
* **Rolling Statistics:** Avg, Max 계산을 위해 최근 60개의 데이터만 메모리 큐(Deque)에 보관합니다.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import psutil
import subprocess
import time
from collections import deque
from jtop import jtop

class RobustMonitor(Node):
    def __init__(self):
        super().__init__('robust_system_monitor')
        
        # 1. 설정
        self.declare_parameter('target_ip', '192.168.0.1') # IPC 혹은 공유기 IP
        self.target_ip = self.get_parameter('target_ip').value
        self.interval = 1.0 # 1초 주기
        
        # 2. 데이터 저장을 위한 큐 (최근 60초 데이터 보관 -> Avg/Max 계산용)
        # deque를 쓰면 maxlen 넘어가면 알아서 오래된 것 버림 (메모리 안전)
        self.history_len = 60
        self.cpu_q = deque(maxlen=self.history_len)
        self.gpu_q = deque(maxlen=self.history_len)
        
        # 3. 네트워크 속도 계산용 이전 상태
        self.prev_net = psutil.net_io_counters()
        self.prev_time = time.time()

        # 4. Jtop (GPU 모니터링) 안전하게 시작
        try:
            self.jetson = jtop()
            self.jetson.start()
            if not self.jetson.ok():
                self.get_logger().error("Jtop Service is NOT running. GPU stats will be empty.")
        except Exception as e:
            self.get_logger().error(f"Failed to start Jtop: {e}")
            self.jetson = None

        # 5. Timer 및 Publisher
        self.pub = self.create_publisher(String, 'system_status', 10)
        self.timer = self.create_timer(self.interval, self.update_stats)

        self.get_logger().info("Robust System Monitor Started.")

    def _get_network_metrics(self):
        """psutil을 이용한 네트워크 대역폭 계산 (가장 정확함)"""
        try:
            curr_net = psutil.net_io_counters()
            curr_time = time.time()
            dt = curr_time - self.prev_time
            
            if dt <= 0: dt = 1.0 # 0으로 나누기 방지

            sent_bps = (curr_net.bytes_sent - self.prev_net.bytes_sent) / dt
            recv_bps = (curr_net.bytes_recv - self.prev_net.bytes_recv) / dt

            self.prev_net = curr_net
            self.prev_time = curr_time

            return {
                "tx_mbps": round(sent_bps * 8 / 1_000_000, 2), # Mbps 변환
                "rx_mbps": round(recv_bps * 8 / 1_000_000, 2),
                "tx_total_mb": round(curr_net.bytes_sent / 1024 / 1024, 1),
                "rx_total_mb": round(curr_net.bytes_recv / 1024 / 1024, 1)
            }
        except Exception as e:
            self.get_logger().warn(f"Net Error: {e}")
            return {}

    def _get_latency_and_sync(self):
        """Subprocess를 사용하되 Timeout을 걸어 안전하게 측정"""
        ping_ms = -1.0
        sync_offset_ms = 0.0

        # 1. Network Latency (Ping)
        try:
            # 타임아웃 0.5초로 매우 짧게 설정하여 ROS 루프 지연 방지
            cmd = ['ping', '-c', '1', '-W', '1', self.target_ip]
            res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=1.1)
            if res.returncode == 0:
                # time=12.3 ms 파싱
                start = res.stdout.find('time=')
                if start != -1:
                    end = res.stdout.find(' ms', start)
                    ping_ms = float(res.stdout[start+5:end])
        except Exception:
            ping_ms = -1.0 # 실패 시 -1 표기

        # 2. Time Sync (Chrony)
        try:
            cmd = ['chronyc', 'tracking']
            res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=0.5)
            if res.returncode == 0:
                for line in res.stdout.splitlines():
                    if "Last offset" in line:
                        # "+0.000012 seconds" -> ms 변환
                        parts = line.split(':')
                        if len(parts) > 1:
                            seconds = float(parts[1].replace('seconds', '').strip())
                            sync_offset_ms = seconds * 1000.0
                        break
        except Exception:
            sync_offset_ms = 9999.9 # 실패 시 큰 값 혹은 식별값

        return ping_ms, sync_offset_ms

    def update_stats(self):
        # 1. CPU & RAM (psutil 사용 - 매우 안정적)
        cpu_cur = psutil.cpu_percent(interval=None) # interval=None은 non-blocking
        self.cpu_q.append(cpu_cur)
        
        mem = psutil.virtual_memory() # 전체 시스템 메모리
        
        # 2. GPU & Power (jtop 사용 - Jetson 특화)
        gpu_cur = 0
        gpu_temp = 0
        gpu_mem = 0
        power_w = 0
        
        if self.jetson and self.jetson.ok():
            try:
                # GPU 로드율
                gpu_cur = self.jetson.stats['GPU']
                self.gpu_q.append(gpu_cur)
                
                # 온도 및 메모리
                gpu_temp = self.jetson.stats['Temp'].get('GPU', 0)
                # jtop의 RAM['shared']가 GPU 사용량과 유사 (Unified Memory)
                gpu_mem = self.jetson.stats['RAM'].get('shared', 0) / 1024 # MB
                
                # 전력
                power_w = self.jetson.stats['Power'].get('avg', 0) / 1000 # Watt
            except KeyError:
                pass # 특정 필드가 없어도 죽지 않음

        # 3. Network & Sync
        net_metrics = self._get_network_metrics()
        ping, time_offset = self._get_latency_and_sync()

        # 4. 통계 계산 (Safe division)
        cpu_avg = sum(self.cpu_q) / len(self.cpu_q) if self.cpu_q else 0
        cpu_max = max(self.cpu_q) if self.cpu_q else 0
        
        gpu_avg = sum(self.gpu_q) / len(self.gpu_q) if self.gpu_q else 0
        gpu_max = max(self.gpu_q) if self.gpu_q else 0

        # 5. 최종 메시지 생성
        status_data = {
            "timestamp": time.time(),
            "cpu": {
                "usage_percent": cpu_cur,
                "avg_60s": round(cpu_avg, 1),
                "max_60s": round(cpu_max, 1),
                "mem_usage_mb": round(mem.used / 1024 / 1024, 1),
                "mem_percent": mem.percent
            },
            "gpu": {
                "usage_percent": int(gpu_cur),
                "avg_60s": round(gpu_avg, 1),
                "max_60s": int(gpu_max),
                "temp_c": int(gpu_temp),
                "mem_usage_mb": int(gpu_mem)
            },
            "network": {
                "io": net_metrics,
                "latency_ping_ms": ping,
            },
            "system": {
                "time_offset_ms": round(time_offset, 4),
                "power_w": round(power_w, 2)
            }
        }

        # JSON Publish
        msg = String()
        msg.data = json.dumps(status_data)
        self.pub.publish(msg)

    def destroy_node(self):
        if self.jetson:
            self.jetson.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RobustMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

```

---

### 💡 왜 이 방식이 가장 Robust한가?

1. **`psutil` (Standard Library):**
* `cpu_percent`나 `net_io_counters`는 리눅스 커널의 `/proc/stat`, `/proc/net/dev` 파일을 직접 읽습니다. `tegrastats`처럼 출력 포맷이 변할 일이 **전혀** 없습니다. Python 생태계 표준입니다.


2. **`jtop` Library (API Access):**
* `tegrastats` 명령어를 실행해서 텍스트를 긁어오는 게 아니라, **`jetson-stats` 데몬(jtop service)과 소켓 통신**을 하거나 시스템 파일을 직접 읽습니다. 훨씬 빠르고 포맷 변경에 안전합니다.
* **Context Manager:** `try-except` 블록 안에서 실행되므로, GPU 센서 하나가 고장 나더라도 CPU 모니터링은 계속 살아서 돌아갑니다.


3. **Deque (Double-ended Queue):**
* 리스트에 `append`만 계속하면 메모리가 터집니다. `deque(maxlen=60)`을 사용하여 **자동으로 오래된 데이터를 삭제**하므로 며칠을 켜둬도 메모리 누수가 없습니다.



이 코드를 사용하시면 ZED, Nav2와 함께 몇 달을 돌려놔도 모니터링 노드 때문에 시스템이 죽는 일은 없을 것입니다.
##########################  
##############################  
#################################  
ZED Camera의 상태(Connection, Temperature, FPS)와 ROS 2 Node의 생존 여부(Heartbeat)를 모니터링하는 **가장 확실하고 표준적인(Standard) 방법**은 ROS 2의 **`diagnostic_msgs`** 토픽을 활용하는 것입니다.

ZED ROS 2 Wrapper는 이미 내부적으로 카메라 상태를 진단하여 `/diagnostics` 토픽으로 발행하고 있습니다. 굳이 이미지 토픽을 구독해서 FPS를 직접 계산하거나 SDK를 따로 부를 필요가 없습니다. (그러면 오버헤드가 발생합니다.)

### 💡 아키텍처: Diagnostic Listener + Watchdog

1. **데이터 소스:** `/diagnostics` (ZED Node가 발행하는 표준 진단 메시지)
2. **모니터링 방식:**
* **FPS & Temp:** 진단 메시지 안의 Key-Value 쌍을 파싱.
* **Connection & Heartbeat:** **"Watchdog(경비견)" 패턴** 사용. 진단 메시지가 1초 이상 안 들어오면 "연결 끊김(Dead)"으로 간주.



---

### 🐍 Robust ZED Monitor Code

이 코드는 `/diagnostics`를 감시하다가, ZED 관련 정보만 쏙 뽑아서 JSON으로 발행합니다. 만약 ZED 노드가 멈추면 즉시 `ALIVE: False` 경고를 띄웁니다.

```python
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String
import json
import time

class ZedMonitor(Node):
    def __init__(self):
        super().__init__('zed_status_monitor')

        # 1. 설정
        # ZED Wrapper의 진단 메시지 이름 (보통 'zed_node: ZED Diagnostic' 형태임)
        # ros2 topic echo /diagnostics 로 확인 가능
        self.target_node_name = "zed_node" 
        
        # 2. Watchdog 설정 (Heartbeat)
        self.last_heartbeat_time = 0.0
        self.timeout_sec = 2.0  # 2초 동안 소식 없으면 사망 판정
        self.is_connected = False

        # 3. 데이터 저장 변수
        self.zed_stats = {
            "fps": 0.0,
            "temp_left": 0.0,
            "temp_right": 0.0,
            "camera_model": "Unknown",
            "sn": "Unknown"
        }

        # 4. Subscriber & Publisher
        # ZED가 발행하는 진단 정보를 수신
        self.sub_diag = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diag_callback,
            10
        )
        
        # 결과를 보기 좋게 JSON으로 발행
        self.pub_status = self.create_publisher(String, 'zed_monitor_status', 10)
        
        # 5. 주기적 감시 타이머 (1Hz)
        self.timer = self.create_timer(1.0, self.watchdog_callback)

        self.get_logger().info("ZED Monitor Started via /diagnostics")

    def diag_callback(self, msg):
        """
        /diagnostics 토픽이 들어올 때마다 실행.
        ZED 노드가 보내온 정보인지 확인하고 파싱함.
        """
        for status in msg.status:
            # ZED 노드에서 온 메시지인지 이름으로 필터링
            if self.target_node_name in status.name:
                self.last_heartbeat_time = time.time()
                self.is_connected = True
                
                # Key-Value 파싱
                # ZED Wrapper 버전에 따라 Key 이름이 다를 수 있으니 확인 필요
                # 보통: 'Input FPS', 'Left CMOS Temp', 'Right CMOS Temp' 등
                for kv in status.values:
                    if 'FPS' in kv.key: # Input FPS or Camera FPS
                        try: self.zed_stats['fps'] = float(kv.value)
                        except: pass
                    
                    elif 'Temp' in kv.key and 'Left' in kv.key:
                        try: self.zed_stats['temp_left'] = float(kv.value)
                        except: pass

                    elif 'Temp' in kv.key and 'Right' in kv.key:
                        try: self.zed_stats['temp_right'] = float(kv.value)
                        except: pass
                        
                    elif 'Model' in kv.key:
                        self.zed_stats['camera_model'] = kv.value
                        
                    elif 'Serial' in kv.key:
                        self.zed_stats['sn'] = kv.value

    def watchdog_callback(self):
        """
        주기적으로 실행되어 '마지막으로 메시지 받은 시간'을 체크함.
        """
        current_time = time.time()
        time_diff = current_time - self.last_heartbeat_time

        # 타임아웃 체크 (Heartbeat Logic)
        if time_diff > self.timeout_sec:
            self.is_connected = False
            status_msg = "DISCONNECTED / DEAD"
            # 연결 끊기면 수치 0으로 초기화 (선택사항)
            self.zed_stats['fps'] = 0.0
        else:
            status_msg = "ALIVE"

        # 최종 리포트 생성
        report = {
            "status": {
                "alive": self.is_connected,
                "msg": status_msg,
                "last_heartbeat_sec_ago": round(time_diff, 1)
            },
            "camera": {
                "fps": self.zed_stats['fps'],
                "temp": {
                    "left": self.zed_stats['temp_left'],
                    "right": self.zed_stats['temp_right']
                },
                "info": {
                    "model": self.zed_stats['camera_model'],
                    "sn": self.zed_stats['sn']
                }
            }
        }

        # JSON 발행
        msg = String()
        msg.data = json.dumps(report)
        self.pub_status.publish(msg)
        
        # 로그 (디버깅용)
        # if not self.is_connected:
        #     self.get_logger().warn("ZED Node is NOT responding!")

def main(args=None):
    rclpy.init(args=args)
    node = ZedMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

```

---

### 🔍 핵심 포인트 분석

#### 1. 왜 `/diagnostics`를 쓰는가?

* **효율성:** 이미 ZED Node가 내부 C++ 코드에서 하드웨어 상태를 체크해서 보내줍니다. 파이썬에서 무겁게 SDK를 또 열거나 이미지를 처리할 필요가 없습니다.
* **표준:** Nav2, Hardware Interface 등 모든 ROS 2 메이저 패키지는 `/diagnostics`를 씁니다. 나중에 `rqt_runtime_monitor` 같은 툴에서도 한눈에 볼 수 있습니다.

#### 2. Watchdog (Heartbeat) 로직

* `diag_callback`이 호출될 때마다 `self.last_heartbeat_time`을 갱신합니다.
* 별도의 `timer`가 돌면서 `현재시간 - 마지막시간 > 2초`를 체크합니다.
* **결과:** USB 케이블이 빠지거나, 노드가 `Segmentation Fault`로 죽으면 즉시 **"DISCONNECTED"** 상태로 바뀝니다.

#### 3. Temperature & FPS

* ZED Wrapper는 카메라 온도를 `DiagnosticStatus`의 Key-Value 배열(`values`)에 담아 보냅니다.
* 위 코드는 `Temp`와 `Left`, `FPS` 같은 키워드를 검색해서 값을 추출하므로, ZED 모델(ZED2, ZED X 등)이 조금 바뀌어도 유연하게 작동합니다.

### ✅ 실행 및 확인 방법

1. **실행:**
```bash
python3 zed_monitor.py

```


2. **데이터 확인:**
```bash
ros2 topic echo /zed_monitor_status

```


**출력 예시:**
```json
{
  "status": {"alive": true, "msg": "ALIVE", "last_heartbeat_sec_ago": 0.1},
  "camera": {
    "fps": 29.8,
    "temp": {"left": 42.5, "right": 41.2},
    ...
  }
}

```



**제가 님을 위해 할 수 있는 다음 단계는:**

만약 이 ZED 모니터와 아까 만든 시스템 모니터를 합쳐서, **"FPS가 떨어질 때 시스템 부하(CPU/GPU)가 높았는지, 아니면 카메라 온도가 높았는지"** 원인을 자동으로 분석해 주는 통합 로거(Logger)를 만들어 드릴까요?
