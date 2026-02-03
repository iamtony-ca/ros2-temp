네, **Watchdog(감시자) 패턴**을 완벽하게 적용하여 ROS 2 노드로 변환한 코드를 작성해 드립니다.

이 코드는 `rclpy`의 **Timer**와 **State Machine(상태 머신)**을 사용하여, Python의 `while` 루프나 `time.sleep()` 없이도 비동기적으로(Asynchronous) 프로세스를 관리합니다. 이것이 진정한 ROS 2 스타일의 프로그래밍입니다.

### 설계: Finite State Machine (FSM)

노드는 다음과 같은 5가지 상태를 순환하며 동작합니다.

1. **`IDLE`**: 초기 상태.
2. **`LAUNCHING`**: `subprocess`로 Launch 파일을 실행하고, 카메라 노드가 켜지기를 기다리는 상태.
3. **`STABILIZING`**: 데이터가 들어오기 시작함. 지정된 시간(5초) 동안 끊김 없는지 엄격하게 검사하는 상태.
4. **`RUNNING`**: 안정화 통과. 시스템이 정상이므로 지속적으로 감시(Monitoring)하는 상태.
5. **`COOLDOWN`**: 실패 발생. 프로세스를 정리하고 잠시 대기하는 상태.

---

### `zed_watchdog_node.py`

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo
from rcl_interfaces.msg import ParameterDescriptor

import subprocess
import time
import signal
import sys
from enum import Enum

# ================= State Definition =================
class State(Enum):
    IDLE = 0
    LAUNCHING = 1
    STABILIZING = 2
    RUNNING = 3
    COOLDOWN = 4
    FATAL_ERROR = 5
# ================================================

class ZedWatchdog(Node):
    def __init__(self):
        super().__init__('zed_watchdog')

        # --- Parameters (ROS 2 파라미터로 설정 가능) ---
        self.declare_parameter('launch_cmd', ["ros2", "launch", "zed_multi_camera", "zed_multi_camera.launch.py"])
        self.declare_parameter('check_topics', [
            "/zed_node_0/left/camera_info",
            "/zed_node_1/left/camera_info",
            "/zed_node_2/left/camera_info",
            "/zed_node_3/left/camera_info"
        ])
        self.declare_parameter('boot_timeout', 60.0)    # 부팅 제한 시간
        self.declare_parameter('stability_duration', 5.0) # 안정화 검사 시간
        self.declare_parameter('msg_timeout', 1.0)      # 데이터 끊김 허용 시간
        self.declare_parameter('cooldown_sec', 10.0)    # 재시작 대기 시간
        self.declare_parameter('max_attempts', 3)       # 최대 시도 횟수

        # 변수 로딩
        self.launch_cmd = self.get_parameter('launch_cmd').value
        self.target_topics = self.get_parameter('check_topics').value
        self.boot_timeout = self.get_parameter('boot_timeout').value
        self.stability_duration = self.get_parameter('stability_duration').value
        self.msg_timeout = self.get_parameter('msg_timeout').value
        self.cooldown_sec = self.get_parameter('cooldown_sec').value
        self.max_attempts = self.get_parameter('max_attempts').value

        # --- Internal Variables ---
        self.process = None
        self.state = State.IDLE
        self.attempt_count = 0
        self.state_start_time = 0.0
        self.last_log_time = 0.0
        
        # 토픽별 마지막 수신 시간 (Watchdog용)
        self.topic_last_seen = {topic: 0.0 for topic in self.target_topics}
        self.subs = []

        # --- Subscribers Setup ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        for topic in self.target_topics:
            self.get_logger().info(f"Subscribing to: {topic}")
            sub = self.create_subscription(
                CameraInfo, topic,
                lambda msg, t=topic: self.topic_callback(msg, t), qos
            )
            self.subs.append(sub)

        # --- Main Logic Timer (Frequency: 10Hz) ---
        self.create_timer(0.1, self.fsm_loop)
        
        # Start immediately
        self.transition_to(State.COOLDOWN) # 쿨다운을 거쳐 시작하도록 유도 (초기 정리)
        self.state_start_time = time.time() - self.cooldown_sec + 1.0 # 즉시 시작되게 시간 조작

        self.get_logger().info("🐶 ZED Watchdog Node Started!")

    def topic_callback(self, msg, topic_name):
        # 데이터 유효성 검사 (sec > 0)
        if msg.header.stamp.sec > 0:
            self.topic_last_seen[topic_name] = time.time()

    def fsm_loop(self):
        """메인 상태 머신 루프"""
        now = time.time()

        # 1. 공통: 프로세스 생존 확인 (LAUNCHING 이후부터)
        if self.state in [State.LAUNCHING, State.STABILIZING, State.RUNNING]:
            if self.process is None or self.process.poll() is not None:
                self.get_logger().error("🚨 Child process died unexpectedly!")
                self.transition_to(State.COOLDOWN)
                return

        # 2. 상태별 로직
        if self.state == State.LAUNCHING:
            # 부팅 타임아웃 체크
            if now - self.state_start_time > self.boot_timeout:
                self.get_logger().error(f"❌ Boot Timeout ({self.boot_timeout}s).")
                self.transition_to(State.COOLDOWN)
                return
            
            # 모든 토픽이 한 번이라도 들어왔는지 확인
            all_started = all(t > 0.0 for t in self.topic_last_seen.values())
            if all_started:
                self.get_logger().info("⚡ All topics detected! Starting Stability Check.")
                self.transition_to(State.STABILIZING)

        elif self.state == State.STABILIZING:
            # 안정화 기간 동안 끊김 확인
            is_healthy, bad_topic = self.check_topic_health()
            if not is_healthy:
                self.get_logger().warn(f"⚠️ Stability Failed: {bad_topic} stalled.")
                self.transition_to(State.COOLDOWN)
                return
            
            # 안정화 시간 달성 확인
            if now - self.state_start_time >= self.stability_duration:
                self.get_logger().info(f"✅ System Stable ({self.stability_duration}s). Swiching to RUNNING.")
                self.transition_to(State.RUNNING)

        elif self.state == State.RUNNING:
            # 지속적인 감시
            is_healthy, bad_topic = self.check_topic_health()
            if not is_healthy:
                self.get_logger().error(f"🚨 Runtime Failure: {bad_topic} stopped publishing.")
                self.transition_to(State.COOLDOWN)
                return
            
            # 생존 로그 (5초 간격)
            if now - self.last_log_time > 5.0:
                self.get_logger().info("🟢 Monitoring: System Healthy.")
                self.last_log_time = now

        elif self.state == State.COOLDOWN:
            # 대기 시간 확인
            if now - self.state_start_time > self.cooldown_sec:
                self.start_launch_sequence()

        elif self.state == State.FATAL_ERROR:
            pass # 아무것도 안 함 (사용자 개입 필요)

    def start_launch_sequence(self):
        """새로운 시도 시작"""
        if self.attempt_count >= self.max_attempts:
            self.get_logger().fatal("💥 Max attempts reached. Giving up.")
            self.transition_to(State.FATAL_ERROR)
            return

        self.attempt_count += 1
        self.get_logger().info(f"🚀 [Attempt {self.attempt_count}/{self.max_attempts}] Launching ZED...")
        
        # 이전 토픽 기록 초기화 (중요)
        self.topic_last_seen = {topic: 0.0 for topic in self.target_topics}
        
        # 프로세스 실행
        try:
            # shell=False 권장
            self.process = subprocess.Popen(self.launch_cmd)
            self.transition_to(State.LAUNCHING)
        except Exception as e:
            self.get_logger().error(f"Failed to popen: {e}")
            self.transition_to(State.COOLDOWN)

    def transition_to(self, new_state):
        """상태 전환 헬퍼"""
        self.state = new_state
        self.state_start_time = time.time()
        
        if new_state == State.COOLDOWN:
            self.cleanup_process()
            self.get_logger().info(f"⏳ Cooldown for {self.cooldown_sec}s...")

    def check_topic_health(self):
        """모든 토픽이 MSG_TIMEOUT 내에 갱신되었는지 확인"""
        now = time.time()
        for topic, last_seen in self.topic_last_seen.items():
            if now - last_seen > self.msg_timeout:
                return False, topic
        return True, None

    def cleanup_process(self):
        """프로세스 정리 (pkill 포함)"""
        if self.process:
            if self.process.poll() is None:
                self.get_logger().info("🧹 Terminating process...")
                self.process.terminate()
                try:
                    self.process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    self.process.kill()
            self.process = None
        
        # ZED 관련 잔여 프로세스 강제 정리 (Wait 0.5s)
        subprocess.run(["pkill", "-f", "zed_wrapper_node"])
        subprocess.run(["pkill", "-f", "zed_multi_camera"])

    def destroy_node(self):
        # 노드 종료 시 정리
        self.cleanup_process()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ZedWatchdog()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

```

### 이 코드의 장점 (ROS 2 Native 방식)

1. **Non-blocking**: `time.sleep()`을 전혀 쓰지 않습니다. 대신 `fsm_loop`가 0.1초마다 호출되어 현재 상태를 체크합니다. 덕분에 노드가 "멈춰있는" 시간이 없어 언제든지 `Ctrl+C`나 다른 콜백에 즉각 반응합니다.
2. **구조적 안정성**: `State` Enum을 사용하여 현재 프로그램이 **"부팅 중"**인지, **"안정화 검사 중"**인지, **"감시 중"**인지 명확하게 구분됩니다. 로직 꼬임이 발생할 확률이 매우 낮습니다.
3. **파라미터화**: 코드 수정 없이 실행 시점에 설정을 바꿀 수 있습니다.
```bash
# 예: 재시도 횟수를 5회로, 안정화 시간을 10초로 늘려서 실행
ros2 run my_pkg zed_watchdog --ros-args -p max_attempts:=5 -p stability_duration:=10.0

```


4. **자가 복구**: 감시 모드(`RUNNING`)에서도 데이터가 1초 이상 끊기면 즉시 `check_topic_health`에서 감지하여 `COOLDOWN` -> `LAUNCHING` 사이클로 자동 복구합니다.

이제 이 스크립트 하나만 실행해 두면, ZED 카메라 노드 4개를 알아서 띄우고, 감시하고, 죽으면 살리는 모든 작업을 전담하게 됩니다.
###################################  
######################################  
#######################################  
네, 정확하게 이해하셨습니다! 💯

제가 작성해 드린 `zed_watchdog_node.py`는 **"초기 구동 실패"**와 **"실행 중 멈춤"** 두 가지 상황을 모두 감지하여 자동으로 복구하도록 설계되었습니다.

질문하신 두 가지 상황이 코드 내부에서 어떻게 처리되는지 매칭해 드리겠습니다.

### 1. 초기 구동 모니터링 (Boot & Stabilization)

* **상황:** 노드를 켰는데 데이터가 안 들어오거나, 들어오다가 바로 끊기는 경우.
* **코드 동작:**
* **`State.LAUNCHING`**: `boot_timeout`(60초) 안에 4개의 카메라 모두에서 최소 1번의 데이터가 들어오지 않으면 → **실패 간주, 프로세스 종료(`COOLDOWN`) 후 재시작.**
* **`State.STABILIZING`**: 데이터가 들어오기 시작한 후, `stability_duration`(5초) 동안 `msg_timeout`(1초) 이상 데이터가 끊기면 → **불안정으로 간주, 프로세스 종료(`COOLDOWN`) 후 재시작.**



### 2. 실행 중 모니터링 (Runtime Watchdog)

* **상황:** 잘 실행되다가(예: 30분 뒤) 갑자기 케이블 접촉 불량이나 드라이버 오류로 데이터가 멈춘 경우.
* **코드 동작:**
* **`State.RUNNING`**: 이 상태에서는 계속해서 `check_topic_health()` 함수가 돕니다.
* 만약 마지막 데이터 수신 시간(`topic_last_seen`)이 현재 시간보다 `msg_timeout`(1초) 이상 뒤쳐지면 → **런타임 에러로 간주, 즉시 프로세스 종료(`cleanup_process`) 후 `COOLDOWN` 상태로 전환되어 재시작 시퀀스로 진입.**



---

### 💡 핵심 요약

이 노드는 **"데이터가 흐르지 않으면 죽은 것과 다름없다"**는 철학으로 동작합니다.
단순히 프로세스가 살아있는지는 중요하지 않습니다. **실제 데이터(Topic)가 1초(설정값) 내에 갱신되지 않으면 무조건 껐다 켭니다.**

따라서 현장이나 로봇에 탑재하고 나가셨을 때, ZED 카메라가 멈추더라도 사람이 개입할 필요 없이 스스로 복구하게 됩니다.
