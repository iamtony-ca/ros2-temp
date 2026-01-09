안녕하세요. ZED SDK 및 Python 전문가로서 답변드립니다.

ZED X는 GMSL2 연결 방식을 사용하며, 시스템에서 0번부터 3번까지의 고유 ID(Camera ID)로 구분됩니다.

요청하신 대로 4개의 카메라를 **순차적으로(Sequential)** 열어서 정보를 확인한 뒤, 리소스를 해제(Close)하고 다음 카메라로 넘어가는 코드를 작성해 드립니다.

### 주요 변경 사항

1. **Loop 구조**: 0~3번 ID를 순회합니다.
2. **`set_from_camera_id(id)`**: `InitParameters`에서 가장 중요한 부분입니다. 이 함수를 호출해야 해당 ID의 카메라를 특정하여 열 수 있습니다.
3. **예외 처리**: 만약 중간에 1번 카메라가 연결이 안 되어 있더라도, 프로그램이 죽지 않고 로그를 남긴 후 2번 카메라 테스트로 넘어가도록 작성했습니다.

### 수정된 코드

```python
########################################################################
#
# Copyright (c) 2022, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

import pyzed.sl as sl
import time # (Optional) 연결 간 안정성을 위해 추가

def main():
    # ZED X 4대를 순차적으로 테스트 (ID: 0, 1, 2, 3)
    # 일반적으로 GMSL 포트 순서대로 ID가 할당됩니다.
    num_cameras = 4

    for cam_id in range(num_cameras):
        print("--------------------------------------------------")
        print(f"Processing Camera ID: {cam_id}")

        # 1. Create a Camera object
        zed = sl.Camera()

        # 2. Create InitParameters and set configuration
        init_params = sl.InitParameters()
        init_params.sdk_verbose = 0
        
        # [중요] 특정 카메라 ID를 타겟으로 설정합니다.
        init_params.set_from_camera_id(cam_id)

        # 3. Open the camera
        print(f"Opening Camera {cam_id}...")
        err = zed.open(init_params)
        
        if err != sl.ERROR_CODE.SUCCESS:
            # 연결 실패 시 에러 출력 후 다음 카메라로 진행 (exit하지 않음)
            print(f"[Error] Failed to open Camera {cam_id}: {err}")
            print("Skipping to next camera...")
            continue

        # 4. Get camera information (Serial Number, Model)
        cam_info = zed.get_camera_information()
        zed_serial = cam_info.serial_number
        zed_model = cam_info.camera_model
        
        print(f"[Success] Camera {cam_id} is Open.")
        print(f" - Model: {zed_model}")
        print(f" - Serial Number: {zed_serial}")

        # 5. Close the camera
        # 반드시 close를 해줘야 해당 GMSL 포트의 리소스가 해제되어 
        # 추후 다시 열 때 문제가 발생하지 않습니다.
        zed.close()
        print(f"Camera {cam_id} Closed.")
        
        # (Optional) 하드웨어 리소스 해제 후 약간의 텀을 주어 안정성 확보
        time.sleep(0.5)

    print("--------------------------------------------------")
    print("All checks finished.")

if __name__ == "__main__":
    main()

```

### 코드 실행 팁 (Expert Note)

1. **순차 처리의 중요성**: `zed.close()`는 매우 중요합니다. ZED X(GMSL)는 하드웨어 리소스를 점유하기 때문에, 하나의 `sl.Camera` 객체가 닫히지 않은 상태에서 동일한 ID나 리소스에 접근하려 하면 충돌이 발생할 수 있습니다.
2. **ID 매핑**: Jetson Orin 등의 환경에서 ZED Box나 GMSL2 카드를 사용할 때, 물리적인 포트 순서와 소프트웨어 ID(0~3)가 일치하지 않는 경우가 간혹 있습니다. 이 코드를 돌려보시면 어떤 물리 포트가 몇 번 ID인지 시리얼 번호를 통해 매핑하실 수 있습니다.
3. **오류 코드**: 만약 `CAMERA_NOT_DETECTED` 에러가 뜬다면, 물리적 연결 혹은 GMSL 드라이버(ZED X Driver) 설치 상태를 확인해야 합니다.
##################
#######################
#########################
네, 요청하신 대로 **4개의 ZED X 카메라를 순차적으로 열어서(Open), 각각 1초 동안 Depth Sensing을 수행하고, 닫는(Close)** 코드를 작성했습니다.

기존 코드에서 반복 횟수(`i < 50`)로 제어하던 부분을 `time` 모듈을 사용하여 **시간 기반(1초)**으로 변경했습니다.

### Python Code

```python
########################################################################
#
# Copyright (c) 2022, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

import pyzed.sl as sl
import math
import numpy as np
import time

def main():
    # 총 4개의 카메라 (ID: 0 ~ 3)
    num_cameras = 4
    
    # 1. 카메라 순차 반복 (Sequential Loop)
    for cam_id in range(num_cameras):
        print("==================================================")
        print(f"Starting Process for Camera ID: {cam_id}")

        # Create a Camera object
        zed = sl.Camera()

        # Create InitParameters and set configuration
        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL  # Use NEURAL depth mode
        init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use meter units
        init_params.sdk_verbose = 0 # 로그 간소화
        
        # [핵심] 현재 루프의 카메라 ID 설정
        init_params.set_from_camera_id(cam_id)

        # Open the camera
        print(f"Opening Camera {cam_id}...")
        status = zed.open(init_params)
        
        # 연결 실패 시 프로그램 종료 대신 다음 카메라로 넘어감 (Robustness)
        if status != sl.ERROR_CODE.SUCCESS:
            print(f"[Error] Open Camera {cam_id} failed: {status}")
            print("Skipping to next camera...")
            continue

        # Create and set RuntimeParameters
        runtime_parameters = sl.RuntimeParameters()

        # 데이터 담을 Mat 객체 생성
        image = sl.Mat()
        depth = sl.Mat()
        point_cloud = sl.Mat()

        print(f"[Success] Camera {cam_id} is ready. Sensing for 1 second...")

        # 2. 시간 측정 시작 (1초 동안 루프)
        start_time = time.time()
        duration = 1.0 # 1초
        frame_count = 0

        while (time.time() - start_time) < duration:
            # A new image is available if grab() returns SUCCESS
            if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                # Retrieve left image
                zed.retrieve_image(image, sl.VIEW.LEFT)
                # Retrieve depth map (Optional, if you need 2D depth map)
                zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
                # Retrieve colored point cloud
                zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

                # Get distance value at the center of the image
                x = round(image.get_width() / 2)
                y = round(image.get_height() / 2)
                
                # get_value returns [err, value]
                err, point_cloud_value = point_cloud.get_value(x, y)

                if err == sl.ERROR_CODE.SUCCESS:
                    # point_cloud_value is [x, y, z, color]
                    # Calculate Euclidean distance: sqrt(x^2 + y^2 + z^2)
                    dist = math.sqrt(point_cloud_value[0]**2 + point_cloud_value[1]**2 + point_cloud_value[2]**2)

                    if math.isfinite(dist):
                        print(f"[Cam {cam_id}] Distance at {{{x};{y}}}: {dist:.2f} mm")
                    else:
                        print(f"[Cam {cam_id}] Distance is Infinite or NaN")
                
                frame_count += 1
            
            # 짧은 sleep을 주어 CPU 점유율을 조금 낮출 수 있으나, 
            # 최대 프레임 확보를 위해 생략 가능.

        print(f"--> Camera {cam_id} finished. Processed {frame_count} frames.")

        # 3. Close the camera (리소스 해제)
        zed.close()
        print(f"Camera {cam_id} Closed.")
        
        # 다음 카메라 연결 전 약간의 안정화 시간
        time.sleep(0.1)

    print("==================================================")
    print("All cameras processed.")

if __name__ == "__main__":
    main()

```

### 코드 작성 포인트 (Expert Guide)

1. **시간 제어 (`time.time()`)**:
* 기존의 `i < 50` 방식은 카메라의 FPS 설정이나 처리 속도에 따라 수행 시간이 달라집니다.
* 정확히 1초를 수행하기 위해 `start_time`을 기록하고 `while (current - start) < 1.0:` 조건을 사용했습니다.


2. **`DEPTH_MODE.NEURAL` 주의사항**:
* `NEURAL` 모드는 AI 모델을 GPU 메모리에 로드해야 하므로 `zed.open()` 시에 시간이 `PERFORMANCE` 모드보다 조금 더 걸릴 수 있습니다.
* 4개의 카메라를 순차적으로 열고 닫을 때, 매번 모델을 로드/언로드하는 오버헤드가 발생합니다. (단, 요청하신 시나리오대로 순차 실행에는 문제가 없습니다.)


3. **메모리 관리**:
* `sl.Mat()` 객체(`image`, `depth`, `point_cloud`)는 루프 안에서 생성해도 되지만, 파이썬의 GC(Garbage Collector)가 작동하므로 큰 문제는 없습니다. 다만, 루프가 매우 빠를 경우 `sl.Mat`을 루프 밖에서 한 번만 선언하고 내부에서 재사용하는 것이 성능상 더 유리할 수 있습니다. (위 코드는 가독성을 위해 루프 내부에 두었습니다.)


4. **`zed.close()`의 필수성**:
* GMSL 카메라는 대역폭을 많이 사용합니다. 이전 카메라를 `close()` 하지 않고 다음 카메라를 열려고 하면 대역폭 부족이나 리소스 충돌로 `CAMERA_NOT_DETECTED` 에러가 발생할 확률이 매우 높습니다. 이 코드는 확실하게 `close`를 호출합니다.
#####################
###########################
############################
안녕하세요! ROS 2 Jazzy와 ZED X 카메라 시스템(Multi-camera setup) 전문가로서 답변 드립니다.

ZED X 4대를 동시에 구동하는 `zed_multi_camera`는 GPU 메모리와 GMSL2 대역폭을 많이 사용하는 무거운 작업입니다. 이를 주기적으로 **"실행 -> 일정 시간 유지 -> 종료(Kill) -> 재시작"** 하는 루프를 만드는 것은 주로 안정성 테스트(Stress Test)나 특정 메모리 이슈를 우회하기 위해 사용됩니다.

이 작업을 수행하는 가장 **세련되고(Pythonic)**, **안전한** 방법 2가지를 제안합니다.

---

### 방법 1: Python 스크립트로 제어 (가장 추천)

Python의 `subprocess` 모듈을 사용하여 프로세스를 실행하고, `SIGINT`(Ctrl+C와 동일) 시그널을 보내 ROS 2 노드가 깔끔하게 종료(Shutdown cleanup)될 시간을 주는 방식입니다.

이 방식은 종료 로그를 확인하거나, 종료 후 ZED 데몬 상태를 점검하는 로직을 추가하기에 가장 적합합니다.

**`run_cycle.py`**

```python
import subprocess
import time
import signal
import os
import sys

# 설정
LAUNCH_CMD = ["ros2", "launch", "zed_multi_camera", "zed_multi_camera.launch.py"]
RUN_DURATION = 60  # 실행 유지 시간 (초)
COOLDOWN = 10      # 재시작 전 대기 시간 (초) - ZED 데몬 안정화 시간 필요

def run_loop():
    loop_count = 1
    
    try:
        while True:
            print(f"=== [Loop {loop_count}] Starting ZED Multi Camera Launch ===")
            
            # 1. 프로세스 실행
            # process = subprocess.Popen(LAUNCH_CMD, shell=False) # 리스트 형태 명령어 사용 시 shell=False 권장
            # 만약 alias나 환경변수 로딩이 필요하면 shell=True 사용하되 명령어는 문자열로 변환 필요
            process = subprocess.Popen(LAUNCH_CMD)

            print(f"--- Running for {RUN_DURATION} seconds... ---")
            
            # 2. 지정된 시간만큼 대기 (실행 유지)
            # 여기서는 단순히 sleep을 썼지만, process.poll()로 조기 종료를 감시할 수도 있습니다.
            try:
                process.wait(timeout=RUN_DURATION)
            except subprocess.TimeoutExpired:
                # 시간이 다 되면 정상적인 흐름이므로 아래 종료 로직으로 이동
                pass

            # 3. 종료 시그널 전송 (Clean Shutdown)
            print("--- Time's up! Sending SIGINT (Ctrl+C) ---")
            process.send_signal(signal.SIGINT)

            # 4. 프로세스가 완전히 죽을 때까지 대기
            try:
                process.wait(timeout=10) # 10초 내에 안 죽으면 강제 종료 고려
            except subprocess.TimeoutExpired:
                print("!!! Warning: Process stuck. Killing forcefully... !!!")
                process.kill()
                process.wait()

            print(f"=== [Loop {loop_count}] Finished. Cooling down for {COOLDOWN}s... ===\n")
            
            # 5. 하드웨어/데몬 안정화를 위한 대기
            time.sleep(COOLDOWN)
            loop_count += 1

    except KeyboardInterrupt:
        print("\n\nStopped by User.")
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait()
        sys.exit(0)

if __name__ == "__main__":
    run_loop()

```

**실행 방법:**

```bash
python3 run_cycle.py

```

---

### 방법 2: Linux `timeout` 명령와 Bash 루프 (가장 간편)

복잡한 로직 없이 터미널에서 바로 실행하고 싶다면, 리눅스 내장 명령어인 `timeout`을 활용하는 것이 가장 좋습니다. `timeout`은 지정된 시간이 지나면 프로세스에 종료 신호를 보냅니다.

**터미널 명령어:**

```bash
while true; do 
    echo "Starting ZED Launch..."
    # 60초(1분) 동안 실행 후 SIGINT(-s 2)를 보내 종료
    timeout -s 2 60s ros2 launch zed_multi_camera zed_multi_camera.launch.py
    
    echo "Waiting for cooldown..."
    sleep 10
done

```

* `-s 2`: `SIGINT` (Ctrl+C) 신호를 보냅니다. (기본값인 `SIGTERM`을 보내면 ROS 2 노드가 비정상 종료로 인식하여 로그가 지저분해지거나 파라미터가 제대로 해제되지 않을 수 있습니다.)
* `sleep 10`: ZED X 카메라 드라이버가 종료된 후, GMSL 링크와 ZED 데몬이 정리될 시간을 벌어줍니다.

---

### 💡 ZED X 전문가의 팁 (중요)

ZED X 4대를 다룰 때 재시작 루프에서 주의해야 할 점이 있습니다:

1. **ZED Daemon 상태**:
ZED X는 백그라운드에서 `zed_x_daemon` 서비스와 통신합니다. 만약 ROS 노드를 재시작했는데 카메라가 인식이 안 된다면, 노드만 껐다 켜는 것으로는 부족할 수 있습니다.
이 경우 Python 스크립트의 종료 단계에 다음 명령을 추가하여 데몬을 강제로 리프레시해야 할 수 있습니다.
```python
subprocess.run(["sudo", "systemctl", "restart", "zed_x_daemon"])

```


*(Docker 내부라면 컨테이너 밖의 데몬과 통신하므로 이 부분은 호스트 설정에 따릅니다)*
2. **Cooldown 시간 필수**:
GMSL 카메라는 USB 카메라보다 하드웨어적인 연결/해제 속도가 느릴 수 있습니다. 프로세스 종료 후 즉시 다시 시작하면 `Camera not detected` 에러가 발생할 확률이 높습니다. 반드시 **5~10초의 대기 시간**(`sleep`)을 두세요.
3. **로그 관리**:
반복 실행하면 로그가 터미널을 가득 채웁니다. Python 스크립트에서 `stdout=subprocess.DEVNULL` 등을 사용해 로그를 숨기거나 파일로 저장하는 것을 추천합니다.

**요약하자면:**
가장 추천하는 방식은 **방법 1 (Python 스크립트)** 입니다. `try-except` 구조로 안전하게 종료를 보장하고, 필요하다면 `zed_x_daemon` 재시작 명령어도 끼워 넣을 수 있어 확장성이 좋습니다.
#######################  
###########################  
############################  
네, 정확한 지적입니다. `ros2 node list`에 노드가 떴다고 해서 카메라가 초기화를 마치고 영상 송출을 시작한 것은 아닙니다. 특히 ZED X는 GMSL 링크 동기화, 펌웨어 로드, 그리고 CUDA 메모리 할당 등의 과정이 있어 노드 실행 후 실제 데이터가 나오기까지 수 초가 더 걸립니다.

**"정상 실행 완료"**의 가장 확실한 정의는 **"실제 센서 데이터가 토픽으로 발행되기 시작했는가?"** 입니다.

이를 위해 Python 스크립트 내에서 가벼운 ROS 2 노드를 하나 만들어, 4개의 카메라 토픽(예: `camera_info`)을 구독(Subscribe)하고, **4개 모두에서 데이터가 한 번 이상 들어왔을 때**를 "준비 완료(Ready)"로 판단하는 코드를 작성해 드리겠습니다.

### 솔루션: Topic Monitoring 방식

이 방식은 단순히 프로세스 존재 여부가 아니라, **데이터 흐름(Data Flow)**을 직접 체크하므로 가장 확실합니다.

**`run_cycle_verified.py`**

```python
import subprocess
import time
import signal
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ================= 사용자 설정 =================
# 실행할 런치 커맨드
LAUNCH_CMD = ["ros2", "launch", "zed_multi_camera", "zed_multi_camera.launch.py"]

# 데이터 수신을 확인할 토픽 리스트 (4개 카메라)
# 'camera_info'는 데이터가 가볍기 때문에 체크용으로 가장 적합합니다.
# 실제 사용하시는 namespace에 맞춰 수정해주세요. (예: /zed_multi/zed_node_0/...)
CHECK_TOPICS = [
    "/zed_node_0/left/camera_info",
    "/zed_node_1/left/camera_info",
    "/zed_node_2/left/camera_info",
    "/zed_node_3/left/camera_info"
]

MAINTAIN_TIME = 30     # 4개 모두 정상 수신 확인 후 유지할 시간 (초)
BOOT_TIMEOUT = 90      # 이 시간 내에 데이터 안 들어오면 실패 간주 (초)
COOLDOWN = 10          # 종료 후 대기 시간 (초)
# ==============================================

class StreamChecker(Node):
    def __init__(self):
        super().__init__('zed_stream_checker')
        self.received_status = {topic: False for topic in CHECK_TOPICS}
        self.subs = []
        
        # Best Effort QoS: 센서 데이터 확인용이므로 빠르게 붙기 위함
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        for topic in CHECK_TOPICS:
            # lambda의 캡처 문제를 해결하기 위해 default argument 사용
            sub = self.create_subscription(
                CameraInfo,
                topic,
                lambda msg, t=topic: self.listener_callback(msg, t),
                qos
            )
            self.subs.append(sub)

    def listener_callback(self, msg, topic_name):
        if not self.received_status[topic_name]:
            self.get_logger().info(f"✅ Data received from: {topic_name}")
            self.received_status[topic_name] = True

    def is_all_ready(self):
        """모든 토픽에서 데이터를 받았는지 확인"""
        return all(self.received_status.values())
    
    def get_missing_topics(self):
        """아직 데이터가 안 온 토픽 리스트 반환"""
        return [t for t, v in self.received_status.items() if not v]

def run_loop():
    rclpy.init()
    loop_count = 1
    
    try:
        while True:
            print(f"\n=== [Loop {loop_count}] Launching ZED Multi Camera... ===")
            
            # 1. Launch 실행
            process = subprocess.Popen(LAUNCH_CMD)
            
            # 2. Checker 노드 생성
            checker_node = StreamChecker()
            
            print(f"waiting for data from {len(CHECK_TOPICS)} topics...")
            start_time = time.time()
            all_ready = False

            # 3. 데이터 수신 대기 루프 (BOOT_TIMEOUT 동안)
            while time.time() - start_time < BOOT_TIMEOUT:
                rclpy.spin_once(checker_node, timeout_sec=0.5)
                
                if checker_node.is_all_ready():
                    all_ready = True
                    setup_duration = time.time() - start_time
                    print(f"🎉 All cameras are streaming! (Setup took {setup_duration:.1f}s)")
                    break
                
                # 로그가 너무 빠르지 않게
                if int(time.time() - start_time) % 5 == 0:
                    missing = checker_node.get_missing_topics()
                    # print(f"Still waiting for: {missing}") 

            # 4. 결과에 따른 처리
            if all_ready:
                print(f"--- Holding verified state for {MAINTAIN_TIME} seconds ---")
                time.sleep(MAINTAIN_TIME)
            else:
                missing = checker_node.get_missing_topics()
                print(f"❌ Timeout! No data from: {missing}")
                print("Process might be stuck or crashed.")

            # 노드 정리
            checker_node.destroy_node()

            # 5. 종료 절차
            print("--- Sending SIGINT (Ctrl+C) ---")
            process.send_signal(signal.SIGINT)

            try:
                process.wait(timeout=20) # 종료 대기 시간 넉넉히
            except subprocess.TimeoutExpired:
                print("!!! Warning: Process stuck. Killing forcefully... !!!")
                process.kill()
                process.wait()

            print(f"=== [Loop {loop_count}] Finished. Cooldown {COOLDOWN}s... ===")
            time.sleep(COOLDOWN)
            loop_count += 1

    except KeyboardInterrupt:
        print("\nStopped by User.")
        if 'process' in locals() and process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait()
        rclpy.shutdown()
        sys.exit(0)

if __name__ == "__main__":
    run_loop()

```

### 코드의 핵심 원리 및 변경점

1. **`StreamChecker` 노드**:
* 스크립트가 실행될 때마다 임시 ROS 2 노드를 생성합니다.
* `CHECK_TOPICS`에 정의된 4개의 토픽을 구독합니다.
* **`CameraInfo` 사용**: 이미지(`Image`) 토픽은 데이터가 커서(특히 4대 동시) 단순 체크용으로 구독하면 네트워크나 CPU에 부하를 줄 수 있습니다. 반면 `CameraInfo`는 메타데이터라 매우 가볍지만, 이미지가 나올 때 같이 나오므로 **"카메라 동작 여부"**를 판단하기에 가장 효율적입니다.


2. **QoS 설정 (`Best Effort`)**:
* ZED 노드가 이미지를 `Best Effort`로 쏘는 경우가 많고, 설령 `Reliable`이라 해도 리스너가 `Best Effort`면 호환됩니다. 빠르게 데이터를 캐치하기 위해 QoS를 느슨하게 설정했습니다.


3. **검증 로직**:
* `rclpy.spin_once()`를 짧게 돌리면서 메시지가 들어오는지 확인합니다.
* 4개 카메라 모두에서 최소 1개의 메시지를 받아야만 `all_ready = True`가 됩니다.
* 하나라도 안 들어오면 타임아웃 처리 후 재시작합니다.



### 사용 전 설정 (필수)

터미널에서 `ros2 topic list`를 통해 실제 사용 중인 네임스페이스를 확인하고 코드 상단의 `CHECK_TOPICS` 리스트를 수정하세요. 보통 멀티 카메라 런치 파일은 아래와 같은 패턴을 가집니다:

```python
CHECK_TOPICS = [
    "/zed_node_0/left/camera_info",  # 네임스페이스가 없다면
    # 또는
    "/zed_multi/zed_node_0/left/camera_info", # 네임스페이스가 있다면
    ...
]

```

이제 "노드가 켜졌나?"라는 불확실한 추측 대신, **"데이터가 실제로 들어오나?"**를 기준으로 완벽하게 테스트 사이클을 돌릴 수 있습니다.
##########################  
################################  
##################################  
이해했습니다. 요구사항이 **"반복적인 스트레스 테스트"**에서 **"성공할 때까지 재시도하는 자동 실행 스크립트"**로 변경되었군요.

이제 로직은 다음과 같이 변경됩니다:

1. **시도(Launch)**: 노드를 실행합니다.
2. **검증(Verify)**: 4개의 카메라 데이터가 모두 들어오는지 확인합니다.
3. **판단(Decision)**:
* **성공 시**: 프로세스를 종료하지 않고 **그대로 유지(Keep Alive)**합니다. (스크립트는 프로세스가 꺼지지 않게 대기 모드로 들어갑니다.)
* **실패 시**: 프로세스를 종료하고 잠시 대기 후 **재시도**합니다.


4. **제한(Limit)**: 재시도 횟수가 **총 3회**를 넘어가면 포기하고 스크립트를 종료합니다.

아래는 이 로직을 구현한 최종 코드입니다.

### `auto_launch_retry.py`

```python
import subprocess
import time
import signal
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ================= 사용자 설정 =================
LAUNCH_CMD = ["ros2", "launch", "zed_multi_camera", "zed_multi_camera.launch.py"]

# 데이터 수신을 확인할 토픽 리스트 (본인의 네임스페이스에 맞게 수정 필수)
CHECK_TOPICS = [
    "/zed_node_0/left/camera_info",
    "/zed_node_1/left/camera_info",
    "/zed_node_2/left/camera_info",
    "/zed_node_3/left/camera_info"
]

BOOT_TIMEOUT = 60      # 이 시간 내에 4개 다 안 켜지면 실패로 간주 (초)
COOLDOWN = 10          # 재시도 전 대기 시간 (초)
MAX_ATTEMPTS = 3       # 최대 시도 횟수
# ==============================================

class StreamChecker(Node):
    def __init__(self):
        super().__init__('zed_startup_checker')
        self.received_status = {topic: False for topic in CHECK_TOPICS}
        
        # Best Effort QoS: 센서 데이터 확인용
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        for topic in CHECK_TOPICS:
            self.create_subscription(
                CameraInfo,
                topic,
                lambda msg, t=topic: self.listener_callback(msg, t),
                qos
            )

    def listener_callback(self, msg, topic_name):
        if not self.received_status[topic_name]:
            self.received_status[topic_name] = True

    def is_all_ready(self):
        return all(self.received_status.values())
    
    def get_missing_topics(self):
        return [t for t, v in self.received_status.items() if not v]

def run_safe_launch():
    rclpy.init()
    
    for attempt in range(1, MAX_ATTEMPTS + 1):
        print(f"\n========================================")
        print(f"🚀 [Attempt {attempt}/{MAX_ATTEMPTS}] Starting ZED Multi Camera...")
        print(f"========================================")

        # 1. 프로세스 실행
        process = subprocess.Popen(LAUNCH_CMD)
        
        # 2. 데이터 수신 확인
        checker_node = StreamChecker()
        start_time = time.time()
        success = False
        
        print(f"Waiting for data streams from {len(CHECK_TOPICS)} cameras...")

        while time.time() - start_time < BOOT_TIMEOUT:
            # 0.5초마다 ROS 콜백 처리
            rclpy.spin_once(checker_node, timeout_sec=0.5)
            
            # 프로세스가 이미 죽었는지 확인 (크래시 등)
            if process.poll() is not None:
                print("🚨 Error: Process terminated unexpectedly during startup.")
                break

            if checker_node.is_all_ready():
                success = True
                print(f"✅ Success! All cameras are streaming correctly.")
                break
        
        # Checker 노드 정리
        checker_node.destroy_node()

        # 3. 결과 처리
        if success:
            print("\n🎉 Launch Successful! Keeping process alive.")
            print("Press Ctrl+C to stop everything.\n")
            try:
                # 성공했으므로 프로세스가 끝날 때까지 무한 대기 (ros2 launch 유지)
                process.wait()
            except KeyboardInterrupt:
                print("\nStopping by User request...")
                process.send_signal(signal.SIGINT)
                process.wait()
            
            # 정상 종료 후 루프 탈출 (프로그램 종료)
            break
        
        else:
            # 실패 처리
            print(f"\n❌ Attempt {attempt} Failed (Timeout or Crash).")
            if attempt < MAX_ATTEMPTS:
                print("Cleaning up and retrying...")
                
                # 프로세스가 살아있다면 종료
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                
                print(f"Waiting {COOLDOWN}s for cooldown...")
                time.sleep(COOLDOWN)
            else:
                print("\n💥 All attempts failed. Please check hardware connection or cables.")
                if process.poll() is None:
                    process.kill()

    rclpy.shutdown()

if __name__ == "__main__":
    run_safe_launch()

```

### 코드 작동 방식

1. **실행 (Attempt 1~3)**:
* `ros2 launch`를 실행합니다.
* ROS 2 노드를 만들어 실제 카메라 데이터(`camera_info`)가 들어오는지 감시합니다.


2. **성공 시 (Success)**:
* 4개의 카메라 데이터가 모두 확인되면, **프로세스를 죽이지 않습니다.**
* `process.wait()` 상태로 진입하여 유저가 Ctrl+C를 누르거나 ZED 노드가 스스로 꺼질 때까지 계속 유지합니다. (목표 달성)


3. **실패 시 (Failure)**:
* `BOOT_TIMEOUT`(60초) 동안 데이터가 안 들어오거나, 도중에 프로세스가 죽으면 **실패**로 간주합니다.
* 실행 중인 프로세스를 `SIGINT`로 정리하고 종료합니다.
* `COOLDOWN`(10초) 동안 대기 후 다음 시도로 넘어갑니다.


4. **최종 실패**:
* 3번 다 실패하면 "All attempts failed" 메시지를 띄우고 종료합니다.



### 팁

* 만약 실행에 성공해서 잘 돌고 있는데, **중간에 카메라 연결이 끊겨서 데이터가 멈추는 상황**까지 감지하고 싶으신가요?
* 현재 코드는 **"초기 구동 성공"**까지만 보장하고 유지 모드로 들어갑니다.
* 만약 구동 중에도 지속적으로 감시하여 문제가 생기면 재시작하길 원하신다면, `process.wait()` 대신 루프를 돌며 `StreamChecker`를 계속 돌리는 로직이 필요합니다. (하지만 보통 초기 구동만 잘 되면 이후에는 잘 유지되므로 현재 코드로 충분할 것입니다.)
#####################
#######################
##########################
네, 사용자의 요구사항을 정확히 반영하여 **"실제로 유효한 타임스탬프(`msg.header.stamp.sec > 0`)가 들어오는지"**까지 검증하는 코드를 작성해 드리겠습니다.

단순히 메시지가 들어오는 것을 넘어, **타임스탬프가 0이 아닌 유효한 값**으로 갱신되고 있는지 확인하면 초기화 단계에서 발생할 수 있는 빈 데이터(Dummy Data) 송출 문제를 걸러낼 수 있어 훨씬 안정적입니다.

### 핵심 변경 사항

1. **타임스탬프 검증 로직 추가**: `msg.header.stamp.sec` 값을 읽어 **0보다 큰지** 확인합니다.
2. **성공 기준**: 4개의 카메라 모두에서 **타임스탬프가 유효한 메시지**를 수신해야 성공으로 간주합니다.
3. **흐름 유지**: 성공 시 프로세스를 종료하지 않고 유지(`wait`)하며, 실패 시에만 재시작합니다. (최대 3회)

### `launch_with_stamp_check.py`

```python
import subprocess
import time
import signal
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ================= 사용자 설정 =================
# 1. 실행할 Launch 명령어
LAUNCH_CMD = ["ros2", "launch", "zed_multi_camera", "zed_multi_camera.launch.py"]

# 2. 검증할 토픽 리스트 (본인 환경에 맞춰 수정 필수)
# 'camera_info'가 데이터가 가볍고 헤더(timestamp)를 포함하고 있어 적합합니다.
CHECK_TOPICS = [
    "/zed_node_0/left/camera_info",
    "/zed_node_1/left/camera_info",
    "/zed_node_2/left/camera_info",
    "/zed_node_3/left/camera_info"
]

# 3. 설정 값
BOOT_TIMEOUT = 60      # 이 시간 내에 검증 실패 시 재시작 (초)
COOLDOWN = 10          # 재시작 전 대기 시간 (초)
MAX_ATTEMPTS = 3       # 최대 시도 횟수
# ==============================================

class TimestampChecker(Node):
    def __init__(self):
        super().__init__('zed_stamp_verifier')
        # 각 토픽별로 검증 성공 여부를 저장 (False: 아직 안 옴, True: 정상 데이터 옴)
        self.verified_status = {topic: False for topic in CHECK_TOPICS}
        
        # Best Effort QoS: 센서 데이터 수신용
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        for topic in CHECK_TOPICS:
            self.create_subscription(
                CameraInfo,
                topic,
                lambda msg, t=topic: self.listener_callback(msg, t),
                qos
            )

    def listener_callback(self, msg, topic_name):
        # 이미 검증된 토픽이면 패스
        if self.verified_status[topic_name]:
            return

        # === 핵심 로직: 타임스탬프 확인 ===
        sec = msg.header.stamp.sec
        nanosec = msg.header.stamp.nanosec
        
        # 타임스탬프가 0이 아닌 유효한 값인지 체크
        if sec > 0:
            self.get_logger().info(f"✅ Verified {topic_name}: stamp.sec={sec}")
            self.verified_status[topic_name] = True
        else:
            # 타임스탬프가 0이면 아직 초기화 중이거나 잘못된 데이터일 수 있음
            # self.get_logger().warn(f"⚠️ Received empty timestamp from {topic_name} (sec=0)")
            pass

    def is_all_verified(self):
        """모든 토픽이 검증되었는지 확인"""
        return all(self.verified_status.values())
    
    def get_pending_topics(self):
        """아직 검증되지 않은 토픽 리스트 반환"""
        return [t for t, v in self.verified_status.items() if not v]

def run_smart_launch():
    rclpy.init()
    
    for attempt in range(1, MAX_ATTEMPTS + 1):
        print(f"\n========================================")
        print(f"🚀 [Attempt {attempt}/{MAX_ATTEMPTS}] Starting ZED Multi Camera...")
        print(f"========================================")

        # 1. 프로세스 실행
        process = subprocess.Popen(LAUNCH_CMD)
        
        # 2. 데이터 및 타임스탬프 검증 시작
        checker = TimestampChecker()
        start_time = time.time()
        success = False
        
        print(f"Waiting for valid timestamps from {len(CHECK_TOPICS)} cameras...")

        while time.time() - start_time < BOOT_TIMEOUT:
            # 0.5초 간격으로 토픽 데이터 확인
            rclpy.spin_once(checker, timeout_sec=0.5)
            
            # 혹시 프로세스가 런치 도중 죽었는지 확인
            if process.poll() is not None:
                print("🚨 Error: Launch process terminated unexpectedly.")
                break

            # 모두 검증 완료되었는지 확인
            if checker.is_all_verified():
                success = True
                elapsed = time.time() - start_time
                print(f"\n🎉 Success! All cameras are publishing valid stamps. (Time: {elapsed:.1f}s)")
                break
        
        # 노드 정리
        checker.destroy_node()

        # 3. 결과에 따른 분기
        if success:
            # 성공했으므로 프로세스를 끄지 않고 무한 대기 (Ctrl+C 누를 때까지)
            print("🟢 System is healthy. Keeping process alive.")
            print("Press Ctrl+C to stop.\n")
            try:
                process.wait() # 여기서 멈춰서 계속 실행 상태 유지
            except KeyboardInterrupt:
                print("\nStopping by User request...")
                process.send_signal(signal.SIGINT)
                process.wait()
            
            # 사용자가 종료했으므로 루프 탈출
            break
        
        else:
            # 실패 처리 (Timeout 또는 Crash)
            pending = checker.get_pending_topics()
            print(f"\n❌ Attempt {attempt} Failed.")
            if pending:
                print(f"   Missing valid data from: {pending}")
            
            if attempt < MAX_ATTEMPTS:
                print("🔄 Cleaning up and retrying...")
                
                # 살아있는 프로세스 종료
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        print("   Force killing process...")
                        process.kill()
                        process.wait()
                
                print(f"⏳ Cooling down for {COOLDOWN}s...")
                time.sleep(COOLDOWN)
            else:
                print("\n💥 All 3 attempts failed. Check ZED X connection/power.")
                if process.poll() is None:
                    process.send_signal(signal.SIGINT) # 마지막 종료 처리

    rclpy.shutdown()

if __name__ == "__main__":
    run_smart_launch()

```

### 코드 분석 및 사용법

1. **`listener_callback` 함수**:
* `msg.header.stamp.sec` 값을 직접 추출합니다.
* `if sec > 0:` 조건을 통해 타임스탬프가 유효한 시간(Epoch time)을 가리킬 때만 `Verified` 상태로 변경합니다. (초기화 중 0으로 들어오는 경우 무시)


2. **성공 시 동작 (`if success:`)**:
* `process.wait()`를 호출하여 파이썬 스크립트가 종료되지 않고, 자식 프로세스(launch)가 계속 실행되도록 붙잡아둡니다.
* 이 상태가 되면 터미널을 끄거나 Ctrl+C를 누르기 전까지 **카메라 4대가 계속 켜져 있는 상태**가 됩니다.


3. **실패 시 동작 (`else:`)**:
* 타임아웃(60초) 내에 4개 모두 확인되지 않으면, `SIGINT`를 보내 프로세스를 종료합니다.
* `COOLDOWN` 후 루프를 돌아 다시 `subprocess.Popen`을 수행합니다.



### 💡 주의사항 (Namespace)

반드시 `ros2 topic list`를 확인하여 `CHECK_TOPICS` 리스트의 토픽명을 본인의 환경에 맞게 수정해주세요. (예: `/zed_multi/zed_node_0/left/camera_info` 등)
###################  
#########################  
############################  
ROS 2에서 특정 이름(`zed_multi...`)을 가진 노드만 골라서 죽이는 것은 생각보다 까다롭습니다. ROS 2에는 ROS 1의 `rosnode kill` 같은 명령어가 없고, 노드는 운영체제(OS) 레벨의 **프로세스**로 존재하기 때문입니다.

따라서 가장 확실한 방법은 **Python 스크립트가 종료될 때(KeyboardInterrupt), ZED 관련 프로세스 이름(`zed_wrapper_node` 등)을 찾아 강제로 종료(`pkill`)**시키는 것입니다.

요청하신 기능을 구현하기 위해 **`cleanup_zed_nodes()` 함수**를 추가한 최종 코드입니다.

### 핵심 추가 기능

1. **`cleanup_zed_nodes()`**:
* `ros2 node list`를 조회하여 `/zed_multi`로 시작하는 노드가 있는지 확인합니다.
* 존재한다면 `pkill` 명령어를 사용해 ZED 관련 프로세스(`zed_wrapper_node`, `component_container`)를 강제로 정리합니다.


2. **`KeyboardInterrupt` 처리**:
* 사용자가 `Ctrl+C`를 누르면 즉시 이 정리 함수가 실행되어 좀비 프로세스가 남지 않도록 합니다.



### `auto_launch_final.py`

```python
import subprocess
import time
import signal
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ================= 사용자 설정 =================
LAUNCH_CMD = ["ros2", "launch", "zed_multi_camera", "zed_multi_camera.launch.py"]

CHECK_TOPICS = [
    "/zed_node_0/left/camera_info",
    "/zed_node_1/left/camera_info",
    "/zed_node_2/left/camera_info",
    "/zed_node_3/left/camera_info"
]

BOOT_TIMEOUT = 60
COOLDOWN = 10
MAX_ATTEMPTS = 3
# ==============================================

class TimestampChecker(Node):
    def __init__(self):
        super().__init__('zed_stamp_verifier')
        self.verified_status = {topic: False for topic in CHECK_TOPICS}
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        for topic in CHECK_TOPICS:
            self.create_subscription(
                CameraInfo, topic,
                lambda msg, t=topic: self.listener_callback(msg, t), qos
            )

    def listener_callback(self, msg, topic_name):
        if self.verified_status[topic_name]: return
        if msg.header.stamp.sec > 0:
            self.verified_status[topic_name] = True

    def is_all_verified(self):
        return all(self.verified_status.values())
    
    def get_pending_topics(self):
        return [t for t, v in self.verified_status.items() if not v]

def cleanup_zed_nodes():
    """
    /zed_multi 로 시작하는 노드들이 남아있는지 확인하고,
    관련 프로세스를 강제로 kill 합니다.
    """
    print("\n🧹 [Cleanup] Checking for stuck ZED nodes...")
    
    try:
        # 1. 현재 살아있는 노드 확인
        result = subprocess.run(["ros2", "node", "list"], capture_output=True, text=True)
        nodes = result.stdout.strip().split('\n')
        
        # /zed_multi 로 시작하는 노드 필터링
        zed_nodes = [n for n in nodes if n.startswith("/zed_multi") or "zed_node" in n]
        
        if zed_nodes:
            print(f"   Found active ZED nodes: {zed_nodes}")
            print("   ⚠️ Force killing ZED processes...")
            
            # 2. 관련 프로세스 강제 종료 (Nuclear Option)
            # ZED ROS2 Wrapper는 보통 'zed_wrapper_node' 또는 'component_container' 이름으로 실행됨
            # -f 옵션으로 커맨드라인 전체 매칭
            subprocess.run(["pkill", "-f", "zed_wrapper_node"]) 
            subprocess.run(["pkill", "-f", "zed_multi_camera"]) # 런치 파일 잔여물
            
            # 잠시 대기 후 확인
            time.sleep(2)
            print("   ✅ Cleanup command sent.")
        else:
            print("   No active ZED nodes found. Clean exit.")
            
    except Exception as e:
        print(f"   Error during cleanup: {e}")

def run_smart_launch():
    rclpy.init()
    process = None # 프로세스 변수 초기화

    try:
        for attempt in range(1, MAX_ATTEMPTS + 1):
            print(f"\n========================================")
            print(f"🚀 [Attempt {attempt}/{MAX_ATTEMPTS}] Starting ZED Multi Camera...")
            print(f"========================================")

            process = subprocess.Popen(LAUNCH_CMD)
            
            checker = TimestampChecker()
            start_time = time.time()
            success = False
            
            print(f"Waiting for valid timestamps from {len(CHECK_TOPICS)} cameras...")

            while time.time() - start_time < BOOT_TIMEOUT:
                rclpy.spin_once(checker, timeout_sec=0.5)
                
                if process.poll() is not None:
                    print("🚨 Error: Launch process terminated unexpectedly.")
                    break

                if checker.is_all_verified():
                    success = True
                    print(f"\n🎉 Success! All cameras are publishing valid stamps.")
                    break
            
            checker.destroy_node()

            if success:
                print("🟢 System is healthy. Keeping process alive.")
                print("Press Ctrl+C to stop and kill all nodes.\n")
                process.wait() # 무한 대기
                break # 정상 종료 시 루프 탈출
            
            else:
                print(f"\n❌ Attempt {attempt} Failed.")
                if attempt < MAX_ATTEMPTS:
                    print("🔄 Retrying...")
                    # 실행 중인 프로세스 종료
                    if process and process.poll() is None:
                        process.terminate()
                        try:
                            process.wait(timeout=5)
                        except:
                            process.kill()
                    
                    # 확실한 정리를 위해 cleanup 호출
                    cleanup_zed_nodes()
                    
                    print(f"⏳ Cooling down for {COOLDOWN}s...")
                    time.sleep(COOLDOWN)
                else:
                    print("\n💥 All attempts failed.")

    except KeyboardInterrupt:
        print("\n\n🛑 User Interrupt detected (Ctrl+C).")
        
    finally:
        # 프로그램 종료 전 최종 정리
        if process and process.poll() is None:
            print("   Terminating launch process...")
            process.terminate()
            try:
                process.wait(timeout=5)
            except:
                process.kill()
        
        # 여기서 사용자가 요청한 '모든 zed 노드 kill' 로직 수행
        cleanup_zed_nodes()
        
        if rclpy.ok():
            rclpy.shutdown()
        print("👋 Bye.")

if __name__ == "__main__":
    run_smart_launch()

```

### 코드 변경점 설명

1. **`cleanup_zed_nodes()` 함수**:
* 이 함수는 프로그램이 종료될 때(`finally` 블록) 또는 재시도(`retry`) 전에 호출됩니다.
* `pkill -f "zed_wrapper_node"`: ZED 노드의 실제 실행 파일 이름이 보통 `zed_wrapper_node`이므로 이를 타겟팅하여 확실하게 죽입니다.
* `pkill -f "zed_multi_camera"`: 혹시 남아있을 수 있는 Python Launch 프로세스를 죽입니다.


2. **`finally` 블록**:
* 코드가 정상적으로 끝나든, 에러가 나든, `Ctrl+C`로 강제 종료하든 **무조건 실행**되는 블록입니다.
* 여기서 `cleanup_zed_nodes()`를 호출하므로 어떤 상황에서도 터미널 종료 시 ZED 노드들이 깔끔하게 사라집니다.



이제 안심하고 Ctrl+C를 눌러도 뒷단에 좀비 노드가 남지 않습니다.
###################  
#######################  
###########################  
`psutil` 라이브러리를 사용하는 것이 가장 안전하고 파이썬다운(Pythonic) 방법입니다. 이 라이브러리는 프로세스 목록을 순회하며 이름이나 실행 인자(command line argument)를 검사할 수 있어, 단순히 실행 파일 이름뿐만 아니라 `ros2 run pkg exec` 형태로 실행된 프로세스도 잘 찾아냅니다.

요청하신 대로 **1. 이름으로 매칭하여 PID를 찾고**, **2. 해당 PID를 kill** 하는 코드를 작성해 드립니다.

### 방법 1: `psutil` 라이브러리 사용 (권장)

이 방법은 프로세스 정보를 객체로 다룰 수 있어 관리가 쉽고, 예외 처리가 용이합니다.

먼저 라이브러리를 설치해야 합니다 (보통 기본 설치되어 있지 않습니다).

```bash
pip install psutil

```

**Python 코드:**

```python
import psutil
import signal
import os

def kill_process_by_name(target_name):
    found_pids = []
    
    # 1. 실행 중인 모든 프로세스를 순회
    # attrs=['pid', 'name', 'cmdline']으로 필요한 정보만 가져와 속도 최적화
    for proc in psutil.process_iter(attrs=['pid', 'name', 'cmdline']):
        try:
            # 프로세스 정보 가져오기
            process_name = proc.info['name']
            process_cmdline = proc.info['cmdline']
            
            # 매칭 로직: 
            # 1) 프로세스 이름이 정확히 일치하거나
            # 2) 실행 커맨드라인(cmdline) 리스트 안에 해당 문자열이 포함된 경우 (Python 노드 등)
            # cmdline은 리스트 형태이므로 join으로 문자열 변환 후 검색하는 것이 안전함
            if target_name in process_name or (process_cmdline and target_name in ' '.join(process_cmdline)):
                
                # 본인(이 파이썬 스크립트)을 죽이지 않도록 방지
                if proc.pid == os.getpid():
                    continue

                found_pids.append(proc.pid)
                
                # 2. 프로세스 종료 (SIGKILL = -9)
                # 부드럽게 종료하려면 proc.terminate() 사용
                print(f"Killing process: {process_name} (PID: {proc.pid})")
                proc.kill() 
                
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            # 프로세스가 검사 도중 이미 사라졌거나 권한이 없는 경우 무시
            pass

    if not found_pids:
        print(f"No process found with name: {target_name}")
    else:
        print(f"Total killed processes: {len(found_pids)}")

if __name__ == "__main__":
    # 여기에 죽이고 싶은 노드(프로세스) 이름을 입력하세요
    # 예: 'turtlesim_node' 또는 ros2 실행 시의 노드 이름
    TARGET_PROCESS_NAME = "my_ros2_node_exec" 
    
    kill_process_by_name(TARGET_PROCESS_NAME)

```

---

### 방법 2: `subprocess`와 `pgrep` 사용 (라이브러리 설치 불필요)

`psutil`을 설치할 수 없는 환경이라면, 리눅스 기본 명령어인 `pgrep`을 파이썬에서 호출하여 구현할 수 있습니다.

**Python 코드:**

```python
import subprocess
import os
import signal

def kill_process_by_linux_cmd(target_name):
    try:
        # 1. pgrep 명령어로 PID 찾기 (-f 옵션은 전체 커맨드라인 매칭)
        # 결과값으로 PID들이 줄바꿈(\n)으로 구분되어 나옴
        result = subprocess.check_output(["pgrep", "-f", target_name])
        
        # 바이트 문자열을 디코딩하고 줄바꿈으로 분리
        pids = result.decode().strip().split('\n')
        
        my_pid = str(os.getpid())

        for pid in pids:
            # 본인 프로세스는 제외
            if pid == my_pid:
                continue
                
            print(f"Found {target_name} at PID: {pid}. Killing...")
            
            # 2. os.kill을 사용하여 시그널 전송
            # signal.SIGKILL (강제 종료, kill -9와 동일)
            # signal.SIGTERM (정상 종료 요청, kill -15와 동일)
            try:
                os.kill(int(pid), signal.SIGKILL)
            except ProcessLookupError:
                print(f"PID {pid} already dead.")
                
    except subprocess.CalledProcessError:
        print(f"No process found matching: {target_name}")

if __name__ == "__main__":
    TARGET_PROCESS_NAME = "my_ros2_node_exec"
    kill_process_by_linux_cmd(TARGET_PROCESS_NAME)

```

### 💡 ROS 2 사용자를 위한 팁

1. **Node Name vs Process Name:** ROS 2에서 `Node Name`과 실제 리눅스 `Process Name`은 다를 수 있습니다.
* C++ 노드: 보통 실행 파일 이름이 프로세스 이름입니다.
* Python 노드: 프로세스 이름은 `python3`이고, 인자(argument)에 스크립트 경로가 포함됩니다.
* 따라서 위 코드들(특히 `psutil` 예제)은 `cmdline`까지 검사하도록 작성되어 있어 **Python 노드와 C++ 노드 모두 잘 잡아냅니다.**


2. **종료 강도:** 위 코드는 확실한 종료를 위해 `proc.kill()` (SIGKILL)을 사용했습니다. 하지만 로그 저장이나 DB 연결 해제 등 종료 처리가 중요하다면 `proc.terminate()` (SIGTERM)을 먼저 사용하는 것이 좋습니다.

**다음 단계:**
작성하신 노드가 종료될 때 데이터를 저장해야 하거나 정리 작업이 필요하다면, `SIGKILL` 대신 `SIGTERM`을 사용하여 안전하게 종료하는 코드로 수정해 드릴까요?
###################  
########################  
##########################  
네, 좋은 접근입니다. 순간적인 노이즈나 초기화 과정에서 잠깐 튀는 데이터(Glitch)를 정상으로 오판하지 않도록 **"2초간의 안정화(Stability) 검증"** 단계를 추가하겠습니다.

이를 구현하려면 `__init__`에서 각 토픽별로 **'안정화 시작 시간'을 기록할 변수**가 추가로 필요합니다. 따라서 `TimestampChecker` 클래스 전체를 업데이트해 드립니다.

### 변경된 로직 설명

1. **`self.stability_timers`**: 각 토픽별로 유효한 데이터가 **처음 들어온 시간**을 기록합니다.
2. **`listener_callback`**:
* 데이터가 유효(`sec > 0`)하면: 타이머가 비어있을 때 시작 시간을 기록하고, 현재 시간과 비교하여 **2.0초가 지났는지** 확인합니다.
* 데이터가 무효(`sec == 0`)하면: 타이머를 리셋하여 다시 처음부터 2초를 기다리게 합니다.



### 수정된 `TimestampChecker` 클래스

```python
class TimestampChecker(Node):
    def __init__(self):
        super().__init__('zed_stamp_verifier')
        
        # 검증 완료 여부 (False: 미완료, True: 완료)
        self.verified_status = {topic: False for topic in CHECK_TOPICS}
        
        # [NEW] 안정화 시간 측정용 타이머 (None: 측정 전, float: 측정 시작 시간)
        self.stability_timers = {topic: None for topic in CHECK_TOPICS}
        
        # Best Effort QoS 설정
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        for topic in CHECK_TOPICS:
            self.create_subscription(
                CameraInfo,
                topic,
                lambda msg, t=topic: self.listener_callback(msg, t),
                qos
            )

    def listener_callback(self, msg, topic_name):
        # 이미 검증이 끝난 토픽은 연산하지 않고 리턴
        if self.verified_status[topic_name]:
            return

        # 1. 데이터 유효성 검사 (Timestamp sec가 0보다 커야 함)
        if msg.header.stamp.sec > 0:
            current_time = time.time()
            
            # 타이머가 안 켜져 있다면(None), 지금부터 측정 시작
            if self.stability_timers[topic_name] is None:
                self.stability_timers[topic_name] = current_time
                # (옵션) 로그: 안정화 측정 시작
                # self.get_logger().info(f"⏳ {topic_name}: Stability check started...")
            
            else:
                # 이미 측정 중이라면, 경과 시간 확인
                elapsed = current_time - self.stability_timers[topic_name]
                
                # 2.0초 이상 경과했다면 검증 성공
                if elapsed >= 2.0:
                    self.verified_status[topic_name] = True
                    self.get_logger().info(f"✅ Verified {topic_name}: Stable for {elapsed:.1f}s")
        
        else:
            # 유효하지 않은 데이터(sec=0)가 들어오면 타이머 리셋!
            if self.stability_timers[topic_name] is not None:
                self.get_logger().warn(f"⚠️ Reset timer for {topic_name}: Invalid data received.")
                self.stability_timers[topic_name] = None

    def is_all_verified(self):
        return all(self.verified_status.values())
    
    def get_pending_topics(self):
        return [t for t, v in self.verified_status.items() if not v]

```

이제 이 클래스를 기존 코드(`auto_launch_final.py`)의 `TimestampChecker` 부분에 덮어씌우면, **"유효한 데이터가 들어오기 시작해서 2초 동안 끊김이나 에러 없이 지속되어야"** 성공으로 판단하게 됩니다.
#####################  
#######################  
#########################  
네, 아주 좋은 지적입니다. 이전 코드에서는 검증이 끝나면 자원을 아끼기 위해 `checker` 노드를 파괴(`destroy_node`)하고 프로세스만 지켜봤는데(`process.wait`), **지속적인 모니터링과 로깅**을 원하신다면 구조를 바꿔야 합니다.

**핵심 변경 사항 2가지:**

1. **`listener_callback` 수정**: 검증이 완료된(`verified=True`) 이후에도 리턴(return)하지 않고, **주기적으로 로그를 출력**하도록 로직을 변경합니다. (터미널 도배 방지를 위해 3~5초 간격 출력 추천)
2. **메인 루프 수정**: 검증 성공 후 `checker.destroy_node()`를 하지 않고, **`while` 루프 안에서 계속 `spin_once`를 돌려서 콜백이 실행되도록** 유지합니다.

---

### 수정된 전체 코드 (`auto_launch_continuous.py`)

이 코드는 초기 검증(2초 안정화)이 끝나면 **"모니터링 모드"**로 전환되어, 런치 파일이 살아있는 동안 계속해서 토픽 상태를 로깅합니다.

```python
import subprocess
import time
import signal
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ================= 사용자 설정 =================
LAUNCH_CMD = ["ros2", "launch", "zed_multi_camera", "zed_multi_camera.launch.py"]

CHECK_TOPICS = [
    "/zed_node_0/left/camera_info",
    "/zed_node_1/left/camera_info",
    "/zed_node_2/left/camera_info",
    "/zed_node_3/left/camera_info"
]

BOOT_TIMEOUT = 60
COOLDOWN = 10
MAX_ATTEMPTS = 3
LOG_INTERVAL = 5.0  # 검증 완료 후, 로그를 남길 간격 (초)
# ==============================================

class TimestampChecker(Node):
    def __init__(self):
        super().__init__('zed_monitor')
        
        # 검증 상태 관리
        self.verified_status = {topic: False for topic in CHECK_TOPICS}
        self.stability_timers = {topic: None for topic in CHECK_TOPICS}
        
        # [NEW] 로깅 간격 조절용 타이머
        self.last_log_time = {topic: 0.0 for topic in CHECK_TOPICS}
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        for topic in CHECK_TOPICS:
            self.create_subscription(
                CameraInfo, topic,
                lambda msg, t=topic: self.listener_callback(msg, t), qos
            )

    def listener_callback(self, msg, topic_name):
        current_time = time.time()
        sec = msg.header.stamp.sec

        # --- 1. 초기 검증 단계 (Validation Phase) ---
        if not self.verified_status[topic_name]:
            if sec > 0:
                if self.stability_timers[topic_name] is None:
                    self.stability_timers[topic_name] = current_time
                else:
                    elapsed = current_time - self.stability_timers[topic_name]
                    if elapsed >= 2.0:
                        self.verified_status[topic_name] = True
                        self.get_logger().info(f"✅ Verified {topic_name}: Stable (Stamp: {sec})")
            else:
                if self.stability_timers[topic_name] is not None:
                    self.stability_timers[topic_name] = None
        
        # --- 2. 지속 모니터링 단계 (Monitoring Phase) ---
        else:
            # 검증이 끝났어도 계속 실행됨.
            # 단, 터미널 도배를 막기 위해 LOG_INTERVAL마다 한 번씩만 출력
            if current_time - self.last_log_time[topic_name] > LOG_INTERVAL:
                self.get_logger().info(f"📊 [Monitoring] {topic_name} is Alive. (Stamp: {sec})")
                self.last_log_time[topic_name] = current_time

    def is_all_verified(self):
        return all(self.verified_status.values())

def cleanup_zed_nodes():
    print("\n🧹 [Cleanup] Checking for stuck ZED nodes...")
    subprocess.run(["pkill", "-f", "zed_wrapper_node"])
    subprocess.run(["pkill", "-f", "zed_multi_camera"])

def run_smart_launch():
    rclpy.init()
    process = None

    try:
        for attempt in range(1, MAX_ATTEMPTS + 1):
            print(f"\n========================================")
            print(f"🚀 [Attempt {attempt}/{MAX_ATTEMPTS}] Starting ZED Multi Camera...")
            print(f"========================================")

            process = subprocess.Popen(LAUNCH_CMD)
            
            # 노드를 여기서 생성하고 계속 유지함
            checker = TimestampChecker()
            
            start_time = time.time()
            success = False
            
            print(f"Waiting for stability check ({len(CHECK_TOPICS)} cameras)...")

            # === [Phase 1: Boot Check Loop] ===
            while time.time() - start_time < BOOT_TIMEOUT:
                rclpy.spin_once(checker, timeout_sec=0.1)
                
                if process.poll() is not None:
                    print("🚨 Error: Launch process terminated unexpectedly.")
                    break

                if checker.is_all_verified():
                    success = True
                    break
            
            # === [Phase 2: Result Handling] ===
            if success:
                print("\n🎉 Launch Successful! Entering Continuous Monitoring Mode.")
                print("Press Ctrl+C to stop everything.\n")
                
                # [핵심] 성공 후 checker를 파괴하지 않고 무한 루프로 진입하여 spin_once 계속 호출
                try:
                    while process.poll() is None:
                        # 1초마다 spin하여 listener_callback 실행 (로깅 발생)
                        rclpy.spin_once(checker, timeout_sec=1.0)
                except KeyboardInterrupt:
                    raise # 바깥의 except 구문으로 던짐
                
                break # 프로세스가 스스로 죽으면 루프 탈출
            
            else:
                # 실패 처리
                print(f"\n❌ Attempt {attempt} Failed.")
                checker.destroy_node() # 실패했으니 노드 정리하고 재시도
                
                if attempt < MAX_ATTEMPTS:
                    if process.poll() is None:
                        process.terminate()
                        time.sleep(2)
                        process.kill()
                    cleanup_zed_nodes()
                    print(f"⏳ Cooling down {COOLDOWN}s...")
                    time.sleep(COOLDOWN)
                else:
                    print("\n💥 All attempts failed.")

    except KeyboardInterrupt:
        print("\n🛑 User Interrupt detected.")
        
    finally:
        if process and process.poll() is None:
            process.terminate()
            process.wait()
        
        cleanup_zed_nodes()
        
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    run_smart_launch()

```

### 코드 수정 포인트 설명

1. **`listener_callback` 로직 분리**:
* **Phase 1 (Validation)**: `if not self.verified_status[topic_name]:` 블록에서는 기존처럼 2초 안정화를 체크합니다.
* **Phase 2 (Monitoring)**: `else:` 블록이 추가되었습니다. 검증이 끝난 후에는 이곳이 실행됩니다. 매번 출력하면 너무 빠르므로 `LOG_INTERVAL`(5초)마다 현재 들어오는 `stamp.sec`를 출력하여 "살아있음"을 표시합니다.


2. **`success` 이후 처리 방식 변경**:
* 이전 코드: `process.wait()` (파이썬 멈춤, 자식 프로세스만 대기)
* **현재 코드**:
```python
while process.poll() is None:
    rclpy.spin_once(checker, timeout_sec=1.0)

```


이 부분이 핵심입니다. 프로세스가 살아있는 동안 **Python도 멈추지 않고 계속 `spin_once`를 돕니다.** 덕분에 `listener_callback`이 계속 호출되어 로깅이 가능해집니다.



이제 실행하시면 **"초기 검증 성공"** 메시지 이후에도, 5초마다 각 카메라가 잘 작동하고 있다는 로그(`[Monitoring] ... Alive`)가 계속 찍히게 됩니다.
##################  
######################  
#########################  
네, 원하시는 로직은 **"초기 구동 후, 엄격한 안정화 검사(Stability Check) 기간을 거쳐, 그 기간 동안 단 한 번이라도 데이터가 끊기면 즉시 재시작"**하는 구조입니다.

이 방식은 단순히 "데이터가 들어왔는가?"를 확인하는 것보다 훨씬 신뢰성이 높습니다. GMSL 카메라나 USB 대역폭 문제로 인해 **"켜지긴 했는데 데이터가 간헐적으로 끊기는"** 불안정한 상태를 걸러낼 수 있기 때문입니다.

### 핵심 로직 변경 사항

1. **Phase 1 (Start)**: 4개의 카메라로부터 첫 번째 유효 데이터가 들어오기를 기다립니다.
2. **Phase 2 (Strict Check)**: 모든 카메라가 켜진 순간부터 `CHECK_DURATION`(예: 5초) 카운트다운을 시작합니다.
* 이 기간 동안 **데이터 수신 간격이 허용치(`MSG_TIMEOUT`, 예: 1초)를 넘어가면 즉시 실패**로 간주하고 재시작합니다.


3. **Phase 3 (Maintain)**: 검증을 통과하면 프로세스를 유지하며 모니터링 로그를 남깁니다.

---

### `auto_launch_strict.py`

```python
import subprocess
import time
import signal
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ================= 사용자 설정 =================
LAUNCH_CMD = ["ros2", "launch", "zed_multi_camera", "zed_multi_camera.launch.py"]

CHECK_TOPICS = [
    "/zed_node_0/left/camera_info",
    "/zed_node_1/left/camera_info",
    "/zed_node_2/left/camera_info",
    "/zed_node_3/left/camera_info"
]

BOOT_TIMEOUT = 60.0    # 초기 데이터 수신까지 기다릴 최대 시간 (초)
CHECK_DURATION = 5.0   # 모든 노드가 켜진 후, '연속으로' 정상을 유지해야 하는 시간 (초)
MSG_TIMEOUT = 1.0      # 검증 기간 중 데이터가 이 시간 이상 안 들어오면 비정상 간주 (초)
COOLDOWN = 10.0        # 재시작 전 대기 시간
MAX_ATTEMPTS = 3       # 최대 재시도 횟수
LOG_INTERVAL = 5.0     # 성공 후 로그 남길 간격
# ==============================================

class StrictChecker(Node):
    def __init__(self):
        super().__init__('zed_strict_monitor')
        
        # 각 토픽별 마지막으로 데이터 수신한 시간 (초기값 0)
        self.last_msg_time = {topic: 0.0 for topic in CHECK_TOPICS}
        self.valid_count = {topic: 0 for topic in CHECK_TOPICS}
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        for topic in CHECK_TOPICS:
            self.create_subscription(
                CameraInfo, topic,
                lambda msg, t=topic: self.listener_callback(msg, t), qos
            )

    def listener_callback(self, msg, topic_name):
        # 타임스탬프가 유효할 때만 시간 갱신
        if msg.header.stamp.sec > 0:
            self.last_msg_time[topic_name] = time.time()
            self.valid_count[topic_name] += 1

    def is_all_started(self):
        """모든 토픽이 최소 1회 이상 데이터를 받았는지 확인"""
        return all(t > 0.0 for t in self.last_msg_time.values())

    def check_stability(self):
        """
        현재 시점에서 모든 토픽이 MSG_TIMEOUT 이내에 데이터를 갱신했는지 확인.
        리턴값: (bool 정상여부, str 실패이유)
        """
        now = time.time()
        for topic, last_time in self.last_msg_time.items():
            if last_time == 0.0:
                return False, f"{topic} not started yet"
            
            diff = now - last_time
            if diff > MSG_TIMEOUT:
                return False, f"{topic} stalled (No data for {diff:.1f}s)"
        
        return True, "Stable"

def cleanup_zed_nodes():
    print("\n🧹 [Cleanup] Checking for stuck ZED nodes...")
    subprocess.run(["pkill", "-f", "zed_wrapper_node"])
    subprocess.run(["pkill", "-f", "zed_multi_camera"])

def run_strict_launch():
    rclpy.init()
    process = None

    try:
        for attempt in range(1, MAX_ATTEMPTS + 1):
            print(f"\n========================================")
            print(f"🚀 [Attempt {attempt}/{MAX_ATTEMPTS}] Starting ZED Multi Camera...")
            print(f"========================================")

            # 1. 프로세스 시작
            process = subprocess.Popen(LAUNCH_CMD)
            checker = StrictChecker()
            
            start_time = time.time()
            nodes_all_started = False
            stability_start_time = None
            success = False
            
            print(f"Waiting for initialization ({len(CHECK_TOPICS)} cameras)...")

            # === [Phase 1 & 2: Boot & Stability Check] ===
            while time.time() - start_time < BOOT_TIMEOUT + CHECK_DURATION:
                rclpy.spin_once(checker, timeout_sec=0.1)
                
                # 프로세스 크래시 확인
                if process.poll() is not None:
                    print("🚨 Error: Process died unexpectedly.")
                    break

                # Step 1: 모든 노드가 데이터를 쏘기 시작했는지 확인
                if not nodes_all_started:
                    if checker.is_all_started():
                        nodes_all_started = True
                        stability_start_time = time.time()
                        print(f"⚡ All nodes started! Beginning {CHECK_DURATION}s stability check...")
                    continue # 아직 시작 안 했으면 계속 대기
                
                # Step 2: 안정화 검사 (Stability Check)
                # 데이터가 끊기지 않고 들어오는지 확인
                is_stable, reason = checker.check_stability()
                
                if not is_stable:
                    print(f"⚠️ Stability Check Failed: {reason}")
                    # 안정화 기간 중 끊겼으므로 즉시 실패 처리 -> 재시작 루프로
                    break 
                
                # 안정화 기간을 무사히 통과했는지 확인
                elapsed_stable = time.time() - stability_start_time
                if elapsed_stable >= CHECK_DURATION:
                    success = True
                    print(f"✅ Success! System stable for {elapsed_stable:.1f}s.")
                    break
            
            # === [Phase 3: Result Handling] ===
            if success:
                print("\n🎉 Launch Verified & Stable! Keeping process alive.")
                print("Press Ctrl+C to stop.\n")
                
                # 성공했으므로 모니터링 모드로 진입 (재시작 로직 탈출)
                last_log = time.time()
                try:
                    while process.poll() is None:
                        rclpy.spin_once(checker, timeout_sec=0.5)
                        
                        # 주기적 생존 로그
                        if time.time() - last_log > LOG_INTERVAL:
                            is_healthy, msg = checker.check_stability()
                            status = "🟢 Healthy" if is_healthy else f"🔴 Unstable ({msg})"
                            print(f"[{time.strftime('%H:%M:%S')}] Monitoring: {status}")
                            last_log = time.time()
                            
                            # (선택사항) 여기서도 끊기면 재시작하고 싶다면?
                            # if not is_healthy: raise Exception("Runtime Failure")
                            
                except KeyboardInterrupt:
                    raise
                break # 정상 종료 시 루프 탈출
            
            else:
                # 실패 (타임아웃, 프로세스 사망, 또는 안정화 실패)
                print(f"\n❌ Attempt {attempt} Failed.")
                checker.destroy_node()
                
                if attempt < MAX_ATTEMPTS:
                    if process.poll() is None:
                        process.terminate()
                        time.sleep(1)
                        process.kill()
                    
                    cleanup_zed_nodes()
                    print(f"⏳ Cooling down {COOLDOWN}s...")
                    time.sleep(COOLDOWN)
                else:
                    print("\n💥 All attempts failed. Please check hardware.")

    except KeyboardInterrupt:
        print("\n🛑 User Interrupt detected.")
        
    finally:
        if process and process.poll() is None:
            process.terminate()
            process.wait()
        cleanup_zed_nodes()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    run_strict_launch()

```

### 코드 작동 원리 (요약)

1. **시작 대기 (Phase 1)**:
* `checker.is_all_started()`가 `True`가 될 때까지 기다립니다. (4개 카메라 모두 최소 1회 데이터 수신)


2. **안정화 검사 (Phase 2)**:
* 모두 시작된 시점(`stability_start_time`)부터 `CHECK_DURATION`(5초) 동안 카운트를 셉니다.
* 이 5초 동안 루프가 돌 때마다 `checker.check_stability()`를 호출합니다.
* **중요:** 만약 어떤 카메라라도 마지막 데이터 수신 시간이 현재 시간보다 `MSG_TIMEOUT`(1초) 이상 차이나면, **즉시 루프를 깨고(`break`) 실패로 간주**하여 재시작합니다.


3. **성공 확정 (Success)**:
* 끊김 없이 5초를 버티면 `success = True`가 되고, `while process.poll() is None` 루프로 들어가서 시스템을 유지합니다.



이 코드를 사용하면 ZED 카메라가 켜지긴 했지만 데이터가 불안정하게 들어오는 상황까지 감지하여 **"완벽하게 안정적인 상태"**일 때만 실행을 유지할 수 있습니다.
##########################################  
###############################################  
###################################################  
네, 아주 좋은 접근입니다. ZED X(GMSL)는 보통 하드웨어 포트(GMSL Port A, B, C, D)에 따라 **Device ID**가 고정되는 특성이 있습니다. 따라서 **"현재 연결된 카메라들의 ID와 S/N을 조회"**한 뒤, 그 매핑 정보를 `ros2 launch` 명령어의 인자(argument)로 동적으로 넣어주는 방식이 가장 깔끔합니다.

이 기능을 구현하기 위해 가장 확실한 방법은 ZED SDK의 Python API(`pyzed.sl`)를 사용하는 것입니다.

기존 코드에 **S/N 자동 탐지 및 명령어 생성 로직**을 추가한 완성된 코드를 드립니다.

### 핵심 추가 기능

1. **`pyzed.sl` 활용**: ZED SDK를 통해 현재 연결된 모든 카메라의 ID와 S/N을 가져옵니다.
2. **동적 매핑**: 카메라 ID(0~3)에 맞는 S/N을 찾아서 Launch Argument(`cam_0_sn:=...`)로 변환합니다.
3. **명령어 조립**: 고정된 `LAUNCH_CMD` 리스트 대신, 실행 시마다 인자가 추가된 리스트를 생성합니다.

---

### `auto_launch_with_sn.py`

```python
import subprocess
import time
import signal
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# [NEW] ZED SDK Import
try:
    import pyzed.sl as sl
except ImportError:
    print("❌ Error: ZED SDK (pyzed) not found. Please install ZED SDK.")
    sys.exit(1)

# ================= 사용자 설정 =================
# 기본 런치 파일 경로 (인자 제외)
BASE_LAUNCH_CMD = ["ros2", "launch", "zed_multi_camera", "zed_multi_camera.launch.py"]

# 예상되는 카메라 개수
EXPECTED_CAM_COUNT = 4

CHECK_TOPICS = [
    "/zed_node_0/left/camera_info",
    "/zed_node_1/left/camera_info",
    "/zed_node_2/left/camera_info",
    "/zed_node_3/left/camera_info"
]

BOOT_TIMEOUT = 60.0
CHECK_DURATION = 5.0
MSG_TIMEOUT = 1.0
COOLDOWN = 10.0
MAX_ATTEMPTS = 3
LOG_INTERVAL = 5.0
# ==============================================

def get_zed_launch_args():
    """
    연결된 ZED 카메라를 스캔하여 ID와 Serial Number 매핑을 찾고,
    ros2 launch 인자 리스트를 생성하여 반환합니다.
    예: ['cam_0_sn:=12345678', 'cam_1_sn:=87654321', ...]
    """
    print("🔍 Scanning ZED X devices via SDK...")
    
    # ZED SDK로 장치 리스트 가져오기
    dev_list = sl.Camera.get_device_list()
    
    if len(dev_list) < EXPECTED_CAM_COUNT:
        print(f"⚠️ Warning: Found {len(dev_list)} cameras, but expected {EXPECTED_CAM_COUNT}.")
    
    # ID 별 Serial Number 딕셔너리 생성 (예: {0: 1234, 1: 5678})
    # ZED X는 GMSL 포트에 따라 ID가 고정되므로 id 값을 신뢰할 수 있습니다.
    id_sn_map = {dev.id: dev.serial_number for dev in dev_list}
    
    launch_args = []
    
    # 0번부터 3번까지 순회하며 인자 생성
    # launch 파일에서 받는 인자 이름(cam_X_sn)이 맞는지 확인 필요!
    for cam_id in range(EXPECTED_CAM_COUNT):
        if cam_id in id_sn_map:
            sn = id_sn_map[cam_id]
            arg = f"cam_{cam_id}_sn:={sn}" # 이 부분이 핵심
            launch_args.append(arg)
            print(f"   -> Found CAM ID {cam_id}: S/N {sn}")
        else:
            print(f"   -> ❌ CAM ID {cam_id} not detected!")
            # 감지 안 된 경우 처리는 사용자가 결정 (여기선 그냥 넘어감 or 더미값)
            
    return launch_args

class StrictChecker(Node):
    def __init__(self):
        super().__init__('zed_strict_monitor')
        self.last_msg_time = {topic: 0.0 for topic in CHECK_TOPICS}
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        for topic in CHECK_TOPICS:
            self.create_subscription(
                CameraInfo, topic,
                lambda msg, t=topic: self.listener_callback(msg, t), qos
            )

    def listener_callback(self, msg, topic_name):
        if msg.header.stamp.sec > 0:
            self.last_msg_time[topic_name] = time.time()

    def is_all_started(self):
        return all(t > 0.0 for t in self.last_msg_time.values())

    def check_stability(self):
        now = time.time()
        for topic, last_time in self.last_msg_time.items():
            if last_time == 0.0: return False, f"{topic} waiting"
            if now - last_time > MSG_TIMEOUT: return False, f"{topic} stalled"
        return True, "Stable"

def cleanup_zed_nodes():
    print("\n🧹 [Cleanup] Force killing ZED nodes...")
    subprocess.run(["pkill", "-f", "zed_wrapper_node"])
    subprocess.run(["pkill", "-f", "zed_multi_camera"])

def run_strict_launch():
    rclpy.init()
    process = None

    try:
        for attempt in range(1, MAX_ATTEMPTS + 1):
            print(f"\n========================================")
            print(f"🚀 [Attempt {attempt}/{MAX_ATTEMPTS}] Preparing Launch...")
            print(f"========================================")

            # [STEP 1] S/N 감지 및 커맨드 생성
            # 매 시도마다 새로 스캔 (혹시 케이블을 뽑았다 꽂았을 수 있으므로)
            try:
                dynamic_args = get_zed_launch_args()
            except Exception as e:
                print(f"❌ Error during SDK scan: {e}")
                time.sleep(2)
                continue

            # 기본 명령어 + 동적 S/N 인자 합치기
            full_cmd = BASE_LAUNCH_CMD + dynamic_args
            
            print("▶ Executing Command:")
            print("  " + " ".join(full_cmd))
            print("----------------------------------------")

            # [STEP 2] 프로세스 실행
            process = subprocess.Popen(full_cmd)
            checker = StrictChecker()
            
            start_time = time.time()
            nodes_all_started = False
            stability_start_time = None
            success = False
            
            # ... (이하 로직은 이전과 동일: 부팅 대기 -> 안정화 체크) ...
            print(f"Waiting for initialization ({len(CHECK_TOPICS)} cameras)...")
            
            while time.time() - start_time < BOOT_TIMEOUT + CHECK_DURATION:
                rclpy.spin_once(checker, timeout_sec=0.1)
                
                if process.poll() is not None:
                    print("🚨 Error: Process died unexpectedly.")
                    break

                if not nodes_all_started:
                    if checker.is_all_started():
                        nodes_all_started = True
                        stability_start_time = time.time()
                        print(f"⚡ All nodes started! Checking stability for {CHECK_DURATION}s...")
                    continue
                
                is_stable, reason = checker.check_stability()
                if not is_stable:
                    print(f"⚠️ Stability Check Failed: {reason}")
                    break 
                
                if time.time() - stability_start_time >= CHECK_DURATION:
                    success = True
                    print(f"✅ Success! System stable.")
                    break
            
            if success:
                print("\n🎉 Launch Verified! Keeping process alive.")
                print("Press Ctrl+C to stop.\n")
                last_log = time.time()
                try:
                    while process.poll() is None:
                        rclpy.spin_once(checker, timeout_sec=0.5)
                        if time.time() - last_log > LOG_INTERVAL:
                            is_healthy, msg = checker.check_stability()
                            status = "🟢 Healthy" if is_healthy else f"🔴 Unstable ({msg})"
                            print(f"[{time.strftime('%H:%M:%S')}] Monitoring: {status}")
                            last_log = time.time()
                except KeyboardInterrupt:
                    raise
                break 
            
            else:
                print(f"\n❌ Attempt {attempt} Failed.")
                checker.destroy_node()
                if attempt < MAX_ATTEMPTS:
                    if process.poll() is None:
                        process.terminate()
                        time.sleep(1)
                        process.kill()
                    cleanup_zed_nodes()
                    print(f"⏳ Cooling down {COOLDOWN}s...")
                    time.sleep(COOLDOWN)
                else:
                    print("\n💥 All attempts failed.")

    except KeyboardInterrupt:
        print("\n🛑 User Interrupt detected.")
    finally:
        if process and process.poll() is None:
            process.terminate()
            process.wait()
        cleanup_zed_nodes()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    run_strict_launch()

```

### 💡 중요 체크 포인트 (필독)

1. **Launch Argument 이름 확인**:
* 코드에서 `arg = f"cam_{cam_id}_sn:={sn}"` 라고 작성했습니다.
* 실제 사용하시는 `zed_multi_camera.launch.py` 파일 내부에서 이 인자를 받도록 설정되어 있는지 확인해야 합니다.
* 보통 Stereolabs 공식 예제는 `zed_multi_camera.launch.py`에서 `cam_0_sn`, `cam_1_sn` 등을 `DeclareLaunchArgument`로 선언해두거나, `common.yaml` 파일을 덮어쓰는 방식을 사용합니다.
* 만약 인자 이름이 다르면(예: `serial_0`, `sn_0` 등) 코드의 해당 문자열 부분을 수정해주세요.


2. **`pyzed` 라이브러리**:
* ZED SDK가 설치된 환경이라면 `import pyzed.sl`이 정상 작동합니다. 도커 컨테이너 내부라면 ZED 전용 도커 이미지를 사용해야 합니다.


3. **카메라 ID 매핑**:
* ZED X(GMSL)는 하드웨어적으로 ID가 할당되므로, `sl.Camera.get_device_list()`를 호출하면 각 카메라 객체(`dev`)에 `dev.id`와 `dev.serial_number`가 들어있습니다.
* 이 코드는 `dev.id`가 0인 카메라의 S/N을 `cam_0_sn`에 할당합니다. 즉, 사용자가 의도한 "1, 2, 0, 3 순서" 등의 섞인 상황이라도, **"ID 0번 포트에 꽂힌 카메라의 S/N은 0번 노드 인자로"** 정확하게 매칭됩니다.
##########################
##############################
##################################
ROS 2 Jazzy 환경과 ZED X 멀티 카메라 설정을 다루고 계시군요. 전문가로서 `launch` 파일 수정을 도와드리겠습니다.

`zed_multi_camera.launch.py`는 **Composable Node** 방식을 사용하고 있습니다. 이 방식에서는 개별 카메라 노드들이 하나의 **Container** 프로세스 안에서 실행됩니다. 따라서 노드가 비정상 종료(Crash)되었을 때 다시 살리려면, 이들을 담고 있는 **Container 자체에 `respawn` 설정**을 해야 합니다.

추가로 `robot_state_publisher`도 별도 노드로 실행되므로, 이 친구도 죽었을 때 되살아나도록 설정하면 시스템이 더 견고해집니다.

수정해야 할 부분은 크게 두 군데입니다.

---

### 수정 방법

`launch_setup` 함수 내부의 `zed_container` 정의 부분과 `multi_rsp_node` 정의 부분에 `respawn=True`와 `respawn_delay` 옵션을 추가합니다.

#### 1. Composable Node Container (메인 컨테이너) 수정

모든 ZED 카메라 노드가 실행되는 컨테이너입니다. 하나라도 치명적인 오류(Segfault 등)가 발생하면 컨테이너 전체가 재시작되어야 모든 카메라가 복구됩니다.

```python
    # [수정 전]
    # zed_container = ComposableNodeContainer(
    #     name=container_name,
    #     namespace=namespace_val,
    #     package='rclcpp_components',
    #     executable=container_exec,
    #     arguments=['--ros-args', '--log-level', 'info'],
    #     output='screen',
    # )

    # [수정 후] respawn=True 추가
    zed_container = ComposableNodeContainer(
        name=container_name,
        namespace=namespace_val,
        package='rclcpp_components',
        executable=container_exec,
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        respawn=True,        # 자동 재실행 활성화
        respawn_delay=2.0,   # (선택) 너무 빠른 재부팅 방지를 위한 2초 딜레이
    )
    actions.append(zed_container)

```

#### 2. Robot State Publisher 수정

TF 트리를 관리하는 `robot_state_publisher`도 만약을 대비해 재실행 설정을 합니다.

```python
    # [수정 전]
    # multi_rsp_node = Node(
    #     package='robot_state_publisher',
    #     namespace=namespace_val,
    #     executable='robot_state_publisher',
    #     name=rsp_name,
    #     output='screen',
    #     parameters=[{
    #         'robot_description': Command(xacro_command).perform(context)
    #     }]
    # )

    # [수정 후] respawn=True 추가
    multi_rsp_node = Node(
        package='robot_state_publisher',
        namespace=namespace_val,
        executable='robot_state_publisher',
        name=rsp_name,
        output='screen',
        parameters=[{
            'robot_description': Command(xacro_command).perform(context)
        }],
        respawn=True,        # 자동 재실행 활성화
        respawn_delay=2.0,   # (선택) 2초 딜레이
    )

```

---

### 주의사항 (Expert Tip)

1. **컨테이너 기반 재실행:**
현재 구조는 하나의 `zed_multi_container` 안에 여러 카메라(Node Component)가 로드되는 방식입니다. 만약 **카메라 A**에서 메모리 오류가 발생해 프로세스가 죽으면, 같은 프로세스를 공유하는 **카메라 B**도 함께 꺼지게 됩니다. `respawn=True`를 설정하면 컨테이너가 다시 뜨면서 **모든 카메라가 동시에 재시작**됩니다. 멀티캠 동기화 관점에서는 오히려 이것이 더 안전한 동작 방식입니다.
2. **개별 재실행 불가:**
위 코드는 단일 컨테이너 방식이므로, 특정 카메라 하나만 죽었다가 혼자 살아나게 하려면, launch 파일을 수정하여 카메라마다 별도의 `ComposableNodeContainer`를 띄우도록 구조를 변경해야 합니다. (일반적으로는 리소스 절약을 위해 단일 컨테이너를 권장합니다.)

적용 후 `ros2 launch ...`를 실행하고, 강제로 프로세스를 죽여보거나(`kill` 명령어 등) 케이블을 뽑아보며 재실행되는지 테스트해보시길 바랍니다.

도움이 필요하시면 언제든 말씀해주세요!
