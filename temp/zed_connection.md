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
