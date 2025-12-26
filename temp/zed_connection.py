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
