JetPack 6.2가 세팅된 Jetson Orin AGX의 시스템 및 하드웨어 구성을 확인하는 명령어와 `alias`의 의미를 설명해 드립니다.

---

### ## 1. Jetson 세팅 및 HW 구성 확인 명령어

터미널에서 다음 명령어들을 사용하여 현재 시스템의 거의 모든 정보를 확인할 수 있습니다.

#### ### 실시간 리소스 및 종합 정보 (가장 유용)

* **`jtop`**
    * 가장 먼저 실행해야 할 필수 도구입니다. CPU/GPU/RAM 사용량, 온도, 전력 모드(nvpmodel), JetPack/L4T 버전, 설치된 SDK(CUDA, TensorRT 등) 버전을 **하나의 대시보드**에서 모두 보여줍니다.
    * (설치가 안 되어 있다면: `sudo -H pip install -U jetson-stats`)

#### ### JetPack / OS 버전

* **`cat /etc/nv_tegra_release`**
    * 현재 설치된 **JetPack(L4T)** 버전을 정확하게 확인하는 가장 표준적인 방법입니다.
* **`lsb_release -a`**
    * 베이스가 되는 Ubuntu 버전(예: 22.04 LTS)을 확인합니다.

#### ### 성능 및 전원 모드

* **`sudo nvpmodel -q`**
    * 현재 설정된 **전원 프로필**을 확인합니다. (예: `MAXN` (최대 성능) 또는 `MODE_15W` (저전력 모드))
    * *Isaac ROS 구동 시 `sudo nvpmodel -m 0` (MAXN 모드) 사용을 권장합니다.*
* **`sudo jetson_clocks --show`**
    * 현재 CPU, GPU, 메모리 등의 클럭이 최대로 고정되어 있는지 상태를 확인합니다.

#### ### 설치된 핵심 SDK 버전

* **`nvcc --version`**
    * NVIDIA **CUDA Toolkit** 버전을 확인합니다.
* **`dpkg -l | grep nvinfer`**
    * **TensorRT** 런타임 라이브러리 버전을 확인합니다.
* **`dpkg -l | grep libcudnn`**
    * **cuDNN** 라이브러리 버전을 확인합니다.

#### ### 하드웨어 및 저장소

* **`df -h`**
    * 현재 마운트된 **저장 장치(디스크) 목록**과 사용량/남은 용량을 보여줍니다. (아래 alias 질문과 직결됩니다.)
* **`lsblk`**
    * 시스템에 연결된 블록 디바이스(내장 eMMC, NVMe SSD, USB 등)의 목록과 마운트 포인트를 보여줍니다.
* **`cat /proc/device-tree/model`**
    * "NVIDIA Orin AGX 64GB" 등 정확한 하드웨어 모델명을 확인합니다.

---

### ## 2. `alias amr='...'` 분석

이것은 `amr`이라는 **단축 명령어(별명)**를 만드는 셸 설정입니다.

터미널에서 `amr`이라고 입력하고 Enter를 누르면, 셸은 실제로 `'cd /mnt/nova_ssd/workspaces/AMR_ROS2_ws'` 명령을 실행합니다. 즉, **지정된 경로로 즉시 이동하는 바로 가기**입니다.

이 경로가 의미하는 것은 다음과 같습니다.

* **`/mnt/nova_ssd`**:
    * 이것은 Jetson Orin AGX의 **기본 내장 저장소(eMMC)가 아닙니다.**
    * `/mnt` 디렉터리는 "mount"의 약자로, 추가 저장 장치를 연결하는 표준 위치입니다.
    * `nova_ssd`라는 이름으로 볼 때, 사용자가 **별도로 장착한 고속 NVMe SSD**를 `/mnt/nova_ssd` 경로에 마운트(연결)한 것으로 보입니다. (Jetson Orin AGX 킷에는 NVMe SSD를 추가할 수 있는 M.2 슬롯이 있습니다.)
    * *앞서 설명드린 `df -h`나 `lsblk` 명령을 실행해 보시면 이 SSD가 마운트된 것을 확인하실 수 있습니다.*

* **`/workspaces/AMR_ROS2_ws`**:
    * 해당 SSD 내부에 `workspaces`라는 폴더를 만들었고, 그 안에 `AMR_ROS2_ws`라는 이름의 폴더가 있습니다.
    * 이름(`_ws` 접미사)으로 볼 때, 이것은 100% **AMR (자율주행로봇) 프로젝트를 위한 ROS 2 워크스페이스**입니다.

**결론:**
이 `alias`는 **"추가로 장착한 `nova_ssd`라는 이름의 SSD에 있는 `AMR_ROS2_ws` ROS 2 작업 폴더로 빠르게 이동하기 위해 만든 개인용 단축 명령어"**입니다.
