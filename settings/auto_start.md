Ubuntu 24.04 및 ROS 2 Jazzy 환경에서의 자동 실행, 특히 **"GUI 터미널(gnome-terminal)이 실제로 화면에 뜨는 것"**이 목표라면 `systemd` 서비스보다는 **GNOME Desktop Entry (`.desktop`)** 방식을 사용하는 것이 가장 확실하고 간편합니다. `systemd`는 백그라운드 서비스 관리에 적합해서 `DISPLAY` 환경 변수 설정 등 GUI 창을 띄우려면 설정이 상당히 복잡해지기 때문입니다.

개발자님의 상황에 딱 맞는 **Startup Applications (자동 시작 프로그램)** 방식을 단계별로 안내해 드리겠습니다.

---

### 방법: `~/.config/autostart` 사용하는 방식 (권장)

이 방식은 사용자가 로그인하여 데스크탑 화면(GNOME Shell)이 로드된 후에 스크립트를 실행하므로, `gnome-terminal` 창을 띄우는 데 가장 적합합니다.

#### 1단계: 실행 스크립트 준비 (Delay 추가)

이미 만드신 `sh` 파일이 있겠지만, 부팅 직후 네트워크나 ROS 데몬 등 시스템 준비를 기다리기 위해 `sleep`을 명시적으로 넣어주는 것이 안전합니다.

기존 스크립트(`start_ros.sh`라고 가정)의 상단에 딜레이를 추가하거나, 아래처럼 래퍼(Wrapper) 스크립트를 만드세요.

```bash
#!/bin/bash

# 1. N초 대기 (시스템 부팅 및 GUI 로드 안정화 대기)
sleep 10

# 2. ROS 2 Jazzy 환경 설정 (로그인 쉘이 아닐 수 있으므로 명시적 source 권장)
source /opt/ros/jazzy/setup.bash

# 필요하다면 워크스페이스 source 추가
# source ~/ros2_ws/install/setup.bash

# 3. 작성하신 gnome-terminal 실행 명령어
# 예시: gnome-terminal -- bash -c "ros2 launch my_package my_launch.py; exec bash"
# (-e 옵션도 되지만, 최신 gnome-terminal에서는 -- bash -c 패턴이 더 안정적일 때가 많습니다.)
gnome-terminal --title="ROS2_Jazzy_Node" -- bash -c "ros2 launch demo_nodes_cpp talker; exec bash"

```

* **Tip:** 명령어 끝에 `; exec bash`를 붙여야 노드가 종료되거나 에러가 나도 터미널 창이 바로 꺼지지 않고 남아서 로그를 볼 수 있습니다.
* **권한 부여:**
```bash
chmod +x path/to/your/start_ros.sh

```



#### 2단계: `.desktop` 파일 생성

Ubuntu가 로그인할 때 이 스크립트를 실행하도록 설정 파일을 만듭니다.

터미널을 열고 다음 명령어로 파일을 생성하세요.

```bash
mkdir -p ~/.config/autostart
nano ~/.config/autostart/ros2_autolaunch.desktop

```

파일 내용은 아래와 같이 작성합니다:

```ini
[Desktop Entry]
Type=Application
Exec=/home/username/path/to/your/start_ros.sh
Hidden=false
NoDisplay=false
X-GNOME-Autostart-enabled=true
Name[en_US]=ROS2 Auto Launch
Name=ROS2 Auto Launch
Comment[en_US]=Start ROS 2 Jazzy nodes on boot
Comment=Start ROS 2 Jazzy nodes on boot

```

* **주의:** `Exec` 부분의 경로는 반드시 **절대 경로**(`/home/사용자명/...`)로 작성해야 합니다. `~` 또는 `$HOME`은 여기서 작동하지 않을 수 있습니다.
* `username`을 본인의 실제 계정명으로 변경하세요.

#### 3단계: 테스트 (재부팅)

이제 재부팅을 해봅니다.

```bash
sudo reboot

```

로그인 후 바탕화면이 나오고 약 10초(스크립트에 설정한 시간) 뒤에 `gnome-terminal` 창이 팝업되면서 ROS 2 런치 파일이 실행되는지 확인합니다.

---

### 추가 팁: `gnome-session-properties` (GUI 도구)

만약 텍스트 편집기로 `.desktop` 파일을 만드는 게 귀찮다면, Ubuntu에 기본 내장된 GUI 도구를 쓸 수도 있습니다.

1. 터미널에서 `gnome-session-properties` 입력.
2. "Startup Applications Preferences" 창이 뜨면 **Add** 클릭.
3. **Command** 란에 준비한 스크립트 경로 입력 (예: `/home/user/start_ros.sh`).
4. **Save** 후 닫기.

이 방법도 결과적으로 2단계와 똑같이 `~/.config/autostart/` 폴더에 파일을 생성해 줍니다.

### 요약

1. **스크립트**에 `sleep 10`을 넣어 부팅 대기 시간을 확보한다.
2. `~/.config/autostart/`에 **.desktop 파일**을 만들어 등록한다.
3. 스크립트 내에서 ROS 2 `source`를 명시적으로 해주는 것이 안전하다.

이 방식이 개발자님이 원하시는 "Display 상에 터미널 창 띄우기"를 가장 확실하게 보장합니다.
########################  
#######################  
##########################  
아니요, **그렇게 하면 작동하지 않을 가능성이 매우 높습니다.**

이유는 **`gnome-terminal`이 실행하는 새 창은 `start_ros.sh`와는 완전히 분리된 "새로운 쉘(Shell)"이기 때문**입니다.

1. 작성하신 `start_ros.sh` 파일 안에서 `source`를 하면, **현재 실행 중인 `start_ros.sh` 스크립트 안에서만** ROS 2 명령어를 알게 됩니다.
2. 하지만 `gnome-terminal` 명령어로 **새로 띄운 창**은 부모 스크립트(`start_ros.sh`)의 환경 변수를 온전히 물려받지 못하거나, 초기화된 상태로 시작합니다.
3. 결과적으로 새 창 안에서는 `ros2` 명령어를 찾지 못해 에러가 발생합니다.

따라서, **각 터미널 창이 열릴 때마다 그 안에서 `source`를 수행하도록 명령어를 묶어줘야 합니다.**

---

### 수정된 코드 (가독성을 높인 버전)

매번 긴 `source` 경로를 적는 것은 번거롭고 지저분하므로, **변수**를 활용해 깔끔하게 정리하는 방법을 추천합니다.

```bash
#!/bin/bash

# 1. 부팅 안정화 대기
sleep 10

# 2. 공통으로 사용할 setup 명령어 문자열 정의
# (주의: 이 변수는 단지 문자열일 뿐, 이 스크립트에서 실행되는 것은 아닙니다.)
ROS_SETUP="source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash"

# 3. 각 터미널 실행
# "$ROS_SETUP" 변수가 확장되면서 각 터미널 내부에서 source가 실행됩니다.

# 첫 번째 노드
gnome-terminal --title="Nav2 Launch" -- bash -c "$ROS_SETUP; ros2 launch [pkg1] [launch1]; exec bash"
sleep 5

# 두 번째 노드
gnome-terminal --title="Robot Control" -- bash -c "$ROS_SETUP; ros2 launch [pkg2] [launch2]; exec bash"
sleep 5

# 세 번째 노드
gnome-terminal --title="Camera Node" -- bash -c "$ROS_SETUP; ros2 launch [pkg3] [launch3]; exec bash"

```

### 핵심 포인트

1. **변수(`ROS_SETUP`) 사용:** `source` 명령어들을 문자열 변수에 담아두고, `gnome-terminal` 명령 안에서 `$ROS_SETUP`으로 불러와 실행합니다.
2. **세미콜론(`;`) 연결:** 명령어들을 `;`로 연결해야 앞의 `source`가 끝나고 나서 뒤의 `ros2 launch`가 실행됩니다.
3. **각 창마다 독립적 실행:** 이렇게 해야 각 터미널 창이 켜질 때마다 스스로 ROS 2 환경을 로드하고 노드를 실행하게 됩니다.

이 방식으로 작성하시면 코드는 깔끔하게 유지하면서, 기능은 확실하게 보장할 수 있습니다.
#####################  
######################  
#####################  
결론부터 말씀드리면 **"먼저 환경(Source)을 불러오고, 그 뒤에 설정(Export)을 덮어쓰는 것"**이 가장 안전하고 일반적인 순서입니다.

ROS 2의 `setup.bash` 스크립트들이 내부적으로 특정 변수를 초기화하거나 기본값으로 설정하는 경우가 간혹 있기 때문에, **나의 설정을 가장 마지막에 적용**하여 확실하게 효력을 발휘하게 하는 것이 좋습니다.

추천하는 순서는 다음과 같습니다:

1. `source /opt/ros/jazzy/setup.bash` (기반 환경 로드)
2. `source ~/ros2_ws/install/setup.bash` (내 워크스페이스 로드)
3. `export ...` (나만의 커스텀 설정 적용: Domain ID, RMW, Log Dir 등)

---

### 수정된 `start_ros.sh` 예시

`.bashrc`에 있는 내용을 그대로 스크립트에 옮길 때, 아래와 같이 작성하시면 됩니다. (`ROS_LOG_DIR` 경로 등은 본인 환경에 맞게 확인해주세요.)

```bash
#!/bin/bash

sleep 10

# 1. 경로 변수 설정 (스크립트 내에서만 쓸 변수)
# 주의: ROS_LOG_DIR 같은 경로는 '~' 대신 '/home/계정명' 또는 '$HOME'을 사용하는 것이 안전합니다.
MY_DOMAIN_ID=30
MY_RMW="rmw_cyclonedds_cpp"  # 예: rmw_cyclonedds_cpp 또는 rmw_fastrtps_cpp
MY_LOG_DIR="$HOME/.ros/log_custom" # 원하는 로그 경로

# 2. ROS_SETUP 문자열 하나에 명령어들을 순서대로 담습니다. (세미콜론으로 연결)
# 순서: Source (Jazzy) -> Source (Workspace) -> Export (설정들)
ROS_SETUP="source /opt/ros/jazzy/setup.bash; \
           source ~/ros2_ws/install/setup.bash; \
           export ROS_DOMAIN_ID=$MY_DOMAIN_ID; \
           export RMW_IMPLEMENTATION=$MY_RMW; \
           export ROS_LOG_DIR=$MY_LOG_DIR"

# 3. 각 터미널 실행
# 팁: 로그 디렉토리가 없다면 생성해주는 명령어도 포함하면 좋습니다. (mkdir -p $MY_LOG_DIR)
mkdir -p $MY_LOG_DIR

gnome-terminal -- bash -c "$ROS_SETUP; ros2 launch [pkg1] [launch1]; exec bash"
sleep 5
gnome-terminal -- bash -c "$ROS_SETUP; ros2 launch [pkg2] [launch2]; exec bash"

```

### 각 변수별 주의사항 (Checklist)

1. **`ROS_DOMAIN_ID`**:
* 가장 중요합니다. 이게 맞지 않으면 `node list`에 안 뜹니다.


2. **`RMW_IMPLEMENTATION`**:
* 보통 `rmw_cyclonedds_cpp`나 `rmw_fastrtps_cpp`를 쓰실 텐데, 이 변수가 설정된 상태에서 노드를 실행해야 해당 미들웨어(DDS)로 통신이 시작됩니다. 따라서 **반드시 `ros2 launch` 명령어보다 앞(왼쪽)에** 있어야 합니다. (위의 코드처럼 하면 됩니다.)


3. **`ROS_LOG_DIR`**:
* 이 경로가 **실제로 존재하는지** 확인해야 합니다. 만약 없는 폴더를 지정하면 ROS 2가 실행되면서 경고를 띄우거나 기본 경로로 돌아갈 수 있습니다. 위 코드처럼 `mkdir -p`를 한번 해주는 것이 안전합니다.



이렇게 구성하면 `.bashrc`에 의존하지 않고도, 재부팅 시 항상 완벽하게 세팅된 환경에서 노드들이 실행됩니다.
네, **CUDA 관련 환경변수도 반드시 추가**해주셔야 합니다.

특히 `LD_LIBRARY_PATH`는 ROS 2 노드가 실행될 때, 시스템 라이브러리가 아닌 CUDA 12.6의 특정 라이브러리를 참조해야 한다면(예: ZED SDK, PyTorch, TensorRT 등 사용 시), 이 경로가 없으면 노드가 시작되자마자 `error while loading shared libraries` 에러를 뱉으며 죽을 수 있습니다.

또한 **Docker 컨테이너**도 `gnome-terminal`을 통해 독립된 창으로 띄우면 됩니다.

요청하신 내용을 모두 통합하여, **완벽한 `start_ros.sh` 스크립트**를 구성해 드립니다.

### 통합된 `start_ros.sh`

```bash
#!/bin/bash

# 1. 부팅 후 시스템 안정화 대기 (네트워크, Docker 데몬 등)
sleep 10

# ---------------------------------------------------------
# 2. 환경 변수 및 Setup 정의 (한 줄로 길게 쓰기보다 변수로 정리)
# ---------------------------------------------------------

# 2-1. CUDA 설정 (User의 .bashrc 내용 반영)
# export는 순서가 중요합니다. 기존 PATH 앞에 붙여야 우선순위를 가집니다.
CUDA_SETUP="export PATH=/usr/local/cuda-12.6/bin:\$PATH; \
            export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:\$LD_LIBRARY_PATH"

# 2-2. ROS 2 및 사용자 설정
# ROS_DOMAIN_ID 등은 본인 환경에 맞게 숫자 변경하세요.
MY_DOMAIN_ID=30
MY_RMW="rmw_cyclonedds_cpp" 

ROS_SETUP="source /opt/ros/jazzy/setup.bash; \
           source ~/ros2_ws/install/setup.bash; \
           export ROS_DOMAIN_ID=$MY_DOMAIN_ID; \
           export RMW_IMPLEMENTATION=$MY_RMW"

# 2-3. 최종 실행용 Setup 문자열 합치기 (CUDA -> ROS 순서 추천)
FULL_SETUP="$CUDA_SETUP; $ROS_SETUP"

# ---------------------------------------------------------
# 3. 각 터미널 실행 (ROS 2 Launch + Docker)
# ---------------------------------------------------------

# [ROS Node 1]
gnome-terminal --title="Nav2 Launch" -- bash -c "$FULL_SETUP; ros2 launch [pkg1] [launch1]; exec bash"
sleep 3

# [ROS Node 2]
gnome-terminal --title="Control Node" -- bash -c "$FULL_SETUP; ros2 launch [pkg2] [launch2]; exec bash"
sleep 3

# [Docker Container]
# docker start -ai는 해당 터미널을 점유하므로, 별도 setup 없이 바로 docker 명령만 수행해도 됩니다.
# (단, 사용자가 docker group에 포함되어 sudo 없이 실행 가능해야 함)
gnome-terminal --title="Docker Container" -- bash -c "docker start -ai [container_name]; exec bash"


```

---

### 💡 주의사항 (Docker 관련)

1. **Docker 권한:**
* 스크립트가 `sudo` 없이 실행되므로, 현재 사용자 계정이 `docker` 그룹에 포함되어 있어야 합니다.
* 확인: 터미널에 `docker ps` 쳤을 때 `permission denied`가 안 뜨면 OK.


2. **Docker와 ROS_DOMAIN_ID:**
* `docker start`는 **컨테이너를 처음 생성(`run`)했을 때의 환경 변수**를 그대로 사용합니다.
* 만약 컨테이너 내부에서도 ROS 2 통신을 해야 한다면, 컨테이너를 처음 만들 때 `ROS_DOMAIN_ID=30`을 주입했거나, `--net=host` 모드로 생성했어야 Host의 ROS 노드들과 통신이 됩니다.
* `docker start` 명령어 시점에는 `-e` 옵션으로 환경 변수를 바꿀 수 **없습니다.** (바꾸려면 `docker run`을 다시 하거나, 켜진 뒤 `docker exec`를 써야 함)



### 요약

* **CUDA 경로:** `FULL_SETUP` 변수 맨 앞에 넣어 ROS 실행 전에 로드되도록 했습니다.
* **Docker:** 별도의 `gnome-terminal` 명령으로 분리하여 `start -ai`가 해당 창을 잡고 실행되도록 했습니다.

이 스크립트로 `.desktop` 설정을 유지하시면, 부팅 후 **ROS 터미널들과 Docker 터미널이 촤라락 펼쳐지는** 환경을 구축하실 수 있습니다.
