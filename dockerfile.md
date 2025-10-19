네, 그럼요. 지금까지 우리가 논의하고 결정한 모든 내용을 종합하여, 최종적으로 완성된 ROS 2 개발 환경 패키지를 완벽하게 정리해 드리겠습니다.

-----

## 📜 최종 Dockerfile

이 `Dockerfile`을 프로젝트 워크스페이스(`ros2_ws`)의 최상위 디렉토리에 저장하세요.

```dockerfile
# ===================================================================
# Stage 1: 최종 이미지의 기반이 될 개발용 이미지 생성
# ===================================================================
# ROS 2 Jazzy Desktop 버전을 기반으로 시작합니다. (Rviz 등 GUI 도구 포함)
FROM osrf/ros:jazzy-desktop-noble AS development_base
# FROM osrf/ros:jazzy-desktop-full AS development_base

# ===================================================================
# 프록시 및 사설 인증서 설정 (Proxy and Private Certificate Setup)
# ===================================================================
# 빌드 시점에 --build-arg 옵션으로 프록시와 인증서 정보를 전달받습니다.
ARG PROXY_URL=""
ARG CERT_PATH=""

# 전달받은 인증서가 있다면 시스템에 복사하고 신뢰하도록 업데이트합니다.
# 이 작업은 apt, curl, git, pip 등 모든 프로그램에 적용됩니다.
COPY ${CERT_PATH:-.} /tmp/
RUN if [ -f /tmp/$(basename ${CERT_PATH:-cert.crt}) ]; then \
        cp /tmp/$(basename ${CERT_PATH:-cert.crt}) /usr/local/share/ca-certificates/company-cert.crt && \
        update-ca-certificates; \
    fi

# 전달받은 프록시 URL이 있다면, 시스템 전체 환경 변수로 설정합니다.
# apt, pip, curl, git 등 대부분의 프로그램이 이 변수를 자동으로 사용합니다.
ENV http_proxy=${PROXY_URL}
ENV https_proxy=${PROXY_URL}
ENV HTTP_PROXY=${PROXY_URL}
ENV HTTPS_PROXY=${PROXY_URL}

# pip 설정을 명시적으로 생성합니다.
RUN if [ -n "${PROXY_URL}" ]; then \
        mkdir -p /root/.config/pip && \
        echo "[global]" > /root/.config/pip/pip.conf && \
        echo "proxy = ${PROXY_URL}" >> /root/.config/pip/pip.conf && \
        echo "trusted-host = pypi.python.org pypi.org files.pythonhosted.org" >> /root/.config/pip/pip.conf; \
    else \
        mkdir -p /root/.config/pip && \
        echo "[global]" > /root/.config/pip/pip.conf && \
        echo "trusted-host = pypi.python.org pypi.org files.pythonhosted.org" >> /root/.config/pip/pip.conf; \
    fi

# ===================================================================
# 환경 변수 설정
# ===================================================================
# apt-get 등 패키지 설치 시 대화형 프롬프트를 비활성화하여 자동 빌드를 보장합니다.
ENV DEBIAN_FRONTEND=noninteractive
# 터미널 색상 및 UTF-8 인코딩을 설정합니다.
ENV TERM=xterm-256color
ENV LANG=C.UTF-8

# ===================================================================
# ROS 2 환경 변수 설정
# ===================================================================
# RMW 구현체를 FastDDS로 설정합니다. (cyclonedds 등 다른 것으로 변경 가능)
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# ROS Domain ID를 42로 설정합니다.
ENV ROS_DOMAIN_ID=42
# ROS 로그 파일이 저장될 디렉토리를 지정합니다.
ENV ROS_LOG_DIR=/root/ros2_ws/log

# ===================================================================
# 기본 시스템 및 모니터링/네트워크 도구 설치
# ===================================================================
# 시스템을 업데이트하고 btop 등 기본 유틸리티를 설치합니다.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    curl \
    btop \
    net-tools \
    # 설치 후 apt 캐시를 정리하여 이미지 용량을 줄입니다.
    && rm -rf /var/lib/apt/lists/*

# ===================================================================
# Git SSL 인증서 설정 (사내 Git 서버 등 사용 시)
# ===================================================================
# 옵션 1: (비권장) 모든 SSL 인증서 검증을 비활성화합니다.
# RUN git config --global http.sslVerify false

# 옵션 2: (권장) 특정 사설 인증서(CA)를 신뢰하도록 설정합니다.
# COPY ./company-cert.crt /usr/local/share/ca-certificates/
# RUN update-ca-certificates && \
#     git config --global http.sslCAInfo /etc/ssl/certs/ca-certificates.crt

# ===================================================================
# ROS, Python 및 개발 라이브러리 설치
# ===================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ROS 개발 도구
    ros-dev-tools \
    # Python 및 Jupyter
    python3-pip \
    # GUI 텍스트 에디터 및 이미지 뷰어
    gedit \
    gimp \
    # 센서 및 드라이버 관련 라이브러리 (유저 공간)
    libserial-dev \
#    librealsense2-utils \
#    librealsense2-dev \
    # 기타 개발 라이브러리
    libeigen3-dev \
    python3-pybind11 \
    graphviz \
    graphviz-dev \
    # Xbox 컨트롤러 (xone) 빌드 의존성
#    dkms \
    # MongoDB & Mosquitto
#    mongodb-org \
#    mosquitto \
    # 설치 후 apt 캐시 정리
    && rm -rf /var/lib/apt/lists/*

# ===================================================================
# Python 패키지 설치 (pip)
# ===================================================================
# ROS 2 Humble과의 호환성 등을 위해 특정 버전의 setuptools가 필요할 수 있습니다.
# RUN pip install --no-cache-dir setuptools==58.2.0
# RUN pip install --no-cache-dir jupyter

# ===================================================================
# ROS 워크스페이스 설정 및 의존성 설치
# ===================================================================
# 작업 디렉토리를 생성하고 설정합니다.
WORKDIR /root/ros2_ws

# Docker 캐시 최적화를 위해 src 폴더를 먼저 복사합니다.
COPY ./src /root/ros2_ws/src

# ROS 의존성을 설치합니다. (src/package.xml 기반)
RUN . /opt/ros/jazzy/setup.sh && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# ===================================================================
# 소스 코드 복사 및 빌드 (독립 실행 가능한 이미지 생성용)
# ===================================================================
# 모든 소스 코드를 다시 복사하여 최신 상태를 반영합니다.
COPY . /root/ros2_ws/

# colcon으로 워크스페이스를 빌드합니다.
# 이전 빌드 결과물을 삭제하고, 빌드 옵션을 추가합니다.
RUN . /opt/ros/jazzy/setup.sh && \
    rm -rf build install log && \
    export MAKEFLAGS="-j4" && \
    colcon build \
        --symlink-install \
        --parallel-workers 3 \
        --packages-skip package_skipped

# ===================================================================
# .bashrc에 ROS 환경 자동 소싱 추가
# ===================================================================
# 새 터미널을 열 때마다 자동으로 ROS 환경이 설정되도록 .bashrc에 source 구문을 추가합니다.
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# ===================================================================
# 컨테이너 실행 환경 설정
# ===================================================================
# 컨테이너 실행 시 항상 ROS 환경이 설정되도록 Entrypoint 스크립트를 준비합니다.
COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# 컨테이너의 기본 실행 명령을 bash로 설정합니다.
CMD ["bash"]
```

-----

## 📜 최종 `entrypoint.sh`

이 스크립트 파일을 `Dockerfile`과 같은 위치에 저장하세요.

```bash
#!/bin/bash
set -e

# ROS 2 환경을 소싱합니다.
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash

# Dockerfile의 CMD나 docker run 명령어로 전달된 명령을 최종적으로 실행합니다.
exec "$@"
```

-----

## ⚙️ 최종 개발 워크플로우

### 1\. 호스트 PC 준비

  * **Docker** 설치
  * **커널 레벨 드라이버** 설치 (NVIDIA Driver, PCAN-Driver, Realsense-DKMS 등)
  * **시스템 서비스** 설정 (`chrony` 시간 동기화)
  * **개발/진단 도구** 설치 (`groot2`)

### 2\. Docker 이미지 빌드

`Dockerfile`이 있는 디렉토리에서 아래 명령어를 실행합니다.

  * **일반 환경:**
    ```bash
    docker build -t my-amr-app:latest .
    docker build -t my-amr-app:latest /home/user/my_project/  
    ```
  * **프록시 환경:** (`--build-arg` 사용)
    ```bash
    docker build \
      --build-arg PROXY_URL="http://your-proxy-address:port" \
      --build-arg CERT_PATH="company-cert.crt" \
      --no-cache \
      --pull \
      -t my-amr-app:latest /home/user/my_project/  
    ```

### 3\. Docker 컨테이너 실행

개발 시에는 **볼륨 마운트**를 포함한 아래 명령어로 컨테이너를 실행합니다.

```bash
# GUI 사용을 위해 호스트 터미널에서 미리 실행
xhost +local:root

# 최종 실행 명령어 ([호스트 PC의 ros2_ws 절대 경로]만 수정)
docker run -it --rm \
--net=host \
--privileged \
--ipc=host \
--ulimit rtprio=99 \
--ulimit memlock=-1 \
--cap-add=SYS_NICE \
--cap-add=SYS_RESOURCE \
--device=/dev:/dev \
-v /dev:/dev \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-e DISPLAY=$DISPLAY \
-e LIBGL_ALWAYS_INDIRECT=0 \
-e QT_X11_NO_MITSHM=1 \
-v [호스트 PC의 ros2_ws/src 절대 경로]:/root/ros2_ws/src \
-v amr_build_vol:/root/ros2_ws/build \
-v amr_install_vol:/root/ros2_ws/install \
-v amr_log_vol:/root/ros2_ws/log \
--name my_amr_container \
my-amr-app:latest
```

### 4\. 개발 작업

  * **코드 수정**: 호스트 PC의 `ros2_ws` 폴더에서 원하는 에디터로 수정합니다.
  * **컴파일/실행**: 컨테이너 터미널에서 `colcon build`, `ros2 launch` 등을 실행합니다.
  * **추가 접속**: `docker exec -it my_amr_container bash` 명령으로 새 터미널을 열 수 있습니다.
  
  
######################################  
물론입니다. 기존에 사용하시던 `Dockerfile`은 프록시 설정, 의존성 관리 등 이미 잘 구성되어 있네요. 여기서 저희가 논의했던 **비-루트(non-root) 사용자 설정**과 **`/workspaces` 디렉터리 표준**을 적용하여 훨씬 더 안정적이고 이식성 높은 개발 환경으로 개선해 보겠습니다.

제가 수정한 `Dockerfile`과, 함께 사용하면 좋은 `entrypoint.sh` 스크립트 예시를 함께 생성해 드릴게요.

### 주요 개선 사항:

1.  **비-루트 사용자(`rosdev`) 생성:** `root` 대신 호스트 PC의 사용자 ID와 동일한 ID를 갖는 `rosdev` 사용자를 생성합니다. 이를 통해 파일 권한 문제를 원천적으로 해결합니다.
2.  **작업 디렉터리 변경:** 모든 경로를 `/root/ros2_ws`에서 `/workspaces/ros2_ws`로 변경하여 VS Code Devcontainer 등 최신 개발 환경 표준을 따릅니다.
3.  **소유권 설정:** 생성된 `rosdev` 사용자에게 `/workspaces` 디렉터리의 소유권을 부여합니다. `pip` 설정 파일 등도 `rosdev` 사용자의 홈 디렉터리에 생성되도록 수정합니다.
4.  **개발 vs. 배포 분리:** 개발 환경에서는 소스 코드를 이미지에 빌드하기보다, 호스트에서 마운트하여 사용하는 것이 일반적입니다. 따라서 `colcon build` 부분을 주석 처리하고, 대신 `rosdep install`까지만 수행하여 어떤 프로젝트든 유연하게 개발할 수 있는 "베이스 환경"을 만드는 데 집중했습니다.

아래 파일들을 확인해 보세요\!

```dockerfile
# ===================================================================
# Stage 1: 최종 이미지의 기반이 될 개발용 이미지 생성
# ===================================================================
# ROS 2 Jazzy Desktop 버전을 기반으로 시작합니다. (Rviz 등 GUI 도구 포함)
FROM osrf/ros:jazzy-desktop-noble AS development_base

# ===================================================================
# 빌드 인자(ARG) 정의
# ===================================================================
# 프록시 및 사설 인증서 정보를 전달받습니다.
ARG PROXY_URL=""
ARG CERT_PATH=""
# [개선] 호스트의 사용자 정보를 전달받기 위한 인자를 추가합니다.
ARG USERNAME=rosdev
ARG USER_ID=1000
ARG GROUP_ID=1000

# ===================================================================
# 환경 변수 설정 (ENV)
# ===================================================================
# 시스템 관련 설정
ENV DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm-256color
ENV LANG=C.UTF-8
# ROS 관련 설정
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_DOMAIN_ID=42
# [개선] 로그 디렉터리 경로를 /workspaces 기준으로 변경합니다.
ENV ROS_LOG_DIR=/workspaces/ros2_ws/log

# ===================================================================
# ROOT 권한으로 필요한 작업 수행
# ===================================================================
# 프록시 및 사설 인증서 설정
COPY ${CERT_PATH:-.} /tmp/
RUN if [ -f /tmp/$(basename ${CERT_PATH:-cert.crt}) ]; then \
        cp /tmp/$(basename ${CERT_PATH:-cert.crt}) /usr/local/share/ca-certificates/company-cert.crt && \
        update-ca-certificates; \
    fi

# 시스템 업데이트 및 핵심 도구 설치
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    build-essential \
    git \
    curl \
    btop \
    net-tools \
    # ROS, Python 및 개발 라이브러리 설치
    ros-dev-tools \
    python3-pip \
    gedit \
    gimp \
    libserial-dev \
    libeigen3-dev \
    python3-pybind11 \
    graphviz \
    graphviz-dev \
    # 설치 후 apt 캐시를 정리하여 이미지 용량을 줄입니다.
    && rm -rf /var/lib/apt/lists/*

# --- [핵심 개선 1] 비-루트(non-root) 사용자 생성 ---
# 호스트와 동일한 GID로 그룹 생성
RUN if [ "$GROUP_ID" != "0" ]; then groupadd --gid $GROUP_ID $USERNAME; fi
# 호스트와 동일한 UID/GID로 사용자 생성 및 홈 디렉터리(-m) 설정
RUN if [ "$USER_ID" != "0" ]; then useradd --uid $USER_ID --gid $GROUP_ID -m -s /bin/bash $USERNAME; fi
# 생성한 사용자가 비밀번호 없이 sudo를 사용할 수 있도록 권한 부여
RUN echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# --- [핵심 개선 2] 작업 디렉터리 생성 및 소유권 부여 ---
RUN mkdir -p /workspaces/ros2_ws \
    && chown -R $USERNAME:$USERNAME /workspaces

# ===================================================================
# 비-루트 사용자(rosdev)로 전환
# ===================================================================
USER $USERNAME

# [개선] pip 설정을 root가 아닌 생성된 사용자의 홈 디렉터리에 생성합니다.
RUN if [ -n "${PROXY_URL}" ]; then \
        mkdir -p ~/.config/pip && \
        echo "[global]" > ~/.config/pip/pip.conf && \
        echo "proxy = ${PROXY_URL}" >> ~/.config/pip/pip.conf && \
        echo "trusted-host = pypi.python.org pypi.org files.pythonhosted.org" >> ~/.config/pip/pip.conf; \
    else \
        mkdir -p ~/.config/pip && \
        echo "[global]" > ~/.config/pip/pip.conf && \
        echo "trusted-host = pypi.python.org pypi.org files.pythonhosted.org" >> ~/.config/pip/pip.conf; \
    fi

# ===================================================================
# ROS 워크스페이스 의존성 설치
# ===================================================================
# 작업 디렉토리를 설정합니다.
WORKDIR /workspaces/ros2_ws

# Docker 캐시 최적화를 위해 src 폴더를 먼저 복사합니다.
# [개선] 복사 시 chown 옵션을 주어 rosdev 사용자 소유로 만듭니다.
COPY --chown=$USERNAME:$USERNAME ./src /workspaces/ros2_ws/src

# ROS 의존성을 설치합니다. (src/package.xml 기반)
# sudo를 사용하여 root 권한으로 설치를 진행합니다.
RUN . /opt/ros/jazzy/setup.sh && \
    sudo apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    sudo rm -rf /var/lib/apt/lists/*

# ===================================================================
# [선택] 소스 코드 빌드 (개발용 이미지에서는 주석 처리 권장)
# ===================================================================
# 개발 환경에서는 호스트의 소스를 마운트하여 사용하므로,
# 이미지 빌드 시점에 코드를 빌드하는 것은 선택 사항입니다.
# COPY --chown=$USERNAME:$USERNAME . /workspaces/ros2_ws/
# RUN . /opt/ros/jazzy/setup.sh && \
#     rm -rf build install log && \
#     export MAKEFLAGS="-j4" && \
#     colcon build \
#         --symlink-install \
#         --parallel-workers 3

# ===================================================================
# .bashrc에 ROS 환경 자동 소싱 추가
# ===================================================================
# [개선] root가 아닌 rosdev 사용자의 .bashrc에 source 구문을 추가합니다.
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /workspaces/ros2_ws/install/setup.bash ]; then source /workspaces/ros2_ws/install/setup.bash; fi" >> ~/.bashrc

# ===================================================================
# 컨테이너 실행 환경 설정
# ===================================================================
# Entrypoint 스크립트를 복사합니다. (root 소유여도 실행에 문제 없습니다)
COPY ./entrypoint.sh /usr/local/bin/entrypoint.sh
RUN sudo chmod +x /usr/local/bin/entrypoint.sh
ENTRYPOINT ["entrypoint.sh"]

# 컨테ainer의 기본 실행 명령을 bash로 설정합니다.
CMD ["bash"]

```



### 사용 방법

이제 이 `Dockerfile`과 `entrypoint.sh`를 사용해 이미지를 빌드하고 컨테이너를 실행하면 됩니다.

1.  **`entrypoint.sh` 파일 저장:**
    위의 `entrypoint.sh` 내용을 `Dockerfile`과 같은 위치에 저장하고 실행 권한을 주세요.
    ```bash
    chmod +x entrypoint.sh

    ```
2.  **이미지 빌드:**
    `--build-arg`를 사용하여 호스트 PC의 사용자 정보를 컨테이너에 전달합니다.
    ```bash
    docker build \
        --build-arg USER_ID=$(id -u) \
        --build-arg GROUP_ID=$(id -g) \
        --build-arg USERNAME=rosdev \
        -t my-ros-jazzy-dev .

    ```
3.  **컨테이너 실행:**
    `-v` 옵션으로 호스트의 소스 코드를 컨테이너의 `/workspaces/ros2_ws/src`에 마운트합니다.
    ```bash
    docker run -it --rm \
        -v ./src:/workspaces/ros2_ws/src \
        my-ros-jazzy-dev

    ```

이제 컨테이너 내부는 `rosdev` 사용자로 실행되며, 호스트와 파일을 주고받을 때 더 이상 `sudo`를 사용할 필요가 없습니다. 훨씬 깔끔하고 안전한 개발 환경이 준비되었습니다\!
