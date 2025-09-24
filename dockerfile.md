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
ENV ROS_LOG_DIR=/root/ros_logs

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
RUN pip install --no-cache-dir jupyter

# ===================================================================
# ROS 워크스페이스 설정 및 의존성 설치
# ===================================================================
# 작업 디렉토리를 생성하고 설정합니다.
WORKDIR /ros2_ws

# Docker 캐시 최적화를 위해 src 폴더를 먼저 복사합니다.
COPY ./src /ros2_ws/src

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
COPY . /ros2_ws/

# colcon으로 워크스페이스를 빌드합니다.
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

# ===================================================================
# .bashrc에 ROS 환경 자동 소싱 추가
# ===================================================================
# 새 터미널을 열 때마다 자동으로 ROS 환경이 설정되도록 .bashrc에 source 구문을 추가합니다.
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

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
source /ros2_ws/install/setup.bash

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
    ```
  * **프록시 환경:** (`--build-arg` 사용)
    ```bash
    docker build \
      --build-arg PROXY_URL="http://your-proxy-address:port" \
      --build-arg CERT_PATH="company-cert.crt" \
      -t my-amr-app:latest .
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
-v [호스트 PC의 ros2_ws 절대 경로]:/ros2_ws \
--name my_amr_container \
my-amr-app:latest
```

### 4\. 개발 작업

  * **코드 수정**: 호스트 PC의 `ros2_ws` 폴더에서 원하는 에디터로 수정합니다.
  * **컴파일/실행**: 컨테이너 터미널에서 `colcon build`, `ros2 launch` 등을 실행합니다.
  * **추가 접속**: `docker exec -it my_amr_container bash` 명령으로 새 터미널을 열 수 있습니다.
