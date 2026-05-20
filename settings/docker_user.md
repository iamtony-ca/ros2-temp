기존에 사용하시던 명령어는 **'root' 계정으로 GUI를 실행하기 위한 전형적인 세팅**입니다. 이제 우리가 목표로 하는 **'호스트와 동일한 UID/GID를 가진 일반 사용자 계정으로 실행 + 비밀번호 없는 sudo 사용'** 환경으로 전환하기 위해, CLI 명령어와 Dockerfile을 다음과 같이 변경해야 합니다.
### 1단계: 커스텀 이미지 빌드 (가장 중요)
기존에 쓰시던 osrf/ros:jazzy-desktop-full은 모든 것을 root로 실행하도록 설계되어 있습니다. 여기에 일반 사용자 계정을 생성하고 sudo를 심는 과정을 추가해야 합니다.
아래 내용으로 Dockerfile을 만드세요.
```dockerfile
FROM osrf/ros:jazzy-desktop-full

# 1. sudo 설치
RUN apt-get update && apt-get install -y sudo && rm -rf /var/lib/apt/lists/*

# 2. 빌드 시 인자로 호스트 유저 정보 받기
ARG USERNAME=ros_user
ARG USER_UID=1000
ARG USER_GID=1000

# 3. 호스트와 동일한 UID/GID 유저 생성 및 sudo NOPASSWD 설정
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# 4. 사용자 전환 및 환경 설정
USER $USERNAME
WORKDIR /home/$USERNAME

# ROS2 환경 변수 로드
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

```
이제 다음 명령어로 이미지를 빌드합니다. (이때 호스트의 UID/GID가 자동 주입됩니다.)
```bash
docker build \
  --build-arg USERNAME=$USER \
  --build-arg USER_UID=$(id -u) \
  --build-arg USER_GID=$(id -g) \
  -t my-ros2-jazzy .

```
### 2단계: 변경된 docker run 명령어
이제 root가 아닌 일반 사용자(본인 계정)로 실행하므로, $XAUTHORITY나 root 관련 설정들을 대폭 간소화할 수 있습니다.
```bash
xhost +local:docker # root 대신 docker 그룹 접근 허용

docker run -it --rm \
  --gpus all \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute,display \
  --net=host \
  --privileged \
  --ipc=host \
  --ulimit rtprio=99 \
  --ulimit memlock=-1 \
  --cap-add=SYS_NICE \
  --cap-add=SYS_RESOURCE \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  -v /home/$USER:/home/$USER \
  --name jazzy_gz_test \
  my-ros2-jazzy:latest

```
### 무엇이 좋아졌나요?
 1. **권한 불일치 해결:** 이제 -v /home/$USER:/home/$USER로 호스트의 홈 디렉토리를 컨테이너와 연결해도, 컨테이너 내부의 ros_user와 호스트의 $USER가 **UID가 동일**하기 때문에 파일 수정/생성 시 권한 에러(Permission Denied)가 절대 발생하지 않습니다.
 2. **sudo 해결:** 컨테이너 안에서 sudo apt update를 쳐도 비밀번호를 묻지 않고 바로 실행됩니다.
 3. **불필요한 설정 삭제:**
   * -v $XAUTHORITY:/root/.Xauthority:ro와 --env XAUTHORITY... 같은 **root용 보안/디스플레이 설정이 이제 필요 없습니다.** 일반 사용자 권한으로 GUI가 자연스럽게 동작합니다.
 4. **보안:** 컨테이너를 root로 띄우지 않으므로, 컨테이너 내부에서 문제가 생겨도 호스트 파일 시스템을 직접적으로 오염시킬 위험이 훨씬 줄어듭니다.
### 추가 팁
 * **파일 공유:** 현재 -v /home/$USER:/home/$USER를 통해 전체 홈 디렉토리를 연결했는데, 너무 넓다면 특정 워크스페이스만 -v $HOME/ros2_ws:/home/$USER/ros2_ws 처럼 연결하시는 것이 더 안전합니다.
 * **컨테이너 진입:** 실행 중인 상태에서 추가 터미널을 열 때는 이전과 같이 docker exec -it -u $(id -u) jazzy_gz_test /bin/bash로 접속하시면 완벽합니다.
이렇게 하면 더 이상 sudo 설치 때문에 고생하거나, 파일 소유권 문제로 chown 명령어를 수시로 날릴 필요가 없습니다.
