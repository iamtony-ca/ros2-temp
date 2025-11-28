Docker 전문가로서 답변드립니다.

현재 상황은 매우 일반적입니다. Docker 컨테이너는 기본적으로 최소한의 권한 설정을 위해 `root` 계정으로 시작하며, 별도로 설정하지 않으면 `/home` 디렉토리도 비어있습니다.

두 가지 방법으로 해결책을 제시해 드립니다.

1.  **현재 실행 중인 컨테이너**에서 바로 계정을 생성하고 전환하는 방법 (일회성)
2.  **Dockerfile**을 수정하여 앞으로 생성될 컨테이너가 자동으로 해당 계정을 사용하게 하는 방법 (권장, 영구적)

-----

### 방법 1: 현재 실행 중인 컨테이너 안에서 해결하기

이미 `docker run`으로 들어와 있는 터미널(root 상태)에서 아래 명령어를 순서대로 입력하세요. (Ubuntu/Debian 기반 이미지 기준)

#### 1\. 사용자 생성 및 설정

```bash
# 1. 새 사용자 생성 (옵션 설명: -m: 홈 디렉토리 생성, -s: 쉘 지정)
useradd -m -s /bin/bash newuser

# 2. 패스워드 설정을 위해 필요한 패키지 설치 (없는 경우)
apt-get update && apt-get install -y sudo

# 3. 패스워드 지정 (입력 후 원하는 비밀번호 두 번 타이핑)
passwd newuser

# 4. (선택사항) 새 사용자에게 sudo 권한 부여
usermod -aG sudo newuser
```

*`newuser` 부분은 원하시는 계정명(예: `developer`, `ros` 등)으로 바꾸시면 됩니다.*

#### 2\. 사용자 전환

이제 생성한 계정으로 전환합니다.

```bash
su - newuser
```

이제 터미널 프롬프트가 `$`로 바뀌고, 위치도 `/home/newuser`로 변경된 것을 확인할 수 있습니다.

-----

### 방법 2: Dockerfile로 영구적으로 만들기 (권장)

컨테이너를 껐다 켜면 위에서 만든 계정은 사라집니다. 매번 생성하지 않으려면 \*\*이미지 빌드 단계(Dockerfile)\*\*에 이 내용을 포함해야 합니다. 이것이 진정한 "Docker스러운" 방식입니다.

**Dockerfile 예시:**

```dockerfile
FROM ubuntu:24.04

# 계정 이름 변수 설정 (원하는 이름으로 변경 가능)
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# 사용자 생성, sudo 설치 및 권한 부여
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# (선택사항) 사용자 패스워드 지정
# 보안상 Dockerfile에 비밀번호를 박제하는 것은 권장되지 않으나, 개발용이라면 아래 줄 주석 해제
# RUN echo "${USERNAME}:yourpassword" | chpasswd

# ★ 핵심: 기본 사용자를 root에서 이 사용자로 변경
USER $USERNAME
WORKDIR /home/$USERNAME

CMD ["/bin/bash"]
```

**이 방식의 장점:**

1.  `docker run`을 할 때마다 자동으로 해당 사용자로 로그인됩니다.
2.  파일 소유권 문제가 줄어듭니다 (호스트의 UID 1000 사용자와 매핑이 용이함).
3.  `sudo`를 사용할 때 비밀번호를 묻지 않도록 설정하여 개발이 편해집니다.

### 요약

지금 당장 급하다면 **방법 1**을 사용하시고, 앞으로 계속 이 환경을 쓰실 거라면 **방법 2**로 이미지를 다시 빌드하는 것을 강력히 추천합니다.

어떤 방식을 더 자세히 알고 싶으신가요?
