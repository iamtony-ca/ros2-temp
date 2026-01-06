Ubuntu 24.04 (Noble Numbat) 환경에서 부팅 시 루트 권한이 필요한 하드웨어 제어 명령어를 실행하는 가장 **현대적이고 안정적인 방법은 `systemd` 서비스**를 만드는 것입니다.

`rc.local`은 구식이며, `.bashrc`는 로그인 시에만 실행되므로 적합하지 않습니다. 특히 `jetson_clocks`와 `ip link` 제어는 시스템 레벨에서 처리되어야 합니다.

아래 단계를 따라 설정하면 정확히 부팅 완료 후 10초 뒤에 해당 명령들이 실행됩니다.

---

### 1단계: 실행 스크립트 작성

먼저, 명령어를 담은 스크립트 파일을 만듭니다. `systemd`가 루트 권한으로 이 스크립트를 실행하므로, 스크립트 내부에는 `sudo`를 뺄 것입니다.

터미널을 열고 다음 명령어로 파일을 생성합니다:

```bash
sudo nano /usr/local/bin/setup_hardware.sh

```

파일 내부에 아래 내용을 붙여넣으세요:

```bash
#!/bin/bash

# 1. 부팅 완료 후 네트워크 등 안정화를 위해 10초 대기
sleep 10

# 2. 네트워크 인터페이스 설정 (MTU 9000 - Jumbo Frame)
ip link set dev eno1 down
ip link set dev eno1 mtu 9000
ip link set dev eno1 up

# 3. Jetson 성능 최대화
# (이 명령어는 Jetson 플랫폼이 아니면 에러가 날 수 있으나, 무시하고 진행됩니다)
if command -v jetson_clocks &> /dev/null; then
    jetson_clocks
fi

```

**저장 및 종료:** `Ctrl + O`, `Enter`, `Ctrl + X`

이제 실행 권한을 부여합니다:

```bash
sudo chmod +x /usr/local/bin/setup_hardware.sh

```

### 2단계: Systemd 서비스 파일 생성

이제 우분투가 부팅될 때 위 스크립트를 실행하도록 서비스 파일을 만듭니다.

```bash
sudo nano /etc/systemd/system/custom-hardware-setup.service

```

아래 내용을 붙여넣으세요:

```ini
[Unit]
Description=Configure Network MTU and Jetson Clocks
# 네트워크가 온라인 상태가 된 후에 실행 (안전장치)
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
User=root
# 위에서 만든 스크립트 실행
ExecStart=/usr/local/bin/setup_hardware.sh
# 실행 후 서비스가 종료된 것으로 간주하지 않고 상태 유지 (선택사항)
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target

```

**저장 및 종료:** `Ctrl + O`, `Enter`, `Ctrl + X`

### 3단계: 서비스 등록 및 테스트

새로 만든 서비스를 시스템에 등록하고 활성화합니다.

```bash
# 1. systemd 데몬 재로드 (새 파일 인식)
sudo systemctl daemon-reload

# 2. 부팅 시 자동 실행 활성화
sudo systemctl enable custom-hardware-setup.service

# 3. (테스트) 지금 바로 실행해보기
sudo systemctl start custom-hardware-setup.service

```

### 4단계: 적용 확인

명령어가 잘 실행되었는지 확인하려면 다음을 체크하세요.

1. **MTU 확인:**
```bash
ip addr show eno1 | grep mtu
# 출력에 'mtu 9000'이 보여야 합니다.

```


2. **Jetson Clocks 확인:**
```bash
sudo jetson_clocks --show
# 팬이 돌거나 클럭이 최대치로 고정되어 있는지 확인

```



---

### 💡 Ubuntu 24.04 전문가의 조언 (Pro Tip)

작성해주신 명령어 방식도 작동하지만, Ubuntu 24.04에서는 네트워크 설정(MTU 등)을 **Netplan**으로 관리하는 것이 정석입니다. `ip link` 명령어로 강제 변경한 설정은 NetworkManager가 재시작되거나 케이블이 재연결될 때 초기화될 수 있기 때문입니다.

만약 위 스크립트 방식이 불안정하다면(MTU가 자꾸 1500으로 돌아간다면), `/etc/netplan/` 디렉토리의 설정 파일(예: `01-netcfg.yaml`)을 수정하여 영구적으로 적용하는 것을 권장합니다:

```yaml
network:
  version: 2
  ethernets:
    eno1:
      dhcp4: true
      mtu: 9000  # 여기서 설정하면 OS 레벨에서 영구 적용됨

```

하지만 `jetson_clocks` 실행과 10초 딜레이 요구사항이 있으므로, **현재 작성해드린 `systemd` 방식이 요청하신 사항을 가장 정확하게 수행**합니다.

이제 재부팅(`sudo reboot`)하여 10초 뒤에 설정이 적용되는지 확인해보세요.
##########################  
###########################  
#############################  
네, **가능합니다.** `crontab`의 `@reboot` 기능을 사용하면 부팅 시 명령어를 실행할 수 있습니다.

`systemd`보다 설정이 훨씬 **간단하다는 장점**이 있지만, 실행 순서나 의존성(네트워크가 준비되었는지 등)을 정교하게 제어하기 어렵다는 단점이 있습니다.

빠르게 설정하고 싶다면 아래 순서를 따르세요.

---

### Crontab을 이용한 설정 방법

**핵심:** 일반 사용자(`crontab -e`)가 아닌 **관리자 권한(`sudo crontab -e`)**으로 설정해야 합니다. `ip` 명령어와 `jetson_clocks`는 루트 권한이 필요하기 때문입니다.

#### 1. 루트 크론탭 편집기 열기

터미널에 다음을 입력합니다.

```bash
sudo crontab -e

```

(처음 실행라면 에디터를 선택하라고 나올 텐데, 가장 쉬운 `nano`를 선택하시면 됩니다.)

#### 2. 명령어 추가하기

파일의 맨 마지막 줄에 아래 **한 줄**을 그대로 복사해서 붙여넣으세요.
(명령어들을 `;`로 연결하여 순차 실행되게 하고, 절대 경로를 사용하여 에러를 방지합니다.)

```bash
@reboot sleep 10; /usr/sbin/ip link set dev eno1 down; /usr/sbin/ip link set dev eno1 mtu 9000; /usr/sbin/ip link set dev eno1 up; /usr/bin/jetson_clocks

```

> **주의:** `sleep 10`은 필수입니다. 크론탭은 부팅 초기 단계에 실행되는데, 이때 네트워크 장치(`eno1`)가 아직 OS에 인식되지 않았을 수 있기 때문입니다.

#### 3. 저장 및 종료

* Nano 에디터인 경우: `Ctrl + O` -> `Enter` -> `Ctrl + X`

---

### 🧐 전문가의 체크포인트 (Crontab 사용 시 주의사항)

Ubuntu 24.04 환경에서 `crontab`을 쓸 때 발생할 수 있는 문제와 해결책입니다.

**1. 경로(Path) 문제 (가장 흔한 실수)**
크론탭은 실행될 때 일반 터미널과 달리 환경변수(`$PATH`)를 거의 가지고 있지 않습니다. 그래서 그냥 `ip`라고만 쓰면 "command not found" 에러가 날 수 있습니다.

* **해결책:** 위 예시처럼 `/usr/sbin/ip`, `/usr/bin/jetson_clocks` 같이 **절대 경로**를 적어주는 것이 안전합니다.
* *경로 확인법:* 터미널에서 `which ip` 입력



**2. 실행 여부 확인의 어려움**
`systemd`는 `systemctl status`로 성공/실패를 바로 알 수 있지만, 크론탭은 로그를 따로 뒤져야 합니다.

* 만약 실행이 안 된다면 로그를 남기도록 줄을 수정해야 합니다:
```bash
@reboot sleep 10; ...명령어들... > /var/log/my_boot_job.log 2>&1

```



**3. 네트워크 매니저와의 충돌**
Ubuntu 데스크탑은 `NetworkManager`가 네트워크를 관리합니다. 크론탭으로 강제로 설정을 바꿔도, NetworkManager가 "어? 연결이 끊겼네?" 하고 다시 연결하면서 설정을 초기화(MTU 1500으로 복구)해버릴 수도 있습니다.

* 이 경우 앞서 추천드린 `systemd` 방식이나 `netplan` 설정이 훨씬 견고합니다.

### 요약

* **간단함:** Crontab 승 (`@reboot` 한 줄이면 끝)
* **안정성:** Systemd 승 (네트워크 서비스가 켜진 것을 확실히 확인 후 실행함)

일단 `crontab`으로 시도해 보시고, 혹시 MTU가 자꾸 풀린다면 그때 `systemd` 방식으로 넘어가셔도 좋습니다.
####################  
######################  
########################  
네, **물론 가능합니다.**

`systemd` 서비스 파일에서 스크립트의 **절대 경로(Absolute Path)**만 정확하게 지정해주면, 파일이 `/home` 디렉토리 안의 어디에 있든 상관없이 실행됩니다.

개발자분들은 보통 Git으로 관리하는 워크스페이스(예: `/home/user/workspace/robot_ws/scripts/`)에 두는 것을 선호하는데, 이렇게 하면 스크립트 수정이나 버전 관리가 훨씬 편해집니다.

다만, 몇 가지 **중요한 주의사항**이 있습니다.

---

### 1. 서비스 파일 수정 방법

만약 스크립트 위치가 `/home/내아이디/my_scripts/setup_hardware.sh`라면, 서비스 파일을 아래와 같이 수정해야 합니다.

```ini
[Service]
Type=oneshot
User=root
# 중요: 절대 경로를 처음부터 끝까지 다 적어야 합니다.
ExecStart=/home/내아이디/my_scripts/setup_hardware.sh
RemainAfterExit=yes

```

### 2. 주의사항 (흔히 하는 실수)

1. **`~` (물결표시) 사용 금지**
* 터미널에서는 `~/my_scripts/setup.sh`라고 하면 알아듣지만, `systemd` 설정 파일에서는 `~`를 인식하지 못합니다.
* 반드시 `/home/username/...` 형태로 **전체 경로**를 적어주세요.


2. **권한 확인**
* 홈 디렉토리에 파일을 만들었더라도 `chmod +x` 실행 권한은 반드시 주어야 합니다.
* `systemd`는 `root` 권한으로 실행되므로, 홈 디렉토리 파일이라도 읽고 실행하는 데는 권한 문제가 없습니다.


3. **파일 삭제 주의**
* `/usr/local/bin` 같은 시스템 폴더는 실수로 지울 일이 적지만, 홈 디렉토리는 정리하다가 스크립트가 담긴 폴더를 실수로 삭제하거나 옮길 수 있습니다. 이 경우 부팅 시 서비스가 실패(Fail)하게 됩니다.



### 요약: 추천하는 방식

질문하신 내용은 **로봇 개발 환경에서 매우 권장되는 방식**입니다.

스크립트를 본인의 Git 레포지토리(예: `/home/robot_dev/src/mobile_robot/scripts/boot_setup.sh`) 내부에 두고, `systemd`가 거기를 바라보게 설정하세요. 그러면 하드웨어 설정값도 Git으로 버전 관리가 되어 나중에 추적하기 훨씬 좋습니다.
