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
