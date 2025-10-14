안녕하세요\! SSH 전문가로서 명확하고 단계별로 목적을 달성하는 방법을 알려드리겠습니다.

현재 네트워크 구성은 PC0에서 PC2나 PC3로 직접 통신할 수 없는, 여러 네트워크를 거쳐야 하는 **'멀티-홉(Multi-Hop)'** 환경입니다. 마치 비행기를 갈아타는 것처럼 SSH 연결도 중간 지점(점프 호스트)을 거쳐 최종 목적지까지 가야 합니다.

가장 효율적이고 현대적인 방법은 SSH의 **`ProxyJump`** 기능을 사용하는 것입니다. 이 기능을 사용하면 터미널이나 VS Code에서 단 한 번의 명령으로 중간 과정을 자동으로 처리하여 최종 목적지에 접속할 수 있습니다.

-----

### **사전 준비: 필요한 정보 확인**

시작하기 전에 각 PC의 사용자 이름과 IP 주소를 알아야 합니다.

  * **PC1 (Windows)**:
      * IP 주소: `명령 프롬프트(cmd)`에서 `ipconfig` 실행 후 확인
      * 사용자 이름: Windows 로그인 시 사용하는 사용자 이름
  * **PC2 (Ubuntu 24.04)**:
      * IP 주소: 터미널에서 `ip a` 실행 후 확인
      * 사용자 이름: Ubuntu 로그인 시 사용하는 사용자 이름
  * **PC3 (Ubuntu 22.04)**:
      * IP 주소: 터미널에서 `ip a` 실행 후 확인
      * 사용자 이름: Ubuntu 로그인 시 사용하는 사용자 이름

> **🚨 중요\! Windows PC1 설정**
> PC0에서 PC1을 경유지로 사용하려면 **PC1(Windows)에 OpenSSH 서버가 설치 및 실행 중이어야 합니다.**
>
> 1.  **설치**: `설정` \> `시스템` \> `선택적 기능` \> `기능 추가`에서 "OpenSSH 서버"를 찾아 설치하세요.
> 2.  **실행**: `서비스(services.msc)`를 열어 `OpenSSH SSH Server`를 찾아 시작하고, `시작 유형`을 `자동`으로 변경하는 것을 권장합니다.
> 3.  **방화벽**: Windows 방화벽에서 SSH 포트(기본값 22)가 열려있는지 확인하세요.

-----

### **Step 1: PC0에서 PC2로 접속하기 (Single-Hop)**

PC0에서 PC1을 경유하여 PC2로 접속하는 설정입니다.

#### **1. PC0에 SSH 설정 파일 생성 및 수정**

PC0의 터미널에서 `~/.ssh/config` 파일을 열어 아래 내용을 추가합니다. 파일이 없다면 새로 생성됩니다.

```bash
code ~/.ssh/config
```

아래 내용을 자신의 환경에 맞게 수정하여 파일에 추가하고 저장하세요.

```ini
# ======================================================
# 경유지 PC1 (Windows)
# ======================================================
Host pc1-jump
    HostName <pc1_ip_address>
    User <pc1_windows_username>

# ======================================================
# 최종 목적지 PC2 (PC1을 통해 점프)
# ======================================================
Host pc2-final
    HostName <pc2_ip_address>
    User <pc2_ubuntu_username>
    ProxyJump pc1-jump
```

  * `<pc1_ip_address>`: PC1의 IP 주소로 변경
  * `<pc1_windows_username>`: PC1의 Windows 사용자 이름으로 변경
  * `<pc2_ip_address>`: PC2의 IP 주소로 변경
  * `<pc2_ubuntu_username>`: PC2의 Ubuntu 사용자 이름으로 변경
  * `Host`: SSH 접속 시 사용할 별칭입니다. `pc1-jump`, `pc2-final` 등 원하는 이름으로 지정할 수 있습니다.
  * `ProxyJump`: `pc1-jump`라는 별칭의 호스트를 경유지로 사용하겠다는 의미입니다.

#### **2. 터미널에서 접속**

이제 PC0의 터미널에서 아래 명령어를 입력하면, PC1을 거쳐 PC2로 바로 접속됩니다. 중간에 PC1과 PC2의 비밀번호를 순서대로 물어볼 것입니다.

```bash
ssh pc2-final
```

#### **3. VS Code에서 접속**

1.  VS Code에서 `Remote - SSH` 익스텐션을 설치합니다.
2.  `F1` 키를 누르고 `Remote-SSH: Connect to Host...`를 선택합니다.
3.  방금 `~/.ssh/config` 파일에 추가한 `pc2-final` 호스트가 목록에 나타납니다. 이를 선택하면 됩니다.
4.  새로운 VS Code 창이 열리면서 PC2에 원격으로 연결됩니다.

-----

### **Step 2: PC0에서 PC3로 접속하기 (Double-Hop)**

PC0에서 PC1을 거치고, 다시 PC2를 거쳐 최종 목적지인 PC3로 접속하는 설정입니다. 비행기를 두 번 갈아타는 것과 같습니다.

#### **1. PC0의 SSH 설정 파일에 내용 추가**

위에서 수정했던 PC0의 `~/.ssh/config` 파일을 다시 열어 아래 내용을 추가합니다.

```ini
# (기존 pc1-jump, pc2-final 설정은 그대로 둡니다)

# ======================================================
# 두 번째 경유지 PC2 (별도 접속용)
# PC3로 가기 위한 경유지로 사용됩니다.
# ======================================================
Host pc2-jump
    HostName <pc2_ip_address>
    User <pc2_ubuntu_username>
    # 이 호스트는 PC1을 통해 접근합니다.
    ProxyJump pc1-jump

# ======================================================
# 최종 목적지 PC3 (PC1 -> PC2를 통해 이중 점프)
# ======================================================
Host pc3-final
    HostName <pc3_ip_address>
    User <pc3_ubuntu_username>
    ProxyJump pc2-jump
```

  * `<pc3_ip_address>`: PC3의 IP 주소로 변경
  * `<pc3_ubuntu_username>`: PC3의 Ubuntu 사용자 이름으로 변경
  * **작동 원리**:
    1.  `pc3-final`에 접속을 시도하면 `ProxyJump pc2-jump` 설정에 따라 `pc2-jump` 호스트로 먼저 연결을 시도합니다.
    2.  `pc2-jump`는 다시 `ProxyJump pc1-jump` 설정에 따라 `pc1-jump`를 경유합니다.
    3.  결과적으로 `PC0 -> PC1 -> PC2 -> PC3` 순서로 연결이 자동으로 이루어집니다.

#### **2. 터미널에서 접속**

PC0의 터미널에서 아래 명령어를 입력하면 PC1, PC2, PC3의 비밀번호를 순서대로 물어본 후 PC3에 최종적으로 접속됩니다.

```bash
ssh pc3-final
```

#### **3. VS Code에서 접속**

PC2 접속과 동일합니다. `F1` \> `Remote-SSH: Connect to Host...`를 선택하면 `pc3-final` 호스트가 목록에 보일 것입니다. 이를 선택하면 PC3으로 바로 원격 개발 환경이 구성됩니다.

-----

### **요약 및 문제 해결**

  * **핵심**: PC0의 `~/.ssh/config` 파일에 모든 경로 정보를 `ProxyJump`를 이용해 "레시피"처럼 작성해두는 것입니다.
  * **비밀번호 없는 접속**: 매번 비밀번호를 입력하는 것이 번거롭다면 `ssh-keygen`과 `ssh-copy-id`를 이용해 각 PC 간에 SSH 공개키를 미리 복사해두면 비밀번호 입력 없이 접속할 수 있습니다.
  * **연결 실패 시**:
    1.  `ssh -v pc2-final` 처럼 `-v`(verbose) 옵션을 붙여서 어느 단계에서 오류가 발생하는지 확인하세요.
    2.  각 PC의 IP 주소와 사용자 이름이 정확한지 다시 확인하세요.
    3.  중간 경유지(PC1, PC2)의 방화벽이 SSH 연결(포트 22)을 막고 있지 않은지 확인하세요.

이 가이드가 원하는 목표를 달성하는 데 도움이 되기를 바랍니다\! 🚀

####################  

네, 물론입니다. Windows에서 OpenSSH 서버를 설치하고 외부에서 접속할 수 있도록 설정하는 방법을 GUI(그래픽 인터페이스) 방식과 CLI(명령줄) 방식으로 나누어 더 자세히 알려드릴게요.

-----

### **방법 1: GUI (설정 앱)를 이용한 설치 및 설정**

가장 쉽고 직관적인 방법입니다.

#### **Step 1: OpenSSH 서버 기능 설치**

1.  **설정 열기**: `Win` 키 + `I` 키를 눌러 Windows 설정을 엽니다.

2.  **앱 메뉴로 이동**: 왼쪽 메뉴에서 `앱`을 선택한 후, `선택적 기능`을 클릭합니다.

3.  **기능 추가**: `설치된 기능` 목록 위쪽에 있는 `기능 보기` 버튼을 클릭합니다.

4.  **OpenSSH 서버 검색 및 설치**: 검색창에 "OpenSSH"를 입력하면 `OpenSSH 서버`가 나타납니다. 체크박스를 선택하고 `다음` \> `설치` 버튼을 누릅니다. 설치가 완료될 때까지 잠시 기다립니다.

#### **Step 2: OpenSSH 서비스 시작 및 자동 실행 설정**

설치만 해서는 안 되고, SSH 서버 프로그램(서비스)이 항상 실행되도록 설정해야 합니다.

1.  **서비스 앱 열기**: `Win` 키 + `R` 키를 눌러 실행창을 열고 `services.msc`를 입력한 후 엔터를 누릅니다.
2.  **서비스 찾기**: 서비스 목록에서 `OpenSSH SSH Server`를 찾습니다.
3.  **서비스 속성 열기**: `OpenSSH SSH Server`를 더블클릭하여 속성 창을 엽니다.
4.  **자동 시작 설정**:
      * **시작 유형**: 드롭다운 메뉴에서 `자동`으로 변경합니다. 이렇게 하면 컴퓨터를 켤 때마다 SSH 서버가 자동으로 실행됩니다.
      * **서비스 시작**: `서비스 상태` 아래의 `시작` 버튼을 클릭하여 지금 바로 서비스를 실행합니다.
      * `적용` \> `확인` 버튼을 눌러 저장합니다.

#### **Step 3: Windows 방화벽 설정**

외부 PC(PC0)가 내 Windows PC(PC1)의 SSH 포트로 접근할 수 있도록 방화벽을 열어줘야 합니다.

1.  **방화벽 설정 열기**: `Win` 키를 누르고 "Windows Defender 방화벽"을 검색하여 실행합니다.
2.  **고급 설정으로 이동**: 왼쪽 메뉴에서 `고급 설정`을 클릭합니다.
3.  **인바운드 규칙 확인**: 왼쪽에서 `인바운드 규칙`을 클릭합니다. 규칙 목록에서 `OpenSSH SSH Server (sshd)`라는 이름의 규칙이 이미 생성되어 있고, `프로필`이 `공용(Public)` 또는 `개인(Private)`으로 설정된 상태에서 녹색 체크 표시로 \*\*'허용'\*\*되어 있는지 확인합니다.
      * **만약 규칙이 없다면?** OpenSSH 서버 설치 시 보통 자동으로 생성됩니다. 만약 없다면 `새 규칙` \> `포트` \> `TCP` 선택, `특정 로컬 포트`에 `22` 입력 \> `연결 허용` \> 모든 프로필(도메인, 개인, 공용) 체크 \> 이름(`SSH` 등)을 지정하여 규칙을 직접 만들어주세요.

이제 GUI를 이용한 모든 설정이 끝났습니다. PC1의 IP 주소와 Windows 사용자 계정으로 다른 PC에서 SSH 접속을 시도할 수 있습니다.

-----

### **방법 2: CLI (PowerShell)를 이용한 설치 및 설정**

명령어에 익숙하다면 더 빠르게 설정할 수 있습니다.

#### **Step 1: PowerShell을 관리자 권한으로 실행**

`Win` 키를 누르고 "PowerShell"을 검색한 뒤, `Windows PowerShell` 앱에 마우스 오른쪽 버튼을 클릭하여 `관리자 권한으로 실행`을 선택합니다.

#### **Step 2: OpenSSH 서버 설치**

아래 명령어를 입력하여 현재 OpenSSH 서버가 설치되어 있는지 확인합니다.

```powershell
Get-WindowsCapability -Online | Where-Object Name -like 'OpenSSH.Server*'
```

`State`가 `NotPresent`라고 나오면 설치가 필요합니다. 아래 명령어로 설치를 진행하세요.

```powershell
Add-WindowsCapability -Online -Name OpenSSH.Server~~~~0.0.1.0
```

#### **Step 3: OpenSSH 서비스 시작 및 자동 실행 설정**

아래 명령어를 순서대로 입력하여 서비스를 시작하고, 부팅 시 자동으로 실행되도록 설정합니다.

```powershell
# 서비스 시작
Start-Service sshd

# 서비스를 자동 시작 유형으로 설정
Set-Service -Name sshd -StartupType 'Automatic'
```

#### **Step 4: Windows 방화벽 규칙 확인 및 추가**

아래 명령어로 방화벽 규칙이 이미 존재하는지 확인합니다.

```powershell
Get-NetFirewallRule -Name "OpenSSH-Server-In-TCP"
```

만약 규칙이 없거나 `Enabled`가 `False`로 나온다면, 아래 명령어로 새로운 방화벽 규칙을 추가하고 활성화합니다. 이 규칙은 TCP 포트 22번으로 들어오는 모든 연결을 허용합니다.

```powershell
New-NetFirewallRule -Name "OpenSSH-Server-In-TCP" -DisplayName "OpenSSH Server (sshd)" -Protocol TCP -LocalPort 22 -Action Allow -Enabled True
```

이제 PowerShell을 이용한 모든 설정이 완료되었습니다.

-----

### **마지막 확인 사항**

  * **IP 주소 확인**: PC1(Windows)의 `명령 프롬프트(cmd)`나 `PowerShell`에서 `ipconfig` 명령어를 실행하여 외부에서 접속할 때 사용해야 할 **IP 주소**(예: `192.168.0.5`)를 확인하세요.
  * **사용자 계정**: 접속 시 사용할 사용자 이름은 Windows 로그인 시 사용하는 계정 이름입니다 (예: `ssh myuser@192.168.0.5`).


#####################  


물론입니다\! 매번 비밀번호를 입력하는 것은 번거롭고 자동화 스크립트를 방해하기도 합니다. SSH 키(Key)를 사용하면 이 문제를 아주 안전하고 편리하게 해결할 수 있습니다.

**핵심 원리**는 '열쇠와 자물쇠' 방식입니다.

  * **비밀 키 (Private Key)**: 나만 가지고 있는 '열쇠'입니다. 절대 외부에 노출되면 안 됩니다. (PC0에 저장)
  * **공개 키 (Public Key)**: 내가 접속하고 싶은 서버에 미리 가져다 두는 '자물쇠'입니다. (PC1, PC2, PC3에 복사)

이제 PC0에서 PC1, PC2, PC3로 접속할 때 비밀번호 없이 로그인하는 완벽한 방법을 단계별로 알려드리겠습니다. 모든 작업은 **출발지인 PC0의 터미널**에서 시작합니다.

-----

### **Step 1: PC0에서 SSH 키(열쇠) 생성하기**

먼저 PC0에 나만의 '열쇠'(비밀 키)와 '자물쇠'(공개 키) 한 쌍을 만듭니다.

1.  PC0의 터미널을 엽니다.

2.  아래 명령어를 입력하고 엔터를 누릅니다.

    ```bash
    ssh-keygen -t rsa -b 4096
    ```

      * `-t rsa`: RSA 암호화 방식을 사용합니다.
      * `-b 4096`: 4096비트의 강력한 암호화 키를 생성합니다.

3.  명령어를 실행하면 몇 가지 질문이 나옵니다. **모두 그냥 엔터**를 눌러 기본값으로 진행하는 것이 가장 간단합니다.

      * `Enter file in which to save the key...`: 키 저장 위치를 묻습니다. 그냥 엔터를 누르면 `~/.ssh/id_rsa` 경로에 비밀 키가, `~/.ssh/id_rsa.pub` 경로에 공개 키가 저장됩니다.
      * `Enter passphrase (empty for no passphrase):`: 키를 위한 추가 비밀번호(암호 구문)를 설정할지 묻습니다. **비밀번호 없는 접속이 목적이므로, 아무것도 입력하지 말고 그냥 엔터를 누릅니다.**
      * `Enter same passphrase again:`: 확인 질문입니다. 다시 한번 엔터를 누릅니다.

    이제 `~/.ssh/` 디렉터리에 `id_rsa`(비밀 키)와 `id_rsa.pub`(공개 키) 파일이 성공적으로 생성되었습니다.

-----

### **Step 2: 각 PC에 공개 키(자물쇠) 복사하기**

이제 생성된 공개 키(`id_rsa.pub`)를 접속하려는 모든 서버(PC1, PC2, PC3)에 복사해서 '자물쇠'를 설치해야 합니다.

#### **2-1. PC2와 PC3 (Ubuntu)에 공개 키 복사하기**

Ubuntu(Linux)는 `ssh-copy-id`라는 매우 편리한 명령어를 제공합니다. 이전에 `~/.ssh/config` 파일에 설정해 둔 별칭(`pc2-final`, `pc3-final`)을 그대로 사용할 수 있습니다.

1.  **PC2에 공개 키 복사**: PC0 터미널에 아래 명령어를 입력합니다.

    ```bash
    ssh-copy-id pc2-final
    ```

      * `pc2-final`의 비밀번호를 **마지막으로 한 번** 물어볼 것입니다. 정확하게 입력하면 공개 키가 PC2의 `~/.ssh/authorized_keys` 파일에 자동으로 추가됩니다.

2.  **PC3에 공개 키 복사**: PC0 터미널에 아래 명령어를 입력합니다.

    ```bash
    ssh-copy-id pc3-final
    ```

      * `pc3-final`에 접속하기 위한 경로(PC1 -\> PC2 -\> PC3)의 비밀번호들을 순서대로 **마지막으로 한 번** 물어봅니다. 모두 정확하게 입력하면 PC3에도 공개 키가 자동으로 설치됩니다.

#### **2-2. PC1 (Windows)에 공개 키 복사하기**

Windows는 `ssh-copy-id`를 지원하지 않으므로, 조금 수동적인 방법이 필요하지만 어렵지 않습니다.

1.  **PC0에서 내 공개 키 내용 확인 및 복사**: PC0 터미널에 아래 명령어를 입력하여 공개 키 파일의 내용을 화면에 출력합니다.

    ```bash
    cat ~/.ssh/id_rsa.pub
    ```

      * `ssh-rsa AAAA...` 로 시작해서 `... user@pc0` 로 끝나는 긴 텍스트가 나타납니다. 이 텍스트 전체를 마우스로 드래그하여 복사합니다 (`Ctrl`+`Shift`+`C`).

2.  **PC1(Windows)에 비밀번호로 SSH 접속**: PC0 터미널에서 기존 방식대로 PC1에 접속합니다.

    ```bash
    ssh pc1-jump
    ```

3.  **PC1(Windows)에 공개 키 붙여넣기**: PC1에 접속된 터미널(PowerShell 또는 CMD)에서 아래 명령어를 **한 줄씩** 실행하여 복사한 키를 `authorized_keys` 파일에 추가합니다.

    ```powershell
    # .ssh 폴더가 없으면 생성합니다.
    mkdir C:\Users\<pc1_windows_username>\.ssh\ -ErrorAction SilentlyContinue

    # 복사한 공개 키를 authorized_keys 파일에 추가합니다.
    # '<복사한_공개키_내용>' 부분에 마우스 오른쪽 클릭 등으로 붙여넣기 하세요.
    Add-Content -Path C:\Users\<pc1_windows_username>\.ssh\authorized_keys -Value "<복사한_공개키_내용>"
    ```

      * `<pc1_windows_username>` 부분은 실제 본인의 Windows 사용자 이름으로 바꿔주세요.
      * `<복사한_공개키_내용>` 부분에 1번 단계에서 복사했던 `ssh-rsa AAAA...` 텍스트를 붙여넣으면 됩니다. **반드시 큰따옴표("")로 감싸야 합니다.**
      * 마지막으로 `exit`를 입력하여 PC1과의 접속을 종료합니다.

-----

### **Step 3: 비밀번호 없는 접속 테스트하기**

모든 설정이 끝났습니다\! 이제 PC0에서 각 PC로 접속을 시도하여 비밀번호를 묻지 않고 바로 로그인되는지 확인합니다.

```bash
# PC1 접속 테스트
ssh pc1-jump

# PC1 접속 종료
exit

# PC2 접속 테스트
ssh pc2-final

# PC2 접속 종료
exit

# PC3 접속 테스트
ssh pc3-final

# PC3 접속 종료
exit
```

위 명령들을 실행했을 때, 어떤 비밀번호도 묻지 않고 각 PC의 셸 프롬프트가 바로 나타난다면 완벽하게 성공한 것입니다. VS Code의 `Remote - SSH` 기능 또한 이 설정을 그대로 사용하여 비밀번호 없이 접속할 수 있습니다. 🥳  


https://seanshkim.tistory.com/m/62
