## ssh 접속 가능
개인 PC - PC1  
PC1 - PC0  
PC1 - PC2  
  

## 전체 연결 흐름(case1)  
개발 환경 접속 경로:개인 PC ➔ PC1_1 (Ubuntu)  
Git 통신 경로 (PC2_1 내부에서 발생):PC1_1 (Ubuntu) ➔ PC0 (Ubuntu) ➔ 사내 Git 서버  


## 전체 연결 흐름(case2)  
개발 환경 접속 경로:개인 PC ➔ PC1_2 (Win10) ➔ PC2_2 (Ubuntu)  
Git 통신 경로 (PC2_2 내부에서 발생):PC2_2 (Ubuntu) ➔ PC1_2 (Win10) ➔ PC0 (Ubuntu) ➔ 사내 Git 서버  

##############################################################3  
네, 알겠습니다. 두 가지 케이스를 모두 자유자재로 오가며 사용할 수 있도록, 모든 설정을 통합한 완벽한 Step-by-Step 가이드를 만들어 드리겠습니다. 이 가이드 하나만 있으면 어떤 상황에서든 원활하게 작업하실 수 있습니다.

HTTPS 주소를 사용하시므로, \*\*SSH 포트 포워딩(터널링)\*\*을 중심으로 설정하겠습니다.

-----

### \#\# ⚙️ 1단계: 개인 PC 설정 (모든 케이스 통합)

먼저 \*\*"개인 PC"\*\*에서 `~/.ssh/config` 파일을 열어 아래 내용을 모두 추가합니다. 이 파일 하나로 Case 1과 Case 2의 접속을 모두 관리할 수 있습니다.

  * **파일 위치:** 개인 PC의 `~/.ssh/config` (파일이 없다면 새로 만드세요)

<!-- end list -->

```ini
# =========================================================
# 💡 기본 정보: 점프 호스트 PC들
# =========================================================
# Case 2에서 사용될 첫 번째 점프 호스트
Host jump-pc1_2
    HostName <PC1_2_Win10의_IP_주소>
    User <PC1_2_접속_사용자명>

# =========================================================
# 🚀 최종 목적지: 실제 개발 환경들
# =========================================================

# --- Case 1 접속 설정: dev-case1 ---
Host dev-case1
    HostName <PC1_1_Ubuntu의_IP_주소>
    User <PC1_1_접속_사용자명>
    # Case 1 작업 시 내 Git 정보를 자동으로 설정
    SetEnv GIT_AUTHOR_NAME="Your Name"
    SetEnv GIT_AUTHOR_EMAIL="your.email@company.com"

# --- Case 2 접속 설정: dev-case2 ---
Host dev-case2
    HostName <PC2_2_Ubuntu의_IP_주소>
    User <PC2_2_접속_사용자명>
    # 💡 PC1_2(jump-pc1_2)를 경유해서 접속
    ProxyJump jump-pc1_2
    # Case 2 작업 시 내 Git 정보를 자동으로 설정
    SetEnv GIT_AUTHOR_NAME="Your Name"
    SetEnv GIT_AUTHOR_EMAIL="your.email@company.com"
```

**[설정 설명]**

  * 이 설정으로 `ssh dev-case1` 명령어를 실행하면 **PC1\_1**으로, `ssh dev-case2`를 실행하면 **PC1\_2를 거쳐 PC2\_2**로 접속됩니다.
  * 어떤 환경에 접속하든, `SetEnv` 설정 덕분에 Git 커밋 시 항상 **내 이름과 이메일**이 자동으로 기록됩니다.
  * `<...>` 안의 내용과 `Your Name`, `your.email@company.com`은 실제 환경과 본인 정보에 맞게 반드시 수정해야 합니다.

-----

### \#\# ⚙️ 2단계: 서버 환경 설정 (관리자 최초 1회)

이제 각 개발 서버가 사내 Git 서버와 통신할 수 있도록 설정합니다. 각 서버의 관리자가 최초 1회만 설정해주면 됩니다.

#### **A. Case 1 서버 설정 (`PC1_1`)**

1.  **Git 'insteadOf' 설정 (HTTPS 주소 자동 변환)**

      * `PC1_1`에 접속해서 아래 명령어를 실행합니다. `https://github.company.net/`으로 시작하는 모든 Git 요청을 로컬 터널(`localhost:8443`)로 보내도록 설정합니다.

    <!-- end list -->

    ```bash
    # PC1_1에서 실행
    sudo git config --system url."https://localhost:8443/".insteadOf "https://github.company.net/"
    ```
    ```bash
    # PC1_1에서 실행(unset)
    # 이전에 추가했던 url.insteadOf 규칙들을 모두 제거합니다.
    sudo git config --system --unset-all url.https://localhost:8443/.insteadof
    sudo git config --system --unset-all url.https://localhost:8443/github.com/.insteadof
    ```    

2.  **SSH 서버 설정 (`AcceptEnv`)**

      * 개인 PC에서 보낸 Git 사용자 정보(`GIT_*`)를 서버가 받도록 허용합니다.

    <!-- end list -->

    ```bash
    # PC1_1에서 실행
    sudo sed -i 's/^AcceptEnv LANG LC_\*/AcceptEnv LANG LC_* GIT_*/g' /etc/ssh/sshd_config
    sudo systemctl restart ssh
    ```

#### **B. Case 2 서버 설정 (`PC2_2`)**

1.  **Git 'insteadOf' 설정 (HTTPS 주소 자동 변환)**

      * `PC2_2`에 접속해서 아래 명령어를 실행합니다. 기능은 위와 동일합니다.

    <!-- end list -->

    ```bash
    # PC2_2에서 실행
    sudo git config --system url."https://localhost:8443/".insteadOf "https://github.company.net/"
    ```

2.  **SSH 서버 설정 (`AcceptEnv`)**

      * 마찬가지로 `PC2_2`가 Git 사용자 정보를 받도록 허용합니다.

    <!-- end list -->

    ```bash
    # PC2_2에서 실행
    sudo sed -i 's/^AcceptEnv LANG LC_\*/AcceptEnv LANG LC_* GIT_*/g' /etc/ssh/sshd_config
    sudo systemctl restart ssh
    ```

-----

### \#\# 🚀 3단계: 실제 작업 흐름 (개발자 매일 사용)

이제 모든 설정이 끝났습니다. 상황에 따라 아래의 작업 흐름을 따르기만 하면 됩니다.

#### **Case 1: `PC1_1`에서 작업할 때**

1.  **`PC1_1`에 접속**

      * 개인 PC 터미널에서 아래 명령어를 실행하거나, VS Code Remote-SSH로 `dev-case1`에 접속합니다.
        ```bash
        ssh dev-case1
        ```

2.  **SSH 터널 열기**

      * `PC1_1`에 접속된 상태에서, **새 터미널 창**을 열고 아래 명령어를 실행해 사내 Git 서버로 가는 터널을 엽니다. 이 터널은 Git 작업을 하는 동안 계속 켜두어야 합니다.
        ```bash
        # PC1_1에서 실행
        # PC0을 경유하여 github.company.net으로 터널링
        ssh -N -L 8443:github.company.net:443 <PC0_사용자명>@<PC0_IP>
        ```

3.  **Git 작업 수행**

      * **또 다른 터미널 창**에서 평소처럼 Git 명령어를 사용하면 됩니다. `insteadOf` 설정 덕분에 Git이 자동으로 터널을 통해 통신합니다.
        ```bash
        # PC1_1에서 실행
        git clone https://github.company.net/org/git-repo.git
        cd git-repo
        # ... 코드 수정 ...
        git commit -m "feat: amazing feature"
        git push
        ```

#### **Case 2: `PC2_2`에서 작업할 때**

1.  **`PC2_2`에 접속**

      * 개인 PC 터미널에서 아래 명령어를 실행하거나, VS Code Remote-SSH로 `dev-case2`에 접속합니다.
        ```bash
        ssh dev-case2
        ```

2.  **SSH 터널 열기**

      * `PC2_2`에 접속된 상태에서, **새 터미널 창**을 열고 아래 명령어를 실행해 사내 Git 서버로 가는 **다중 홉 터널**을 엽니다.
        ```bash
        # PC2_2에서 실행
        # PC1_2와 PC0을 순차적으로 경유하여 터널링
        ssh -N -L 8443:github.company.net:443 -o "ProxyJump <PC1_2_사용자명>@<PC1_2_IP>" <PC0_사용자명>@<PC0_IP>
        ```
      * **팁:** 이 명령어가 너무 길다면, `PC2_2`의 `~/.ssh/config`에 `PC0`으로 가는 경로를 미리 정의해두면 더 간단하게 만들 수 있습니다.

3.  **Git 작업 수행**

      * 마찬가지로 **또 다른 터미널 창**에서 평소처럼 Git 명령어를 사용하면 됩니다.
        ```bash
        # PC2_2에서 실행
        git clone https://github.company.net/org/git-repo.git
        cd git-repo
        # ... 코드 수정 ...
        git commit -m "fix: critical bug"
        git push
        ```

이 가이드만 있으면, 이제 어떤 복잡한 네트워크 환경에서도 고민 없이 개발에만 집중하실 수 있습니다.
