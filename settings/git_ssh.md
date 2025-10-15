네, Git과 네트워크 전문가로서 명쾌한 해결책을 드리겠습니다.

PC1을 **'점프 호스트(Jump Host)'** 또는 '징검다리'로 사용하여 PC2가 사내망에 있는 것처럼 사내 Git 서버에 접근하게 만드는 것이 가장 깔끔하고 안전한 방법입니다. **SSH의 `ProxyJump` 기능**을 사용하면 별도의 터널링 명령어를 계속 입력할 필요 없이 자동으로 연결을 중계해줘서 매우 편리합니다.

아래 단계를 차근차근 따라 해 보세요.

-----

### **1단계: 사전 정보 확인**

먼저 PC2에서 설정을 진행하기 위해 필요한 정보 3가지를 확인해야 합니다.

1.  **PC1의 로컬 네트워크 IP 주소**

      * PC1 터미널에서 아래 명령어를 입력하여 로컬 무선 네트워크 IP를 확인합니다. (보통 `192.168.x.x` 와 같은 형태입니다.)
        ```bash
        hostname -I
        ```
      * 예시 결과: `192.168.1.100`

2.  **PC1의 사용자 이름**

      * PC1에 로그인할 때 사용하는 사용자 계정 이름입니다.
      * 예시: `ubuntu-pc1`

3.  **사내 Git 서버의 주소 (Hostname)**

      * `git clone` 할 때 사용하는 주소에서 `@` 뒷부분입니다.
      * 예시: `git.my-company.com`

-----

### **2단계: PC2에 SSH 설정 구성하기 (핵심 단계)**

PC2에서 SSH 클라이언트 설정을 수정하여, 사내 Git 서버로 접속을 시도할 때 자동으로 PC1을 거쳐가도록 설정합니다.

1.  PC2에서 터미널을 열고 `~/.ssh/config` 파일을 엽니다. 파일이 없다면 새로 생성됩니다.

    ```bash
    nano ~/.ssh/config
    ```

2.  열린 파일에 아래 내용을 추가합니다. 위에서 확인한 정보를 각 항목에 맞게 수정하여 입력하세요.

    ```ini
    # 사내 Git 서버 접속을 위한 별칭 및 점프 설정
    Host corp-git
      HostName git.my-company.com
      User git
      ProxyJump ubuntu-pc1@192.168.1.100
    ```

      * **`Host corp-git`**: 사내 Git 서버를 부를 **별칭**입니다. `my-git` 등 원하는 이름으로 자유롭게 정할 수 있습니다.
      * **`HostName git.my-company.com`**: **실제 사내 Git 서버의 주소**를 정확히 입력합니다.
      * **`User git`**: Git 서버에 접속할 때 사용하는 사용자 이름입니다. 대부분 `git`을 사용합니다.
      * **`ProxyJump ubuntu-pc1@192.168.1.100`**: **가장 중요한 부분**입니다. `<PC1 사용자 이름>@<PC1 IP 주소>` 형식으로 입력하여 PC1을 점프 호스트로 지정합니다.

3.  파일을 저장하고 닫습니다. (`Ctrl+X` -\> `Y` -\> `Enter`)

4.  설정 파일의 권한을 변경하여 안전하게 보호합니다.

    ```bash
    chmod 600 ~/.ssh/config
    ```

**💡 잠깐\! SSH 키 설정**
만약 PC2에서 PC1으로, 그리고 PC1에서 사내 Git 서버로 접속할 때 비밀번호를 매번 입력하는 것이 번거롭다면, PC2의 SSH 공개키(`~/.ssh/id_rsa.pub`)를 PC1의 `~/.ssh/authorized_keys` 파일과 사내 Git 서버에 미리 등록해두는 것을 강력히 추천합니다.

-----

### **3단계: PC2에서 사내 Git 사용하기**

이제 모든 설정이 끝났습니다. PC2에서 Git 명령어를 사용할 때, 실제 서버 주소 대신 \*\*2단계에서 지정한 별칭(`corp-git`)\*\*을 사용하기만 하면 됩니다.

#### **새로운 저장소 `clone` 받기**

  * **원래 PC1에서 사용하던 명령어:**
    ```bash
    git clone git@git.my-company.com:my-project/my-repo.git
    ```
  * **이제 PC2에서 사용할 명령어:**
    ```bash
    git clone corp-git:my-project/my-repo.git
    ```
    (`git@` 부분을 제거하고 서버 주소를 별칭으로 바꾼 것에 주목하세요.)

#### **기존에 `clone` 받아둔 저장소의 원격 주소 변경하기**

이미 프로젝트를 가지고 있다면, 원격 저장소(remote)의 주소만 변경해주면 됩니다.

1.  해당 프로젝트 폴더로 이동합니다.

    ```bash
    cd my-repo
    ```

2.  현재 설정된 원격 주소를 확인합니다.

    ```bash
    git remote -v
    ```

3.  `origin` 이라는 이름의 원격 주소를 새로운 별칭을 사용하도록 변경합니다.

    ```bash
    git remote set-url origin corp-git:my-project/my-repo.git
    ```

이제 `git pull`, `git push`, `git fetch` 등 모든 Git 명령어가 PC1을 통해 사내망과 자동으로 통신하며 완벽하게 동작할 것입니다.

-----

### **🤔 만약 사내 Git이 HTTPS를 사용한다면?**

만약 Git 주소가 `https://git.my-company.com/...` 형태라면 방법이 약간 다릅니다. 이 경우 SSH 터널을 직접 만들어줘야 합니다.

1.  **PC2에서 터널 생성:** 아래 명령어로 PC2의 특정 포트(예: 8443)를 PC1을 통해 사내 Git 서버의 HTTPS 포트(443)로 연결합니다. 이 터미널은 계속 켜두거나 백그라운드로 실행해야 합니다.

    ```bash
    # <PC1 사용자 이름>@<PC1 IP 주소> 와 git.my-company.com 은 실제 값으로 변경
    ssh -N -L 8443:git.my-company.com:443 ubuntu-pc1@192.168.1.100
    ```

2.  **PC2의 Git 전역 설정 변경:** Git이 사내 서버 주소를 보면 로컬 터널 포트로 요청을 보내도록 전역 설정을 추가합니다.

    ```bash
    git config --global url."http://localhost:8443/".insteadOf "https://git.my-company.com/"
    ```

이제부터 `https://git.my-company.com/...` 로 시작하는 모든 Git 명령어는 자동으로 `http://localhost:8443/...`를 통해 터널링됩니다.
