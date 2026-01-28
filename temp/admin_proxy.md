에러 메시지를 보면 원인을 정확히 알 수 있습니다.

> `Failed to fetch ... connection failed [IP: 91.189.91.105 80]`

이 메시지는 `apt`가 Proxy 서버(`12.26.204.100`)를 거치지 않고, **Ubuntu 저장소(`91.189.91.105`)에 "Direct(직접)"로 연결을 시도하다가 방화벽에 막혀 실패**한 것입니다.

즉, `sudo -E`를 썼음에도 불구하고, **`admin` 계정의 현재 쉘에 Proxy 환경변수가 전혀 로드되지 않았거나**, `apt` 설정 파일이 무시되고 있는 상황입니다.

아래 순서대로 해결해보세요.

---

### 1. 가장 강력한 방법: 명령어에 직접 Proxy 주입 (즉시 해결용)

설정 파일이나 환경변수가 꼬여 있을 때, `apt` 명령어 자체에 옵션(`-o`)으로 Proxy를 강제로 주입하면 무조건 동작해야 합니다. 아래 명령어를 그대로 입력해보세요.

```bash
# admin 계정에서 실행 (한 줄로 입력)
sudo apt -o Acquire::http::proxy="http://12.26.204.100:8080/" -o Acquire::https::proxy="http://12.26.204.100:8080/" update

```

* **성공한다면?** -> 네트워크나 인증서 문제는 없습니다. 오직 **설정 파일이 제대로 로드되지 않은 문제**입니다. (아래 2번으로 이동)
* **실패한다면?** -> `admin` 계정 자체가 네트워크 권한이 없거나 오타 문제입니다.

---

### 2. 원인 분석: 환경변수 로드 확인

`admin` 계정으로 전환할 때 어떻게 하셨나요? 보통 `su admin`으로 전환하면 `/etc/environment` 파일을 다시 읽지 않습니다.

현재 상태를 확인하기 위해 `admin` 계정에서 아래 명령어를 쳐보세요.

```bash
env | grep -i proxy

```

만약 **아무것도 나오지 않는다면**, `/etc/environment` 설정이 로드되지 않은 것입니다.
이 경우 아래 명령어로 환경변수를 수동으로 로드한 뒤 다시 시도해 보세요.

```bash
source /etc/environment
sudo -E apt update

```

*(참고: `admin` 계정으로 로그인할 때 `su - admin` (하이픈 포함)을 사용해야 환경설정을 새로 읽어옵니다.)*

---

### 3. Apt 설정 파일 충돌 확인 (우선순위 문제)

작성하신 `/etc/apt/apt.conf.d/05proxy` 파일이 있지만, 다른 파일이 이 설정을 덮어쓰고 있을 수 있습니다. (예: `99proxy`, `20auto-upgrades` 등 숫자가 높은 파일이 우선순위를 가짐)

`admin` 계정에서 아래 명령어로 Proxy 설정이 들어있는 모든 파일을 찾아보세요.

```bash
grep -r "Proxy" /etc/apt/apt.conf.d/

```

* **체크 포인트:**
1. `05proxy` 외에 다른 파일(예: `90curtin-aptproxy` 등)에서 `DIRECT` 혹은 다른 IP로 설정을 덮어쓰고 있지 않은지 확인하세요.
2. 만약 다른 파일이 있다면, 그 내용을 주석 처리하거나 삭제해야 합니다.
3. 혹시 `05proxy` 파일의 권한이 `admin`이 읽을 수 없는 상태인지 확인하세요 (`ls -l /etc/apt/apt.conf.d/05proxy`).



---

### 요약 및 추천

지금 당장 `apt update`를 성공시키려면 **1번 방법(명령어에 직접 주입)**을 사용하세요. 그 후 영구적인 해결을 위해 `admin` 계정의 `.bashrc` 맨 아래에 Proxy 설정을 추가하는 것이 가장 확실합니다.

```bash
# admin 계정에서 실행
echo 'export http_proxy="http://12.26.204.100:8080/"' >> ~/.bashrc
echo 'export https_proxy="http://12.26.204.100:8080/"' >> ~/.bashrc
source ~/.bashrc

```
#######################  
###########################  
############################  
상황이 진전되었습니다! **"IP: 12.26.204.100 8080"** 에러 메시지가 떴다는 것은, 이제 **Proxy 서버까지는 도달했다**는 뜻입니다.

다만, 회사 Proxy 서버가 HTTPS 통신을 중간에서 복호화/재암호화(Man-in-the-Middle)하고 있는데, `sudo apt` 명령어가 이 **회사 보안 인증서(Samsung CA)를 신뢰하지 않아서** 연결을 끊어버린 것입니다.

두 가지 해결 방법이 있습니다. (보안이 중요한 사내망이므로 **방법 1**을 먼저 시도하여 되는지 보고, 영구 적용은 **방법 2**로 하세요.)

---

### 방법 1: SSL 인증서 검증 무시 (가장 확실하고 빠른 방법)

사내망(Intranet)은 이미 신뢰할 수 있는 네트워크이므로, `apt`가 인증서를 깐깐하게 검사하지 않도록 옵션을 추가합니다.

아래 명령어를 **한 줄로** 입력해보세요. (`Verify-Peer=false` 옵션 추가)

```bash
sudo apt -o Acquire::http::proxy="http://12.26.204.100:8080/" \
         -o Acquire::https::proxy="http://12.26.204.100:8080/" \
         -o Acquire::https::Verify-Peer=false \
         -o Acquire::https::Verify-Host=false \
         update

```

**이 명령어가 성공한다면?**
매번 이렇게 칠 수 없으니, 설정 파일에 영구적으로 박아넣어야 합니다. **방법 2**를 진행하세요.

---

### 방법 2: 설정 파일(`/etc/apt/apt.conf.d/05proxy`) 수정

아까 만드신 `05proxy` 파일에 SSL 검증 무시 옵션을 추가하고, 혹시 모를 문법 오류를 수정합니다.

1. 파일 열기:
```bash
sudo gedit /etc/apt/apt.conf.d/05proxy

```


(또는 `nano`, `vi` 사용)
2. **기존 내용을 다 지우고** 아래 내용으로 덮어씌우세요. (세미콜론 `;` 필수)
```text
Acquire::http::proxy "http://12.26.204.100:8080/";
Acquire::https::proxy "http://12.26.204.100:8080/";
Acquire::ftp::proxy "ftp://12.26.204.100:8080/";
Acquire::socks::proxy "socks://12.26.204.100:8080/";
Acquire::https::Verify-Peer "false";
Acquire::https::Verify-Host "false";

```


3. 저장 후 다시 `admin` 계정에서 테스트:
```bash
sudo apt update

```



---

### 방법 3: 인증서 재등록 (방법 1, 2가 찜찜할 경우)

만약 "나는 보안상 검증을 끄기 싫다"고 하시면, 인증서가 시스템에 제대로 등록되지 않은 것입니다. `admin` 계정에서도 시스템 전역 인증서를 다시 로드해 봅니다.

1. 인증서 포맷 변환 및 이동 (확실하게 하기 위해):
```bash
# samsung_semi.crt 파일이 있는 곳에서 실행
sudo cp samsung_semi.crt /usr/local/share/ca-certificates/samsung_semi.crt

```


2. 인증서 갱신 (강제 적용):
```bash
sudo update-ca-certificates --fresh

```


*결과 메시지에 `1 added` (또는 그 이상)가 나오는지 확인해야 합니다.*
3. 그 후 다시 `sudo apt update` 시도.

하지만, **Jetson/Ubuntu 환경의 사내망에서는 `방법 2` (Verify-Peer false)가 정신 건강에 가장 좋습니다.** 회사 Proxy 인증서 갱신 주기마다 문제가 생길 수 있기 때문입니다.
