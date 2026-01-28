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
