좋습니다. SSH 전문가로서 답변해 드리죠.

이것은 **'점프 호스트(jump host)'** 또는 **'배스천 호스트(bastion host)'** 시나리오라고 부릅니다. `pc1`에서 `pc2`를 경유하여 `pc3`로 파일을 보내는 가장 효율적이고 안전한 방법은 `scp`의 `ProxyJump` 옵션(또는 `-J` 플래그)을 사용하는 것입니다.

이 방식은 `pc2`에 임시 파일을 만들지 않고, `pc1`에서 `pc3`로 직접 암호화된 터널을 통해 파일을 스트리밍합니다.

-----

### 1\. 가장 현대적이고 간결한 방법 (`-J` 플래그)

`pc1`의 터미널에서 다음 명령어를 실행하세요.

```bash
scp -J [user2]@[pc2] [local_file_path] [user3]@[pc3]:[remote_destination_path]
```

  * **`-J [user2]@[pc2]`**: `pc2`를 점프 호스트로 지정합니다. `pc1`이 `[user2]` 계정으로 `[pc2]`에 먼저 SSH 접속을 시도합니다.
  * 이 접속이 성공하면, `pc2`를 통해 `pc3`로의 터널이 열립니다.
  * 그다음 `scp`가 `[local_file_path]` 파일을 `[user3]@[pc3]`의 `[remote_destination_path]`로 전송합니다.

**예시:**
`pc1`에 있는 `my_data.zip` 파일을 `user_on_pc2` 계정으로 `pc2.example.com`을 경유하여 `user_on_pc3` 계정의 `pc3.internal.net`의 `/home/user_on_pc3/uploads/` 디렉터리로 보내는 경우:

```bash
scp -J user_on_pc2@pc2.example.com ./my_data.zip user_on_pc3@pc3.internal.net:/home/user_on_pc3/uploads/
```

-----

### 2\. 구형 방식 (`ProxyCommand` 옵션)

만약 사용 중인 OpenSSH 버전이 오래되어 `-J` 옵션을 지원하지 않는다면 (OpenSSH 7.3 이전), `ProxyCommand`를 사용할 수 있습니다.

```bash
scp -o ProxyCommand="ssh [user2]@[pc2] -W %h:%p" [local_file_path] [user3]@[pc3]:[remote_destination_path]
```

  * **`-o ProxyCommand="..."`**: `scp`가 `pc3`에 연결하기 위해 실행할 쉘 명령을 지정합니다.
  * **`ssh [user2]@[pc2] -W %h:%p`**: `pc2`에 SSH로 접속한 뒤, `-W` (netcat 모드)를 사용해 `scp` 명령이 요청한 최종 목적지(`%h` = 호스트, `%p` = 포트)로 표준 입출력(stdin/stdout)을 포워딩합니다.

-----

### 💡 전문가의 팁: `~/.ssh/config` 설정하기

매번 이 긴 명령어를 입력하는 것은 번거롭습니다. `pc1`의 `~/.ssh/config` 파일을 설정해 두면 이 작업을 매우 간단하게 만들 수 있습니다.

`pc1`에서 `~/.ssh/config` 파일을 열어 다음 내용을 추가하세요.

```ssh-config
# pc2 (점프 호스트) 정보
Host pc2
    HostName [pc2_hostname_or_ip]
    User [user2]
    # 필요시 IdentityFile (SSH 키) 경로 지정
    # IdentityFile ~/.ssh/id_rsa_for_pc2

# pc3 (최종 목적지) 정보
Host pc3
    HostName [pc3_hostname_or_ip]
    User [user3]
    # "pc3로 가려면 pc2를 거쳐가라"는 설정
    ProxyJump pc2
    # 필요시 IdentityFile (SSH 키) 경로 지정
    # IdentityFile ~/.ssh/id_rsa_for_pc3
```

이렇게 설정하면, `pc1`의 터미널에서 `pc3`로 파일을 보낼 때 `scp`가 자동으로 `pc2`를 경유합니다. 명령어는 다음과 같이 간단해집니다.

```bash
# config 파일 덕분에 'pc3'가 모든 정보를 알고 있음
scp [local_file_path] pc3:[remote_destination_path]
```

이 설정은 `scp`뿐만 아니라 `ssh`에도 동일하게 적용됩니다. `ssh pc3`라고만 입력해도 `pc2`를 거쳐 `pc3`로 바로 접속됩니다.

이 방법들이 도움이 되길 바랍니다. 다른 SSH 관련 질문이 있으신가요?
