
아래는 **Ubuntu 24.04 + RTX 5090** 환경을 기준으로 정리한 **Isaac Sim 5.0.0 설치 가이드**야. 두 가지 방식(로컬 워크스테이션 설치 vs. Docker 컨테이너 설치)을 모두 담았고, 5090(Blackwell) GPU에서 흔히 부딪히는 드라이버 이슈까지 체크리스트에 넣었어.

---

# Isaac Sim 5.0.0 설치 가이드 (Ubuntu 24.04 + RTX 5090)

## 0) 한눈에 보기 (권장 경로)

* **가장 간단/안정:** **컨테이너 설치(NGC)** → 드라이버와 Docker만 맞추면 바로 실행/업데이트 쉬움. ([NVIDIA NGC Catalog][1])
* **오프라인/로컬 실행 선호:** **워크스테이션(Standalone ZIP)** → 압축 해제 후 스크립트로 실행. ([docs.isaacsim.omniverse.nvidia.com][2])
* **소스 빌드(개발자용):** 24.04에선 GCC 11 필요(12+ 미지원). 빠르게 쓰려면 컨테이너나 Standalone을 권장. ([GitHub][3])

---

## 1) 사전 요구사항 & 드라이버 체크

### OS / 하드웨어

* **지원 OS:** Ubuntu **22.04/24.04**(Windows 10/11도 지원). ([docs.isaacsim.omniverse.nvidia.com][4])
* **GPU 권장:** RTX 4080 이상(컨테이너/문서 기준). 5090이면 충분. ([NVIDIA Developer Forums][5])

### NVIDIA 드라이버(Blackwell 권장 포인트)

* Isaac Sim 5.0은 **최신 Production Branch 드라이버 권장**. 새 GPU(예: RTX 5090)에서 문제 시 **.run 설치본** 사용도 권장. ([docs.isaacsim.omniverse.nvidia.com][4])
* **Omniverse/Kit 측 권고:** Linux에서 **570.144 이하** 일부 드라이버의 셰이더 컴파일 크래시가 **575.64에서 수정**됨 → 가능하면 **R575.64 이상** 권장. ([docs.omniverse.nvidia.com][6])
* (컨테이너 배포 문서) 로컬 배포 전제: Ubuntu 22.04/24.04, **NVIDIA Container Toolkit ≥ 1.17.0**. ([NVIDIA NGC Catalog][1])

**빠른 점검**

```bash
nvidia-smi         # 드라이버/GPU 인식 확인
sudo apt install -y vulkan-tools
vulkaninfo | head  # Vulkan 초기화 확인(선택)
```

---

## 2) 방법 A — 워크스테이션 설치(Standalone ZIP)

### A-1. 호환성 사전 점검(강력 추천)

* **Isaac Sim Compatibility Checker**를 받아서 실행 → 머신 호환성 빠르게 확인.
  실행 예:

```bash
chmod +x omni.isaac.sim.compatibility_check.sh
./omni.isaac.sim.compatibility_check.sh
```

(문서에 라이트웨이트 점검 앱 권장 문구와 실행 안내 있음) ([docs.isaacsim.omniverse.nvidia.com][2])

### A-2. 다운로드 & 설치

1. **Download Isaac Sim** 페이지에서 **Standalone(리눅스 x86\_64 ZIP)** 받기. ([docs.isaacsim.omniverse.nvidia.com][7])
2. 압축 해제 & 설치 스크립트:

```bash
mkdir -p ~/apps && cd ~/apps
unzip ~/Downloads/isaac-sim-standalone-5.0.0-linux-x86_64.zip -d isaac-sim-5.0.0
cd isaac-sim-5.0.0
./isaac-sim.post_installation.sh   # 필요한 추가 구성/캐시 준비
```

3. **런처(Selector)로 실행 모드 선택**

```bash
./isaac-sim.selector.sh            # GUI에서 "Isaac Sim Full" 선택 후 실행
```

(문서 예시: ZIP 해제, post\_install, selector로 **“Isaac Sim Full”** 선택) ([docs.isaacsim.omniverse.nvidia.com][2])

> 팁: 첫 실행은 셰이더/확장 캐시 생성 때문에 다소 오래 걸림. 이후부터는 훨씬 빨라짐. ([GitHub][3])

---

## 3) 방법 B — 컨테이너 설치(NGC, 권장)

### B-1. Docker/NVIDIA Container Toolkit 준비

* **NVIDIA Container Toolkit ≥ 1.17.0** 필요. 설치 후 `nvidia-smi`가 컨테이너 내부에서도 보여야 함. ([NVIDIA NGC Catalog][1])

### B-2. 이미지 풀 & 실행

```bash
# 이미지 받기
docker pull nvcr.io/nvidia/isaac-sim:5.0.0

# 캐시/로그/데이터 볼륨 준비(예시)
mkdir -p ~/docker/isaac-sim/{cache/{ov,pip,glcache,computecache,asset_browser},logs,data,pkg,documents}

# 인터랙티브 셸로 컨테이너 진입
docker run --name isaac-sim --entrypoint bash -it --runtime=nvidia --gpus all \
  --rm --network=host \
  -e "ACCEPT_EULA=Y" -e "PRIVACY_CONSENT=Y" \
  -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
  -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
  -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
  -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim/cache/asset_browser:/isaac-sim/exts/isaacsim.asset.browser/cache:rw \
  -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
  -v ~/docker/isaac-sim/pkg:/root/.local/share/ov/pkg:rw \
  -v ~/docker/isaac-sim/documents:/root/Documents:rw \
  nvcr.io/nvidia/isaac-sim:5.0.0
```

(위 실행·볼륨 마운트 구성은 NGC 가이드 그대로임) ([NVIDIA NGC Catalog][1])

컨테이너 내부에서:

```bash
# 헤드리스(서버) 모드 실행(로그 상세)
./runheadless.sh -v
```

* 기본 **라이브스트리밍(WebRTC)** 클라이언트로 원격 접속 가능(Omniverse Streaming Client는 5.0에서 미지원). ([NVIDIA NGC Catalog][1])

> 참고: 컨테이너 배포 공식 단계는 **nvidia-smi 확인 → pull → docker run(동일 옵션)** 순으로 제시되어 있어. ([NVIDIA NGC Catalog][1])

---

## 4) (선택) 소스에서 빌드할 경우 참고

* Isaac Sim 5.0은 오픈소스 공개. 단, **Ubuntu 24.04에서 빌드 시 GCC/G++ 11 필수**(12+ 미지원).

  ```bash
  sudo apt-get install gcc-11 g++-11 build-essential
  sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 200
  sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 200
  ```

  (공식 README의 24.04 주의사항과 설치 예시) ([GitHub][3])

---

## 5) 첫 실행 후 체크리스트

* **ROS 2 Jazzy 지원**: 5.0에서 공식 지원(ROS Bridge 활성화 시 내부 라이브러리 자동 로드). ([NVIDIA NGC Catalog][1])
* **튜토리얼/예제 실행**: Basic/Robot 튜토리얼로 렌더·물리·센서 확인. ([docs.isaacsim.omniverse.nvidia.com][7])
* **캐시/프리팹 다운로드 경로**: 컨테이너에선 위에서 마운트한 볼륨에 축적되어, 재실행 속도 향상. ([NVIDIA NGC Catalog][1])

---

## 6) 문제 해결(특히 RTX 5090/Ubuntu 24.04)

1. **드라이버 호환**

   * Blackwell 계열은 **R575.64 이상** 권장(이전 버전에서 Kit 앱 크래시 이슈가 보고됨). ([docs.omniverse.nvidia.com][6])
   * 새 GPU에서 문제 시 **Unix Driver Archive의 최신 Production Branch**로 업그레이드(.run 설치본) 권장. ([docs.isaacsim.omniverse.nvidia.com][4])

2. **Vulkan/드라이버 버전 검사 경고 우회(임시)**

   * 특정 드라이버/환경에서 버전 감지 문제로 ‘드라이버 검증 실패’ 경고가 뜰 수 있음 →
     **명령행 옵션** `--/rtx/verifyDriverVersion/enabled=false` 로 체크 비활성화 가능(근본 원인은 드라이버 업데이트로 해결 권장). ([docs.omniverse.nvidia.com][8])

3. **Ubuntu 24.04에서 소스 빌드 실패**

   * GCC 12+ 기본 탑재로 빌드 에러 발생 가능 → **GCC/G++ 11로 대체**. 빌드보다 **컨테이너/Standalone** 사용을 추천. ([GitHub][3])

---

## 7) 왜 컨테이너를 권장하나요?

* OS/드라이버 이슈를 최소화하고, 캐시/데이터 볼륨만 유지하면 **업데이트와 재현성**이 가장 편함.
* **NGC 공식 레시피**(볼륨/환경변수/네트워크 설정)가 잘 정리되어 있으며, 24.04 + RTX 50xx 조합에서도 안정적. ([NVIDIA NGC Catalog][1])

---

### 공식 문서/링크 (요약)

* **설치 인덱스 / 워크스테이션 설치 / 다운로드 / 컨테이너 설치**: Isaac Sim 5.0 문서. ([docs.isaacsim.omniverse.nvidia.com][2])
* **요구사항(24.04 지원 명시)**: System Requirements. ([docs.isaacsim.omniverse.nvidia.com][4])
* **NGC 컨테이너 안내(명령어/요구사항/스트리밍)**: Isaac Sim 5.0 컨테이너 페이지. ([NVIDIA NGC Catalog][1])
* **드라이버 권고(Production Branch / .run)**: Driver Requirements 노트. ([docs.isaacsim.omniverse.nvidia.com][4])
* **Linux 드라이버 이슈 수정 버전(575.64+)**: Omniverse 기술 요구사항. ([docs.omniverse.nvidia.com][6])
* **소스 빌드 주의(24.04는 GCC 11 필요)**: Isaac Sim GitHub README. ([GitHub][3])
* **컨테이너 단계 요약(명령·볼륨)**: 컨테이너 가이드/FAQ. ([docs.isaacsim.omniverse.nvidia.com][9])

---

필요하면, \*\*네 환경에 맞춘 스크립트(컨테이너 실행 쉘, 데스크톱 런처, ROS 2 브리지 예시 등)\*\*도 바로 만들어줄게. 어떤 방식으로 운용할지(로컬 GUI, 헤드리스+WebRTC, ROS 2 통합 등)만 알려줘!

[1]: https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim "Isaac Sim | NVIDIA NGC"
[2]: https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/install_workstation.html "Workstation Installation — Isaac Sim Documentation"
[3]: https://github.com/isaac-sim/IsaacSim "GitHub - isaac-sim/IsaacSim: NVIDIA Isaac Sim™ is an open-source application on NVIDIA Omniverse for developing, simulating, and testing AI-driven robots in realistic virtual environments."
[4]: https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/requirements.html?utm_source=chatgpt.com "Isaac Sim Requirements"
[5]: https://forums.developer.nvidia.com/t/gpu-requirement/305727?utm_source=chatgpt.com "GPU Requirement - Isaac Sim"
[6]: https://docs.omniverse.nvidia.com/dev-guide/latest/common/technical-requirements.html?utm_source=chatgpt.com "Technical Requirements — Omniverse Developer Guide"
[7]: https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/download.html "Download Isaac Sim — Isaac Sim Documentation"
[8]: https://docs.omniverse.nvidia.com/guide-kit-cae/latest/common/technical-requirements.html?utm_source=chatgpt.com "Technical Requirements — Kit CAE User Guide"
[9]: https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/install_container.html?utm_source=chatgpt.com "Container Installation - Isaac Sim Documentation"

########  
추가 참고:  
https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/index.html  
https://github.com/isaac-sim/IsaacSim  
https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim  
https://ahnbk.dev/?p=612  
https://sblim.tistory.com/entry/Isaac-Sim-50-on-Ubuntu-2404-Unitree-G1-RL-LAB-1  

################  


문제 2개가 섞여 있어요:

1. `xdg-open` 없음 → 예제 뷰어가 리눅스 데스크탑 유틸리티를 찾다 실패
2. **Isaac 자산(assets) 루트**를 못 찾아서 예제(Nova Carter) 리소스를 못 여는 상황

아래대로 처리하면 됩니다.

---

# 1) `xdg-open` 오류는 치명적이지 않습니다

예제 패널의 *Browse Scenes*가 컨테이너 안에서 파일 브라우저를 열려고 하다 실패한 겁니다. 시뮬레이션 자체엔 영향 없고, 필요하면 컨테이너에 `xdg-utils`만 넣으면 사라집니다:

```bash
apt-get update && apt-get install -y xdg-utils   # 선택 사항
```

(이 단계는 정보용—공식 문서 이슈는 아니고, 핵심은 2번 자산 경로 설정입니다.)

---

# 2) “Could not find assets root folder” 해결 (정식 방법)

Isaac Sim 5.0은 기본적으로 **Isaac 자산 루트**가 설정되어 있어야 예제(로봇/환경)를 불러옵니다. Docker에서 가장 확실한 건 **Nucleus 서버의 Isaac 5.0 자산**을 기본 루트로 지정하는 겁니다.

## A) Nucleus 서버를 쓰는 경우(권장)

Docker 실행 시 **OMNI\_SERVER** 환경변수나 **명령행 플래그**로 Isaac 5.0 자산 루트를 지정하세요. (공식 5.0 설치 FAQ 예시)

```bash
docker run --gpus all --network=host \
  -e ACCEPT_EULA=Y \
  -e OMNI_SERVER=omniverse://<NUCLEUS_IP>/NVIDIA/Assets/Isaac/5.0 \
  -e OMNI_USER=<username> -e OMNI_PASS=<password> \
  nvcr.io/nvidia/isaac-sim:5.0.0
```

또는 앱 실행 플래그:

```bash
./isaac-sim.sh \
  --/persistent/isaac/asset_root/default="omniverse://<NUCLEUS_IP>/NVIDIA/Assets/Isaac/5.0"
```

이 값은 Isaac Sim 내부 API가 자산 루트를 찾을 때 그대로 사용됩니다. ([Isaac Sim Documentation][1])

> 참고: 컨테이너에서 로컬 Nucleus(예: `omniverse://localhost/NVIDIA/Assets/...`)를 쓰는 예시가 포럼에도 정리돼 있습니다. ([NVIDIA Developer Forums][2])

## B) 로컬 디스크의 자산 팩을 쓰는 경우

오프라인/사내망이라면 **Isaac 5.0 자산 팩을 로컬에 내려받아** 컨테이너에 마운트하고, 그 경로를 기본 루트로 지정할 수 있습니다. (FAQ에 로컬 경로 지정/브라우저 폴더 설정 예시 포함)

```bash
./isaac-sim.sh \
  --/persistent/isaac/asset_root/default="/isaac-assets/Assets/Isaac/5.0"
```

필요하면 `~/.local/share/ov/data/Kit/Isaac-Sim/5.0/user.config.json` 에도 동일 값을 넣어 둘 수 있습니다. ([Isaac Sim Documentation][1])

## C) 인터넷(S3) 기본 루트를 그대로 쓰려면

기본값은 퍼블릭 S3의 Isaac 5.0 자산입니다. 방화벽/프록시로 S3가 막혀 있으면 위 A/B 방식으로 전환하세요. (FAQ에 기본값/설정 위치 명시) ([Isaac Sim Documentation][1])

---

# 3) Nova Carter 예제가 보이는지 확인

자산 루트가 제대로 잡히면 **Content/Asset Browser**에서 다음 경로가 보여야 합니다:

* `omniverse://<서버>/NVIDIA/Assets/Isaac/5.0/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd`
  (5.0 문서의 로봇 자산 목록에 Nova Carter 경로가 기재되어 있습니다.) ([Isaac Sim Documentation][3])

그다음 `Window ▸ Examples ▸ Robotics Examples ▸ ROS2 ▸ Navigation ▸ Nova Carter` 를 다시 열면, 더 이상 “assets root” 오류가 나오면 안 됩니다. Isaac 내부의 “자산 루트 탐색” 유틸은 루트를 못 찾으면 RuntimeError를 던지도록 정의돼 있는데, 위 설정을 하면 통과합니다. ([Isaac Sim Documentation][4])

---

## 한 번에 점검 체크리스트

1. 컨테이너 실행 옵션에 **OMNI\_SERVER**(또는 `--/persistent/isaac/asset_root/default=...`)가 들어갔는가? ([Isaac Sim Documentation][1])
2. Isaac에서 Content/Asset Browser에 **`NVIDIA/Assets/Isaac/5.0/Isaac`** 폴더들이 보이는가? (FAQ의 폴더 예시와 일치 확인) ([Isaac Sim Documentation][1])
3. Nova Carter 경로(`.../Robots/NVIDIA/NovaCarter/nova_carter.usd`)를 직접 더블클릭해 열 수 있는가? ([Isaac Sim Documentation][3])

---

필요하시면, 지금 쓰시는 Docker 실행 스크립트/compose 파일을 보내주세요. **Nucleus 주소/계정/캐시**까지 포함해 5.0 권장값으로 깔끔히 정리해 드릴게요.

[1]: https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/install_faq.html?utm_source=chatgpt.com "Setup Tips - Isaac Sim Documentation"
[2]: https://forums.developer.nvidia.com/t/could-not-find-isaac-sim-assets-folder/278426?utm_source=chatgpt.com "Could not find Isaac Sim assets folder"
[3]: https://docs.isaacsim.omniverse.nvidia.com/5.0.0/assets/usd_assets_robots.html?utm_source=chatgpt.com "Robot Assets - Isaac Sim Documentation"
[4]: https://docs.isaacsim.omniverse.nvidia.com/5.0.0/py/source/extensions/isaacsim.storage.native/docs/index.html?utm_source=chatgpt.com "[isaacsim.storage.native] Isaac Sim Native Storage"

