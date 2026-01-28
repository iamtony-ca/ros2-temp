협업 부서와의 원활한 개발 및 통합을 위해 **ROS 2 Jazzy + Ubuntu 24.04** 기반의 표준 개발 환경 정의서(초안)를 제안합니다.

이 정의서는 호환성 문제(Dependency Hell)를 방지하고, C++ 및 Python 개발 표준을 통일하여 코드 품질을 유지하는 데 중점을 두었습니다. 특히, 부서 간 협업 시 가장 큰 마찰 지점인 **패키지 의존성**과 **빌드 시스템**을 명확히 하는 것이 핵심입니다.

---

## 📋 협업을 위한 ROS 2 Jazzy 개발 환경 표준안

### 1. 운영체제 (OS) 및 기본 설정

협업 시 OS 버전의 파편화를 막기 위해 커널 및 기본 설정을 통일합니다.

* **OS:** Ubuntu 24.04 LTS (Noble Numbat)
* **Kernel:** Generic (기본) 또는 필요 시 `linux-image-rt` (Real-time 커널, 제어 주기 민감도에 따라 합의 필요)
* **Locale:** `UTF-8` (필수, ROS 2 통신 이슈 방지)
* **Time Zone:** `Asia/Seoul` (로그 타임스탬프 동기화)

### 2. ROS 2 환경 (Jazzy Jalisco)

ROS 2 Jazzy는 Ubuntu 24.04를 기본 지원하는 첫 LTS 버전입니다.

* **Distribution:** ROS 2 Jazzy Jalisco
* **Installation Type:**
* 개발용 PC: `ros-jazzy-desktop-full` (시뮬레이션 및 GUI 툴 포함)
* 배포/로봇용: `ros-jazzy-ros-base` (최소 설치)


* **Build System:** `colcon` (mixin 설정 공유 권장)
* **Middleware (RMW):** **[중요]** 협업 부서와 반드시 통일해야 합니다.
* 기본값: `rmw_fastrtps_cpp` (Fast DDS)
* 대안: `rmw_cyclonedds_cpp` (네트워크 부하가 크거나 WiFi 환경에서 선호됨)
* *제안: `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` (또는 합의된 RMW)를 `.bashrc`에 명시.*



### 3. 언어 및 컴파일러 표준

Ubuntu 24.04의 시스템 컴파일러 버전을 기준으로 표준을 잡습니다.

#### A. C++ (Primary)

ROS 2 Jazzy의 핵심 라이브러리들과 호환성을 유지합니다.

* **Standard:** **C++17** (Jazzy 권장 표준) 또는 C++20 (GCC 13 지원, 필요 시)
* **Compiler:** GCC 13.2.0 (Ubuntu 24.04 기본)
* **Build Tool:** CMake 3.28 이상
* **Style Guide:** Google C++ Style Guide 또는 ROS C++ Style
* **Formatter:** `clang-format` (버전 통일 필수, 예: v18)

#### B. Python (Secondary)

Ubuntu 24.04부터는 시스템 Python에 직접 패키지 설치가 제한(PEP 668)되므로 가상 환경 정책이 중요합니다.

* **Version:** Python 3.12
* **Environment Management:** `python3 -m venv` 사용 권장 (시스템 패키지 오염 방지)
* **Style Guide:** PEP 8
* **Linter/Formatter:** `flake8`, `black`
* **Type Checking:** `mypy` (선택 사항이나 추천)

### 4. 통합 개발 환경 (IDE) 및 도구

VS Code를 표준으로 하되, 필수 확장 프로그램을 정의하여 설정 시간을 단축합니다.

* **IDE:** Visual Studio Code
* **필수 Extensions:**
* `ms-vscode.cpptools` (C/C++)
* `ms-python.python` (Python)
* `ms-vscode.cmake-tools` (CMake)
* `ms-iot.vscode-ros` (ROS 2 지원)
* `eamodio.gitlens` (Git 협업 가시성)


* **Version Control:** Git
* `git-lfs` 설치 필수 (대용량 모델/Bag 파일 관리)



### 5. 가상화 및 배포 (Docker)

협업 부서와 하드웨어 환경이 다를 경우(예: 개발자 PC vs 로봇), Docker 사용을 **강력하게 권장**합니다.

* **Container Runtime:** Docker Engine + **NVIDIA Container Toolkit** (GPU 가속 및 시뮬레이션 용)
* **Base Image:** `osrf/ros:jazzy-desktop-full` 기반 커스텀 이미지
* **Dev Container:** VS Code의 Dev Containers 기능을 활용하여, `devcontainer.json` 파일을 프로젝트 루트에 공유. (이러면 신규 입사자나 타 부서 인원도 "Open in Container" 버튼 하나로 환경 세팅이 끝납니다.)

---

### 💡 전문가의 팁 (협업 시 고려사항)

1. **의존성 관리 (`rosdep`):**
프로젝트의 `package.xml`에 모든 의존성을 명시하고, 협업 부서에는 다음 명령어로 환경을 셋업하도록 가이드하세요.
```bash
rosdep install --from-paths src --ignore-src -r -y

```


2. **PEP 668 대응:**
Ubuntu 24.04에서는 `pip install`이 막힐 수 있습니다. 협업 가이드에 "가상 환경(venv) 활성화 후 패키지 설치"를 명시하거나, Docker를 사용하여 이 문제를 원천 차단하는 것이 정신 건강에 좋습니다.

**다음 단계:**
이 내용을 바탕으로 협업 부서에 공유할 **`Dockerfile`** 또는 **`devcontainer.json`** 템플릿을 작성해 드릴까요? (VS Code에서 바로 사용할 수 있어 세팅 시간을 획기적으로 줄여줍니다.)
