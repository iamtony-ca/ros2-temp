CPU-only x86 / Ubuntu 24.04 기준입니다. 문서 절차를 그대로 따르고, 제가 덧붙인 것은 **[선택]**으로 표시했습니다.

**0. [선택] 사전 점검**

```bash
uname -r                    # 5.15/5.19/6.5/6.8/6.11/6.14 중 하나
dpkg -l | grep realsense    # 출력 있으면 purge (deb·소스 혼용 금지)
```

**1. 의존성 설치** — 명령을 묶지 말고 나눠서 실행 (문서 명시)

```bash
sudo apt-get update && sudo apt-get upgrade

sudo apt-get install libusb-1.0-0-dev
sudo apt-get install libudev-dev
sudo apt-get install libssl-dev pkg-config libgtk-3-dev

sudo apt-get install git wget cmake build-essential
```

RealSense 카메라를 **모두 뽑은 뒤** 다음을 실행:

```bash
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
```

> `libudev-dev`는 필수는 아니지만 권장됩니다 — 설치 시 SDK가 이벤트 기반으로 USB를 감지하고, 없으면 타이머 폴링 방식이라 장치 감지가 둔해집니다.
> glfw3 / mesa / gtk는 OpenGL 예제(=뷰어)를 빌드할 때 필요합니다.

**2. 소스 받기**

```bash
git clone https://github.com/realsenseai/librealsense.git
cd librealsense
```

**3. udev 룰 등록**

```bash
./scripts/setup_udev_rules.sh
```
되돌리려면 `./scripts/setup_udev_rules.sh --uninstall`

**4. 커널 모듈 패치**

```bash
./scripts/patch-realsense-ubuntu-lts-hwe.sh
```
스크립트가 RealSense 관련 커널 모듈을 내려받아 패치·빌드한 뒤 기존 모듈 대신 삽입합니다. 실패하면 원래 uvc 모듈로 복구됩니다.

확인:
```bash
sudo dmesg | tail -n 50      # 새 uvcvideo 드라이버 등록 로그
```
> 커널 6.8 미만/이상 여부와 무관하게, 20.04에서 커널 5.13 미만이면 `patch-realsense-ubuntu-lts.sh`를 씁니다. 24.04는 hwe 쪽입니다.

**5. 빌드 & 설치**

```bash
mkdir build && cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=true
make -j$(($(nproc)-1))
sudo make install
```
- 기본 빌드는 Debug이므로 `-DCMAKE_BUILD_TYPE=Release` 권장
- `-DBUILD_EXAMPLES=true`면 데모·튜토리얼이 함께 빌드됩니다. OpenGL/X11이 없는 헤드리스 환경이라면 `-DBUILD_GRAPHICAL_EXAMPLES=false`를 추가하지만, 뷰어를 쓰실 거니 붙이지 마세요.
- 설치 위치: 공유 라이브러리 `/usr/local/lib`, 헤더 `/usr/local/include`, 바이너리 `/usr/local/bin`
- Linux 빌드는 **V4L2 백엔드가 기본**입니다.

**6. 실행** — 이제 카메라를 USB 3.x 포트에 연결

```bash
realsense-viewer
```

**트러블슈팅 (문서 표)**

| 증상 | 조치 |
|---|---|
| `Multiple realsense udev-rules were found!` | deb와 소스 둘 다 설치됨 → 한쪽 제거 |
| 패치 스크립트 `fatal error: openssl/opensslv.h` | openssl 패키지 설치 |
| `dmesg: uvcvideo: module verification failed ... tainting kernel` | 커널 4.4-30+ 표준 경고, 기능 영향 없음 |
| `modprobe uvcvideo` → `uvc kernel module is not loaded` | 패치 모듈이 현재 커널과 불일치. `uname -r` 확인 후 1단계부터 재진행 |
| fastrtps / fastcdr 빌드 실패 | `-DBUILD_WITH_DDS=OFF` |
| curl 의존성 빌드 실패 | `-DCHECK_FOR_UPDATES=OFF` |
| `gcc: internal compiler error` | 메모리/스왑 부족 |

**[선택] 알아두면 좋은 것**
- 재빌드할 때는 `sudo make uninstall && make clean && make && sudo make install`
- CUDA 플래그 등 일부 옵션은 cmake 3.8+ 필요
- 커널이 업데이트되면 패치가 무효화되므로, 새 커널 부팅 후 카메라가 안 잡히면 4단계를 다시 실행하세요
- VM은 USB3 변환 계층 때문에 공식 미지원입니다

문서 원문: [installation.md](https://github.com/realsenseai/librealsense/blob/master/doc/installation.md) · [지원 커널 버전 확인](https://github.com/realsenseai/librealsense/releases/)

##
##
##
##

괜찮은 접근입니다. 패치 스크립트는 **실패하면 원래 uvc 모듈로 복구**하도록 되어 있어서, 최악의 경우도 "안 됨"이지 시스템이 깨지는 건 아닙니다. 게다가 이 머신엔 NVIDIA 드라이버도 없으니 되돌릴 게 더 적고요.

다만 순서를 조금 바꾸면 실패해도 헛수고가 줄어듭니다.

**1) 패치를 먼저 시도** (여기가 성패 갈림)
```bash
cd librealsense
./scripts/setup_udev_rules.sh
./scripts/patch-realsense-ubuntu-lts-hwe.sh
```

**2) 결과 판정**
```bash
sudo dmesg | tail -n 50
```
- 새 uvcvideo 등록 로그가 보이면 → 성공. 그대로 빌드 진행
- 스크립트가 에러로 죽거나 등록 로그가 없으면 → 커널 7.0 미지원 확정

`module verification failed: signature ... tainting kernel` 경고는 정상이니 실패로 오해하지 마세요.

**3) 빌드는 판정 후에**
- 성공 → `cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=true`
- 실패 → 같은 명령에 `-DFORCE_RSUSB_BACKEND=true`만 추가

빌드가 오래 걸리니 패치 결과를 먼저 확정하고 백엔드를 정하는 게 낫습니다. 이미 V4L2로 빌드했는데 패치가 안 된 상태면 `sudo make uninstall && make clean` 후 재빌드해야 합니다.

**되돌리기**
```bash
./scripts/setup_udev_rules.sh --uninstall   # udev 룰 제거
```

패치는 실패 시 자동 복구되고, 부분적으로 적용됐다 싶으면 재부팅만 해도 기본 모듈로 돌아옵니다.

진행하시다 스크립트 출력이나 dmesg에서 막히는 부분 있으면 그대로 붙여주세요.
##
##
##
##
예상했던 실패고, 메시지가 명확하네요. 그런데 **Docker는 이 문제를 해결하지 못합니다.** 컨테이너는 호스트 커널을 그대로 공유하기 때문에, 컨테이너 안에서 V4L2 백엔드로 빌드해도 호스트 uvcvideo가 패치되지 않았다는 사실은 똑같습니다.

앞에서 5090 머신에 Docker를 권했던 건 "호스트를 안 건드리려고"였고, 실제로 커널 문제를 우회하는 건 Docker가 아니라 **RSUSB 백엔드**입니다. 지금 이 머신은 호스트를 건드릴 이유가 없으니 Docker를 끼울 이유도 없습니다 — 그냥 로컬에서 RSUSB로 다시 빌드하는 게 가장 간단합니다.

udev 룰은 이미 깔렸으니 그대로 두고:

```bash
cd ~/librealsense
rm -rf build && mkdir build && cd build

cmake ../ -DCMAKE_BUILD_TYPE=Release \
          -DBUILD_EXAMPLES=true \
          -DFORCE_RSUSB_BACKEND=true

make -j$(($(nproc)-1))
sudo make install
```
(이전 cmake 캐시가 남아 있으면 백엔드 플래그가 안 먹을 수 있어 `build`를 지우고 갑니다.)

빌드 후 카메라를 USB 3.x 포트에 연결하고:
```bash
rs-enumerate-devices -s
realsense-viewer
```

**참고로 알아두실 것**
- 공식 Docker 이미지가 V4L2로 빌드됐는지 RSUSB로 빌드됐는지는 제가 확인하지 못했습니다. 만약 V4L2라면 컨테이너 안에서도 지금과 같은 이유로 장치가 안 잡힙니다. Isaac ROS 문서가 컨테이너 빌드에 `--force_rsusb`를 쓰는 것도 같은 맥락입니다.
- RSUSB로 잘 돌아가는데 나중에 CPU 부하나 멀티캠이 걸림돌이 되면, 그때 GA 커널 6.8을 추가로 설치해서 부팅하고 정식 패치 경로로 가면 됩니다. 지금 결정할 필요는 없습니다.

RSUSB 빌드도 실패하면 그때 cmake/make 출력 붙여주세요.
##
##
##
##
udev 룰이 등록은 됐는데 커널에 반영이 안 된 상태로 보입니다. 순서대로 확인해 보세요.

**1) 파일이 실제로 있는지**
```bash
ls -l /etc/udev/rules.d/ | grep -i realsense
```
`99-realsense-libusb.rules`가 보여야 합니다. **두 개 이상** 나오면 그게 원인입니다 (deb·소스 혼용 케이스).

**2) 룰 리로드 + 카메라 재인식**
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```
그다음 **카메라 USB를 뽑았다 다시 꽂으세요.** 룰은 장치가 연결되는 시점에 적용되므로, 이미 꽂혀 있던 장치에는 소급 적용되지 않습니다. 이게 제일 흔한 원인입니다.

**3) 그래도 안 되면 룰 재설치**
```bash
cd ~/librealsense
./scripts/setup_udev_rules.sh --uninstall
./scripts/setup_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```
재부팅 한 번이 가장 확실합니다.

**4) 권한이 실제로 붙었는지 확인**
```bash
lsusb | grep -i intel          # RealSense 장치 확인 (Bus/Device 번호 메모)
ls -l /dev/bus/usb/<Bus>/<Device>
```
소유자가 `root:plugdev`이고 그룹 쓰기 권한(`rw-rw-`)이 있으면 정상입니다. `root:root`에 `rw-r--r--`이면 룰이 안 먹은 겁니다.

**5) 그룹 확인**
```bash
groups | grep plugdev
```
`plugdev`가 없다면:
```bash
sudo usermod -aG plugdev $USER
```
후 **로그아웃/재로그인**(또는 재부팅)해야 반영됩니다.

RSUSB 백엔드는 libusb로 직접 통신하기 때문에 이 udev 권한이 V4L2 때보다 더 결정적입니다. 임시 확인용으로 `sudo realsense-viewer`를 띄워서 장치가 잡히면 권한 문제가 확실하니, 위 4)~5)를 집중해서 보시면 됩니다. 다만 sudo로 상시 쓰는 건 권하지 않습니다.

각 단계 출력 붙여주시면 어디서 끊기는지 짚어드릴게요.
