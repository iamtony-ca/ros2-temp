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
