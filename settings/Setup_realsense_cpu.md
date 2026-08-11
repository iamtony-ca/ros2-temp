CPU-only 머신이면 커널 건드리는 부담이 없으니, **소스 빌드 + 커널 패치**가 가장 완전한 구성입니다(메타데이터 지원 포함, V4L2 백엔드 기본).

**0. 사전 점검**

```bash
uname -r          # 5.15 / 5.19 / 6.5 / 6.8 / 6.11 / 6.14 중 하나여야 함
lsb_release -cs   # noble
dpkg -l | grep realsense   # 출력 있으면 먼저 purge (deb/소스 혼용 금지)
```
24.04 기본 커널은 6.8이라 보통 그대로 통과합니다. 카메라는 아직 꽂지 마세요.

**1. 의존성** — 문서대로 명령을 나눠서 실행하세요 (한 번에 묶으면 의존성 꼬이는 플랫폼이 있다고 명시돼 있습니다)

```bash
sudo apt-get update && sudo apt-get upgrade
sudo apt-get install libusb-1.0-0-dev
sudo apt-get install libudev-dev
sudo apt-get install libssl-dev pkg-config libgtk-3-dev
sudo apt-get install git wget cmake build-essential
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
```
커널이 올라갔다면 여기서 **재부팅** 후 `uname -r` 재확인.

**2. 클론**

```bash
git clone https://github.com/realsenseai/librealsense.git
cd librealsense
git tag -l "v2.*" | tail -5     # 최신 stable 태그 확인
git checkout v2.XX.X            # master보다 릴리스 태그 권장
```

**3. udev 룰 + 커널 패치**

```bash
./scripts/setup_udev_rules.sh
./scripts/patch-realsense-ubuntu-lts-hwe.sh
sudo dmesg | tail -n 50          # 새 uvcvideo 등록 확인
```
`module verification failed: signature ... tainting kernel` 경고는 커널 4.4-30+ 표준 경고로 기능에 영향 없습니다.

**4. 빌드**

```bash
mkdir build && cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release \
          -DBUILD_EXAMPLES=true \
          -DBUILD_GRAPHICAL_EXAMPLES=true
make -j$(($(nproc)-1))
sudo make install
sudo ldconfig
```

**5. 검증** — 이제 카메라를 **USB 3.x 포트**에 연결

```bash
rs-enumerate-devices -s        # 모델/FW/USB descriptor 확인
realsense-viewer
```
`Usb Type Descriptor`가 2.1로 뜨면 케이블이나 포트 문제입니다(해상도·fps가 크게 깎임).

**막힐 때**
| 증상 | 조치 |
|---|---|
| `fatal error: openssl/opensslv.h` | `libssl-dev` 설치 누락 |
| fastrtps/fastcdr 빌드 실패 | `-DBUILD_WITH_DDS=OFF` |
| curl 의존성 실패 | `-DCHECK_FOR_UPDATES=OFF` |
| `Multiple realsense udev-rules were found!` | deb와 소스 혼용 — 한쪽 제거 |
| 뷰어에 장치 안 뜸 | 재부팅 후 `modinfo uvcvideo \| grep -i realsense` |

**앞으로 주의할 것 하나:** 커널이 업데이트되면 패치가 무효화됩니다. 새 커널로 부팅한 뒤 카메라가 안 잡히면 `patch-realsense-ubuntu-lts-hwe.sh`를 다시 돌리시면 됩니다. 자주 겪기 싫으면 `sudo apt-mark hold linux-generic linux-image-generic linux-headers-generic`으로 묶어두는 방법도 있습니다.

뷰어가 뜨면 펌웨어 업데이트 알림이 나올 수 있는데, ROS 2 쪽과 버전 궁합이 있으니 realsense-ros까지 붙일 계획이면 업데이트 전에 한 번 확인하시는 게 좋습니다.
