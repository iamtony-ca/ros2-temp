 **소프트웨어(드라이버/커널) 설치는 완벽하게 되었습니다.** 하지만 **하드웨어(카메라/케이블/전원) 연결에 문제**가 있어 GMSL 링크가 맺어지지 않고 있습니다.

정확한 원인과 해결 방법을 순서대로 알려드립니다.

-----

### 1\. 분석 결과 (로그 해석)

1.  **소프트웨어 매칭 성공:**

      * OS(`L4T 36.4.0`)와 드라이버(`1.3.2-SL-MAX96712...L4T36.4.0`) 버전이 정확히 일치합니다.
      * `ZEDX Driver State`에서 `sl_max96712`의 사용 카운트(Used)가 **4**로 잡혀 있습니다. 이는 리눅스 커널이 장치 트리(Device Tree)를 통해 "카메라 4대가 있을 것이다"라고 인지하고 드라이버를 로드시켰다는 뜻입니다.
      * `dmesg`의 `module verification failed` 메시지는 서명 관련 경고일 뿐, 실제 드라이버 로드에는 성공했으므로 **무시해도 됩니다.**

2.  **결정적 문제 (I2C & DMESG):**

      * `dmesg` 로그에 `sl_max96712: Link Locked` 또는 `GMSL2 Link Detected` 같은 **성공 메시지가 전혀 없습니다.**
      * 드라이버는 로드되었지만, 캡처 카드가 카메라와 **전기적으로 통신을 못 하고 있는 상태**입니다.

-----

### 2\. 해결 방법 (하드웨어 점검 필수)

이 문제는 90% 이상의 확률로 다음 세 가지 중 하나입니다. 순서대로 확인해주세요.

#### ① 외부 전원 연결 확인 (가장 유력)

Jetson Orin AGX의 PCIe 슬롯 전력만으로는 **ZED X 4대를 구동하기에 부족**할 수 있습니다.

  * **체크:** ZED Link Quad 캡처 카드 옆면에 있는 \*\*외부 전원 커넥터(Molex 4-pin 또는 6-pin PCIe 파워)\*\*에 전원 케이블을 꽂으셨나요?
  * **조치:** 반드시 파워 서플라이의 전원을 캡처 카드에 직접 연결해야 합니다.

#### ② GMSL2 케이블 방향 (가장 흔한 실수)

ZED X의 GMSL2 케이블은 \*\*방향성(Directional)\*\*이 있습니다. 반대로 꽂으면 전기는 흐르지만 데이터 통신이 안 됩니다.

  * **체크:** 케이블 양끝에 라벨이 붙어 있습니다.
      * **`Host`** (또는 Server) 라벨 쪽 ➡ \*\*캡처 카드(Jetson)\*\*에 연결
      * **`Camera`** 라벨 쪽 ➡ **ZED X 카메라**에 연결
  * **조치:** 케이블 방향이 맞는지 4개 모두 확인하고, "딸깍" 소리가 날 때까지 꽉 끼워졌는지 확인하세요.

#### ③ 카메라 포트 순서 (Port A부터 사용)

  * **체크:** 카메라를 랜덤하게 꽂으셨나요?
  * **조치:** 드라이버는 보통 Port A(1번)부터 순차적으로 탐색합니다. 테스트를 위해 **Port A에 카메라 1대만 연결**하고(나머지는 제거), Reboot 후 인식되는지 확인하세요.

-----

### 3\. 검증 방법

하드웨어 조치 후 Reboot을 하고 터미널에서 아래 명령어를 입력했을 때, 다음과 같은 메시지가 나와야 정상입니다.

1.  **커널 메시지 확인:**

    ```bash
    sudo dmesg | grep max96712
    ```

      * **정상:** `Link 0 locked` , `Video Link detected` 같은 문구가 보여야 합니다.
      * **현재 상태:** 위 문구가 없어서 실패한 것입니다.

2.  **비디오 장치 확인:**

    ```bash
    ls /dev/video*
    ```

      * `/dev/video0`, `/dev/video1` ... 등이 보여야 성공입니다.

### 요약

소프트웨어는 건드리지 마세요. (설치 잘 되었습니다).
\*\*"외부 전원 연결"\*\*과 \*\*"케이블 방향(Host/Camera)"\*\*을 하드웨어적으로 다시 점검하시면 해결될 것입니다.
##############№#  
################  
################  
핵심부터 말하면,
**드라이버/SDK 설치는 거의 정상인데, Stereolabs용 DTB 오버레이가 실제 부팅에 안 올라와서 ZED Link Quad 카드가 “하드웨어로” 안 보이는 상태**예요.
그래서 `ZED_Diagnostic` 에서도 ZED X 카메라를 못 찾는 겁니다.

아래에서 왜 그렇게 판단했는지 + 바로 해볼 수 있는 해결 절차까지 정리해줄게요.

---

## 1. 로그에서 보이는 것들 정리

### (1) 소프트웨어 버전은 호환 범위 안에 있음

* ZED SDK: 5.1.1
* JetPack: R36.4.x (로그에 `R36 (release), REVISION: 4.7` → JetPack 6.2 계열)
* ZED Link Quad Driver: `stereolabs-zedlink-quad 1.3.2-SL-MAX96712-all-L4T36.4.0`

Stereolabs 드라이버 페이지에서 v1.3.2는 **L4T 36.4 / JetPack 6.1 & 6.2, 6.2.1 (36.4.4, 36.4.7 포함)** 을 공식 지원한다고 되어 있습니다.([stereolabs.com][1])

즉, “버전 호환 문제”는 아닌 걸로 봐도 됩니다.

---

### (2) 커널 모듈과 데몬은 잘 올라와 있음

로그 중:

```text
####################################
############ Driver list ###########
####################################

sl_zedxone_uhd 36864 0
sl_zedx 28672 0
sl_zedxpro 20480 0
sl_max9295 16384 3 sl_zedxpro,sl_zedxone_uhd,sl_zedx
sl_max96712 24576 4 sl_zedxpro,sl_zedxone_uhd,sl_max9295,sl_zedx
tegra_camera 217088 6 sl_zedxpro,nvhost_isp5,nvhost_nvcsi_t194,sl_zedxone_uhd,nvhost_vi5,sl_zedx
videodev 249856 7 v4l2_async,sl_zedxpro,videobuf2_v4l2,tegra_camera,sl_zedxone_uhd,videobuf2_common,sl_zedx
```

그리고

```text
● zed_x_daemon.service - ZED-X Daemon service
Active: active (running)
```

→ 드라이버 `.ko` 들은 로드되어 있고, `zed_x_daemon` 도 실행 중이라 **“소프트웨어 설치/서비스 기동” 단계는 통과**한 상태입니다.

---

### (3) 진짜 문제: 드라이버가 “하드웨어에 붙질 않았다”

#### 3-1. DMESG 섹션이 비어 있음

로그에서:

```text
###########################################
############ DMESG [ZEDX] LOG #############
###########################################

(아무 내용 없음)

###########################################
############ DMESG [MAX9x] LOG #############
###########################################

[ 14.789634] sl_max96712: module verification failed: signature and/or required key missing - tainting kernel
```

* 정상이라면 여기에는
  `sl_max96712 30-00xx: probing...`
  `sl_zedx 2-003b: ZED X camera detected`
  같은 메시지가 여러 줄 찍혀야 합니다.([Stereolabs Forums][2])
* 그런데 지금은 **모듈이 로드될 때 한 번 찍히는 “signature missing” 로그만 있고, 실제 I2C 디바이스를 찾았다는 로그가 하나도 없음.**

> `module verification failed ... tainting kernel`
> → 이건 *서명 없는 외부 커널 모듈*을 올릴 때 항상 나오는 경고일 뿐이고, **오류가 아니라 그냥 “커널이 외부 모듈을 허용했다”는 표시**라서 원인은 아닙니다.

즉,

* 모듈은 올라왔는데
* 붙어야 할 I2C 디바이스(= MAX96712, MAX9295, 카메라들)가 **DTB 상에 존재하지 않거나, I2C 상에서 안 보이는 상태**라는 뜻입니다.

#### 3-2. I2C 버스 번호도 이상함

로그:

```text
############ I2C Trace [-l] #############

i2c-0 ... 
...
i2c-9 ...
```

그리고 각 `/dev/i2c-0` ~ `/dev/i2c-8`만 스캔 결과가 있고, **i2c-30, i2c-31 같은 GMSL용 버스는 전혀 없음**.

ZED Link Quad + ZED X가 제대로 붙으면, Stereolabs 쪽 예시 로그에서는 보통 `/dev/i2c-9`, `/dev/i2c-10`, `/dev/i2c-13` 이런 추가 버스들이 보이고, 거기에 카메라가 매핑됩니다.([Stereolabs Forums][2])

지금 상태는:

* 드라이버는 올라와 있는데
* **GMSL2 카드용 I2C 노드/버스 자체가 DTB에서 정의되지 않음 → 실제 하드웨어와 바인딩이 안됨 → ZED_Diagnostic에서 카메라 0대처럼 보이는 상태.**

---

### (4) extlinux.conf를 보면 DT 오버레이가 “정의만 되어 있고, 실제로 부팅할 때 안 쓰는 상태”

당신이 준 `extlinux` 부분:

```text
TIMEOUT 30
DEFAULT primary

MENU TITLE L4T boot options

LABEL primary
MENU LABEL primary kernel
LINUX /boot/Image
INITRD /boot/initrd
APPEND ${cbootargs} root=/dev/nvme0n1p1 rw ...

LABEL Stereolabs
MENU LABEL Stereolabs kernel
LINUX /boot/Image
FDT /boot/dtb/kernel_tegra234-p3737-0000+p3701-0005-nv.dtb
INITRD /boot/initrd
APPEND ${cbootargs} root=/dev/nvme0n1p1 rw ...
OVERLAYS /boot/tegra234-p3737-camera-zedlink-quad-sl-overlay.dtbo
```

포인트:

* **Stereolabs가 설치해준 부팅 엔트리 (`LABEL Stereolabs`)는 잘 들어가 있음.**
* 하지만 `DEFAULT primary` 라고 되어 있어서,
  Jetson이 부팅할 때 항상 **`primary` 엔트리로 부팅**하고 있습니다.
* `primary` 엔트리는 `OVERLAYS ...zedlink-quad-sl-overlay.dtbo` 설정이 없기 때문에,
  **ZED Link Quad용 카메라/serializer/deserializer DT 오버레이가 전혀 적용되지 않음.**

→ 그래서

* `sl_max96712` 모듈은 로드되지만
* 붙을 I2C 디바이스가 DT에 없으니 probe가 안 되고,
* DMESG [ZEDX] / [MAX9x]는 텅 비어 있고,
* I2C 버스도 0~9만 있는 상황입니다.

이게 **지금 ZED 카메라 감지 실패의 핵심 원인**이에요.

---

## 2. 바로 시도해볼 해결 방법 (우선순위 순서대로)

### 1단계: 부팅 기본 엔트리를 Stereolabs로 바꾸기

1. Jetson에서:

   ```bash
   sudo nano /boot/extlinux/extlinux.conf
   ```

2. 맨 위 부분을 찾아서

   ```text
   DEFAULT primary
   ```

   를

   ```text
   DEFAULT Stereolabs
   ```

   로 바꾼 뒤 저장(+ 종료).

3. Jetson 재부팅:

   ```bash
   sudo reboot
   ```

4. 재부팅 후, 다시 진단 실행:

   ```bash
   sudo ZED_Diagnostic --dmesg
   ```

   여기서 기대하는 변화:

   * `DMESG [MAX9x] LOG` 쪽에

     ```text
     sl_max96712 30-0029: probing...
     sl_max9295 2-0048: ...
     ```

     같은 메시지들이 여러 줄 나와야 합니다.
   * `DMESG [ZEDX] LOG` 쪽에도

     ```text
     sl_zedx 2-003b: ZED X camera detected
     ```

     같은 로그가 보여야 해요.
   * `I2C Trace [-l]` 부분에 0~9뿐 아니라, 10번 이후 (예: i2c-30 등) 버스가 새로 생길 수 있습니다.

5. 그 다음, 카메라 인식 체크:

   ```bash
   ZED_Explorer -a
   ```

   * 정상일 경우:

     ```text
     ## Cam 0
     Model : "ZED X"
     S/N   : ...
     State : "AVAILABLE"
     Path  : /dev/i2c-9
     Type  : "GMSL"
     ...
     ```

   * 이렇게 ZED X들이 리스트에 떠야 `ZED SDK` 레벨에서 카메라 인식이 된 것입니다.([Stereolabs Forums][2])

---

### 2단계: 하드웨어 기본 점검 (혹시라도 안 될 때)

위 1단계까지 했는데도 여전히 `ZED_Explorer -a`에 아무것도 안 뜨면, 그때는 **하드웨어 쪽**을 의심해야 합니다.

1. **ZED Link Quad 카드 전원**

   * 이 카드는 PCIe만으로는 카메라 전원을 공급할 수 없고,
     보통 12V (또는 권장 범위) DC 잭으로 별도 전원을 넣어줘야 합니다.([stereolabs.com][1])
   * 카드 상의 파워 LED가 켜져 있는지, 전원 어댑터가 제대로 연결되어 있는지 확인.

2. **카메라–카드 간 GMSL2 케이블**

   * 각 ZED X의 FAKRA 커넥터가 ZED Link Quad의 GMSL 포트에 단단히 연결되어 있는지 확인.
   * 가능하면, **카메라 1대만 연결한 상태에서 테스트**해서 배선 문제인지도 체크하면 좋습니다.
   * Stereolabs 포럼에서도, 특정 카메라나 케이블만 문제가 있었던 사례들이 꽤 있습니다.([Stereolabs Forums][2])

3. **카메라 연결 후 재부팅 / 데몬 재시작**

   * 모든 카메라를 연결한 상태에서 Jetson을 재부팅하거나,

   * 부팅 후에 다음 명령으로 데몬 재시작:

     ```bash
     sudo systemctl restart zed_x_daemon
     ZED_Explorer -a
     ```

   * 일부 케이스에서 데몬 재시작 후에야 카메라 상태가 `AVAILABLE`로 바뀌는 경우도 보고되었습니다.([Stereolabs Forums][2])

---

## 3. 정리해서 한 줄로 말하면…

* **SDK/드라이버 버전, JetPack 버전은 문제 없고**,
* 로그상으로 보면 **ZED Link Quad용 DTB 오버레이가 실제 부팅에 적용되지 않아서, 드라이버가 하드웨어(GMSL2 칩·카메라)에 전혀 붙지 못하는 상태**입니다.
* `DEFAULT Stereolabs` 로 바꿔서 Stereolabs 엔트리로 부팅시키면, 대부분의 경우 ZED_Diagnostic 에 ZED X 가 잡힐 거예요.

---

원하는 거면,
`ZED_Diagnostic --dmesg` 결과를 한 번 더 보내주면
“지금은 어디까지 성공했고, 다음에 뭐를 만져야 되는지” 단계별로 다시 디버깅해줄게!

[1]: https://www.stereolabs.com/en-mt/developers/drivers "Stereolabs ZED X GMSL2 Drivers | Stereolabs"
[2]: https://community.stereolabs.com/t/zed-x-one-gs-camera-not-detected/9541 "ZED X One GS camera not detected - Stereolabs Forums"
