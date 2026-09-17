사양표를 받아 TS-HD10S 칸을 모두 채웠습니다. **결정적인 사실 두 가지가 확인됐습니다.**

## 먼저 핵심 발견

**① 상향 틸트 +90도 — 요구사항 충족**

팬틸트 각도가 **수평 ±178도 / 수직 -30도 ~ +90도**입니다. 천장 OHT 관측에 충분합니다. 게다가 설치 방식에 **천장 / 벽걸이 / 삼각대 / 책상**이 모두 명시되고 **Mirror & Flip을 지원**하므로, AMR 상부에 세워 설치하는 구성도 문제없습니다. 세 후보 중 **상향 틸트가 공식 문서로 확정된 유일한 제품**입니다.

**② 내장 웹GUI — 별도 소프트웨어 불필요**

웹GUI에서 카메라 컨트롤, 비디오 미리보기, MP4 녹화, 스틸 이미지, 프리셋을 모두 지원합니다. 브라우저만으로 실시간 모니터링과 제어가 됩니다. 프로토콜은 **UVC / PELCO-D,P / VISCA / ONVIF**로, 앞서 말씀드린 대로 모두 공개 표준입니다.

## 최종 비교표

| 항목 | SIYI A8 mini | **PIXEL TS-HD10S** | AXIS M5075 |
|---|---|---|---|
| **해상도(스트림)** | 최대 1080p [1] | **1080P60/50/30/25, 1080I, 720P** [6] | 1920×1080, 최대 50/60fps [3] |
| **해상도(녹화)** | 4K 3840×2160 @25fps [1] | MP4 녹화 (웹GUI) [6] | — |
| **Zoom (광학)** | **없음** [1] | **10배** (f=5.1~51.0mm) [6] | 5배 (2.2~11.0mm) [3] |
| **Zoom (디지털)** | 6배 (4K 불가) [1] | 12배 [6] | 12배 [3] |
| **Sensor** | Sony 1/1.7", 8MP, F2.8 [1] | 1/2.8" Exmor CMOS, 2.21MP [6] | 1/2.8" progressive RGB CMOS [3] |
| **Pan** | -135°~+135° [1][5] | **±178°** [6] | ±179°, 100°/s [3] |
| **Tilt** | -90°~**+25°** [1][5] | **-30°~+90°** [6] | 90° (기준선 미표기) [3] |
| **화각(수평)** | 81° 고정 [1] | 72° / 2.9° (W/T 표기 확인 필요) [6] | 14°~71° [3] |
| **최저조도** | 스타라이트 CMOS [1] | 0.5Lux / 0.095Lux [6] | — |
| **Interface** | Ethernet/CVBS/Micro-HDMI [1] | **3G-SDI / HDMI / USB2.0 / IP / LAN** [6] | RJ45 10/100BASE-TX PoE [3] |
| **전원 (소비전력)** | DC 11~25.2V, 평균 5W / 최대 12W [1] | **DC 12V 1A (12W) / PoE+** [6] | PoE Class3, 일반 4.2W / 최대 9.3W [3] |
| **사이즈 (mm)** | 55×55×70 [1] | 120×120×145 [6] | φ130 × 63 [3] |
| **무게 (kg)** | **0.095** [1] | **1.4** [6] | 0.380 [3] |
| **흔들림 보정** | **3축 기계식 짐벌** (±0.01°) [1] | 없음 (DNR 2D&3D만) [6] | 없음 [7] |
| **동작 온도** | -10~50°C (IP4X) [1] | -10~50°C [6] | 0~45°C (IP51) [3] |
| **SW (기본 GUI)** | UniGCS, SIYI QGC (Win/Android) [1] | **내장 웹GUI** (제어·미리보기·녹화·프리셋) [6] | 내장 웹뷰어 + Camera Station [3] |
| **SW (SDK/API)** | SIYI SDK Protocol v0.1.1 (UDP/TCP) [5] | **ONVIF / VISCA / PELCO-D,P / UVC** [6] | VAPIX + ACAP, ONVIF G/M/S/T [3] |
| **통신** | S.Bus / UART / Ethernet [1] | UVC / RS-485 / RS-232C / IP / 리모컨 [6] | IP (PoE) [3] |
| **IP 프로토콜** | RTSP [1] | RTSP, RTMP, HTTP, UDP, SNMP 등 [6] | RTSP, ONVIF [3] |
| **프리셋** | — | 256개 [6] | 256개 [3] |
| **가격 (만원)** | 36~42 [8][9] | **79** [4] | 미확인 (-G 기준 89~101) [10] |
| **연속 운용** | **8시간 제한** [1] | 미확인 | 24시간 [3] |
| **국내 정식 판매** | 미확인 | **국내 업체** [4] | KR 품번 02346-001 [7] |

## 배선 비교

| 항목 | SIYI A8 mini | PIXEL TS-HD10S | AXIS M5075 |
|---|---|---|---|
| 전원 | SMPS → 카메라 직결 [1] | **SMPS → 카메라 직결** (DC12V, 정품 아답터 동봉) [6] | SMPS → 미드스팬 → PoE [11] |
| 데이터 | RJ45 변환 케이블 → EDS-205 [1] | **LAN(RJ45) → EDS-205 직결** [6] | 미드스팬 경유 → EDS-205 [11] |
| 케이블 수 | 2가닥 | **2가닥** | 3가닥 |
| 추가 구매품 | 없음 | **없음** (브라켓·케이블·리모컨 동봉) [6] | 미드스팬 02208-001 [11] |
| 12V 전류 | 평균 0.42A / 피크 1.0A (계산) | **1.0A (정격)** [6] | 약 0.41A / 최대 0.91A (계산) |
| PoE 대안 | 불가 | **PoE+ 지원** [6] | PoE 전용 |

## 전원 관련 주의 (중요)

TS-HD10S 정격이 **DC 12V 1A**인데, 현재 가용 여유가 1A입니다. **여유분이 전혀 없습니다.** PTZ 구동 시 기동 전류를 감안하면 위험합니다. 두 가지 선택지가 있습니다.

- 다른 전원을 빼서 **1.5~2A 확보** 후 DC 직결
- **PoE+로 급전** — 앞서 검토한 AXIS 30W Midspan AC/DC(10~28V DC 입력)를 그대로 쓸 수 있습니다. 이 경우 12V 부담은 미드스팬 쪽으로 옮겨가지만 총 전력은 비슷하니, 결국 전원 여유 확보가 필요합니다

## 세 제품의 성격 정리

**TS-HD10S** — 상향 +90도, 광학 10배, 내장 웹GUI, ONVIF 표준, 79만원, 국내 A/S. 요구사항 대부분을 충족하는 유일한 제품입니다. 약점은 **1.4kg 무게**(A8 mini의 15배)와 **흔들림 보정 부재**, 그리고 연속 운용 미확인입니다.

**A8 mini** — 95g에 3축 짐벌로 주행 중 촬영에는 압도적이지만, **상향 +25도**로 천장 관측이 사실상 불가능하고 8시간 제한이 있습니다.

**M5075** — 문서화와 API 품질, 저소비전력(4.2W), 380g이 장점이나 **광학 5배로 가장 약하고**, 틸트 기준선이 미확인이며 미드스팬이 추가로 필요합니다.

## 남은 확인 항목

| 대상 | 확인할 것 |
|---|---|
| PIXEL | **24시간 연속 운용 가능 여부**, 수평 화각 W/T 표기(72°/2.9°가 뒤바뀐 듯), 웹GUI 접속 방식과 기본 IP |
| SIYI | Upside Down 모드 실제 상향각 (+25도 한계 재확인) |
| AXIS | 틸트 기준선, 국내 공급가 |

**제 판단으로는 TS-HD10S가 현재 1순위**입니다. 천장 관측이라는 핵심 요구사항을 문서로 충족하는 유일한 제품이고, 가격과 A/S도 유리합니다. 무게 1.4kg가 AMR 상부 탑재에 문제없는지, 그리고 주행 중이 아닌 정지 상태 관측으로 운용 가능한지만 확인되면 됩니다.

**출처**

1. [SIYI A8 mini 매뉴얼 v1.10 (제조사 공식)](https://res.siyi.biz/oss/other/2026/06/15/A8_mini_User_Manual_v1_10_563cde30.pdf)
3. [AXIS M5075 데이터시트 202511 (제조사 공식)](https://www.axis.com/dam/public/d1/6d/a7/datasheet-axis-m5075-ptz-camera-en-US-506699.pdf)
4. [PIXEL TS-HD10S 790,000원 (다나와)](https://search.danawa.com/dsearch.php?query=%ED%92%80HD+%EC%BA%A0%EC%BD%94%EB%8D%94)
5. [SIYI Gimbal Camera External SDK Protocol v0.1.1 (제조사 공식)](https://siyi.biz/siyi_file/A8%20mini/SIYI_Gimbal_Camera_External_SDK_Protocol_Update_Log%20V0.1.1.pdf)
6. **PIXEL TS-HD10S 제품 사양표 (판매처 자료, Tony 님 제공 이미지)**
7. [AXIS M5075 제품 페이지 (제조사 공식)](https://www.axis.com/products/axis-m5075)
8. [A8 mini 크기·무게 (MotioNew)](https://www.motionew.com/shop/gimbal-and-payload/siyi-a8-mini/) / 9. [A8 mini $253 (insightfpv)](https://insightfpv.com/products/siyi-a8-mini-gimbal-camera) / 10. [AXIS M5075-G $694.99 (IP Phone Warehouse)](https://www.ipphone-warehouse.com/axis-m5075-g-1080p-ptz-z-wave-indoor-mini-dome-ip-camera-02347-004/) / 11. [AXIS 30W Midspan AC/DC (공식)](https://www.axis.com/products/axis-30-w-midspan-acdc)
