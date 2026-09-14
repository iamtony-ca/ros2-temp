요청하신 항목 구성으로 5개 제품 전체를 재작성했습니다. 대괄호 숫자는 출처 번호입니다.

**전체 비교표**

| 항목 | SIYI A8 mini | Topotek DIY10S90 | OBSBOT Tail Air | Axis M5075-G | 한화 QNP-6250R |
|---|---|---|---|---|---|
| **해상도** | 4K 녹화 / 1080p 스트림 [1][8] | 1080p 30fps RTSP [3] | 4K 30fps / 1080p 60fps [5] | 1920×1080, 최대 60fps [14] | 1080p, H.265/264 최대 60fps [11] |
| **Zoom (광학)** | 없음 [1] | **10배** (5.15~47.38mm) [3] | 없음 [5] | 5배 (2.2~11.0mm) [14] | **25배** (4.44~111mm) [11] |
| **Zoom (디지털)** | 6배 [1] | — | 4배 [5] | 12배 [14] | 32배 (합계 800배) [11] |
| **Sensor** | 1/1.7" Sony 8MP [1] | 1/2.8" 2MP WDR CMOS [3] | 1/1.8" 8.4MP (2μm) [5] | 1/2.8" progressive RGB CMOS [14] | 1/2.8" CMOS 2MP [11] |
| **Pan** | 미확인 | -285°~+285° [3] | ±150° (기계 ±160°) [5] | ±179°, 100°/s [14] | **360° 무한회전** [11] |
| **Tilt** | -90°~+25° [8] ※웹 -135°~+45° | **-45°~+120°** [3] | -65°~+32° (기계 ±90°) [5] | 90° (기준선 불명) [14] | **-20°~+90°** [11] |
| **Interface** | Ethernet/HDMI/CVBS, S.Bus·UART·UDP [1] | IP(RTSP) + UART [3] | USB-C/Ethernet(어댑터)/HDMI/Wi-Fi [4] | RJ45 10/100BASE-TX PoE [14] | RJ45 10/100BASE-T PoE+ [11] |
| **전원 (소비전력)** | DC 11~25.2V, 평균 5W/피크 12W [1][8] | DC 12~26.2V, 5W [3] | USB-C / PoE [4] | **PoE 802.3af/at Class3, 일반 4.4W/최대 9.5W** [14] | PoE+ Class4, 최대 25.5W/일반 14.7W [11] |
| **사이즈 (mm)** | 55×55×70 [2] | φ150 × H140 [3] | 69.65×73.25×132.5 [5] | φ130 × 63 [14] | φ158 × 293.3 [11] |
| **무게 (kg)** | **0.095** [1] | 0.425 [3] | 0.345 [5] | 0.380 [14] | **3.1** [11] |
| **흔들림 보정** | 3축 기계식 짐벌 [1] | 3축 짐벌 (±0.02°) [3] | 2축 짐벌 + 6축 자이로 [5] | **없음** [14] | 자이로 기반 DIS (전자식) [11] |
| **SW (기본 GUI)** | UniGCS, SIYI QGC (Win/Android) [8] | PC network GCS (Win) [9] | OBSBOT Start / WebCam [4] | **내장 웹뷰어** + AXIS Companion [14] | **내장 웹뷰어** [11] |
| **SW (SDK/API)** | SIYI Gimbal SDK (UDP/TCP 37260) [8] | UDP&UART 프로토콜 + **Python 샘플** [9] | Win/**Linux**/macOS SDK, VISCA over IP [10] | **VAPIX + ACAP**, ONVIF G/M/S/T [14] | **SUNAPI(HTTP) + ONVIF S/G/T** [11] |
| **가격 (만원)** | **36~42** ($253~299) [2][13] | **82** ($582) [3] | **70** ($499) [4] | **89~101** ($634~722) [15][16][17] | 미확인 |
| **연속 운용** | **8시간 제한** (30분 냉각) [8] | 미확인 | 미확인 | 24시간 (0~45°C, IP51) [14] | 24시간 (-35~+55°C, IP66) [11] |
| **비고** | 최경량·최저가, 문서화 최고 수준. 상향 틸트와 줌이 약점 | 광학줌+상향 틸트 동시 충족. GUI 모델 대응 불명확, mega.nz 배포 | 광학줌 없음, 2축. Ubuntu SDK 유일 제공 | **최저 소비전력**, 프리셋 256개, **5년 보증**. 판매지역 제한(미·캐·일·유럽), PoE 전용 | IP66/IK10, 프리셋 300개·±0.2°, NDAA 등재 [12]. 3.1kg 무게가 관건 |

※ 환율 1,400원/달러 기준, 배송·관세 별도

**항목별로 본 강약**

**줌** — 한화 25배 > Topotek 10배 > Axis 5배 > A8 mini·Tail Air(광학 없음). OHT 세부 판독이 목적이면 이 순서가 그대로 우선순위가 됩니다.

**상향 틸트** — Topotek +120° > 한화 +90° > Axis 90°(기준 불명) > Tail Air +32° > A8 mini +25°. Axis만 데이터시트에 기준선 표기가 없어 확인이 필요합니다.

**AMR 탑재 적합성(무게·전력)** — A8 mini 95g·5W와 Axis 380g·4.4W가 가장 유리하고, 한화 3.1kg·14.7W가 가장 부담입니다.

**연속 운용** — Axis와 한화만 24시간 설계가 확인됐습니다. A8 mini의 8시간 제한은 상시 모니터링 용도에서 실질적 탈락 사유가 될 수 있습니다.

**미확인 항목**

| 대상 | 확인할 것 |
|---|---|
| SIYI | 틸트 값 불일치, Pan 범위, upside down 실제 상향각, 8시간 제한 |
| Topotek | 적용 프로토콜 문서, 전용 GUI, 연속 운용, 기본 IP/RTSP |
| OBSBOT | 상향 틸트 SDK 확장 가능성, 연속 운용 |
| Axis | **역방향 설치 시 천장 관측 가능 여부**, 한국 판매 여부 |
| 한화 | **국내 견적**, 경량 실내 모델 유무, 역방향 설치 |

**출처**

1. [SIYI A8 mini 사양 (ARRIS Hobby)](https://www.arrishobby.com/products/siyi-a8-mini-4k-8mp-ultra-hd-6x-digital-zoom-camera-with-3-axis-gimbal)
2. [A8 mini 크기·무게 (MotioNew)](https://www.motionew.com/shop/gimbal-and-payload/siyi-a8-mini/)
3. [Topotek DIY10S90 공식 제품 페이지](https://topotek.com/DIY10S90-10x-Optical-zoom-IRCUT-3-Axis-Gimbal-IP-output-p5832404.html)
4. [OBSBOT Tail Air 공식](https://www.obsbot.com/obsbot-tail-air-streaming-camera) / [B&H (PoE·어댑터)](https://www.bhphotovideo.com/c/product/1793976-REG/obsbot_owb_taaiptz_tail_air_ai_powered_ptz.html)
5. [Tail Air 상세 스펙 (The Gadgeteer)](https://the-gadgeteer.com/2025/02/02/obsbot-tail-air-review-a-ptz-camera-with-no-strings-cables-attached/)
8. [SIYI A8 mini 매뉴얼 v1.10 (공식)](https://res.siyi.biz/oss/other/2026/06/15/A8_mini_User_Manual_v1_10_563cde30.pdf)
9. [Topotek 다운로드 리소스 (공식)](https://topotek.com/pages/TOPOTEK-Download-Resources-11763444.html)
10. [OBSBOT SDK 신청 페이지 (공식)](https://www.obsbot.com/sdk)
11. [한화 QNP-6250R 공식 데이터시트 Ver 202402](https://www.networkwebcams.co.uk/content/pdf/hanwha-wisenet/wisenet-qnp-6250r-datasheet.pdf)
12. [한화비전 미주 QNP-6250R (NDAA)](https://hanwhavisionamerica.com/product/qnp-6250r/)
13. [A8 mini $253 (insightfpv)](https://insightfpv.com/products/siyi-a8-mini-gimbal-camera)
14. **[Axis M5075-G 공식 데이터시트 PDF](https://www.axis.com/dam/public/ef/95/c9/datasheet-axis-m5075-g-ptz-camera-en-US-397908.pdf)** / [판매지역 안내](https://www.axis.com/products/axis-m50-series)
15. [Axis $634 (Tech-America)](https://www.tech-america.com/item/axis-m5075-g-2-megapixel-full-hd-network-camera/02347-004)
16. [Axis $694.99 (IP Phone Warehouse)](https://www.ipphone-warehouse.com/axis-m5075-g-1080p-ptz-z-wave-indoor-mini-dome-ip-camera-02347-004/)
17. [Axis $722.35 (A1 Security Cameras)](https://www.a1securitycameras.com/axis-m5075-g-us-2mp-indoor-mini-ptz-camera-with-5x-optical-zoom-z-wave-and-wireless-in-out-built-in-microphone-02347-004.html)

11번과 5번은 제조사 공식 사본이 아닌 리셀러·리뷰 자료입니다. 14번(Axis)과 3번(Topotek), 8번(SIYI)은 제조사 공식 문서를 직접 확인한 값입니다.
