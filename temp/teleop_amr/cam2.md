요청하신 항목으로 전면 재작성했습니다. 대괄호 숫자는 출처 번호입니다.

**전체 비교표**

| 항목 | SIYI A8 mini | Topotek DIY10S90 | OBSBOT Tail Air | 한화 QNP-6250R |
|---|---|---|---|---|
| **해상도** | 4K 녹화 / 1080p 스트림 [1][8] | 1080p 30fps RTSP [3] | 4K 30fps / 1080p 60fps [5] | 1080p 60fps [11] |
| **Zoom (광학)** | 없음 [1] | **10배** (f=5.15~47.38mm) [3] | 없음 [5] | **25배** (f=4.44~111mm) [11] |
| **Zoom (디지털)** | 6배 [1] | — | 4배 [5] | 32배 (합계 800배) [11] |
| **Sensor** | 1/1.7" Sony 8MP [1] | 1/2.8" 2MP WDR CMOS [3] | 1/1.8" CMOS 8.4MP (2μm) [5] | 1/2.8" CMOS 2MP [11] |
| **Pan** | 미확인 | -285°~+285° [3] | ±150° (기계 ±160°) [5] | **360° 무한회전** [11] |
| **Tilt** | -90°~+25° [8] ※웹 표기 -135°~+45° | **-45°~+120°** [3] | -65°~+32° (기계 ±90°) [5] | **-20°~+90°** [11] |
| **Interface** | Ethernet/HDMI/CVBS, 제어 S.Bus/UART/UDP [1] | IP(RTSP) + UART [3] | USB-C/Ethernet(어댑터)/HDMI/Wi-Fi, PoE [4] | RJ-45 10/100BASE-T [11] |
| **전원 (소비전력)** | DC 11~25.2V, 평균 5W / 피크 12W [1][8] | DC 12~26.2V, 5W [3] | USB-C / PoE [4] | **PoE+ (802.3at Class4)**, 최대 25.5W / 일반 14.7W [11] |
| **사이즈 (mm)** | 55×55×70 [2] | φ150 × H140 [3] | 69.65×73.25×132.5 [5] | φ158 × 293.3 [11] |
| **무게** | 0.095kg [1] | 0.425kg [3] | 0.345kg [5] | **3.1kg** [11] |
| **흔들림 보정** | **3축 기계식 짐벌** [1] | **3축 짐벌** (지터 ±0.02°) [3] | 2축 짐벌 + 6축 자이로 [5] | 자이로 기반 DIS (전자식) [11] |
| **기본 GUI** | UniGCS, SIYI QGC (Win/Android) [8] | PC network GCS (Win) [9] | OBSBOT Start / WebCam [4] | **내장 웹뷰어** (브라우저) [11] |
| **SDK/API** | SIYI Gimbal SDK (UDP/TCP 37260) [8] | UDP&UART 프로토콜 + **Python 샘플** [9] | **Win/Linux/macOS SDK**, VISCA over IP [10] | **SUNAPI(HTTP API) + ONVIF S/G/T** [11] |
| **가격 (만원)** | **약 36~42** ($253~299) [2][13] | **약 82** ($582) [3] | **약 70** ($499) [4] | 미확인 |
| **연속 운용** | **8시간 제한** (이후 30분 냉각) [8] | 미확인 | 미확인 | **24시간** (-35~+55°C) [11] |
| **비고** | 최경량·최저가. 문서화 최고 수준. 상향 틸트 부족, 디지털줌 한계 | 광학줌+상향 +120° 동시 충족. GUI 모델 대응 불명확, mega.nz 배포 | 광학줌 없음, 2축 짐벌. Ubuntu SDK 유일 제공 | IP66/IK10, 프리셋 300개·정확도 ±0.2°, 802.1X/HTTPS, NDAA 등재 [11][12]. **3.1kg 무게가 관건** |

※ 환율 1,400원/달러 기준 환산, 배송·관세 별도

**재확인 과정에서 드러난 중요한 점**

한화 QNP-6250R 공식 데이터시트를 직접 확인한 결과 두 가지가 명확해졌습니다.

첫째, **틸트 110도(-20°~+90°)가 공식 데이터시트에 확정 기재**되어 있습니다 [11]. 천장 OHT 관측에 문제없습니다.

둘째, **무게 3.1kg에 길이 293mm**입니다 [11]. A8 mini(95g) 대비 32배입니다. AMR 상부 탑재 시 하중과 무게중심, 장착 구조를 별도로 검토해야 합니다. 앞서 "Q 시리즈가 경량화됐다"고 한 건 기존 PTZ 대비 상대적인 것이지, 절대 무게는 여전히 큽니다.

셋째, Ethernet이 **10/100BASE-T**입니다 [11]. 앞서 논의한 EDS-205와 규격이 맞습니다.

**미확인 항목**

| 대상 | 확인할 것 |
|---|---|
| SIYI | 틸트 값 불일치(매뉴얼 vs 웹), upside down 모드 실제 상향각, Pan 범위, 8시간 제한 |
| Topotek | DIY10S90 적용 프로토콜 문서 버전, 전용 GUI, 연속 운용, 기본 IP/RTSP 주소 |
| OBSBOT | 연속 운용, SDK로 제어 범위를 기계적 한계(+90°)까지 확장 가능한지 |
| 한화 | **국내 견적**, AMR 탑재용 장착 브래킷, PoE+ 급전 방식 |

**출처**

1. [SIYI A8 mini 사양 (ARRIS Hobby)](https://www.arrishobby.com/products/siyi-a8-mini-4k-8mp-ultra-hd-6x-digital-zoom-camera-with-3-axis-gimbal)
2. [SIYI A8 mini 크기·무게 (MotioNew)](https://www.motionew.com/shop/gimbal-and-payload/siyi-a8-mini/)
3. [Topotek DIY10S90 공식 제품 페이지](https://topotek.com/DIY10S90-10x-Optical-zoom-IRCUT-3-Axis-Gimbal-IP-output-p5832404.html)
4. [OBSBOT Tail Air 공식](https://www.obsbot.com/obsbot-tail-air-streaming-camera) / [B&H (PoE·Ethernet 어댑터)](https://www.bhphotovideo.com/c/product/1793976-REG/obsbot_owb_taaiptz_tail_air_ai_powered_ptz.html) / [가격 $499 (RedShark)](https://www.redsharknews.com/save-xxx-on-the-excellent-obsbot-tail-air-4k-streaming-camera)
5. [Tail Air 상세 스펙 (The Gadgeteer)](https://the-gadgeteer.com/2025/02/02/obsbot-tail-air-review-a-ptz-camera-with-no-strings-cables-attached/)
8. [SIYI A8 mini 매뉴얼 v1.10 (공식)](https://res.siyi.biz/oss/other/2026/06/15/A8_mini_User_Manual_v1_10_563cde30.pdf)
9. [Topotek 다운로드 리소스 (공식)](https://topotek.com/pages/TOPOTEK-Download-Resources-11763444.html)
10. [OBSBOT SDK 신청 페이지 (공식)](https://www.obsbot.com/sdk)
11. **[한화 QNP-6250R 공식 데이터시트 Ver 202402 (PDF)](https://www.networkwebcams.co.uk/content/pdf/hanwha-wisenet/wisenet-qnp-6250r-datasheet.pdf)** — 이번 표의 한화 수치 대부분의 근거
12. [한화비전 미주 QNP-6250R (NDAA 목록)](https://hanwhavisionamerica.com/product/qnp-6250r/)
13. [SIYI A8 mini $253 (insightfpv)](https://insightfpv.com/products/siyi-a8-mini-gimbal-camera)

11번은 한화 공식 데이터시트이지만 영국 리셀러 서버에 호스팅된 사본입니다. 한화비전 공식 사이트의 다운로드 페이지에서 원본을 받아 최종 대조하시길 권합니다. A8 mini의 크기와 무게(2번), Tail Air 스펙(5번)도 제조사 공식 페이지가 아니니 같은 방식으로 확인이 필요합니다.
