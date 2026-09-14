전체를 재정리했습니다. 각 값 뒤 대괄호 숫자가 출처 번호입니다.

**핵심 비교표**

| 항목 | SIYI A8 mini | Topotek DIY10S90 | OBSBOT Tail Air | 한화 QNP-6250R/6320R |
|---|---|---|---|---|
| 가격 | $253~299 [1][2] | $582 [3] | $499 [4] | 미확인 |
| 광학줌 | 없음 (6배 디지털) [1] | **10배** [3] | 없음 (4배 디지털) [5] | **25배 / 32배** [6][7] |
| **상향 틸트** | +25° (매뉴얼) [8] | **+120°** [3] | +32° (제어) [5] | **+90°** [6] |
| 전체 틸트 | -90°~+25° [8] | -45°~+120° [3] | -65°~+32° [5] | -20°~+90° [6] |
| 기본 GUI | UniGCS, SIYI QGC [8] | PC network GCS [9] | OBSBOT Start / WebCam [4] | **내장 웹뷰어** [7] |
| SDK/API | SIYI Gimbal SDK [8] | UDP&UART + **Python 샘플** [9] | **Win/Linux/macOS SDK** [10] | **SUNAPI + ONVIF** [7] |
| 연속 운용 | **8시간 제한** [8] | 미확인 | 미확인 | 24시간 설계 |
| 무게 | 95g [1] | 425g [3] | 344.5g [5] | 수 kg 급 |

**상세 사양**

**SIYI A8 mini**
1/1.7인치 Sony 8MP 센서, 4K 녹화·1080p 스트림 [1]. 전원 11~25.2V, 평균 5W·피크 12W [1][8]. 영상 출력은 Ethernet/HDMI/CVBS, 제어는 S.Bus/UART/UDP [1]. RTSP는 `rtsp://192.168.144.25:8554/main.264`, 제어 포트 37260 [8]. 크기 55×55×70mm [2]. microSD는 녹화와 펌웨어 업데이트에 필요 (Class10, 최대 256GB) [8]. **웹사이트는 틸트를 -135°~+45°로 표기해 매뉴얼과 불일치 — SIYI 확인 필요** [8].

**Topotek DIY10S90**
1/2.8인치 2MP WDR CMOS, 10배 광학줌 f=5.15~47.38mm [3]. RTSP 1080p 30fps + TF카드 저장, AF 1초 미만 [3]. Yaw -285°~+285°, Roll -45°~+45°, 지터 Pitch/Roll ±0.02°·수평 ±0.03° [3]. 전원 DC 12~26.2V, 5W [3]. 크기 φ150×H140mm, 동작 온도 -10~+50°C [3]. AI 추적 내장 (사람·차량, 최대 100개 동시 검출, 최소 32×32픽셀) [3]. SDK는 UDP&UART 프로토콜 PDF, Python 샘플 코드, CommandTool을 로그인 없이 제공 [9]. **단, 프로토콜 문서의 적용 모델 목록에 DIY10S90이 명시되어 있지 않음 — 확인 필요** [9].

**OBSBOT Tail Air**
1/1.8인치 CMOS 8.4MP(3856×2176), 4K 30fps·1080p 60fps, 2μm 픽셀, f/1.8 [5]. **2축 짐벌 + 6축 자이로** (3축 기계식 짐벌 아님) [5]. 기계적 범위는 Pan ±160°·Tilt ±90°지만 제어 가능 범위는 Pan ±150°·Tilt -65°~+32° [5]. 최대 제어 속도 120°/s, 각도 지터 ±0.02° [5]. USB-C to Ethernet 어댑터로 유선 연결, **PoE 지원** [4]. 크기 69.65×73.25×132.5mm [5]. ISO 100~6400으로 저조도는 약한 편 [5]. 펌웨어 업데이트에 U3 microSD 필요.

**한화 QNP-6250R / 6320R**
2MP 1080p 60fps, H.265/H.264/MJPEG, WiseStreamII [7]. QNP-6250은 25배 광학(4.44~111mm) + 32배 디지털 [6]. **110도 틸트로 카메라보다 위쪽 대상과 경사 환경 감시가 가능하다고 공식 명시** [6]. Pan 360도 무한 회전, 프리셋 속도 700°/s, 프리셋 정확도 ±0.2° [6][7]. WDR 120dB, 자이로 내장 DIS, Day&Night(ICR) [7]. 전원은 PoE+ 최대 25.5W [6], QNP-6250은 최대 14.5W·일반 12W [11]. microSD 최대 256GB [6]. **Q 시리즈는 기존 PTZ 대비 무게를 최대 55% 줄이고 크기도 축소** [6]. 보안 기능으로 802.1X, HTTPS/SRTP, IP 필터링 [7], NDAA 호환 제품 목록 등재 [12].

**미확인 항목 (문의 필요)**

| 대상 | 확인할 것 |
|---|---|
| SIYI | 틸트 값 불일치, upside down 모드 실제 상향각, 8시간 제한 |
| Topotek | DIY10S90 적용 프로토콜 문서, 전용 GUI, 연속 운용, 기본 IP/RTSP |
| 한화 | **국내 견적**, PoE 급전 규격, AMR 탑재 시 장착 방식 |

**출처**

1. [SIYI A8 mini 사양 (ARRIS Hobby)](https://www.arrishobby.com/products/siyi-a8-mini-4k-8mp-ultra-hd-6x-digital-zoom-camera-with-3-axis-gimbal) · 2. [insightfpv A8 mini $253](https://insightfpv.com/products/siyi-a8-mini-gimbal-camera), [MotioNew 크기 정보](https://www.motionew.com/shop/gimbal-and-payload/siyi-a8-mini/) · 3. [Topotek DIY10S90 공식 제품 페이지](https://topotek.com/DIY10S90-10x-Optical-zoom-IRCUT-3-Axis-Gimbal-IP-output-p5832404.html) · 4. [OBSBOT Tail Air 공식](https://www.obsbot.com/obsbot-tail-air-streaming-camera), [B&H (PoE·이더넷 어댑터)](https://www.bhphotovideo.com/c/product/1793976-REG/obsbot_owb_taaiptz_tail_air_ai_powered_ptz.html) · 5. [Tail Air 상세 스펙 (The Gadgeteer)](https://the-gadgeteer.com/2025/02/02/obsbot-tail-air-review-a-ptz-camera-with-no-strings-cables-attached/) · 6. [한화비전 QNP-6250R 공식 (틸트 110°)](https://www.hanwhavision.com/en/products/camera/network/ptz/qnp-6250r/) · 7. [QNP-6250 사양 (IP Security Depot)](https://www.ipsecuritydepot.com/hanwha-qnp-6250-2mp-high-speed-ptz-ir-wdr-outdoor-dome-ip-camera/qnp-6250/) · 8. [SIYI A8 mini 매뉴얼 v1.10 (공식)](https://res.siyi.biz/oss/other/2026/06/15/A8_mini_User_Manual_v1_10_563cde30.pdf) · 9. [Topotek 다운로드 리소스 (공식)](https://topotek.com/pages/TOPOTEK-Download-Resources-11763444.html) · 10. [OBSBOT SDK 신청 페이지](https://www.obsbot.com/sdk) · 11. [QNP-6250 데이터시트 (한화비전 EU)](https://hanwhavision.eu/wp-content/uploads/2022/12/DataSheet_QNP-6250_221201_EN.pdf) · 12. [한화비전 미주 QNP-6250 제품 페이지](https://hanwhavisionamerica.com/product/qnp-6250/)

7번과 5번은 제조사 공식이 아닌 판매처·리뷰 자료라, 최종 결정 전에 한화 공식 데이터시트와 OBSBOT 공식 스펙 페이지로 한 번 더 대조하시길 권합니다.
