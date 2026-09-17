AMR 실제 구성(PC ― EDS-2010-ML ― EDS-205, 12V DC SMPS)을 반영해 배선 항목을 추가했습니다. 대괄호 숫자가 출처 번호입니다.

**전체 비교표**

| 항목 | SIYI A8 mini | PIXEL TS-HD10S | AXIS M5075 |
|---|---|---|---|
| **해상도** | 4K 녹화 / 1080p 스트림 [1] | 미확인 | 1920×1080, 최대 50/60fps [3] |
| **Zoom (광학)** | 없음 [2] | **10배** [4] | 5배 (2.2~11.0mm, F1.4~2.5) [3] |
| **Zoom (디지털)** | 6배 [2] | 미확인 | 12배 [3] |
| **Sensor** | 1/1.7" Sony 8MP [2] | 미확인 | 1/2.8" progressive RGB CMOS [3] |
| **Pan** | 미확인 | 미확인 | ±179°, 100°/s [3] |
| **Tilt** | -90°~+25° [1] ※웹 -135°~+45° | 미확인 | 90°, 100°/s (기준선 미표기) [3] |
| **화각(수평)** | 81° 고정 [2] | 미확인 | 14°~71° [3] |
| **전원 (소비전력)** | DC 11~25.2V, 평균 5W / 피크 12W [1][2] | 미확인 (12V 2A 어댑터 추정) | **PoE Class3, 일반 4.2W / 최대 9.3W** [3] |
| **12V 기준 전류** | 평균 0.42A / 피크 1.0A (계산) | 미확인 | **약 0.41A / 최대 0.91A** (미드스팬 손실 포함, 계산) |
| **사이즈 (mm)** | 55×55×70 [6] | 미확인 | φ130 × 63 [3] |
| **무게 (kg)** | **0.095** [2] | 미확인 | 0.380 [3] |
| **흔들림 보정** | **3축 기계식 짐벌** [2] | 미확인 | **없음** (EIS 미지원) [3] |
| **SW (기본 GUI)** | UniGCS, SIYI QGC (Win/Android) [1] | 미확인 | **내장 웹뷰어** (브라우저) [3] |
| **SW (SDK/API)** | SIYI Gimbal SDK (UDP/TCP 37260) [1] | **ONVIF / VISCA / Pelco-D,P / UVC** (공개 표준) [5] | **VAPIX + ACAP**, ONVIF G/M/S/T [3] |
| **가격 (만원)** | 36~42 ($253~299) [6][7] | **79** [4] | 미확인 (M5075-G 기준 89~101) [8] |
| **추가 구매품** | 없음 | 미확인 | **미드스팬 02208-001** [9][10] |
| **연속 운용** | **8시간 제한** (30분 냉각) [1] | 미확인 | 24시간 (0~45°C) [3] |
| **국내 정식 판매** | 미확인 (직구/유통) | **국내 업체** [4] | **KR 품번 02346-001** [11] |

**배선 비교**

| 항목 | SIYI A8 mini | PIXEL TS-HD10S | AXIS M5075 |
|---|---|---|---|
| **전원 연결** | SMPS → 카메라 직결 [1] | 미확인 (DC 어댑터 추정) | SMPS → 미드스팬 → PoE [9] |
| **데이터 연결** | 카메라 → **RJ45 변환 케이블** → EDS-205 [1] | ONVIF면 RJ45 직결 (확인 필요) [5] | 카메라 → 미드스팬 → EDS-205 [9] |
| **케이블 수** | 2가닥 (전원 1, 랜 1) | 2가닥 추정 | **3가닥** (전원 1, 랜 2) |
| **중계 장비** | 없음 | 없음 (추정) | **미드스팬 1대** (33×53×140mm급) |
| **커넥터** | 전용 소형 Ethernet → RJ45 변환 [1] | 미확인 | **Shielded RJ45 필수**, Cat5 이상 STP [9] |
| **EDS-205 호환성** | **링크 불안 우려** (10M 고정 이슈, 비공식) [12] | 표준 IP 카메라면 문제없음 | 표준 IP 카메라, 문제없음 |
| **IP 설정** | 고정 192.168.144.25 (변경 가능) [1] | 미확인 | DHCP 기본, AXIS IP Utility로 설정 |
| **비고** | 배선 최단순. 입력 하한 11V로 12V 여유 1V뿐 | ONVIF/RJ45 여부가 관건 | 미드스팬 설치 공간·고정(DIN 클립 T91A03) 필요 |

**AMR 구성별 배선도**

```
[SIYI A8 mini]
12V SMPS ──→ A8 mini ──(RJ45 변환)──→ EDS-205 ──→ EDS-2010-ML ──→ PC

[AXIS M5075]
12V SMPS ──→ Midspan(02208-001) ──PoE──→ M5075
                    ↑
              EDS-205 ──(데이터)──┘ ──→ EDS-2010-ML ──→ PC
```

**배선 관점 요약**

**A8 mini**가 가장 단순합니다. 미드스팬 없이 전원과 랜 2가닥이면 끝입니다. 다만 두 가지가 걸립니다. 입력 하한이 11V라 12V에서 여유가 1V뿐이고, 비관리형 스위치인 EDS-205에서 링크가 안 잡힐 가능성(비공식 보고)이 있습니다.

**M5075**는 미드스팬이 추가되어 배선과 부품이 늘어나지만, 표준 IP 카메라라 스위치 호환성 문제가 없고 전력 여유도 가장 큽니다. 미드스팬 입력이 10~28V DC라 12V와 24V 모두 대응됩니다.

**TS-HD10S**는 ONVIF 지원이 확인되면 A8 mini와 M5075의 중간 수준이 될 것으로 보이나, RJ45 포트 유무와 전원 규격이 확인되지 않아 배선 판단이 불가능합니다.

**출처**

1. [SIYI A8 mini 매뉴얼 v1.10 (제조사 공식)](https://res.siyi.biz/oss/other/2026/06/15/A8_mini_User_Manual_v1_10_563cde30.pdf)
2. [SIYI A8 mini 사양 (ARRIS Hobby, 판매처)](https://www.arrishobby.com/products/siyi-a8-mini-4k-8mp-ultra-hd-6x-digital-zoom-camera-with-3-axis-gimbal)
3. [AXIS M5075 데이터시트 T10175558/EN/M17.2/202511 (제조사 공식)](https://www.axis.com/dam/public/d1/6d/a7/datasheet-axis-m5075-ptz-camera-en-US-506699.pdf)
4. [PIXEL TS-HD10S 790,000원 (다나와, 판매처 티에스아이시스템)](https://search.danawa.com/dsearch.php?query=%ED%92%80HD+%EC%BA%A0%EC%BD%94%EB%8D%94)
5. Tony 님이 제품 자료에서 확인해 주신 프로토콜 정보 (UVC, Pelco-D/P, VISCA, ONVIF) — 원문 미확인
6. [A8 mini 크기·무게 (MotioNew, 판매처)](https://www.motionew.com/shop/gimbal-and-payload/siyi-a8-mini/)
7. [A8 mini $253 (insightfpv, 판매처)](https://insightfpv.com/products/siyi-a8-mini-gimbal-camera)
8. [AXIS M5075-G $694.99 (IP Phone Warehouse, 판매처)](https://www.ipphone-warehouse.com/axis-m5075-g-1080p-ptz-z-wave-indoor-mini-dome-ip-camera-02347-004/)
9. [AXIS 30W Midspan AC/DC 데이터시트 (제조사 공식)](https://www.axis.com/dam/public/b6/ae/6b/datasheet-axis-30-w-midspan-acdc-en-US-321538.pdf)
10. [AXIS 30W Midspan AC/DC 제품 페이지 (제조사 공식)](https://www.axis.com/products/axis-30-w-midspan-acdc)
11. [AXIS M5075 제품 페이지 — KR 품번 02346-001 (제조사 공식)](https://www.axis.com/products/axis-m5075)
12. [NVIDIA 개발자 포럼 — A8 mini 10Mbps 고정 이슈 (커뮤니티, 비공식)](https://forums.developer.nvidia.com/t/solved-siyi-a8-mini-connection-issues-on-jetson-rpi-pc/372611)

**미확인 항목 정리**

| 대상 | 확인할 것 |
|---|---|
| SIYI | 틸트 값 불일치, Pan 범위, 8시간 제한 |
| PIXEL | **RJ45/RTSP 지원 여부**, 틸트 범위, 전원·크기·무게, 연속 운용 |
| AXIS | **틸트 기준선(역방향 설치)**, 국내 공급가 |

12V 기준 전류는 데이터시트 소비전력에서 제가 계산한 값이고(미드스팬 효율 85% 가정), 제조사가 명시한 수치가 아닙니다. 실측으로 확인하시는 게 정확합니다.
