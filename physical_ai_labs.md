([Past chat][1])([Past chat][1])([Past chat][1])([Past chat][1])([Past chat][2])

좋아. 네가 고른 4곳은 **“VLA/IL + 실제 로봇(조작) + end-to-end(혹은 계층형) 제어”** 축에서 서로 역할이 겹치지 않으면서도 **산학 PoC를 만들기 좋은 조합**이야. 아래는 **각 랩이 강하게 가져갈 수 있는 포지션**과 **산학 제안서(1~2페이지)로 바로 연결되는 협력 시나리오**까지 정리한 거야.

---

## 1) KAIST CLVR (Joseph J. Lim)

**무엇에 강한가**

* 랩 소개 자체가 “**perception–action–reasoning으로 sequential decision**”을 목표로 하고, 토픽이 **RL/robot learning + world model/representation learning + visual perception + symbolic manipulation**으로 명시돼 있어. ([CLVR][3])
  **너의 과제에 어떻게 붙나**
* 듀얼암/휴머노이드 조작을 “그냥 정책학습”이 아니라
  **(1) world model/representation → (2) long-horizon decision → (3) 조작 스킬**로 설계하고 싶을 때 최적.
* 특히 **장기 시퀀스(조립/삽입/다단계 협동)** 같이 “계획 + 실행”이 모두 필요한 과제에서 강점이 나기 쉬움.

**산학 제안 포맷 추천**

* “언어 지시 → 서브태스크 분해(상징/계획) → 스킬 정책(IL/RL)”의 **2계층**을 기본 골격으로 제안.

---

## 2) KAIST RIRO (Daehyung Park, School of Computing)

**무엇에 강한가**

* 연구 토픽 페이지에 **휴머노이드 whole-body manipulation**을 정면으로 내걸고(예: Unitree G1), ([rirolab.github.io][4])
* 모집/연구 토픽에 **Robot foundation model / VLA**, **Humanoid navigation + whole-body manipulation**, **LLM/VLM guided task planning**, **IRL/ICL**, **deformable manipulation**까지 “산학에서 바로 과제화 가능한 주제들”이 구체적으로 적혀 있어. ([rirolab.github.io][5])

**너의 과제에 어떻게 붙나**

* “**휴머노이드/모바일 매니퓰레이션**에서 VLA를 실제로 굴리는” 방향(현장 task, 안전/실행까지 포함)으로 PoC가 잘 나옴.
* 듀얼암도 가능하지만, **휴머노이드/전신 제어 + 언어기반 task 실행**을 목표로 잡으면 RIRO의 강점이 더 직접적으로 드러남.

**산학 제안 포맷 추천**

* 8~12주 PoC로는 “**언어 지시 기반 Handover / Bimanual pick-and-place / simple assembly**” 같은 **성공률로 측정되는 데모**가 적합.

---

## 3) Yonsei RL & Robot Learning Lab (Youngwoon Lee)

**무엇에 강한가**

* 랩 소개에서 **reinforcement learning + imitation learning + generative modeling + robotics**를 명시하고, ([RLLAB][6])
* 교수 연구 목표에서 “**humanoid robots 같은 complex robotic systems**”를 직접 언급해. ([RLLAB][7])

**너의 과제에 어떻게 붙나**

* 듀얼암/휴머노이드 조작을 **데이터 효율**(적은 데모로 성능)과 **일반화**(새 물체/새 배치) 중심으로 끌고 가고 싶을 때 강함.
* 특히 최근 트렌드인 **IL + generative policy(예: diffusion 계열)** 같이 end-to-end 정책 쪽으로 산학을 설계하기 좋음(랩이 그 축을 “명시”함). ([RLLAB][6])

**산학 제안 포맷 추천**

* “데모 N개로 시작 → 성능 목표(성공률/시간) → 데이터 추가 시 개선곡선”을 **정량 프레임**으로 제시하면 설득력이 커짐.

---

## 4) SNU Biointelligence Lab (Byoung-Tak Zhang, CSE)

**무엇에 강한가**

* 랩이 “**vision, language, action을 통합하는 모델**”을 개발한다고 **정면으로 명시**하고 있어, VLA 스토리라인이 매우 깔끔해. ([SNU Biointelligence Lab][8])
* 로봇 조작 관련 연구(예: **language-guided robotic manipulation의 visual grounding** 등)도 연구 리스트에서 확인돼. ([SNU Biointelligence Lab][9])

**너의 과제에 어떻게 붙나**

* “로봇 하드웨어 자체(휴머노이드/듀얼암) end-to-end 제어”보다는,
  **VLA의 핵심(언어-시각 grounding, 멀티모달 추론, action representation)**을 **조작 문제로 묶어서** 산학을 만들 때 강점이 큼.
* 즉, 산업 과제를 **‘언어로 task 스펙을 주고, 로봇이 알아서 수행하게 만드는’** VLA 형태로 설계하면 핏이 가장 좋음.

---

# 4개 랩에 공통으로 “먹히는” 산학 PoC 주제 3개

아래는 **휴머노이드/듀얼암 + VLA/IL end-to-end**로 묶었을 때, 4개 랩 모두가 관심 가질 확률이 높은 형태야.

1. **Language-instructed bimanual manipulation**

* 예: “왼손은 고정, 오른손은 삽입”, “두 손으로 정렬해서 끼워 넣기”
* 측정: 성공률, cycle time, 실패 recovery 횟수

2. **Data-efficient imitation learning for dual-arm assembly**

* 데모 20/50/100개 스케일에서 성능 곡선 제시
* sim→real 또는 domain randomization 포함 가능

3. **Long-horizon task with skill composition**

* 언어 지시 → subtask decomposition → skill library(IL policy) 실행
* 실제 현장에서는 이게 “끝까지 수행”이 핵심 KPI가 됨

---

# 컨택 전에 준비하면 “답장 확률”이 확 뛰는 6줄 요약(템플릿)

* (1) **로봇 형태**: 휴머노이드 vs 듀얼암(고정 베이스/모바일)
* (2) **작업 1개**: 예) bimanual handover / 삽입 / 조립 / 케이블
* (3) **센서/제약**: RGB-D/FT/tactile 여부, 안전거리/속도/cycle time
* (4) **데이터 계획**: teleop 가능? 데모 몇 개로 시작?
* (5) **성공지표**: 성공률/시간/일반화(OOD)/복구율 중 상위 2개
* (6) **기간/리소스**: PoC 8~12주 vs 6~12개월 + 제공 가능 장비/현장

---

원하면, 위 4개 랩 각각에 맞춰서 **“같은 과제라도 포지셔닝을 다르게”** 잡아주는 1페이지 제안서(랩별 버전 4개)로 만들어줄게.

바로 진행하려면 딱 3가지만 알려줘:

1. **휴머노이드 vs 듀얼암** 중 우선순위
2. 목표 task 1개(예: handover/조립/삽입/케이블)
3. 데모 데이터 수집 방식(teleop 가능 여부)

[1]: https://chatgpt.com/c/69513408-74c8-8323-90ff-8600eea60f25 "최신 AI 로보틱스 교육과정"
[2]: https://chatgpt.com/c/6798991b-1c00-8006-bce6-faec8c29182e "Publications and PhD Applications"
[3]: https://www.clvrai.com/?utm_source=chatgpt.com "CLVR Lab"
[4]: https://rirolab.github.io/research/topics?utm_source=chatgpt.com "Research Areas"
[5]: https://rirolab.github.io/position/graduates/?utm_source=chatgpt.com "Graduate Research Assistant Positions"
[6]: https://youngwoon.github.io/?utm_source=chatgpt.com "RLLAB @ Yonsei"
[7]: https://youngwoon.github.io/youngwoon?utm_source=chatgpt.com "Youngwoon Lee"
[8]: https://bi.snu.ac.kr/?utm_source=chatgpt.com "SNU Biointelligence Lab"
[9]: https://bi.snu.ac.kr/research/?search=%22%EB%B0%95%EC%98%88%EC%86%94%22+%22Yesol+Park%22+%22yspark%22+&utm_source=chatgpt.com "Research"

############################  
https://www.clvrai.com/

https://rirolab.kaist.ac.kr/

https://youngwoon.github.io/

https://bi.snu.ac.kr/

#####################################33  ###############################3  
##########################################################################  
좋아. 부서원들과 공유하기 좋게 **4개 랩을 한 표로** 정리했어(필요한 정보는 각 랩 공식 페이지 기준).

| 학교    | 랩 / PI                                                          | 소속(학과/스쿨)                                                     | 핵심 키워드(공식 소개 기준)                                                                                                                                                | 휴머노이드/듀얼암·E2E 적합도                                                                                                             | 산학 PoC 아이디어(예시)                                                                                                                                       |
| ----- | --------------------------------------------------------------- | ------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------- |
| KAIST | **CLVR Lab** (Joseph J. Lim, 임재환)                               | KAIST (교수 페이지에 **Kim Jaechul School of AI**로 명시) ([CLVR][1])  | **Reinforcement Learning, Robot Learning, World Model/Representation Learning, Visual Perception, Symbolic Manipulation** ([CLVR][2])                           | **중~상**: “Perception–Action–Reasoning로 sequential decision” 중심이라 **장기/복합 조작(계획+실행)** 설계에 강점 ([CLVR][2])                       | (1) **Long-horizon 조립/삽입**: 고수준 계획+저수준 스킬 학습 (2) **스킬 조합형 조작**: subtask decomposition + skill policy(IL/RL)                                           |
| KAIST | **RIRO Lab** (Daehyung Park, 박대형)                               | **School of Computing** (GSAI 겸임 표기) ([rirolab.github.io][3]) | **Mobile manipulation**, semantic learning & reasoning, **IRL**, task-and-motion planning, **HRI**, **natural language understanding** ([rirolab.github.io][3]) | **상**: “실로봇이 사람과 협업” + **언어/계획/학습/실행**을 한 축으로 명시. 듀얼암·모바일 매니퓰레이션 산학 PoC에 매우 적합 ([rirolab.github.io][3])                       | (1) **Language-conditioned mobile manipulation** (2) **Handover/협동 조작**(dual-arm/협동로봇) (3) **VR/Teleop 데모 수집→IL**(변형물/조작 포함) ([rirolab.github.io][4]) |
| 연세대   | **Yonsei RL & Robot Learning Lab (RLLAB)** (Youngwoon Lee, 이영운) | 연세대 **AI/첨단컴퓨팅** 연구실 목록에 공식 등재 ([연세대학교 첨단컴퓨팅학부][5])           | **Reinforcement Learning, Imitation Learning, Generative Modeling, Robotics** ([youngwoon.github.io][6])                                                        | **상**: 로봇 실세계 task + IL/생성모델 축이 뚜렷하고, 뉴스/활동에서 **whole-body / bimanual** 관련 워크숍·벤치마크 언급이 있어 방향성이 강함 ([youngwoon.github.io][6]) | (1) **데이터 효율 듀얼암 조작(조립/정렬/삽입)**: 데모 20/50/100개 스케일 성능곡선 (2) **Generative policy 기반 E2E**(시각→행동)                                                       |
| 서울대   | **SNU Biointelligence Lab** (Byoung-Tak Zhang, 장병탁)             | SNU **컴퓨터공학부(CSE)** 소속으로 명시 ([bi.snu.ac.kr][7])               | **Embodied AI**, **Multimodal AI(vision-language-action 통합)**, sensorimotor learning, grounded reasoning, lifelong adaptation ([bi.snu.ac.kr][7])               | **중~상**: “VLA 통합 + Embodied AI”를 공식 미션으로 명시. 로봇 **E2E 정책**도 가능하지만, 강점은 **VLA/멀티모달 추론·표현**을 로봇 행동으로 연결하는 쪽 ([bi.snu.ac.kr][7]) | (1) **VLA 기반 task specification**(언어 지시→행동) (2) **멀티모달(비전+언어+행동) 데이터/모델** 공동개발 (3) 현장 시나리오에서 **grounded reasoning + 실행 안정화**                          |

원하면, 이 표를 더 “보고서/메일용”으로 쓰기 좋게 **열을 추가**해서(예: *연락 포인트/협력 형태(위탁·공동·과제)/빠른 PoC 기간/필요 장비*) 깔끔하게 확장해줄게.

[1]: https://clvrai.com/web_lim/?utm_source=chatgpt.com "Joseph Lim's Home Page - CLVR Lab"
[2]: https://www.clvrai.com/ "CLVR"
[3]: https://rirolab.github.io/ "Robust Intelligence and Robotics Laboratory"
[4]: https://rirolab.github.io/publication/domestic/?utm_source=chatgpt.com "Publications"
[5]: https://cais.yonsei.ac.kr/csai_en/researchAll.do?utm_source=chatgpt.com "Research Labs | All"
[6]: https://youngwoon.github.io/ "RLLAB @ Yonsei"
[7]: https://bi.snu.ac.kr/ "SNU Biointelligence Lab"

######################################3  
################################################  
##################################################  
좋아—주신 4개 링크의 publication/project/topic 페이지를 전부 훑어서 “듀얼암/휴머노이드 + end-to-end/VLA/IL” 관련 근거를 뽑고, ‘로봇 바퀴 교체’ 태스크에 맞춰 표(강점/적합도/PoC 아이디어/리스크)까지 업데이트하겠어.


좋아. 아래는 **각 링크(논문/토픽/프로젝트 페이지)를 직접 읽고**, Seri가 하려는 **“듀얼암/휴머노이드가 다른 로봇의 바퀴를 교체하는 작업”**에 맞춰 **표를 업데이트**한 버전이야. (바퀴 교체는 보통 *접촉이 많은 조립/분해 + 공구 사용(볼트) + 장기 시퀀스 + 정밀 정렬/토크 관리*가 핵심이야.)

---

## 4개 랩 비교 표 (Wheel Replacement Task 관점 업데이트)

| 랩(학교)                                           | 링크에서 확인되는 대표 연구축 (근거)                                                                                                                                                                                                                                                                                          | 듀얼암/휴머노이드·E2E/VLA/IL 관점 “바퀴 교체” 적합 포인트                                                                                                                                                                                                              | 산학 PoC를 이렇게 잡으면 “딱” 맞음                                                                                                                                                                                                 | 리스크/전제                                                                                                         |
| ----------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------- |
| **CLVR Lab (KAIST)**                            | **장기·복합 조작 벤치마크/환경**(FurnitureBench, IKEA Furniture Assembly), **대규모 로봇 조작 데이터/파운데이션**(DROID, Open X-Embodiment/RT-X), **데이터 수집/학습 효율화**(PATO, EXTRACT), **언어 기반 학습 신호/리워드**(ReWiND, SPRINT) ([CLVR][1])                                                                                                       | 바퀴 교체는 “**장기 시퀀스 조작(분해→교체→조립)**”이라 FurnitureBench/IKEA류와 결이 같고 ([CLVR][1]), 데이터가 부족한 공정에 대해 **Teleop 데이터 수집(PATO)** + **오프라인 데이터에서 스킬 추출(EXTRACT)** 같은 접근이 매우 잘 맞음 ([CLVR][1]). 또한 **언어로 태스크/보상 신호를 설계**하는 라인(ReWiND/SPRINT)도 공정 변경에 강함 ([CLVR][1]) | **(추천 PoC)** 1) 바퀴 교체를 6~8개 서브태스크로 쪼갠 뒤(볼트 풀기/휠 분리/정렬/체결 등) 2) PATO식으로 **정책 보조 Teleop**로 데모 수집 ([CLVR][1]) 3) EXTRACT로 **재사용 가능한 스킬 라이브러리**화 ([CLVR][1]) 4) 공정/모델 변경 시 ReWiND/SPRINT로 **언어 기반 신호로 빠르게 적응** ([CLVR][1]) | 듀얼암/휴머노이드 “플랫폼 자체”에 대한 명시는 이 페이지에서 직접적이지 않음(대신 데이터/학습/조작 벤치마크가 강점). 실제 바퀴 교체는 **힘/토크·안전·툴링(렌치) 하드웨어 통합** 이슈가 큼 |
| **RIRO Lab (KAIST)**                            | **휴머노이드 whole-body manipulation**을 전면에 두고, **IL/RL 기반 policy learning + LLM/VLA 기반 고수준 계획**을 핵심 방향으로 명시 ([rirolab.kaist.ac.kr][2]) (Unitree G1 등 플랫폼 언급 포함) ([rirolab.kaist.ac.kr][2])                                                                                                                         | 바퀴 교체는 “**휴머노이드가 공구를 들고 체결/분해**”까지 가야 해서, RIRO가 적어둔 연구 방향(whole-body manipulation + IL/RL + VLA/LLM plan)이 과제 정의와 거의 1:1로 매칭됨 ([rirolab.kaist.ac.kr][2]). 특히 “복잡·실세계 태스크 일반화”를 목표로 하고 산학 협력 평가를 언급함 ([rirolab.kaist.ac.kr][2])                      | **(추천 PoC)** 1) LLM/VLA로 “작업 계획/체크리스트/안전 규칙”을 생성하고 ([rirolab.kaist.ac.kr][2]) 2) 저수준은 IL/RL/디퓨전 정책 등으로 “접촉 구간(볼트 풀기/조립 정렬)”을 학습 ([rirolab.kaist.ac.kr][2]) 3) 실패 시 recovery(재정렬/재그립)까지 포함한 **계층형 정책 데모**               | 휴머노이드 whole-body는 실세계에서 **안전/하드웨어 안정성/데이터 수집 비용**이 커서 PoC 범위를 잘 쪼개야 함(예: 처음엔 “듀얼암 매니퓰레이터+고정 베이스”로 시작 후 확장)     |
| **Yonsei RL & Robot Learning Lab (RLLAB, 연세대)** | 랩 소개에서 **RL/IL/Generative modeling의 로봇 적용**을 명시 ([youngwoon.github.io][3]). 또한 최신 연구로 **VLA 기반 bimanual 조작(TwinVLA)**을 다룸(“bimanual tasks”, “compose pretrained single-arm VLA into coordinated bimanual VLA”) ([arXiv][4]). 뉴스에 **HumanoidBench/DROID** 등 휴머노이드/조작 벤치·데이터 관여도 언급 ([youngwoon.github.io][3])   | 바퀴 교체는 사실상 “**bimanual coordination + (가능하면) VLA로 지시/상태를 함께 쓰는 E2E**”가 강력한 후보인데, TwinVLA는 **듀얼암 데이터를 대량으로 추가하지 않고도**(공개 단일팔 데이터 활용) bimanual을 구성하는 아이디어라, “처음부터 듀얼암 데모를 왕창 모으기 어려운 산업 PoC”에 특히 유리한 방향성임 ([arXiv][4])                                | **(추천 PoC)** 1) 단일팔 데이터/정책으로 “볼트 접근·공구 그립·회전” 같은 primitive를 먼저 안정화 2) TwinVLA류의 **모듈 합성**으로 듀얼암 협응(한 팔 고정/다른 팔 회전 등) ([arXiv][4]) 3) 이후 최소량의 듀얼암 데모로 “정렬/끼움” 구간만 보강                                                    | 실제 바퀴 교체는 공구/토크가 중요해서 **비전-only E2E만으로는 한계**가 있을 수 있음 → 촉각/힘센서/토크 제한 같은 관측·제약을 어떻게 넣을지가 핵심                     |
| **SNU Biointelligence Lab (서울대)**               | 프로젝트 페이지에서 “**perception–cognition–action 통합**”을 명시 ([bi.snu.ac.kr][5]). **Embodied AI(로봇 파운데이션 모델)** 항목에 *Visual grounding & manipulation*, *Arm robot 조작*, *자연어 기반 로봇 평생학습*, *ROS 기반 실세계 로봇 제어* 등을 적시 ([bi.snu.ac.kr][5]). 또한 **Alchemist: humanoid robotics capabilities** 프로젝트를 별도로 명시 ([bi.snu.ac.kr][5]) | 바퀴 교체는 “행동만 잘하는 것”뿐 아니라 **상황 이해/추론(무슨 휠 모델? 어떤 순서? 예외처리?)**이 중요해서, SNU쪽은 VLA/멀티모달 추론 + 실행을 “시스템으로” 묶는 협력에 강점이 있음(특히 자연어 기반 평생학습/grounding 방향) ([bi.snu.ac.kr][5])                                                                                   | **(추천 PoC)** 1) “바퀴 교체 SOP(표준 작업절차)”를 자연어/멀티모달로 모델링 2) 로봇이 작업 중 관측 불확실성을 감지하고(볼트 안보임/툴 미스그립 등) **재관측/재시도 전략**까지 포함한 **인지-행동 통합 파이프라인** 구축 ([bi.snu.ac.kr][5])                                                          | 페이지 기준으로는 “특정 듀얼암 조작 논문”을 나열하기보다 **프로젝트/시스템 축**이 강하게 보임 → PoC를 “추론+정책” 통합 데모로 잡는 게 설득력 큼                       |

---

## 바퀴 교체 태스크를 “연구 협력 과제”로 잘 보이게 만드는 5가지 핵심 요구사항

각 랩에 공통으로 제안할 때, 아래 5개를 명확히 써주면 **VLA/IL/E2E 연구로 자연스럽게 포지셔닝**돼.

1. **서브태스크 분해**: 접근 → 볼트 탐지/정렬 → 공구 그립 → 풀기 → 휠 분리 → 신규 휠 정렬/삽입 → 체결 → 검증
2. **접촉-rich 구간** 정의: “정렬/삽입/체결”은 force/torque/constraint가 핵심
3. **데이터 전략**: Teleop(최소 데모) + 오프라인 데이터/스킬 재사용 + 실패 데이터까지 활용
4. **계층형 정책**: (LLM/VLA planner) + (low-level policy: diffusion/BC/RL)
5. **평가 지표**: 성공률, 평균 시간, 재시도 횟수, 토크 오버/언더, 부품 손상률(안전)

---

원하면, 위 표를 **부서 공유용**으로 더 실무적으로 바꿔서

* “우리가 제공해야 할 것(하드웨어/데이터/현장 접근/안전 규정)”
* “랩이 제공할 것(모델/학습/벤치마크/데이터 파이프라인)”
* “3개월 PoC 범위(최소기능) / 6~12개월 확장 로드맵”
  열을 추가한 버전으로도 정리해줄게.

[1]: https://www.clvrai.com/publications/ "Publications | CLVR"
[2]: https://rirolab.kaist.ac.kr/research/topics "Research Areas"
[3]: https://youngwoon.github.io/ "RLLAB @ Yonsei"
[4]: https://arxiv.org/abs/2511.05275 "[2511.05275] TwinVLA: Data-Efficient Bimanual Manipulation with Twin Single-Arm Vision-Language-Action Models"
[5]: https://bi.snu.ac.kr/projects/ "Projects | SNU Biointelligence Lab"

##################################  
######################################3  
####################################  
아래 표는 **4개 랩을 같은 기준으로 비교**해서, “**휴머노이드/듀얼암이 다른 로봇의 바퀴를 교체**(분해·정렬·체결·접촉-rich 장기 시퀀스)” 태스크 관점에서 **1개만 뽑아야 할 때** 의사결정에 바로 쓰도록 정리한 거야. (키워드/연구축은 **각 랩이 제공한 Topics/Publications/Projects 페이지** 기반)

| 학교     | 랩 / PI                                              | 소속(학과/스쿨)                                      | 핵심 키워드(공식 소개 기준)                                                                               | 대표 연구축 (링크 기준)                                                                                                                                         | 휴머노이드/듀얼암·E2E/VLA/IL 적합도                                | 리스크                                                                                              | 추천도 순위 |
| ------ | --------------------------------------------------- | ---------------------------------------------- | ---------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------- | ------------------------------------------------------------------------------------------------ | ------ |
| KAIST  | **RIRO Lab / Daehyung Park**                        | **School of Computing**                        | **Humanoid whole-body manipulation**, **policy learning(IL/RL)**, **LLM/VLA 기반 계획**, 실세계 로봇 지능 | 토픽 페이지에서 **휴머노이드 전신 조작**을 전면에 두고, **IL/RL + LLM/VLA planning**을 함께 가져가는 방향을 명시                                                                         | **매우 높음** (휴머노이드/전신+조작+학습+계획을 한 프레임으로 명시)               | 휴머노이드 실물 PoC는 **안전/하드웨어 안정성/데이터 수집 비용**이 큼. PoC 범위를 잘 쪼개야 함(처음엔 듀얼암/고정 베이스로 시작 후 확장 등).          | **1**  |
| Yonsei | **RL & Robot Learning Lab (RLLAB) / Youngwoon Lee** | (연세대) 로봇러닝·RL/IL 중심 연구실(홈페이지/퍼블리케이션 기반)        | **RL, Imitation Learning, Generative modeling, Robotics**                                      | publications에 **TwinVLA**처럼 **single-arm VLA를 조합해 bimanual(듀얼암)로 확장**하는 연구를 명시                                                                         | **높음** (듀얼암+VLA/IL로 “데이터 적게” bimanual을 만들 수 있는 방향성이 강함) | 바퀴 교체는 **토크/접촉 제약**이 크므로, 비전 중심 E2E만으로는 한계 가능 → force/torque/tactile, 안전 제약을 정책에 넣는 설계가 필요       | **2**  |
| KAIST  | **CLVR Lab / Jaechul School of AI 교수진(페이지 기준)**     | **Kim Jaechul School of AI**(교수 페이지 표기)        | (링크 기반으로는) 로봇 조작 학습, 데이터/벤치마크, 언어-학습 신호 등으로 드러남                                                | publications에 **FurnitureBench/조립**, **DROID/대규모 조작 데이터**, **Teleop 보조(PATO)**, **오프라인 스킬 추출(EXTRACT)**, **언어 기반 reward/신호(ReWiND/SPRINT)** 등이 확인      | **중~높음** (바퀴 교체를 “조립/장기 시퀀스 + 데이터/스킬 재사용” 문제로 푸는 데 강점)  | 듀얼암/휴머노이드 **플랫폼 자체를 강하게 명시**하기보단 데이터/학습/벤치마크 축이 강함 → 하드웨어 통합(툴/토크) 파트는 공동 설계 필요                  | **3**  |
| SNU    | **Biointelligence Lab / Byoung-Tak Zhang**          | **School of Computer Science and Engineering** | **Embodied AI**, **vision-language-action 통합**, grounded reasoning, sensorimotor learning      | projects에서 **Embodied AI(로봇 파운데이션 모델)**, **Arm robot manipulation**, **자연어 기반 평생학습**, **ROS 기반 실세계 로봇 제어**, **Alchemist(humanoid capabilities)** 등을 명시 | **중~높음** (VLA/추론+행동 통합의 “시스템” 산학에 매우 강함)                | 바퀴 교체 같은 공정은 “접촉/툴/정밀 체결”이 핵심인데, 여기서는 **VLA/추론/통합 시스템** 비중이 더 커 보임 → PoC를 “추론+정책 통합 데모”로 잘 잡아야 함 | **4**  |

### 한 줄 결론(1개만 고른다면)

* **휴머노이드까지 포함한 end-to-end(계층형 포함)로 “바퀴 교체”를 가장 정면으로 밀기 좋은 1순위**는 **KAIST RIRO**야. (토픽 페이지에서 휴머노이드 전신 조작 + IL/RL + LLM/VLA 계획을 한 덩어리로 명시)

원하면, 위 “1순위(리로)”로 컨택할 때 바로 붙일 수 있게 **바퀴 교체 태스크를 8~10개 서브태스크로 분해한 PoC 스펙(성공지표/데이터 계획/안전 제약 포함) 1페이지**도 같이 만들어줄게.
#####################  
#########################  
###########################  
좋아—오성희 교수님 RLLAB(SNU) 연구/프로젝트 페이지를 웹에서 확인해서, 기존 표와 같은 기준(키워드·연구축·바퀴교체 적합도·리스크·순위)으로 한 줄 추가해 업데이트할게.


아래는 **서울대 오성회( Songhwai Oh ) 교수님 RLLAB**을 포함해 **5개 랩**으로 확장한 표야.
참고로 사용자가 주신 `rllab.snu.ac.kr/research`는 제 쪽에서 접속이 간헐적으로 타임아웃이 나서(페이지 자체는 검색엔 잡힘), **같은 공식 도메인의 PI/랩 소개 + SNU ECE 공식 연구그룹 페이지 + RLLAB publications/software 페이지**를 근거로 정리했어. 

> 태스크: **듀얼암/휴머노이드가 다른 로봇의 바퀴를 교체(분해→정렬→체결)하는 장기·접촉-rich 작업** 기준

| 학교     | 랩 / PI                                                   | 소속(학과/스쿨)                                                | 핵심 키워드(공식 소개 기준)                                                                        | 대표 연구축                                                                                                                                        | 휴머노이드/듀얼암·E2E/VLA/IL 적합도                                                                                      | 리스크                                                                                            | 추천도 순위 |
| ------ | -------------------------------------------------------- | -------------------------------------------------------- | --------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------- | ------ |
| KAIST  | **RIRO Lab / Daehyung Park**                             | **School of Computing**                                  | **Humanoid whole-body manipulation**, **policy learning(IL/RL)**, **LLM/VLA 기반 계획**     | 휴머노이드 전신 조작 + IL/RL + LLM/VLA 계획을 한 프레임으로 명시                                                                                                  | **매우 높음**                                                                                                     | 휴머노이드 실물 PoC는 안전/데이터/하드웨어 리스크 큼 → 단계적 스코프(듀얼암→휴머노이드) 설계 필요                                     | **1**  |
| Yonsei | **RL & Robot Learning Lab / Youngwoon Lee**              | (연세대) RLLAB 사이트/퍼블리케이션 기반                                | **RL, Imitation Learning, Generative modeling, Robotics**                               | **TwinVLA** 등: single-arm VLA를 조합해 **bimanual(듀얼암) VLA**로 확장                                                                                  | **높음** (듀얼암 + VLA/IL에 직접 연결)                                                                                  | 바퀴 교체는 토크/접촉 제약이 커서 비전 중심 E2E만으로는 한계 가능 → 힘/토크/안전 제약 통합 설계 필요                                  | **2**  |
| KAIST  | **CLVR Lab / (KAIST) Joseph J. Lim 그룹**                  | KAIST (Publications 페이지 기반)                              | (페이지에 드러난 축) 조작 학습, 대규모 데이터, 오프라인 스킬/언어 신호                                              | **FurnitureBench/가구 조립**, **DROID/RT-X류 대규모 조작 데이터**, **PATO(teleop 지원)**, **EXTRACT(오프라인 스킬 추출)**, **ReWiND/SPRINT(언어 기반 학습 신호)**            | **중~높음** (조립형 장기 시퀀스 + 데이터/스킬 재사용에 강함)                                                                        | “휴머노이드/듀얼암 플랫폼”을 전면에 명시하기보단 데이터/학습 축이 강함 → 하드웨어(공구/토크) 통합은 공동 설계가 필요                           | **3**  |
| SNU    | **Biointelligence Lab / Byoung-Tak Zhang**               | **SNU CSE**                                              | **Embodied AI**, **vision-language-action 통합**, grounded reasoning, lifelong adaptation | 프로젝트에 **Embodied AI(로봇 파운데이션 모델)**, **arm manipulation**, **언어 기반 평생학습**, **Alchemist(휴머노이드 역량)** 등을 명시                                       | **중~높음** (VLA/추론+행동 통합 “시스템”에 매우 강함)                                                                          | “바퀴 교체”처럼 공구·체결 중심 공정은 접촉/토크가 승부처 → VLA/추론 강점을 **현장 실행 안정화**로 연결하는 PoC 설계가 중요                  | **4**  |
| SNU    | **Robot Learning Laboratory (RLLAB) / Songhwai Oh(오성회)** | **SNU ECE(전기·컴퓨터공학부)** 공식 연구그룹에 등재 ([서울대학교 전기·정보공학부][1]) | robotics + computer vision + machine learning ([rllab.snu.ac.kr][2])                    | publications/software에서 **Learning from Demonstrations/Behavior Cloning/Inverse RL**, 로봇 그라스핑·비주얼 플래닝, 모방학습 관련 항목이 확인됨 ([rllab.snu.ac.kr][3]) | **중간** (IL/RL 기반 “로봇학습” 역량은 강하지만, 이 페이지 근거로는 VLA·듀얼암/휴머노이드·E2E 조작을 전면에 내세우는 형태는 덜 명시적) ([rllab.snu.ac.kr][4]) | 바퀴 교체는 듀얼암/전신+툴링이 핵심인데, 공개 페이지 근거만으로는 해당 플랫폼/조작 파이프라인이 확정적이지 않음 → 미팅에서 최근 조작/매니퓰레이션 프로젝트 확인 필요 | **5**  |

### 1개만 뽑아야 한다면(현재 정보 기준)

* **RIRO(1순위)**: “휴머노이드 전신 조작 + IL/RL + LLM/VLA 계획”을 **토픽 페이지에 명시**해서, 바퀴 교체 같은 **현장형 장기·접촉 task**에 가장 정면으로 맞아.

원하면, “바퀴 교체”를 **8~10개 서브태스크**로 분해해서(볼트 접근/툴 그립/풀기/정렬/체결/검증/실패복구), RIRO에 보내기 좋은 **PoC 1페이지 제안서(기간·데이터·KPI·안전 포함)** 형태로 바로 만들어줄게.

[1]: https://ece.snu.ac.kr/en/research-faculty/research/groups?labidx=66&md=view&utm_source=chatgpt.com "Robot Learning Laboratory - Research Groups - Faculty & ..."
[2]: https://rllab.snu.ac.kr/people/songhwai-oh?utm_source=chatgpt.com "Songhwai Oh"
[3]: https://rllab.snu.ac.kr/software?utm_source=chatgpt.com "Software from RLLAB — Robot Learning Laboratory"
[4]: https://rllab.snu.ac.kr/publications?utm_source=chatgpt.com "Publications"
