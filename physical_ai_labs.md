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

