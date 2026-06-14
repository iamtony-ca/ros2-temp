https://developer.nvidia.com/blog/r2d2-unlocking-robotic-assembly-and-contact-rich-manipulation-with-nvidia-research/

해당 링크 내용을 한국말로 설명해줄래?

핵심부터 말하면, **7개는 같은 종류가 아닙니다.** 서로 경쟁하는 7개의 대안이 아니라, 하나의 contact-rich 파이프라인에서 **서로 다른 역할(층)을 맡는 조각들**입니다. "프레임워크냐 방법론이냐"의 답도 제각각입니다. 각자가 답하는 질문으로 구분하면 가장 명확합니다.

- **Factory** — "접촉을 어떻게 빠르게 시뮬레이션하지?" → **시뮬레이션 엔진/프레임워크** (인프라). 나머지가 모두 이 위에 올라갑니다.
- **TacSL** — "촉각 센서를 어떻게 시뮬레이션하지?" → **센서 시뮬레이션 라이브러리** (인프라, Factory와 병렬)
- **MatchMaker** — "학습할 부품(에셋)을 어디서 구하지?" → **에셋 생성 도구/파이프라인**
- **IndustReal** — "sim에서 배운 정책을 실로봇에서 어떻게 동작시키지?" → **sim-to-real 방법론/알고리즘 모음**
- **FORGE** — "pose 불확실성을 force로 어떻게 흡수하지?" → **sim-to-real 방법론** (force 입력 버전)
- **AutoMate** — "여러 형상의 정책을 어떻게 대량으로 학습하지?" → **정책 학습 프레임워크**
- **SRSA** — "이미 학습한 스킬을 새 task에 어떻게 재사용하지?" → **스킬 재사용 방법론/프레임워크**

즉 크게 나누면 **(1) 시뮬레이션 인프라(Factory, TacSL), (2) 학습 재료=에셋(MatchMaker), (3) sim-to-real 학습 방법(IndustReal, FORGE), (4) 정책 운용 프레임워크(AutoMate, SRSA)** 네 부류입니다. 층으로 그리면 이렇게 쌓입니다:

(시각화 도구가 응답하지 않아서 텍스트로 정리하겠습니다.)

```
┌─────────────────────────────────────────────────────────┐
│ (4) 정책 운용 프레임워크 — "여러/기존 정책을 어떻게 다루나"     │
│     • AutoMate  : 다형상 정책 대량 학습 (RL+IL)  [프레임워크]  │
│     • SRSA      : 기존 스킬 재사용·적응          [방법론]      │
├─────────────────────────────────────────────────────────┤
│ (3) sim-to-real 학습 방법 — "sim 정책을 실로봇에서 되게 하나"  │
│     • IndustReal: 전이 알고리즘 모음            [방법론]      │
│     • FORGE     : force 기반 전이              [방법론]      │
├─────────────────────────────────────────────────────────┤
│ (2) 학습 재료(에셋) — "훈련할 부품을 어디서 구하나"            │
│     • MatchMaker: 조립 에셋 쌍 자동 생성        [도구]        │
├─────────────────────────────────────────────────────────┤
│ (1) 시뮬레이션 인프라 — "접촉/촉각을 어떻게 시뮬레이션하나"     │
│     • Factory   : 접촉 물리 엔진 (SDF, GPU)     [프레임워크]   │
│     • TacSL     : 촉각 센서 시뮬레이션          [라이브러리]   │
└─────────────────────────────────────────────────────────┘
                  ↑ 위 모든 것이 Factory 위에 올라감
```

## "프레임워크 vs 방법론" 구분이 헷갈리는 이유

사실 이 구분이 안 와닿는 건 자연스럽습니다. **이들 대부분은 "논문 + 공개 코드" 형태**라, 보는 각도에 따라 둘 다로 불립니다. 정확히는:

- **프레임워크/라이브러리** = 그 위에 올려 쓰는 재사용 소프트웨어 기반 → Factory, TacSL, AutoMate
- **방법론/알고리즘** = 하나의 기법·레시피(논문으로 발표, 코드 동반) → IndustReal, FORGE, SRSA
- **도구/파이프라인** = 무언가를 만들어내는 처리기 → MatchMaker(에셋을 산출)

경계가 흐린 이유는 예컨대 FORGE가 "방법론"이면서 동시에 Factory 위에 구현된 "코드 환경"이기도 하기 때문입니다. 그래서 **"어떤 카테고리냐"보다 "어떤 역할(질문)을 하느냐"로 보는 게** 실무에서 훨씬 명확합니다.

## 사용자 프로젝트에 대입하면

7개를 다 쓰는 게 아닙니다. 사용자가 실제로 타는 것만 추리면:

- **Factory** ✅ — 사용자의 `forge_env_cfg.py`, `factory_wheel_held.usd`가 다 Factory 기반(Isaac Lab에 포함됨). 접촉 시뮬레이션의 토대.
- **FORGE** ✅ — "FORGE port"를 진행 중이라 하셨죠. force 입력 + 5mm pose 오차 흡수가 핵심 방법론.
- **IndustReal** ✅(개념) — sim-to-real 전이 레시피(SDF 보상, 커리큘럼 등)를 차용.
- **TacSL** ❌ — 전용 촉각 센서(GelSight) 전제라 F/T-only 구성과 안 맞음.
- **MatchMaker** △ — 사용자는 에셋을 직접 author하므로 불필요. 단 clearance 조절 발상은 참고됨.
- **AutoMate / SRSA** △ — 여러 형상·여러 task를 다룰 때 의미. 지금은 단일 wheel-shaft task라 당장은 불필요. (나중에 DFK+SEMES 등 형상이 늘면 SRSA가 유용해질 수 있음.)

정리하면, 사용자에게 실질적으로 의미 있는 건 **Factory(토대) + FORGE(방법) + IndustReal(전이 레시피)** 세 가지이고, 나머지는 "필요해지면 꺼내 쓰는 선택지"로 두시면 됩니다.

혹시 이 중 IndustReal과 FORGE의 방법론적 차이(언제 어느 쪽을 쓰나)를 더 구체적으로 비교해드릴까요?

##
##
https://developer.nvidia.com/blog/bridging-the-sim-to-real-gap-for-industrial-robotic-assembly-applications-using-nvidia-isaac-lab/

해당 링크도 자세히 설명해줄래?

이 글은 앞의 R²D² 개요와 달리, **실제 UR10e gear assembly sim-to-real 데모를 어떻게 구현했는지** 구체적으로 풀어쓴 글입니다. 사용자의 wheel-shaft가 gear insertion과 거의 동일하므로, 이 글이 사실상 **사용자 프로젝트의 가장 가까운 레퍼런스 구현**입니다. 핵심을 단계별로 정리하고, 끝에 사용자 접근(FORGE port)과의 중요한 차이를 짚겠습니다.

## 1. 이 데모가 무엇인가

Isaac Lab에서 설계·학습한 gear assembly task를 UR10e에 zero-shot sim-to-real로 전이한 것으로, Isaac ROS와 UR의 저수준 torque interface로 배포 [arxiv](https://arxiv.org/pdf/2604.06949) 했습니다. NVIDIA-UR 협업이고, reality gap은 IndustReal의 알고리즘·도구 개념을 써서 메웁니다. [arxiv](https://arxiv.org/pdf/2604.06949) 즉 이건 **IndustReal 레시피 기반**입니다 (FORGE 아님 — 이게 뒤에서 중요).

## 2. Task를 어떻게 쪼갰나 — 스킬 분해

gear assembly는 인지·grasp·운반·삽입으로 이뤄지고, 3개의 핵심 스킬 — grasp 생성, 자유공간 motion 생성, 삽입 — 이 필요 [arxiv](https://arxiv.org/pdf/2604.06949) 합니다. 중요한 설계 결정:
- grasp 생성은 기성(off-the-shelf) grasp planner로 처리 [arxiv](https://arxiv.org/pdf/2604.06949) (학습 안 함).
- motion 생성과 삽입만 RL로 학습 [arxiv](https://arxiv.org/pdf/2604.06949) .
- motion 생성은 고전 trajectory planner로도 잘 되지만, RL로 학습해보는 것이 더 어려운 삽입 task에 앞서 학습 프레임워크를 캘리브레이션·디버깅하는 유용한 단계 [arxiv](https://arxiv.org/pdf/2604.06949) 가 됩니다.

→ **단일 monolithic 정책이 아니라 스킬별 분리 학습**이고, 쉬운 motion-gen을 먼저 돌려 프레임워크를 검증한 뒤 어려운 삽입으로 넘어가는 순서입니다. 사용자도 그대로 차용할 만한 실전 프로세스입니다.

## 3. 두 개의 RL 문제 (독립적으로 학습)

motion 생성과 삽입은 각각 별개의 RL 문제로 독립 학습 [arxiv](https://arxiv.org/pdf/2604.06949) 됩니다.

- **Motion generation**: 작업공간 내 랜덤 초기 관절각에서 시작해 EE를 목표 pose로 이동. 관측은 로봇 관절 위치 + 목표 EE pose, 행동은 관절 위치 목표, 보상은 EE-목표 거리 최소화 + 급격한 동작 페널티 [arxiv](https://arxiv.org/pdf/2604.06949) .
- **Insertion**: 기어가 그리퍼에 쥐어진 채 목표 샤프트 근처 랜덤 pose에서 초기화되고, 목표는 기어를 샤프트 바닥까지 이동. 관측은 관절 위치 + 목표 샤프트 pose, 행동은 관절 위치 목표, 보상은 기어-목표 거리 최소화 + 급격한 동작 페널티 [arxiv](https://arxiv.org/pdf/2604.06949) .

여기서 핵심: **관측에 F/T가 없습니다.** 관절 위치 + 목표 pose뿐입니다. 이게 제가 ①에서 말한 "F/T 없이 출발 가능"의 근거입니다.

## 4. 제어 구조 — 60Hz 정책 / 500Hz impedance

관절 위치 목표는 60Hz로 생성되고, 더 낮은 수준의 impedance 컨트롤러가 이를 실행 [arxiv](https://arxiv.org/pdf/2604.06949) 합니다. 즉 **두 개의 레이트**가 있습니다 — 정책 60Hz, impedance 500Hz. 사용자 spec 시트의 "500Hz impedance"가 바로 이 저수준 부분이고, 여기에 더해 정책 출력은 60Hz라는 점을 추가로 알 수 있습니다.

왜 impedance인가: position 컨트롤러는 정확하지만 경직돼서 인지·정렬 오차가 있을 때 큰 의도치 않은 힘을 유발하는 반면, impedance control은 안전하고 compliant한 상호작용을 가능하게 [arxiv](https://arxiv.org/pdf/2604.06949) 합니다. impedance control은 직접 토크 명령이 필요한데 산업용 로봇엔 드물어서, UR이 direct torque control 인터페이스를 early access로 제공 [arxiv](https://arxiv.org/pdf/2604.06949) 합니다.

→ 이게 사용자의 near-zero clearance와 직결됩니다. 팔 repeatability(50µm)가 clearance(27µm)보다 큰 상황에서, **compliance가 chamfer와 함께 마지막 정렬을 기계적으로 흡수**하는 게 핵심 메커니즘입니다.

## 5. 네트워크·학습 디테일

각 정책은 256 유닛 LSTM 뒤에 256/128/64 뉴런 3층 MLP를 붙인 구조이고, rl-games의 PPO로 학습 [arxiv](https://arxiv.org/pdf/2604.06949) 했습니다. Isaac Sim 4.5, Isaac Lab 2.1, RTX 4090에서 학습 [arxiv](https://arxiv.org/pdf/2604.06949) 됐습니다.

두 가지 짚을 점:
- **LSTM(recurrent)** 입니다. 제가 앞서 "작은 MLP"라 단순화했는데, 정확히는 recurrent라 **시간에 걸친 접촉 이력을 통합**할 수 있습니다. F/T 없이 proprioception만으로 접촉을 암묵 감지하려면 이 시간적 메모리가 도움이 됩니다 — 사용자도 LSTM 채택을 권합니다.
- **RTX 4090으로 충분**했으니 사용자의 **RTX 5090은 차고 넘칩니다.**

도메인 randomization: 초기 팔 pose, 그리퍼 내 기어 pose, 기어 크기, task 단계를 다양화하고, sim-to-real을 위해 로봇 동역학(관절 마찰·damping)과 컨트롤러 gain, 관측 노이즈에 domain randomization을 적용 [arxiv](https://arxiv.org/pdf/2604.06949) 했습니다.

## 6. 배포 파이프라인

RGB 이미지를 Segment Anything에 넣어 세그멘테이션 마스크를 얻고, 이를 depth와 결합해 FoundationPose로 기어의 6D pose를 추정 [arxiv](https://arxiv.org/pdf/2604.06949) 합니다. 그다음 기어 pose와 UR 엔코더의 관절 위치가 관측이 되어 정책이 delta 관절 위치를 예측하고, 이를 절대 목표 관절 위치로 변환해 impedance 컨트롤러에 입력하며, URScript로 만든 저수준 impedance 컨트롤러가 500Hz로 관절 토크를 계산 [arxiv](https://arxiv.org/pdf/2604.06949) 합니다.

## 7. 사용자 접근(FORGE port)과의 중요한 차이 — 꼭 확인하세요

여기서 짚어야 할 게 있습니다. 이 블로그 데모와 사용자 spec 시트 사이에 미묘한 불일치 가능성이 있습니다.

- 이 블로그: 두 task의 정책이 Isaac Lab에 구현된 저수준 impedance 컨트롤러에 의존 [arxiv](https://arxiv.org/pdf/2604.06949) — 즉 **sim 학습 루프 안에도 impedance(compliant) 제어가 들어갑니다.**
- 사용자 spec 시트: "sim에서는 stiff joint PD로 학습하고, 500Hz impedance는 배포 전용 아티팩트"라고 적혀 있습니다.

이 차이의 가장 그럴듯한 설명은 **사용자가 IndustReal(이 블로그)이 아니라 FORGE를 port 중**이라는 점입니다. 두 레시피는 다릅니다 — 이 블로그는 IndustReal + impedance(force 관측 없음) 기반이고, FORGE는 force 관측을 입력으로 쓰는 별개 레시피입니다. 그래서 action space·저수준 제어 설계가 다를 수 있습니다.

다만 확인하실 점: **만약 sim은 stiff PD(경직)인데 실제는 compliant impedance라면 System Response 불일치**가 생깁니다(앞서 gear assembly 배포 문서가 강조한 그 일관성). sim 저수준 컨트롤러의 유효 compliance가 실제 500Hz impedance와 맞는지 `phase9_forge_port_plan.md`에서 점검해 보시길 권합니다. ("stiff joint PD"는 수학적으로 high-gain joint-space impedance라, gain만 맞추면 같은 것이 될 수도 있습니다 — 핵심은 compliance 수치 정합입니다.)

## 사용자 프로젝트에 주는 실전 시사점

1. **스킬 분해**: grasp(planner) + 운반(motion-gen RL) + 삽입(RL)을 분리 학습. monolithic 금지.
2. **motion-gen 먼저**: 쉬운 motion-gen으로 프레임워크 검증 후 삽입 진입.
3. **LSTM 채택**: 시간적 접촉 통합 → F/T 없는 구간에서 proprioceptive 접촉 감지에 유리.
4. **compliance가 핵심 메커니즘**: near-zero clearance는 impedance compliance + chamfer로 흡수. 단 이 데모의 기어보다 사용자 공차(0–27µm)가 더 빡빡해서, 더 어려운 영역 → tight 끝단에서 F/T 필요성 재확인.
5. **60Hz 정책 / 500Hz impedance** 레이트 구조를 그대로 참고.
6. RTX 5090은 충분.

이 블로그가 IndustReal 레시피인 점, 그리고 사용자가 FORGE를 가는 점의 차이를 더 깊게(IndustReal vs FORGE 언제 무엇을 쓰나) 비교해드릴까요?
