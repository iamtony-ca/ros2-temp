좋다—코드 쓰기 전에 “어떻게 움직일지”를 깔끔하게 정리하자. 아래는 **Decision Node**의 정책/설계 초안이야. 키워드는 영어(Policy, State, TTC 등)로, 설명은 한국어로 풀었어. 이대로 가면 바로 구현하기 쉬워.

# 목적

* 주행 중 **다른 로봇(agents)** 또는 **비-에이전트 장애물**로 인해 경로 리스크가 감지되면, 본인 로봇의 행동을 **Slowdown / Yield / Reroute / Stop** 중 하나로 결정하고 안정적으로 전환.
* 신뢰 가능한 트리거:

  * `/path_agent_collision_info` (에이전트 기인 충돌 위험)
  * `/replan_flag` (비-에이전트 장애물에 의한 경로 차단)
  * `/multi_agent_infos` (상대 로봇들의 현재 상태/진로/속도 추정)
* Nav2와의 연동: 속도 제한(velocity cap), BT blackboard 플래그, Replan 요청, Pause/Resume 등.

---

# 입력/출력

## Subscriptions

1. **`/path_agent_collision_info`** (`PathAgentCollisionInfo`)

   * 다수의 충돌 후보: `(machine_id[], type_id[], x[], y[], ttc_first[], note[])`
   * **agent-caused collision**을 빠르게 인지하는 1순위 입력.

2. **`/replan_flag`** (`std_msgs/Bool`)

   * **non-agent** 차단. true 펄스가 오면 비-에이전트 원인의 차단으로 분류.

3. **`/multi_agent_infos`** (`MultiAgentInfoArray`)

   * 상대 로봇들의 **status/mode**, **current_pose/twist**, **truncated_path**, **footprint**.
   * 우측통행/진로 충돌, 교차지점 우선권 등 **right-of-way 정책**에 활용.

4. (선택) **`/current_velocity`** 또는 odom

   * 본인 속도 기준 speed scaling, Stop/Resume 히스테리시스.

## Publications

1. **`/behavior_decision`** (custom, e.g., `replan_monitor_msgs/BehaviorDecision`)

   * 현재 Behavior: `RUN | SLOWDOWN | YIELD | REROUTE | STOP`
   * reason / culprit agents / timers 등의 메타 포함

2. **`/cmd_vel_limit`** (`geometry_msgs/Twist` 또는 커스텀)

   * 최대 선속/각속 제한. Nav2 controller(eg. RegulatedPurePursuit, graceful) 파라미터 runtime update 대신 토픽 기반 limiter로 간단히 적용.

3. **`/yield_to_agent`** (`std_msgs/UInt16[]` or custom)

   * 양보 대상 agent IDs (downstream 로거/UX/모니터링용)

4. **`/request_replan`** (`std_msgs/Bool`)

   * 내부 정책으로 재계획을 트리거하고 싶을 때(예: agent 교차 TTC가 아주 작을 때 미리 선제적 Reroute)

---

# QoS / 신뢰성

* `/path_agent_collision_info`: **Reliable, Keep Last(10)** (짧은 이벤트 놓치지 않도록)
* `/replan_flag`: **Reliable, Keep Last(1)** (펄스 기반)
* `/multi_agent_infos`: **Best Effort, Keep Last(10)** (주기적 스트림, 누락 허용)
* Output 토픽: 기본 Reliable.

---

# Core Policy (상태기계)

## States

* **RUN**: 정상 주행
* **SLOWDOWN**: 속도 제한 (예: 0.3 m/s)
* **YIELD**: 상대에게 양보(거의 정지 근접 속도 제한, 또는 정지 후 release)
* **REROUTE**: Nav2에 재계획 요청 후 RUN/Slowdown으로 복귀
* **STOP**: 완전 정지(안전 최후 수단)

### 전이(Transitions) 개요

1. **RUN → SLOWDOWN**

   * agent TTC in (TTC_slowdown_low, TTC_slowdown_high) 범위
   * 또는 cross-path risk (same-lane closing) 낮음-중간

2. **RUN/SLOWDOWN → YIELD**

   * agent TTC ≤ TTC_yield
   * right-of-way 정책상 양보 필요 (상대가 main corridor 우선, 또는 나보다 우선권 높은 상황)
   * 교차/합류지점에서 상대가 이미 진입 중이고 나의 stopping distance 내

3. **RUN/SLOWDOWN/YIELD → REROUTE**

   * `/replan_flag` 수신(비-에이전트 차단) **AND** (agent_hold_timer==0)
     → 즉, agent 충돌 이벤트 직후에는 바로 Reroute 하지 않도록 **에이전트-우선 홀드(hold)** 유지
   * 또는 **agent TTC 매우 낮음** & local deadlock 패턴(일정 시간 이상 YIELD 상태 지속) 감지 시 선제 Reroute

4. **ANY → STOP**

   * 안전 한계: emergency condition (agent distance < d_emergency, 또는 속도×반응시간 내 정지 불가)
   * 센서 신뢰도 하락/TF 오류 등 시스템 에러

5. **SLOWDOWN/YIELD/STOP → RUN**

   * **release 조건** 충족 (아래 Release Policy 참고) + hysteresis 타이머 만료

---

# Collision 판단/우선권(right-of-way)

## Agent-based 판단

* `/path_agent_collision_info` 내 각 충돌점에 대해:

  * **TTC(초)**, **거리**, **closing speed** 기반 Severity Score:

    * `severity = w1*(1/max(ttc, eps)) + w2*(1/distance) + w3*same_lane_bonus + w4*crossing_bonus`
  * 가장 큰 severity의 agent를 **primary culprit**로 선정 (behavior에 기록)
* **same-lane vs crossing**:

  * 상대 truncated_path의 접선 방향과 내 진행방향 각도차 < θ_same_lane → same-lane
  * 교차점 근처(trajectory intersection) 감지되면 crossing
* **right-of-way 룰(간단 버전)**:

  * main corridor에 이미 진입한 로봇 우선
  * 좁은 통로에서 **상류(먼저 들어온) 우선**, 후진 반출이 어려운 쪽 우선
  * 동일 우선시 **machine_id** tie-break (작은 ID 우선 또는 정책화)
  * 이 룰 위반 시 내 행동은 **YIELD**로 세팅

## Non-agent 판단

* `/replan_flag` true:

  * agent_hold 윈도(예: 1.0~2.0s) 안이라면 아직 Reroute하지 말고 **SLOWDOWN/YIELD 유지**
  * hold 끝나고도 여전히 `/replan_flag` 재발 시 **REROUTE** 트리거

---

# Timers / Hysteresis

* **agent_hold_timer (s)**:

  * `/path_agent_collision_info` 수신 시 **hold 시작**(예: 2.0s).
  * hold 중에는 `/replan_flag`가 와도 바로 Reroute하지 말고 **agent 문제에 집중**(YIELD/Slowdown 유지).
  * 같은 agent ID에서 연속 이벤트 시 hold 연장(최대치 clamp).
* **behavior_min_duration (s)**:

  * 상태 변환 후 최소 지속시간을 보장(예: 0.7s) → 상태 깜빡임 방지.
* **release_hysteresis (s)**:

  * 위험 해소 후 RUN으로 돌아가기 전에 유지(예: 0.5s) → 살짝 흔들려도 바로 RUN으로 복귀하지 않도록.

---

# Release Policy (위험 해소 판정)

* 최근 `K` 주기 동안 다음 모두 만족:

  * `/path_agent_collision_info` 미수신
  * 최근 TTC 추정치가 `TTC_release` 이상 (예: > 5s)
  * 최근 최소거리 `d_release` 이상 (예: > 2.0m)
  * `/replan_flag` 미발생
* 만족 시:

  * **YIELD → SLOWDOWN** (0.5~1.0s) → **RUN** 단계적으로 복귀(속도 제한 완화 ramp)

---

# Speed Control (Slowdown/Yield 구현)

* **SLOWDOWN**: `v_max = min(v_nominal, v_slow)` (예: 0.3 m/s)
* **YIELD**: `v_max = v_yield` (거의 정지 수준, 예: 0.05~0.1 m/s) 또는 정지 명령
* **STOP**: `v_max = 0`, `omega_max = 0`
* `v_max`/`omega_max`는 `/cmd_vel_limit`로 퍼블리시 → controller에서 적용.
  (대안: controller server dynamic parameter update, 또는 BT blackboard 플래그로 speed_scale 적용)

---

# Reroute Policy

* **조건**:

  * hold 기간 종료 후 `/replan_flag`가 재발하거나 지속
  * YIELD가 `T_deadlock` 초 이상 지속(예: 6~10s)
  * same-lane face-to-face(정면 교행)에서 양쪽 모두 YIELD로 장시간 교착 시
* **행동**:

  * `/request_replan` true 펄스 발행
  * Behavior를 잠깐 **REROUTE**로 전이 → planner 완료 or 신호 감지 시 RUN/Slowdown 복귀

---

# 데이터 신선도/유효성

* `/multi_agent_infos` **freshness_timeout_ms** 관리(예: 1000ms).
  넘으면 agent 기반 판단 무시(ghost 방지), 대신 `/replan_flag` 중심으로 판단.
* `/path_agent_collision_info`가 큰 `agent_mask_manhattan_buffer`로 발생한 넓은 영역인 경우,
  collision point 근방의 실제 agent footprint 포함 여부를 cross-check(현재 구조 그대로).
* 좌표 프레임 확인: 모든 입력이 `map` 기준임을 보장(다르면 transform 실패 시 skip).

---

# 파라미터(제안값)

* **TTC_yield**: 2.5 s (이하 → YIELD)
* **TTC_slowdown_low/high**: 2.5~6.0 s (범위 내 → SLOWDOWN)
* **d_emergency**: 0.8 m (이내면 STOP)
* **behavior_min_duration**: 0.7 s
* **agent_hold_sec**: 1.5 s
* **release_hysteresis**: 0.5 s
* **deadlock_sec**: 8.0 s
* **v_slow**: 0.30 m/s
* **v_yield**: 0.08 m/s
* **v_nominal**: 0.6~1.0 m/s (로봇 성능/환경에 맞춰)
* **severity weights**: w1=1.0, w2=0.2, w3=0.4, w4=0.3 (필드 튜닝)

---

# Decision 흐름(요약)

1. 수신 이벤트 업데이트:

   * 새 `path_agent_collision_info` → agent_hold 시작/연장, culprit 선정, TTC/거리 계산
   * 새 `replan_flag` → hold 중이면 무시, hold 끝났고 재발이면 Reroute 후보
   * 새 `multi_agent_infos` → right-of-way/교차/차선 판단에 반영
2. 위험도 평가(Severity, TTC, 거리, same-lane/crossing, 우선권)
3. 상태 전환(behavior_min_duration, release_hysteresis 고려)
4. 속도 제한/명령 퍼블리시(`/cmd_vel_limit`) + 상태 브로드캐스트(`/behavior_decision`)
5. 필요 시 `/request_replan` 펄스
6. 타이머/카운터 업데이트

---

# 구현 팁

* **Callback Group**: MutuallyExclusive 2개(Subs/Timers 분리) → 상태기계 race 방지.
* **Clock/Time Source**: node clock 사용. 이전 문제처럼 time source 달라서 예외 발생하지 않게 동일 clock type 유지.
* **Metrics/Debug**:

  * `/behavior_decision`에 현재 state, primary agent id(s), TTC min, hold 남은 시간, last trigger reason 포함.
  * throttle 로그: 전이 시점, release 시점, deadlock 진입/해제.

---

# 예외/엣지케이스

* **collision topic 폭주**: 동일 agent로부터 초당 다수 이벤트 → debouncing (N ms 내 중복 무시).
* **agent와 non-agent 동시**: agent_hold 우선. hold 종료 후에도 non-agent 지속이면 Reroute.
* **ghost cost**: agent-mask stale 시 collision 인식됐는데 culprit 매칭 실패 → 비-에이전트로 다운그레이드(슬로다운만).

---

# 테스트 플랜(간단)

1. **단일 교차**: agent가 내 경로를 가로지름 → SLOWDOWN → YIELD → release 후 RUN.
2. **same-lane 추돌**: 앞선 agent 느리게 주행 → SLOWDOWN, TTC 증가하면 RUN 복귀.
3. **비-에이전트 박스**: `/replan_flag`만 발생 → agent_hold 없을 때 즉시 REROUTE.
4. **경합/데드락**: face-to-face 좁은 통로, 양쪽 YIELD 장시간 → deadlock_sec 이후 REROUTE.
5. **sensor drop**: `/multi_agent_infos` 끊김 → agent 판단 스킵, non-agent 흐름만 동작.

---

원하면 바로 이 설계대로 ROS2 노드 코드를 짜줄게. 함수/파라미터 시그니처까지 맞춰서 drop-in 하도록 해줄 수 있어.
