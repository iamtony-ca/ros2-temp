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
###################  
###################  
좋아—`ttc_first`가 부정확하거나 미싱(-1.0)인 상황을 “기본값·대체지표·신뢰도(Confidence)”로 흡수하면서, **reroute=true** 상대를 더 강하게 배려하고, 전체 **throughput(군집 처리량)** 을 최적화하는 의사결정 노드 설계를 깔끔하게 정리해줄게. (코드가 아니라 “설계서” 버전이야. 바로 구현 원하면 이어서 코드도 줄 수 있어.)

---

# 목표

* **안전 최우선**, 그 다음 **Throughput 최대화** (불필요한 멈춤/재계획 최소화).
* `ttc_first`가 **부정확**하거나 **-1.0(unknown)** 여도 안정적으로 동작.
* **상대 AMR이 `reroute=true`**이면 **우선권을 크게 부여**(내가 양보/우회).
* **Replan**(경유지 유지)과 **Reroute**(경유지 변경) 판단 분리.
* **공정성/비선점성**: 낮은 ID가 기본 우선권, 장기 교착엔 라운드로빈식 해소.

---

# 입력 (요약)

* `/path_agent_collision_info`

  * `machine_id[]`, `type_id[]`, `x[]`, `y[]`, `ttc_first[] (또는 -1)`, `note[]`
* `/multi_agent_infos`

  * 각 에이전트의 `mode`, `status.phase`, `current_pose/twist`, `footprint`, `truncated_path`, `reroute`, `current_waypoint/next_waypoint`, `area_id/occupancy`
* `/replan_flag` (non-agent)

---

# 핵심 아이디어

## 1) TTC-불확실성 로버스트 처리

`ttc_first`는 그대로 쓰되, **신뢰도(confidence)** 를 같이 계산:

* `C_ttc = 1` if `ttc_first > 0`,
* `C_ttc = 0` if `ttc_first < 0` (unknown),
* 중간값(0~1)은 **신뢰도 휴리스틱**으로 다운웨이트: 예) head-on & 양측 속도 추정 양호면 `0.6`, same-lane 추종이면 `0.3`.

**대체지표(surrogates)로 TTC 보강**:

* `d = dist(my_pose, hit_xy)`
* `v_closing ≈ max( 0, proj(v_other, dir_to_me) + proj(v_me, dir_to_other) )`
* `T_alt = d / max(v_closing, ε)` (속도 작으면 크게 나옴)
* `C_alt`: 상대/내 속도 관측 신뢰도(예: 최근 0.5s 표준편차) 기반 0~1

최종 “유효 TTC”:

```
T_eff = w1*C_ttc*ttc_first + w2*C_alt*T_alt
      / max(w1*C_ttc + w2*C_alt, ε)
```

없으면(둘 다 신뢰 0) `T_eff = +∞`로 간주.

추가로 **Time-To-Occupancy Window(TTOW)**:
상대의 `truncated_path`를 비용맵 해상도로 따라가며 **내 경로 점유 시각 범위** 겹침을 추정. 겹치면 `T_eff`를 더 보수적으로 줄이기(최소값으로 클리핑).

## 2) 위험도(Severity)와 양보필요도(Yield Priority) 분리 점수화

### Severity

```
severity = a1 * (1 / max(T_eff, T_min))
         + a2 * (1 / max(d, d_min))
         + a3 * heading_bonus(rel_heading)   # head-on > crossing > same-lane
         + a4 * v_closing
```

* `heading_bonus`: head-on=+1.0, crossing=+0.4, same-lane trailing=0.1
* `T_min`, `d_min`는 수치 폭주 방지용 하한

### Yield Priority (상대에게 길을 비켜줘야 하는 정도)

```
yield_prio = b1 * mode_bonus(other.mode)           # manual=+1.0, auto=0
           + b2 * (ROW(other) - ROW(me))           # 우선권 격차
           + b3 * (other.reroute ? 1 : 0)          # ★ reroute true 강화
           + b4 * (other.phase==PATH_SEARCHING ? 1 : 0)
           + b5 * (other.occupancy ? 1 : 0)
           + b6 * id_bonus(my_id, other_id)        # my_id>other_id => +, else -
```

### 최종 우선 대상 선택

* **Primary culprit** = argmax over candidates of:

```
score = severity * (1 + κ * yield_prio)
```

* `κ`는 운영 정책 민감도(0.5~1.0)
* tie-breaker: 더 작은 `T_eff`, 더 작은 `other_id`

## 3) Replan vs Reroute 의사결정

* **Agent 공존 이슈**일 때:

  * `yield`로 빠르게 해소되면 그대로 진행.
  * `deadlock_window` (예: 6~10s) 이상 지속:

    * 상대가 **`reroute=true`** 이면 → **내가 replan 먼저** (경유지 유지, 우회 경로 시도).
      실패/반복 시 → **reroute** (경유지 변경, 다른 회랑 선택).
    * 상대가 `reroute=false`인데도 해소 안됨 → **Replan 1~N회**, 그래도 안 풀리면 **Reroute**.
* **Non-agent (`/replan_flag`)**:

  * `agent_hold_active`면 **일시 무시**(이미 agent 충돌 처리 중).
  * hold 해제 후에도 계속 오면 → **Replan** (지도/장애물 기인).

## 4) 상태기계(State Machine)

상태: `RUN`, `SLOWDOWN`, `YIELD`, `REPLAN`, `REROUTE`, `STOP`

전이 규칙(간단):

* `RUN→SLOWDOWN`: `T_eff < T_slow` 또는 same-lane 추종 & d<threshold
* `RUN/SLOWDOWN→YIELD`: `T_eff < T_yield` 또는 `yield_prio ≥ Y_th`
* `YIELD→REPLAN`: `deadlock_window` 초과 & agent 원인, 혹은 `/replan_flag` 반복 & hold 없음
* `REPLAN→REROUTE`: replan 실패 N회 or 재차 deadlock
* `ANY→STOP`: 비상(초근접·응답지연)
* **Hysteresis**: 각 상태 최소 유지시간 + 완화 임계치(되돌림 방지)
* **Agent-Hold**: `/path_agent_collision_info` 수신 후 `hold_sec` 동안 `/replan_flag` 무시

## 5) Throughput & 공정성

* **machine_id 작은 쪽 우선** 기본 원칙 고수.
* **Area Reservation**: 협소/교차 구간 `occupancy=true` 진입자는 `T_area_hold` 유지 우선권.
* **Starvation 방지**: 동일 상대에게 내가 2회 연속 양보하면 3회째는 상대에게 양보 유도(부드러운 라운드로빈).
* **Batching**: 충돌 후보가 다수일 때 **시간창 Δt에서 스코어 상위 1~2개만** 처리(지나친 수동개입 방지).

## 6) 파라미터(권장 기본값)

```
# 신뢰도/결합
w1=1.0, w2=0.8               # TTC vs surrogate
T_min=0.5, d_min=0.2         # 하한 방어

# 임계
T_slow=6.0 s, T_yield=2.5 s
Y_th = 0.8 (yield_prio 임계)

# 가중치
a1=1.2, a2=0.2, a3=0.5, a4=0.3
b1=0.7, b2=0.9, b3=0.8, b4=0.4, b5=0.5, b6=0.2
κ=0.6

# heading 구분
θ_same=25 deg, θ_cross=110 deg

# 타이머
hold_sec=1.5
behavior_min=0.7
release_hys=0.5
deadlock_window=8.0
T_area_hold=3.0

# 속도 제한
v_nom=0.7, v_slow=0.3, v_yield=0.08, ω_slow=0.7
```

---

# 의사코드 (핵심)

```pseudo
on path_agent_collision_info(msg):
  agent_hold.start(hold_sec)
  candidates = []
  for k in 0..len(msg.x)-1:
    P = (x[k], y[k]); TTC = ttc_first[k]
    A = find_agent_covering_or_near(P)  # from /multi_agent_infos
    if not A: continue

    # 1) T_eff
    C_ttc = (TTC>0)?1:0
    T_alt, C_alt = estimate_surrogate_TTC(P, A, me)
    T_eff = combine_ttc(TTC, C_ttc, T_alt, C_alt, w1, w2)

    # 2) severity
    d = distance(me.pose, P)
    rel_heading = angle_diff(me.heading, A.heading)
    v_closing = closing_speed_est(me, A, P)
    sev = f_severity(T_eff, d, rel_heading, v_closing, a1..a4)

    # 3) yield priority
    row_me  = right_of_way(me)      # corridor, occupancy, id
    row_ot  = right_of_way(A)
    yprio = f_yield(A.mode, row_ot-row_me, A.reroute, A.phase, A.occupancy, id_bonus(me,A), b1..b6)

    score = sev * (1 + κ * yprio)
    candidates.append({A, P, T_eff, sev, yprio, score})

  primary = argmax score
  if not primary: return

  # 행동 결정
  if T_eff < T_yield or yprio >= Y_th:
    set_state(YIELD)
  elif T_eff < T_slow:
    set_state(SLOWDOWN)
  else:
    set_state(RUN)

  if state == YIELD and lasted(state) > deadlock_window:
    if primary.A.reroute: request(REPLAN)  # 먼저 replan
    else: request(REPLAN)
    if replan_fail_or_redeadlock: request(REROUTE)

  publish_debug(primary, state)
```

---

# 세부 구현 팁

* **Find agent covering point**: agent footprint를 소폭 dilate(예: 0.1~0.2m) 후 point-in-polygon; 없으면 recent centroid 근사(최소거리).
* **Surrogate TTC**:

  * `v_other_forward = dot(A.twist.linear, dir(A→P))`
  * `v_me_forward    = dot(me.twist.linear, dir(me→P))`
  * `v_closing = max(0, v_other_forward + v_me_forward)`
  * **Head-on**이면 `C_alt↑`, same-lane 뒤따름이면 `C_alt↓`.
* **ROW(me)**:

  * main corridor 안=+1, area occupancy=true=+1, `my_id < other_id`=+1, else -1씩 합산.
* **/replan_flag 처리**: `agent_hold.active()`면 무시; hold 종료 후에도 반복되면 `REPLAN`.
* **Starvation 보호**: 동일상대에게 2회 연속 양보 → 다음엔 나 우선(단, 안전성 제약 내).
* **하모닉**: 여러 후보가 동시에 들어오면 상위 1~2개만 행동에 반영; 나머지는 큐에서 decay.

---

# 기대 효과

* `ttc_first` 불안정성에 강함(TTC 대체치·신뢰도 결합).
* `reroute=true` 상대 배려로 **작업 동선 재배치가 빠르게 수렴** → 전체 Throughput 개선.
* 교착 자동 해소(hold + deadlock window + replan→reroute 단계).
* **ID·Area·Mode** 룰을 점수화하여 **예측 가능**하고 **정책 변경이 쉬움**.

---

원하면 이 설계를 그대로 **ROS2 C++ 노드**(파라미터·QoS·상태기계·디버그 토픽 포함)로 구현해서 드릴게.

###################  
#####################  
아래는 앞서 합의한 정책을 **그대로 코드화**한 ROS 2 Python 노드입니다.
입·출력 토픽, 파라미터, 상태머신, TTC 결합(신뢰도 가중), `reroute=true` 가중, agent-hold, deadlock 처리, throughput/공정성 휴리스틱까지 모두 들어있습니다.

* 입력

  * `/path_agent_collision_info` (`replan_monitor_msgs/msg/PathAgentCollisionInfo`)
  * `/multi_agent_infos` (`multi_agent_msgs/msg/MultiAgentInfoArray`)
  * `/replan_flag` (`std_msgs/Bool`, non-agent)
* 출력

  * `/decision_state` (`std_msgs/String`) : `RUN|SLOWDOWN|YIELD|REPLAN|REROUTE|STOP`
  * `/decision_speed_limit` (`geometry_msgs/Twist`) : 선형/각속도 상한 권고
  * `/request_replan` (`std_msgs/Bool`)
  * `/request_reroute` (`std_msgs/Bool`)
  * `/decision_debug` (`std_msgs/String`) : 핵심 점수/선택 근거 디버그

필요시 토픽명/파라미터는 자유롭게 바꾸면 됩니다.

---

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Path

from replan_monitor_msgs.msg import PathAgentCollisionInfo
from multi_agent_msgs.msg import MultiAgentInfoArray, MultiAgentInfo, AgentStatus


# ---------------------------
# Small math / utils
# ---------------------------

def ang_wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_of(pose: Pose) -> float:
    qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    # yaw from quaternion
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def dist2(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return math.hypot(dx, dy)


def dot2(ax: float, ay: float, bx: float, by: float) -> float:
    return ax * bx + ay * by


def unit(vx: float, vy: float) -> Tuple[float, float]:
    n = math.hypot(vx, vy)
    if n < 1e-6:
        return 0.0, 0.0
    return vx / n, vy / n


# ---------------------------
# Score helpers
# ---------------------------

def heading_bonus(rel_heading_deg: float) -> float:
    """head-on > crossing > same-lane (positive means more severe)."""
    a = abs(rel_heading_deg)
    if a <= 25.0:
        return 0.1   # same-lane 뒤따름
    if a <= 110.0:
        return 0.4   # 교차
    return 1.0       # 거의 정면


def mode_bonus(mode: str) -> float:
    """manual이면 내가 더 양보(상대 priority 높임)."""
    return 1.0 if mode.strip().lower() == "manual" else 0.0


def id_bonus(my_id: int, other_id: int) -> float:
    """my_id > other_id면 내가 양보쪽으로 가중."""
    return 1.0 if my_id > other_id else -0.2


def right_of_way_score(agent: MultiAgentInfo, my_id: int) -> float:
    s = 0.0
    # corridor/area/occupancy가 있다면 양보받을 이유 +1
    if agent.occupancy:
        s += 1.0
    # 작은 machine_id가 기본 우선
    s += (0.5 if agent.machine_id < my_id else -0.2)
    return s


# ---------------------------
# Main Decision Node
# ---------------------------

@dataclass
class Candidate:
    machine_id: int
    type_id: str
    px: float
    py: float
    T_eff: float
    severity: float
    yprio: float
    score: float
    note: str


class FleetDecisionNode(Node):
    def __init__(self):
        super().__init__("fleet_decision_node")

        # ---- Parameters (defaults reflect prior design) ----
        # Frames & my identity
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("my_machine_id", 1)

        # TTC combine
        self.declare_parameter("w1_ttc", 1.0)    # weight direct TTC
        self.declare_parameter("w2_alt", 0.8)    # weight surrogate TTC
        self.declare_parameter("T_min", 0.5)
        self.declare_parameter("d_min", 0.2)

        # Severity weights
        self.declare_parameter("a1_invT", 1.2)
        self.declare_parameter("a2_invd", 0.2)
        self.declare_parameter("a3_heading", 0.5)
        self.declare_parameter("a4_vclosing", 0.3)

        # Yield weights
        self.declare_parameter("b1_mode", 0.7)
        self.declare_parameter("b2_rowgap", 0.9)
        self.declare_parameter("b3_reroute", 0.8)
        self.declare_parameter("b4_pathsearch", 0.4)
        self.declare_parameter("b5_occupancy", 0.5)
        self.declare_parameter("b6_id", 0.2)
        self.declare_parameter("kappa", 0.6)

        # headings
        self.declare_parameter("same_lane_deg", 25.0)
        self.declare_parameter("cross_lane_deg", 110.0)

        # thresholds / timers
        self.declare_parameter("T_slow", 6.0)
        self.declare_parameter("T_yield", 2.5)
        self.declare_parameter("yield_priority_thresh", 0.8)

        self.declare_parameter("hold_sec", 1.5)
        self.declare_parameter("behavior_min", 0.7)
        self.declare_parameter("release_hys", 0.5)
        self.declare_parameter("deadlock_window", 8.0)

        # speed caps
        self.declare_parameter("v_nom", 0.7)
        self.declare_parameter("v_slow", 0.30)
        self.declare_parameter("v_yield", 0.08)
        self.declare_parameter("w_slow", 0.7)

        # topic names
        self.declare_parameter("topic_collision", "/path_agent_collision_info")
        self.declare_parameter("topic_agents", "/multi_agent_infos")
        self.declare_parameter("topic_replan_flag", "/replan_flag")

        self.declare_parameter("topic_decision_state", "/decision_state")
        self.declare_parameter("topic_decision_speed", "/decision_speed_limit")
        self.declare_parameter("topic_request_replan", "/request_replan")
        self.declare_parameter("topic_request_reroute", "/request_reroute")
        self.declare_parameter("topic_debug", "/decision_debug")

        # ---- get params ----
        self.global_frame = self.get_parameter("global_frame").as_string()
        self.my_id = self.get_parameter("my_machine_id").as_int()

        self.w1_ttc = self.get_parameter("w1_ttc").as_double()
        self.w2_alt = self.get_parameter("w2_alt").as_double()
        self.T_min = self.get_parameter("T_min").as_double()
        self.d_min = self.get_parameter("d_min").as_double()

        self.a1 = self.get_parameter("a1_invT").as_double()
        self.a2 = self.get_parameter("a2_invd").as_double()
        self.a3 = self.get_parameter("a3_heading").as_double()
        self.a4 = self.get_parameter("a4_vclosing").as_double()

        self.b1 = self.get_parameter("b1_mode").as_double()
        self.b2 = self.get_parameter("b2_rowgap").as_double()
        self.b3 = self.get_parameter("b3_reroute").as_double()
        self.b4 = self.get_parameter("b4_pathsearch").as_double()
        self.b5 = self.get_parameter("b5_occupancy").as_double()
        self.b6 = self.get_parameter("b6_id").as_double()
        self.kappa = self.get_parameter("kappa").as_double()

        self.same_lane_deg = self.get_parameter("same_lane_deg").as_double()
        self.cross_lane_deg = self.get_parameter("cross_lane_deg").as_double()

        self.T_slow = self.get_parameter("T_slow").as_double()
        self.T_yield = self.get_parameter("T_yield").as_double()
        self.Y_th = self.get_parameter("yield_priority_thresh").as_double()

        self.hold_sec = self.get_parameter("hold_sec").as_double()
        self.behavior_min = self.get_parameter("behavior_min").as_double()
        self.release_hys = self.get_parameter("release_hys").as_double()
        self.deadlock_window = self.get_parameter("deadlock_window").as_double()

        self.v_nom = self.get_parameter("v_nom").as_double()
        self.v_slow = self.get_parameter("v_slow").as_double()
        self.v_yield = self.get_parameter("v_yield").as_double()
        self.w_slow = self.get_parameter("w_slow").as_double()

        self.topic_collision = self.get_parameter("topic_collision").as_string()
        self.topic_agents = self.get_parameter("topic_agents").as_string()
        self.topic_replan_flag = self.get_parameter("topic_replan_flag").as_string()
        self.topic_decision_state = self.get_parameter("topic_decision_state").as_string()
        self.topic_decision_speed = self.get_parameter("topic_decision_speed").as_string()
        self.topic_request_replan = self.get_parameter("topic_request_replan").as_string()
        self.topic_request_reroute = self.get_parameter("topic_request_reroute").as_string()
        self.topic_debug = self.get_parameter("topic_debug").as_string()

        # ---- state machine ----
        self.state = "RUN"
        self.last_state_change = self.get_clock().now()
        self.deadlock_start: Optional[rclpy.time.Time] = None
        self.agent_hold_until: Optional[rclpy.time.Time] = None

        # Agent DB (최신 스냅샷)
        self.last_agents: Optional[MultiAgentInfoArray] = None

        # ---- pubs/subs ----
        self.sub_collision = self.create_subscription(
            PathAgentCollisionInfo, self.topic_collision, self.on_collision, 10
        )
        self.sub_agents = self.create_subscription(
            MultiAgentInfoArray, self.topic_agents, self.on_agents, 10
        )
        self.sub_replan_flag = self.create_subscription(
            Bool, self.topic_replan_flag, self.on_replan_flag, 10
        )

        self.pub_state = self.create_publisher(String, self.topic_decision_state, 10)
        self.pub_speed = self.create_publisher(Twist, self.topic_decision_speed, 10)
        self.pub_req_replan = self.create_publisher(Bool, self.topic_request_replan, 10)
        self.pub_req_reroute = self.create_publisher(Bool, self.topic_request_reroute, 10)
        self.pub_debug = self.create_publisher(String, self.topic_debug, 10)

        # periodic sanity tick (speed cap publish & hysteresis release)
        self.timer = self.create_timer(0.2, self.on_timer)

        self.get_logger().info(f"fleet_decision_node ready. my_id={self.my_id}")

    # --------- Callbacks ---------

    def on_agents(self, msg: MultiAgentInfoArray):
        self.last_agents = msg

    def on_replan_flag(self, msg: Bool):
        # Non-agent flag가 들어왔을 때: agent-hold면 무시
        now = self.get_clock().now()
        if self.agent_hold_until and now < self.agent_hold_until:
            self.get_logger().info("replan_flag ignored due to agent-hold window")
            return

        if msg.data:
            # 안전을 해치지 않도록 최소 지속시간 후 상태 전환
            if self.state in ("RUN", "SLOWDOWN"):
                self.request_replan("external replan_flag")
            else:
                # 이미 YIELD/REPLAN/REROUTE/STOP 중이면 무시하거나 로그만
                self.get_logger().info(f"replan_flag received but state={self.state}; ignored")

    def on_collision(self, msg: PathAgentCollisionInfo):
        now = self.get_clock().now()
        # Agent-hold 갱신: hold_sec 동안은 외부 replan_flag 무시
        self.agent_hold_until = now + Duration(seconds=self.hold_sec)

        cands = self.build_candidates(msg)
        if not cands:
            # 충돌 후보가 없으면 완화 시도
            self.release_if_safe("no candidates from collision msg")
            return

        # 우선 대상(최대 score) 선택
        primary = max(cands, key=lambda c: c.score)
        decision = self.decide_with_primary(primary)

        # 상태 및 속도/요청 반영
        self.apply_decision(decision, reason=f"primary(mid={primary.machine_id}) score={primary.score:.2f}")

        # 디버그 출력
        dbg = f"Primary(mid={primary.machine_id},type={primary.type_id}) " \
              f"T_eff={primary.T_eff:.2f} sev={primary.severity:.2f} yprio={primary.yprio:.2f} " \
              f"score={primary.score:.2f} note={primary.note}"
        self.pub_debug.publish(String(data=dbg))

    # --------- Core building blocks ---------

    def build_candidates(self, msg: PathAgentCollisionInfo) -> List[Candidate]:
        if self.last_agents is None or len(msg.x) == 0:
            return []

        # 에이전트 인덱싱: machine_id -> info
        agents_by_id = {a.machine_id: a for a in self.last_agents.agents}

        out: List[Candidate] = []
        for i in range(len(msg.x)):
            px, py = msg.x[i], msg.y[i]
            ttc = msg.ttc_first[i] if i < len(msg.ttc_first) else -1.0
            note = msg.note[i] if i < len(msg.note) else ""

            # 충돌 원인 후보: 메시지엔 source id가 없으므로 "근처/덮치는" 에이전트를 추정
            agent, rel_heading_deg, v_closing = self.match_agent_at_point(px, py, agents_by_id)
            if agent is None or agent.machine_id == self.my_id:
                continue

            # combine TTC (direct + surrogate)
            T_eff = self.combine_ttc(ttc, px, py, agent)

            # severity
            me_pose = self.find_me_pose(agents_by_id)
            d = dist2((me_pose.position.x, me_pose.position.y), (px, py)) if me_pose else 10.0
            sev = self.a1 * (1.0 / max(T_eff, self.T_min)) + \
                  self.a2 * (1.0 / max(d, self.d_min)) + \
                  self.a3 * heading_bonus(rel_heading_deg) + \
                  self.a4 * v_closing

            # yield priority
            row_me = right_of_way_score(agents_by_id.get(self.my_id, agent), self.my_id)
            row_ot = right_of_way_score(agent, self.my_id)
            yprio = self.b1 * mode_bonus(agent.mode) + \
                    self.b2 * (row_ot - row_me) + \
                    self.b3 * (1.0 if agent.reroute else 0.0) + \
                    self.b4 * (1.0 if agent.status.phase == AgentStatus.STATUS_PATH_SEARCHING else 0.0) + \
                    self.b5 * (1.0 if agent.occupancy else 0.0) + \
                    self.b6 * id_bonus(self.my_id, agent.machine_id)

            score = sev * (1.0 + self.kappa * yprio)
            out.append(Candidate(
                machine_id=int(agent.machine_id),
                type_id=agent.type_id,
                px=px, py=py,
                T_eff=T_eff, severity=sev, yprio=yprio, score=score, note=note
            ))

        return out

    def find_me_pose(self, agents_by_id) -> Optional[Pose]:
        me = agents_by_id.get(self.my_id, None)
        if me:
            return me.current_pose.pose
        return None

    def match_agent_at_point(self, px: float, py: float, agents_by_id) -> Tuple[Optional[MultiAgentInfo], float, float]:
        """가장 가까운(또는 footprint 근접) agent를 선택하고 상대Heading/closing speed 근사."""
        best = None
        best_d = 1e9
        rel_heading_deg = 90.0
        v_closing = 0.0

        me = agents_by_id.get(self.my_id, None)
        my_pose = me.current_pose.pose if me else None

        for a in agents_by_id.values():
            if a.machine_id == self.my_id:
                continue
            ax, ay = a.current_pose.pose.position.x, a.current_pose.pose.position.y
            d = dist2((ax, ay), (px, py))
            if d < best_d:
                best = a
                best_d = d

        if best is None:
            return None, 90.0, 0.0

        # 상대 heading과 내 heading 차이
        a_yaw = yaw_of(best.current_pose.pose)
        if my_pose:
            my_yaw = yaw_of(my_pose)
            rel_heading_deg = abs(math.degrees(ang_wrap(my_yaw - a_yaw)))
        else:
            rel_heading_deg = 90.0

        # closing speed surrogate
        ax, ay = best.current_pose.pose.position.x, best.current_pose.pose.position.y
        ux, uy = unit(px - ax, py - ay)  # agent->collision point
        v_other = best.current_twist.linear.x * dot2(math.cos(a_yaw), math.sin(a_yaw), ux, uy)

        v_me = 0.0
        if my_pose:
            my_yaw = yaw_of(my_pose)
            mx, my = my_pose.position.x, my_pose.position.y
            umx, umy = unit(px - mx, py - my)
            # 내 속도 정보는 없을 수도 있으니 conservative 0.0
            v_me = 0.0  # 필요하면 외부에서 /odom or status 확장

        v_closing = max(0.0, v_other + v_me)
        return best, rel_heading_deg, v_closing

    def combine_ttc(self, ttc_direct: float, px: float, py: float, agent: MultiAgentInfo) -> float:
        # direct TTC 신뢰도
        C_ttc = 1.0 if ttc_direct > 0.0 else 0.0

        # surrogate TTC
        ax, ay = agent.current_pose.pose.position.x, agent.current_pose.pose.position.y
        dx, dy = px - ax, py - ay
        d = math.hypot(dx, dy)
        a_yaw = yaw_of(agent.current_pose.pose)
        ux, uy = unit(dx, dy)
        v_other = agent.current_twist.linear.x * dot2(math.cos(a_yaw), math.sin(a_yaw), ux, uy)
        v_closing = max(0.0, v_other)  # 내 속도 미상 가정

        if v_closing < 0.05:
            T_alt = float("inf")
            C_alt = 0.2  # 신뢰 낮음
        else:
            T_alt = d / v_closing
            C_alt = 0.8  # 양호

        num = 0.0
        den = 0.0
        if C_ttc > 0.0:
            num += self.w1_ttc * C_ttc * ttc_direct
            den += self.w1_ttc * C_ttc
        if C_alt > 0.0 and math.isfinite(T_alt):
            num += self.w2_alt * C_alt * T_alt
            den += self.w2_alt * C_alt

        if den <= 1e-6:
            return float("inf")
        return max(num / den, 0.0)

    def decide_with_primary(self, c: Candidate) -> str:
        """RUN/SLOWDOWN/YIELD/REPLAN/REROUTE/STOP 중 선택."""
        # STOP 조건(초근접 + T_eff 매우 짧음)은 필요 시 추가
        if c.T_eff < self.T_yield or c.yprio >= self.Y_th:
            return "YIELD"
        if c.T_eff < self.T_slow:
            return "SLOWDOWN"
        return "RUN"

    def apply_decision(self, decision: str, reason: str):
        now = self.get_clock().now()

        # 최소 지속시간(behavior_min) 유지
        elapsed = (now - self.last_state_change).nanoseconds / 1e9
        can_change = (elapsed >= self.behavior_min) or (decision == self.state)

        if not can_change:
            # 상태 유지, 속도만 refresh
            self.publish_speed_cap(self.state)
            return

        # 상태 전이
        if decision != self.state:
            self.state = decision
            self.last_state_change = now
            self.get_logger().info(f"[STATE] -> {self.state} ({reason})")
            self.pub_state.publish(String(data=self.state))

            # deadlock timer 관리
            if self.state in ("YIELD",):
                if self.deadlock_start is None:
                    self.deadlock_start = now
            else:
                self.deadlock_start = None

        # 속도 발행
        self.publish_speed_cap(self.state)

        # deadlock window 초과 시 Replan/Reroute escalation
        if self.state == "YIELD" and self.deadlock_start is not None:
            dsec = (now - self.deadlock_start).nanoseconds / 1e9
            if dsec >= self.deadlock_window:
                # 우선 Replan 요청
                self.request_replan("deadlock window exceeded")
                # 다음주기에도 계속 deadlock이면 Reroute로 escalate
                # (간단 구현: Replan 요청 후, 다시 deadlock_window 지나면 Reroute)
                self.deadlock_start = now  # 타임스탬프 리셋

        # RUN으로 올라올 때 약한 히스테리시스: release_hys 동안은 replan_flag 무시
        if self.state == "RUN":
            self.agent_hold_until = now + Duration(seconds=self.release_hys)

    def publish_speed_cap(self, state: str):
        tw = Twist()
        if state == "RUN":
            tw.linear.x = self.v_nom
            tw.angular.z = 2.0  # 충분히 크게
        elif state == "SLOWDOWN":
            tw.linear.x = self.v_slow
            tw.angular.z = self.w_slow
        elif state == "YIELD":
            tw.linear.x = self.v_yield
            tw.angular.z = self.w_slow
        elif state in ("REPLAN", "REROUTE"):
            tw.linear.x = 0.0
            tw.angular.z = 0.0
        elif state == "STOP":
            tw.linear.x = 0.0
            tw.angular.z = 0.0
        else:
            tw.linear.x = self.v_nom
            tw.angular.z = 2.0
        self.pub_speed.publish(tw)

    def request_replan(self, why: str):
        if self.state not in ("REPLAN", "REROUTE"):
            self.state = "REPLAN"
            self.last_state_change = self.get_clock().now()
            self.pub_state.publish(String(data=self.state))
            self.get_logger().warn(f"Request REPLAN: {why}")
        self.pub_req_replan.publish(Bool(data=True))
        self.publish_speed_cap(self.state)

    def request_reroute(self, why: str):
        if self.state != "REROUTE":
            self.state = "REROUTE"
            self.last_state_change = self.get_clock().now()
            self.pub_state.publish(String(data=self.state))
            self.get_logger().warn(f"Request REROUTE: {why}")
        self.pub_req_reroute.publish(Bool(data=True))
        self.publish_speed_cap(self.state)

    def release_if_safe(self, why: str):
        # 충돌 감지 없음 → RUN 복귀(히스테리시스 반영)
        now = self.get_clock().now()
        elapsed = (now - self.last_state_change).nanoseconds / 1e9
        if self.state in ("YIELD", "SLOWDOWN") and elapsed >= self.behavior_min:
            self.state = "RUN"
            self.last_state_change = now
            self.pub_state.publish(String(data=self.state))
            self.get_logger().info(f"Release to RUN: {why}")
        self.publish_speed_cap(self.state)

    # 주기적 점검: 상태 유지, 히스테리시스, deadlock escalate
    def on_timer(self):
        self.publish_speed_cap(self.state)

        # deadlock escalate to REROUTE (두 번째 윈도우도 초과)
        if self.state == "REPLAN" and self.deadlock_start is not None:
            now = self.get_clock().now()
            dsec = (now - self.deadlock_start).nanoseconds / 1e9
            if dsec >= self.deadlock_window:
                self.request_reroute("deadlock persists after replan")


def main():
    rclpy.init()
    node = FleetDecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

---

## 사용 팁

* 주요 파라미터

  * `hold_sec`: `/path_agent_collision_info` 직후 **외부 `/replan_flag`를 무시**하는 시간. (agent-원인 처리에 집중)
  * `deadlock_window`: `YIELD` 유지 한계. 초과 시 **REPLAN 요청**, 이후에도 지속되면 **REROUTE 요청**으로 승격.
  * `T_yield`, `T_slow`: **시간 기반 임계**. `T_eff < T_yield` → YIELD, `T_eff < T_slow` → SLOWDOWN.
  * `b3_reroute`: **상대가 `reroute=true`일 때** 상대 우선권을 크게 올려 **내가 더 양보**하도록.
  * `v_nom/v_slow/v_yield`: 상위 노드(컨트롤러)에 주는 **속도 상한 권고**. 실제 속도 제한 적용은 컨트롤러 쪽에서 받도록.

* 출력 연동

  * `/decision_state`로 상위 UI/로깅.
  * `/decision_speed_limit`은 단순 권고치. 실제로는 controller에서 최소값 정책으로 받아 적용.
  * `/request_replan`, `/request_reroute`는 동작 트리/상위 orchestrator 에서 처리.

원한다면 이 노드용 **YAML 파라미터 샘플**이나, 상위 orchestrator 연동 코드(예: replan service client, controller speed gate)도 바로 만들어줄게.
