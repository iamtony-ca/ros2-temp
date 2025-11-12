아래 문서는 `fleet_decision_node.py`의 **모든 함수/메서드**를 코드에 기반해 자세히 설명한 software document입니다. 설명은 한국어로 하되 **기술 용어는 영어 그대로** 사용합니다. 마지막에 **상태 머신 개요**, **파라미터/토픽 표**, **튜닝 팁/안전 장치**도 덧붙였습니다.

---

# 개요

`FleetDecisionNode`는 `/path_agent_collision_info` 이벤트(다른 에이전트와의 충돌 가능성)와 `/multi_agent_infos`(에이전트들의 최신 상태)를 **event-driven** 방식으로 결합해 **RUN / SLOWDOWN / YIELD / STOP / REPLAN / REROUTE** 중 하나의 **의사결정 상태**를 내립니다.
핵심 절차는 다음과 같습니다.

1. 에이전트 충돌 이벤트 수신 → 후보(`Candidate`) 생성/스코어링
2. 가장 위험한 후보(primary) 선정 → 규칙 기반 결정(`decide_with_primary`)
3. **hysteresis**(clean K 연속), **idle-resume**, **timeout-resume**로 **안전 복귀** 제어
4. 상태 전환 시 **one-shot command pulse** 및 **상태 토픽** 발행

---

# 유틸 함수 (module-level)

### `ang_wrap(a: float) -> float`

* **역할:** 라디안 각 `a`를 `[-π, π]` 범위로 wrap.
* **이유:** heading 차이를 안정적으로 비교하기 위해.

### `yaw_of(pose: Pose) -> float`

* **역할:** `geometry_msgs/Pose`의 quaternion에서 **yaw(라디안)** 추출.
* **세부:** 표준 `atan2` 기반 변환. roll/pitch 무시.

### `dist2(ax, ay, bx, by) -> float`

* **역할:** 2D 평면상의 유클리드 거리.

### `dot2(ax, ay, bx, by) -> float`

* **역할:** 2D 내적.

### `unit(vx, vy) -> (ux, uy)`

* **역할:** 2D 벡터 정규화. 노름이 매우 작으면 `(0,0)` 반환.

### `heading_bonus(rel_heading_deg: float) -> float`

* **역할:** 상대 heading 각도에 따른 **severity 가중치**.
* **규칙:** 같은 차선(~25°) < 교차(~110°) < 정면충돌(>110°).

### `mode_bonus(mode: str) -> float`

* **역할:** 상대 agent의 `mode`가 `"manual"`이면 추가 가중(1.0), 아니면 0.0.

### `id_bonus(my_id: int, other_id: int) -> float`

* **역할:** ID 기반 tie-breaker. 내 ID가 크면 +1.0, 작으면 -0.2.

### `right_of_way_score(agent: MultiAgentInfo, my_id: int) -> float`

* **역할:** YOLO한 **right-of-way** surrogate.
* **규칙:** `agent.occupancy`면 +1.0, 그 외에 `agent.machine_id < my_id`면 +0.5, 아니면 -0.2.
* **용도:** `yield priority(yprio)` 계산에 사용.

---

# 데이터 모델

### `@dataclass Candidate`

* **필드:** `machine_id, type_id, px, py, T_eff, severity, yprio, score, note`
* **의미:**

  * `px, py`: 충돌 보고 지점(월드)
  * `T_eff`: 결합된 TTC(직접+대체)
  * `severity`: 위험도(시간/거리/heading/v_closing 종합)
  * `yprio`: 양보 우선권(모드/ROW/ID/상태 등)
  * `score`: 최종 의사결정을 위한 ranking 값
  * `note`: 원본 이벤트 주석

---

# 클래스: `FleetDecisionNode(Node)`

## `__init__(self)`

* **역할:** 파라미터 선언/획득, 상태/캐시 초기화, Pub/Sub 생성, 주기 타이머 설정, 초기 상태 발행.
* **주요 파라미터(샘플):**

  * TTC 결합: `w1_ttc, w2_alt, T_min, d_min`
  * Severity 가중: `a1_invT, a2_invd, a3_heading, a4_vclosing`
  * Yield 가중: `b1_mode, b2_rowgap, b3_reroute, b4_pathsearch, b5_occupancy, b6_id, kappa`
  * 진입 임계: `T_slow, T_yield, yield_priority_thresh(Y_th)`
  * 복귀 임계: `T_resume_*`, `Y_exit`, `K_*_clean`
  * resume 제어: `resume_idle_sec`, `resume_timeout_*`
  * 디바운스/무시: `agent_event_silence_sec`, `replan_ignore_sec_after_agent`, `run_pulse_silence_sec`
  * 토픽 이름들: collision/agents/replan_flag 입력, decision_state/request_* 출력, cmd_* 명령 펄스
* **런타임 상태:**

  * `state`: 현재 의사결정 상태(초기 `"RUN"`)
  * `last_state_change`: 상태 변경 시각
  * `last_agents`: 최신 `MultiAgentInfoArray`
  * `last_agent_event_time`: agent 단위 디바운스용 맵
  * `last_agent_event_any`: 가장 최근 agent 이벤트 시각
  * `slow_clean/yield_clean/stop_clean`: clean K 카운터
  * `last_run_cmd_time`: RUN 펄스 디바운스
* **Pub/Sub:**

  * Sub: `/multi_agent_infos`, `/path_agent_collision_info`, `/replan_flag`
  * Pub: `/decision_state`, `/request_replan`, `/request_reroute`, `/cmd/run|slowdown|yield|stop`, `/decision_debug`
* **Timer:** 0.2 s 주기 `on_timer()` (idle/timeout 기반 복귀 전용)
* **초기 동작:** `self._set_state_and_emit("RUN", "node start")`

---

## Subscribers

### `on_agents(self, msg: MultiAgentInfoArray)`

* **역할:** 최신 에이전트 상태 스냅샷 보관(`last_agents`).

### `on_replan_flag(self, msg: Bool)`

* **역할:** 외부에서 온 replan 트리거 처리.
* **로직:**

  1. `msg.data`가 False면 무시.
  2. 최근 agent 이벤트 직후 `replan_ignore_sec_after_agent` 내이면 **무시**(agent 이벤트에 우선권).
  3. 그렇지 않으면 상태를 `"REPLAN"`으로 전환하고 `/request_replan` **one-shot** publish.
* **의도:** agent 이벤트가 발생한 직후엔 외부 replan 플래그가 **불필요하게 상태를 흔들지 않도록** 디바운스.

### `on_collision(self, msg: PathAgentCollisionInfo)`

* **역할:** agent 충돌 이벤트의 메인 엔트리.
* **로직:**

  1. `last_agent_event_any` 갱신
  2. `build_candidates(msg)`로 후보 생성
  3. 후보 없음 → `_maybe_resume_on_clean(None)` 시도(현 상태가 완화되었는지 체크)
  4. 후보들 per-agent **디바운스**(`agent_event_silence_sec`) 후 비어지면 역시 clean 시도
  5. 남은 후보 중 **최대 `score`**를 primary로 선정
  6. primary agent의 reroute 상태 조회(`_agent_is_reroute`)
  7. **현재 상태**와 **primary**를 기반으로 `decide_with_primary` 실행 → 제안 상태 얻음
  8. 먼저 `_maybe_resume_on_clean(primary)`로 **복귀 히스테리시스** 우선 적용 (충족 시 바로 RUN)
  9. 아니면 `self._set_state_and_emit(decision, reason=...)`로 상태 전환

---

## Core building blocks

### `_agents_by_id(self) -> Dict[int, MultiAgentInfo]`

* **역할:** `last_agents`를 `{machine_id: info}` dict로 변환.
* **용도:** 빠른 agent 조회.

### `_agent_is_reroute(self, machine_id: int) -> bool`

* **역할:** 특정 agent가 `reroute` 플래그를 켰는지 확인.
* **의미:** 상대가 reroute 중이면 내가 크게 양보할 근거가 될 수 있음(`decide_with_primary` 규칙 3).

### `build_candidates(self, msg: PathAgentCollisionInfo) -> List[Candidate]`

* **역할:** 충돌 이벤트 메시지에서 **의사결정 후보**들을 생성.
* **입력:** `msg`(복수의 충돌 지점/메모/TTC)
* **절차:**

  1. `last_agents` 없거나 이벤트 좌표가 비었으면 빈 리스트
  2. `agents_by_id` 사전과 내 pose(`me_pose`) 준비
  3. 각 충돌 지점 i에 대해:

     * 좌표(px,py), direct TTC, note 추출
     * `match_agent_at_point(px,py,agents_by_id)`로 **가장 가까운 agent + rel_heading_deg + v_closing** 획득
     * 자기 자신이면 skip
     * `combine_ttc(direct, px,py, agent)`로 **T_eff** 산출(직접 TTC와 surrogate TTC 결합)
     * 내 위치와의 거리 `d_me` 계산(없으면 10.0 가정)
     * **severity** 계산:
       `a1*(1/max(T_eff,T_min)) + a2*(1/max(d_me,d_min)) + a3*heading_bonus + a4*v_closing`
     * **yield priority(yprio)**:
       상대/나의 `right_of_way_score` 차 + `mode_bonus` + `reroute` + `PATH_SEARCHING` + `occupancy` + `id_bonus`
     * **score**: `sev * (1 + kappa * yprio)`
     * `Candidate`로 누적
* **결과:** candidate 리스트(후속 디바운스/선정 과정 입력)

### `match_agent_at_point(self, px, py, agents_by_id) -> (Optional[MultiAgentInfo], float, float)`

* **역할:** 충돌 좌표에 **가장 가까운** agent를 매칭하고, 상대적 heading과 closing 속도 추정.
* **절차:**

  1. 모든 agent 중 자기 자신 제외하고 `(px,py)`와의 거리 최솟값 `best` 선택
  2. `best` 없으면 `(None, 90°, 0.0)`
  3. `a_yaw = yaw_of(best)`, 내 yaw(`my_yaw`)가 있으면 `rel_heading_deg = |my_yaw - a_yaw|` (deg)
  4. **closing 속도 추정:** `(px,py)`까지의 방향 단위벡터(ux,uy)와 agent 진행 방향 벡터의 dot → `v_other = v * dot`
     `v_closing = max(0, v_other)` (내 속도는 모른다고 가정)
* **의미:** heading/closing은 severity와 TTC 대체치 계산에 영향.

### `combine_ttc(self, ttc_direct, px, py, agent) -> float`

* **역할:** 제공된 **direct TTC**와 **surrogate TTC**를 가중 결합.
* **로직:**

  * `C_ttc = 1.0` if direct TTC>0 else 0
  * surrogate 계산:

    * `d = |(px,py) - agent.position|`
    * agent 진행 방향과 `(px,py)` 방향 단위벡터 dot → `v_other`
      `v_closing = max(0, v_other)`
    * `v_closing < 0.05`면 `T_alt=inf, C_alt=0.2`(신뢰도 매우 낮음), 그 외 `T_alt=d/v_closing, C_alt=0.8`
  * 가중 평균:
    `num = w1*C_ttc*ttc_direct + w2*C_alt*T_alt`, `den = w1*C_ttc + w2*C_alt`
    `den`이 매우 작으면 `inf`, 그 외 `max(num/den, 0)`
* **의도:** direct TTC가 있으면 우선 반영, 없거나 불신이면 surrogate로 보강.

### `decide_with_primary(self, c: Candidate, primary_is_reroute: bool) -> str`

* **역할:** primary candidate 기준으로 상태 제안.
* **규칙:**

  1. `T_eff < T_yield` **또는** `yprio >= Y_th` → `"YIELD"`
  2. 그 외 `T_eff < T_slow` → `"SLOWDOWN"`
  3. 상대가 reroute 중이고 `yprio`가 매우 크면 → `"REROUTE"`
  4. 그 외 → `"RUN"`
* **포인트:** 단순하지만 파라미터로 민감도 튜닝 가능.

---

## Resume & Publishing

### `_emit_state_pulse(self, state: str)`

* **역할:** 상태 문자열을 `/decision_state`로 publish하고, 상태에 해당하는 **one-shot command pulse**를 발행.
* **pulse 매핑:**

  * `RUN` → `/cmd/run` (디바운스: `run_pulse_silence_sec`)
  * `SLOWDOWN` → `/cmd/slowdown`
  * `YIELD` → `/cmd/yield`
  * `STOP` → `/cmd/stop`
  * `REPLAN` → `/request_replan`
  * `REROUTE` → `/request_reroute`

### `_set_state_and_emit(self, state: str, reason: str = "")`

* **역할:** 내부 상태 변경 + 로그 + `_emit_state_pulse` 호출.
* **부가:** `last_state_change` 갱신.

### `_maybe_resume_on_clean(self, primary: Optional[Candidate]) -> bool`

* **역할:** 현재 상태가 `SLOWDOWN/YIELD/STOP`일 때 **안전 복귀**를 판단.
* **규칙:**

  * **안전 조건(safe):**

    * `primary is None`(후보 없음) **또는**
    * 상태별 임계 통과:

      * `SLOWDOWN`: `primary.T_eff >= T_resume_slow` **AND** `primary.yprio < Y_exit`
      * `YIELD`:    `primary.T_eff >= T_resume_yield` **AND** `primary.yprio < Y_exit`
      * `STOP`:     `primary.T_eff >= T_resume_stop` **AND** `primary.yprio < Y_exit`
  * safe면 해당 상태의 **clean K 카운터** 증가(`K_*_clean` 도달 시 `"RUN"` 전환)
  * safe가 아니면 해당 상태의 clean 카운터 **리셋**
* **반환:** 복귀하여 상태를 바꾸었으면 `True`, 아니면 `False`.
* **의도:** **hysteresis**로 플리커 없이 안정적으로 복귀.

---

## Timeout / Idle resume

### `on_timer(self)`

* **역할:** **주기 타이머(0.2s)**로 **idle-resume** 및 **hard timeout-resume** 처리.
* **Idle-resume:**

  * 상태가 `SLOWDOWN` 또는 `YIELD`이고, `last_agent_event_any` 이후 `resume_idle_sec` 이상 **이벤트가 없으면** `"RUN"`.
* **Hard timeout-resume:**

  * 상태 지속 시간이 각 timeout(`resume_timeout_*`)을 초과하면 강제 `"RUN"`.
* **의도:** 이벤트 누락/사일런스 상황에서 **멈춰있지 않도록** 보장.

---

## 엔트리 포인트

### `main()`

* **역할:** rclpy init/spin/destroy 표준 루프.
* **의미:** 단일 노드로 실행될 것을 가정.

---

# 상태 머신 개요

* **상태:** `RUN`, `SLOWDOWN`, `YIELD`, `STOP`, `REPLAN`, `REROUTE`
* **전이:**

  * Collision 이벤트 → `build_candidates` → `primary` → `decide_with_primary`
  * `RUN → (SLOWDOWN|YIELD|REROUTE|REPLAN)` 등
  * **복귀:** `_maybe_resume_on_clean`(clean K), `on_timer`(idle/timeout)
* **출력:** 상태 문자열 + one-shot command pulse (상태에 매핑)

---

# 주요 파라미터 (요약)

* **TTC 결합:** `w1_ttc`, `w2_alt`, `T_min`, `d_min`
* **Severity:** `a1_invT`, `a2_invd`, `a3_heading`, `a4_vclosing`
* **Yield Priority:** `b1_mode`, `b2_rowgap`, `b3_reroute`, `b4_pathsearch`, `b5_occupancy`, `b6_id`, `kappa`
* **진입 임계:** `T_slow`, `T_yield`, `yield_priority_thresh(Y_th)`
* **복귀 임계:** `T_resume_slow`, `T_resume_yield`, `T_resume_stop`, `Y_exit`, `K_slow_clean`, `K_yield_clean`, `K_stop_clean`
* **resume:** `resume_idle_sec`, `resume_timeout_slow/yield/stop`
* **디바운스:** `agent_event_silence_sec`, `replan_ignore_sec_after_agent`, `run_pulse_silence_sec`

---

# 토픽 I/F (요약)

* **Sub:**

  * `/multi_agent_infos` (`MultiAgentInfoArray`)
  * `/path_agent_collision_info` (`PathAgentCollisionInfo`)
  * `/replan_flag` (`Bool`)
* **Pub:**

  * `/decision_state` (`String`) — 현재 상태 브로드캐스트
  * `/request_replan` (`Bool`) — REPLAN 요청 펄스
  * `/request_reroute` (`Bool`) — REROUTE 요청 펄스
  * `/cmd/run | /cmd/slowdown | /cmd/yield | /cmd/stop` (`Bool`) — 제어 명령 펄스
  * `/decision_debug` (`String`) — 디버그

---

# 튜닝 팁 & 안전 장치

* **민감도 조절:**

  * aggressive: `T_slow/T_yield` ↑, `a1/a2/a3/a4` ↑, `kappa` ↑
  * conservative: `T_resume_*` ↑, `K_*_clean` ↑, `Y_exit` ↓
* **Reroute 우선권:** `b3_reroute` ↑, `Y_th` ↓ → 상대가 reroute면 내가 크게 양보.
* **플리커 방지:** `K_*_clean`과 `run_pulse_silence_sec` 조합으로 안정화.
* **이벤트 폭주/중복 방지:** `agent_event_silence_sec`로 per-agent debounce.
* **Fail-safe:** `idle-resume`와 `timeout-resume`가 **영구 정지**를 방지.

---

필요하면 위 문서를 README 형태(표/도표 포함)로 재구성해 드릴게요. 또한 실제 로그 예시를 넣어 상태 전이 케이스 스터디도 추가할 수 있습니다.
####################  
######################  
####################  
좋은 질문들이에요. 코드 기준으로 핵심만 ‘정확히’ 풀어볼게요. (영문 용어는 그대로 유지)

# [path_validator_node]

## 0) “튜브(tube) shape”가 뭐고, 왜 그런 말을 쓰나?

* 의미: **상대 로봇의 truncated_path를 따라** 그 로봇의 **footprint(다각형)** 를 **얇게 확장(dilate)** 하며 **연속적으로 배치**했을 때 생기는 **연속적인 충돌 영역**을 말해요.

  * `whoCoversPoint()`/`pathTubeCoversPoint()`에서 path의 각 pose에 footprint를 소확장해서 월드로 변환하고, 그 **연속된 폴리곤 띠**가 마치 **관(tube)** 처럼 경로를 감싸므로 “tube”라고 부릅니다.
* 왜 필요?

  * 상대 로봇의 **현재 위치뿐 아니라 곧 지나갈 경로**까지 **선점 영역**으로 다루고, 우리 경로가 이 **관형 영역**과 교차하면 **미리** agent 충돌로 간주해 리플랜/양보 결정을 내리기 위함입니다.
* 이 개념은 `agent_layer`에서도 비슷하게 쓰여요. agent의 현재 pose와 truncated_path에 footprint를 찍어 **cost 254**로 칠해 **관 같은** 차단帯를 만들어 `agent_mask`를 구성하는 방식이죠.

---

# [fleet_decision_node]

## 1) `on_replan_flag()`에서 두 번 publish 되는가?

코드:

```py
self._set_state_and_emit("REPLAN", reason="external replan_flag")
self.pub_req_replan.publish(Bool(data=True))
```

* `_set_state_and_emit("REPLAN")` → 내부에서 `_emit_state_pulse()` 호출 → **`/request_replan`** 에 **한 번** publish.
* 그 다음 줄에서 **같은 `/request_replan`** 에 **한 번 더** publish.
* 즉, **`/request_replan`이 두 번** 나갑니다. (질문에 쓴 /replan_flag 토픽과는 **다른 토픽**이에요. 입력은 `/replan_flag`, 출력은 `/request_replan`.)
* 개선: 아래 둘 중 하나만 남기면 됩니다.

  1. `_set_state_and_emit("REPLAN")`만 호출하고 **직접 publish 삭제**
  2. 상태 전환 없이 **직접 publish만** 수행

---

## 2) `right_of_way_score`, `self.kappa`, direct TTC vs surrogate TTC

* **right_of_way_score(agent, my_id)**

  * **간단한 우선통행(ROW) 대용치**. `agent.occupancy`면 +1.0, ID가 나보다 작으면 +0.5, 아니면 -0.2를 더해 **상대가 양보받을 근거**를 만듭니다.
  * `yprio`(yield priority)에 들어가 **누가 더 양보해야 하는지**에 기여.
* **kappa**

  * 최종 점수: `score = severity * (1 + kappa * yprio)` 에서 **yprio를 얼마나 증폭**할지 조절하는 **gain**.
  * 크면 **양보 우선권의 영향이 커짐**(행동이 더 사회적/보수적), 작으면 **순수 위험도(severity) 위주**.
* **direct TTC vs surrogate TTC**

  * **direct TTC**: 상류 노드(예: path_validator)가 직접 계산해 `PathAgentCollisionInfo.ttc_first`로 준 **직접 추정 시간**. 신뢰도 높을 때가 많음.
  * **surrogate TTC**: direct가 없거나 신뢰 낮을 때, **상대 현재 pose→충돌점** 방향의 **closing speed**로 `거리/closing_speed`를 만든 **대체치**. 속도가 낮으면 `inf` 취급하고 신뢰도 낮게 가중.

---

## 3) `match_agent_at_point()`는 뭐고 왜 하나?

* **무엇:** 충돌 리포트 포인트 `(px,py)`와 **가장 가까운 agent**를 찾아 **상대 heading과 closing 속도**를 추정합니다.
* **왜:** `severity`와 `yprio` 계산에 필요한 **상대적 상황(정면/교차/동행, closing 정도)** 를 얻기 위함. direct TTC가 없을 때는 **surrogate TTC** 계산의 핵심 입력(거리, 진행방향 대비 성분 속도)이 됩니다.

---

## 4) 모든 파라미터 설명 + 튜닝 가이드

### (A) ID/프레임/내 정보

* `global_frame`: 메시지 일관 프레임 이름. 보통 `"map"`.
* `my_machine_id`: 내 agent ID. ROW/ID 우선 등 계산에 사용.

### (B) TTC 결합 관련

* `w1_ttc`: direct TTC 가중.
* `w2_alt`: surrogate TTC 가중.
* `T_min`: `1/max(T_eff, T_min)`에서 **폭주 방지** 하한(너무 작은 시간 역수 폭증 방지).
* `d_min`: 거리 항 역수의 하한.
  **튜닝:**
* direct TTC 품질이 좋으면 `w1_ttc`↑, 나쁘면 `w2_alt`↑.
* 오탐으로 과민하면 `T_min`↑, `d_min`↑.

### (C) Severity 가중

* `a1_invT`: **시간 역수(긴박도)** 가중. 급박할수록 ↑.
* `a2_invd`: **거리 역수** 가중. 가까울수록 ↑.
* `a3_heading`: **상대 heading** 가중(정면>교차>동행).
* `a4_vclosing`: **closing 속도** 가중. 다가오는 속도 빠를수록 ↑.
  **튜닝:**
* 보수적으로 느끼려면 `a1/a2/a3/a4`를 높여 **severity**를 키움.
* 동선 충돌(교차)에서 과민하면 `a3_heading`↓.
* 정지 대상에 과민하면 `a4_vclosing`↓.

### (D) Yield Priority (yprio)

* `b1_mode`: **manual 모드** 가중(수동 조작 우선).
* `b2_rowgap`: **ROW 차이(상대-나)** 가중.
* `b3_reroute`: 상대가 **reroute 중**이면 가점.
* `b4_pathsearch`: 상대가 **PATH_SEARCHING**이면 가점.
* `b5_occupancy`: 상대가 **occupancy**(점유)면 가점.
* `b6_id`: `id_bonus` 가중(간단한 우선순위 룰).
* `kappa`: `severity`에 yprio를 섞는 **증폭 계수**.
  **튜닝:**
* **사회적 양보**를 강화: `kappa`↑, `b2/b3/b4/b5`↑.
* **위험 위주**: `kappa`↓, `b*` 전반↓.

### (E) 진입 임계(상태 전환)

* `T_slow`: `T_eff < T_slow`면 최소 SLOWDOWN.
* `T_yield`: `T_eff < T_yield`이거나 `yprio >= Y_th`면 YIELD.
* `yield_priority_thresh`(= `Y_th`): **양보 우선권이 큰 상황** 판정 임계.
  **튜닝:**
* 자주 느려지면 `T_slow`↓. 너무 늦게 양보하면 `T_yield`↑ 또는 `Y_th`↓.

### (F) 복귀 Hysteresis (exit & K-연속)

* `T_resume_slow`, `T_resume_yield`, `T_resume_stop`: 각 상태에서 **안전 복귀**를 위한 `T_eff` 임계(들어갈 때보다 **더 널널**한 값 권장).
* `Y_exit`: 복귀 시 **yprio 상한**(양보 우선권 높으면 복귀 지연).
* `K_slow_clean`, `K_yield_clean`, `K_stop_clean`: **K-연속** clean 체크 횟수.
  **튜닝:**
* 플리커가 심하면 `K_*_clean`↑, 또는 `T_resume_*`↑, `Y_exit`↓.

### (G) Idle/Timeout resume

* `resume_idle_sec`: 최근 충돌 이벤트가 **없을 때** SLOWDOWN/YIELD 상태를 **자동 복귀**.
* `resume_timeout_slow/yield/stop`: 상태가 너무 오래 지속되면 **강제 복귀**.
  **튜닝:**
* 정체를 피하려면 `resume_idle_sec`↓, `resume_timeout_*`↓.
* 성급 복귀가 위험하면 반대로 ↑.

### (H) Debounce / Ignore

* `agent_event_silence_sec`: **같은 agent** 연속 이벤트 무시 시간.
* `replan_ignore_sec_after_agent`: agent 이벤트 직후 **외부 replan_flag** 무시 시간.
* `run_pulse_silence_sec`: `/cmd/run` 펄스 **디바운스**(빈번한 RUN 펄스 방지).
  **튜닝:**
* 이벤트가 너무 잦으면 `agent_event_silence_sec`↑.
* 외부 replan이 중요한 시스템이면 `replan_ignore_sec_after_agent`↓.

### (I) Topics

* 입력: `topic_collision`, `topic_agents`, `topic_replan_flag`
* 출력: `topic_decision_state`, `topic_request_replan`, `topic_request_reroute`, `topic_cmd_*`

---

## 5) “cost function” 형태의 계산은 뭔가, 왜 그렇게 하나?

### a) Severity (위험도 세기)

```
severity = a1*(1/max(T_eff, T_min))
         + a2*(1/max(d_me,  d_min))
         + a3*heading_bonus(rel_heading_deg)
         + a4*v_closing
```

* **해석:**

  * **시간이 짧다**, **가깝다**, **정면/교차다**, **다가오는 속도가 크다** → 위험도 ↑
* **왜:** 인간 직관과 유사한 **다요소 위험 평가**를 **선형 조합**으로 단순화.
* **장점:** 각 항목의 민감도를 `a*`로 **독립 튜닝** 가능, 해석 쉬움.

### b) Yield Priority (누가 더 양보?)

```
yprio = b1*mode_bonus(manual)
      + b2*(ROW(other) - ROW(me))
      + b3*1[r eroute]
      + b4*1[PATH_SEARCHING]
      + b5*1[occupancy]
      + b6*id_bonus
```

* **해석:** 상대가 수동/경로탐색/점유 중이거나 reroute이면 **양보받아야 할 이유**가 큼.
* **왜:** 교차로 암묵 규칙, 운용 정책을 **간단 신호**로 반영.

### c) 최종 Score

```
score = severity * (1 + kappa * yprio)
```

* **해석:** 위험이 큰 상황에서 **양보 우선권이 큰 상대**가 등장하면 **점수가 더 커져** 그 상대를 **우선 처리**.
* **왜:** 단순한 곱셈 구조로 **정책(사회성)**과 **물리적 위험**을 동시에 반영하면서, `kappa`로 비중을 한 번에 조절할 수 있음.

---

### 빠른 체크리스트 (현장 튜닝)

* **너무 자주 YIELD**: `T_yield`↓ 또는 `Y_th`↑, `kappa/b*`↓
* **정면 충돌에 둔감**: `a3_heading`↑, `a1_invT`↑
* **느리게 복귀**: `T_resume_*`↓, `K_*_clean`↓, `Y_exit`↑
* **외부 replan이 자주 씹힘**: `replan_ignore_sec_after_agent`↓

궁금한 파라미터 조합(예: 창고 좁은 회랑/4-way 교차)별 프리셋도 만들어 드릴 수 있어요.
###########################  
#############################  
############################  
좋아, 이 `fleet_decision_node.py`가 “상태(state)가 결정될 때까지” 실제로 어떤 순서와 기준으로 판단하는지를, **이벤트 흐름 기준**으로 정리해볼게. (용어는 영어 그대로, 설명은 한국어)

---

# 전체 흐름 개요

노드는 크게 **세 가지 입력 이벤트**로 움직여:

1. `/path_agent_collision_info` (agent 충돌 이벤트) → `on_collision`
2. `/replan_flag` (외부에서 리플랜 요청) → `on_replan_flag`
3. 주기 타이머(0.2s) → `on_timer` (resume/timeout)

그리고 상태 변경은 항상 `_set_state_and_emit(state, reason)`를 통해 일어나며, 그 내부에서 **상태 토픽과 1회성 명령 펄스**를 발행해.

---

# 1) Replan 플래그 이벤트 흐름 (`on_replan_flag`)

* 입력: `Bool(data=True)`가 오면 처리.
* 직전 agent 이벤트 이후 `replan_ignore_sec_after_agent`(기본 0.5s) 안이면 **무시** (agent 이벤트 여파로 플러딩되는 리플랜을 억제).
* 아니면 **즉시 `REPLAN` 상태로 전환**:
  `self._set_state_and_emit("REPLAN", reason="external replan_flag")`
  → `_emit_state_pulse`가 `topic_request_replan`에 `true` 펄스를 내보냄.
* 주의: 소스 코드에서 과거엔 `pub_req_replan.publish()`를 따로 한번 더 쐈지만, **현재 주석 처리**되어 중복 발행 안 함.

**결과:** 외부 신호 한 방에 상태는 `REPLAN`으로 바뀌고, replan 요청 펄스가 1회 발행됨.

---

# 2) Agent 충돌 이벤트 흐름 (`on_collision`)

이게 핵심. 상태 결정 대부분이 여기서 일어난다.

## 2-1) 충돌 후보 생성 (`build_candidates`)

* 메시지의 각 충돌 지점 `(px, py)`/ `ttc_first` / `note` / (가능하면) `machine_id`를 바탕으로 **Candidate**를 만든다.
* 에이전트 매칭:

  * 1순위는 **ID 매칭**(msg.machine_id[i] → `agents_by_id[mid]`).
    (일관성 체크에서 너무 멀면 폴백)
  * 실패 시 **폴백**으로 **nearest agent** 매칭(`match_agent_at_point`).
* 각 후보에 대해 다음을 계산:

  * `T_eff`: `combine_ttc()`로 direct TTC(`ttc_first`)와 surrogate TTC(거리/closing velocity 기반)를 **가중 평균**.
  * `severity`:

    ```
    severity = a1*(1/max(T_eff,T_min)) 
             + a2*(1/max(d_me,d_min))
             + a3*heading_bonus(rel_heading) 
             + a4*v_closing
    ```

    (가까울수록/정면에 가까울수록/closing 크면 위험↑)
  * `yprio`(yield priority):
    상대의 mode(수동/자동), right-of-way 차이(`right_of_way_score`), 상대가 `reroute`/`path_searching`인지, occupancy, id조건 등을 선형 결합.
  * 최종 `score = severity * (1 + kappa * yprio)`.

> 포인트: **severity**가 물리적 위험도, **yprio**가 “양보 필요성(사회적 규칙/작업상태)” 가중, `kappa`는 그 가중의 영향력.

## 2-2) Per-agent 디바운스

* 동일 `(machine_id, type_id)`에 대해 **최근 처리 시간**을 기억.
* `agent_event_silence_sec`(기본 1.0s) 안에 또 들어온 후보는 **무시**해서 채터링을 줄인다.

## 2-3) Primary 후보 선택

* 남은 후보 중 `score`가 **가장 큰** 것을 `primary`로 선택.
* 해당 agent의 최근 처리 시간을 갱신.

## 2-4) 상태 결정 1차 규칙 (`decide_with_primary`)

입력: `primary` 후보와, 그 agent가 현재 `reroute` 중인지 여부.

간단 룰:

1. `T_eff < T_yield` **또는** `yprio >= Y_th` → **`YIELD`**
2. 그 외 `T_eff < T_slow` → **`SLOWDOWN`**
3. 상대가 `reroute` 중이고 `yprio`도 높음 → **`REROUTE`**
4. 그 외 → **`RUN`**

(기본 파라미터: `T_yield=2.5`, `T_slow=6.0`, `Y_th=0.8`)

## 2-5) Resume 히스테리시스 검사 (`_maybe_resume_on_clean`)

* 현재 상태가 `SLOWDOWN`/`YIELD`/`STOP`일 때만 동작.
* **safe 판정**:

  * `primary`가 **없으면** 안전.
  * `primary`가 있으면, 상태별 **재개 임계**를 만족해야 안전:

    * SLOWDOWN: `T_eff >= T_resume_slow`(6.5) **and** `yprio < Y_exit`(0.5)
    * YIELD:    `T_eff >= T_resume_yield`(3.5) **and** `yprio < Y_exit`
    * STOP:     `T_eff >= T_resume_stop`(5.0) **and** `yprio < Y_exit`
* 안전이면 해당 상태별 **clean counter**(K-연속) 증가:

  * SLOWDOWN: `K_slow_clean`(2)회 연속 안전 → `RUN`
  * YIELD:    `K_yield_clean`(3)회 → `RUN`
  * STOP:     `K_stop_clean`(3)회 → `RUN`
* 안전 실패면 해당 상태의 카운터는 **리셋**.

> 포인트: **들어올 때 임계(T_slow/T_yield)**와 **나갈 때 임계(T_resume_*)**를 다르게 해서 **히스테리시스**로 상태 깜빡임을 방지.

## 2-6) 상태 적용

* 위 resume 검사에서 `RUN`으로 복귀했다면 종료.
* 아니면 **2-4의 결정 결과**(`RUN`/`SLOWDOWN`/`YIELD`/`REROUTE`)로
  `_set_state_and_emit(decision, reason=...)` 수행 → 실제 명령 펄스 발행.

---

# 3) 주기 타이머(`on_timer`)에 의한 자동 복귀

이벤트가 없을 때도 **과도한 정지/양보 지속**을 방지하는 보호장치:

* **Idle resume** (SLOWDOWN/YIELD일 때만):
  `last_agent_event_any` 이후 충돌 이벤트가 `resume_idle_sec`(1.5s) 동안 없으면 **`RUN` 복귀**.
* **Hard timeout resume** (SLOWDOWN/YIELD/STOP 공통):
  현재 상태로 머문 시간이 각각

  * SLOWDOWN: `resume_timeout_slow`(6s)
  * YIELD:    `resume_timeout_yield`(10s)
  * STOP:     `resume_timeout_stop`(15s)
    를 넘으면 무조건 **`RUN` 복귀**.

---

# 상태 전환 시 실제 출력 (`_emit_state_pulse`)

`_set_state_and_emit()` → 내부에서 `_emit_state_pulse(state)` 호출:

* 항상 `topic_decision_state`에 현재 state를 문자열로 publish.
* 그리고 **해당 state에 맞는 1회성 펄스**를 발행:

  * `RUN`  → `topic_cmd_run`

    * 단, `run_pulse_silence_sec`(0.5s) 내 과발행 **디바운스**.
  * `SLOWDOWN` → `topic_cmd_slowdown`
  * `YIELD` → `topic_cmd_yield`
  * `STOP` → `topic_cmd_stop`
  * `REPLAN` → `topic_request_replan`
  * `REROUTE` → `topic_request_reroute`

---

# 의사코드 형태로 한 번에 보기

```text
on_collision(msg):
  last_agent_event_any = now
  C_all = build_candidates(msg)                     # 물리 + 정책 스코어 만들기
  if C_all empty: return _maybe_resume_on_clean(None)

  C = debounce_per_agent(C_all, agent_event_silence_sec)
  if C empty: return _maybe_resume_on_clean(None)

  primary = argmax(C, by=score)
  remember_last_time(primary.agent)

  decision = decide_with_primary(primary, _agent_is_reroute(primary.agent))

  if _maybe_resume_on_clean(primary):
      return

  _set_state_and_emit(decision, reason=...)


on_replan_flag(true):
  if (now - last_agent_event_any) < replan_ignore_sec_after_agent:
      ignore
  else:
      _set_state_and_emit("REPLAN", reason="external replan_flag")


on_timer():
  if state in {SLOWDOWN, YIELD} and idle(no collision for resume_idle_sec):
      _set_state_and_emit("RUN", reason="idle resume")
      return

  if state == SLOWDOWN and dt_state >= resume_timeout_slow: RUN
  if state == YIELD   and dt_state >= resume_timeout_yield: RUN
  if state == STOP    and dt_state >= resume_timeout_stop:  RUN
```

---

# 실무 튜닝 포인트 (짧게 요약)

* **들어오는 임계**: `T_yield`, `T_slow`, `Y_th`
  (과민하면 빈번히 YIELD/SLOWDOWN으로 들어감)
* **나가는 임계**: `T_resume_*`, `Y_exit`, `K_*_clean`
  (너무 크면 잘 안 풀림, 너무 작으면 깜빡임)
* **스코어링**: `a1..a4`, `b1..b6`, `kappa`

  * 충돌 물리 위험은 `a*` 위주 (속도/거리/정면성)
  * 양보 우선순위는 `b*`/`kappa`로 영향력 조정
* **폴백 안정성**: `match_agent_at_point`는 예외 케이스용.
  `dist` 일관성 임계(10m)는 환경에 맞게 2~4m로 낮추는 걸 권장.
* **디바운스/보호**:
  `agent_event_silence_sec`, `run_pulse_silence_sec`, `replan_ignore_sec_after_agent`는 채터링 억제에 중요.

---

이렇게 보면, 이 노드는 **이벤트 드리븐 + 히스테리시스 + 타임아웃 보호**의 조합으로 상태를 고른다고 이해하면 딱 맞아. 원하는 구간(예: YIELD 진입을 더 보수적으로/더 공격적으로)만 골라 수치 조정하면 전체 행동을 꽤 예측 가능하게 다듬을 수 있어.
##################  
###################  
#####################  
좋아, `decide_with_primary()`를 “코드 그대로” 뜯어서 설명해볼게. 결론부터 말하면 **하나의 대표 충돌 후보(primary)**에 대해 **즉시 전이할 이벤트 상태**를 간단 규칙으로 고르는 함수야.

```python
def decide_with_primary(self, c: Candidate, primary_is_reroute: bool) -> str:
    # 1) 매우 위험(T_eff<T_yield or yprio>=Y_th) → YIELD
    # 2) 중간(T_eff<T_slow) → SLOWDOWN
    # 3) 상대가 reroute 상태고 yprio가 매우 크면 → REROUTE(내가 크게 양보)
    # 4) 그 외 → RUN
    if c.T_eff < self.T_yield or c.yprio >= self.Y_th:
        return "YIELD"
    if c.T_eff < self.T_slow:
        return "SLOWDOWN"
    if primary_is_reroute and c.yprio >= (self.Y_th - 0.2):
        return "REROUTE"
    return "RUN"
```

---

# 1) 입력과 리턴

* **입력**

  * `c: Candidate`
    충돌 후보 하나의 요약치. 주요 필드:

    * `c.T_eff`: **유효 TTC(Time-To-Collision)**. direct TTC와 surrogate TTC를 가중 결합해서 만든 “충돌까지 남은 시간” 추정치(초). **작을수록 위험**.
    * `c.yprio`: **yield priority 스칼라**. “상대에게 양보할 이유” 크기. **클수록 상대에게 양보해야 함**.
    * `c.score`: 상위 선택용 총점(여기 함수 내부에서는 사용하지 않음).
    * (그 외 `machine_id`, `px,py`, `severity`, `note` 등은 이 함수에서 직접 사용하지 않음.)
  * `primary_is_reroute: bool`
    이 primary agent가 **reroute 상태 플래그**를 들고 있는지(= 상대가 자체적으로 길을 바꾸려고 애쓰는 중인지) 표시.

* **리턴**: 전환할 이벤트 상태 문자열 중 하나

  * `"YIELD"`, `"SLOWDOWN"`, `"REROUTE"`, `"RUN"`

> 주의: 여기서는 `"STOP"`을 직접 반환하지 않는다. 완전 정지는 상위 정책/다른 로직에서만 일어남.

---

# 2) 사용되는 파라미터/변수 의미

* `self.T_yield`
  **YIELD 진입을 유발하는 시간 임계**. `c.T_eff < T_yield`면 **매우 급박**으로 보고 바로 YIELD.
* `self.Y_th` (= `yield_priority_thresh`)
  **양보 우선권(yprio) 임계**. `c.yprio >= Y_th`이면 시간 여건과 무관하게 **양보 필요**로 판정 → YIELD.
* `self.T_slow`
  **SLOWDOWN 진입 임계**. `T_eff`가 이 값보다 작지만 YIELD 조건(위 두 가지)에 해당하지 않으면 **속도만 낮추는 완화**로 결정.
* `(self.Y_th - 0.2)`
  REROUTE 조건에서 쓰는 **완화된 yprio 임계**. reroute 중인 상대에게는 **조금 더 쉽게 큰 양보(REROUTE)**를 선택하도록 **여유 0.2**를 준 것.
* `primary_is_reroute`
  상대가 reroute 중이면 “내가 크게 양보해서 먼저 가게 해 주는” REROUTE 결정을 열어 줌.

---

# 3) 의사결정 흐름(우선순위)

1. **YIELD 최우선**

   * 트리거 A: `c.T_eff < T_yield`
     → 충돌까지 남은 시간이 **아주 짧다** → 급정적/강한 양보 필요 → **"YIELD"**.
   * 트리거 B: `c.yprio >= Y_th`
     → 상대의 **양보 우선권**이 크다(수동/점유/경로탐색/ROW 차이 등) → **"YIELD"**.
   * 두 조건 중 하나라도 참이면 바로 반환하고 종료.

2. **SLOWDOWN 다음**

   * `c.T_eff < T_slow`
     → **위험은 있으나** YIELD급은 아님 → **"SLOWDOWN"**.
     (속도만 내려도 위험 완화가 가능하다고 보는 구간)

3. **REROUTE 특례**

   * `primary_is_reroute and c.yprio >= (Y_th - 0.2)`
     → 상대가 이미 reroute 중이고, **양보 우선권이 거의 YIELD 임계급**이면
     **내가 크게 양보(REROUTE)**해서 길을 열어주는 정책적 선택.

     * 시간(T_eff)이 급박하지 않아도 **“사회성/운용 정책”**을 반영.

4. **그 외는 RUN**

   * 위험도와 양보 우선권이 모두 낮으면 **"RUN"**.

> **판정의 특징**
>
> * **시간 기반(YIELD/SLOWDOWN)**이 **유도 우선**,
> * 그 다음 **정책 기반(REROUTE)**,
> * 마지막으로 **RUN**.

---

# 4) 경계조건(<= vs <)과 미묘함

* 코드가 **모두 `<`** 비교를 사용.

  * 예: `T_eff == T_yield`인 경우 **YIELD로 가지 않고** 다음 조건으로 넘어간다.
  * **바싹 붙은 케이스에서의 민감도**는 이 부등식 선택에 크게 좌우된다.
  * 더 보수적으로 하려면 `<=`로 바꾸거나 **임계값에 소량의 버퍼**(±0.1~0.3)를 둔다.

---

# 5) 왜 이런 순서/규칙인가?

* **안전 최우선:** **아주 급박한 시간 조건** 또는 **사회적 양보가 분명한 상황**은 **무조건 YIELD**로 빨리 개입.
* **연속성 유지:** 덜 급박하면 **SLOWDOWN**으로 완화해 **불필요한 완전 양보/정체 방지**.
* **정책적 배려:** 상대가 **reroute 중**이면, **조금만 우선권이 커도** 내가 크게 양보해서 **교행 비효율을 줄임**.
* **기본 동작:** 위가 아니면 **RUN**으로 생산성 유지.

---

# 6) 튜닝 가이드(실전 팁)

* **YIELD가 과도**하다 →
  `T_yield ↓` 또는 `Y_th ↑`. (reroute 특례가 자주 뜨면 `0.2` 버퍼를 `0.1`로 줄이거나 없애기)
* **SLOWDOWN만 계속 이어져 답답** →
  `T_slow ↓`(덜 민감), 또는 상위 resume 히스테리시스(`T_resume_*`, `K_*_clean`)를 완화.
* **reroute 상대에게 과양보** →
  REROUTE 조건의 `(Y_th - 0.2)`에서 **0.2를 줄이거나 제거**.
* **정면/교차 급박 상황인데 늦게 반응** →
  upstream에서 `T_eff`가 너무 느슨할 수 있으니 **severity/TTC 가중(w1_ttc/w2_alt, a3_heading 등)**을 재조정.

---

# 7) 예시로 감 잡기

* **Case A:** `T_eff=1.8s`, `yprio=0.3`, `primary_is_reroute=False`

  * `1.8 < T_yield(2.5)` → **YIELD**
* **Case B:** `T_eff=3.0s`, `yprio=0.85`, `Y_th=0.8`

  * `yprio >= Y_th` → **YIELD**
* **Case C:** `T_eff=4.5s`, `yprio=0.5`, `T_slow=6.0`

  * `T_eff < T_slow` → **SLOWDOWN**
* **Case D:** `T_eff=7.0s`, `yprio=0.65`, `primary_is_reroute=True`, `Y_th=0.8`

  * `yprio >= 0.6 (=Y_th-0.2)` → **REROUTE**
* **Case E:** `T_eff=8.0s`, `yprio=0.2`, reroute 아님

  * 위 조건 없음 → **RUN**

---

# 8) 이 함수의 ‘역할 범위’

* **즉시 상태 전환 “제안”**만 담당.
* **복귀 조건(히스테리시스, idle/timeout resume)** 은 `on_timer()`와 `_maybe_resume_on_clean()`이 따로 관리.
* 실제 퍼블리시(펄스)는 상위에서 `_set_state_and_emit()`가 처리.

필요하면 `STOP`도 여기서 결정하도록 분기(예: `T_eff < T_stop`)를 추가할 수 있는데, 그러면 **resume 임계(`T_resume_stop`)와 clean K 값**도 같이 손봐야 해.
################  
###################  
###################  
좋은 질문이야, Seri. 핵심은 두 가지:

1. “점수(Score) 계산은 그대로” 두고도, 최종 행동은 표에 맞게 **충돌 쌍(나 vs 상대)** 단위로 **역할(role) 배정** + **동시행동 방지(arbitration)**가 되게 만드는 것
2. `current_twist`, `ttc_first`가 믿기 어렵다면 **Severity/TTC 의존도를 임시로 낮추거나 끌 수 있는 토글**과 합리적인 **Fallback**을 넣는 것

아래는 네 현재 코드에 **최소 침습(minimal invasive)** 으로 넣는 방법이야.

---

# 1) “동일 상황에서 서로 같은 행동 금지” — 점수는 유지, 최종행동은 표에 맞추기

## 아이디어

* 여전히 `build_candidates()`로 각 후보마다 `severity`, `yprio`, `score`를 만든다.
* 단, **최종 행동 결정 직전**에 “나와 Primary 상대(=score 최고) 사이”에 대해 **역할(role)을 정하는 매핑 함수**와 **경합 해결(arbitration)**을 추가한다.
* 역할은 표(policy)에 맞게 “Leader / Follower”, “Pass / Yield”, “Replan / Hold” 등으로 매핑하고, **둘이 같은 행동을 택하지 않도록** 한다.

## 변경 포인트(개념)

* `decide_with_primary()` 호출 직전에:

  1. **충돌 키(conflict key)** 생성: `(min(my_id, other_id), max(my_id, other_id))` + 충돌 지점 격자셀 등
  2. **역할 배정**: `role = assign_roles(me, other, yprio, right_of_way, mode, policy_table)`

     * 예: ROW 높은 쪽 = “Pass/Run”, 낮은 쪽 = “Yield/Slowdown”
     * 동순위면 `machine_id` 또는 `type_id` 사전 우선 규칙, 그 다음 **작은 랜덤 지터**(고정 seed=pair)로 깨기
  3. **행동 합의(arbitration)**: 아주 얇은 Pub/Sub 1개로 “내가 이 conflict에 대해 이 행동을 ‘Claim’하겠다” 알리고, 1~2틱(예: 100~200ms) 안에 상대 Claim을 관찰. 충돌 시 deterministic 룰(점수>yprio>id)로 한쪽만 확정, 다른 쪽은 대체 행동으로 다운그레이드.

## 코드 스케치

### A) 메시지/퍼블리셔(아주 얇게)

```python
# __init__ 안
from std_msgs.msg import String
self.pub_claim = self.create_publisher(String, "/decision_claims", 10)
self.sub_claim = self.create_subscription(String, "/decision_claims", self.on_claim, 20)

# 런타임 캐시
self.claims: Dict[str, Tuple[int, str, float, float, float]] = {}  
# conflict_id -> (machine_id, action, score, yprio, stamp_sec)
```

```python
def conflict_id_for(self, my_id: int, other_id: int, px: float, py: float) -> str:
    a, b = (my_id, other_id) if my_id < other_id else (other_id, my_id)
    # 거칠게 셀 단위로 스냅 (같은 지점 충돌을 동일 키로)
    gx = round(px, 1); gy = round(py, 1)
    return f"{a}-{b}@{gx:.1f},{gy:.1f}"
```

```python
def on_claim(self, msg: String):
    # 포맷: "conflict_id|mid|action|score|yprio|ts"
    try:
        cid, mid, act, sc, yp, ts = msg.data.split("|")
        self.claims[cid] = (int(mid), act, float(sc), float(yp), float(ts))
    except Exception:
        return
```

### B) 역할 배정 + 행동 매핑 (표 기반)

```python
def assign_roles_and_action(self, me_id: int, other_id: int, c: Candidate) -> str:
    """
    점수/ROW/우선권을 표(policy)에 맞게 최종 행동으로 매핑.
    - 예) ROW 높은 쪽 RUN/Pass, 낮은 쪽 YIELD/Slowdown
    - reroute 상대면 난 RUN 또는 SLOWDOWN (표에 맞게)
    """
    # 예시 규칙(네 표에 맞춰 바꿔도 됨)
    # 기준: yprio (상대가 더 우선) / reroute / right_of_way 비교
    other_more_priority = (c.yprio >= self.Y_th)  # 혹은 row 비교 등
    if other_more_priority:
        # 난 양보측
        if c.T_eff < self.T_yield: 
            return "YIELD"
        elif c.T_eff < self.T_slow:
            return "SLOWDOWN"
        else:
            return "SLOWDOWN"  # 표가 원하면 RUN 대신 SLOWDOWN 고정
    else:
        # 난 우선측
        if c.T_eff < self.T_yield:
            return "YIELD"  # 너무 위험하면 둘 다 멈출 수 있게
        return "RUN"
```

### C) 경합(동시행동) 방지

```python
def arbitrate(self, cid: str, my_action: str, my_score: float, my_yprio: float, other_id: int) -> str:
    # 1) 내 claim 송신
    ts = self.get_clock().now().nanoseconds * 1e-9
    line = f"{cid}|{self.my_id}|{my_action}|{my_score:.3f}|{my_yprio:.3f}|{ts:.3f}"
    self.pub_claim.publish(String(data=line))

    # 2) 짧은 대기 (예: 0.1s) 동안 상대 claim 관찰
    wait_until = ts + 0.12
    while (self.get_clock().now().nanoseconds * 1e-9) < wait_until:
        rclpy.spin_once(self, timeout_sec=0.02)

    tup = self.claims.get(cid, None)
    if tup and tup[0] != self.my_id:
        other_mid, other_act, other_sc, other_y, other_ts = tup
        # 충돌 행동이면 tie-break
        if other_act == my_action:
            # 우선: score, 다음 yprio, 다음 machine_id 작은 쪽 우선 등
            mine_better = (my_score > other_sc) or \
                          (abs(my_score - other_sc) < 1e-3 and my_yprio > other_y) or \
                          (abs(my_score - other_sc) < 1e-3 and abs(my_yprio - other_y) < 1e-3 and self.my_id < other_mid)
            if not mine_better:
                # 난 행동 다운그레이드
                if my_action == "REPLAN": return "YIELD"
                if my_action == "YIELD":  return "SLOWDOWN"
                if my_action == "SLOWDOWN": return "RUN"
        # 행동이 달라도 표에서 금지 조합이면(예: REPLAN/REPLAN) 한쪽 다운그레이드
        if my_action == "REPLAN" and other_act == "REPLAN":
            # 동일하면 한쪽만 REPLAN, 나머지는 YIELD
            if self.my_id < other_mid: return "REPLAN"
            else: return "YIELD"
    return my_action
```

### D) 흐름에 끼우는 위치

`on_collision()`에서 primary 뽑은 다음:

```python
primary = max(cands, key=lambda c: c.score)
cid = self.conflict_id_for(self.my_id, primary.machine_id, primary.px, primary.py)

# 1) 역할 기반 1차 행동
action0 = self.assign_roles_and_action(self.my_id, primary.machine_id, primary)

# 2) 경합 방지
action = self.arbitrate(cid, action0, primary.score, primary.yprio, primary.machine_id)

# 3) 상태 전환
self._set_state_and_emit(action, reason=f"cid={cid} score={primary.score:.2f} T={primary.T_eff:.2f} Y={primary.yprio:.2f}")
```

> 이렇게 하면 **동일 충돌에 대해 양쪽이 동시에 같은 행동을 고르더라도** 짧은 합의 단계에서 **한 쪽만 확정**되고 다른 한 쪽은 **표가 요구하는 보조 행동**으로 자동 다운그레이드돼. 표(policy) 룰은 `assign_roles_and_action()` 안에만 반영하면 되니 유지보수도 쉬워.

---

# 2) `current_twist`/`ttc_first`가 신뢰 안 될 때 — Severity/TTC 임시 제거(또는 축소)

현재 코드에서 이 값들이 쓰이는 곳:

* `combine_ttc()` : `T_eff` 계산 시 `ttc_first`(직접 TTC)와 **대체 TTC(surrogate)**에 `agent.current_twist.linear.x`가 필요.
* `build_candidates()` : `severity` 안에 `1/T_eff`, `1/d`, `heading_bonus`, `v_closing`이 포함.
* 최종 의사결정은 `decide_with_primary()`에서 `T_eff`와 `yprio`만 사용.

## 문제

* 기본/더미 값이면 `T_eff → inf`가 잘 나오고, 그럼 `T_yield/T_slow` 비교에서 거의 항상 **RUN**이 되어버림.
* `severity`를 0으로 빼도 `score=sev*(1+kappa*yprio)`가 0이 되어 **모두 같은 점수**로 엮일 수 있음.

## 안전한 임시 대응(토글 + Fallback)

### A) 파라미터 토글 추가

```python
# __init__
self.declare_parameter("use_severity", True)
self.declare_parameter("use_ttc_inputs", True)  # ttc_first, current_twist 사용 여부
self.use_severity = bool(self.get_parameter("use_severity").value)
self.use_ttc_inputs = bool(self.get_parameter("use_ttc_inputs").value)
```

### B) combine_ttc Fallback

```python
def combine_ttc(self, ttc_direct: float, px: float, py: float, agent: MultiAgentInfo) -> float:
    if not self.use_ttc_inputs:
        # TTC를 신뢰하지 않을 때: 거리 기반 완전 대체 (v_nom으로 가정)
        ax, ay = agent.current_pose.pose.position.x, agent.current_pose.pose.position.y
        d = math.hypot(px - ax, py - ay)
        v_nom = 0.3  # 임시 상수 (param으로 뺄 수 있음)
        return float("inf") if v_nom < 1e-3 else max(d / v_nom, 0.0)

    # 기존 로직 유지
    ...
```

### C) severity 제거(또는 축소)

```python
# build_candidates 내부
if self.use_severity:
    sev = self.a1 * (1.0 / max(T_eff, self.T_min)) + \
          self.a2 * (1.0 / max(d_me, self.d_min)) + \
          self.a3 * heading_bonus(rel_heading_deg) + \
          self.a4 * v_closing
else:
    # 완전 제거 대신 아주 작은 상수 부여(순위 동률 방지)
    sev = 1e-3
```

### D) score 대체식 (severity 배제 시)

```python
if self.use_severity:
    score = sev * (1.0 + self.kappa * yprio)
else:
    # yprio만으로 소팅: sev 영향 배제
    # 필요하면 d_me 역수 등 소량 가중치를 더해 tie-breaking
    score = (1.0 + self.kappa * yprio)
```

### E) decide_with_primary도 TTC 미사용 시 보정

`use_ttc_inputs=False`일 때는 `T_eff`가 거리/상수속도 기반이라 과소/과대평가가 가능하니, **스레시홀드도 살짝 상향**하거나 **거리 기반 추가 게이트**를 넣는 게 안정적이야.

```python
def decide_with_primary(self, c: Candidate, primary_is_reroute: bool) -> str:
    T_y = self.T_yield if self.use_ttc_inputs else (self.T_yield * 1.3)
    T_s = self.T_slow  if self.use_ttc_inputs else (self.T_slow  * 1.3)

    if c.T_eff < T_y or c.yprio >= self.Y_th:
        return "YIELD"
    if c.T_eff < T_s:
        return "SLOWDOWN"
    if primary_is_reroute and c.yprio >= (self.Y_th - 0.2):
        return "REROUTE"
    return "RUN"
```

> 이렇게 하면 `ttc_first`/`current_twist`가 정착될 때까지 **점수와 정책 프레임은 유지**하면서도, **거짓 데이터로 인한 과도한 RUN/REPLAN/STOP** 쏠림을 막을 수 있어. 나중에 센서/추정이 안정화되면 `use_severity=True`, `use_ttc_inputs=True`만 다시 켜면 원상복구.

---

## 요약 체크리스트

* [ ] `assign_roles_and_action()`에 네가 만든 **정책 표**를 그대로 옮겨적는다(입력: 나/상대/지표들 → 출력: RUN/SLOWDOWN/YIELD/REPLAN 등).
* [ ] `arbitrate()`로 **동일 conflict_id**에 대한 **행동 충돌**을 100~200ms 내에 **결정론적으로** 해결.
* [ ] 임시로 TTC/속도를 신뢰하지 않을 땐 `use_ttc_inputs=False`, `use_severity=False`로 두고,
  `combine_ttc()`는 거리/상수속도 Fallback, `score`는 `yprio` 중심으로만 정렬.
* [ ] 이 기간엔 `decide_with_primary()`의 `T_*` 문턱을 10~30% 상향해서 보수적으로 운용.

필요하면, 네 표(policy) 스냅샷을 주면 `assign_roles_and_action()` 안을 딱 맞춰서 구체 코드로 펼쳐줄게.
#########################  
##########################  
########################  
좋아 Seri, “질문만”이라고 했으니 지금 당장 바꿔야 할 최소 변경 포인트와 이유만 깔끔히 정리해줄게. 핵심은 두 가지:

* (1) **동시 중복 행동 방지(탈중앙 mutual exclusion)** 를, 점수 선택은 유지하면서 “최종 행동”만 표의 policy에 맞춰 강제하기
* (2) **ttc_first / current_twist가 더미일 때 임시 운용** (severity/TT C 영향 제거·완화)

---

# 1) 서로 독립적으로 결정하면서 “같은 행동을 동시에” 하지 않게 만드는 법

네 제약(상호 공유 없음)을 지키면서 할 수 있는 건 **관측 가능한 공통 정보**(= 이미 쓰고 있는 `/multi_agent_infos`의 `machine_id`, `status.phase`, `reroute`, `occupancy` 등)만으로 **결과 동기화**를 유도하는 거야. 구현은 “점수로 primary 후보를 고르는 건 그대로” 두고, **최종 행동을 내리기 직전에 ‘정책 게이트’** 를 한 번 더 통과시키는 방식으로 가면 돼.

### A. 충돌 집합에서의 “로컬 리더 선출” + 행동 게이트

* 입력: `on_collision()`의 `PathAgentCollisionInfo`에는 충돌 포인트별 `machine_id[]`가 있어. 여긴 “이번 이벤트에 얽힌 상대 집합”의 근사치로 쓸 수 있어.
* 규칙: **결정적으로 1대만 “무거운 행동(예: REPLAN/REROUTE/STOP)”** 을 하도록, **로컬 리더**를 **결정적 규칙**으로 뽑아. (예: `min(machine_id)` 또는 “양보권 낮은 쪽” 등)
* 게이트: 내가 리더가 아니면 **무거운 행동을 YIELD/SLOWDOWN으로 다운시프트**.

> 이렇게 하면 통신 없이도 두 로봇이 동시에 “둘 다 REPLAN” 하는 걸 크게 줄일 수 있어.

**코드 스케치 (변경점만):**

```python
# 1) 충돌 이벤트에서 '현재 충돌에 얽힌 상대 집합'을 뽑는다.
def _conflict_group_ids(self, msg: PathAgentCollisionInfo) -> List[int]:
    ids = []
    for i in range(len(msg.machine_id)):
        mid = int(msg.machine_id[i])
        if mid and mid != self.my_id:
            ids.append(mid)
    return sorted(set(ids))

# 2) 결정적 리더 선출 규칙 (예: 가장 작은 ID가 리더)
def _elect_leader(self, group_ids: List[int]) -> Optional[int]:
    if not group_ids:
        return None
    return min(group_ids)

# 3) 최종 행동 매핑 전 게이트
def _policy_gate(self, desired: str, primary: Candidate, group_ids: List[int]) -> str:
    leader = self._elect_leader(group_ids)

    # 관측 기반 상호배제: 리더만 heavy action 허용
    heavy = {"REPLAN", "REROUTE", "STOP"}
    if desired in heavy and leader is not None and leader != self.my_id:
        # 난 리더가 아니니까 heavy 금지 → 완화 동작으로 다운시프트
        # 표(policy)가 있다면 여기서 표에 맞춰 매핑
        return "YIELD" if desired in {"REPLAN", "REROUTE"} else "SLOWDOWN"

    # 보조: 상대가 이미 PATH_SEARCHING(=replan 중) / reroute 중이면
    # 난 한 단계 낮춘다(중복 방지).
    agents = self._agents_by_id()
    someone_busy = any(
        (a.machine_id in group_ids) and
        (a.status.phase == AgentStatus.STATUS_PATH_SEARCHING or a.reroute)
        for a in agents.values()
    )
    if someone_busy and desired in heavy:
        return "YIELD"

    return desired
```

`on_collision()` 끝부분만 살짝 바꿔:

```python
group_ids = self._conflict_group_ids(msg)
decision_raw = self.decide_with_primary(primary, primary_is_reroute=self._agent_is_reroute(primary.machine_id))
decision = self._policy_gate(decision_raw, primary, group_ids)

# resume 히스테리시스는 그대로
if self._maybe_resume_on_clean(primary):
    return

self._set_state_and_emit(decision,
    reason=f"agent(mid={primary.machine_id}) score={primary.score:.2f} T={primary.T_eff:.2f} Y={primary.yprio:.2f}")
```

### B. 정책 표(behavior table)를 반영하는 “최종 행동 매퍼”

점수(=primary 선택)는 그대로 두고, **최종 행동 결정만 표에 맞춰 매핑 함수**를 둬. 예시:

```python
def _map_policy(self, c: Candidate, primary_is_reroute: bool) -> str:
    # 예시 테이블:
    # - yprio >= Y_th 강한 양보 -> YIELD
    # - reroute 상대 + yprio 높음 -> REROUTE
    # - 그 외는 상향/하향 조건자에 따라 SLOWDOWN/RUN ...
    if c.yprio >= self.Y_th:
        return "YIELD"
    if primary_is_reroute and c.yprio >= (self.Y_th - 0.2):
        return "REROUTE"
    # TTC/거리 없이 yprio만 쓸 수도 있고(임시 운용 시), 기존과 혼합 가능
    if math.isfinite(c.T_eff) and c.T_eff < self.T_slow:
        return "SLOWDOWN"
    return "RUN"
```

그리고 `decide_with_primary()`를 얇은 래퍼로 바꿔도 좋아:

```python
def decide_with_primary(self, c: Candidate, primary_is_reroute: bool) -> str:
    return self._map_policy(c, primary_is_reroute)
```

> 이렇게 “선택(점수)”과 “행동결정(표)”을 분리하면, 표를 교체해도 스코어링은 계속 재사용 가능해.

### C. 추가로 권장하는 **백오프/디바운스**

* **Heavy action 지연 랜덤 지터**: REPLAN/REROUTE를 하려면 0~0.3s 랜덤 지연을 두고, 그 사이에 상대가 PATH_SEARCHING으로 들어가면 내 쪽은 YIELD로 다운시프트. (실전에서 같은 타임스탬프 간섭을 더 줄여줌)
* **Heavy action rate-limit**: 이미 코드에 `resume_timeout_*`/`cooldown`이 있어. Heavy에 별도 `heavy_cooldown_sec`를 둬서 한 번 heavy 했으면 잠깐은 다시 heavy 금지.

---

# 2) ttc_first / current_twist가 더미일 때의 임시 운용

질문 요지: “이 값들이 default라 신뢰 못 하니 **severity 영향 제거**하고 싶다.”
현 코드에서 이 값들은 주로 **`combine_ttc`(T_eff)**, **`v_closing`**, 그리고 **severity** 계산에 들어가. 하지만 **최종 행동은 `decide_with_primary()`에서 `T_eff`와 `yprio`로** 내려. 즉, **score만 바꿔도 행동에 `T_eff`가 여전히 끼어들 수 있어.**

그래서 선택지는 둘 중 하나야:

### A. **선택(정렬)만 yprio 기반으로** 바꾸기 (가장 간단/안전)

* **primary 선정 키**를 `score` 대신 **`yprio`**로 바꾸거나, `score = (eps + self.kappa*yprio)`로 대체.
* `decide_with_primary()`는 **T_eff를 보지 않게**(혹은 `T_eff=inf`로 간주) 변경 → 임시 정책에선 **표만으로** 결정.

**변경 예시:**

```python
# build_candidates()에서 임시로 severity 고정/무시
sev = 1.0  # 임시 상수 (또는 0.0)
score = (1e-3) + self.kappa * yprio  # yprio만 반영한 정렬 점수

# on_collision()에서 primary 선정도 score로 두되 사실상 yprio 순서
primary = max(cands, key=lambda c: c.score)

# decide_with_primary(): T_eff 조건 제거(임시)
def decide_with_primary(self, c: Candidate, primary_is_reroute: bool) -> str:
    # T_eff가 신뢰 불가하니 yprio 중심 정책 테이블만 사용
    return self._map_policy(c, primary_is_reroute)
```

> 장점: 더미 데이터가 결과를 왜곡하지 않음.
> 단점: TTC 정보가 복구되기 전까지는 시간적 위험도를 반영 못 함.

### B. **T_eff를 ‘무해화’** 하는 파라미터 세팅

* `combine_ttc()`에서 `w1_ttc=0`, `w2_alt=0`로 두면 `T_eff=inf`가 되도록 만들 수 있어.
* 또는 `T_min`/`d_min`을 크게 해서 severity 항이 거의 0이 되게 할 수도 있어.
* 이 경우 **기존 decide_with_primary**를 그대로 둬도 `T_eff` 조건( `< T_slow`, `< T_yield`)이 사실상 작동하지 않음 → yprio만으로 표가 동작.

> 임시 플래그를 한 번에 제어하고 싶다면 `use_severity_in_score(bool)` 같은 파라미터를 추가하고, `build_candidates()` 안에서 분기해도 좋아.

---

## 빠른 체크포인트

* **동일 Heavy 액션 방지**:

  * [x] 로컬 리더 선출(`_conflict_group_ids` + `_elect_leader`)
  * [x] Heavy gate(`_policy_gate`)로 다운시프트
  * [x] 상대 PATH_SEARCHING / reroute 관측 시 Heavy 억제
  * [ ] (선택) 랜덤 지터 + Heavy rate-limit

* **더미 TTC/속도 임시 운용**:

  * [x] score에서 severity 제거(상수) + `score ≈ yprio`
  * [x] decide에서 T_eff 무시(또는 `w1_ttc=w2_alt=0`로 무해화)
  * [x] 정책 표로만 행동 매핑

---

원하면, 네가 말한 “표(policy)”를 그대로 코드화한 `_map_policy()` 버전을 내가 만들어줄 수도 있어(조건문 or 룩업 테이블). 지금은 구조만 잡아놨고, 나중에 표를 넘겨주면 그 룰 그대로 넣어줄게.

##########  
#############  
###########  
좋지! 네 “의도(점수 기반 선별은 유지 + 최종 행동은 정책표로 매핑 + 중복 heavy action 방지 + TTC/속도 더미일 때 임시 운용)”를 그대로 반영한 **완전한 대체 코드**를 아래에 제공해.
변경 핵심:

* `_map_policy()`로 **정책표 매핑** 분리 (yprio 중심 / 옵션으로 TTC 사용)
* **관측 기반 상호배제**: `_conflict_group_ids()` + `_elect_leader()` + `_policy_gate()`
  → 같은 충돌 집합에서 **리더만** REPLAN/REROUTE/STOP 가능, 비리더는 자동 다운시프트
* **임시 운용 플래그**

  * `use_severity_in_score`: False면 severity 비활성(=score≈yprio)
  * `use_ttc_in_decide`: False면 TTC 미사용, 전적으로 표(yprio 기반)로 결정
* **Heavy action rate-limit**: `heavy_cooldown_sec` (중복 heavy 억제)

---

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple, Dict

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose
from multi_agent_msgs.msg import PathAgentCollisionInfo
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
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

def dist2(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(ax - bx, ay - by)

def dot2(ax: float, ay: float, bx: float, by: float) -> float:
    return ax * bx + ay * by

def unit(vx: float, vy: float):
    n = math.hypot(vx, vy)
    if n < 1e-6:
        return (0.0, 0.0)
    return (vx / n, vy / n)

def heading_bonus(rel_heading_deg: float) -> float:
    # head-on > crossing > same-lane (positive means more severe)
    a = abs(rel_heading_deg)
    if a <= 25.0:   # same lane
        return 0.1
    if a <= 110.0:  # crossing
        return 0.4
    return 1.0      # almost head-on

def mode_bonus(mode: str) -> float:
    return 1.0 if (mode or "").strip().lower() == "manual" else 0.0

def id_bonus(my_id: int, other_id: int) -> float:
    return 1.0 if my_id > other_id else -0.2

def right_of_way_score(agent: MultiAgentInfo, my_id: int) -> float:
    s = 0.0
    if agent.occupancy:
        s += 1.0
    s += (0.5 if agent.machine_id < my_id else -0.2)
    return s


# ---------------------------
# Candidate model
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


# ---------------------------
# Event-driven decision node (with policy mapping & mutual exclusion)
# ---------------------------

class FleetDecisionNode(Node):
    """
    - build_candidates(): 점수(기본: severity * (1 + kappa*yprio)) 계산. 옵션으로 severity 비활성.
    - decide_with_primary(): _map_policy() 호출 → 정책표(주로 yprio)로 최종 행동 결정.
    - _policy_gate(): 동일 충돌 그룹에서 리더만 heavy(REPLAN/REROUTE/STOP)를 허용.
                      상대가 이미 PATH_SEARCHING/reroute면 내 heavy를 다운시프트.
                      heavy_cooldown_sec로 반복 heavy 억제.
    """
    def __init__(self):
        super().__init__("fleet_decision_node_ev")

        # ---- Parameters ----
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

        # Yield weights (priority)
        self.declare_parameter("b1_mode", 0.7)
        self.declare_parameter("b2_rowgap", 0.9)
        self.declare_parameter("b3_reroute", 0.8)
        self.declare_parameter("b4_pathsearch", 0.4)
        self.declare_parameter("b5_occupancy", 0.5)
        self.declare_parameter("b6_id", 0.2)
        self.declare_parameter("kappa", 0.6)

        # thresholds for enter (TTC 기반)
        self.declare_parameter("T_slow", 6.0)
        self.declare_parameter("T_yield", 2.5)
        self.declare_parameter("yield_priority_thresh", 0.8)  # Y_th

        # resume hysteresis (exit thresholds & K-연속)
        self.declare_parameter("T_resume_slow", 6.5)
        self.declare_parameter("T_resume_yield", 3.5)
        self.declare_parameter("T_resume_stop", 5.0)
        self.declare_parameter("Y_exit", 0.5)
        self.declare_parameter("K_slow_clean", 2)
        self.declare_parameter("K_yield_clean", 3)
        self.declare_parameter("K_stop_clean", 3)

        # idle-based resume
        self.declare_parameter("resume_idle_sec", 1.5)

        # timeouts (hard cap)
        self.declare_parameter("resume_timeout_slow", 6.0)
        self.declare_parameter("resume_timeout_yield", 10.0)
        self.declare_parameter("resume_timeout_stop", 15.0)

        # debounce / ignore windows
        self.declare_parameter("agent_event_silence_sec", 1.0)
        self.declare_parameter("replan_ignore_sec_after_agent", 0.5)
        self.declare_parameter("run_pulse_silence_sec", 0.5)

        # topics (inputs)
        self.declare_parameter("topic_collision", "/path_agent_collision_info")
        self.declare_parameter("topic_agents", "/multi_agent_infos")
        self.declare_parameter("topic_replan_flag", "/replan_flag")

        # topics (outputs)
        self.declare_parameter("topic_decision_state", "/decision_state")
        self.declare_parameter("topic_request_replan", "/request_replan")
        self.declare_parameter("topic_request_reroute", "/request_reroute")

        # state command topics (pulse true)
        self.declare_parameter("topic_cmd_run", "/cmd/run")
        self.declare_parameter("topic_cmd_slowdown", "/cmd/slowdown")
        self.declare_parameter("topic_cmd_yield", "/cmd/yield")
        self.declare_parameter("topic_cmd_stop", "/cmd/stop")

        # ---- NEW: Policy / Mutual exclusion knobs ----
        self.declare_parameter("use_severity_in_score", False)         # 임시 운용: severity 제거
        self.declare_parameter("use_ttc_in_decide", False)             # 임시 운용: TTC 미사용
        self.declare_parameter("policy_yprio_slow_thresh", 0.4)        # TTC 미사용 시 SLOWDOWN 기준
        self.declare_parameter("heavy_cooldown_sec", 2.0)              # heavy action rate-limit

        # ---- get params ----
        self.global_frame = self.get_parameter("global_frame").value
        self.my_id = int(self.get_parameter("my_machine_id").value)

        self.w1_ttc = float(self.get_parameter("w1_ttc").value)
        self.w2_alt = float(self.get_parameter("w2_alt").value)
        self.T_min = float(self.get_parameter("T_min").value)
        self.d_min = float(self.get_parameter("d_min").value)

        self.a1 = float(self.get_parameter("a1_invT").value)
        self.a2 = float(self.get_parameter("a2_invd").value)
        self.a3 = float(self.get_parameter("a3_heading").value)
        self.a4 = float(self.get_parameter("a4_vclosing").value)

        self.b1 = float(self.get_parameter("b1_mode").value)
        self.b2 = float(self.get_parameter("b2_rowgap").value)
        self.b3 = float(self.get_parameter("b3_reroute").value)
        self.b4 = float(self.get_parameter("b4_pathsearch").value)
        self.b5 = float(self.get_parameter("b5_occupancy").value)
        self.b6 = float(self.get_parameter("b6_id").value)
        self.kappa = float(self.get_parameter("kappa").value)

        self.T_slow = float(self.get_parameter("T_slow").value)
        self.T_yield = float(self.get_parameter("T_yield").value)
        self.Y_th = float(self.get_parameter("yield_priority_thresh").value)

        self.T_resume_slow = float(self.get_parameter("T_resume_slow").value)
        self.T_resume_yield = float(self.get_parameter("T_resume_yield").value)
        self.T_resume_stop  = float(self.get_parameter("T_resume_stop").value)
        self.Y_exit = float(self.get_parameter("Y_exit").value)

        self.K_slow_clean  = int(self.get_parameter("K_slow_clean").value)
        self.K_yield_clean = int(self.get_parameter("K_yield_clean").value)
        self.K_stop_clean  = int(self.get_parameter("K_stop_clean").value)

        self.resume_idle_sec = float(self.get_parameter("resume_idle_sec").value)

        self.resume_timeout_slow  = float(self.get_parameter("resume_timeout_slow").value)
        self.resume_timeout_yield = float(self.get_parameter("resume_timeout_yield").value)
        self.resume_timeout_stop  = float(self.get_parameter("resume_timeout_stop").value)

        self.agent_event_silence_sec = float(self.get_parameter("agent_event_silence_sec").value)
        self.replan_ignore_sec_after_agent = float(self.get_parameter("replan_ignore_sec_after_agent").value)
        self.run_pulse_silence_sec = float(self.get_parameter("run_pulse_silence_sec").value)

        self.topic_collision = self.get_parameter("topic_collision").value
        self.topic_agents = self.get_parameter("topic_agents").value
        self.topic_replan_flag = self.get_parameter("topic_replan_flag").value

        self.topic_decision_state = self.get_parameter("topic_decision_state").value
        self.topic_request_replan = self.get_parameter("topic_request_replan").value
        self.topic_request_reroute = self.get_parameter("topic_request_reroute").value

        self.topic_cmd_run = self.get_parameter("topic_cmd_run").value
        self.topic_cmd_slowdown = self.get_parameter("topic_cmd_slowdown").value
        self.topic_cmd_yield = self.get_parameter("topic_cmd_yield").value
        self.topic_cmd_stop = self.get_parameter("topic_cmd_stop").value

        # NEW params
        self.use_severity_in_score = bool(self.get_parameter("use_severity_in_score").value)
        self.use_ttc_in_decide = bool(self.get_parameter("use_ttc_in_decide").value)
        self.policy_yprio_slow_thresh = float(self.get_parameter("policy_yprio_slow_thresh").value)
        self.heavy_cooldown_sec = float(self.get_parameter("heavy_cooldown_sec").value)

        # ---- runtime caches ----
        self.last_agents: Optional[MultiAgentInfoArray] = None
        self.last_agent_event_time: Dict[Tuple[int, str], Time] = {}
        self.last_agent_event_any: Optional[Time] = None

        # state & timers
        self.state: str = "RUN"
        self.last_state_change: Time = self.get_clock().now()
        self.last_run_cmd_time: Optional[Time] = None

        # resume clean streaks
        self.slow_clean = 0
        self.yield_clean = 0
        self.stop_clean = 0

        # heavy rate-limit
        self.last_heavy_time: Optional[Time] = None

        # ---- pubs/subs ----
        self.sub_agents = self.create_subscription(MultiAgentInfoArray, self.topic_agents, self.on_agents, 10)
        self.sub_collision = self.create_subscription(PathAgentCollisionInfo, self.topic_collision, self.on_collision, 20)
        self.sub_replan_flag = self.create_subscription(Bool, self.topic_replan_flag, self.on_replan_flag, 10)

        self.pub_state = self.create_publisher(String, self.topic_decision_state, 10)
        self.pub_req_replan = self.create_publisher(Bool, self.topic_request_replan, 10)
        self.pub_req_reroute = self.create_publisher(Bool, self.topic_request_reroute, 10)

        self.pub_cmd_run = self.create_publisher(Bool, self.topic_cmd_run, 10)
        self.pub_cmd_slow = self.create_publisher(Bool, self.topic_cmd_slowdown, 10)
        self.pub_cmd_yield = self.create_publisher(Bool, self.topic_cmd_yield, 10)
        self.pub_cmd_stop = self.create_publisher(Bool, self.topic_cmd_stop, 10)

        self.pub_debug = self.create_publisher(String, "/decision_debug", 10)

        # lightweight timer only for timeouts & idle-resume (no heavy logic)
        self.timer = self.create_timer(0.2, self.on_timer)

        self.get_logger().info(f"[event] fleet_decision_node ready. my_id={self.my_id}")

        # 최초 상태(기본 RUN) 공지 + 1회 명령 펄스
        self._set_state_and_emit("RUN", reason="node start")

    # ---------- Subscribers ----------

    def on_agents(self, msg: MultiAgentInfoArray):
        self.last_agents = msg

    def on_replan_flag(self, msg: Bool):
        if not msg.data:
            return
        now = self.get_clock().now()
        if self.last_agent_event_any is not None:
            dt = (now - self.last_agent_event_any).nanoseconds * 1e-9
            if dt < self.replan_ignore_sec_after_agent:
                self.get_logger().info(f"replan_flag ignored ({dt:.2f}s after agent event)")
                return
        self.get_logger().warn("external replan_flag -> request REPLAN")
        self._set_state_and_emit("REPLAN", reason="external replan_flag")
        # 퍼블리시는 _emit_state_pulse()가 책임짐

    def on_collision(self, msg: PathAgentCollisionInfo):
        now = self.get_clock().now()
        self.last_agent_event_any = now

        cands_all = self.build_candidates(msg)
        if not cands_all:
            self._maybe_resume_on_clean(None)
            return

        # per-agent 디바운스
        cands: List[Candidate] = []
        for c in cands_all:
            key = (c.machine_id, c.type_id)
            t_last = self.last_agent_event_time.get(key, None)
            if t_last is not None:
                dt = (now - t_last).nanoseconds * 1e-9
                if dt < self.agent_event_silence_sec:
                    continue
            cands.append(c)

        if not cands:
            self._maybe_resume_on_clean(None)
            return

        # primary 선택: 점수는 유지(임시 운용 시 score≈kappa*yprio)
        primary = max(cands, key=lambda c: c.score)
        self.last_agent_event_time[(primary.machine_id, primary.type_id)] = now

        # 정책표 기반 결정
        decision_raw = self.decide_with_primary(primary, primary_is_reroute=self._agent_is_reroute(primary.machine_id))

        # 동일 충돌 집합 내 상호배제 게이트 적용
        group_ids = self._conflict_group_ids(msg)
        decision = self._policy_gate(decision_raw, primary, group_ids)

        # resume 히스테리시스
        if self._maybe_resume_on_clean(primary):
            return

        self._set_state_and_emit(decision,
                                 reason=f"agent(mid={primary.machine_id}) score={primary.score:.2f} "
                                        f"T={primary.T_eff:.2f} Y={primary.yprio:.2f}")

    # ---------- Core building blocks ----------

    def _agents_by_id(self):
        if self.last_agents is None:
            return {}
        return {a.machine_id: a for a in self.last_agents.agents}

    def _agent_is_reroute(self, machine_id: int) -> bool:
        agents = self._agents_by_id()
        a = agents.get(machine_id, None)
        return bool(a and a.reroute)

    # === NEW: conflict group & gate ===

    def _conflict_group_ids(self, msg: PathAgentCollisionInfo) -> List[int]:
        ids = []
        n = len(msg.machine_id)
        for i in range(n):
            try:
                mid = int(msg.machine_id[i])
            except Exception:
                mid = 0
            if mid and mid != self.my_id:
                ids.append(mid)
        return sorted(set(ids))

    def _elect_leader(self, group_ids: List[int]) -> Optional[int]:
        return min(group_ids) if group_ids else None

    def _policy_gate(self, desired: str, primary: Candidate, group_ids: List[int]) -> str:
        heavy = {"REPLAN", "REROUTE", "STOP"}
        # 1) leader only heavy
        leader = self._elect_leader(group_ids)
        if desired in heavy and leader is not None and leader != self.my_id:
            # 표에 맞춘 완화: REPLAN/REROUTE → YIELD, STOP → SLOWDOWN
            return "YIELD" if desired in {"REPLAN", "REROUTE"} else "SLOWDOWN"

        # 2) 상대 busy면 내 heavy 억제
        agents = self._agents_by_id()
        someone_busy = any(
            (aid in group_ids) and
            (a.status.phase == AgentStatus.STATUS_PATH_SEARCHING or a.reroute)
            for aid, a in agents.items()
        )
        if desired in heavy and someone_busy:
            return "YIELD"

        # 3) heavy rate-limit
        if desired in heavy and self.last_heavy_time is not None:
            dt = (self.get_clock().now() - self.last_heavy_time).nanoseconds * 1e-9
            if dt < self.heavy_cooldown_sec:
                return "YIELD"

        return desired

    # === Candidate build ===

    def build_candidates(self, msg: PathAgentCollisionInfo) -> List[Candidate]:
        if self.last_agents is None or len(msg.x) == 0:
            return []

        agents_by_id = self._agents_by_id()
        me_pose: Optional[Pose] = None
        if self.my_id in agents_by_id:
            me_pose = agents_by_id[self.my_id].current_pose.pose

        out: List[Candidate] = []
        N = len(msg.x)
        for i in range(N):
            px, py = msg.x[i], msg.y[i]
            ttc = msg.ttc_first[i] if i < len(msg.ttc_first) else -1.0
            note = msg.note[i] if i < len(msg.note) else ""

            # --- 1) ID 매칭 우선 ---
            agent = None
            rel_heading_deg = 90.0
            v_closing = 0.0

            mid = 0
            if i < len(msg.machine_id):
                try:
                    mid = int(msg.machine_id[i])
                except Exception:
                    mid = 0
            if mid:
                a = agents_by_id.get(mid, None)
                if a is not None:
                    agent = a
                    a_yaw = yaw_of(a.current_pose.pose)
                    if me_pose is not None:
                        rel_heading_deg = abs(math.degrees(ang_wrap(yaw_of(me_pose) - a_yaw)))
                    ux, uy = unit(px - a.current_pose.pose.position.x, py - a.current_pose.pose.position.y)
                    v_other = a.current_twist.linear.x * dot2(math.cos(a_yaw), math.sin(a_yaw), ux, uy)
                    v_closing = max(0.0, v_other)
                    if dist2(a.current_pose.pose.position.x, a.current_pose.pose.position.y, px, py) > 10.0:
                        agent = None
                        rel_heading_deg, v_closing = 90.0, 0.0

            # --- 2) 폴백: 근접 매칭 ---
            if agent is None:
                agent, rel_heading_deg, v_closing = self.match_agent_at_point(px, py, agents_by_id)

            if agent is None or agent.machine_id == self.my_id:
                continue

            # TTC 결합 (여기서 계산하지만, decide 단계에서 미사용 가능)
            T_eff = self.combine_ttc(ttc, px, py, agent)

            d_me = dist2(me_pose.position.x, me_pose.position.y, px, py) if me_pose else 10.0

            if self.use_severity_in_score:
                sev = self.a1 * (1.0 / max(T_eff, self.T_min)) + \
                      self.a2 * (1.0 / max(d_me, self.d_min)) + \
                      self.a3 * heading_bonus(rel_heading_deg) + \
                      self.a4 * v_closing
            else:
                # 임시: severity 끔
                sev = 1.0

            row_me = right_of_way_score(agents_by_id.get(self.my_id, agent), self.my_id)
            row_ot = right_of_way_score(agent, self.my_id)
            yprio = self.b1 * mode_bonus(agent.mode) + \
                    self.b2 * (row_ot - row_me) + \
                    self.b3 * (1.0 if agent.reroute else 0.0) + \
                    self.b4 * (1.0 if agent.status.phase == AgentStatus.STATUS_PATH_SEARCHING else 0.0) + \
                    self.b5 * (1.0 if agent.occupancy else 0.0) + \
                    self.b6 * id_bonus(self.my_id, agent.machine_id)

            # 점수: 기본 형태 유지. (임시 운용 시 거의 ≈ kappa*yprio)
            score = sev * (1.0 + self.kappa * yprio)

            out.append(Candidate(
                machine_id=int(agent.machine_id),
                type_id=agent.type_id,
                px=px, py=py,
                T_eff=T_eff, severity=sev, yprio=yprio, score=score, note=note
            ))
        return out

    def match_agent_at_point(self, px: float, py: float, agents_by_id) -> Tuple[Optional[MultiAgentInfo], float, float]:
        best = None
        best_d = 1e9
        for a in agents_by_id.values():
            if a.machine_id == self.my_id:
                continue
            d = dist2(a.current_pose.pose.position.x, a.current_pose.pose.position.y, px, py)
            if d < best_d:
                best = a; best_d = d
        if best is None:
            return None, 90.0, 0.0

        a_yaw = yaw_of(best.current_pose.pose)

        me = agents_by_id.get(self.my_id, None)
        rel_heading_deg = 90.0
        if me is not None:
            my_yaw = yaw_of(me.current_pose.pose)
            rel_heading_deg = abs(math.degrees(ang_wrap(my_yaw - a_yaw)))

        ux, uy = unit(px - best.current_pose.pose.position.x, py - best.current_pose.pose.position.y)
        v_other = best.current_twist.linear.x * dot2(math.cos(a_yaw), math.sin(a_yaw), ux, uy)
        v_closing = max(0.0, v_other)  # 내 속도 미상 가정
        return best, rel_heading_deg, v_closing

    def combine_ttc(self, ttc_direct: float, px: float, py: float, agent: MultiAgentInfo) -> float:
        C_ttc = 1.0 if ttc_direct > 0.0 else 0.0

        ax, ay = agent.current_pose.pose.position.x, agent.current_pose.pose.position.y
        dx, dy = px - ax, py - ay
        d = math.hypot(dx, dy)
        a_yaw = yaw_of(agent.current_pose.pose)
        ux, uy = unit(dx, dy)
        v_other = agent.current_twist.linear.x * dot2(math.cos(a_yaw), math.sin(a_yaw), ux, uy)
        v_closing = max(0.0, v_other)

        if v_closing < 0.05:
            T_alt = float("inf"); C_alt = 0.2
        else:
            T_alt = d / v_closing; C_alt = 0.8

        num = 0.0; den = 0.0
        if C_ttc > 0.0:
            num += self.w1_ttc * C_ttc * max(ttc_direct, 0.0)
            den += self.w1_ttc * C_ttc
        if C_alt > 0.0 and math.isfinite(T_alt):
            num += self.w2_alt * C_alt * T_alt
            den += self.w2_alt * C_alt
        if den <= 1e-6:
            return float("inf")
        return max(num / den, 0.0)

    # ---------- Policy mapping ----------

    def _map_policy(self, c: Candidate, primary_is_reroute: bool) -> str:
        """
        정책표를 코드화:
        - use_ttc_in_decide=False (임시): yprio 기반
            if yprio >= Y_th         -> YIELD
            elif primary_is_reroute & yprio >= (Y_th - 0.2) -> REROUTE
            elif yprio >= Y_slow     -> SLOWDOWN
            else                      -> RUN
        - use_ttc_in_decide=True  : TTC + yprio 혼합
            if T_eff < T_yield or yprio >= Y_th -> YIELD
            elif T_eff < T_slow                 -> SLOWDOWN
            elif primary_is_reroute & yprio >= (Y_th - 0.2) -> REROUTE
            else RUN
        """
        if not self.use_ttc_in_decide:
            if c.yprio >= self.Y_th:
                return "YIELD"
            if primary_is_reroute and c.yprio >= (self.Y_th - 0.2):
                return "REROUTE"
            if c.yprio >= self.policy_yprio_slow_thresh:
                return "SLOWDOWN"
            return "RUN"

        # TTC 사용 버전
        if (math.isfinite(c.T_eff) and c.T_eff < self.T_yield) or (c.yprio >= self.Y_th):
            return "YIELD"
        if math.isfinite(c.T_eff) and c.T_eff < self.T_slow:
            return "SLOWDOWN"
        if primary_is_reroute and c.yprio >= (self.Y_th - 0.2):
            return "REROUTE"
        return "RUN"

    def decide_with_primary(self, c: Candidate, primary_is_reroute: bool) -> str:
        return self._map_policy(c, primary_is_reroute)

    # ---------- Resume & publishing ----------

    def _emit_state_pulse(self, state: str):
        self.pub_state.publish(String(data=state))
        if state == "RUN":
            now = self.get_clock().now()
            if self.last_run_cmd_time is not None:
                dt = (now - self.last_run_cmd_time).nanoseconds * 1e-9
                if dt < self.run_pulse_silence_sec:
                    return
            self.last_run_cmd_time = now
            self.pub_cmd_run.publish(Bool(data=True))
        elif state == "SLOWDOWN":
            self.pub_cmd_slow.publish(Bool(data=True))
        elif state == "YIELD":
            self.pub_cmd_yield.publish(Bool(data=True))
        elif state == "STOP":
            self.pub_cmd_stop.publish(Bool(data=True))
            self.last_heavy_time = self.get_clock().now()
        elif state == "REPLAN":
            self.pub_req_replan.publish(Bool(data=True))
            self.last_heavy_time = self.get_clock().now()
        elif state == "REROUTE":
            self.pub_req_reroute.publish(Bool(data=True))
            self.last_heavy_time = self.get_clock().now()

    def _set_state_and_emit(self, state: str, reason: str = ""):
        self.state = state
        self.last_state_change = self.get_clock().now()
        if reason:
            self.get_logger().info(f"[STATE] -> {state} ({reason})")
        else:
            self.get_logger().info(f"[STATE] -> {state}")
        self._emit_state_pulse(state)

    def _maybe_resume_on_clean(self, primary: Optional[Candidate]) -> bool:
        if self.state not in ("SLOWDOWN", "YIELD", "STOP"):
            self.slow_clean = self.yield_clean = self.stop_clean = 0
            return False

        safe = False
        if primary is None:
            safe = True
        else:
            if self.state == "SLOWDOWN":
                safe = (math.isfinite(primary.T_eff) and primary.T_eff >= self.T_resume_slow) and (primary.yprio < self.Y_exit) \
                       if self.use_ttc_in_decide else (primary.yprio < self.Y_exit)
            elif self.state == "YIELD":
                safe = (math.isfinite(primary.T_eff) and primary.T_eff >= self.T_resume_yield) and (primary.yprio < self.Y_exit) \
                       if self.use_ttc_in_decide else (primary.yprio < self.Y_exit)
            elif self.state == "STOP":
                safe = (math.isfinite(primary.T_eff) and primary.T_eff >= self.T_resume_stop) and (primary.yprio < self.Y_exit) \
                       if self.use_ttc_in_decide else (primary.yprio < self.Y_exit)

        if not safe:
            if self.state == "SLOWDOWN": self.slow_clean = 0
            if self.state == "YIELD":    self.yield_clean = 0
            if self.state == "STOP":     self.stop_clean = 0
            return False

        if self.state == "SLOWDOWN":
            self.slow_clean += 1
            if self.slow_clean >= self.K_slow_clean:
                self._set_state_and_emit("RUN", reason="resume clean(SLOWDOWN)")
                self.slow_clean = self.yield_clean = self.stop_clean = 0
                return True
        elif self.state == "YIELD":
            self.yield_clean += 1
            if self.yield_clean >= self.K_yield_clean:
                self._set_state_and_emit("RUN", reason="resume clean(YIELD)")
                self.slow_clean = self.yield_clean = self.stop_clean = 0
                return True
        elif self.state == "STOP":
            self.stop_clean += 1
            if self.stop_clean >= self.K_stop_clean:
                self._set_state_and_emit("RUN", reason="resume clean(STOP)")
                self.slow_clean = self.yield_clean = self.stop_clean = 0
                return True

        return False

    # ---------- Timeout / Idle resume timer ----------

    def on_timer(self):
        now = self.get_clock().now()

        if self.state in ("SLOWDOWN", "YIELD"):
            if self.last_agent_event_any is not None:
                dt_idle = (now - self.last_agent_event_any).nanoseconds * 1e-9
                if dt_idle >= self.resume_idle_sec:
                    self._set_state_and_emit("RUN", reason=f"idle resume ({dt_idle:.2f}s no collisions)")
                    return

        dt_state = (now - self.last_state_change).nanoseconds * 1e-9
        if self.state == "SLOWDOWN" and dt_state >= self.resume_timeout_slow:
            self._set_state_and_emit("RUN", reason=f"SLOWDOWN timeout {dt_state:.2f}s")
            return
        if self.state == "YIELD" and dt_state >= self.resume_timeout_yield:
            self._set_state_and_emit("RUN", reason=f"YIELD timeout {dt_state:.2f}s")
            return
        if self.state == "STOP" and dt_state >= self.resume_timeout_stop:
            self._set_state_and_emit("RUN", reason=f"STOP timeout {dt_state:.2f}s")
            return


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

### 빠른 튜닝 가이드 (이 버전 기준)

* **임시 운용(더미 데이터)**:

  * `use_severity_in_score: false`, `use_ttc_in_decide: false`
  * `policy_yprio_slow_thresh`: 0.3~0.5 사이에서 조정 (크면 덜 느려짐)
* **정책 표의 강도**:

  * `yield_priority_thresh(Y_th)`: 높일수록 YIELD는 덜 발생
  * `b*` 가중치: yprio 구성. 예) 수동(cart) 우선 양보를 세게 → `b1_mode`↑
* **중복 heavy 억제**:

  * `heavy_cooldown_sec`: 1.5~3.0s 추천
* **리더 규칙**: 현재 `min(machine_id)` 결정적. 현장 규약과 맞추려면 다른 규칙으로 변경 가능.

원하는 표가 더 구체적이라면, 그대로 조건을 `_map_policy()`에 옮겨 적어줄게.
