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
