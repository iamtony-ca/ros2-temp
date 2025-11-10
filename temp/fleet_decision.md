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
