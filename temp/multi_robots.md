좋은 포인트야. 결론부터 말하면:

* **PathValidator는 “가볍고 단정적인 판별기(guardian)”**로 유지:
  “경로 차단 감지” + “로봇 기인 여부 식별(인덱스 매칭 + 작은 커널 허용)” + “최소한의 요약 메타만 추출”
* **Conflict Decider가 “정책/판단의 무게 중심”**을 맡는 게 베스트:
  시나리오 분류, 우선권/토큰, 협소구간(area) 처리, is_reroute/is_manual 등 **상태 해석**과 **행동(Wait/Replan/Slowdown/Keepout)**

이렇게 나누면 “실시간성 + 유지보수성 + 디버깅”이 모두 좋아져. PathValidator는 항상 빠르게, 일관되게 동작하고, 변경이 많은 정책은 Decider 안에서만 바꾸면 돼.

---

# 최종 구조(역할 분리 원칙)

## 0) Area/협소구간 전제

* 네가 말한 area는 **cross-section/협소공간** 같은 “임계 구간(Critical Section)”으로 정의.
* RobotLayer/Decider 양쪽에서 **area_id 다각형**을 알고 있고, 점유/진입 여부 판단에 사용.

## 1) RobotLayer (코스트 생성 + “희소 메타” 제공)

* **하는 일**

  * 다른 로봇들의 swept-footprint/예측 궤적 → 코스트맵 래스터.
  * **희소 메타 토픽**(`robot_layer_meta`) 발행:
    “경로-결정에 의미 있는 셀/구간”만 담는 **인덱스 기반 sparse 데이터**.
* **메타의 예(셀 단위 또는 세그먼트 단위)**

  ```
  struct CellMeta {
    uint32 mx, my;          // 또는 world pose
    string amr_id;
    string phase;           // MOVING/WAITING/DOCKED/...
    bool is_manual;
    bool is_reroute;
    bool is_replanning;
    string area_id;         // 있으면 채움
    float t_first_hit;      // 이 셀을 상대가 점유하는 최초 예상 시각(초)
    float sigma;            // 불확실도
    rclcpp::Time stamp;     // 생성 기준
  }
  ```

  * 퍼포먼스: **sparse**만 내보내므로 `costmap_raw` 전체를 또 복제하지 않음.
* **QoS**

  * 메타: `KeepLast(1) + reliable + transient_local` (late-join 보호)
  * (필요 시) 고빈도 업데이트는 `volatile`도 고려

> 참고: 위에서 “robot_layer_raw”라고 하던 걸, **실제로는 메타 전용 토픽**으로 바꾸자고 제안.
> 경로 검사는 `global_costmap/costmap_raw`로 하고, “로봇 기인 여부 확인”은 이 **메타**에서 인덱스 매칭으로 판정.

## 2) PathValidator (얇고 빠르게)

* **핵심 원칙**: “**판정은 얇게, 이벤트는 풍부하게**”
* **하는 일**

  1. `/global_costmap/costmap_raw`에서 글로벌 경로 위 **차단 감지**(지속성/커널/쿨다운 이미 구현됨).
  2. 같은 인덱스(±kernel)로 `robot_layer_meta`를 **lookup**:

     * 매칭 있으면 ⇒ “**ROBOT_CONFLICT**”
     * 없으면 ⇒ “**ENV_BLOCKED**”
  3. **최소 요약 메타**만 뽑아서 이벤트 발행(Decider가 깊이 판단하도록).
* **발행**

  * 환경 기인: `/replan_flag` (기존 그대로)
  * 로봇 기인: `/path_robot_conflict_event` (이벤트) + `/path_robot_conflict_state` (상태; 선택)
* **이벤트 필드(최소 요약)**

  ```
  amr_ids                # 관여 로봇 id set (복수 가능)
  first_conflict_pose    # 경로상의 최초 충돌 지점
  ttc_first              # 가능하면 robot_layer_meta의 t_first_hit 사용
  is_manual_any
  is_reroute_any
  is_replanning_any
  shared_area_id         # 동일 area면 값
  meta_staleness_ms      # 메타 최신성
  ```
* **하지 않는 일**

  * **정책 판단/우선권/토큰/협소구간 판정** X
  * is_reroute/is_manual을 **판단의 트리거로 사용하지 않음**(단, 이벤트에 값은 담아줌)

> 이렇게 해야 PathValidator가 **RT-safe**하고, 정책 바뀌어도 수정 없이 그대로 쓸 수 있어.

## 3) Conflict Decider (정책의 대부분)

* **하는 일**

  * 이벤트를 받아 **시나리오 분류(Head-on/Crossing/Following/Blocking/Fault)**
  * **우선권/토큰**(manual/area_occupancy/transferring/reroute 가중치 반영)
  * **협소구간(area_id)** 판단 + 자원(critical section) 충돌 해결
  * **행동**: WAIT / REPLAN_SOFT / REPLAN_WITH_KEEP_OUT / SLOWDOWN
  * **마스크 생성**: keepout(상대 swept + dilation), speed-limit(로컬 ROI)
  * **타임아웃/공정성**: deadlock, yield history
* **추가 구독(선택)**

  * `MultiRobotStateArray`(Aggregator)로 세부 상태/속도 보강
  * area polygon 맵(정적 파라미터)
* **출력**

  * `/decision_cmd` (WAIT/GO/REPLAN_* 등)
  * `/planner/keepout_mask`, `/controller/speed_limit_mask`
  * `/conflict_decider_state`(상태)

---

# 왜 이렇게 나누나? (장점)

1. **실시간성**: PathValidator는 단순 인덱스 매칭 + 스트릭 판단만 하므로 μs~ms 급.
2. **정책 독립성**: Decider만 손대면 전략(협소구간 규칙/우선권/토큰) 전부 교체 가능.
3. **테스트 용이**:

   * RobotLayer 단독 테스트: 코스트/메타가 기대대로 나오나
   * PathValidator 단독 테스트: true/false 판정 재현 가능
   * Decider 단독 테스트: 이벤트 리플레이로 정책 튜닝
4. **네트워크/CPU 효율**: 로봇 레이어는 **희소 메타**만 내보내서 대역 절감.

---

# 데이터 플로우 (최종)

```
[Other Robots/Server] 
   └─(원자료)──▶ [Aggregator]
                   └─ MultiRobotStateArray(pub)

[RobotLayer (Costmap plugin)]
   ├─ MultiRobotStateArray(sub)
   ├─ 내부 코스트맵 write (global/local)
   └─ robot_layer_meta(pub; sparse)

[PathValidator]
   ├─ global_costmap/costmap_raw(sub)
   ├─ robot_layer_meta(sub)
   └─ 경로 차단 감지 → 
        - 로봇 기인: /path_robot_conflict_event(pub)
        - 환경 기인: /replan_flag(pub)

[Conflict Decider]
   ├─ /path_robot_conflict_event(sub)
   ├─ (opt) MultiRobotStateArray, area map(sub)
   └─ 결정:
        - /decision_cmd(pub)
        - /planner/keepout_mask(pub)
        - /controller/speed_limit_mask(pub)
        - /conflict_decider_state(pub)
```

---

# 세부 아이디어(운용 팁)

* **인덱스 매칭 내성**: PathValidator에서 ±1셀(또는 너의 `kernel_half_size`) 이내에 메타가 있으면 “로봇 기인”으로 인정.
* **메타 최신성**: `meta_staleness_ms > τ`이면 이벤트는 내보내되 `confidence↓`로 표기 → Decider가 보수적으로 처리.
* **reroute 의미**: 상위가 기존 goal을 취소/새 waypoints 할당한 **전환 구간**이므로, Decider는 짧은 **WAIT(예: 2s)** 후 여전히 충돌이면 **keepout + REPLAN**로 전환.
* **manual**: 사람 개입 우선. Decider는 **WAIT/REPLAN_WITH_KEEP_OUT** 선호, 협소구간이면 반드시 WAIT.
* **area(협소 공간/교차로)**: Decider가 **분산 락**처럼 사용(점유 중이면 상대 우선). PathValidator는 단지 “shared_area_id가 있다” 정도만 올려주면 충분.
* **QoS**:

  * `robot_layer_meta`: `transient_local + reliable`
  * 이벤트/명령: `volatile + reliable`
  * 상태/마스크: `transient_local + reliable`
* **디버깅**: RViz에

  * PathValidator 마커: 경로상의 충돌 포인트 + 원인(ENV/ROBOT)
  * RobotLayer: sparse 메타 히트 셀 시각화(색=phase, 점=mx,my)
  * Decider: keepout/speed-limit overlay + 현재 상태(FSM)

---

# 최종 결론

* **PathValidator는 “로봇 메타를 ‘읽기’만 하고, 해석은 하지 않는다.”**

  * 판단은 “ENV vs ROBOT” + 최소 요약 메타 추출까지만.
* **Conflict Decider가 is_reroute/is_manual/area 점유 등 모든 컨텍스트 해석을 담당**하고,

  * WAIT/REPLAN/SLOWDOWN/KEEPOUT 같은 **행동 정책**을 결정한다.
* **RobotLayer는 코스트 + 희소 메타를 잘 만들어 주는 생산자**로 머물고,

  * 메타는 “의미 있는 지점만” 내보내 성능을 지킨다.

이렇게 가져가면 각 모듈이 “단일 책임”을 갖고, 실시간성과 확장성이 둘 다 살아난다.
원하면 다음 턴에 **robot_layer_meta.msg / PathRobotConflictEvent.msg / keepout_mask.msg** 같은 메시지 스펙과, 각 노드의 파라미터 YAML 스켈레톤을 C++17/ROS 2 Jazzy 스타일로 바로 짜줄게.

  
########################  

좋아! 바로 쓸 수 있게 **ROS 2 Jazzy** 기준의 커스텀 메시지 스펙을 정리했어.
패키지명은 예시로 `multi_robot_msgs`라고 둘게. (필요에 맞게 바꿔도 됨)

아래 정의는 네가 말한 신호들을 모두 담고, 모듈 간 역할 분리에 맞춰 **3계층**으로 나눴어:

1. **상태/예측 계층**: 다른 로봇 상태 공유 (`MultiRobotState*`)
2. **코스트맵 메타 계층**: RobotLayer가 뽑아 주는 희소 메타 (`RobotLayer*`)
3. **의사결정 이벤트/명령 계층**: Validator→Decider, Decider→Nav2 (`PathRobotConflict*`, `KeepoutMask*`, `DecisionCmd` 등)

---

# 0) 공통 원칙 (권장)

* 좌표계: REP-105/103 준수. `header.frame_id`는 보통 `map`(글로벌), 시간은 `header.stamp` 사용.
* 단위: 거리[m], 속도[m/s], 각속도[rad/s], 시간[s].
* ID는 `string`(AMR/Fleet ID).
* Enum은 `uint8` + 상수로 정의.
* 대역을 아끼려면 “원시 그리드” 대신 **희소(sparse)** 또는 폴리곤/세그먼트 기반 단위 사용.

---

# 1) 상태/예측 계층 (Aggregator → RobotLayer/Decider)

## `msg/RobotFootprint.msg`

```text
# 로봇 외곽선(발자국) 지정. circle 또는 polygon 중 하나 사용.
uint8 TYPE_CIRCLE=0
uint8 TYPE_POLYGON=1
uint8 type

# circle
float32 radius

# polygon (CCW, base_link 기준)
geometry_msgs/Point32[] polygon
```

## `msg/TimedPose.msg`

```text
# 포즈와 상대시간(초). t=0이 현재 기준.
geometry_msgs/Pose pose
float32 t
```

## `msg/TimedTrajectory.msg`

```text
# 0~T 예측 궤적(이산화). frame_id는 상위의 header.frame_id를 따름.
std_msgs/Header header
multi_robot_msgs/TimedPose[] points
```

## `msg/RobotStatus.msg`

```text
# 상위 상태를 단일 enum + 플래그로 축약
uint8 PHASE_MOVING=0
uint8 PHASE_WAITING=1
uint8 PHASE_DOCKED=2
uint8 PHASE_LOADING=3
uint8 PHASE_UNLOADING=4
uint8 PHASE_ARRIVED=5
uint8 PHASE_ERROR=6
uint8 phase

bool is_manual         # mode==manual
bool is_transferring   # 부하 이송 중
bool is_reroute        # 기존 goal cancel 이후 첫 waypoint 도달 전까지 true
bool is_replanning     # 상위가 리플랜 중
```

## `msg/AreaTag.msg`

```text
# 협소구간/교차로 등 "임계 구역" 태깅
string area_id
bool   occupancy        # 이 area를 현재 점유(1) 중인지
```

## `msg/MultiRobotState.msg`

```text
# 단일 로봇의 현재 상태 + 근미래 궤적 프리뷰
std_msgs/Header header          # frame_id: map
string amr_id

geometry_msgs/PoseStamped current_pose
geometry_msgs/Twist       current_twist   # 없으면 (0,0,0)

multi_robot_msgs/RobotStatus status
multi_robot_msgs/RobotFootprint footprint

# 작업/경로 컨텍스트
string current_waypoint
string next_waypoint

# 0~5 m 경로 프리뷰(필요 시)
geometry_msgs/PoseStamped[] truncated_path  # frame_id: map

# 예측 궤적(옵션, 있으면 RobotLayer가 우선 사용)
multi_robot_msgs/TimedTrajectory trajectory

# 상호 인지/관계
string[] peer_cross_ids       # 교차 가능성이 높은 상대 id 리스트

# area(협소구간/교차로 등)
multi_robot_msgs/AreaTag area

# 품질/불확실도
float32 pose_covariance       # 간단화를 위해 1스칼라(표준편차[m]). 정교화 필요 시 Covariance 사용.
```

## `msg/MultiRobotStateArray.msg`

```text
std_msgs/Header header    # stamp: 생성 기준 시각, frame_id: map
multi_robot_msgs/MultiRobotState[] robots
```

---

# 2) 코스트맵 메타 계층 (RobotLayer → PathValidator/Decider)

RobotLayer는 내부 Costmap2D에 직접 쓰고, 별도로 **희소 메타**를 퍼블리시해서 PathValidator가 “로봇 기인 여부”를 빠르게 식별할 수 있게 한다.

## `msg/RobotLayerCellMeta.msg`

```text
# 코스트 셀과 결부된 "로봇 유래" 메타정보 (희소 형태)
# 좌표: map 좌표계(world) 또는 grid index 중 택1 (권장은 world)
# world가 디버깅에 유리하고, PathValidator는 world→map 변환해 인덱스 매칭

std_msgs/Header header         # frame_id: map
string amr_id
uint8  phase                   # RobotStatus.phase 값 재사용
bool   is_manual
bool   is_reroute
bool   is_replanning
string area_id                 # 해당되면 값, 아니면 빈 문자열

# 위치 표기 선택지 1) world
geometry_msgs/Point position   # 셀 센터 world 좌표 [m]

# 위치 표기 선택지 2) grid index (둘 다 채우면 world 우선)
uint32 mx
uint32 my

# 시간/불확실도
float32 t_first_hit            # 이 셀을 상대가 점유할/점유한 최초 예상 시각 [s], 없으면 음수
float32 sigma                  # [m]
```

## `msg/RobotLayerMetaArray.msg`

```text
# RobotLayer가 발행하는 희소 메타 패킷
std_msgs/Header header          # frame_id: map
multi_robot_msgs/RobotLayerCellMeta[] cells
```

> 퍼포먼스: 한 프레임에 전체 셀을 내보내지 말고, “경로 결정에 의미 있는 셀들(LETHAL 근처, 예측 교차 지대, area 경계 부근)”만 샘플링해서 발행해. PathValidator는 ±kernel 내에서 **하나라도 매칭되면 ‘로봇 기인’**으로 본다.

---

# 3) 의사결정 이벤트/명령 계층

## `msg/PathRobotConflictEvent.msg` (PathValidator → Decider)

```text
# 경로 위 충돌/교차 이벤트(로봇 기인)
std_msgs/Header header                 # stamp: 판정 시각, frame_id: map
string path_id                         # 검사한 경로 식별(해시/증분 id)

# 관련 로봇들(복수 가능)
string[] amr_ids
uint8[]  phases                        # RobotStatus.phase 값들
bool     is_manual_any
bool     is_reroute_any
bool     is_replanning_any

# 협소구간/교차로
string   shared_area_id                # 둘 다 같은 area면 채움, 아니면 빈 문자열
bool     area_occupancy_any            # 관련 로봇 중 누군가가 area 점유 중인지

# 수치 요약(있으면 채움)
float32  ttc_first                     # 최초 추돌 예상시각 [s], 모르면 음수
float32  conflict_length_m             # 경로상의 충돌 구간 길이 [m]
float32  min_distance_m                # 최근접 거리 [m]
float32  confidence                    # 0~1 (메타 최신성/품질 반영)

geometry_msgs/PoseStamped first_conflict_pose  # 경로상의 최초 충돌 지점
string   source_layer                  # 예: "global_costmap/robot_layer"
float32  meta_staleness_ms             # robot_layer_meta 최신성(추정)
string   note                          # 디버그 메시지(선택)
```

## `msg/PathRobotConflictState.msg` (상태 스냅샷; 선택)

```text
std_msgs/Header header
bool active
multi_robot_msgs/PathRobotConflictEvent last_event
```

> 이벤트는 `volatile`, 상태는 `transient_local`로 운영해 late-join 보호.

## `msg/DecisionCmd.msg` (Decider → 상위/로컬)

```text
uint8 CMD_GO=0
uint8 CMD_WAIT=1
uint8 CMD_SLOWDOWN=2
uint8 CMD_REPLAN_SOFT=3
uint8 CMD_REPLAN_WITH_KEEP_OUT=4

std_msgs/Header header
uint8 cmd
string reason
```

## `msg/AreaPolygon.msg` (정적/파라메터로도 가능)

```text
# 협소구역/교차로 영역 정의(지도 좌표계)
std_msgs/Header header          # frame_id: map
string area_id
geometry_msgs/Polygon polygon
```

## `msg/KeepoutMask.msg` (Decider → Planner)

```text
# 플래너에 주입할 임시 금지영역. 다각형 여러 개 + TTL
std_msgs/Header header          # frame_id: map
geometry_msgs/Polygon[] polygons
float32 dilation_m              # 추가 팽창 반경
float32 ttl_sec                 # 유효시간(초). 0 또는 <0이면 즉시 적용+즉시 만료(비권장).
string  source                  # "conflict_decider"
```

(필요 시 서비스 버전도 제공: `srv/ApplyKeepoutMask.srv`)

```text
# ApplyKeepoutMask.srv
multi_robot_msgs/KeepoutMask request
---
bool accepted
string message
```

## `msg/SpeedLimitMask.msg` (Decider → Controller/MPPI)

```text
# 로컬 ROI에서 속도 상한을 적용하기 위한 마스크
std_msgs/Header header          # frame_id: map (또는 base_link 근방이면 local)
geometry_msgs/Polygon[] regions
float32 v_max                   # [m/s] 해당 ROI 내 최대 속도
float32 ttl_sec
string  source
```

---

# 4) 토픽/큐오에스(QoS) 권장

* `multi_robot_states` (`MultiRobotStateArray`): `reliable + volatile` (상태는 10Hz 내외, 예측은 5–10Hz)
* `robot_layer_meta` (`RobotLayerMetaArray`): `reliable + transient_local + KeepLast(1)`
* `global_costmap/costmap_raw` (Nav2 표준): 기존 QoS 그대로
* `path_robot_conflict_event`: `reliable + volatile + KeepLast(10)`
* `path_robot_conflict_state`: `reliable + transient_local + KeepLast(1)`
* `decision_cmd`: `reliable + volatile + KeepLast(10)`
* `planner/keepout_mask`, `controller/speed_limit_mask`: `reliable + transient_local + KeepLast(1)`

---

# 5) 패키지 구성 힌트

`package.xml` (요약)

```xml
<package format="3">
  <name>multi_robot_msgs</name>
  <version>0.1.0</version>
  <description>Custom messages for multi-robot navigation (Nav2)</description>
  <maintainer email="you@example.com">You</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>builtin_interfaces</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

`CMakeLists.txt` (요약)

```cmake
cmake_minimum_required(VERSION 3.16)
project(multi_robot_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(MSGS
  msg/RobotFootprint.msg
  msg/TimedPose.msg
  msg/TimedTrajectory.msg
  msg/RobotStatus.msg
  msg/AreaTag.msg
  msg/MultiRobotState.msg
  msg/MultiRobotStateArray.msg
  msg/RobotLayerCellMeta.msg
  msg/RobotLayerMetaArray.msg
  msg/PathRobotConflictEvent.msg
  msg/PathRobotConflictState.msg
  msg/DecisionCmd.msg
  msg/AreaPolygon.msg
  msg/KeepoutMask.msg
  msg/SpeedLimitMask.msg
)

set(SRVS
  srv/ApplyKeepoutMask.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSGS}
  ${SRVS}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_package()
```

---

# 6) 사용 가이드(짧게)

* **Aggregator** → `MultiRobotStateArray` 발행
* **RobotLayer** → 이를 구독해 코스트맵 업데이트 + `RobotLayerMetaArray`(희소) 발행
* **PathValidator** → `costmap_raw`와 `robot_layer_meta`를 함께 보고

  * 환경 기인: `/replan_flag`
  * 로봇 기인: `PathRobotConflictEvent` 발행(필요 요약 메타 채워서)
* **Conflict Decider** → `PathRobotConflictEvent` 구독, 정책을 적용하고

  * `DecisionCmd` 발행
  * 필요 시 `KeepoutMask`/`SpeedLimitMask` 발행(or 서비스 호출)

---

# 7) 필드 활용 요약 (중요한 매핑)

* `RobotStatus.phase`: RobotLayer 코스트 가중, Decider 정책 분기(정적/동적/치명)
* `is_manual`: Decider에서 **WAIT/강한 keepout** 선호
* `is_reroute/is_replanning`: Decider가 짧은 WAIT 후 keepout 리플랜으로 전환
* `AreaTag(area_id, occupancy)`: Decider의 **critical section** 판정/락
* `peer_cross_ids`: 교차 가능성 신뢰도 가점(Decider)
* `RobotLayerCellMeta.{t_first_hit, sigma}`: TTC/불확실도 근거(Validator→Event, Decider 판정)

---

필요하면 이 스펙으로 바로 빌드되는 **샘플 패키지 스켈레톤**(폴더 구조/빈 메시지 파일 포함)과,
`PathValidator`, `ConflictDecider`, `RobotLayer`에서의 **퍼블/서브 코드 스니펫(C++17, Jazzy)**도 만들어줄게.
###################################  
####################################3  
#######################################  
좋아—요구사항 반영해서 **“robot → agent”로 전면 교체**하고, `RobotStatus.msg`도 **제공한 STATUS 테이블 기반으로 확장**했어. 패키지명도 `multi_agent_msgs`로 바꿨고, 모든 메시지/필드 참조를 일관되게 수정했다. (ROS 2 Jazzy 기준)

---

# 1) 상태/공유 계층 (Aggregator → Layer/Decider)

### `msg/MultiAgentInfo.msg`

```text
# frame_id: 보통 "map"
std_msgs/Header header

# --- 식별자 ---
uint16 machine_id                   # 에이전트 고유 숫자 ID (ex: 1001)
string type_id                      # 에이전트/플랫폼 타입 (ex: "MIR250", "CUSTOM_AMR_A")

# --- 현재 상태 ---
geometry_msgs/PoseStamped current_pose
geometry_msgs/Twist       current_twist   # 없으면 (0,0,0) 권장

# 발자국(다각형)
geometry_msgs/PolygonStamped footprint

# 상위 상태(확장 STATUS 테이블 기반)
multi_agent_msgs/AgentStatus status

# 리루트 전환 구간 플래그
bool   reroute                    # 기존 goal cancel→새 waypoints 배정~첫 waypoint 도달 전까지 true

# 웨이포인트 인덱스(0~255)
uint8  current_waypoint
uint8  next_waypoint

# 0~5 m 전방 경로 프리뷰(시간 정보 없는 포즈 배열)
geometry_msgs/PoseStamped[] truncated_path   # frame_id: header.frame_id

# --- 상호 관계/교차 인지 ---
uint16[] cross_agent_ids          # 교차 가능성이 높은 다른 agent들의 machine_id 목록

# --- 운용 컨텍스트 ---
string mode                       # "auto" | "manual" | 기타 플릿 표준 문자열

# --- 에어리어(협소/교차 섹션 등) ---
uint16 area_id                    # 해당 없으면 0(NONE)
bool   occupancy                  # 해당 area 점유 중인지

# --- 작업/리플랜 상태 플래그 ---
bool   transferring               # 부하 이송/스테이션 상호작용 중
bool   re_path_search             # 상위/플래너가 경로 재탐색 중

# --- 품질/불확실도 ---
float32 pos_std_m          # XY 위치 표준편차 [m], unknown이면 -1
float32 yaw_std_rad        # 요 표준편차 [rad], unknown이면 -1
```

### `msg/MultiAgentInfoArray.msg`

```text
std_msgs/Header header                 # stamp: 생성 기준 시각, frame_id: map
multi_agent_msgs/MultiAgentInfo[] agents
```

### `msg/AgentStatus.msg`  ← (STATUS 테이블 반영, 불리언 제거)

```text
# 상위 상태 열거형 (값은 요구 테이블과 동일하게 매핑)
uint8 STATUS_INIT                         = 0
uint8 STATUS_PATH_SEARCHING               = 1
uint8 STATUS_WAITING_FOR_OBS              = 2
uint8 STATUS_MOVING                       = 3
uint8 STATUS_ARRIVED                      = 4
uint8 STATUS_MARKING                      = 5
uint8 STATUS_UNLOADING                    = 6
uint8 STATUS_UNLOADED                     = 7
uint8 STATUS_LOADING                      = 8
uint8 STATUS_LOADED                       = 9
uint8 STATUS_AUTORECOVERY                 = 10   # 0x0A
uint8 STATUS_RECOVERING                   = 11

uint8 STATUS_MANUAL_RUNNING               = 12
uint8 STATUS_MANUAL_COMPLETE              = 13

uint8 STATUS_ERROR                        = 14   # 0x0E
uint8 STATUS_PAUSE                        = 15
uint8 STATUS_WAITING_FOR_SAFETY           = 16
uint8 STATUS_WAITING_FOR_FLOWCONTROL      = 17
uint8 STATUS_WAITING_FOR_ROS_STATUS       = 18

uint8 STATUS_CHARGING                     = 19
uint8 STATUS_CHARGE_DONE                  = 20
uint8 STATUS_UNKNOWN                      = 21

# 현재 상태 값
uint8 phase
```

> 요구대로 `is_manual / is_transferring / is_reroute / is_replanning` **모두 제거**했고, 위 불리언들은 `MultiAgentInfo.msg`의 별도 필드로 유지돼 정책/레이어에서 활용하면 된다.

---

# 2) 코스트맵 메타 계층 (AgentLayer → Validator/Decider)

### `msg/AgentLayerCellMeta.msg`

```text
std_msgs/Header header              # frame_id: map

uint16 machine_id                   # 유래 agent
uint8  phase                        # AgentStatus.phase 값
string mode                         # "auto"/"manual"/...
bool   reroute
bool   re_path_search
bool   transferring

uint16 area_id                      # 없으면 0

# 위치: world 또는 grid index (둘 다 있을 경우 world 우선)
geometry_msgs/Point position        # 셀 센터 world 좌표 [m]
uint32 mx
uint32 my

# 시간/불확실도
float32 t_first_hit                 # [s] 최초 점유/교차 예상 시각(모르면 음수)
float32 sigma                       # [m] 위치 불확실도 등가 반경
```

### `msg/AgentLayerMetaArray.msg`

```text
std_msgs/Header header              # frame_id: map
multi_agent_msgs/AgentLayerCellMeta[] cells
```

> AgentLayer는 Costmap2D 내부에 비용을 쓰고, **의미 있는 셀만 희소 메타**로 발행한다. PathValidator는 ±커널 내에 메타가 하나라도 매칭되면 “agent 기인”으로 판정.

---

# 3) 의사결정 이벤트/명령 계층

### `msg/PathAgentConflictEvent.msg`  ← (용어 교체)

```text
std_msgs/Header header                     # stamp: 판정 시각, frame_id: map
string path_id                             # 검사한 경로 식별자(해시 등)

# 관련 agent들
uint16[] machine_ids
uint8[]  phases                            # AgentStatus.phase 배열 (machine_ids와 길이 동일)
bool     is_manual_any
bool     is_reroute_any
bool     is_re_path_search_any
bool     transferring_any

# 협소/교차 area
uint16   shared_area_id                    # 동일 area면 값, 아니면 0
bool     area_occupancy_any                # 관련 agent 중 area 점유자가 있는가

# 수치 요약
float32  ttc_first                         # [s], 모르면 음수
float32  conflict_length_m                 # [m]
float32  min_distance_m                    # [m]
float32  confidence                        # 0~1 (메타 최신성/품질 반영)

geometry_msgs/PoseStamped first_conflict_pose
string   source_layer                      # 예: "global_costmap/agent_layer"
float32  meta_staleness_ms
string   note
```

### `msg/PathAgentConflictState.msg`

```text
std_msgs/Header header
bool active
multi_agent_msgs/PathAgentConflictEvent last_event
```

### `msg/DecisionCmd.msg`

```text
uint8 CMD_GO=0
uint8 CMD_WAIT=1
uint8 CMD_SLOWDOWN=2
uint8 CMD_REPLAN_SOFT=3
uint8 CMD_REPLAN_WITH_KEEP_OUT=4

std_msgs/Header header
uint8  cmd
string reason
```

### `msg/AreaPolygon.msg`

```text
std_msgs/Header header          # frame_id: map
uint16 area_id
geometry_msgs/Polygon polygon
```

### `msg/KeepoutMask.msg`

```text
std_msgs/Header header          # frame_id: map
geometry_msgs/Polygon[] polygons
float32 dilation_m              # [m]
float32 ttl_sec                 # [s]
string  source
```

### `msg/SpeedLimitMask.msg`

```text
std_msgs/Header header          # frame_id: map (또는 local)
geometry_msgs/Polygon[] regions
float32 v_max                   # [m/s] ROI 내 속도 상한
float32 ttl_sec                 # [s]
string  source
```

### `srv/ApplyKeepoutMask.srv`  (선택)

```text
multi_agent_msgs/KeepoutMask request
---
bool accepted
string message
```

---

# 4) 권장 토픽 & QoS

* `multi_agent_infos` (`MultiAgentInfoArray`): **reliable + volatile**, 5–10 Hz
* `agent_layer_meta` (`AgentLayerMetaArray`): **reliable + transient_local + KeepLast(1)**
* `path_agent_conflict_event`: **reliable + volatile + KeepLast(10)**
* `path_agent_conflict_state`: **reliable + transient_local + KeepLast(1)**
* `decision_cmd`: **reliable + volatile + KeepLast(10)**
* `planner/keepout_mask`, `controller/speed_limit_mask`: **reliable + transient_local + KeepLast(1)**

---

# 5) 패키지 빌드 스켈레톤

### `package.xml`

```xml
<?xml version="1.0"?>
<package format="3">
  <name>multi_agent_msgs</name>
  <version>1.0.0</version>
  <description>Custom messages for multi-agent navigation (Nav2)</description>
  <maintainer email="you@example.com">You</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>builtin_interfaces</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.16)
project(multi_agent_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(MSGS
  msg/AgentStatus.msg
  msg/MultiAgentInfo.msg
  msg/MultiAgentInfoArray.msg
  msg/AgentLayerCellMeta.msg
  msg/AgentLayerMetaArray.msg
  msg/PathAgentConflictEvent.msg
  msg/PathAgentConflictState.msg
  msg/DecisionCmd.msg
  msg/AreaPolygon.msg
  msg/KeepoutMask.msg
  msg/SpeedLimitMask.msg
)

set(SRVS
  srv/ApplyKeepoutMask.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSGS}
  ${SRVS}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_package()
```

---

# 6) 운용 메모

* `AgentStatus.phase`는 **단일 소스 오브 트루스**로 상위 상태를 제공하고,
  `MultiAgentInfo`의 `mode/reroute/transferring/re_path_search` 등 **운영 플래그**는 별도 정책 입력으로 활용.
* `area_id==0`을 NONE으로 통일하면 레이어/디사이더 로직 단순화.
* `cross_agent_ids`는 `machine_id` 네임스페이스와 동일하게 맞출 것.
* 레이어/밸리데이터/디사이더에서 “robot” 문자열이 남지 않도록 토픽/네임스페이스/로그 메시지도 **agent**로 정리.

원하면, 이 스펙으로 바로 생성 가능한 **빈 .msg/.srv 파일 세트**와 간단 퍼블/서브 **C++ 예제(Modern C++17, Jazzy)**도 만들어줄게.
