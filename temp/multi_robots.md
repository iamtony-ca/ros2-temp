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
######################  
######################  
좋아요! 아래 파이썬 노드는 `/multi_agent_infos` 토픽에 **10 Hz**로 `MultiAgentInfoArray` 를 퍼블리시합니다. 에이전트는 2대:

* **agent A**: 정지(STATUS_WAITING)
* **agent B**: 이동(STATUS_MOVING, 원형 트랙 주행)

프레임은 `map` 기준이고, 각 에이전트의 footprint(사각형)와 truncated_path(최대 20포즈)를 넣었습니다.

저장 파일명 예: `multi_agent_mock_pub.py`

```python
#!/usr/bin/env python3
import math
from typing import List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point32
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import Path

# --- multi_agent_msgs (사용자 패키지) ---
from multi_agent_msgs.msg import MultiAgentInfoArray, MultiAgentInfo, AgentStatus

FRAME = "map"
PUB_TOPIC = "/multi_agent_infos"
HZ = 10.0
MAX_POSES = 20

def rect_footprint(w: float, l: float) -> PolygonStamped:
    """centered rectangle footprint (width w, length l) in robot local frame"""
    fp = PolygonStamped()
    fp.header.frame_id = FRAME  # frame은 레이어에서 검증 안하지만 맞춰둡니다
    hw, hl = w * 0.5, l * 0.5
    pts = [
        Point32(x= hl, y= hw, z=0.0),
        Point32(x= hl, y=-hw, z=0.0),
        Point32(x=-hl, y=-hw, z=0.0),
        Point32(x=-hl, y= hw, z=0.0),
    ]
    fp.polygon.points = pts
    return fp

def pose_xyth(x: float, y: float, th: float) -> Pose:
    p = Pose()
    p.position.x = x
    p.position.y = y
    # z=0, quaternion from yaw
    c = math.cos(th * 0.5)
    s = math.sin(th * 0.5)
    p.orientation.z = s
    p.orientation.w = c
    return p

def make_path(poses: List[Pose]) -> Path:
    path = Path()
    path.header.frame_id = FRAME
    for po in poses:
        ps = PoseStamped()
        ps.header.frame_id = FRAME
        ps.pose = po
        path.poses.append(ps)
    return path


class MultiAgentMockPub(Node):
    def __init__(self):
        super().__init__("multi_agent_mock_pub")
        self.pub = self.create_publisher(MultiAgentInfoArray, PUB_TOPIC, 10)
        self.t = 0.0
        self.dt = 1.0 / HZ
        self.timer = self.create_timer(self.dt, self.on_timer)

        # 공통 footprint (폭 0.5 m, 길이 0.7 m)
        self.fp = rect_footprint(0.50, 0.70)

        # 고정 에이전트 파라미터
        self.agentA_id = 101  # 정지
        self.agentB_id = 202  # 이동
        self.type_id = "amr"

        self.get_logger().info(f"Publishing {PUB_TOPIC} at {HZ:.1f} Hz")

    def on_timer(self):
        now = self.get_clock().now().to_msg()

        # --- Agent A: 정지 (대기) ---
        aA = MultiAgentInfo()
        aA.machine_id = self.agentA_id
        aA.type_id = self.type_id
        aA.mode = "auto"
        aA.pos_std_m = 0.03  # 위치 표준편차 예시
        aA.footprint = self.fp

        aA.status = AgentStatus()
        aA.status.phase = AgentStatus.STATUS_WAITING

        # 고정 위치 (x=1.0, y=0.0)
        A_theta = 0.0
        A_pose = pose_xyth(1.0, 0.0, A_theta)
        aA.current_pose = PoseStamped()
        aA.current_pose.header.stamp = now
        aA.current_pose.header.frame_id = FRAME
        aA.current_pose.pose = A_pose

        aA.truncated_path = make_path([A_pose] * MAX_POSES)  # 정지: 동일 포즈 반복

        # --- Agent B: 이동 (원형 경로) ---
        aB = MultiAgentInfo()
        aB.machine_id = self.agentB_id
        aB.type_id = self.type_id
        aB.mode = "auto"
        aB.pos_std_m = 0.05
        aB.footprint = self.fp

        aB.status = AgentStatus()
        aB.status.phase = AgentStatus.STATUS_MOVING

        # 원 중심 (x=-1.0, y=0.0), 반경 1.0 m, 각속도 0.25 rad/s
        R = 1.0
        w = 0.25
        cx, cy = -1.0, 0.0

        # 경로 포즈들 (미래 진행방향으로 MAX_POSES 샘플)
        path_poses = []
        for k in range(MAX_POSES):
            th = w * (self.t + k * self.dt)
            x = cx + R * math.cos(th)
            y = cy + R * math.sin(th)
            yaw = th + math.pi / 2.0  # 진행방향을 접선방향으로 설정
            path_poses.append(pose_xyth(x, y, yaw))

        aB.truncated_path = make_path(path_poses)
        aB.current_pose = PoseStamped()
        aB.current_pose.header.stamp = now
        aB.current_pose.header.frame_id = FRAME
        aB.current_pose.pose = path_poses[0]  # 현재는 첫 포즈로

        # --- Array 채우기 ---
        arr = MultiAgentInfoArray()
        arr.header.stamp = now
        arr.header.frame_id = FRAME
        arr.agents = [aA, aB]

        self.pub.publish(arr)
        self.t += self.dt


def main():
    rclpy.init()
    node = MultiAgentMockPub()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### 실행 방법

1. ROS2 환경과 당신의 워크스페이스를 source:

```bash
source /opt/ros/jazzy/setup.bash
source ~/temp_ws/install/setup.bash
```

2. 스크립트 저장 후 실행:

```bash
python3 multi_agent_mock_pub.py
```

> `AgentLayer` 의 기본 구독 토픽이 `/multi_agent_infos` 이므로 별도 파라미터 변경 없이 바로 들어갑니다.
> 프레임은 `map` 으로 맞췄고, 레이어 파라미터 `use_path_header_frame:=true` 인 경우에도 통과합니다.

필요하면 ID나 경로 모양, 주행 각속도/반경만 바꿔서 다양한 테스트 케이스를 바로 만들 수 있어요.
  
##########################################  
##########################################  
##########################################  
맞아요. **agent_layer만으로도** 로봇의 footprint(다각형)와 truncated_path(포즈들)를 따라 **직접 코스트를 칠해서** “충돌금지 영역”을 만들 수 있어요. 그래서 *반드시* `inflation_layer`가 있어야 하는 건 아니에요. 다만 `inflation_layer`를 뒤에 두면, 우리가 칠한 lethal(254) 주변으로 **그라데이션 완충대**가 생겨 경로계획이 더 부드러워집니다.

---

# 병합 방식(Max-merge) 상세

현재 코드는 `fillFootprintAt()`에서 **master 그리드에 바로 set**하고 있어요:

```cpp
grid->setCost(static_cast<unsigned int>(i),
              static_cast<unsigned int>(j), cost);
```

이렇게 “그냥 덮어쓰기” 하면, **뒤에서 동작하는 다른 레이어가 값을 낮추거나(또는 덮어) 버릴 가능성**이 있어요.
이를 막는 가장 단순하고 효과적인 방법이 바로 **Max 병합**이에요:

> “쓸 때, **기존 값보다 클 때만** 올린다”

즉,

```cpp
const unsigned char old = grid->getCost(i, j);
if (cost > old) {
  grid->setCost(i, j, cost);
}
```

이렇게 하면 **다른 레이어가 얼마를 쓰든**, 최종 master에선 **가장 높은 비용(=가장 보수적인 장애물 해석)** 이 남습니다. 그래서 플래너가 그 셀을 **반드시 회피**하게 만들 수 있어요.

---

# 어디에 넣어야 하나? (구체 위치)

당신 코드 기준으로, **이 두 군데 중 한 곳**에 넣으면 됩니다. (1번이 가장 직관적)

## 1) `fillFootprintAt()` 안에서 “셀 찍는” 바로 그 자리

```cpp
// (중략) for (int j = min_j; j <= max_j; ++j) {
  for (int i = min_i; i <= max_i; ++i) {
    double wx, wy; grid->mapToWorld(i, j, wx, wy);
    // inside 계산 (생략)

    if (inside) {
      // ★ 바꾸는 부분 (Max-merge)
      const unsigned char old = grid->getCost(i, j);
      if (cost > old) {
        grid->setCost(static_cast<unsigned int>(i),
                      static_cast<unsigned int>(j), cost);
      }

      if (meta_hits) meta_hits->emplace_back(
          static_cast<unsigned int>(i), static_cast<unsigned int>(j));
    }
  }
// }
```

* 장점: 지금 구조 그대로, **가장 작은 수정**으로 “항상 최댓값 유지”를 강제합니다.
* 효과: 다른 레이어가 이후에 값을 덮어도 **우리를 낮출 수 없음**.

## 2) `updateCosts()`로 옮겨서 한 번 더 안전망

혹시 추후에 `fillFootprintAt()` 내부 구현을 바꿔도 안전하도록, `updateCosts()` 말미에 “우리가 찍은 좌표들”(`meta_hits`)을 **최댓값으로 보정**하는 것도 선택지입니다:

```cpp
for (auto & cell : meta_hits) {
  auto [mx, my] = cell;
  const unsigned char old = master_grid.getCost(mx, my);
  if (my_value > old) { // my_value = 이번 사이클 우리가 의도한 코스트(예: waiting_cost_ 또는 moving_cost_)
    master_grid.setCost(mx, my, my_value);
  }
}
```

* 단, 이 경우 `my_value`(우리가 의도한 cost)를 함께 보관해야 해서 1)보다 코드가 조금 더 커집니다.

---

# 한 단계 더: 내부 costmap을 쓰는 방법(선택)

* Nav2의 많은 플러그인은 `nav2_costmap_2d::CostmapLayer`를 상속해 **자기 레이어 내부 costmap**에 먼저 그린 뒤,
* 마지막에 `updateWithMax()` 같은 **헬퍼 병합**으로 master에 반영합니다.

지금은 `nav2_costmap_2d::Layer`를 상속 중이라 내부 costmap과 헬퍼가 없어요.
**상속을 `CostmapLayer`로 바꾸고** 그 흐름을 쓰면 구조가 더 견고해지지만, 구현량이 늘기 때문에 **현재 목적(확실히 막기)** 으론 위의 “Max-merge 한 줄”이 제일 간단하고 충분합니다.

---

# 요약 실전 팁

1. **agent_layer만으로도** footprint+path를 칠해 “장애영역” 만들 수 있음(=inflation 없이도 됨).
2. 경로가 관통하지 않게 하려면, **반드시 Max-merge** 하세요.

   * 바꾸는 곳: `fillFootprintAt()` 내부의 `setCost()` 직전에
   * 코드: `if (cost > grid->getCost(i, j)) setCost(i, j, cost);`
3. 인플레이션을 쓰면(**agent_layer → inflation_layer** 순서), agent 영역 주변에 **그라데이션 완충대**가 생겨 더 자연스러운 회피가 됩니다.
##################
######################
######################
좋은 포인트 잡았어요—지금 “아무 것도 안 보이는” 건 **Max-merge 비교식** 때문입니다.

## 왜 안 보였나?

`master_grid.getCost(i,j)`가 **NO_INFORMATION(=255)** 인 셀에서,

```cpp
if (cost > old)  // old == 255
```

는 항상 거짓이 됩니다(당신이 쓰는 cost는 0~254 범위). 그래서 **전혀 쓰이지 않아요**.
Global costmap은 초기값이 NO_INFORMATION(255)이고, static_layer가 아직 그 셀을 채우지 않았거나 창 범위 바깥이면 255가 남아있을 수 있어요.

## 바로 고치는 한 줄

NO_INFORMATION은 “0으로 간주”해서 비교해야 합니다. 아래처럼 바꿔주세요.

```cpp
const unsigned char old_raw = grid->getCost(i, j);
const unsigned char old = (old_raw == nav2_costmap_2d::NO_INFORMATION) ? 0 : old_raw;

if (cost > old) {
  grid->setCost(static_cast<unsigned int>(i),
                static_cast<unsigned int>(j), cost);
}
```

> 이 변경만으로 다시 RViz에 agent_layer가 정상적으로 칠해질 겁니다.

---

## “inflation을 넣어도 똑같이 보이는” 이유 & 제대로 쓰는 법

* 지금 agent_layer는 **단일 상수 코스트**(예: 200 또는 254)를 직접 칠합니다. 그러니 **단색**으로 보이는 게 정상이에요.
* **그라데이션(색 번짐)** 을 원하면 `inflation_layer`가 **agent_layer 이후**에 와야 해요.

  * 올바른 순서 예:

    ```yaml
    plugins: ["static_layer", "obstacle_layer", "voxel_layer", "agent_layer", "inflation_layer"]
    ```
  * 만약 `inflation_layer`가 agent_layer **앞**에 있으면, agent_layer가 나중에 덮어써 버려서 **inflation 효과가 안 보입니다.**
* 또 하나, 당신 코드가 **상수 코스트**만 찍기 때문에 인플레이션 없이는 당연히 단색입니다. “인플레이션 없이도 완충대”를 원하면, `fillFootprintAt()`에서 가장자리에서 멀어질수록 코스트를 줄이는 **거리기반 falloff**(예: `LETHAL - k*distance`)로 직접 칠하면 됩니다.

---

## 체크리스트 (한 번에 정리)

1. **NO_INFO 처리 추가**: 위 “한 줄 수정” 적용.
2. **플러그인 순서**: `agent_layer` → `inflation_layer` 순으로 배치.
3. **경로가 관통하는 문제**:

   * 지금처럼 254(LETHAL에 근접)로 칠하고 Max-merge를 쓰면 **관통하지 않습니다**.
   * 여전히 관통하면, 순서를 다시 확인하거나, planner가 쓰는 costmap이 global인지(local 아님)를 재확인하세요.
4. **색이 단색인 이유**: agent_layer가 상수 코스트만 칠하기 때문.

   * 그라데이션을 원하면 위 순서를 지키거나, agent_layer 내부에서 거리기반 코스트로 변경.

원하시면, `fillFootprintAt()` 내부에 **거리기반 코스트 예시**(에지에서 중심으로 갈수록 낮아지는/높아지는 방식) 코드도 바로 넣어드릴게요.


#################  
################  
맞아요—이 증상은 **파라미터 네임스페이스가 안 맞아서** YAML 값이 코드에 안 들어오는 전형적인 경우예요.
지금 코드는 `node_shared_->declare_parameter("waiting_cost", ...)`처럼 **루트(전역) 파라미터**로 선언/조회하고 있고, 실제 YAML은 보통

```
global_costmap:
  global_costmap:
    agent_layer:
      plugin: multi_agent_nav2::AgentLayer
      waiting_cost: 254
```

처럼 **플러그인 인스턴스 이름(`agent_layer`) 아래**에 들어가 있습니다. 즉, 선언/조회할 때 **플러그인 이름 접두사**를 붙여야 해요.

## 고치는 법 (안전·권장)

Costmap2D의 `Layer`가 제공하는 **헬퍼**를 쓰면 깔끔합니다.

* 선언은 `declareParameter("키", 기본값)`
* 조회는 `node_->get_parameter(name_ + ".키", 변수)`

  * 여기서 `name_`은 이 레이어 인스턴스의 이름(예: `agent_layer`)이고, `Layer`의 멤버입니다.

### 코드 수정 예

```cpp
// before (전역 이름으로 선언 → YAML 네임스페이스와 불일치)
node_shared_->declare_parameter("waiting_cost", rclcpp::ParameterValue(200));
int tmp = 200;
node_shared_->get_parameter("waiting_cost", tmp);

// after (플러그인 이름 접두사 자동 처리)
declareParameter("waiting_cost", rclcpp::ParameterValue(200));  // ← Layer::declareParameter 사용
int tmp = 200;
node_->get_parameter(name_ + std::string(".waiting_cost"), tmp);
waiting_cost_ = static_cast<unsigned char>(std::clamp(tmp, 0, 254));
```

> 위와 같은 패턴으로 `enabled`, `moving_cost`, `lethal_cost` 등 **모든 플러그인 파라미터**를 바꿔주세요.
> (`declareParameter(...)`는 `Layer`가 제공, `node_`/`name_`도 `Layer`의 멤버입니다.)

## 대안 (헬퍼 안 쓰고 직접 접두사 붙이기)

헬퍼 대신 직접 문자열을 합쳐도 됩니다.

```cpp
const std::string ns = name_ + ".";
node_shared_->declare_parameter(ns + "waiting_cost", rclcpp::ParameterValue(200));
int tmp = 200;
node_shared_->get_parameter(ns + "waiting_cost", tmp);
```

## YAML 구조 체크리스트

Iron/Jazzy 스타일(단일 `plugins` + `plugin:` 키)일 때:

```yaml
global_costmap:
  global_costmap:
    plugins: ["static_layer", "obstacle_layer", "agent_layer", "inflation_layer"]

    agent_layer:
      plugin: multi_agent_nav2::AgentLayer
      enabled: true
      waiting_cost: 254
      moving_cost: 180
      lethal_cost: 254
      # ... 기타 파라미터
```

* **플러그인 인스턴스 이름**(`agent_layer`)이 `name_`와 일치해야 합니다.
* `plugin:` 타입은 `agent_layer_plugin.xml`의 `type`과 같아야 합니다(예: `multi_agent_nav2::AgentLayer`).
* 이 YAML은 **해당 노드**(`/global_costmap/global_costmap`)의 파라미터로 들어갑니다.
  확인:

  ```
  ros2 param get /global_costmap/global_costmap agent_layer.waiting_cost
  ```

## 반영 타이밍

* 대부분의 레이어 파라미터는 **configure 시점**에 읽습니다. YAML만 바꾸고 **Nav2 재실행**(또는 해당 lifecycle 노드 `deactivate→cleanup→configure→activate`)을 해야 적용됩니다.
* 런타임 동적 파라미터가 아니면 live-update는 안 돼요.

## 왜 C++ 상수만 바꾸면 “바로 먹히는” 것처럼 보였나?

YAML이 네임스페이스 불일치로 무시되니, 결국 코드 기본값(상수)이 그대로 적용된 겁니다. C++의 기본값을 바꾸면 그게 곧바로 반영된 것처럼 느껴진 거죠.

---

필요하시면, 제가 **전체 파라미터 선언/조회 블록**을 `declareParameter`/`name_` 방식으로 전부 바꾼 패치를 한 번에 정리해 드릴게요.

  #####################  
######################  
좋아요. 아래는 **상태별 정책(현재 footprint=치사, 경로=감쇠 소프트 필드)**를 반영한 **완전한 hpp/cpp**입니다.
기존 구조와 최대한 호환되도록 했고, 다음이 핵심입니다.

* 현재(각 에이전트의 **current_pose**)는 **footprint + 상태별 dilation**으로 **치사(대개 254)** 채움.
* 미래(**truncated_path**)는 각 포즈의 **누적 길이 s**와 셀의 **횡거리 d**로 계산한
  ( c = \min\big(c_\text{base} + C_\text{max} e^{-s/\lambda} e^{-d^2/(2\sigma^2)},, c_\text{cap}\big) )
  를 **Max-merge**로 반영.
* 진행방향 콘(옵션): 앞쪽 콘에 추가 버프(`cone_boost_`)를 가산.
* 파라미터 전부 declare/get, YAML로 튜닝 가능.
* 기존 `fillFootprintAt()`·`rasterizeAgentPath()`를 유지하면서, **경로 소프트 필드용 새 루틴**(원형 가우시안 주변만 스캔) 추가.

---

# `include/multi_agent_nav2/agent_layer.hpp`

```cpp
#pragma once

#include <mutex>
#include <vector>
#include <string>
#include <utility>
#include <algorithm>
#include <cmath>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <multi_agent_msgs/msg/multi_agent_info_array.hpp>
#include <multi_agent_msgs/msg/agent_layer_meta_array.hpp>

namespace multi_agent_nav2
{

class AgentLayer : public nav2_costmap_2d::Layer
{
public:
  AgentLayer();

  // lifecycle
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override { current_ = true; }

  // costmap callbacks
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;

  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;

  bool isClearable() override { return true; }

private:
  // node
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_shared_;

  // I/O
  rclcpp::Subscription<multi_agent_msgs::msg::MultiAgentInfoArray>::SharedPtr sub_;
  rclcpp::Publisher<multi_agent_msgs::msg::AgentLayerMetaArray>::SharedPtr meta_pub_;

  // last data
  std::mutex data_mtx_;
  multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr last_infos_;
  rclcpp::Time last_stamp_;

  // basic parameters
  bool        enabled_{true};
  std::string topic_{"/multi_agent_infos"};
  uint16_t    self_machine_id_{0};
  std::string self_type_id_{};
  bool        use_path_header_frame_{true};
  double      roi_range_m_{12.0};
  int         freshness_timeout_ms_{800};
  int         max_poses_{40};
  bool        qos_reliable_{true};
  bool        publish_meta_{false};
  int         meta_stride_{3};

  // status → cost policy parameters
  // footprint(현재 위치) 강도
  unsigned char fp_cost_wait_like_{254};
  unsigned char fp_cost_arrived_like_{254};
  unsigned char fp_cost_error_like_{254};
  unsigned char fp_cost_moving_like_{254};
  unsigned char fp_cost_default_{240};

  // 경로 소프트 필드 기본/캡/최대가산
  unsigned char path_cost_base_{170};
  unsigned char path_cost_cap_{253};
  int           path_soft_cmax_{100};      // C_max
  double        lambda_m_{2.0};            // 종방향 감쇠 길이
  double        sigma_lat_m_{0.45};        // 횡방향 가우시안 폭

  // 상태별 dilation (footprint 추가 여유)
  double        dilate_wait_m_{0.2};
  double        dilate_arrived_m_{0.35};
  double        dilate_error_m_{0.5};
  double        dilate_moving_curr_m_{0.1};
  double        pos_sigma_k_{2.0};         // 위치 표준편차 가산 계수(불확실도)

  // 진행방향 콘 옵션
  double        heading_cone_deg_{40.0};
  int           cone_boost_{40};

  // bounds cache
  double touch_min_x_{0.0}, touch_min_y_{0.0}, touch_max_x_{0.0}, touch_max_y_{0.0};
  bool   touched_{false};

  // helpers
  void infosCallback(const multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr msg);
  bool isSelf(const multi_agent_msgs::msg::MultiAgentInfo & a) const;
  bool stale(const rclcpp::Time & stamp) const;

  // 상태 프로파일
  struct CostProfile
  {
    unsigned char fp_cost;  // 현재 footprint용 (대부분 254)
    unsigned char base;     // 경로 소프트 필드의 c_base
    double        dilate;   // footprint dilation
  };
  CostProfile profileFor(uint8_t phase) const;

  // rasterizers
  void rasterizeAgent(const multi_agent_msgs::msg::MultiAgentInfo & a,
                      nav2_costmap_2d::Costmap2D * grid,
                      std::vector<std::pair<unsigned int,unsigned int>> & meta_hits);

  void fillFootprintAt(const geometry_msgs::msg::PolygonStamped & fp,
                       const geometry_msgs::msg::Pose & pose,
                       double extra_dilation_m,
                       nav2_costmap_2d::Costmap2D * grid,
                       unsigned char cost,
                       std::vector<std::pair<unsigned int,unsigned int>> * meta_hits = nullptr);

  // 경로 소프트 필드(라디얼 가우시안, 원형 이웃만 스캔)
  void softenAlongPath(const std::vector<geometry_msgs::msg::PoseStamped> & path,
                       nav2_costmap_2d::Costmap2D * grid,
                       unsigned char c_base,
                       std::vector<double> * prefix_s,   // 누적길이(미리계산)
                       std::vector<std::pair<unsigned int,unsigned int>> * meta_hits);

  // 유틸
  static inline double yawOf(const geometry_msgs::msg::Quaternion & q);
  static inline double hypot2(double dx, double dy) { return std::sqrt(dx*dx + dy*dy); }
};

} // namespace multi_agent_nav2
```

---

# `src/agent_layer.cpp`

```cpp
#include "multi_agent_nav2/agent_layer.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>

namespace multi_agent_nav2
{

AgentLayer::AgentLayer() {}

void AgentLayer::onInitialize()
{
  node_shared_ = node_.lock();
  if (!node_shared_) {
    throw std::runtime_error("AgentLayer: failed to lock lifecycle node");
  }

  // === Declare parameters ===
  node_shared_->declare_parameter("enabled", rclcpp::ParameterValue(true));
  node_shared_->declare_parameter("topic", rclcpp::ParameterValue(std::string("/multi_agent_infos")));
  node_shared_->declare_parameter("self_machine_id", rclcpp::ParameterValue(0));
  node_shared_->declare_parameter("self_type_id", rclcpp::ParameterValue(std::string("")));
  node_shared_->declare_parameter("use_path_header_frame", rclcpp::ParameterValue(true));
  node_shared_->declare_parameter("roi_range_m", rclcpp::ParameterValue(12.0));
  node_shared_->declare_parameter("freshness_timeout_ms", rclcpp::ParameterValue(800));
  node_shared_->declare_parameter("max_poses", rclcpp::ParameterValue(40));
  node_shared_->declare_parameter("qos_reliable", rclcpp::ParameterValue(true));
  node_shared_->declare_parameter("publish_meta", rclcpp::ParameterValue(false));
  node_shared_->declare_parameter("meta_stride", rclcpp::ParameterValue(3));

  // footprint costs
  node_shared_->declare_parameter("fp_cost_wait_like", rclcpp::ParameterValue(254));
  node_shared_->declare_parameter("fp_cost_arrived_like", rclcpp::ParameterValue(254));
  node_shared_->declare_parameter("fp_cost_error_like", rclcpp::ParameterValue(254));
  node_shared_->declare_parameter("fp_cost_moving_like", rclcpp::ParameterValue(254));
  node_shared_->declare_parameter("fp_cost_default", rclcpp::ParameterValue(240));

  // path soft field
  node_shared_->declare_parameter("path_cost_base", rclcpp::ParameterValue(170));
  node_shared_->declare_parameter("path_cost_cap", rclcpp::ParameterValue(253));
  node_shared_->declare_parameter("path_soft_cmax", rclcpp::ParameterValue(100));
  node_shared_->declare_parameter("lambda_m", rclcpp::ParameterValue(2.0));
  node_shared_->declare_parameter("sigma_lat_m", rclcpp::ParameterValue(0.45));

  // dilations
  node_shared_->declare_parameter("dilate_wait_m", rclcpp::ParameterValue(0.2));
  node_shared_->declare_parameter("dilate_arrived_m", rclcpp::ParameterValue(0.35));
  node_shared_->declare_parameter("dilate_error_m", rclcpp::ParameterValue(0.5));
  node_shared_->declare_parameter("dilate_moving_curr_m", rclcpp::ParameterValue(0.1));
  node_shared_->declare_parameter("pos_sigma_k", rclcpp::ParameterValue(2.0));

  // heading cone
  node_shared_->declare_parameter("heading_cone_deg", rclcpp::ParameterValue(40.0));
  node_shared_->declare_parameter("cone_boost", rclcpp::ParameterValue(40));

  // === Get parameters ===
  node_shared_->get_parameter("enabled", enabled_);
  node_shared_->get_parameter("topic", topic_);
  {
    int tmp = 0; node_shared_->get_parameter("self_machine_id", tmp);
    self_machine_id_ = static_cast<uint16_t>(tmp);
  }
  node_shared_->get_parameter("self_type_id", self_type_id_);
  node_shared_->get_parameter("use_path_header_frame", use_path_header_frame_);
  node_shared_->get_parameter("roi_range_m", roi_range_m_);
  node_shared_->get_parameter("freshness_timeout_ms", freshness_timeout_ms_);
  node_shared_->get_parameter("max_poses", max_poses_);
  node_shared_->get_parameter("qos_reliable", qos_reliable_);
  node_shared_->get_parameter("publish_meta", publish_meta_);
  node_shared_->get_parameter("meta_stride", meta_stride_);

  int tmp_uc = 0;
  node_shared_->get_parameter("fp_cost_wait_like", tmp_uc);        fp_cost_wait_like_   = static_cast<unsigned char>(std::clamp(tmp_uc, 0, 254));
  node_shared_->get_parameter("fp_cost_arrived_like", tmp_uc);     fp_cost_arrived_like_= static_cast<unsigned char>(std::clamp(tmp_uc, 0, 254));
  node_shared_->get_parameter("fp_cost_error_like", tmp_uc);       fp_cost_error_like_  = static_cast<unsigned char>(std::clamp(tmp_uc, 0, 254));
  node_shared_->get_parameter("fp_cost_moving_like", tmp_uc);      fp_cost_moving_like_ = static_cast<unsigned char>(std::clamp(tmp_uc, 0, 254));
  node_shared_->get_parameter("fp_cost_default", tmp_uc);          fp_cost_default_     = static_cast<unsigned char>(std::clamp(tmp_uc, 0, 254));

  node_shared_->get_parameter("path_cost_base", tmp_uc);           path_cost_base_      = static_cast<unsigned char>(std::clamp(tmp_uc, 0, 254));
  node_shared_->get_parameter("path_cost_cap", tmp_uc);            path_cost_cap_       = static_cast<unsigned char>(std::clamp(tmp_uc, 0, 254));
  node_shared_->get_parameter("path_soft_cmax", path_soft_cmax_);
  node_shared_->get_parameter("lambda_m", lambda_m_);
  node_shared_->get_parameter("sigma_lat_m", sigma_lat_m_);

  node_shared_->get_parameter("dilate_wait_m", dilate_wait_m_);
  node_shared_->get_parameter("dilate_arrived_m", dilate_arrived_m_);
  node_shared_->get_parameter("dilate_error_m", dilate_error_m_);
  node_shared_->get_parameter("dilate_moving_curr_m", dilate_moving_curr_m_);
  node_shared_->get_parameter("pos_sigma_k", pos_sigma_k_);

  node_shared_->get_parameter("heading_cone_deg", heading_cone_deg_);
  node_shared_->get_parameter("cone_boost", cone_boost_);

  current_ = true;
  matchSize();

  if (enabled_) activate();
}

void AgentLayer::activate()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  if (qos_reliable_) qos.reliable(); else qos.best_effort();

  sub_ = node_shared_->create_subscription<multi_agent_msgs::msg::MultiAgentInfoArray>(
      topic_, qos, std::bind(&AgentLayer::infosCallback, this, std::placeholders::_1));

  if (publish_meta_) {
    meta_pub_ = node_shared_->create_publisher<multi_agent_msgs::msg::AgentLayerMetaArray>(
        "agent_layer_meta", rclcpp::QoS(1).reliable().transient_local());
  }
}

void AgentLayer::deactivate()
{
  sub_.reset();
  meta_pub_.reset();
}

void AgentLayer::infosCallback(
  const multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(data_mtx_);
  last_infos_ = msg;
  last_stamp_ = msg->header.stamp;
}

bool AgentLayer::stale(const rclcpp::Time & stamp) const
{
  return (node_shared_->now() - stamp) >
         rclcpp::Duration::from_nanoseconds(
           static_cast<int64_t>(freshness_timeout_ms_) * 1000000LL);
}

bool AgentLayer::isSelf(const multi_agent_msgs::msg::MultiAgentInfo & a) const
{
  return (a.machine_id == self_machine_id_) && (a.type_id == self_type_id_);
}

AgentLayer::CostProfile AgentLayer::profileFor(uint8_t phase) const
{
  using S = multi_agent_msgs::msg::AgentStatus;
  switch (phase) {
    case S::STATUS_ERROR:
    case S::STATUS_WAITING_FOR_SAFETY:
    case S::STATUS_AUTORECOVERY:
    case S::STATUS_RECOVERING:
      return {fp_cost_error_like_, static_cast<unsigned char>(std::max<int>(path_cost_base_, 190)), dilate_error_m_};
    case S::STATUS_ARRIVED:
    case S::STATUS_CHARGING:
    case S::STATUS_CHARGE_DONE:
      return {fp_cost_arrived_like_, static_cast<unsigned char>(std::max<int>(path_cost_base_, 185)), dilate_arrived_m_};
    case S::STATUS_WAITING_FOR_OBS:
    case S::STATUS_PAUSE:
    case S::STATUS_WAITING_FOR_FLOWCONTROL:
    case S::STATUS_MARKING:
    case S::STATUS_LOADING:
    case S::STATUS_UNLOADING:
      return {fp_cost_wait_like_, static_cast<unsigned char>(std::max<int>(path_cost_base_, 180)), dilate_wait_m_};
    case S::STATUS_MOVING:
    case S::STATUS_PATH_SEARCHING:
    case S::STATUS_MANUAL_RUNNING:
      return {fp_cost_moving_like_, path_cost_base_, dilate_moving_curr_m_};
    default:
      return {fp_cost_default_, static_cast<unsigned char>(std::max<int>(path_cost_base_, 170)), 0.15};
  }
}

void AgentLayer::updateBounds(double robot_x, double robot_y, double /*robot_yaw*/,
                              double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_) return;

  touched_ = false;
  touch_min_x_ =  std::numeric_limits<double>::infinity();
  touch_min_y_ =  std::numeric_limits<double>::infinity();
  touch_max_x_ = -std::numeric_limits<double>::infinity();
  touch_max_y_ = -std::numeric_limits<double>::infinity();

  std::vector<multi_agent_msgs::msg::MultiAgentInfo> infos;
  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    if (!last_infos_ || stale(last_stamp_)) return;
    infos.assign(last_infos_->agents.begin(), last_infos_->agents.end());
  }

  const std::string & global_frame = layered_costmap_->getGlobalFrameID();

  for (const auto & a : infos) {
    if (isSelf(a)) continue;

    // ROI by distance from our robot
    const double dx = a.current_pose.pose.position.x - robot_x;
    const double dy = a.current_pose.pose.position.y - robot_y;
    if (hypot2(dx, dy) > roi_range_m_) continue;

    // frame check (optional)
    if (use_path_header_frame_ && a.truncated_path.header.frame_id != global_frame) continue;

    // 후보: current pose + truncated path 포즈들
    const int limit = std::min<int>(a.truncated_path.poses.size(), max_poses_);

    auto addPt = [&](double x, double y){
      if (x < touch_min_x_) touch_min_x_ = x;
      if (y < touch_min_y_) touch_min_y_ = y;
      if (x > touch_max_x_) touch_max_x_ = x;
      if (y > touch_max_y_) touch_max_y_ = y;
      touched_ = true;
    };

    addPt(a.current_pose.pose.position.x, a.current_pose.pose.position.y);
    for (int i = 0; i < limit; ++i) {
      const auto & p = a.truncated_path.poses[i].pose.position;
      addPt(p.x, p.y);
    }
  }

  if (touched_) {
    *min_x = std::min(*min_x, touch_min_x_);
    *min_y = std::min(*min_y, touch_min_y_);
    *max_x = std::max(*max_x, touch_max_x_);
    *max_y = std::max(*max_y, touch_max_y_);
  }
}

void AgentLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                             int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  if (!enabled_) return;

  std::vector<multi_agent_msgs::msg::MultiAgentInfo> infos;
  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    if (!last_infos_ || stale(last_stamp_)) return;
    infos.assign(last_infos_->agents.begin(), last_infos_->agents.end());
  }

  std::vector<std::pair<unsigned int,unsigned int>> meta_hits;
  meta_hits.reserve(512);

  for (const auto & a : infos) {
    if (isSelf(a)) continue;
    rasterizeAgent(a, &master_grid, meta_hits);
  }

  if (publish_meta_ && meta_pub_) {
    multi_agent_msgs::msg::AgentLayerMetaArray arr;
    arr.header.frame_id = layered_costmap_->getGlobalFrameID();
    arr.header.stamp = node_shared_->now();
    for (size_t k = 0; k < meta_hits.size(); k += std::max(1, meta_stride_)) {
      auto [mx, my] = meta_hits[k];
      double wx, wy; master_grid.mapToWorld(mx, my, wx, wy);
      multi_agent_msgs::msg::AgentLayerCellMeta cm;
      cm.header = arr.header;
      cm.machine_id = 0;
      cm.phase = 0;
      cm.mode = "";
      cm.reroute = false;
      cm.re_path_search = false;
      cm.transferring = false;
      cm.area_id = 0;
      cm.position.x = wx; cm.position.y = wy; cm.position.z = 0.0;
      cm.mx = mx; cm.my = my;
      cm.t_first_hit = -1.0f;
      cm.sigma = 0.0f;
      arr.cells.emplace_back(std::move(cm));
    }
    meta_pub_->publish(std::move(arr));
  }
}

static inline std::vector<geometry_msgs::msg::Point>
dilatePolygon(const std::vector<geometry_msgs::msg::Point32> & in, double d)
{
  std::vector<geometry_msgs::msg::Point> out; out.reserve(in.size());
  if (in.empty()) return out;

  // centroid
  double cx=0, cy=0;
  for (auto & p : in) { cx += p.x; cy += p.y; }
  cx /= static_cast<double>(in.size());
  cy /= static_cast<double>(in.size());

  for (auto & p : in) {
    double vx = p.x - cx, vy = p.y - cy;
    double n = std::hypot(vx, vy); if (n < 1e-6) n = 1.0;
    geometry_msgs::msg::Point q;
    q.x = p.x + d * (vx / n);
    q.y = p.y + d * (vy / n);
    q.z = 0.0;
    out.push_back(q);
  }
  return out;
}

inline double AgentLayer::yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return tf2::getYaw(q);
}

void AgentLayer::rasterizeAgent(
  const multi_agent_msgs::msg::MultiAgentInfo & a,
  nav2_costmap_2d::Costmap2D * grid,
  std::vector<std::pair<unsigned int,unsigned int>> & meta_hits)
{
  const auto prof = profileFor(a.status.phase);

  // 1) 현재 footprint를 '치사' 등으로 채움 (상태별 dilation + 위치불확실도)
  double dilation = prof.dilate;
  if (a.pos_std_m >= 0.0) dilation += pos_sigma_k_ * a.pos_std_m;
  fillFootprintAt(a.footprint, a.current_pose.pose, dilation, grid, prof.fp_cost, &meta_hits);

  // 2) 경로 소프트 필드(선택: truncated_path가 있으면)
  if (!a.truncated_path.poses.empty()) {
    // 프레임 일치 검사 (옵션)
    if (use_path_header_frame_) {
      if (a.truncated_path.header.frame_id != layered_costmap_->getGlobalFrameID())
        return;
    }
    // 누적 길이 prefix sum
    const int limit = std::min<int>(a.truncated_path.poses.size(), max_poses_);
    std::vector<geometry_msgs::msg::PoseStamped> path;
    path.reserve(limit);
    for (int i=0; i<limit; ++i) path.push_back(a.truncated_path.poses[i]);

    std::vector<double> prefix_s(limit, 0.0);
    for (int i=1; i<limit; ++i) {
      const auto &p0 = path[i-1].pose.position;
      const auto &p1 = path[i  ].pose.position;
      const double ds = hypot2(p1.x - p0.x, p1.y - p0.y);
      prefix_s[i] = prefix_s[i-1] + ds;
    }
    softenAlongPath(path, grid, prof.base, &prefix_s, &meta_hits);
  }
}

void AgentLayer::fillFootprintAt(const geometry_msgs::msg::PolygonStamped & fp,
                                 const geometry_msgs::msg::Pose & pose,
                                 double extra_dilation_m,
                                 nav2_costmap_2d::Costmap2D * grid,
                                 unsigned char cost,
                                 std::vector<std::pair<unsigned int,unsigned int>> * meta_hits)
{
  // 1) inflate local polygon
  auto poly = dilatePolygon(fp.polygon.points, extra_dilation_m);

  // 2) transform to world by pose
  const double yaw = yawOf(pose.orientation);
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (auto & p : poly) {
    const double x = p.x, y = p.y;
    p.x = pose.position.x + c * x - s * y;
    p.y = pose.position.y + s * x + c * y;
  }

  // 3) bbox in world
  double minx=std::numeric_limits<double>::infinity();
  double miny=std::numeric_limits<double>::infinity();
  double maxx=-std::numeric_limits<double>::infinity();
  double maxy=-std::numeric_limits<double>::infinity();
  for (auto & p : poly) {
    if (p.x < minx) minx = p.x;
    if (p.y < miny) miny = p.y;
    if (p.x > maxx) maxx = p.x;
    if (p.y > maxy) maxy = p.y;
  }

  // 4) bbox→map index
  int min_i, min_j, max_i, max_j;
  grid->worldToMapEnforceBounds(minx, miny, min_i, min_j);
  grid->worldToMapEnforceBounds(maxx, maxy, max_i, max_j);

  // 5) fill by point-in-polygon (ray-casting)
  const size_t n = poly.size();
  for (int j = min_j; j <= max_j; ++j) {
    for (int i = min_i; i <= max_i; ++i) {
      double wx, wy; grid->mapToWorld(i, j, wx, wy);

      bool inside = false;
      for (size_t k=0, h=n-1; k<n; h=k++) {
        const double xi = poly[k].x, yi = poly[k].y;
        const double xh = poly[h].x, yh = poly[h].y;
        const bool hit = ((yi > wy) != (yh > wy)) &&
                         (wx < (xh - xi) * (wy - yi) / std::max(1e-12, (yh - yi)) + xi);
        if (hit) inside = !inside;
      }

      if (inside) {
        // True overwrite: footprint은 치사 성격이므로 그대로 덮어씀(또는 Max-merge도 OK)
        grid->setCost(static_cast<unsigned int>(i),
                      static_cast<unsigned int>(j), cost);
        if (meta_hits) meta_hits->emplace_back(
            static_cast<unsigned int>(i), static_cast<unsigned int>(j));
      }
    }
  }

  // 6) bounds bookkeeping for updateBounds
  if (!touched_) {
    touch_min_x_ = minx; touch_min_y_ = miny;
    touch_max_x_ = maxx; touch_max_y_ = maxy;
    touched_ = true;
  } else {
    touch_min_x_ = std::min(touch_min_x_, minx);
    touch_min_y_ = std::min(touch_min_y_, miny);
    touch_max_x_ = std::max(touch_max_x_, maxx);
    touch_max_y_ = std::max(touch_max_y_, maxy);
  }
}

// 경로 소프트 필드: 각 포즈 주변 반지름 R ≈ 3*sigma 내 셀만 스캔, 가우시안 * 종방향 감쇠 * (옵션)헤딩콘버프
void AgentLayer::softenAlongPath(const std::vector<geometry_msgs::msg::PoseStamped> & path,
                                 nav2_costmap_2d::Costmap2D * grid,
                                 unsigned char c_base,
                                 std::vector<double> * prefix_s,
                                 std::vector<std::pair<unsigned int,unsigned int>> * meta_hits)
{
  if (path.empty()) return;

  const double R = std::max(1.0, 3.0 * sigma_lat_m_); // 검색 반경(m)
  const double R2 = R * R;
  const double inv_2sigma2 = (sigma_lat_m_ > 1e-6) ? (1.0 / (2.0 * sigma_lat_m_ * sigma_lat_m_)) : 1e6;
  const double cone_rad = heading_cone_deg_ * M_PI / 180.0;

  // 셀 크기
  const double res = grid->getResolution();

  const size_t N = path.size();
  for (size_t idx = 0; idx < N; ++idx) {
    const auto & ps = path[idx].pose;
    const double x = ps.position.x, y = ps.position.y;
    const double yaw = yawOf(ps.orientation);
    const double cos_y = std::cos(yaw), sin_y = std::sin(yaw);

    // 종방향 감쇠 w_long(s)
    const double s = (*prefix_s)[idx];
    const double wlong = (lambda_m_ > 1e-6) ? std::exp(-s / lambda_m_) : 1.0;

    // 포즈 중심을 포함하는 맵 인덱스 박스
    double minx = x - R, miny = y - R, maxx = x + R, maxy = y + R;
    int min_i, min_j, max_i, max_j;
    grid->worldToMapEnforceBounds(minx, miny, min_i, min_j);
    grid->worldToMapEnforceBounds(maxx, maxy, max_i, max_j);

    for (int j = min_j; j <= max_j; ++j) {
      for (int i = min_i; i <= max_i; ++i) {
        double wx, wy; grid->mapToWorld(i, j, wx, wy);
        const double dx = wx - x, dy = wy - y;
        const double r2 = dx*dx + dy*dy;
        if (r2 > R2) continue; // 원 반경 바깥은 skip

        // 횡방향 거리 근사: 로컬 프레임으로 회전 → |y_local|
        const double x_local =  cos_y * dx + sin_y * dy;
        const double y_local = -sin_y * dx + cos_y * dy;
        const double d = std::fabs(y_local);

        // 가우시안 횡감쇠
        const double wlat = std::exp( - (d*d) * inv_2sigma2 );

        // 헤딩 콘(앞쪽만 가산)
        int cone = 0;
        if (cone_boost_ > 0) {
          // 셀 벡터의 각도(로컬 x축 기준)
          const double theta = std::atan2(y_local, x_local);
          if (std::fabs(theta) <= cone_rad && x_local >= 0.0) {
            cone = cone_boost_;
          }
        }

        // 최종 소프트 코스트
        int c_soft = static_cast<int>(c_base) +
                     static_cast<int>(std::round(wlong * wlat * static_cast<double>(path_soft_cmax_))) +
                     cone;

        c_soft = std::clamp(c_soft, static_cast<int>(c_base), static_cast<int>(path_cost_cap_));

        // Max-merge
        const unsigned char old = grid->getCost(i, j);
        const unsigned char neu = static_cast<unsigned char>(std::max<int>(old, c_soft));
        if (neu != old) {
          grid->setCost(static_cast<unsigned int>(i),
                        static_cast<unsigned int>(j), neu);
          if (meta_hits) meta_hits->emplace_back(
              static_cast<unsigned int>(i), static_cast<unsigned int>(j));
        }
      }
    }

    // bounds bookkeeping (updateBounds에서 전체 경로 bbox를 이미 포함하지만 보수적으로 갱신)
    touch_min_x_ = std::min(touch_min_x_, minx);
    touch_min_y_ = std::min(touch_min_y_, miny);
    touch_max_x_ = std::max(touch_max_x_, maxx);
    touch_max_y_ = std::max(touch_max_y_, maxy);
    touched_ = true;
  }
}

} // namespace multi_agent_nav2

// pluginlib export
PLUGINLIB_EXPORT_CLASS(multi_agent_nav2::AgentLayer, nav2_costmap_2d::Layer)
```

---

## 사용/튜닝 포인트

* YAML 예시:

```yaml
global_costmap:
  global_costmap:
    plugins: ["static_layer", "obstacle_layer", "agent_layer"]
    agent_layer:
      plugin: multi_agent_nav2::AgentLayer
      enabled: true
      topic: /multi_agent_infos
      roi_range_m: 12.0
      freshness_timeout_ms: 800
      max_poses: 40
      qos_reliable: true

      # footprint 강도
      fp_cost_wait_like: 254
      fp_cost_arrived_like: 254
      fp_cost_error_like: 254
      fp_cost_moving_like: 254
      fp_cost_default: 240

      # 경로 소프트 필드
      path_cost_base: 170
      path_cost_cap: 253
      path_soft_cmax: 110
      lambda_m: 2.0
      sigma_lat_m: 0.45

      # 상태별 dilation + 불확실도 가산
      dilate_wait_m: 0.2
      dilate_arrived_m: 0.35
      dilate_error_m: 0.5
      dilate_moving_curr_m: 0.1
      pos_sigma_k: 2.0

      # 진행방향 콘
      heading_cone_deg: 40.0
      cone_boost: 40
```

* **경로를 ‘뚫고 간다’**면: `path_cost_base`↑, `path_soft_cmax`↑, `lambda_m`↑, `sigma_lat_m`↑, `cone_boost`↑ 순으로 강화.
* **너무 우회**하면: 반대로 낮추거나 `max_poses`/경로간격을 줄여 필드를 희박화.

필요하면 이 코드에 맞춘 **작은 벤치마크/가시화(셀별 색상 다른 팔레트)**도 곧바로 붙여드릴게요.
##################  
###################  
#########################  
좋지! 요청하신 “변경점 포함 완전체”를 hpp / cpp로 깔끔하게 정리해서 드립니다. 그대로 덮어쓰고 빌드하시면 됩니다.

---

### `include/multi_agent_nav2/agent_layer.hpp`

```cpp
#pragma once

#include <mutex>
#include <vector>
#include <string>
#include <utility>
#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include <multi_agent_msgs/msg/multi_agent_info_array.hpp>
#include <multi_agent_msgs/msg/agent_layer_meta_array.hpp>

namespace multi_agent_nav2
{

class AgentLayer : public nav2_costmap_2d::Layer
{
public:
  AgentLayer();

  // lifecycle
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override { current_ = true; }

  // costmap callbacks
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;

  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;

  bool isClearable() override { return true; }

private:
  // node handle (lock base weak_ptr)
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_shared_;

  // I/O
  rclcpp::Subscription<multi_agent_msgs::msg::MultiAgentInfoArray>::SharedPtr sub_;
  rclcpp::Publisher<multi_agent_msgs::msg::AgentLayerMetaArray>::SharedPtr meta_pub_;

  // last data
  std::mutex data_mtx_;
  multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr last_infos_;
  rclcpp::Time last_stamp_;

  // parameters (기본 동작)
  bool        enabled_{true};
  std::string topic_{"/multi_agent_infos"};
  uint16_t    self_machine_id_{0};
  std::string self_type_id_{};
  bool        use_path_header_frame_{true};
  double      roi_range_m_{12.0};
  double      time_decay_sec_{1.0}; // reserved
  unsigned char lethal_cost_{254};
  unsigned char moving_cost_{180};
  unsigned char waiting_cost_{200};
  int         manual_cost_bias_{30};
  double      dilation_m_{0.05};       // footprint 기본 팽창(안전 여유)
  double      forward_smear_m_{0.25};  // 이동중 footprint 전방 여유
  double      sigma_k_{2.0};           // 위치 표준편차 가중
  bool        publish_meta_{true};
  int         meta_stride_{3};
  int         freshness_timeout_ms_{800};
  int         max_poses_{40};
  bool        qos_reliable_{true};

  // 경로 소프트필드 튜닝(이번 수정 핵심)
  bool   soft_path_only_when_moving_{true}; // 이동 중일 때만 경로 코스트 생성
  double path_sigma_lat_m_{0.25};           // 경로 횡방향 표준편차(좁게)
  double path_lambda_long_m_{1.5};          // 경로 진행방향 감쇠 길이(짧게)
  int    path_cost_base_{180};              // 경로 코스트 바닥값(약하게)
  int    path_cost_cap_{230};               // 경로 코스트 상한(LETHAL 미만)
  double path_cone_boost_{0.0};             // 전방 콘 부스트(기본 0 → 비활성)

  // bounds cache for this cycle
  double touch_min_x_{0.0}, touch_min_y_{0.0}, touch_max_x_{0.0}, touch_max_y_{0.0};
  bool   touched_{false};

  // helpers
  void infosCallback(const multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr msg);
  bool isSelf(const multi_agent_msgs::msg::MultiAgentInfo & a) const;
  bool stale(const rclcpp::Time & stamp) const;

  void rasterizeAgentPath(const multi_agent_msgs::msg::MultiAgentInfo & a,
                          nav2_costmap_2d::Costmap2D * grid,
                          std::vector<std::pair<unsigned int,unsigned int>> & meta_hits);

  void fillFootprintAt(const geometry_msgs::msg::PolygonStamped & fp,
                       const geometry_msgs::msg::Pose & pose,
                       double extra_dilation_m,
                       nav2_costmap_2d::Costmap2D * grid,
                       unsigned char cost,
                       std::vector<std::pair<unsigned int,unsigned int>> * meta_hits = nullptr);

  static bool pointInPolygon(const std::vector<geometry_msgs::msg::Point> & poly,
                             double x, double y);

  unsigned char computeCost(const multi_agent_msgs::msg::MultiAgentInfo & a) const;
  double computeDilation(const multi_agent_msgs::msg::MultiAgentInfo & a) const;

  static inline bool isMovingPhase(uint8_t phase);
};

} // namespace multi_agent_nav2
```

---

### `src/agent_layer.cpp`

```cpp
#include "multi_agent_nav2/agent_layer.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>

namespace multi_agent_nav2
{

AgentLayer::AgentLayer() {}

void AgentLayer::onInitialize()
{
  node_shared_ = node_.lock();
  if (!node_shared_) {
    throw std::runtime_error("AgentLayer: failed to lock lifecycle node");
  }

  // Declare parameters (defaults)
  node_shared_->declare_parameter("enabled", rclcpp::ParameterValue(true));
  node_shared_->declare_parameter("topic", rclcpp::ParameterValue(std::string("/multi_agent_infos")));
  node_shared_->declare_parameter("self_machine_id", rclcpp::ParameterValue(0));
  node_shared_->declare_parameter("self_type_id", rclcpp::ParameterValue(std::string("")));
  node_shared_->declare_parameter("use_path_header_frame", rclcpp::ParameterValue(true));
  node_shared_->declare_parameter("roi_range_m", rclcpp::ParameterValue(12.0));
  node_shared_->declare_parameter("time_decay_sec", rclcpp::ParameterValue(1.0));
  node_shared_->declare_parameter("lethal_cost", rclcpp::ParameterValue(254));
  node_shared_->declare_parameter("moving_cost", rclcpp::ParameterValue(180));
  node_shared_->declare_parameter("waiting_cost", rclcpp::ParameterValue(200));
  node_shared_->declare_parameter("manual_cost_bias", rclcpp::ParameterValue(30));
  node_shared_->declare_parameter("dilation_m", rclcpp::ParameterValue(0.05));
  node_shared_->declare_parameter("forward_smear_m", rclcpp::ParameterValue(0.25));
  node_shared_->declare_parameter("sigma_k", rclcpp::ParameterValue(2.0));
  node_shared_->declare_parameter("publish_meta", rclcpp::ParameterValue(true));
  node_shared_->declare_parameter("meta_stride", rclcpp::ParameterValue(3));
  node_shared_->declare_parameter("freshness_timeout_ms", rclcpp::ParameterValue(800));
  node_shared_->declare_parameter("max_poses", rclcpp::ParameterValue(40));
  node_shared_->declare_parameter("qos_reliable", rclcpp::ParameterValue(true));

  // Path soft-field params (new)
  node_shared_->declare_parameter("soft_path_only_when_moving", rclcpp::ParameterValue(true));
  node_shared_->declare_parameter("path_sigma_lat_m", rclcpp::ParameterValue(0.25));
  node_shared_->declare_parameter("path_lambda_long_m", rclcpp::ParameterValue(1.5));
  node_shared_->declare_parameter("path_cost_base", rclcpp::ParameterValue(180));
  node_shared_->declare_parameter("path_cost_cap",  rclcpp::ParameterValue(230));
  node_shared_->declare_parameter("path_cone_boost", rclcpp::ParameterValue(0.0));

  // Get parameters
  node_shared_->get_parameter("enabled", enabled_);
  node_shared_->get_parameter("topic", topic_);
  {
    int tmp = 0;
    node_shared_->get_parameter("self_machine_id", tmp);
    self_machine_id_ = static_cast<uint16_t>(tmp);
  }
  node_shared_->get_parameter("self_type_id", self_type_id_);
  node_shared_->get_parameter("use_path_header_frame", use_path_header_frame_);
  node_shared_->get_parameter("roi_range_m", roi_range_m_);
  node_shared_->get_parameter("time_decay_sec", time_decay_sec_);
  {
    int tmp = 254; node_shared_->get_parameter("lethal_cost", tmp);
    lethal_cost_ = static_cast<unsigned char>(std::clamp(tmp, 0, 254));
  }
  {
    int tmp = 180; node_shared_->get_parameter("moving_cost", tmp);
    moving_cost_ = static_cast<unsigned char>(std::clamp(tmp, 0, 254));
  }
  {
    int tmp = 200; node_shared_->get_parameter("waiting_cost", tmp);
    waiting_cost_ = static_cast<unsigned char>(std::clamp(tmp, 0, 254));
  }
  node_shared_->get_parameter("manual_cost_bias", manual_cost_bias_);
  node_shared_->get_parameter("dilation_m", dilation_m_);
  node_shared_->get_parameter("forward_smear_m", forward_smear_m_);
  node_shared_->get_parameter("sigma_k", sigma_k_);
  node_shared_->get_parameter("publish_meta", publish_meta_);
  node_shared_->get_parameter("meta_stride", meta_stride_);
  node_shared_->get_parameter("freshness_timeout_ms", freshness_timeout_ms_);
  node_shared_->get_parameter("max_poses", max_poses_);
  node_shared_->get_parameter("qos_reliable", qos_reliable_);

  node_shared_->get_parameter("soft_path_only_when_moving", soft_path_only_when_moving_);
  node_shared_->get_parameter("path_sigma_lat_m", path_sigma_lat_m_);
  node_shared_->get_parameter("path_lambda_long_m", path_lambda_long_m_);
  node_shared_->get_parameter("path_cost_base", path_cost_base_);
  node_shared_->get_parameter("path_cost_cap",  path_cost_cap_);
  node_shared_->get_parameter("path_cone_boost", path_cone_boost_);

  current_ = true;
  matchSize();

  if (enabled_) activate();
}

void AgentLayer::activate()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  if (qos_reliable_) qos.reliable(); else qos.best_effort();

  sub_ = node_shared_->create_subscription<multi_agent_msgs::msg::MultiAgentInfoArray>(
      topic_, qos, std::bind(&AgentLayer::infosCallback, this, std::placeholders::_1));

  if (publish_meta_) {
    meta_pub_ = node_shared_->create_publisher<multi_agent_msgs::msg::AgentLayerMetaArray>(
        "agent_layer_meta", rclcpp::QoS(1).reliable().transient_local());
  }
}

void AgentLayer::deactivate()
{
  sub_.reset();
  meta_pub_.reset();
}

void AgentLayer::infosCallback(
  const multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(data_mtx_);
  last_infos_ = msg;
  last_stamp_ = msg->header.stamp;
}

bool AgentLayer::stale(const rclcpp::Time & stamp) const
{
  return (node_shared_->now() - stamp) >
         rclcpp::Duration::from_nanoseconds(
           static_cast<int64_t>(freshness_timeout_ms_) * 1000000LL);
}

bool AgentLayer::isSelf(const multi_agent_msgs::msg::MultiAgentInfo & a) const
{
  return (a.machine_id == self_machine_id_) && (a.type_id == self_type_id_);
}

unsigned char AgentLayer::computeCost(const multi_agent_msgs::msg::MultiAgentInfo & a) const
{
  using S = multi_agent_msgs::msg::AgentStatus;
  const uint8_t p = a.status.phase;
  const bool is_moving =
      (p == S::STATUS_MOVING) || (p == S::STATUS_PATH_SEARCHING);

  unsigned char base = is_moving ? moving_cost_ : waiting_cost_;

  if (a.mode == "manual") {
    int c = static_cast<int>(base) + manual_cost_bias_;
    return static_cast<unsigned char>(std::clamp(c, 0, 254));
  }
  return base;
}

bool inline AgentLayer::isMovingPhase(uint8_t phase)
{
  using S = multi_agent_msgs::msg::AgentStatus;
  return (phase == S::STATUS_MOVING) || (phase == S::STATUS_PATH_SEARCHING);
}

double AgentLayer::computeDilation(const multi_agent_msgs::msg::MultiAgentInfo & a) const
{
  double r = dilation_m_;
  // 위치 불확실도 반영
  if (a.pos_std_m >= 0.0) r += sigma_k_ * a.pos_std_m;
  // 진행방향 스미어: 이동중일 때만
  if (forward_smear_m_ > 0.0 && isMovingPhase(a.status.phase)) {
    r += forward_smear_m_;
  }
  return r;
}

void AgentLayer::updateBounds(double robot_x, double robot_y, double /*robot_yaw*/,
                              double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_) return;

  touched_ = false;
  touch_min_x_ =  1e9; touch_min_y_ =  1e9;
  touch_max_x_ = -1e9; touch_max_y_ = -1e9;

  std::vector<multi_agent_msgs::msg::MultiAgentInfo> infos;
  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    if (!last_infos_ || stale(last_stamp_)) return;
    infos.assign(last_infos_->agents.begin(), last_infos_->agents.end());
  }

  const std::string & global_frame = layered_costmap_->getGlobalFrameID();

  for (const auto & a : infos) {
    if (isSelf(a)) continue;

    // ROI by distance from our robot
    const double dx = a.current_pose.pose.position.x - robot_x;
    const double dy = a.current_pose.pose.position.y - robot_y;
    if (std::hypot(dx, dy) > roi_range_m_) continue;

    // frame check (optional)
    if (use_path_header_frame_ && a.truncated_path.header.frame_id != global_frame) {
      continue;
    }

    // 현재 pose와 truncated path 둘 다 bounds 반영
    const auto consider_point = [&](double x, double y){
      if (x < touch_min_x_) touch_min_x_ = x;
      if (y < touch_min_y_) touch_min_y_ = y;
      if (x > touch_max_x_) touch_max_x_ = x;
      if (y > touch_max_y_) touch_max_y_ = y;
      touched_ = true;
    };

    consider_point(a.current_pose.pose.position.x, a.current_pose.pose.position.y);

    const int limit = std::min<int>(a.truncated_path.poses.size(), max_poses_);
    for (int i = 0; i < limit; ++i) {
      const auto & p = a.truncated_path.poses[i].pose.position;
      consider_point(p.x, p.y);
    }
  }

  if (touched_) {
    *min_x = std::min(*min_x, touch_min_x_);
    *min_y = std::min(*min_y, touch_min_y_);
    *max_x = std::max(*max_x, touch_max_x_);
    *max_y = std::max(*max_y, touch_max_y_);
  }
}

void AgentLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                             int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  if (!enabled_) return;

  std::vector<multi_agent_msgs::msg::MultiAgentInfo> infos;
  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    if (!last_infos_ || stale(last_stamp_)) return;
    infos.assign(last_infos_->agents.begin(), last_infos_->agents.end());
  }

  std::vector<std::pair<unsigned int,unsigned int>> meta_hits;
  meta_hits.reserve(256);

  for (const auto & a : infos) {
    if (isSelf(a)) continue;
    rasterizeAgentPath(a, &master_grid, meta_hits);
  }

  if (publish_meta_ && meta_pub_) {
    multi_agent_msgs::msg::AgentLayerMetaArray arr;
    arr.header.frame_id = layered_costmap_->getGlobalFrameID();
    arr.header.stamp = node_shared_->now();

    for (size_t k = 0; k < meta_hits.size(); k += std::max(1, meta_stride_)) {
      auto [mx, my] = meta_hits[k];
      double wx, wy; master_grid.mapToWorld(mx, my, wx, wy);
      multi_agent_msgs::msg::AgentLayerCellMeta cm;
      cm.header = arr.header;

      // (간략 메타; 필요시 agent id 등 확장 가능)
      cm.machine_id = 0;
      cm.phase = 0;
      cm.mode = "";
      cm.reroute = false;
      cm.re_path_search = false;
      cm.transferring = false;
      cm.area_id = 0;

      cm.position.x = wx; cm.position.y = wy; cm.position.z = 0.0;
      cm.mx = mx; cm.my = my;

      cm.t_first_hit = -1.0f;
      cm.sigma = 0.0f;

      arr.cells.emplace_back(std::move(cm));
    }
    meta_pub_->publish(std::move(arr));
  }
}

static inline std::vector<geometry_msgs::msg::Point>
dilatePolygon(const std::vector<geometry_msgs::msg::Point32> & in, double d)
{
  std::vector<geometry_msgs::msg::Point> out; out.reserve(in.size());
  if (in.empty()) return out;

  // centroid
  double cx=0, cy=0;
  for (auto & p : in) { cx += p.x; cy += p.y; }
  cx /= static_cast<double>(in.size());
  cy /= static_cast<double>(in.size());

  for (auto & p : in) {
    double vx = p.x - cx, vy = p.y - cy;
    double n = std::hypot(vx, vy); if (n < 1e-6) n = 1.0;
    geometry_msgs::msg::Point q;
    q.x = p.x + d * (vx / n);
    q.y = p.y + d * (vy / n);
    q.z = 0.0;
    out.push_back(q);
  }
  return out;
}

void AgentLayer::rasterizeAgentPath(
  const multi_agent_msgs::msg::MultiAgentInfo & a,
  nav2_costmap_2d::Costmap2D * grid,
  std::vector<std::pair<unsigned int,unsigned int>> & meta_hits)
{
  // 1) 현재 footprint는 항상 표기 (정지/이동 공통)
  const unsigned char cost_now = computeCost(a);
  const double extra_now = computeDilation(a);
  fillFootprintAt(a.footprint, a.current_pose.pose, extra_now, grid, cost_now, &meta_hits);

  // 2) 경로 소프트필드는 "이동 중"일 때만 적용(옵션)
  if (soft_path_only_when_moving_ && !isMovingPhase(a.status.phase)) {
    // 정지 로봇은 footprint만. 콘/경로 소프트필드 생성 안 함.
    return;
  }

  if (a.truncated_path.poses.empty()) return;

  // === 경로 소프트 코스트 쓰기 ===
  const int limit = std::min<int>(a.truncated_path.poses.size(), max_poses_);
  for (int i = 0; i < limit; ++i) {
    const auto & ps = a.truncated_path.poses[i].pose;

    const double cx = ps.position.x;
    const double cy = ps.position.y;
    const double yaw = tf2::getYaw(ps.orientation);
    const double c = std::cos(yaw), s = std::sin(yaw);

    // 영향 반경(3시그마) → 좁은 띠
    const double r = 3.0 * path_sigma_lat_m_;

    // bbox(world)
    const double minx = cx - r, maxx = cx + r;
    const double miny = cy - r, maxy = cy + r;

    int min_i, min_j, max_i, max_j;
    grid->worldToMapEnforceBounds(minx, miny, min_i, min_j);
    grid->worldToMapEnforceBounds(maxx, maxy, max_i, max_j);

    for (int j = min_j; j <= max_j; ++j) {
      for (int ii = min_i; ii <= max_i; ++ii) {
        double wx, wy; grid->mapToWorld(ii, j, wx, wy);

        // 경로 좌표계로 회전하여 횡/종 분해
        const double dx = wx - cx;
        const double dy = wy - cy;
        const double x_local =  c * dx + s * dy; // 진행방향
        const double y_local = -s * dx + c * dy; // 횡방향

        // 횡방향 가우시안, 종방향 지수감쇠
        const double g_lat  = std::exp(-(y_local * y_local) / (2.0 * path_sigma_lat_m_ * path_sigma_lat_m_));
        const double g_long = std::exp(-std::max(0.0, std::abs(x_local)) / std::max(1e-6, path_lambda_long_m_));

        // 앞쪽 콘 부스팅(기본 0 → 비활성)
        double cone = 1.0;
        if (path_cone_boost_ > 1e-6 && x_local > 0.0) {
          cone += path_cone_boost_ * std::exp(-x_local / std::max(1e-6, path_lambda_long_m_));
        }

        int raw = static_cast<int>(std::round(path_cost_base_ * g_lat * g_long * cone));
        raw = std::clamp(raw, 0, path_cost_cap_);
        if (raw <= 0) continue;

        const unsigned char cand = static_cast<unsigned char>(raw);

        // Max-merge with NO_INFORMATION(255) 보정
        const unsigned char old_raw = grid->getCost(ii, j);
        const int old = (old_raw == nav2_costmap_2d::NO_INFORMATION) ? 0 : static_cast<int>(old_raw);
        if (static_cast<int>(cand) > old) {
          grid->setCost(static_cast<unsigned int>(ii),
                        static_cast<unsigned int>(j), cand);
          meta_hits.emplace_back(static_cast<unsigned int>(ii), static_cast<unsigned int>(j));
        }
      }
    }
  }
}

void AgentLayer::fillFootprintAt(const geometry_msgs::msg::PolygonStamped & fp,
                                 const geometry_msgs::msg::Pose & pose,
                                 double extra_dilation_m,
                                 nav2_costmap_2d::Costmap2D * grid,
                                 unsigned char cost,
                                 std::vector<std::pair<unsigned int,unsigned int>> * meta_hits)
{
  // 1) inflate local polygon
  auto poly = dilatePolygon(fp.polygon.points, extra_dilation_m);

  // 2) transform to world by pose
  const double yaw = tf2::getYaw(pose.orientation);
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (auto & p : poly) {
    const double x = p.x, y = p.y;
    p.x = pose.position.x + c * x - s * y;
    p.y = pose.position.y + s * x + c * y;
  }

  // 3) bbox in world
  double minx=1e9, miny=1e9, maxx=-1e9, maxy=-1e9;
  for (auto & p : poly) {
    if (p.x < minx) minx = p.x;
    if (p.y < miny) miny = p.y;
    if (p.x > maxx) maxx = p.x;
    if (p.y > maxy) maxy = p.y;
  }

  // 4) bbox→map index (enforce bounds uses int&)
  int min_i, min_j, max_i, max_j;
  grid->worldToMapEnforceBounds(minx, miny, min_i, min_j);
  grid->worldToMapEnforceBounds(maxx, maxy, max_i, max_j);

  // 5) fill by point-in-polygon
  for (int j = min_j; j <= max_j; ++j) {
    for (int i = min_i; i <= max_i; ++i) {
      double wx, wy; grid->mapToWorld(i, j, wx, wy);

      // ray-casting
      bool inside = false;
      const size_t n = poly.size();
      for (size_t k=0, h=n-1; k<n; h=k++) {
        const double xi = poly[k].x, yi = poly[k].y;
        const double xh = poly[h].x, yh = poly[h].y;
        const bool hit = ((yi > wy) != (yh > wy)) &&
                         (wx < (xh - xi) * (wy - yi) / std::max(1e-12, (yh - yi)) + xi);
        if (hit) inside = !inside;
      }

      if (inside) {
        // 치사 또는 강한 footprint 코스트: TrueOverwrite(footprint 우선) 느낌으로 덮되
        // Max-merge로도 충분히 강함
        const unsigned char old_raw = grid->getCost(i, j);
        const int old = (old_raw == nav2_costmap_2d::NO_INFORMATION) ? 0 : static_cast<int>(old_raw);
        const int cand = static_cast<int>(cost);
        if (cand > old) {
          grid->setCost(static_cast<unsigned int>(i),
                        static_cast<unsigned int>(j), cost);
        }
        if (meta_hits) meta_hits->emplace_back(
            static_cast<unsigned int>(i), static_cast<unsigned int>(j));
      }
    }
  }

  // 6) bounds bookkeeping for updateBounds
  if (!touched_) {
    touch_min_x_ = minx; touch_min_y_ = miny;
    touch_max_x_ = maxx; touch_max_y_ = maxy;
    touched_ = true;
  } else {
    touch_min_x_ = std::min(touch_min_x_, minx);
    touch_min_y_ = std::min(touch_min_y_, miny);
    touch_max_x_ = std::max(touch_max_x_, maxx);
    touch_max_y_ = std::max(touch_max_y_, maxy);
  }
}

bool AgentLayer::pointInPolygon(const std::vector<geometry_msgs::msg::Point> & poly,
                                double x, double y)
{
  bool inside = false;
  const size_t n = poly.size();
  for (size_t i=0, j=n-1; i<n; j=i++) {
    const double xi = poly[i].x, yi = poly[i].y;
    const double xj = poly[j].x, yj = poly[j].y;
    const bool hit = ((yi > y) != (yj > y)) &&
                     (x < (xj - xi) * (y - yi) / std::max(1e-12, (yj - yi)) + xi);
    if (hit) inside = !inside;
  }
  return inside;
}

} // namespace multi_agent_nav2

// pluginlib export (cpp 마지막에 단 한 번)
PLUGINLIB_EXPORT_CLASS(multi_agent_nav2::AgentLayer, nav2_costmap_2d::Layer)
```

---

### 메모 (중요 포인트)

* **정지 로봇**: footprint만(치사) 표기. 콘/경로 소프트필드는 생성하지 않도록 분기했습니다.
* **이동 로봇**: truncated_path를 따라 **좁고 약한 소프트 코스트**를 생성(가우시안 띠 + 짧은 종방향 감쇠).
* **NO_INFORMATION(255) 보정**: `getCost()`가 255일 때 0으로 간주하고 Max-merge 비교하여 시각화 누락을 방지.
* **전방 콘**: 기본 `path_cone_boost=0.0`으로 비활성(원하시면 아주 작은 값으로만 사용 권장).
* **forward_smear_m_**: **이동 중일 때만** footprint 팽창에 추가되도록 유지(정지 시 콘/꼬리 방지).
* `agent_layer`는 **플러그인 목록의 마지막**에 두는 걸 권장(다른 레이어에 의해 덮이지 않게).
* 파라미터 오버라이드는 `nav2_params.yaml`에서 반드시 같은 네임스페이스에 두세요(이전 이슈 원인).

필요하면 `nav2_params.yaml`의 기본값(폭/세기/길이)을 더 낮춰서 더 얌전하게 만들 수 있어요.
