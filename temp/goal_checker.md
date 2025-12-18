Nav2에서 제공하는 3가지 Goal Checker 플러그인은 **"로봇이 목표 지점에 도착했다고 판단하는 기준"**이 서로 다릅니다.

제공해주신 코드를 바탕으로 각각의 작동 방식과 핵심 차이점을 분석해 드립니다.

---

### 1. 요약 비교 (Comparison Table)

| 특성 | **SimpleGoalChecker** | **StoppedGoalChecker** | **PositionGoalChecker** |
| --- | --- | --- | --- |
| **확인 항목** | 위치(XY) + 각도(Yaw) | 위치(XY) + 각도(Yaw) + **속도(Velocity)** | **위치(XY)만 확인** |
| **각도(Yaw) 검사** | O (필수) | O (필수) | **X (무시함)** |
| **정지 여부 검사** | X (지나가도 성공 가능) | **O (완전히 멈춰야 성공)** | X |
| **Stateful (Latch)** | O (XY 만족 후엔 각도만 봄) | O (Simple 상속) | O (한 번 닿으면 계속 성공) |
| **주 사용처** | 일반적인 목적지 도착 | 정밀 주차, 작업 전 정지 | 경유지(Waypoint), 회전 불필요 시 |

---

### 2. 상세 분석

#### 1) `SimpleGoalChecker` (가장 기본적이고 범용적)

가장 표준적인 검사기입니다. **"지정된 위치 근처에 가서, 지정된 방향을 바라보면 성공"**으로 간주합니다.

* **체크 기준:**
1. **위치(XY):** 현재 위치가 목표점 반경(`xy_goal_tolerance`) 내에 있는가?
2. **각도(Yaw):** 현재 헤딩이 목표 헤딩(`yaw_goal_tolerance`) 내에 있는가?


* **Stateful (중요):**
* 이 옵션이 `true`이면, 로봇이 한 번 **XY 위치 범위** 안에 들어오는 순간 **"위치 검사 완료(XY Check Passed)"** 상태로 고정(Latch)됩니다.
* 이후 제자리 회전(Final Rotation) 중에 로봇이 미세하게 밀려서 XY 범위를 살짝 벗어나더라도, **위치 오차는 무시하고 각도(Yaw)만 맞으면 성공**으로 처리합니다. (사용자님이 겪으신 진동 문제를 완화하는 핵심 기능입니다.)



#### 2) `StoppedGoalChecker` (완전 정지 보장)

`SimpleGoalChecker`를 상속받아 기능을 확장한 클래스입니다. **"위치랑 각도도 맞아야 하고, 로봇이 완전히 멈춰 있어야 성공"**으로 간주합니다.

* **체크 기준:**
1. `SimpleGoalChecker`의 조건(XY + Yaw)을 모두 만족해야 함.
2. **추가 조건:** 현재 로봇의 선속도와 각속도가 설정된 `stopped_velocity` 이하로 떨어져야 함.


* **특징:**
* 로봇이 목표 지점을 스쳐 지나가는 것을 "성공"으로 보지 않습니다.
* **진동 문제 시 주의:** 만약 제어기가 목표 지점에서 미세하게 떨리며(Oscillation) 속도가 0이 되지 않는다면, 이 체커는 영원히 `Goal Reached`를 띄우지 않을 수 있습니다.



#### 3) `PositionGoalChecker` (위치만 중요할 때)

매우 단순화된 검사기입니다. **"방향은 상관없고, 저 위치에 발만 담그면 성공"**으로 간주합니다.

* **체크 기준:**
1. **위치(XY):** 오직 XY 좌표가 오차 범위(`xy_goal_tolerance`) 내에 있는지만 봅니다.
2. **각도(Yaw):** **검사하지 않습니다.** (코드상 `vel_tolerance.angular.z` 등을 무효화 처리함)


* **Stateful:**
* 한 번이라도 해당 위치를 스치면 `position_reached_ = true`가 되어, 이후 로봇이 어디로 가든 계속 성공 상태를 유지합니다(Reset 전까지).


* **사용처:**
* 최종 목적지가 아닌 **경유지(Waypoint)**를 지나갈 때 유용합니다. (멈추거나 회전하지 않고 부드럽게 지나감)
* 전방향 로봇(Omni-directional)이 아닌 경우, 굳이 최종 자세를 맞출 필요가 없는 단순 이동 명령에 적합합니다.



---

### 3. 사용자님 상황에 비춰본 결론

사용자님은 **"정밀한 제자리 회전 및 주차"**를 테스트하고 계시므로:

1. **`PositionGoalChecker`**: **부적합합니다.** Yaw(헤딩)를 맞추지 않아도 성공해버리기 때문에, 1도 차이 회전 테스트 자체가 불가능합니다.
2. **`StoppedGoalChecker`**: **가장 엄격합니다.** 로봇이 목표에 도착해서 진동(Oscillation)하고 있다면, 속도가 0이 아니므로 성공 판정이 절대 나지 않습니다. 디버깅에는 좋지만, 제어기가 불안정하면 영원히 끝나지 않을 수 있습니다.
3. **`SimpleGoalChecker`**: **가장 적합합니다.** 특히 `stateful: true` 옵션 덕분에, "위치 도착 -> 제자리 회전 중 위치 밀림 -> 그래도 각도만 맞으면 성공"이라는 로직이 가능하여 현재 상황에서 가장 안정적인 결과를 줄 것입니다.

**추천:** 현재 `SimpleGoalChecker`를 사용 중이신 것으로 보이며, 앞서 제안드린 **Graceful Controller 코드 수정(XY Latch 강화)**과 함께 사용하면 가장 좋은 퍼포먼스를 낼 것입니다.
#####################  
######################  
#####################  
Nav2 스택에서 **Controller Server**, **Goal Checker**, **Controller (Graceful Controller)** 세 가지 요소가 어떻게 얽혀서 돌아가는지, 그 **상호작용의 흐름(Interaction Flow)**을 코드 레벨과 로직 레벨에서 명확히 설명해 드리겠습니다.

핵심은 **"Controller Server가 중재자(Mediator)이고, Graceful Controller는 Goal Checker의 '값'만 빌려 쓴다"**는 점입니다.

---

### 1. 전체 구조 (Big Picture)

이들은 수직적인 **계층 구조**를 가집니다.

1. **Boss (Controller Server):** 메인 루프를 돌며 로봇을 제어합니다.
2. **Inspector (Goal Checker):** "목표에 도착했나?"를 판단합니다 (`isGoalReached`).
3. **Driver (Graceful Controller):** "어떻게 움직일까?"를 계산합니다 (`computeVelocityCommands`).

> **중요:** Graceful Controller가 Goal Checker에게 "나 도착했어?"라고 직접 묻지 않습니다. 그건 Boss(Server)가 합니다. Graceful Controller는 단지 Goal Checker에게 **"기준(Tolerance)이 얼마야?"**라고만 물어봅니다.

---

### 2. 코드 레벨 상호작용 (Code Level)

Graceful Controller의 소스코드(`computeVelocityCommands`)를 보면 `goal_checker` 포인터를 인자로 받아오는 것을 볼 수 있습니다.

#### [nav2_controller::ControllerServer (The Boss)]

Server는 매 주기(Tick)마다 대략 이런 순서로 일을 시킵니다.

```cpp
// (의사 코드) Controller Server의 메인 루프
void computeControl() {
    // 1. Inspector에게 물어봄: "우리 도착했어?"
    if (goal_checker_->isGoalReached(current_pose, velocity, ...)) {
        publishZeroVelocity(); // 정지 명령
        return; // 종료
    }

    // 2. 도착 안 했으면 Driver에게 시킴: "계속 운전해. 여기 Inspector 정보도 줄게."
    cmd_vel = controller_->computeVelocityCommands(current_pose, velocity, goal_checker_);
    
    // 3. 속도 명령 발행
    publishVelocity(cmd_vel);
}

```

#### [nav2_graceful_controller::GracefulController (The Driver)]

사용자님이 제공해주신 코드의 핵심 부분입니다.

```cpp
geometry_msgs::msg::TwistStamped GracefulController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker) // <--- [연결 고리] Server가 Checker 포인터를 넘겨줌
{
  // 1. Checker에게 "너의 기준(Tolerance)이 몇이야?"라고 물어봄 (값만 가져옴)
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  
  // 여기서 Checker는 자신의 설정값(0.1m 등)을 리턴함.
  // [문제점]: Checker가 내부적으로 XY를 통과했는지(Stateful) 여부는 여기서 안 알려줌!
  if (!goal_checker->getTolerances(pose_tolerance, velocity_tolerance)) {
     // 에러 처리
  } else {
     goal_dist_tolerance_ = pose_tolerance.position.x; // 값 저장 (예: 0.1)
  }

  // 2. Driver(Controller)가 독자적으로 거리를 재봄
  double dist_to_goal = calculate_path_length(...);

  // 3. Driver의 판단
  // "Inspector 기준은 0.1인데, 내가 재보니 0.11이네? 아직 멀었군. 주행 모드 유지!"
  if (dist_to_goal < goal_dist_tolerance_ || goal_reached_) {
      // 제자리 회전 모드
  } else {
      // 주행 모드 (XY 보정 시도) -> 여기서 진동 발생
  }
}

```

---

### 3. 상호작용 로직의 허점 (The Gap)

여기서 사용자님이 겪으신 문제의 **근본적인 원인**이 드러납니다.

#### A. Goal Checker (SimpleGoalChecker)의 입장

* `isGoalReached()` 함수에서: "어? XY 오차(0.1m) 안에 들어왔네? `check_xy_ = false`로 바꿈! 이제 위치 신경 안 씀!" (**Stateful Latch**)
* 하지만 `getTolerances()` 함수에서: "내 XY 허용 오차는 **0.1m**야." (그냥 설정값을 그대로 줌. "이미 통과했음"이라는 정보는 안 줌)

#### B. Graceful Controller의 입장

* `computeVelocityCommands()`에서 `goal_checker->getTolerances()`를 호출.
* 받은 값: **0.1m**.
* Controller의 생각: "아, 목표까지 0.1m 이내여야 하는구나."
* 현재 상황: 로봇이 회전하다가 0.11m로 밀려남.
* Controller의 판단: **"내부 계산 결과 0.11m > 0.1m 이므로 아직 도착 안 함. XY 위치 맞춰야 함!"**

#### C. 결과

* **Boss (Server):** `isGoalReached`를 호출하면 Checker는 (XY 무시하므로) **"각도만 맞으면 도착이야"**라고 할 준비가 되어 있음.
* **Driver (Controller):** 하지만 Driver는 Checker의 속마음(Stateful 상태)을 모르고, **"거리 0.1m 넘었으니 후진해서라도 위치 맞춰!"**라고 하며 이상한 `cmd_vel`을 보냄.
* 이로 인해 로봇은 덜덜 떨게 되고, 속도가 안정되지 않으니(혹은 각도가 계속 흔들리니) Boss는 "아직 도착 안 했네"라며 루프를 계속 돌림.

---

### 4. 요약: 왜 코드를 수정해야 했나?

`Graceful Controller`와 `Goal Checker`는 **느슨하게 연결(Loosely Coupled)**되어 있습니다.
Controller는 Checker의 **"현재 상태(XY 통과했음)"**를 공유받는 게 아니라, **"설정값(0.1m)"**만 공유받습니다.

그렇기 때문에 **Controller 내부 코드(`computeVelocityCommands`)**에다가 **"한 번 가까워졌으면(Tolerance 근처), 네가 알아서 눈치껏 위치 무시해라(Hysteresis)"**라는 로직을 심어주어야 비로소 Goal Checker의 `Stateful` 동작과 합이 맞게 되는 것입니다.
