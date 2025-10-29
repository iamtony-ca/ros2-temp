네, 정확히 짚으셨습니다. `Spin` 동작 중에도 **지속적으로 장애물 체크를 합니다.**

사용자님의 오해는 `onRun`과 `onCycleUpdate`의 역할을 혼동하신 것일 수 있습니다. `Spin` 노드는 Nav2의 `TimedBehavior`를 상속받으며, Behavior Tree(BT)에 의해 반복적으로 '틱(tick)'됩니다.

코드를 자세히 살펴보면 동작 방식은 다음과 같습니다.

-----

### 1\. `onRun()` : 최초 설정 (단 한 번)

`onRun` 함수는 `Spin` 행동이 **처음 시작될 때 단 한 번만 호출**됩니다.

  * 이 함수는 목표 회전각(`cmd_yaw_`)을 설정하고, 타임아웃(`end_time_`)을 설정하는 등 초기화 작업만 수행합니다.
  * 여기서는 `vel_pub_->publish()`나 `isCollisionFree()`가 호출되지 않습니다. 즉, **`onRun`에서는 실제 로봇을 움직이거나 충돌 검사를 하지 않습니다.**

-----

### 2\. `onCycleUpdate()` : 실제 동작 루프 (반복 호출)

`onCycleUpdate` 함수는 BT가 `Spin` 노드를 실행하는 동안 **매 틱(tick)마다 반복적으로 호출**됩니다. 실제 회전 명령과 충돌 검사는 모두 이 함수 안에서 이루어집니다.

`onCycleUpdate`의 매 틱마다의 흐름은 다음과 같습니다.

1.  **시간 초과 확인**: `command_time_allowance_`를 넘었는지 확인합니다.
2.  **현재 자세 획득**: 현재 로봇의 자세(`current_pose`)를 가져옵니다.
3.  **진행 상황 계산**: 얼마나 회전했는지(`relative_yaw_`) 계산합니다.
4.  **목표 도달 확인**: 목표 각도(`cmd_yaw_`)에 도달했으면 `stopRobot()`을 호출하고 `SUCCEEDED`를 반환합니다.
5.  **속도 계산**: 목표까지 남은 각도(`remaining_yaw`)를 바탕으로 이번 틱에서 사용할 각속도(`vel`)를 계산합니다.
6.  **`cmd_vel` 생성**: 계산된 속도로 `TwistStamped` 메시지를 생성합니다.
7.  **💥 핵심: 충돌 검사 및 시뮬레이션**
    ```cpp
    if (!isCollisionFree(relative_yaw_, cmd_vel->twist, pose2d)) {
        stopRobot();
        RCLCPP_WARN(logger_, "Collision Ahead - Exiting Spin");
        return ResultStatus{Status::FAILED, SpinActionResult::COLLISION_AHEAD};
    }
    ```
      * 바로 이 부분에서 `isCollisionFree` 함수가 호출됩니다.
      * `isCollisionFree`는 **현재 계산된 속도(`cmd_vel`)로 앞으로 `simulate_ahead_time_` (기본값 2.0초) 동안 회전할 경우의 미래 경로를 시뮬레이션**합니다.
      * 이 시뮬레이션 경로는 `local_collision_checker_` (즉, **Local Costmap**)를 기반으로 충돌 여부를 검사합니다.
8.  **안전할 경우에만 속도 게시**
    ```cpp
    vel_pub_->publish(std::move(cmd_vel));
    ```
      * `isCollisionFree` 검사를 통과(충돌 없음)했을 때만 `vel_pub_`를 통해 실제 로봇에 속도 명령을 전송합니다.
9.  **`RUNNING` 반환**: 아직 목표에 도달하지 않았으므로 `RUNNING` 상태를 반환하여 BT가 다음 틱에도 `onCycleUpdate`를 호출하도록 합니다.

-----

### 요약

`Spin` 노드는 "회전 시작" 명령을 한 번 내리고 끝나는 것이 아니라, BT의 틱 주기에 맞춰 \*\*"살짝 회전 → 미래 경로 충돌 검사 → 안전하면 살짝 회전 → 미래 경로 충돌 검사 → ..."\*\*를 반복하는 구조입니다.

따라서 `Spin` 동작 중에 Local Costmap이 업데이트되어 장애물이 감지되면, **다음 `onCycleUpdate` 틱의 `isCollisionFree` 호출에서 즉시 감지**되고 로봇은 멈추게 됩니다.


#####################  
######################  
네, `Spin` 생성자 리스트에 있는 각 멤버 변수들의 역할은 다음과 같습니다.

---

### 1. 기반 클래스 및 피드백

* **`TimedBehavior<SpinAction>()`**
    * `Spin` 노드의 **기반(Base) 클래스 생성자**입니다.
    * `Spin`이 Nav2 Behavior Tree의 플러그인으로 동작하는 데 필요한 모든 기본 기능을 초기화합니다. `onRun`, `onCycleUpdate` 같은 핵심 함수와 `end_time_` 같은 타임아웃 로직을 이 클래스로부터 상속받습니다.

* **`feedback_(std::make_shared<SpinAction::Feedback>())`**
    * **액션(Action) 피드백**을 위한 메시지 객체입니다.
    * `Spin` 동작이 실행되는 동안 BT(Behavior Tree)나 다른 상위 노드에게 현재 상태를 알리는 데 사용됩니다.
    * `onCycleUpdate` 루프 내에서 `angular_distance_traveled`(현재까지 회전한 각도) 같은 정보를 담아 게시(publish)합니다.

---

### 2. ROS 파라미터 (운동 역학 및 시뮬레이션)

이 변수들은 `onConfigure` 함수에서 ROS 파라미터 서버로부터 값을 읽어와 채워집니다.

* **`min_rotational_vel_(0.0)`**
    * **최소 회전 속도 (rad/s)**입니다. (ROS 파라미터: `min_rotational_vel`)
    * `onCycleUpdate`에서 속도를 계산할 때, 이 값보다 느려지지 않도록 보장합니다. 로봇이 너무 느리게 움직이거나 멈칫거리는 것을 방지합니다.

* **`max_rotational_vel_(0.0)`**
    * **최대 회전 속도 (rad/s)**입니다. (ROS 파라미터: `max_rotational_vel`)
    * 로봇의 물리적 한계를 넘지 않도록 속도를 제한하는 상한선입니다.

* **`rotational_acc_lim_(0.0)`**
    * **각가속도 제한 (rad/s²)**입니다. (ROS 파라미터: `rotational_acc_lim`)
    * 목표 각도까지 부드럽게 감속하며 도달하기 위한 속도를 계산할 때 사용됩니다. (`vel = sqrt(2 * accel * remaining_angle)` 공식에 사용됨)

* **`simulate_ahead_time_(0.0)`**
    * **미래 시뮬레이션 시간 (sec)**입니다. (ROS 파라미터: `simulate_ahead_time`)
    * `isCollisionFree` 함수가 충돌 검사를 수행할 때, **현재 속도로 "몇 초" 앞의 미래까지를 예측하여 검사할지** 결정합니다. 기본값은 2.0초입니다.

---

### 3. 내부 상태 변수 (State Tracking)

이 변수들은 `Spin` 동작이 실행되는 동안 내부적으로 상태를 추적하기 위해 사용됩니다.

* **`cmd_yaw_(0.0)`**
    * **목표 회전 각도 (rad)**입니다.
    * `onRun`이 호출될 때 BT로부터 전달받은 최종 목표 회전량(예: 1.57 rad)을 저장합니다.

* **`prev_yaw_(0.0)`**
    * **이전 틱(tick)의 Yaw (rad)**입니다.
    * `onCycleUpdate`가 매번 호출될 때마다, 직전 주기에서 측정한 로봇의 Yaw 값을 저장하는 데 사용됩니다. *현재* Yaw 값과의 차이(`delta_yaw`)를 계산하기 위해 필요합니다.

* **`relative_yaw_(0.0)`**
    * **누적 상대 회전 각도 (rad)**입니다.
    * `Spin` 동작이 시작된 후 **현재까지 실제로 회전한 총 각도**를 누적합니다.
    * `onRun`에서 0으로 초기화되며, `onCycleUpdate`에서 계산된 `delta_yaw`가 이 변수에 계속 더해집니다. 이 값이 `cmd_yaw_`에 도달하면 동작이 성공합니다.
###################
######################
정확하게 파악하셨습니다. BT.xml에서 사용하려는 것이라면 `nav2_behavior_tree`의 `spin_action.cpp` (SpinAction)가 맞습니다.

두 코드는 Nav2의 Behavior Tree가 ROS 2 액션(Action)을 호출하는 방식을 보여주는 전형적인 **클라이언트-서버(Client-Server) 구조**입니다.

간단히 말해, \*\*`nav2_behavior_tree`의 코드는 '손님(Client)'\*\*이고, \*\*`nav2_behaviors`의 코드는 '요리사(Server)'\*\*입니다.

-----

### 1\. `nav2_behavior_tree/plugins/action/spin_action.cpp` (BT 노드 / 클라이언트)

이 코드는 **Behavior Tree XML 파일 내부에서 사용되는 `<Spin>` 태그 그 자체**를 정의합니다.

  * **역할**: Behavior Tree의 "프론트엔드" 또는 "인터페이스"입니다.
  * **상속**: `BtActionNode<nav2_msgs::action::Spin>`를 상속받습니다.
      * `BtActionNode`는 ROS 2 액션 클라이언트를 Behavior Tree 노드로 쉽게 포장해주는 템플릿입니다.
  * **주요 기능**:
    1.  **XML 파라미터 읽기**: `initialize()` 함수에서 `getInput("spin_dist", ...)`를 호출합니다. 이는 `bt.xml` 파일에 `<Spin spin_dist="3.14" ... />`와 같이 정의된 **입력 포트(port)에서 값을 읽어오는** 역할입니다.
    2.  **액션 목표(Goal) 생성**: 읽어온 `spin_dist` 값을 `goal_.target_yaw`에 채워 넣어 `nav2_msgs::action::Spin` 타입의 ROS 2 액션 목표 메시지를 만듭니다.
    3.  **액션 서버 호출**: `on_tick()`이 호출되면 (부모 클래스인 `BtActionNode`가) 이 `goal_` 메시지를 `/spin`이라는 이름의 ROS 2 액션 서버로 전송합니다.
    4.  **결과 반환**: 액션 서버로부터 응답(성공, 실패, 중단)이 오면, `on_success()`, `on_aborted()` 등이 호출되어 BT 상태(`BT::NodeStatus::SUCCESS` 또는 `FAILURE`)로 변환하여 트리에 보고합니다.
  * **플러그인 등록**:
    ```cpp
    BT_REGISTER_NODES(factory)
    {
    ...
    factory.registerBuilder<nav2_behavior_tree::SpinAction>("Spin", builder);
    }
    ```
    이 부분이 `BehaviorTree.Cpp` 라이브러리에 "Spin"이라는 이름의 노드를 등록하는 코드입니다. **이 코드가 있어야만 XML에서 `<Spin ... />` 태그를 인식할 수 있습니다.**

-----

### 2\. `nav2_behaviors/plugins/spin.cpp` (행동 플러그인 / 서버)

이 코드는 \*\*실제로 로봇을 회전시키는 모든 로직을 수행하는 "엔진"\*\*입니다.

  * **역할**: Behavior Tree의 "백엔드" 또는 "실행부"입니다. Nav2의 **Behavior Server** (`behavior_server` 노드)에 의해 로드되는 플러그인입니다.
  * **상속**: `TimedBehavior<SpinAction>`를 상속받습니다.
      * `TimedBehavior`는 `nav2_core::Behavior`를 기반으로 하며, `/spin` 액션 서버의 기능을 구현합니다.
  * **주요 기능**:
    1.  **액션 목표 수신**: `/spin` 액션 서버로 요청이 들어오면 `onRun()`이 호출됩니다. 여기서 `command->target_yaw` (즉, BT 노드가 보낸 `spin_dist`)를 `cmd_yaw_`에 저장합니다.
    2.  **지속적인 실행 및 검사**: `onCycleUpdate()`가 주기적으로 호출됩니다.
    3.  **충돌 검사**: `isCollisionFree()`를 호출하여 **Local Costmap을 기반으로 미래 경로의 충돌을 시뮬레이션**합니다.
    4.  **로봇 제어**: 충돌이 없다고 판단되면 `vel_pub_->publish()`를 통해 **실제 `/cmd_vel` 토픽에 속도 명령을 게시**하여 로봇을 회전시킵니다.
    5.  **피드백 및 결과 전송**: 목표 각도에 도달하면 `ResultStatus{Status::SUCCEEDED, ...}`를 반환하여 액션 클라이언트(BT 노드)에게 성공을 알립니다. 충돌이 감지되면 `FAILED`를 반환합니다.
  * **플러그인 등록**:
    ```cpp
    PLUGINLIB_EXPORT_CLASS(nav2_behaviors::Spin, nav2_core::Behavior)
    ```
    이 부분이 `pluginlib`를 통해 이 클래스를 `nav2_core::Behavior` 타입의 플러그인으로 등록하는 코드입니다. **이 코드가 있어야만 `behavior_server`가 이 플러그인을 로드할 수 있습니다.**

-----

### 🚀 전체 동작 흐름

1.  **BT.xml**: 사용자가 `<Spin spin_dist="1.57" />` 노드를 BT.xml에 추가합니다.
2.  **BT 실행**: Behavior Tree가 이 `<Spin>` 노드를 "틱(tick)"합니다.
3.  **`SpinAction` (BT 노드 / 클라이언트)**:
      * `spin_action.cpp`의 `SpinAction` 클래스가 활성화됩니다.
      * `initialize()`가 `spin_dist="1.57"` 값을 읽습니다.
      * `target_yaw = 1.57`로 설정된 `nav2_msgs::action::Spin` 목표(goal)를 생성하여 `/spin` 액션 서버로 전송합니다.
4.  **`behavior_server` (중간 관리자)**:
      * `/spin` 액션 요청을 수신합니다.
      * 자신에게 로드된 플러그인 중 `nav2_behaviors::Spin` (서버) 플러그인에게 이 요청을 전달합니다.
5.  **`Spin` (행동 플러그인 / 서버)**:
      * `spin.cpp`의 `Spin` 클래스가 활성화됩니다.
      * `onRun()`이 호출되어 `cmd_yaw_ = 1.57`로 설정됩니다.
      * `onCycleUpdate()` 루프가 시작됩니다.
      * 루프마다 `isCollisionFree()`로 충돌을 검사하고, 안전하면 `vel_pub_->publish()`로 로봇을 조금씩 회전시킵니다.
      * 로봇이 1.57 라디안을 모두 회전하면, `SUCCEEDED`를 반환합니다.
6.  **결과 전송**:
      * `behavior_server`는 이 `SUCCEEDED` 결과를 `SpinAction` (클라이언트)에게 다시 전송합니다.
7.  **BT 노드 완료**:
      * `SpinAction`의 `on_success()`가 호출됩니다.
      * `SpinAction` 노드는 `BT::NodeStatus::SUCCESS`를 Behavior Tree에 반환하고, BT는 다음 노드를 실행하러 갑니다.
