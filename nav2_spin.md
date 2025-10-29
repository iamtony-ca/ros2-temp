네, 정확히 짚으셨습니다. Spin 동작 중에도 지속적으로 장애물 체크를 합니다.

사용자님의 오해는 onRun과 onCycleUpdate의 역할을 혼동하신 것일 수 있습니다. Spin 노드는 Nav2의 TimedBehavior를 상속받으며, Behavior Tree(BT)에 의해 반복적으로 '틱(tick)'됩니다.

코드를 자세히 살펴보면 동작 방식은 다음과 같습니다.

1. onRun() : 최초 설정 (단 한 번)
onRun 함수는 Spin 행동이 처음 시작될 때 단 한 번만 호출됩니다.

이 함수는 목표 회전각(cmd_yaw_)을 설정하고, 타임아웃(end_time_)을 설정하는 등 초기화 작업만 수행합니다.

여기서는 vel_pub_->publish()나 isCollisionFree()가 호출되지 않습니다. 즉, onRun에서는 실제 로봇을 움직이거나 충돌 검사를 하지 않습니다.

2. onCycleUpdate() : 실제 동작 루프 (반복 호출)
onCycleUpdate 함수는 BT가 Spin 노드를 실행하는 동안 매 틱(tick)마다 반복적으로 호출됩니다. 실제 회전 명령과 충돌 검사는 모두 이 함수 안에서 이루어집니다.

onCycleUpdate의 매 틱마다의 흐름은 다음과 같습니다.

시간 초과 확인: command_time_allowance_를 넘었는지 확인합니다.

현재 자세 획득: 현재 로봇의 자세(current_pose)를 가져옵니다.

진행 상황 계산: 얼마나 회전했는지(relative_yaw_) 계산합니다.

목표 도달 확인: 목표 각도(cmd_yaw_)에 도달했으면 stopRobot()을 호출하고 SUCCEEDED를 반환합니다.

속도 계산: 목표까지 남은 각도(remaining_yaw)를 바탕으로 이번 틱에서 사용할 각속도(vel)를 계산합니다.

cmd_vel 생성: 계산된 속도로 TwistStamped 메시지를 생성합니다.

💥 핵심: 충돌 검사 및 시뮬레이션

C++

if (!isCollisionFree(relative_yaw_, cmd_vel->twist, pose2d)) {
    stopRobot();
    RCLCPP_WARN(logger_, "Collision Ahead - Exiting Spin");
    return ResultStatus{Status::FAILED, SpinActionResult::COLLISION_AHEAD};
}
바로 이 부분에서 isCollisionFree 함수가 호출됩니다.

isCollisionFree는 현재 계산된 속도(cmd_vel)로 앞으로 simulate_ahead_time_ (기본값 2.0초) 동안 회전할 경우의 미래 경로를 시뮬레이션합니다.

이 시뮬레이션 경로는 local_collision_checker_ (즉, Local Costmap)를 기반으로 충돌 여부를 검사합니다.

안전할 경우에만 속도 게시

C++

vel_pub_->publish(std::move(cmd_vel));
isCollisionFree 검사를 통과(충돌 없음)했을 때만 vel_pub_를 통해 실제 로봇에 속도 명령을 전송합니다.

RUNNING 반환: 아직 목표에 도달하지 않았으므로 RUNNING 상태를 반환하여 BT가 다음 틱에도 onCycleUpdate를 호출하도록 합니다.

요약
Spin 노드는 "회전 시작" 명령을 한 번 내리고 끝나는 것이 아니라, BT의 틱 주기에 맞춰 **"살짝 회전 → 미래 경로 충돌 검사 → 안전하면 살짝 회전 → 미래 경로 충돌 검사 → ..."**를 반복하는 구조입니다.

따라서 Spin 동작 중에 Local Costmap이 업데이트되어 장애물이 감지되면, 다음 onCycleUpdate 틱의 isCollisionFree 호출에서 즉시 감지되고 로봇은 멈추게 됩니다.
