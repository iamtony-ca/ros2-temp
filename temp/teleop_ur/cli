바로 할 수 있습니다. 터미널 4개, 순서대로 띄우고 마지막에 engage 합니다. 모든 터미널에서 먼저:

source /opt/ros/jazzy/setup.bash && source /isaac-sim/volume/ur_ws/install/setup.bash

시작 전 안전 3가지: 펜던트 안전 설정의 관절 속도 한계를 낮게, E-stop 을 손에, UR16e 베이스 뒤 0.55 m 비우기(랑데부 자세가 상완이 뒤로 수평). 첫 engage 는 속도 상한 0.3 으로 합니다.

T1 리더
ros2 launch open_manipulator_bringup omy_l100_leader_ai.launch.py port_name:=/dev/ttyUSB0 use_self_collision_avoidance:=false
확인: ros2 control list_controllers -c /leader/controller_manager 4개 active, 손을 떼면 그 자세로 서 있음.

T2 UR16e
ros2 launch ur_bringup ur16e.launch.py use_sim:=false robot_ip:=192.168.0.10
펜던트에서 External Control 프로그램 ▶. 확인: ros2 topic echo /joint_states --once 에 NaN 없음.

T3 MoveIt (랑데부 sync 가 충돌검사 계획에 씀)
ros2 launch ur_bringup ur16e_moveit.launch.py use_sim:=false

T4 브리지
ros2 launch ur_bringup teleop_omy.launch.py use_sim_time:=false max_joint_speed:=0.3
로그에 omy_to_ur16e up (DISABLED) 와 "put the LEADER at [...] deg" 가 뜹니다. 그리퍼가 없어서 5 초마다 gripper action server not ready 경고가 나오는데 무해합니다.

engage 절차 (T5)
# 1) 리더를 손 떼도 서 있는 rest pose 에 두고, UR16e 를 랑데부로 (MoveIt, 느리게)
ros2 service call /omy_bridge/sync std_srvs/srv/Trigger
ros2 topic echo /omy_bridge/status          # sync:moving → synced 까지 기다림

# 2) 리더와 로봇이 얼마나 어긋났는지 (rad, 관절별 리더매핑 − 로봇)
ros2 topic echo /omy_bridge/engage_error
전 관절이 0.15 rad(8.6°) 이내면 바로 3)으로. 크게 어긋나 있으면(엔코더 영점·손목 오프셋이 다른 게 정상) 그 자리에서 오프셋을 맞춥니다:
ros2 run ur_bringup omy_leader_calib.py --mode match     # offset 6개 출력
ros2 param set /omy_to_ur16e offset "[<출력값 6개>]"     # DISABLED 상태라 즉시 반영
ros2 run ur_bringup omy_leader_calib.py --mode verify    # 잔차 확인
# 3) engage — 리더를 잡은 상태에서
ros2 service call /omy_bridge/enable std_srvs/srv/Trigger    # "engaged"
이제 리더를 천천히 움직이면 UR16e 가 따라옵니다. 멈출 때는:
ros2 service call /omy_bridge/disable std_srvs/srv/Trigger

보면서 판단할 것
- 느리게 따라오면 T4 로그의 slew capped N%(pull 한 코드 기준) 가 있는지 → 있으면 max_joint_speed 를 1.0 → 1.5 → 2.0 으로 재기동. 2.0 은 넘기지 마세요.
- 브리지가 스스로 watchdog/leader_jump 로 끊기면 리더 스트림 문제(케이블·전원)입니다.
- 어느 관절이 반대로 움직이면 disable 후 ros2 param set /omy_to_ur16e sign "[...]" 로 그 관절 부호를 뒤집습니다.

잘 되면 match 로 얻은 offset 을 teleop_omy.launch.py 기본값에 넣어 두세요. engage 거부 메시지나 engage_error 값이 나오면 붙여 주세요.
