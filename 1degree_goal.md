반갑습니다. ROS 2 Jazzy와 Nav2 환경에서 **제자리 정밀 회전(In-place Rotation)** 테스트를 위한 파이썬 스크립트를 작성해 드리겠습니다.

요청하신 대로 **현재 위치(Position)는 고정**한 상태에서, **Yaw(Heading) 각도만 딱 1도(Degree) 차이**가 나는 두 개의 Goal을 번갈아 가며 N회 반복 수행하는 코드입니다.

이 작업에서 가장 중요한 것은 코드가 아니라 **Nav2 파라미터 설정**입니다. 1도는 매우 작은 각도이기 때문에, Nav2의 허용 오차(Tolerance)가 1도보다 크다면 로봇은 움직이지 않습니다. 이 부분도 함께 설명해 드립니다.

###핵심 포인트1. **Nav2 Simple Commander API (`BasicNavigator`)** 사용.
2. **Quaternion 변환 로직**: Euler(Degree) \to Radian \to Quaternion.
3. **동기 실행**: 이전 Goal이 성공/실패할 때까지 대기 후 다음 명령 하달.

---

###1. 사전 준비: `nav2_params.yaml` 확인 (필수)이 코드를 실행하기 전에 로봇의 `controller_server` 파라미터 중 `yaw_goal_tolerance`를 확인해야 합니다.

* **기본값:** 보통 `0.25` (약 14도) 혹은 `0.1` (약 5.7도)로 설정되어 있습니다.
* **문제점:** 1도(약 0.017 rad)만 움직이라고 명령해도, 허용 오차가 5도라면 로봇은 "이미 목표에 도달했다"고 판단하고 움직이지 않습니다.
* **해결:** `nav2_params.yaml`에서 `yaw_goal_tolerance`를 **0.01 (약 0.57도)** 이하로 낮춰야 합니다.

```yaml
# 예시: nav2_params.yaml
controller_server:
  ros__parameters:
    # ... (생략)
    general_goal_checker:
      yaw_goal_tolerance: 0.01  # <--- 이 값을 0.017 (1도) 보다 작게 설정하세요.

```

---

###2. Python Script (`rotate_1_degree.py`)아래 코드는 `nav2_simple_commander` 패키지가 설치되어 있어야 합니다.
(`sudo apt install ros-jazzy-nav2-simple-commander`)

```python
#!/usr/bin/env python3
import math
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

"""
Description: 
    Nav2 Jazzy 환경에서 제자리 회전 정밀 테스트를 수행합니다.
    현재 위치를 기준으로 0도 <-> 1도(상대각)를 N회 반복합니다.
    
Note:
    실행 전 nav2_params.yaml의 'yaw_goal_tolerance'가 0.017 rad (1도)보다 작은지 확인하세요.
"""

def euler_to_quaternion(yaw_deg):
    """
    Degree 단위의 Yaw 각도를 ROS 2 Quaternion(x, y, z, w)으로 변환합니다.
    """
    # Degree -> Radian 변환
    yaw_rad = math.radians(yaw_deg)
    
    # Z축 회전(Yaw)에 대한 Quaternion 계산
    # qx = 0, qy = 0, qz = sin(theta/2), qw = cos(theta/2)
    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)
    
    return 0.0, 0.0, qz, qw

def create_goal_pose(navigator, x, y, yaw_deg):
    """
    좌표(x, y)와 각도(yaw_deg)를 받아 PoseStamped 메시지를 생성합니다.
    """
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.z = 0.0
    
    # Quaternion 변환
    _, _, qz, qw = euler_to_quaternion(yaw_deg)
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = qz
    goal_pose.pose.orientation.w = qw
    
    return goal_pose

def main():
    # 1. ROS 2 초기화
    rclpy.init()
    
    # 2. Nav2 Navigator 인스턴스 생성
    navigator = BasicNavigator()
    
    # 3. Nav2 활성화 대기 (Lifecycle)
    # Autostart가 True가 아니라면 초기화될 때까지 기다립니다.
    # navigator.waitUntilNav2Active() 

    # 4. 설정 변수
    REPEAT_COUNT = 5        # 반복 횟수 (n회)
    ANGLE_DIFF_DEG = 1.0    # 차이낼 각도 (1도)
    
    print("Nav2 활성화를 기다리는 중...")
    navigator.waitUntilNav2Active()
    print("Nav2 준비 완료!")

    # 5. 기준 위치 설정 (현재 로봇의 위치를 가져오거나 수동 설정)
    # 초기 위치를 가져오기 위해 잠시 기다림 (amcl/slam 위치 추정 확보)
    initial_pose = None
    while initial_pose is None:
        # 주의: getGlobalPose()는 로봇이 localization이 되어 있어야 값을 반환함
        # 만약 값을 못 받아온다면 수동으로 좌표를 입력하세요.
        try:
            # Jazzy API에서는 getGlobalPose 사용 (pose, stamp 반환)
            initial_pose_stamped = navigator.getGlobalPose() 
            initial_pose = initial_pose_stamped
            print(f"현재 로봇 위치 획득: x={initial_pose.pose.position.x:.2f}, y={initial_pose.pose.position.y:.2f}")
        except Exception:
            print("현재 위치를 받아오는 중... (Localization 상태 확인 필요)")
            rclpy.spin_once(navigator, timeout_sec=1.0)
            
    # 현재 위치 고정 (Position Lock)
    base_x = initial_pose.pose.position.x
    base_y = initial_pose.pose.position.y
    
    # 현재의 Yaw를 기준으로 할 것인지, 절대 각도 0도를 기준으로 할 것인지 결정
    # 여기서는 계산 편의를 위해 절대 각도 0도와 1도를 사용하겠습니다.
    # (필요시 현재 로봇의 yaw를 구해서 +1을 해도 됩니다)
    yaw_1 = 0.0
    yaw_2 = yaw_1 + ANGLE_DIFF_DEG # 1도 차이

    # 6. 반복 루프 시작
    for i in range(1, REPEAT_COUNT + 1):
        print(f"\n--- 반복 횟수: {i} / {REPEAT_COUNT} ---")
        
        # --- [Step 1] 첫 번째 각도 (0도) ---
        goal_pose_1 = create_goal_pose(navigator, base_x, base_y, yaw_1)
        print(f"1. 목표 전송: Yaw {yaw_1}도")
        navigator.goToPose(goal_pose_1)

        # 작업 완료 대기 Loop
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            # 필요하다면 여기서 피드백 출력 (남은 거리 등)
            # print(f"이동 중...", end='\r')
            pass
        
        # 결과 확인
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(">> Goal 1 도달 완료.")
        else:
            print(f">> Goal 1 실패/취소됨. 상태: {result}")
            break # 실패 시 루프 종료

        # --- [Step 2] 두 번째 각도 (1도) ---
        goal_pose_2 = create_goal_pose(navigator, base_x, base_y, yaw_2)
        print(f"2. 목표 전송: Yaw {yaw_2}도 (+{ANGLE_DIFF_DEG} deg)")
        navigator.goToPose(goal_pose_2)

        while not navigator.isTaskComplete():
            pass

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(">> Goal 2 도달 완료.")
        else:
            print(f">> Goal 2 실패/취소됨. 상태: {result}")
            break

    # 7. 종료
    print("\n모든 반복이 완료되었습니다.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

###코드 설명 및 전문가 팁1. **`euler_to_quaternion` 함수**:
* ROS 2 Jazzy라도 `tf_transformations` 라이브러리가 기본 설치되지 않은 환경이 있을 수 있습니다. 따라서 의존성을 줄이기 위해 순수 `math` 라이브러리로 쿼터니언 변환(Euler Z \to Quaternion)을 직접 구현했습니다.


2. **`navigator.getGlobalPose()`**:
* 로봇의 현재 위치를 기준으로 하기 위해 초기 위치를 받아옵니다. 만약 로봇이 `amcl` 등을 통해 Localization이 되어 있지 않다면 이 함수는 데이터를 반환하지 못할 수 있습니다.


3. **반복 로직 (`goToPose` & `isTaskComplete`)**:
* `goToPose()`는 비동기(Non-blocking) 함수입니다. 따라서 `while not navigator.isTaskComplete(): pass` 구문을 통해 로봇이 동작을 완료할 때까지 파이썬 스크립트가 대기하도록 만들었습니다.
* 이 방식이 정확히 "명령어 1개 종료 -> 다음 명령어 전송"의 요구사항을 충족합니다.



###실행 방법1. 터미널 1: Nav2 시뮬레이션 또는 실제 로봇 실행
2. 터미널 2: 스크립트 실행
```bash
python3 rotate_1_degree.py

```



이 코드는 Nav2 Stack이 정상적으로 동작하고 있다는 가정하에, 가장 깔끔하고 의존성 없이 동작하는 모던한 ROS 2 Python 스타일입니다.
#########################  
###########################  
###########################  
네, 아주 좋은 접근입니다. **AMCL이 아닌 다른 SLAM(Cartographer, SLAM Toolbox 등)을 사용하거나 오도메트리 기반으로 테스트할 때**는 `/tf` 토픽에서 `map` \to `base_link` 간의 변환(Transform)을 직접 읽어오는 것이 가장 확실한 방법입니다.

기존 코드에서 `navigator.getGlobalPose()` 대신 **TF Listener**를 사용하여 현재 로봇의 정확한 위치와 방향(Yaw)을 가져오도록 수정한 코드를 작성해 드리겠습니다.

###핵심 변경 사항1. **`tf2_ros` 모듈 추가**: TF 트리를 조회하기 위해 `Buffer`와 `TransformListener`를 사용합니다.
2. **`get_current_pose_from_tf` 함수**: `map` 좌표계 기준으로 `base_link`의 위치를 조회하여 x, y, yaw를 반환합니다.
3. **Quaternion \to Euler 변환 로직 추가**: TF는 쿼터니언을 주므로, 현재 각도(Yaw)를 알기 위해 오일러 변환 수식을 추가했습니다.

###수정된 Python Script (`rotate_1_degree_tf.py`)```python
#!/usr/bin/env python3
import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# TF 관련 모듈 임포트
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

"""
Description: 
    Nav2 Jazzy 환경에서 /tf 토픽을 통해 현재 위치를 파악하고,
    제자리에서 Yaw 각도만 1도씩 변경하며 반복 회전하는 스크립트입니다.
"""

def euler_to_quaternion(yaw_deg):
    """ Degree -> Quaternion 변환 (Goal 전송용) """
    yaw_rad = math.radians(yaw_deg)
    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)
    return 0.0, 0.0, qz, qw

def quaternion_to_yaw(qx, qy, qz, qw):
    """ Quaternion -> Yaw(Degree) 변환 (현재 위치 파악용) """
    # roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp) # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(yaw_rad)

class TFReader(Node):
    """ TF 데이터를 읽기 위한 보조 노드 클래스 """
    def __init__(self):
        super().__init__('tf_reader_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_current_pose(self, target_frame='map', source_frame='base_link'):
        """ map -> base_link 변환을 조회하여 x, y, yaw(deg) 반환 """
        try:
            # 타임아웃 1.0초로 최신 transform 조회
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                target_frame, 
                source_frame, 
                now, 
                timeout=Duration(seconds=1.0)
            )
            
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            
            # Quaternion 추출
            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w
            
            yaw = quaternion_to_yaw(qx, qy, qz, qw)
            return x, y, yaw
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None, None, None

def create_goal_pose(navigator, x, y, yaw_deg):
    """ 좌표와 각도로 Goal Pose 생성 """
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.z = 0.0
    _, _, qz, qw = euler_to_quaternion(yaw_deg)
    goal_pose.pose.orientation.z = qz
    goal_pose.pose.orientation.w = qw
    return goal_pose

def main():
    rclpy.init()
    
    # 1. Nav2 Navigator 및 TF Reader 생성
    navigator = BasicNavigator()
    tf_reader = TFReader()

    # 2. Nav2 활성화 대기
    print("Nav2 활성화를 기다리는 중...")
    navigator.waitUntilNav2Active()
    print("Nav2 준비 완료!")

    # 3. 설정 변수
    REPEAT_COUNT = 5        # 반복 횟수
    ANGLE_DIFF_DEG = 1.0    # 각도 차이 (1도)

    # 4. 현재 위치(TF) 가져오기
    # TF가 준비될 때까지 잠시 대기
    current_x, current_y, current_yaw = None, None, None
    print("TF에서 현재 위치를 조회합니다...")
    
    while current_x is None:
        rclpy.spin_once(tf_reader, timeout_sec=0.5)
        current_x, current_y, current_yaw = tf_reader.get_current_pose()
        
    print(f"현재 위치 고정: x={current_x:.4f}, y={current_y:.4f}")
    print(f"현재 헤딩(Yaw): {current_yaw:.4f} 도")

    # 5. 두 개의 Goal 각도 설정
    # Goal 1: 현재 각도
    yaw_1 = current_yaw
    # Goal 2: 현재 각도 + 1도
    yaw_2 = current_yaw + ANGLE_DIFF_DEG

    # 6. 반복 루프 시작
    for i in range(1, REPEAT_COUNT + 1):
        print(f"\n--- 반복 {i} / {REPEAT_COUNT} ---")
        
        # [Step 1] 첫 번째 각도 (원래 각도)
        goal_pose_1 = create_goal_pose(navigator, current_x, current_y, yaw_1)
        print(f"1. 이동 명령: Yaw {yaw_1:.2f}도")
        navigator.goToPose(goal_pose_1)

        while not navigator.isTaskComplete():
            pass # 이동 완료 대기

        if navigator.getResult() != TaskResult.SUCCEEDED:
            print(f">> 실패! (Result: {navigator.getResult()})")
            break
        print(">> Goal 1 도착.")

        # [Step 2] 두 번째 각도 (+1도)
        goal_pose_2 = create_goal_pose(navigator, current_x, current_y, yaw_2)
        print(f"2. 이동 명령: Yaw {yaw_2:.2f}도")
        navigator.goToPose(goal_pose_2)

        while not navigator.isTaskComplete():
            pass

        if navigator.getResult() != TaskResult.SUCCEEDED:
            print(f">> 실패! (Result: {navigator.getResult()})")
            break
        print(">> Goal 2 도착.")

    print("\n테스트 종료.")
    
    # 종료 처리
    tf_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

###코드 분석 및 주의사항1. **`TFReader` 클래스**:
* `BasicNavigator`와 별도로 `rclpy.node.Node`를 상속받는 클래스를 만들었습니다.
* 이 노드는 오직 `/tf` 토픽을 수신(Listen)하는 역할만 수행합니다.
* `rclpy.spin_once(tf_reader)`를 통해 메인 루프가 차단되지 않으면서 TF 데이터를 업데이트합니다.


2. **`map` 프레임 필수**:
* SLAM을 사용 중이시므로 `map` 프레임이 존재해야 합니다. 만약 SLAM 없이 오도메트리만 있다면 `get_current_pose` 함수 내의 `target_frame='map'`을 `target_frame='odom'`으로 변경하셔야 합니다.


3. **수학적 보완**:
* `tf_transformations` 라이브러리가 없는 환경에서도 작동하도록 순수 수학(`math` 모듈)으로 오일러 \leftrightarrow 쿼터니언 변환을 구현했습니다.



이 코드를 사용하면 AMCL 상태와 무관하게 **SLAM이 추정한 현재 로봇의 정확한 `map` 좌표**를 기준으로 테스트가 진행됩니다. `nav2_params.yaml`의 `yaw_goal_tolerance` 설정은 여전히 중요하니 꼭 체크해 주세요!
