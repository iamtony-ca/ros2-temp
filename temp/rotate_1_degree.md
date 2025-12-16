네, 아주 좋은 접근입니다. **AMCL이 아닌 다른 SLAM(Cartographer, SLAM Toolbox 등)을 사용하거나 오도메트리 기반으로 테스트할 때**는 `/tf` 토픽에서 `map` \to `base_link` 간의 변환(Transform)을 직접 읽어오는 것이 가장 확실한 방법입니다.

기존 코드에서 `navigator.getGlobalPose()` 대신 **TF Listener**를 사용하여 현재 로봇의 정확한 위치와 방향(Yaw)을 가져오도록 수정한 코드를 작성해 드리겠습니다.

###핵심 변경 사항1. **`tf2_ros` 모듈 추가**: TF 트리를 조회하기 위해 `Buffer`와 `TransformListener`를 사용합니다.
2. **`get_current_pose_from_tf` 함수**: `map` 좌표계 기준으로 `base_link`의 위치를 조회하여 x, y, yaw를 반환합니다.
3. **Quaternion \to Euler 변환 로직 추가**: TF는 쿼터니언을 주므로, 현재 각도(Yaw)를 알기 위해 오일러 변환 수식을 추가했습니다.

###수정된 Python Script (`rotate_1_degree_tf.py`)
```python
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
######################  
############################  
###############################  
네, 요청하신 두 가지 사항에 맞춰 답변 드립니다.

###1. 도착 후 헤딩 값 출력 (기존 코드 수정)기존 코드의 `main()` 함수 내부, 반복 루프 부분만 수정하면 됩니다. `TaskResult.SUCCEEDED` 이후에 `tf_reader`를 통해 현재 값을 다시 읽어와 출력하는 로직을 추가했습니다.

**수정된 `main` 함수 루프 부분:**

```python
    # ... (이전 코드와 동일) ...

    # 6. 반복 루프 시작
    for i in range(1, REPEAT_COUNT + 1):
        print(f"\n================ [ 반복 {i} / {REPEAT_COUNT} ] ================")
        
        # ---------------- Goal 1 ----------------
        goal_pose_1 = create_goal_pose(navigator, current_x, current_y, yaw_1)
        print(f"1. 명령 전송: Yaw {yaw_1:.2f}도")
        navigator.goToPose(goal_pose_1)

        wait_for_task_completion(navigator)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            # [추가됨] 도착 후 실제 위치/헤딩 업데이트 및 출력
            rclpy.spin_once(tf_reader, timeout_sec=0.1) 
            _, _, final_yaw = tf_reader.get_current_pose()
            
            print(">> Goal 1 도착 성공.")
            if final_yaw is not None:
                error = final_yaw - yaw_1
                print(f"   [결과] 실제 헤딩: {final_yaw:.4f}도 (목표 오차: {error:.4f}도)")
        else:
            print(f">> Goal 1 실패/취소됨. (상태: {result})")
            break
        
        print("   (다음 명령 전 2초 대기...)")
        time.sleep(2.0)

        # ---------------- Goal 2 ----------------
        goal_pose_2 = create_goal_pose(navigator, current_x, current_y, yaw_2)
        print(f"2. 명령 전송: Yaw {yaw_2:.2f}도 (+1.0 deg)")
        navigator.goToPose(goal_pose_2)

        wait_for_task_completion(navigator)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            # [추가됨] 도착 후 실제 위치/헤딩 업데이트 및 출력
            rclpy.spin_once(tf_reader, timeout_sec=0.1)
            _, _, final_yaw = tf_reader.get_current_pose()

            print(">> Goal 2 도착 성공.")
            if final_yaw is not None:
                error = final_yaw - yaw_2
                print(f"   [결과] 실제 헤딩: {final_yaw:.4f}도 (목표 오차: {error:.4f}도)")
        else:
            print(f">> Goal 2 실패/취소됨. (상태: {result})")
            break
        
        print("   (다음 명령 전 2초 대기...)")
        time.sleep(2.0)

    # ... (이하 동일)

```

---

###2. Localization 안정성 테스트 (New Script)로봇이 정지해 있을 때 `map` -> `base_link` 간의 TF를 지속적으로 모니터링하여 **현재값, 평균, 표준편차**를 실시간으로 출력하는 새로운 스크립트(`check_localization_stability.py`)입니다.

* **기능**: 0.5초(설정 가능)마다 TF를 조회하여 통계를 누적 계산합니다.
* **목적**: SLAM이나 Localization 노드가 정지 상태에서 얼마나 튀는지(Jitter) 수치로 확인합니다.
* **참고**: 표준편차(Std Dev)가 클수록 Localization이 불안정하다는 뜻입니다.

```python
#!/usr/bin/env python3
import math
import time
import statistics
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

"""
Description:
    로봇이 정지해 있는 동안 /tf (map -> base_link)를 지속적으로 모니터링하여
    X, Y, Yaw에 대한 실시간 현재값, 평균, 표준편차(Jitter)를 계산해 출력합니다.
"""

def quaternion_to_yaw(qx, qy, qz, qw):
    # yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(yaw_rad)

class LocalizationMonitor(Node):
    def __init__(self):
        super().__init__('localization_monitor')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 데이터 저장소
        self.history_x = []
        self.history_y = []
        self.history_yaw = []
        
        # 0.5초마다 실행 (주기 변경 가능)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.start_time = time.time()
        
        print(f"{'Time(s)':<8} | {'X (curr/avg/std)':<28} | {'Y (curr/avg/std)':<28} | {'Yaw (curr/avg/std)':<28}")
        print("-" * 100)

    def get_tf(self):
        try:
            now = rclpy.time.Time()
            # map -> base_link 조회
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_link', now, timeout=Duration(seconds=0.1))
            
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            
            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w
            
            yaw = quaternion_to_yaw(qx, qy, qz, qw)
            return x, y, yaw
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None, None, None

    def timer_callback(self):
        x, y, yaw = self.get_tf()
        
        if x is None:
            return # TF 실패 시 패스

        # 데이터 저장
        self.history_x.append(x)
        self.history_y.append(y)
        self.history_yaw.append(yaw)
        
        # 통계 계산 (데이터가 2개 이상일 때부터 표준편차 계산)
        count = len(self.history_x)
        if count < 2:
            return

        avg_x = statistics.mean(self.history_x)
        std_x = statistics.stdev(self.history_x)
        
        avg_y = statistics.mean(self.history_y)
        std_y = statistics.stdev(self.history_y)
        
        avg_yaw = statistics.mean(self.history_yaw)
        std_yaw = statistics.stdev(self.history_yaw)
        
        elapsed = time.time() - self.start_time
        
        # 출력 포맷팅
        # 예: 10.5 | 1.000 / 1.001 / 0.005 ...
        print(f"{elapsed:6.1f}   | "
              f"{x:.3f} / {avg_x:.3f} / {std_x:.4f}   | "
              f"{y:.3f} / {avg_y:.3f} / {std_y:.4f}   | "
              f"{yaw:.3f} / {avg_yaw:.3f} / {std_yaw:.4f}")

def main():
    rclpy.init()
    monitor = LocalizationMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n모니터링 종료.")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

```

###팁* 이 코드를 실행하고 로봇을 가만히 두세요.
* **Std (표준편차)** 값이 `0.00xx` 수준으로 유지되면 매우 안정적입니다.
* 만약 로봇이 가만히 있는데 Std 값이 계속 증가하거나 튀면, SLAM의 스캔 매칭이 불안정하거나 IMU 드리프트가 보정되지 않고 있다는 신호입니다.
