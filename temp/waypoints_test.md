네, 알겠습니다. `get_current_pose` 메서드를 수정하여, 선택적으로 현재 로봇의 방향(yaw)을 지정된 각도만큼 회전시킨 후 `PoseStamped`를 반환하도록 기능을 확장해 드리겠습니다.

다음은 수정된 전체 스크립트입니다. 주요 변경 사항은 다음과 같습니다.

1.  **`quaternion_to_yaw` 함수 추가:** `geometry_msgs/Quaternion`에서 yaw 각도를 추출하기 위한 헬퍼 함수입니다.
2.  **`get_current_pose` 메서드 시그니처 변경:** `rotation_angle_deg: float = 0.0`와 `direction: str = 'ccw'` 두 개의 선택적 파라미터를 추가했습니다.
3.  **회전 로직 추가:** `get_current_pose` 내부에서 `rotation_angle_deg` 값이 0보다 크면, TF에서 얻어온 현재 방향을 yaw로 변환하고, 요청된 각도만큼 더하거나 뺀 후, 다시 Quaternion으로 변환하여 `pose`에 설정하는 로직을 추가했습니다.

`run_shuttle` 부분은 변경하지 않았으므로, 기본 동작은 이전과 동일합니다. 하지만 이제 필요에 따라 `get_current_pose(90.0, 'cw')`와 같이 호출하여 회전 기능을 사용할 수 있습니다.

-----

### 🤖 Nav2 왕복 주행 Python 스크립트 (회전 기능 추가)

```python
#!/usr/bin/env python3
import rclpy
import time
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateThroughPoses
from action_msgs.msg import GoalStatus
import tf2_ros
from tf2_ros import TransformException

# --- 헬퍼 함수 ---

def yaw_to_quaternion(yaw: float) -> Quaternion:
    """ Z축 Yaw (라디안) 값을 Quaternion 메시지로 변환합니다. """
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def quaternion_to_yaw(q: Quaternion) -> float:
    """ Quaternion 메시지에서 Z축 Yaw (라디안) 값을 추출합니다. """
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

# --- 노드 클래스 ---

class WaypointShuttleNode(Node):
    """
    NavigateThroughPoses 액션을 사용하여 지정된 웨이포인트 사이를 왕복하는 노드.
    각 여정의 시작점은 TF를 통해 동적으로 현재 로봇의 위치를 가져와 설정하며,
    필요 시 현재 방향을 회전시킬 수 있습니다.
    """

    def __init__(self, node_name: str = 'waypoint_shuttle_node'):
        super().__init__(node_name)
        
        # --- 사용자 설정 ---
        self.robot_base_frame = 'base_link'
        self.global_frame = 'map'
        self.waypoints = {
            'A': (0.0, 0.0, 0.0),
            'B': (5.0, 0.0, 0.0),
            'C': (5.0, 5.0, 1.57),
            'D': (0.0, 5.0, 3.14)
        }
        # --- 사용자 설정 끝 ---

        self.static_poses = {name: self.create_pose_stamped(x, y, yaw) 
                             for name, (x, y, yaw) in self.waypoints.items()}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        self.action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        self.get_logger().info("웨이포인트 왕복 주행 노드가 초기화되었습니다.")

    def create_pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = yaw_to_quaternion(yaw)
        return pose

    def get_current_pose(self, rotation_angle_deg: float = 0.0, direction: str = 'ccw') -> PoseStamped | None:
        """
        TF를 사용하여 로봇의 현재 위치를 가져옵니다.
        추가적으로 지정된 각도만큼 현재 방향(yaw)을 회전시킬 수 있습니다.

        Args:
            rotation_angle_deg (float): 회전시킬 각도 (도 단위). 기본값은 0 (회전 없음).
            direction (str): 회전 방향. 'cw' (시계 방향) 또는 'ccw' (반시계 방향).
        
        Returns:
            PoseStamped | None: (선택적으로 회전된) 현재 포즈 또는 실패 시 None.
        """
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                now,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            pose = PoseStamped()
            pose.header.stamp = transform.header.stamp
            pose.header.frame_id = self.global_frame
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            
            original_orientation = transform.transform.rotation

            # --- 회전 로직 시작 ---
            if rotation_angle_deg > 0.0:
                dir_str = '반시계' if direction.lower() == 'ccw' else '시계'
                self.get_logger().info(f"현재 방향을 {rotation_angle_deg}도 {dir_str} 방향으로 회전합니다.")
                
                # 1. 원본 Quaternion을 Yaw로 변환
                current_yaw = quaternion_to_yaw(original_orientation)
                
                # 2. 회전 각도를 라디안으로 변환하고 방향에 따라 계산
                rotation_rad = math.radians(rotation_angle_deg)
                if direction.lower() == 'cw':
                    new_yaw = current_yaw - rotation_rad
                else:  # ccw (기본값)
                    new_yaw = current_yaw + rotation_rad
                
                # 3. 새로운 Yaw 값을 [-pi, pi] 범위로 정규화
                new_yaw = math.atan2(math.sin(new_yaw), math.cos(new_yaw))

                # 4. 새로운 Yaw를 다시 Quaternion으로 변환하여 pose에 설정
                pose.pose.orientation = yaw_to_quaternion(new_yaw)
            else:
                # 회전이 없으면 원본 orientation 사용
                pose.pose.orientation = original_orientation
            # --- 회전 로직 끝 ---
            
            self.get_logger().info(f"현재 위치 획득: (x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f})")
            return pose
        
        except TransformException as e:
            self.get_logger().error(f"TF 변환 중 오류 발생: {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"현재 위치 획득 중 알 수 없는 오류: {e}")
            return None

    def send_navigation_goal(self, poses: list[PoseStamped]) -> bool:
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("액션 서버 'navigate_through_poses'를 사용할 수 없습니다.")
            return False

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses
        goal_msg.behavior_tree = ""

        self.get_logger().info(f"{len(poses)}개의 웨이포인트로 주행 목표 전송 중...")

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        try:
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
        except Exception as e:
            self.get_logger().error(f'Goal 전송 중 예외 발생: {e}')
            return False

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("목표가 거부되었습니다.")
            return False

        self.get_logger().info("목표가 수락되었습니다. 결과 대기 중...")
        result_future = goal_handle.get_result_async()
        
        try:
            rclpy.spin_until_future_complete(self, result_future)
        except Exception as e:
            self.get_logger().error(f'결과 대기 중 예외 발생: {e}')
            return False

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("주행 목표 성공!")
            return True
        else:
            self.get_logger().error(f"주행 목표 실패. 상태: {status}")
            return False

    def run_shuttle(self):
        """ 메인 왕복 주행 로직을 실행합니다. """
        while rclpy.ok():
            # --- 1. 정방향 주행 (A -> B -> C -> D) ---
            self.get_logger().info("정방향 주행 시작 (A -> B -> C -> D)")
            
            # 예시: 정방향 출발 시 현재 위치에서 반시계 방향으로 45도 회전한 방향을 시작 방향으로 설정
            # current_pose_a = self.get_current_pose(rotation_angle_deg=45.0, direction='ccw')
            current_pose_a = self.get_current_pose() # 기본 (회전 없음)
            
            if current_pose_a is None:
                self.get_logger().warn("현재 위치를 가져올 수 없어 2초 후 재시도합니다.")
                time.sleep(2.0)
                continue

            forward_poses = [current_pose_a, self.static_poses['B'], self.static_poses['C'], self.static_poses['D']]
            if not self.send_navigation_goal(forward_poses):
                self.get_logger().warn("정방향 주행에 실패했습니다. 2초 후 재시도합니다.")
                time.sleep(2.0)
                continue
            
            self.get_logger().info("정방향 주행 완료. 5초 후 역방향 주행 시작...")
            time.sleep(5.0)

            # --- 2. 역방향 주행 (D -> C -> B -> A) ---
            self.get_logger().info("역방향 주행 시작 (D -> C -> B -> A)")
            
            # 예시: 역방향 출발 시 현재 위치에서 시계 방향으로 180도 회전한 방향을 시작 방향으로 설정
            # current_pose_d = self.get_current_pose(rotation_angle_deg=180.0, direction='cw')
            current_pose_d = self.get_current_pose() # 기본 (회전 없음)

            if current_pose_d is None:
                self.get_logger().warn("현재 위치를 가져올 수 없어 2초 후 재시도합니다.")
                time.sleep(2.0)
                continue

            backward_poses = [current_pose_d, self.static_poses['C'], self.static_poses['B'], self.static_poses['A']]
            if not self.send_navigation_goal(backward_poses):
                self.get_logger().warn("역방향 주행에 실패했습니다. 2초 후 재시도합니다.")
                time.sleep(2.0)
                continue
            
            self.get_logger().info("역방향 주행 완료. 5초 후 정방향 주행 시작...")
            time.sleep(5.0)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointShuttleNode()
    try:
        node.run_shuttle()
    except KeyboardInterrupt:
        node.get_logger().info("키보드 인터럽트로 종료")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
```
