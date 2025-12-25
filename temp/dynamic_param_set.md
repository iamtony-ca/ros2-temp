반갑습니다. ROS 2 Jazzy 및 Python 전문가로서 답변 드립니다.

Nav2에서 `xy_goal_tolerance`를 동적으로 변경하는 것은 로봇이 좁은 공간에 정밀하게 주차해야 할 때는 값을 줄이고, 넓은 공간을 빠르게 이동할 때는 값을 늘려 주행 효율을 높이는 데 매우 유용한 기술입니다.

이 기능은 **Parameter Client**를 사용하여 실행 중인 `controller_server` 노드의 파라미터를 업데이트하는 방식으로 구현합니다.

다음은 프로덕션 레벨에서 사용할 수 있도록 비동기 방식(`AsyncParameterClient`)을 적용한 Python 노드 예제입니다.

### 📋 주요 포인트

1. **타겟 노드:** `controller_server`
2. **타겟 파라미터:** 보통 두 가지를 함께 고려해야 합니다.
* **Goal Checker:** `general_goal_checker.xy_goal_tolerance` (실제 목표 도달 성공 여부를 판단)
* **Controller (Plugin):** `FollowPath.xy_goal_tolerance` (컨트롤러가 수렴하려고 노력하는 오차 범위)
* *Note: `general_goal_checker`나 `FollowPath`는 `nav2_params.yaml`에 정의된 플러그인 이름에 따라 달라질 수 있습니다.*



---

### 💻 Python Code: `dynamic_tolerance_client.py`

이 노드는 5초마다 정밀 모드(0.05m)와 일반 모드(0.30m)를 번갈아 가며 변경합니다.

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType
from rclpy.parameter_client import AsyncParameterClient
from rcl_interfaces.srv import SetParameters

class DynamicToleranceAdjuster(Node):
    def __init__(self):
        super().__init__('dynamic_tolerance_adjuster')

        # 타겟 노드 이름 (Nav2 Controller Server)
        self.target_node = 'controller_server'
        
        # AsyncParameterClient 생성
        self.client = AsyncParameterClient(self, self.target_node)
        
        # 서비스가 준비될 때까지 대기
        self.get_logger().info(f'Waiting for {self.target_node} to be available...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.get_logger().info(f'{self.target_node} is up! Starting dynamic adjustment.')

        # 테스트를 위한 타이머 설정 (5초마다 변경)
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.is_precise = False

    def timer_callback(self):
        # 모드 토글
        self.is_precise = not self.is_precise
        
        if self.is_precise:
            tolerance = 0.05  # 5cm (정밀)
            mode_str = "PRECISE"
        else:
            tolerance = 0.30  # 30cm (일반)
            mode_str = "RELAXED"

        self.get_logger().info(f'Attempting to set mode: {mode_str} (Tolerance: {tolerance})')
        self.set_goal_tolerance(tolerance)

    def set_goal_tolerance(self, tolerance_value):
        """
        controller_server의 파라미터를 비동기로 업데이트합니다.
        """
        # 주의: 여러분의 params.yaml 설정에 따라 플러그인 이름(접두사)이 다를 수 있습니다.
        # 일반적인 기본값: 'general_goal_checker' 및 'FollowPath'
        
        # 1. Goal Checker 업데이트 (성공 판정 기준)
        param_checker = Parameter(
            'general_goal_checker.xy_goal_tolerance',
            Parameter.Type.DOUBLE,
            tolerance_value
        )

        # 2. Controller Plugin 업데이트 (제어 알고리즘의 목표 기준 - 예: DWB, MPPI)
        # 필요에 따라 이 부분은 주석 처리하거나 본인의 컨트롤러 이름에 맞게 수정하세요.
        param_controller = Parameter(
            'FollowPath.xy_goal_tolerance',
            Parameter.Type.DOUBLE,
            tolerance_value
        )

        # 파라미터 변경 요청 전송
        future = self.client.set_parameters([param_checker, param_controller])
        future.add_done_callback(self.callback_param_update)

    def callback_param_update(self, future):
        try:
            result = future.result()
            # 결과 확인 (각 파라미터별 성공 여부)
            for res in result:
                if res.successful:
                    self.get_logger().info('Successfully updated parameter.')
                else:
                    self.get_logger().warn(f'Failed to update parameter: {res.reason}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DynamicToleranceAdjuster()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

```

---

### 💡 전문가의 조언 (Tips for ROS 2 Jazzy & Nav2)

1. **Goal Checker vs Controller:**
* 초보자들이 흔히 하는 실수는 **Controller** 파라미터(`FollowPath.xy_goal_tolerance`)만 바꾸는 것입니다.
* Nav2는 **Goal Checker**(`general_goal_checker`)가 "도착했다"고 판단해야 액션이 성공으로 끝납니다. 따라서 **Goal Checker의 값을 우선적으로 변경**해야 하며, 컨트롤러가 그 정밀도를 따라갈 수 있도록 컨트롤러 값도 같이 맞춰주는 것이 가장 이상적입니다.


2. **플러그인 이름 확인:**
* `nav2_params.yaml` 파일에서 `controller_server` 섹션을 확인하세요.
* 만약 `GoalChecker` 이름을 `precise_goal_checker`로 설정했다면, 파라미터 이름은 `precise_goal_checker.xy_goal_tolerance`가 됩니다.


3. **Behavior Tree (BT)에서의 접근:**
* Python 노드로 외부에서 제어하는 것도 좋지만, 복잡한 시나리오에서는 Nav2 Behavior Tree XML 내부에서 **Context**나 **Blackboard**를 이용해 동적으로 Goal Checker를 교체하는 방식이 시스템적으로 더 깔끔할 수 있습니다.



**다음 단계로 무엇을 도와드릴까요?**
이 코드를 실제 로봇의 `params.yaml` 설정에 맞춰 수정하는 법을 알려드리거나, 혹은 특정 토픽(예: `/current_mode`)을 구독하여 이벤트를 트리거하는 방식으로 코드를 확장해 드릴까요?
