네, 현재 코드도 기능적으로는 잘 작동하겠지만, **실제 로봇 주행(Real-world Driving)** 시 발생할 수 있는 **"떨림(Chattering)"** 현상과 **"수렴 실패(Stuck)"** 문제를 해결하기 위해 3가지 로직 개선을 추천합니다.

아래 다이어그램은 개선된 로직이 어떻게 상태를 전환하며 안정적인 제어를 수행하는지 보여줍니다.

### 🚀 개선해야 할 로직 3가지

#### 1. 히스테리시스 (Hysteresis) 적용 [가장 중요]

* **문제:** 현재는 `if abs(cte) > 0.02`로 되어 있습니다. 만약 오차가 0.02 근처에서 `0.021 -> 0.019 -> 0.021`로 변하면, 제어기가 **Bypass(Nav2) ↔ Correction(Custom)** 모드를 1초에 수십 번 왔다 갔다 하며 로봇이 부르르 떨게 됩니다.
* **해결:** 진입 조건과 탈출 조건을 다르게 설정합니다.
* **진입 (Correction ON):** 오차 > **3cm**
* **탈출 (Correction OFF):** 오차 < **1cm**
* 이렇게 하면 한 번 보정을 시작하면 확실히 잡을 때까지 놓지 않습니다.



#### 2. 최소 기동 속도 (Minimum Creep Speed) 보장

* **문제:** Nav2 Controller는 Goal에 가까워질수록 속도를 0으로 줄입니다. 만약 Y 오차가 남았는데 선속도(v_x)가 0이 되어버리면, 로봇은 **제자리에서 회전만 하고 Y축 이동을 못 하는 상태(Stuck)**가 됩니다. (차동 구동 로봇은 전진해야만 Y를 바꿀 수 있기 때문입니다.)
* **해결:** Correction 모드가 활성화된 상태라면, 목표 지점에 도달할 때까지 **아주 느린 속도(예: 0.02 m/s)를 강제로 유지**시켜서 Y 오차를 수정할 "동력"을 줘야 합니다.

#### 3. 후진 주행 방지 (Safety)

* **문제:** 본 로직은 로봇이 "전진"한다고 가정하고 설계되었습니다. 만약 Nav2가 후진 명령(음수 속도)을 내릴 때 이 로직이 개입하면, 조향 방향이 반대가 되어 오차가 더 커질 수 있습니다.
* **해결:** `cmd_vel.linear.x < 0`인 경우 안전하게 **무조건 Bypass** 합니다.

---

### 🛠️ 최종 완성된 코드 (Advanced)

위 개선 사항을 모두 반영한 최종 코드입니다.

```python
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import math
import numpy as np
import tf_transformations

def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

class StrictLateralDockingNode(Node):
    def __init__(self):
        super().__init__('strict_lateral_docking_node')

        # --- Parameters ---
        self.path_length_threshold = 2.5    # 활성화 거리
        
        # [개선 1] 히스테리시스 임계값
        self.cte_enable_threshold = 0.03    # 3cm 벗어나면 개입 시작
        self.cte_disable_threshold = 0.01   # 1cm 이내로 들어와야 개입 종료
        
        self.max_yaw_deviation = math.radians(35.0) 
        self.cte_correction_gain = 2.5      
        self.angular_p_gain = 2.0           
        
        # [개선 2] 최소 기동 속도 (Creep Speed)
        self.min_creep_speed = 0.02 # Y보정을 위해 필요한 최소 전진 속도

        # --- State ---
        self.is_correcting = False  # 히스테리시스 상태 플래그
        
        # TF & Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.current_odom = None
        self.pruned_path = None
        self.latest_cmd_vel = Twist()
        self.latest_cmd_time = self.get_clock().now() # 통신 안전용
        self.path_frame_id = 'map'

        # Communication
        self.create_subscription(Path, '/plan_pruned', self.pruned_path_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_smoothed', self.cmd_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_input_monitor', 10)
        
        self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg): self.current_odom = msg
    def pruned_path_callback(self, msg): 
        self.pruned_path = msg
        self.path_frame_id = msg.header.frame_id
        
    def cmd_callback(self, msg): 
        self.latest_cmd_vel = msg
        self.latest_cmd_time = self.get_clock().now()

    def get_robot_pose_in_path_frame(self):
        if not self.current_odom: return None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.path_frame_id, self.current_odom.header.frame_id,
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
            input_pose = PoseStamped()
            input_pose.header = self.current_odom.header
            input_pose.pose = self.current_odom.pose.pose
            return do_transform_pose(input_pose, transform).pose
        except Exception: return None

    def calculate_path_error(self, robot_pose):
        if not self.pruned_path or len(self.pruned_path.poses) < 2:
            return 0.0, 0.0, 0.0

        path_arr = np.array([(p.pose.position.x, p.pose.position.y) for p in self.pruned_path.poses])
        robot_xy = np.array([robot_pose.position.x, robot_pose.position.y])
        dists = np.linalg.norm(path_arr - robot_xy, axis=1)
        min_idx = np.argmin(dists)
        
        if min_idx >= len(path_arr) - 1: min_idx = len(path_arr) - 2

        p_curr = path_arr[min_idx]
        p_next = path_arr[min_idx + 1]

        path_vec = p_next - p_curr
        path_yaw = math.atan2(path_vec[1], path_vec[0])

        q = robot_pose.orientation
        _, _, robot_yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        dx = robot_xy[0] - p_curr[0]
        dy = robot_xy[1] - p_curr[1]
        
        cte = -dx * math.sin(path_yaw) + dy * math.cos(path_yaw)
        return cte, path_yaw, robot_yaw

    def get_path_length(self):
        if not self.pruned_path: return 0.0
        coords = np.array([(p.pose.position.x, p.pose.position.y) for p in self.pruned_path.poses])
        if len(coords) < 2: return 0.0
        return np.sum(np.linalg.norm(coords[1:] - coords[:-1], axis=1))

    def control_loop(self):
        # 0. Safety Check: 입력 명령이 너무 오래되었으면 정지
        if (self.get_clock().now() - self.latest_cmd_time).nanoseconds > 0.5 * 1e9:
            self.cmd_pub.publish(Twist())
            return

        final_cmd = Twist()
        final_cmd.linear = self.latest_cmd_vel.linear
        final_cmd.angular = self.latest_cmd_vel.angular

        # [개선 3] 후진 중이면 로직 Bypass (전진 가정 로직이므로)
        if final_cmd.linear.x < 0.0:
            self.cmd_pub.publish(final_cmd)
            self.is_correcting = False
            return

        robot_pose = self.get_robot_pose_in_path_frame()
        if robot_pose is None or self.pruned_path is None:
            self.cmd_pub.publish(final_cmd)
            return

        path_len = self.get_path_length()

        # 1. Goal 근처 진입
        if path_len < self.path_length_threshold:
            
            cte, path_yaw, robot_yaw = self.calculate_path_error(robot_pose)
            abs_cte = abs(cte)

            # [개선 1] Hysteresis Logic
            # 현재 보정 중이 아니면 -> 진입 임계값(3cm) 체크
            if not self.is_correcting:
                if abs_cte > self.cte_enable_threshold:
                    self.is_correcting = True
            
            # 현재 보정 중이면 -> 탈출 임계값(1cm) 체크
            else:
                if abs_cte < self.cte_disable_threshold:
                    self.is_correcting = False

            # 2. 제어 적용 (Correcting State)
            if self.is_correcting:
                # Approach Angle Calculation
                approach_angle = math.atan2(self.cte_correction_gain * cte, 1.0)
                approach_angle = max(min(approach_angle, self.max_yaw_deviation), -self.max_yaw_deviation)
                
                target_yaw = path_yaw - approach_angle
                yaw_error = normalize_angle(target_yaw - robot_yaw)
                
                final_cmd.angular.z = self.angular_p_gain * yaw_error
                
                # [개선 2] Creep Speed Check
                # 감속은 하되, 최소한의 이동 속도는 보장 (멈추면 Y보정 불가)
                speed_factor = max(0.2, 1.0 - (abs_cte * 2.0))
                target_vx = final_cmd.linear.x * speed_factor
                
                # 만약 원래 명령 속도가 너무 낮지만, Y보정이 필요하다면 강제로 최저 속도 부여
                # (단, 원래 명령이 0.01 이상일 때만 -> 완전 정지 명령엔 반응 X가 안전)
                if final_cmd.linear.x > 0.001:
                    final_cmd.linear.x = max(target_vx, self.min_creep_speed)
                else:
                    final_cmd.linear.x = 0.0 # 원래 멈추라 했으면 멈춤

        else:
            # Goal 근처가 아니면 상태 리셋
            self.is_correcting = False

        self.cmd_pub.publish(final_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = StrictLateralDockingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

```

### 요약: 이 코드가 더 강력한 이유

1. **떨지 않습니다:** 1cm ~ 3cm 사이의 "회색 지대"를 두어 모드 전환이 부드럽고 안정적입니다.
2. **멈추지 않습니다:** Nav2가 도착했다고 판단해서 속도를 너무 줄여도, `min_creep_speed` 덕분에 Y 오차를 끝까지 맞추며 슬금슬금 들어갑니다.
3. **안전합니다:** 통신 끊김 체크와 후진 시 바이패스 로직이 추가되어 예기치 않은 급발진을 막습니다.
4. #######################
5. ############################
6. #############################
7. ###############################
8. 사용자님의 고민 포인트가 무엇인지 정확히 이해했습니다. 사용자님은 **"Nav2가 도착했다고 판단하고 멈춰버리기(Linear Vel = 0) 전에, 미리미리 y축을 맞춰놓고 싶다"**는 의도시군요.

결론부터 말씀드리면, **그 의도를 달성하기 위해서라도 `cte_enable_threshold`는 `xy_goal_tolerance`보다 작아야(더 엄격해야) 합니다.**

사용자님의 생각과 실제 로직 사이의 **오해(Gap)**가 있는 부분을 명확한 **상황 예시(시나리오)**로 풀어서 설명해 드릴게요.

---

### 🚨 오해의 핵심: "Threshold가 크면 미리 맞춘다?"

사용자님은 `cte_enable_threshold`가 크면(예: 5cm), 로봇이 Goal에 도달하기 전부터 미리 작동할 것이라고 생각하셨을 수 있습니다. 하지만 코드를 보면 `cte_enable_threshold`는 **"이 값보다 오차가 클 때만 내가 개입하겠다"**는 **트리거(Trigger)** 조건입니다.

#### 시나리오 A: 사용자님 생각대로 설정했을 때 (Threshold > Tolerance)

* **설정:**
* Nav2 Goal Tolerance: **0.03m (3cm)**
* My Node Enable Threshold: **0.05m (5cm)** (더 크게 설정)
* 현재 로봇 상태: Goal 도착 직전, **y 오차 0.04m (4cm)**



1. **Nav2 상태:** 아직 오차가 3cm보다 크므로(4cm), Nav2는 "아직 도착 안 했어"라고 판단하고 계속 제어를 시도합니다.
2. **My Node 상태:** 코드의 조건문 `if abs(cte) > 0.05`를 봅니다. 현재 오차가 0.04이므로 **거짓(False)**입니다.
* **결과:** "오차가 5cm도 안 되네? 난 안 나설게." 하고 **Bypass** 합니다.


3. **최종 결과:** 로봇은 Nav2(Graceful Controller)에만 의존해서 갑니다. 그러다 오차가 **0.029m (2.9cm)**가 되는 순간, Nav2는 **"3cm 이내 진입! 도착 완료!"**를 선언하고 **멈춥니다.**
* **문제점:** 결국 y 오차 **2.9cm**를 남긴 채 어정쩡하게 주행이 끝나버립니다.



---

#### 시나리오 B: 제 추천대로 설정했을 때 (Threshold < Tolerance)

* **설정:**
* Nav2 Goal Tolerance: **0.03m (3cm)**
* My Node Enable Threshold: **0.025m (2.5cm)** (더 작게 설정)
* 현재 로봇 상태: Goal 도착 직전, **y 오차 0.029m (2.9cm)**



1. **Nav2 상태:** "오차 2.9cm네? Tolerance(3cm) 안쪽이니까 이제 멈춰야지. **속도(Linear Vel) 0으로 줄여!**" (Graceful Controller의 정상 동작)
2. **My Node 상태:** Nav2가 멈추려고 하는 그 찰나에, 내 노드가 `if abs(cte) > 0.025`를 체크합니다.
* 현재 오차(0.029) > 설정값(0.025) 이므로 **참(True)**입니다.
* **동작:** "잠깐! Nav2는 만족했을지 몰라도, 나는 만족 못 해! **아직 2.5cm보다 오차가 크잖아!**"


3. **개입 (Override):**
* 내 노드가 `is_correcting = True`로 변합니다.
* Nav2가 보낸 `Linear Vel = 0` 명령을 무시하고, 코드에 있는 `min_creep_speed`(최저 기동 속도)를 주입합니다.
* 로봇은 멈추지 않고 **아주 느리게(0.02 m/s) 기어가면서 y축을 1cm(Disable threshold)까지 맞춥니다.**


4. **최종 결과:** y 오차가 0.01m(1cm) 미만이 된 후에야 비로소 로봇이 정지합니다.

---

### 🖼️ 그림으로 보는 "제어 데드존(Blind Zone)"

만약 `Threshold`를 `Tolerance`보다 크게 잡으면, 그 사이 구간이 **"관리되지 않는 사각지대"**가 됩니다.

```text
[ Case A: Threshold(5cm) > Tolerance(3cm) ] -> 비추천 ❌

   오차 0cm     오차 3cm       오차 5cm
      |-----------|--------------|-------------------> 오차 크기
      [  Nav2 OK  ] [   Nav2 제어   ]
                    [   Blind Zone ] [ Custom Node 제어 ]
                        ^^^^^^
             (Nav2는 도착했다고 멈추고,
              Custom Node는 개입 안 하는 구간 = 2.9cm 오차 발생)


[ Case B: Threshold(2.5cm) < Tolerance(3cm) ] -> 추천 ✅

   오차 0cm   오차 2.5cm  오차 3cm
      |-----------|---------|------------------------> 오차 크기
      [  Nav2 OK  ]
      [Custom OFF][     Custom Node 제어 (Override)      ]
                  ^^^^^^^^^^^^^^^^^^^^^^^^
            (Nav2가 멈추려 해도, Custom Node가
             2.5cm보다 크면 강제로 뺏어서 제어함 = 정밀 주차 성공)

```

### 💡 요약: "더 깐깐한 시어머니"가 되어야 합니다.

Nav2(`xy_goal_tolerance`)가 "이 정도면 됐어(3cm)"라고 할 때, 사용자님의 노드(`cte_enable_threshold`)가 **"무슨 소리야? 더 정확해야지(2.5cm)!"**라고 따지고 들어야 합니다.

* 그래서 **Enable Threshold**는 **Tolerance**보다 **반드시 작아야** Nav2가 놓치는 미세한 오차를 잡을 수 있습니다.
* "정밀하게 맞추면서 진입"하고 싶다면, 이 Threshold를 0.025가 아니라 **0.01(1cm)** 처럼 더 낮춰도 됩니다. (단, 너무 낮추면 진동이 생길 수 있어 2.5cm를 추천한 것입니다.)

```python
# [Parameter Setting]
# Goal Tolerance가 0.03이므로, 그보다 살짝 타이트하게 잡아서 개입 보장
self.cte_enable_threshold = 0.025   # 2.5cm 벗어나면 정밀 제어 시작

# 센서 노이즈를 고려해 1cm 이내로 들어오면 충분하다고 판단하고 놔줌
self.cte_disable_threshold = 0.010  # 1.0cm 이내면 Bypass
```

이 설명으로 오해가 풀리셨을까요? 이해가 되셨다면 이 로직을 그대로 유지하시는 것이 좋습니다.
######################  
######################  
########################  
네, 개발하신 `StrictLateralDockingNode`의 제어 로직에 대한 **상세 설명서**와 실제 로봇에 적용할 때 필요한 **튜닝 가이드**를 정리해 드립니다.

이 문서는 팀원들에게 공유하거나, 추후 유지보수 할 때 참고 자료로 활용하실 수 있도록 체계적으로 구성했습니다.

---

# 📘 제어 로직 설명서 (Control Logic Description)

이 노드는 **"Goal 근처 정밀 접근 제어기 (Precision Docking Supervisor)"**입니다. Nav2의 기본 제어기(Graceful Controller)가 목표 지점에 거의 도달했지만 y축 오차(Lateral Error)를 남기고 멈추려 할 때, **강제로 개입(Override)**하여 오차를 수정합니다.

### 1. 핵심 작동 원리: 히스테리시스 (Hysteresis)

로봇의 떨림(Chattering)을 방지하기 위해 **진입(ON) 조건**과 **해제(OFF) 조건**을 다르게 가져갑니다.

* **감시 모드 (Monitoring):** 로봇이 Goal 기준 2.5m 이내로 진입하면 활성화됩니다.
* **개입 시작 (Trigger):** y축 오차가 `2.5cm`를 넘으면 **"정밀 제어 모드"**가 켜집니다.
* **개입 유지 (Holding):** 오차가 줄어들어도 바로 끄지 않고, 오차가 `1.0cm` 미만이 될 때까지 제어권을 유지합니다.
* **해제 (Release):** 오차가 `1.0cm` 미만이 되면 제어권을 Nav2로 넘겨줍니다(Bypass).

### 2. 제어 알고리즘: Stanley-like Steering

y축 오차를 줄이기 위해 로봇이 취해야 할 **목표 헤딩(Target Heading)**을 계산합니다.

1. **Look-at Logic:** 오차(e_{lat})가 클수록 경로 쪽으로 고개를 더 많이 돌립니다. (\arctan 사용)
2. **Safety Clamp:** 하지만 로봇이 경로에서 수직으로 서지 않도록, 최대 꺾는 각도를 `35도`로 제한합니다.
3. **PID Rotation:** 현재 로봇의 헤딩을 계산된 \theta_{target}$으로 맞추기 위해 P-Control로 각속도($\omega)를 생성합니다.

### 3. 정지 방지: Creep Speed (기어가는 속도)

차동 구동(Differential Drive) 로봇은 제자리에서 y축 이동이 불가능합니다.

* Nav2가 "도착했으니 멈춰(v=0)"라고 명령해도, y축 보정이 필요하면 **최소 속도(`0.02 m/s`)를 강제로 주입**하여 로봇이 움직이면서 위치를 맞추게 합니다.

---

# 🎛️ 튜닝 가이드 (Tuning Guide)

로봇을 실제 환경에 투입했을 때, 아래 순서대로 파라미터를 조절하여 최적의 성능을 찾으세요.

### 0. 튜닝 전 준비

* **Global Plan:** 로봇이 Goal을 향해 직선으로 진입할 수 있도록 경로가 잘 나오는지 확인합니다.
* **기본 Nav2:** Graceful Controller가 Goal 근처까지는(약 0.5m 전방) 로봇을 잘 데려다 놓아야 합니다.

### Step 1. 활성화 시점 잡기 (Thresholds)

가장 먼저 노드가 **"제때 켜지고 제때 꺼지는지"** 확인합니다.

| 파라미터 | 추천값 | 튜닝 팁 |
| --- | --- | --- |
| `cte_enable_threshold` | **0.025** (2.5cm) | **Goal Tolerance(3cm)보다 반드시 작아야 함.** 너무 작으면(1cm) Goal 한참 전부터 로봇이 뱀처럼 꼬물거림. |
| `cte_disable_threshold` | **0.010** (1.0cm) | 너무 작으면(0.005) 센서 노이즈 때문에 영원히 안 꺼질 수 있음. 1cm 정도가 적당. |

### Step 2. 조향 민감도 조절 (Steering Gain)

로봇이 오차를 감지했을 때 **"얼마나 확 꺾어서 들어오느냐"**를 결정합니다.

| 파라미터 | 추천값 | 튜닝 팁 |
| --- | --- | --- |
| `cte_correction_gain` | **2.0 ~ 3.0** | **값을 키우면:** 경로 쪽으로 머리를 확 돌립니다 (반응 빠름).<br>

<br>**값을 줄이면:** 완만하게 들어옵니다.<br>

<br>👉 **증상:** 로봇이 경로를 지나쳐서 반대편으로 넘어가면(Overshoot) 값을 줄이세요. |
| `max_yaw_deviation` | **30~40도** (0.5~0.7 rad) | 로봇의 폭이 넓다면 좁은 길에서 벽을 칠 수 있으므로 각도를 줄이세요. |

### Step 3. 회전 반응 속도 (Angular P Gain)

목표 각도가 정해졌을 때, **"얼마나 빨리 고개를 돌리느냐"**입니다.

| 파라미터 | 추천값 | 튜닝 팁 |
| --- | --- | --- |
| `angular_p_gain` | **2.0 ~ 3.0** | **값을 키우면:** 각도 오차를 잡기 위해 휙휙 돕니다.<br>

<br>👉 **증상:** 로봇이 좌우로 부르르 떨면(Oscillation) 값을 줄이세요. |

### Step 4. 최저 속도 (Creep Speed) - [중요]

Goal에 도착했는데 y 오차를 못 잡고 멈춰버리는 것을 방지합니다.

| 파라미터 | 추천값 | 튜닝 팁 |
| --- | --- | --- |
| `min_creep_speed` | **0.02 ~ 0.05** (m/s) | **값을 키우면:** y를 맞추기도 전에 x축 Goal을 지나쳐버릴 수 있음.<br>

<br>**값을 줄이면:** 모터 마찰 때문에 로봇이 실제로 안 움직일 수 있음.<br>

<br>👉 로봇이 움직일 수 있는 **'최소한의 힘'**으로 설정하세요. |

---

# 🩺 트러블슈팅 (Troubleshooting)

현장에서 발생할 수 있는 증상과 해결책입니다.

| 증상 (Symptom) | 원인 (Cause) | 해결책 (Solution) |
| --- | --- | --- |
| **로봇이 Goal 앞에서 멈추고 제자리 회전만 함** | 선속도가 0이라 y축 이동 불가 | `min_creep_speed`를 조금 높여서(0.02 -> 0.03) 로봇이 전진하게 만드세요. |
| **로봇이 좌우로 심하게 뱀처럼 움직임 (Hunting)** | 보정 게인이 너무 큼 | `cte_correction_gain`을 낮추거나(2.5 -> 1.5), `angular_p_gain`을 낮추세요. |
| **로봇이 Goal을 지나쳐버림 (Overshoot)** | 보정 중 속도가 너무 빠름 | `speed_factor` 로직을 확인하거나, `min_creep_speed`가 너무 높은지 확인하세요. |
| **제어가 켜졌다 꺼졌다를 반복하며 떰** | 히스테리시스 구간이 너무 좁음 | `cte_enable`과 `cte_disable`의 차이를 더 벌리세요. (예: 2.5cm / 1.0cm 유지) |
| **Goal에 도착했는데 Nav2가 완료 선언을 안 함** | y는 맞췄는데 Yaw가 안 맞음 | Nav2의 **Yaw Tolerance**를 확인하세요. y를 맞추느라 로봇이 비스듬히 섰을 수 있습니다. (Nav2가 마지막에 제자리 회전으로 맞춥니다) |

### 💡 튜닝 팁: 시각화 (Visualization)

디버깅할 때 `rqt_plot`을 켜고 다음 데이터를 실시간으로 모니터링하면 아주 편합니다.

1. `/cmd_vel_input_monitor/linear/x` (내 노드가 주는 속도)
2. `/cmd_vel_input_monitor/angular/z` (내 노드가 주는 회전)
3. **계산된 CTE 값** (로그로 찍어서 확인)

이제 이 가이드를 바탕으로 실제 로봇에서 파라미터를 조금씩 만지면서 "쫀득한" 주행감을 찾아보시기 바랍니다!

###########################  
################################  
#################################  
`target_yaw` 부호를 바꿨더니 발산했다면, 원래의 부호(`-`)가 방향 자체는 맞았던 것입니다. 그런데도 **"원래 코드를 써도 오차가 더 커진다"**는 것은 **"제어 방식" 자체에 구조적인 문제**가 있다는 뜻입니다.

가장 유력한 원인은 **"가장 가까운 점(Nearest Point)만 보고 제어했기 때문"**입니다.

### 🔴 문제의 원인: "너무 가까운 곳을 보고 있다"

기존 코드는 로봇 바로 옆에 있는 **가장 가까운 경로점**을 기준으로 CTE를 계산하고 핸들을 꺾습니다.

* 이렇게 되면 로봇이 조금만 방향을 틀어도 CTE가 급격하게 변하고, 로봇은 **S자로 휘청거리거나(Oscillation)** 경로에 수렴하지 못하고 **밀려나는 현상**이 발생합니다.
* 사람이 운전할 때 범퍼 바로 밑을 보고 운전하면 차가 비틀거리는 것과 같습니다. **약간 앞(Look-ahead)**을 보고 운전해야 부드럽게 라인을 탑니다.

---

### 🟢 해결책: "Look-ahead (전방 주시) 방식"으로 변경

복잡한 Stanley(CTE 계산) 방식 대신, 훨씬 직관적이고 강력한 **Pure Pursuit 스타일의 Look-ahead 로직**으로 변경합시다.

1. **로직:** 경로 상에서 로봇보다 **약 0.3m ~ 0.5m 앞에 있는 점(Look-ahead Point)**을 찾습니다.
2. **제어:** 로봇이 **그 점을 바라보도록** 회전시킵니다.
3. **장점:** 이렇게 하면 자연스럽게 경로(y=0)로 부드럽게 빨려 들어갑니다. 수학적으로 수렴성이 훨씬 좋습니다.

### 🛠️ 개선된 코드 (Look-ahead 적용)

기존 `StrictLateralDockingNode` 클래스 내부의 메소드들을 아래 코드로 **완전히 교체**해 주세요.
(import 부분은 기존과 동일하게 유지하시면 됩니다.)

```python
    # ... (기존 import 및 __init__ 등은 유지) ...

    # [수정됨] Nearest Point가 아니라, 앞서 있는 점(Look-ahead)을 찾음
    def get_lookahead_point(self, robot_pose, lookahead_dist=0.4):
        """
        로봇 위치에서 경로상 전방 lookahead_dist 만큼 떨어진 점을 반환
        """
        if not self.pruned_path or len(self.pruned_path.poses) < 2:
            return None

        path_arr = np.array([(p.pose.position.x, p.pose.position.y) for p in self.pruned_path.poses])
        robot_xy = np.array([robot_pose.position.x, robot_pose.position.y])
        
        # 1. 가장 가까운 점 인덱스 찾기
        dists = np.linalg.norm(path_arr - robot_xy, axis=1)
        min_idx = np.argmin(dists)
        
        # 2. 거기서부터 경로를 따라가며 lookahead 거리만큼 떨어진 점 찾기
        curr_dist = 0.0
        target_pt = path_arr[min_idx] # Default: nearest
        
        for i in range(min_idx, len(path_arr) - 1):
            p1 = path_arr[i]
            p2 = path_arr[i+1]
            segment_len = np.linalg.norm(p2 - p1)
            
            if curr_dist + segment_len >= lookahead_dist:
                # 이 세그먼트 위에 타겟이 있음 (보간)
                ratio = (lookahead_dist - curr_dist) / segment_len
                target_pt = p1 + (p2 - p1) * ratio
                return target_pt # [x, y] 반환
            
            curr_dist += segment_len
            target_pt = p2 # 끝까지 못 찾으면 마지막 점

        return target_pt

    # [수정됨] 제어 루프: CTE 대신 Look-ahead 각도 추종
    def control_loop(self):
        # 0. Safety Check
        if (self.get_clock().now() - self.latest_cmd_time).nanoseconds > 0.5 * 1e9:
            self.cmd_pub.publish(Twist())
            return

        final_cmd = Twist()
        final_cmd.linear = self.latest_cmd_vel.linear
        final_cmd.angular = self.latest_cmd_vel.angular

        # 후진 시 Bypass
        if final_cmd.linear.x < 0.0:
            self.cmd_pub.publish(final_cmd)
            self.is_correcting = False
            return

        # 1. 로봇 위치 (Map Frame) 구하기
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_pose = Pose()
            robot_pose.position.x = trans.transform.translation.x
            robot_pose.position.y = trans.transform.translation.y
            robot_pose.position.z = trans.transform.translation.z
            robot_pose.orientation = trans.transform.rotation
            _, _, robot_yaw = tf_transformations.euler_from_quaternion(
                [robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w])
        except Exception:
            self.cmd_pub.publish(final_cmd)
            return

        if self.pruned_path is None: return

        path_len = self.get_path_length()

        # 2. Goal 진입 (2.5m 이내)
        if path_len < self.path_length_threshold:
            
            # --- Look-ahead Logic 적용 ---
            # 전방 0.4m 지점을 바라보게 함 (속도에 따라 가변적으로 해도 좋음)
            lookahead_pt = self.get_lookahead_point(robot_pose, lookahead_dist=0.4)
            
            if lookahead_pt is not None:
                # 타겟 점까지의 각도 계산
                dy = lookahead_pt[1] - robot_pose.position.y
                dx = lookahead_pt[0] - robot_pose.position.x
                target_yaw = math.atan2(dy, dx)
                
                # Yaw Error 계산
                yaw_error = normalize_angle(target_yaw - robot_yaw)
                
                # Y 오차 추정 (활성화 여부 판단용 - 근사치)
                # 현재 로봇이 타겟 벡터에서 얼마나 떨어져 있나 체크
                # 간단하게는 path 상의 nearest point와의 거리 사용
                path_arr = np.array([(p.pose.position.x, p.pose.position.y) for p in self.pruned_path.poses])
                robot_xy = np.array([robot_pose.position.x, robot_pose.position.y])
                min_dist = np.min(np.linalg.norm(path_arr - robot_xy, axis=1)) # 이것이 대략적인 y 오차
                
                # --- Hysteresis Logic ---
                if not self.is_correcting:
                    if min_dist > self.cte_enable_threshold: # 2.5cm
                        self.is_correcting = True
                else:
                    if min_dist < self.cte_disable_threshold: # 1.0cm
                        self.is_correcting = False

                # --- Control Execution ---
                if self.is_correcting:
                    # 1. 회전 제어 (Look-ahead point를 향해 돌기)
                    # P Gain을 적절히 조절 (너무 크면 흔들림)
                    final_cmd.angular.z = 2.0 * yaw_error 
                    
                    # 2. 최대 회전 속도 제한 (안전)
                    final_cmd.angular.z = max(min(final_cmd.angular.z, 0.6), -0.6)

                    # 3. Creep Speed (최소 속도 유지)
                    # 오차(yaw error)가 크면 속도를 줄이고, 방향이 맞으면 속도를 냄
                    if abs(yaw_error) > 0.2: # 각도가 많이 틀어졌으면 천천히
                        final_cmd.linear.x = self.min_creep_speed # 0.02
                    else:
                        # 각도가 대충 맞으면 조금 더 속도를 냄 (y축 수렴 가속화)
                        final_cmd.linear.x = max(final_cmd.linear.x, 0.05)

        else:
            self.is_correcting = False

        self.cmd_pub.publish(final_cmd)

```

### 💡 왜 이 방식이 더 좋을까요?

1. **자연스러운 수렴:** "내 옆에 있는 선(CTE)에 맞춰라!"라고 하면 로봇이 게걸음(Sway)을 칠 수 없어서 비틀거리지만, "저기 앞에 있는 점(Look-ahead)을 보고 가라!"라고 하면 자동차가 차선을 변경하듯 **자연스러운 곡선**을 그리며 합류합니다.
2. **부호 문제 없음:** `atan2(dy, dx)`를 사용하므로 좌표계나 방향 부호가 틀릴 일이 거의 없습니다. 무조건 타겟 점을 향해 돕니다.
3. **튜닝 포인트:**
* `lookahead_dist=0.4`: 이 값을 **늘리면(0.6)** 반응은 느리지만 주행이 부드러워지고, **줄이면(0.2)** 반응은 빠르지만 흔들릴 수 있습니다. **0.4m** 정도가 실내 주행로봇에 적당합니다.



이 코드로 변경하시면 기존의 "발산하거나 오차가 커지는 문제"가 확실히 잡힐 것입니다.
######################  
########################  
#########################  
정확한 지적입니다. 이전 코드는 **"각도(Heading)만 맞추려는 성향"**이 너무 강해서, 커브 길에서 lookahead 지점과의 각도 차이가 벌어지면 일단 멈추고 각도부터 맞추려는(Point Turn) 현상이 발생한 것입니다.

이를 해결하기 위해 자동차 주행 알고리즘의 정석인 **Pure Pursuit (순수 추종)의 곡률(Curvature) 제어 방식**을 제대로 적용해야 합니다.

이 방식은 단순히 "저 점을 바라봐!"가 아니라, **"저 점까지 부드러운 호(Arc)를 그리며 가라"**는 명령을 내리므로, 멈추지 않고 자연스럽게 곡선을 그리며 합류하게 됩니다.

---

### 🚀 개선 포인트: "회전"이 아니라 "곡선 주행"으로

1. **Lookahead Point를 로봇 기준 좌표로 변환:**
* Global Map 상의 목표점을 로봇이 봤을 때  어디에 있는지 계산합니다.


2. **곡률(Curvature) 계산:**
* 단순 각도 차이가 아니라, 목표점까지 도달하기 위한 원의 휘어짐 정도()를 계산합니다.
* 공식:  (: 거리, : 로봇 기준 좌우 거리)


3. **속도 유지 (Momentum):**
* 각도가 틀어졌다고 멈추는 게 아니라, **선속도를 유지하면서 곡률에 비례한 각속도를 주입**합니다. ()



---

### 🛠️ 부드러운 주행을 위한 최종 수정 코드

기존 `control_loop`와 헬퍼 함수를 아래 내용으로 교체해 주세요. 특히 **`transform_global_to_local`** 함수가 핵심입니다.

```python
    # ... (기존 import 유지) ...

    # [신규 추가] Global 좌표(Map)를 로봇 기준 Local 좌표(Base_link)로 변환
    def transform_global_to_local(self, global_pt, robot_pose):
        dx = global_pt[0] - robot_pose.position.x
        dy = global_pt[1] - robot_pose.position.y
        
        # Robot Quaternion -> Yaw
        import tf_transformations
        _, _, robot_yaw = tf_transformations.euler_from_quaternion(
            [robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w])

        # 회전 변환 (Rotation Matrix)
        # local_x: 로봇 정면 방향 거리, local_y: 로봇 좌우 방향 거리
        local_x = dx * math.cos(robot_yaw) + dy * math.sin(robot_yaw)
        local_y = -dx * math.sin(robot_yaw) + dy * math.cos(robot_yaw)
        
        return local_x, local_y

    def control_loop(self):
        # 0. Safety & Init
        if (self.get_clock().now() - self.latest_cmd_time).nanoseconds > 0.5 * 1e9:
            self.cmd_pub.publish(Twist())
            return

        final_cmd = Twist()
        final_cmd.linear = self.latest_cmd_vel.linear
        final_cmd.angular = self.latest_cmd_vel.angular

        if final_cmd.linear.x < 0.0: # 후진 시 Bypass
            self.cmd_pub.publish(final_cmd)
            self.is_correcting = False
            return

        # 1. 로봇 위치 (Map Frame) 직접 조회
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_pose = Pose()
            robot_pose.position.x = trans.transform.translation.x
            robot_pose.position.y = trans.transform.translation.y
            robot_pose.position.z = trans.transform.translation.z
            robot_pose.orientation = trans.transform.rotation
        except Exception:
            self.cmd_pub.publish(final_cmd)
            return

        if self.pruned_path is None: return
        path_len = self.get_path_length()

        # 2. Goal 진입 (활성화 구간)
        if path_len < self.path_length_threshold:
            
            # [튜닝 포인트 1] Lookahead 거리: 길수록 부드럽지만 반응 느림 (0.5 ~ 0.8 추천)
            L = 0.6 
            lookahead_pt = self.get_lookahead_point(robot_pose, lookahead_dist=L)
            
            if lookahead_pt is not None:
                # 3. Pure Pursuit Logic
                # 목표점을 로봇 기준 좌표계로 변환 (제일 중요!)
                local_x, local_y = self.transform_global_to_local(lookahead_pt, robot_pose)
                
                # 거리 제곱
                dist_sq = local_x**2 + local_y**2
                
                # 오차(Lateral Error) 추정 -> Hysteresis 판단용
                # local_y가 곧 로봇 기준의 횡방향 오차 근사치임
                current_cte = abs(local_y)

                # --- Hysteresis Logic ---
                if not self.is_correcting:
                    if current_cte > self.cte_enable_threshold: # 2.5cm
                        self.is_correcting = True
                else:
                    if current_cte < self.cte_disable_threshold: # 1.0cm
                        self.is_correcting = False

                if self.is_correcting:
                    # 4. 곡률(Curvature) 계산: kappa = 2 * y / L^2
                    # 로봇이 원을 그리며 목표점에 도달하기 위한 곡률
                    curvature = 2.0 * local_y / dist_sq
                    
                    # 5. 속도 프로파일 (부드러운 주행 핵심)
                    # Nav2가 감속했더라도, 보정을 위해 최소 속도(Creep Speed)는 유지
                    target_v = max(final_cmd.linear.x, self.min_creep_speed)
                    
                    # [튜닝 포인트 2] 곡률이 너무 크면(급커브) 속도를 살짝 줄임 (안전)
                    # 곡률 1.0(반지름 1m)일 때 속도 80% 유지, 더 급하면 더 줄임
                    scaling_factor = max(0.3, 1.0 - (abs(curvature) * 0.2))
                    target_v = target_v * scaling_factor

                    # 6. 각속도 명령 생성 (w = v * k)
                    target_w = target_v * curvature

                    # 7. 최종 명령 적용
                    final_cmd.linear.x = target_v
                    
                    # 각속도 제한 (급격한 회전 방지)
                    final_cmd.angular.z = max(min(target_w, 0.8), -0.8)

        else:
            self.is_correcting = False

        self.cmd_pub.publish(final_cmd)

```

### 💡 무엇이 달라졌나요?

1. **각도 대신 `local_y` 사용:**
* 이전에는 `atan2`로 각도를 구해서 로봇을 그 자리에서 돌리려 했습니다.
* 지금은 로봇 기준 목표점이 왼쪽(`+y`)에 있는지 오른쪽(`-y`)에 있는지만 보고, **"핸들을 얼마나 꺾어야(Curvature) 저기로 자연스럽게 갈까?"**를 계산합니다.


2. **`w = v * curvature` 공식 적용:**
* 선속도()가 있어야 각속도()가 생깁니다. 즉, **움직이면서 돕니다.**
* 이로 인해 제자리에서 비비는(Point Turn) 현상이 사라지고, 부드러운 아치(Arch) 형태의 궤적이 나옵니다.


3. **`local_x, local_y` 변환:**
* `transform_global_to_local` 함수를 통해 Global Map 좌표를 신경 쓰지 않고, 오직 **"내 몸 기준 왼쪽/오른쪽"**만 생각하므로 제어가 훨씬 직관적이고 안정적입니다.



### 🔬 튜닝 팁

* **코너에서 안쪽으로 너무 파고든다면?**
* `L = 0.6` (Lookahead Distance) 값을 조금 **줄이세요** (예: 0.5).


* **여전히 움직임이 끊기거나 부자연스럽다면?**
* `L` 값을 **키우세요** (예: 0.7 ~ 0.8). 멀리 볼수록 더 완만하게 움직입니다.


* **Goal Tolerance (3cm) 근처에서 계속 미세하게 움직인다면?**
* `min_creep_speed`를 아주 낮게(0.01) 잡거나, `cte_disable_threshold`를 조금 더 넉넉하게(1.5cm) 잡으세요.



이제 커브 구간에서도 로봇이 멈칫하지 않고 물 흐르듯 라인을 타면서 Goal로 들어갈 것입니다.
##########################  
############################  
###############################  
사용자님의 요구사항은 **"Goal 5cm 이내(Last Inch)에 진입하면 를 모두 완벽하게 맞추는 정밀 주차 모드(Final Adjustment)를 추가"**하는 것입니다.

기존 코드는 `is_correcting` 플래그 하나로 축 보정만 담당했습니다. 이제 이를 확장하여 **두 단계(2-Stage) 제어 전략**을 적용해야 합니다.

1. **Stage 1 (접근 단계):** 기존 로직 유지. Look-ahead로 부드럽게 경로()를 맞추며 접근.
2. **Stage 2 (초근접 단계):** Goal 기준 거리 0.05m 이내 진입 시 발동.  오차를 모두 0으로 수렴시킴.

---

### 🛠️ 기능 추가 및 개선된 코드

기존 코드를 바탕으로 **`control_loop`** 내부에 **Stage 2 (Final Docking)** 로직을 추가했습니다.

```python
    # ... (기존 메소드들: get_lookahead_point, get_path_length 등은 그대로 사용) ...

    # [신규 추가] Goal까지 남은 유클리드 거리 계산
    def get_dist_to_global_goal(self, robot_pose):
        if not self.pruned_path or len(self.pruned_path.poses) == 0:
            return float('inf')
        # Pruned Path의 마지막 점이 곧 현재의 Global Goal Target임
        goal_pt = self.pruned_path.poses[-1].pose.position
        dx = goal_pt.x - robot_pose.position.x
        dy = goal_pt.y - robot_pose.position.y
        return math.hypot(dx, dy)

    def control_loop(self):
        # 0. Safety Check & Init
        current_time = self.get_clock().now()
        if (current_time - self.latest_cmd_time).nanoseconds > 0.5 * 1e9:
            self.cmd_pub.publish(Twist())
            return

        final_cmd = Twist()
        final_cmd.linear = self.latest_cmd_vel.linear
        final_cmd.angular = self.latest_cmd_vel.angular

        # 후진 시 Bypass
        if final_cmd.linear.x < 0.0:
            self.cmd_pub.publish(final_cmd)
            self.is_correcting = False
            return

        # 1. 로봇 위치 (Map Frame) 구하기
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_pose = Pose()
            robot_pose.position.x = trans.transform.translation.x
            robot_pose.position.y = trans.transform.translation.y
            robot_pose.position.z = trans.transform.translation.z
            robot_pose.orientation = trans.transform.rotation
            _, _, robot_yaw = tf_transformations.euler_from_quaternion(
                [robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w])
        except Exception:
            self.cmd_pub.publish(final_cmd)
            return

        if self.pruned_path is None: return

        # 거리 측정
        path_len = self.get_path_length()
        dist_to_goal = self.get_dist_to_global_goal(robot_pose)

        # =========================================================
        # [Stage 2] 초근접 정밀 보정 (0.05m 이내) -> X, Y 모두 보정
        # =========================================================
        if dist_to_goal < 0.05:
            # 여기서는 Lookahead 안 씀. 바로 Goal 점을 향해 감.
            goal_pt = self.pruned_path.poses[-1].pose.position
            dx = goal_pt.x - robot_pose.position.x
            dy = goal_pt.y - robot_pose.position.y
            
            # Goal 방향 각도
            target_yaw = math.atan2(dy, dx)
            yaw_error = normalize_angle(target_yaw - robot_yaw)
            
            # Yaw Tolerance Check (ex: 20도 이상 틀어져 있으면 회전만 함)
            if abs(yaw_error) > math.radians(20.0):
                final_cmd.linear.x = 0.0
                final_cmd.angular.z = 1.5 * yaw_error # 제자리 회전
            else:
                # 각도가 대충 맞으면 아주 천천히 전진하며 미세 조정
                # 남은 거리(dist_to_goal)가 0이 될 때까지 감
                final_cmd.linear.x = min(0.02, dist_to_goal) # 최대 0.02m/s, 가까워지면 더 감속
                final_cmd.angular.z = 2.0 * yaw_error # 가면서도 계속 방향 맞춤
            
            # Safety Clamp
            final_cmd.angular.z = max(min(final_cmd.angular.z, 0.5), -0.5)
            
            # [중요] 이 단계에서는 무조건 Custom 제어가 Nav2 명령을 덮어씀
            # Nav2가 멈추라고(0) 해도, 내가 0.05m 안에 있으면 밀고 들어감.
            self.is_correcting = True 

        # =========================================================
        # [Stage 1] 접근 단계 보정 (2.5m 이내 ~ 0.05m 이상) -> Y 위주 보정
        # =========================================================
        elif path_len < self.path_length_threshold:
            
            lookahead_pt = self.get_lookahead_point(robot_pose, lookahead_dist=0.4)
            
            if lookahead_pt is not None:
                dy = lookahead_pt[1] - robot_pose.position.y
                dx = lookahead_pt[0] - robot_pose.position.x
                target_yaw = math.atan2(dy, dx)
                yaw_error = normalize_angle(target_yaw - robot_yaw)
                
                # Y 오차(CTE) 추정
                path_arr = np.array([(p.pose.position.x, p.pose.position.y) for p in self.pruned_path.poses])
                robot_xy = np.array([robot_pose.position.x, robot_pose.position.y])
                min_dist = np.min(np.linalg.norm(path_arr - robot_xy, axis=1))

                # Hysteresis Logic
                if not self.is_correcting:
                    if min_dist > self.cte_enable_threshold: 
                        self.is_correcting = True
                else:
                    if min_dist < self.cte_disable_threshold: 
                        self.is_correcting = False

                if self.is_correcting:
                    final_cmd.angular.z = 2.0 * yaw_error 
                    final_cmd.angular.z = max(min(final_cmd.angular.z, 0.6), -0.6)

                    if abs(yaw_error) > 0.2: 
                        final_cmd.linear.x = self.min_creep_speed 
                    else:
                        final_cmd.linear.x = max(final_cmd.linear.x, 0.05)
        
        else:
            self.is_correcting = False

        self.cmd_pub.publish(final_cmd)

```

### 🧠 로직 상세 설명

1. **우선순위 역전 (Priority Inversion):**
* `if dist_to_goal < 0.05:` 조건을 가장 먼저 체크합니다. Goal 바로 앞에서는 기존의 Lookahead나 Path Length 조건보다 **"Goal 점으로의 직행"**이 최우선이기 때문입니다.


2. **Stage 2 (0.05m 이내) 동작 방식:**
* **Goal Point Tracking:** 더 이상 경로(Path)를 따라가는 게 아니라, 마지막 점(Target Point) 좌표 자체를 타겟으로 잡습니다.
* **제자리 회전 허용:** 만약 진입 각도가 너무 틀어져 있다면(20도 이상), `linear.x = 0`으로 두고 제자리에서 회전하여 Goal을 바라봅니다.
* **Micro Approach:** 각도가 맞으면 `0.02 m/s`라는 아주 느린 속도로 Goal 좌표에 겹칠 때까지( 오차 0 수렴) 밀고 들어갑니다.


3. **Nav2와의 관계:**
* 이 로직이 추가됨으로써, Nav2가 0.03m(Tolerance)에서 멈추려고 해도, 이 노드가 **"아직 0.05m 안쪽이니까 내가 마무리할게"** 하고 제어권을 가져와서 0.00m가 될 때까지 주차를 완료합니다.



이 코드를 적용하면 축 보정뿐만 아니라, 마지막 순간에 축 거리까지 정확하게 0으로 맞추는 **완전 정밀 주차**가 가능해집니다.

############################  
##############################  
###############################  
매우 좋은 지적입니다. **"완벽한 0.0"**을 목표로 하면 로봇은 목표 지점을 미세하게 지나치는 순간 다시 뒤로 가려 하고, 다시 앞으로 가려 하는 **무한 진동(Oscillation)**에 빠지게 됩니다.

따라서 **Final Stage(0.05m 이내)**에서도 **Disable Threshold(정지 허용 오차)**가 반드시 필요합니다.

또한, **후진 주행**의 경우, 기존 코드에 있던 `if linear.x < 0: return` 구문이 후진을 막고 있었으므로 이를 제거하고, 상황에 맞게 후진을 허용하거나 제어하도록 로직을 개선했습니다.

---

### 🚀 개선된 코드의 주요 변경점

1. **Oscillation 방지 (Deadband):** `final_xy_tolerance` (예: 5mm)를 추가했습니다. 이 범위 안에 들어오면 **강제로 (0, 0) 속도**를 주어 제자리에 멈추게 합니다.
2. **후진 주행 허용:** 무조건적인 후진 차단 로직을 삭제했습니다.
3. **Stage 2 (Final Docking) 로직 강화:**
* Goal을 향해 회전 후 전진합니다.
* 만약 Goal을 지나쳤다면(Overshoot), **후진**하는 것이 아니라 **제자리에서 회전하여 다시 Goal을 바라보고 전진**하는 방식(Turn & Drive)을 사용하여 더 직관적이고 안전하게 0.0을 맞춥니다.



### 🛠️ 최종 개선 코드

```python
    # ... (기존 import 등은 유지) ...

    def __init__(self):
        super().__init__('strict_lateral_docking_node')
        # ... (기존 파라미터들) ...
        
        # [신규] Final Stage (0.05m 이내) 정지 허용 오차
        # 이 값 이내로 들어오면 Oscillation 방지를 위해 정지(0,0) 처리
        self.final_xy_tolerance = 0.005  # 5mm

    # ... (get_dist_to_global_goal, get_path_length, get_lookahead_point 등 기존 함수 유지) ...

    def control_loop(self):
        # 0. Safety Check
        if (self.get_clock().now() - self.latest_cmd_time).nanoseconds > 0.5 * 1e9:
            self.cmd_pub.publish(Twist())
            return

        final_cmd = Twist()
        final_cmd.linear = self.latest_cmd_vel.linear
        final_cmd.angular = self.latest_cmd_vel.angular

        # [삭제됨] 기존의 '후진 시 무조건 Bypass' 코드를 삭제하여 후진 허용
        # if final_cmd.linear.x < 0.0: ... (삭제)

        # 1. 로봇 위치 및 경로 데이터 확인
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_pose = Pose()
            robot_pose.position.x = trans.transform.translation.x
            robot_pose.position.y = trans.transform.translation.y
            robot_pose.position.z = trans.transform.translation.z
            robot_pose.orientation = trans.transform.rotation
            _, _, robot_yaw = tf_transformations.euler_from_quaternion(
                [robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w])
        except Exception:
            self.cmd_pub.publish(final_cmd)
            return

        if self.pruned_path is None: return

        path_len = self.get_path_length()
        dist_to_goal = self.get_dist_to_global_goal(robot_pose)

        # =========================================================
        # [Stage 2] Final Docking (0.05m 이내) -> X, Y 정밀 보정
        # =========================================================
        if dist_to_goal < 0.05:
            
            # [Oscillation 방지] 목표 오차(5mm) 이내 진입 시 강제 정지
            if dist_to_goal < self.final_xy_tolerance:
                final_cmd.linear.x = 0.0
                final_cmd.angular.z = 0.0
                self.is_correcting = True # 내가 제어 중임 (정지 명령 유지)
                
            else:
                # Goal Point 좌표
                goal_pt = self.pruned_path.poses[-1].pose.position
                dx = goal_pt.x - robot_pose.position.x
                dy = goal_pt.y - robot_pose.position.y
                
                # Goal을 바라보는 각도 계산
                target_yaw = math.atan2(dy, dx)
                yaw_error = normalize_angle(target_yaw - robot_yaw)
                
                # [로직] 제자리 회전 후 전진 (Turn & Drive)
                # 오차가 클 때는 제자리 회전만 수행
                if abs(yaw_error) > math.radians(10.0): # 10도 이상 틀어지면
                    final_cmd.linear.x = 0.0
                    final_cmd.angular.z = 2.0 * yaw_error # P제어
                else:
                    # 각도가 맞으면 아주 천천히 접근
                    # 거리에 비례하여 속도 줄임 (Soft Landing)
                    target_speed = min(0.05, dist_to_goal * 2.0) 
                    final_cmd.linear.x = max(target_speed, 0.01) # 최소 1cm/s는 유지
                    final_cmd.angular.z = 2.5 * yaw_error # 가면서도 방향 유지
                
                # 각속도 제한
                final_cmd.angular.z = max(min(final_cmd.angular.z, 0.8), -0.8)
                self.is_correcting = True

        # =========================================================
        # [Stage 1] Approach (경로 추종 및 Y 보정)
        # =========================================================
        elif path_len < self.path_length_threshold:
            
            # [후진 주행 처리]
            # 만약 Nav2가 후진 명령(음수)을 내리고 있고, 아직 정밀 보정(is_correcting) 상태가 아니라면
            # Nav2의 후진 의도를 존중하여 Bypass 함.
            if final_cmd.linear.x < 0.0 and not self.is_correcting:
                self.cmd_pub.publish(final_cmd)
                return

            lookahead_pt = self.get_lookahead_point(robot_pose, lookahead_dist=0.4)
            
            if lookahead_pt is not None:
                # Global -> Local 변환 (Pure Pursuit용)
                local_x, local_y = self.transform_global_to_local(lookahead_pt, robot_pose)
                
                # Local y가 곧 CTE(Lateral Error)
                current_cte = abs(local_y)
                
                # Hysteresis Logic
                if not self.is_correcting:
                    if current_cte > self.cte_enable_threshold:
                        self.is_correcting = True
                else:
                    if current_cte < self.cte_disable_threshold:
                        self.is_correcting = False

                if self.is_correcting:
                    # 곡률 계산 및 적용
                    dist_sq = local_x**2 + local_y**2
                    curvature = 2.0 * local_y / dist_sq if dist_sq > 0 else 0
                    
                    # 속도: Nav2 명령 유지하되 최소 속도 보장 (단, 전진 시에만)
                    target_v = max(final_cmd.linear.x, self.min_creep_speed)
                    
                    # 급커브 시 감속
                    scaling = max(0.3, 1.0 - (abs(curvature) * 0.2))
                    target_v *= scaling
                    
                    final_cmd.linear.x = target_v
                    final_cmd.angular.z = target_v * curvature
                    final_cmd.angular.z = max(min(final_cmd.angular.z, 0.8), -0.8)

        else:
            self.is_correcting = False

        self.cmd_pub.publish(final_cmd)

```

### 💡 로직 핵심 포인트

1. **`final_xy_tolerance` (5mm):**
* Goal과의 거리가 5mm 미만이 되면 `linear.x = 0, angular.z = 0`을 쏘아서 로봇을 진정시킵니다.
* 이게 없으면 로봇이 0.0을 맞추려고 미세하게 앞뒤로 움직이는 진동이 발생합니다.


2. **후진 정책 (Reverse Policy):**
* **Stage 1 (접근 중):** Nav2가 후진(`v < 0`)을 명령하면, 우리 노드가 개입하지 않는 상태(`not is_correcting`)일 때는 그대로 통과시킵니다. (장애물 회피나 경로 수정 의도 존중)
* **Stage 2 (Final):** 여기서는 무조건 **"Goal을 바라보고 전진"**합니다. 만약 로봇이 Goal을 살짝 지나쳤다면, 뒤로 가는 게 아니라 제자리에서 180도(또는 필요한 만큼) 돌아서 다시 앞으로 가서 맞춥니다. 이 방식이 차동 구동 로봇의 정밀 위치 보정에 가장 확실합니다.
########################
##########################
##########################
사용자님의 요구사항을 완벽하게 이해했습니다. 핵심 변경 사항은 다음과 같습니다.

1. **후진 허용:** Goal을 지나쳤거나(Overshoot), 애초에 후진으로 진입하는 경로라면 **축 속도를 음수(`-`)로 주어 후진**합니다. (제자리 회전 금지)
2. **성공 조건(Tolerance) 우선:** Goal을 지나쳤더라도, 직선 거리상 `final_xy_tolerance` 안이면 멈춥니다.
3. **통합 제어:** 전진/후진을 별도 로직으로 분리하지 않고, **Local 좌표계의 값**을 기준으로 속도의 부호를 결정하여 자연스럽게 동작하게 합니다.

이를 위해 **Stage 2 (Final Docking)** 로직을 **"Local Coordinate P-Control"** 방식으로 전면 수정했습니다.

---

### 🛠️ 최종 완성 코드 (후진 지원 & 오버슈트 대응)

```python
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Path, Odometry
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import math
import numpy as np
import tf_transformations

def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

class StrictLateralDockingNode(Node):
    def __init__(self):
        super().__init__('strict_lateral_docking_node')

        # --- Parameters ---
        self.path_length_threshold = 2.0    # 접근 제어 시작 거리
        self.cte_enable_threshold = 0.025   # Y오차 2.5cm 이상 시 개입
        self.cte_disable_threshold = 0.010  # Y오차 1.0cm 이하 시 해제
        self.final_xy_tolerance = 0.01      # [중요] 1cm 이내면 성공으로 간주하고 정지
        
        self.min_creep_speed = 0.02         # 최소 기동 속도
        
        # --- State ---
        self.is_correcting = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.latest_cmd_vel = Twist()
        self.latest_cmd_time = self.get_clock().now()
        self.pruned_path = None
        
        # --- Pub/Sub ---
        self.create_subscription(Path, '/plan_pruned', self.pruned_path_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_smoothed', self.cmd_callback, 10)
        self.create_subscription(Odometry, '/odom', lambda msg: None, 10) # TF 조회하므로 더미 처리
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_input_monitor', 10)
        self.create_timer(0.05, self.control_loop)

    def pruned_path_callback(self, msg): self.pruned_path = msg
    def cmd_callback(self, msg): 
        self.latest_cmd_vel = msg
        self.latest_cmd_time = self.get_clock().now()

    def get_path_length(self):
        if not self.pruned_path or len(self.pruned_path.poses) < 2: return 0.0
        coords = np.array([(p.pose.position.x, p.pose.position.y) for p in self.pruned_path.poses])
        return np.sum(np.linalg.norm(coords[1:] - coords[:-1], axis=1))

    def get_dist_to_global_goal(self, robot_pose):
        if not self.pruned_path or len(self.pruned_path.poses) == 0: return float('inf')
        goal_pt = self.pruned_path.poses[-1].pose.position
        return math.hypot(goal_pt.x - robot_pose.position.x, goal_pt.y - robot_pose.position.y)

    # [핵심 1] Global 좌표 -> Robot Base 좌표로 변환
    def transform_global_to_local(self, global_pt, robot_pose):
        dx = global_pt[0] - robot_pose.position.x
        dy = global_pt[1] - robot_pose.position.y
        
        import tf_transformations
        _, _, robot_yaw = tf_transformations.euler_from_quaternion(
            [robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w])

        # 회전 변환 (Rotation Matrix)
        # local_x가 양수면 전방, 음수면 후방에 위치
        local_x = dx * math.cos(robot_yaw) + dy * math.sin(robot_yaw)
        local_y = -dx * math.sin(robot_yaw) + dy * math.cos(robot_yaw)
        
        return local_x, local_y

    def get_lookahead_point(self, robot_pose, lookahead_dist=0.4):
        # (기존 코드와 동일하므로 생략 - 위에서 제공한 보간 로직 그대로 사용)
        if not self.pruned_path or len(self.pruned_path.poses) < 2: return None
        path_arr = np.array([(p.pose.position.x, p.pose.position.y) for p in self.pruned_path.poses])
        robot_xy = np.array([robot_pose.position.x, robot_pose.position.y])
        dists = np.linalg.norm(path_arr - robot_xy, axis=1)
        min_idx = np.argmin(dists)
        curr_dist = 0.0
        target_pt = path_arr[min_idx]
        for i in range(min_idx, len(path_arr) - 1):
            p1 = path_arr[i]; p2 = path_arr[i+1]
            seg_len = np.linalg.norm(p2 - p1)
            if curr_dist + seg_len >= lookahead_dist:
                ratio = (lookahead_dist - curr_dist) / seg_len
                return p1 + (p2 - p1) * ratio
            curr_dist += seg_len
            target_pt = p2
        return target_pt

    def control_loop(self):
        # Safety Check
        if (self.get_clock().now() - self.latest_cmd_time).nanoseconds > 0.5 * 1e9:
            self.cmd_pub.publish(Twist()); return

        final_cmd = Twist()
        final_cmd.linear = self.latest_cmd_vel.linear
        final_cmd.angular = self.latest_cmd_vel.angular

        # 1. 로봇 위치 (Map Frame)
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_pose = Pose()
            robot_pose.position.x = trans.transform.translation.x
            robot_pose.position.y = trans.transform.translation.y
            robot_pose.position.z = trans.transform.translation.z
            robot_pose.orientation = trans.transform.rotation
            _, _, robot_yaw = tf_transformations.euler_from_quaternion(
                [robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w])
        except Exception:
            self.cmd_pub.publish(final_cmd); return

        if self.pruned_path is None: return

        path_len = self.get_path_length()
        dist_to_goal = self.get_dist_to_global_goal(robot_pose)

        # =========================================================
        # [Stage 2] Final Docking (0.05m 이내) -> X, Y 정밀 보정
        # =========================================================
        if dist_to_goal < 0.05:
            
            # 1. Tolerance Check (성공 시 정지)
            if dist_to_goal < self.final_xy_tolerance:
                final_cmd.linear.x = 0.0
                final_cmd.angular.z = 0.0
                self.is_correcting = True
            
            else:
                # 2. Goal 좌표를 로봇 기준(Local)으로 변환
                goal_pt_global = [self.pruned_path.poses[-1].pose.position.x, 
                                  self.pruned_path.poses[-1].pose.position.y]
                
                local_x, local_y = self.transform_global_to_local(goal_pt_global, robot_pose)
                
                # 3. X축 제어 (전진/후진 결정)
                # local_x가 양수면 전진, 음수면(지나쳤으면) 후진
                # 거리에 비례하여 속도 조절 (P 제어)
                kp_dist = 1.5 
                target_vx = kp_dist * local_x 
                
                # 속도 제한 (최대 0.05 m/s, 최소 creep speed)
                # 방향(Sign) 유지하면서 min/max 적용
                speed_limit = 0.05
                if abs(target_vx) > speed_limit:
                    target_vx = math.copysign(speed_limit, target_vx)
                
                # 너무 느리면 최소 속도 보장 (방향 유지)
                if abs(target_vx) < self.min_creep_speed:
                     target_vx = math.copysign(self.min_creep_speed, target_vx)
                
                final_cmd.linear.x = target_vx

                # 4. 각도 제어 (Steering)
                # 후진 중일 때는 조향 로직이 달라져야 함
                # atan2(y, x)는 후진 시(x<0) +-pi에 가까워짐 -> 후방을 타겟으로 맞춤
                
                # 타겟 지점을 향하는 각도
                target_yaw_local = math.atan2(local_y, local_x)
                
                # 후진 시(local_x < 0)에는, 로봇의 "뒤쪽"이 타겟을 향해야 함.
                # atan2 결과가 pi 근처이므로, pi를 빼거나 더해서 0 근처로 만듦 (후방 기준 오차)
                if local_x < 0:
                    yaw_error = normalize_angle(target_yaw_local - math.pi)
                else:
                    yaw_error = target_yaw_local

                final_cmd.angular.z = 2.5 * yaw_error
                final_cmd.angular.z = max(min(final_cmd.angular.z, 0.8), -0.8)

                self.is_correcting = True

        # =========================================================
        # [Stage 1] Approach (Look-ahead)
        # =========================================================
        elif path_len < self.path_length_threshold:
            
            lookahead_pt = self.get_lookahead_point(robot_pose, lookahead_dist=0.4)
            
            if lookahead_pt is not None:
                local_x, local_y = self.transform_global_to_local(lookahead_pt, robot_pose)
                current_cte = abs(local_y)
                
                # Hysteresis
                if not self.is_correcting:
                    if current_cte > self.cte_enable_threshold: self.is_correcting = True
                else:
                    if current_cte < self.cte_disable_threshold: self.is_correcting = False

                if self.is_correcting:
                    # 곡률 제어
                    dist_sq = local_x**2 + local_y**2
                    curvature = 2.0 * local_y / dist_sq if dist_sq > 0 else 0
                    
                    # 속도: Nav2 명령 기반 (후진 명령이면 그대로 후진)
                    # 만약 Nav2가 멈췄는데 보정이 필요하면 -> 전진으로 가정하고 Creep Speed
                    if abs(final_cmd.linear.x) < 0.001:
                        target_v = self.min_creep_speed
                    else:
                        target_v = final_cmd.linear.x 
                        # 너무 느리면 최소 속도 보장 (부호 유지)
                        if abs(target_v) < self.min_creep_speed:
                            target_v = math.copysign(self.min_creep_speed, target_v)

                    # 급커브 감속
                    scaling = max(0.3, 1.0 - (abs(curvature) * 0.2))
                    target_v *= scaling
                    
                    final_cmd.linear.x = target_v
                    final_cmd.angular.z = target_v * curvature
                    final_cmd.angular.z = max(min(final_cmd.angular.z, 0.8), -0.8)
        else:
            self.is_correcting = False

        self.cmd_pub.publish(final_cmd)

```

### 💡 로직 동작 설명 (Why this works)

1. **후진 자동 감지 (`local_x`):**
* `transform_global_to_local` 함수를 통해 Goal이 로봇의 **앞(`local_x > 0`)**에 있는지 **뒤(`local_x < 0`)**에 있는지 판단합니다.
* Goal을 지나쳤다면 `local_x`는 음수가 되고, `target_vx`도 음수가 되어 **로봇은 자연스럽게 후진**합니다.


2. **후진 조향 로직 (`yaw_error`):**
* **전진 시:** 목표점이 약간 왼쪽에 있으면, 왼쪽으로 회전합니다.
* **후진 시:** 목표점이 로봇 뒤편 기준 약간 왼쪽에 있다면? 로봇 뒷꽁무니를 왼쪽으로 보내야 합니다. 이를 위해 `math.atan2` 값에서 `pi`를 빼주어 **"로봇 뒷면 기준 각도 오차"**를 계산합니다. 이로 인해 후진 중에도 엉뚱한 방향으로 돌지 않고 Goal을 향해 정확히 뒷걸음질 칩니다.


3. **Tolerance 처리:**
* 로봇이 Goal을 살짝 지나쳤더라도(Overshoot), 그 거리가 `final_xy_tolerance` 안이라면 `dist_to_goal < tolerance` 조건에 걸려 즉시 정지합니다. 불필요한 후진을 하지 않습니다.



이 코드는 사용자의 "제자리 회전 금지", "후진 허용", "오버슈트 시 허용 범위 내 성공 처리" 요구사항을 모두 만족합니다.
############################  
##############################  
################################  
사용자님의 피드백을 정확히 이해했습니다. 수정해야 할 핵심 포인트는 두 가지입니다.

1. **정지 조건 (Stop Condition) 수정:**
* 기존:  오차만 작으면 정지.
* **수정:** ** 오차**와 ** 오차**가 **모두** 허용 범위(Tolerance) 내에 들어왔을 때만 `cmd_vel`을 0.0으로 만듭니다.
* **제어 방식:** Yaw를 맞추기 위해 강제로 제자리 회전을 시키지는 않되(사용자 요청), 주행하면서 자연스럽게 조건이 만족되기를 기다립니다.


2. **속도 저하 문제 (Speed Issue) 해결:**
* 기존: 곡률(Curvature)이 크면 안전을 위해 속도를 강제로 줄이는 로직(`scaling`)이 있었습니다. 이 때문에 Nav2가 0.5m/s를 줘도 제 노드가 0.2m/s로 깎아버리는 현상이 발생했습니다.
* **수정:** Nav2가 생성한 속도(`cmd_vel_smoothed`)를 **최대한 그대로 사용**합니다. 단, 정밀 제어를 위해 **'최소 속도(Creep Speed)'보다 느릴 때만** 속도를 올려줍니다. (감속 로직 제거)



---

### 🛠️ 최종 개선된 코드

이 코드는 **Yaw Tolerance를 체크**하며, **Nav2의 속도를 깎지 않고** 그대로 활용하여 부드럽고 빠르게 진입합니다.

```python
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Path, Odometry
import tf2_ros
import math
import numpy as np
import tf_transformations

def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

class StrictLateralDockingNode(Node):
    def __init__(self):
        super().__init__('strict_lateral_docking_node')

        # --- Parameters ---
        self.path_length_threshold = 2.0    # 접근 제어 시작 거리
        self.cte_enable_threshold = 0.025   # Y오차 2.5cm 이상 시 개입
        self.cte_disable_threshold = 0.010  # Y오차 1.0cm 이하 시 해제
        
        # [수정 1] 정지 조건 파라미터 (XY와 Yaw 모두 만족해야 정지)
        self.final_xy_tolerance = 0.01      # 1cm
        self.final_yaw_tolerance = 0.05     # 약 2.8도 (라디안)
        
        # [수정 2] 속도 관련 파라미터
        self.min_creep_speed = 0.02         # 보정을 위한 최소 속도
        
        # --- State ---
        self.is_correcting = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.latest_cmd_vel = Twist()
        self.latest_cmd_time = self.get_clock().now()
        self.pruned_path = None
        
        # --- Pub/Sub ---
        self.create_subscription(Path, '/plan_pruned', self.pruned_path_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_smoothed', self.cmd_callback, 10)
        self.create_subscription(Odometry, '/odom', lambda msg: None, 10) 
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_input_monitor', 10)
        self.create_timer(0.05, self.control_loop)

    def pruned_path_callback(self, msg): self.pruned_path = msg
    def cmd_callback(self, msg): 
        self.latest_cmd_vel = msg
        self.latest_cmd_time = self.get_clock().now()

    def get_path_length(self):
        if not self.pruned_path or len(self.pruned_path.poses) < 2: return 0.0
        coords = np.array([(p.pose.position.x, p.pose.position.y) for p in self.pruned_path.poses])
        return np.sum(np.linalg.norm(coords[1:] - coords[:-1], axis=1))

    def get_dist_to_global_goal(self, robot_pose):
        if not self.pruned_path or len(self.pruned_path.poses) == 0: return float('inf')
        goal_pt = self.pruned_path.poses[-1].pose.position
        return math.hypot(goal_pt.x - robot_pose.position.x, goal_pt.y - robot_pose.position.y)

    def transform_global_to_local(self, global_pt, robot_pose):
        dx = global_pt[0] - robot_pose.position.x
        dy = global_pt[1] - robot_pose.position.y
        
        import tf_transformations
        _, _, robot_yaw = tf_transformations.euler_from_quaternion(
            [robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w])

        local_x = dx * math.cos(robot_yaw) + dy * math.sin(robot_yaw)
        local_y = -dx * math.sin(robot_yaw) + dy * math.cos(robot_yaw)
        return local_x, local_y, robot_yaw

    def get_lookahead_point(self, robot_pose, lookahead_dist=0.4):
        if not self.pruned_path or len(self.pruned_path.poses) < 2: return None
        path_arr = np.array([(p.pose.position.x, p.pose.position.y) for p in self.pruned_path.poses])
        robot_xy = np.array([robot_pose.position.x, robot_pose.position.y])
        dists = np.linalg.norm(path_arr - robot_xy, axis=1)
        min_idx = np.argmin(dists)
        curr_dist = 0.0
        target_pt = path_arr[min_idx]
        for i in range(min_idx, len(path_arr) - 1):
            p1 = path_arr[i]; p2 = path_arr[i+1]
            seg_len = np.linalg.norm(p2 - p1)
            if curr_dist + seg_len >= lookahead_dist:
                ratio = (lookahead_dist - curr_dist) / seg_len
                return p1 + (p2 - p1) * ratio
            curr_dist += seg_len
            target_pt = p2
        return target_pt

    def control_loop(self):
        # Safety Check
        if (self.get_clock().now() - self.latest_cmd_time).nanoseconds > 0.5 * 1e9:
            self.cmd_pub.publish(Twist()); return

        # Nav2에서 오는 명령을 기본으로 사용
        final_cmd = Twist()
        final_cmd.linear = self.latest_cmd_vel.linear
        final_cmd.angular = self.latest_cmd_vel.angular

        # Robot Pose Lookup
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_pose = Pose()
            robot_pose.position.x = trans.transform.translation.x
            robot_pose.position.y = trans.transform.translation.y
            robot_pose.position.z = trans.transform.translation.z
            robot_pose.orientation = trans.transform.rotation
        except Exception:
            self.cmd_pub.publish(final_cmd); return

        if self.pruned_path is None: return

        path_len = self.get_path_length()
        dist_to_goal = self.get_dist_to_global_goal(robot_pose)

        # =========================================================
        # [Stage 2] Final Docking (0.05m 이내)
        # =========================================================
        if dist_to_goal < 0.05:
            
            # Goal Pose 계산 (Yaw 오차 확인용)
            goal_pose_global = self.pruned_path.poses[-1].pose
            _, _, goal_yaw = tf_transformations.euler_from_quaternion(
                [goal_pose_global.orientation.x, goal_pose_global.orientation.y, goal_pose_global.orientation.z, goal_pose_global.orientation.w])
            
            _, _, current_yaw = tf_transformations.euler_from_quaternion(
                [robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w])
            
            yaw_error = normalize_angle(goal_yaw - current_yaw)

            # [수정 1] 정지 조건 강화: XY 와 Yaw 모두 만족해야 0.0 전송
            xy_ok = dist_to_goal < self.final_xy_tolerance
            yaw_ok = abs(yaw_error) < self.final_yaw_tolerance

            if xy_ok and yaw_ok:
                final_cmd.linear.x = 0.0
                final_cmd.angular.z = 0.0
                self.is_correcting = True
            
            else:
                # 둘 중 하나라도 만족 못하면 계속 미세 조정 (X,Y 타겟으로 이동)
                goal_pt_global = [goal_pose_global.position.x, goal_pose_global.position.y]
                local_x, local_y, _ = self.transform_global_to_local(goal_pt_global, robot_pose)
                
                # P-Control for Linear Velocity (Local X)
                kp_dist = 1.5 
                target_vx = kp_dist * local_x 
                
                # 속도 제한 (최대 0.1 m/s - Final 단계이므로 너무 빠르면 안됨)
                speed_limit = 0.1 
                if abs(target_vx) > speed_limit:
                    target_vx = math.copysign(speed_limit, target_vx)
                
                # 최소 기동 속도 보장 (이동해야 보정이 되므로)
                if abs(target_vx) < self.min_creep_speed:
                     target_vx = math.copysign(self.min_creep_speed, target_vx)
                
                final_cmd.linear.x = target_vx

                # Steering Control (Target Point를 향해 조향)
                target_yaw_local = math.atan2(local_y, local_x)
                
                # 후진 시 조향 보정
                if local_x < 0:
                    steering_error = normalize_angle(target_yaw_local - math.pi)
                else:
                    steering_error = target_yaw_local

                final_cmd.angular.z = 2.5 * steering_error
                final_cmd.angular.z = max(min(final_cmd.angular.z, 0.8), -0.8)

                self.is_correcting = True

        # =========================================================
        # [Stage 1] Approach (Look-ahead)
        # =========================================================
        elif path_len < self.path_length_threshold:
            
            lookahead_pt = self.get_lookahead_point(robot_pose, lookahead_dist=0.4)
            
            if lookahead_pt is not None:
                local_x, local_y, _ = self.transform_global_to_local(lookahead_pt, robot_pose)
                current_cte = abs(local_y)
                
                # Hysteresis
                if not self.is_correcting:
                    if current_cte > self.cte_enable_threshold: self.is_correcting = True
                else:
                    if current_cte < self.cte_disable_threshold: self.is_correcting = False

                if self.is_correcting:
                    # [수정 2] 속도 로직 개선
                    # 곡률에 따른 강제 감속(Scaling) 삭제.
                    # Nav2가 주는 속도(final_cmd.linear.x)를 최대한 신뢰함.
                    
                    target_v = final_cmd.linear.x
                    
                    # 단, 보정 중인데 Nav2가 정지 명령(0)에 가깝게 주면 
                    # 로봇이 멈춰서 보정을 못하므로 '최소 속도'만 부여함 (Floor)
                    if abs(target_v) < self.min_creep_speed:
                         # Nav2가 멈추라고 했어도, 내가 보정 중이면 움직여야 함
                         # 원래 명령의 부호가 있으면 그 부호를 따르고, 없으면 전진으로 가정
                         if abs(target_v) < 0.001:
                             target_v = self.min_creep_speed # 기본 전진
                         else:
                             target_v = math.copysign(self.min_creep_speed, target_v)

                    # Pure Pursuit Steering
                    dist_sq = local_x**2 + local_y**2
                    curvature = 2.0 * local_y / dist_sq if dist_sq > 0 else 0
                    
                    final_cmd.linear.x = target_v
                    final_cmd.angular.z = target_v * curvature
                    final_cmd.angular.z = max(min(final_cmd.angular.z, 0.8), -0.8)

        else:
            self.is_correcting = False

        self.cmd_pub.publish(final_cmd)

```

### 🧐 주요 변경 사항 요약

**1. Stop Condition: "X, Y, Yaw 모두 만족해야 멈춤"**

* `if xy_ok and yaw_ok:` 조건을 사용하여, 목표 지점에 도달했더라도 각도가 틀어져 있다면 멈추지 않고 계속 미세 주행(Local X, Y 제어)을 합니다.
* 사용자 요청대로 각도 보정을 위한 별도의 `angular_z` 제어 로직을 넣지는 않았습니다. 대신 로봇이 목표점(Local 0,0)을 향해 계속 움직이면, 차동 구동 로봇 특성상 Yaw도 자연스럽게 맞춰집니다.

**2. 속도 개선: "Nav2 속도 신뢰"**

* 기존: `scaling = 1.0 - curvature...` 로직 때문에 커브에서 무조건 감속했습니다.
* **변경:** `target_v = final_cmd.linear.x`로 설정하여 **Nav2(Velocity Smoother)가 계산한 속도를 그대로 사용**합니다.
* 단, `is_correcting` 상태인데 속도가 너무 느리면(0.02 미만), 보정을 수행하기 위해 **최소 속도(Creep Speed)로만 올려주는(Floor)** 역할만 수행합니다. 이제 훨씬 시원하게 주행할 것입니다.
#########################
############################
#############################
사용자님의 요구사항을 반영하여 코드의 논리를 다음과 같이 구체화했습니다.

1. **XY 안정화 후 바이패스:** X, Y 오차가 허용 범위 내에서 특정 시간(예: 1.0초) 유지되면, `xy_completed` 상태가 되어 더 이상 개입하지 않고 Nav2(Graceful Controller)가 나머지 Yaw 제어를 하도록 둡니다.
2. **완전 정지 조건:** X, Y, Yaw가 모두 만족되면 강제로 `0.0`을 보냅니다.
3. **급격한 변동 방지 (Clamping):** 보정 로직이 계산한 속도가 Nav2가 원래 주려던 속도와 너무 큰 차이가 나지 않도록 제한(Clamp)을 겁니다.

수정이 필요한 **`__init__`** 부분과 **`control_loop` 내부의 Stage 2 (Final Docking)** 부분만 보여드립니다.

---

### 1. `__init__` 부분 (변수 추가)

```python
    def __init__(self):
        super().__init__('strict_lateral_docking_node')

        # --- Parameters ---
        self.path_length_threshold = 2.0
        self.cte_enable_threshold = 0.025
        self.cte_disable_threshold = 0.010
        
        # Tolerance
        self.final_xy_tolerance = 0.01      # 1cm
        self.final_yaw_tolerance = 0.05     # 약 2.8도
        
        # [신규] XY 안정화 판단 파라미터
        self.xy_stable_duration = 1.0       # 1초 동안 XY 오차 없으면 Bypass
        
        # [신규] 급격한 제어 변경 방지 (Nav2 명령과의 최대 차이 허용치)
        self.max_linear_diff = 0.05         # 5cm/s 이상 차이나게 급가속/감속 금지
        self.max_angular_diff = 0.5         # 0.5 rad/s 이상 급회전 차이 금지

        self.min_creep_speed = 0.02

        # --- State ---
        self.is_correcting = False
        
        # [신규] 안정화 타이머 상태 변수
        self.xy_stable_start_time = None    # XY 만족 시작 시각
        self.xy_completed = False           # XY 안정화 완료 여부 플래그

        # ... (나머지 초기화 코드는 동일) ...

```

---

### 2. `control_loop` 내부 (Stage 2 로직 전면 수정)

`control_loop` 함수 내에서 `if dist_to_goal < 0.05:` 블록을 아래 코드로 완전히 교체해 주세요.

```python
        # =========================================================
        # [Stage 2] Final Docking (0.05m 이내)
        # =========================================================
        if dist_to_goal < 0.05:
            
            # Goal Pose 및 Yaw 오차 계산
            goal_pose_global = self.pruned_path.poses[-1].pose
            _, _, goal_yaw = tf_transformations.euler_from_quaternion(
                [goal_pose_global.orientation.x, goal_pose_global.orientation.y, goal_pose_global.orientation.z, goal_pose_global.orientation.w])
            
            _, _, current_yaw = tf_transformations.euler_from_quaternion(
                [robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w])
            
            yaw_error = normalize_angle(goal_yaw - current_yaw)

            # 현재 상태 확인
            xy_satisfied = dist_to_goal < self.final_xy_tolerance
            yaw_satisfied = abs(yaw_error) < self.final_yaw_tolerance

            # -----------------------------------------------------
            # 조건 1: 모든 조건 만족 시 완전 정지 (cmd_vel = 0)
            # -----------------------------------------------------
            if xy_satisfied and yaw_satisfied:
                final_cmd.linear.x = 0.0
                final_cmd.angular.z = 0.0
                self.cmd_pub.publish(final_cmd)
                self.is_correcting = True # 정지 명령 강제 유지를 위해 True
                return

            # -----------------------------------------------------
            # 조건 2: XY Tolerance가 일정 시간 만족되면 Bypass (제어권 넘김)
            # -----------------------------------------------------
            if xy_satisfied:
                if self.xy_stable_start_time is None:
                    self.xy_stable_start_time = self.get_clock().now()
                
                # 경과 시간 계산
                elapsed = (self.get_clock().now() - self.xy_stable_start_time).nanoseconds / 1e9
                
                if elapsed > self.xy_stable_duration:
                    self.xy_completed = True
            else:
                # XY 오차가 다시 커지면 타이머 및 완료 플래그 리셋
                self.xy_stable_start_time = None
                self.xy_completed = False

            # XY 안정화가 완료되었다면 -> Nav2 명령 그대로 통과 (Bypass)
            if self.xy_completed:
                self.is_correcting = False 
                self.cmd_pub.publish(final_cmd) # Nav2 원래 명령 전송
                return

            # -----------------------------------------------------
            # 보정 로직 (XY가 아직 불안정하거나 만족 시간이 짧을 때)
            # -----------------------------------------------------
            
            # 1. 보정값 계산 (P-Control)
            goal_pt_global = [goal_pose_global.position.x, goal_pose_global.position.y]
            local_x, local_y, _ = self.transform_global_to_local(goal_pt_global, robot_pose)
            
            # Linear Velocity Calculation
            kp_dist = 1.5 
            calc_vx = kp_dist * local_x 
            
            # Limit & Creep Speed
            speed_limit = 0.1 
            if abs(calc_vx) > speed_limit:
                calc_vx = math.copysign(speed_limit, calc_vx)
            if abs(calc_vx) < self.min_creep_speed:
                 calc_vx = math.copysign(self.min_creep_speed, calc_vx)

            # Angular Velocity Calculation
            target_yaw_local = math.atan2(local_y, local_x)
            if local_x < 0:
                steering_error = normalize_angle(target_yaw_local - math.pi)
            else:
                steering_error = target_yaw_local

            calc_w = 2.5 * steering_error
            calc_w = max(min(calc_w, 0.8), -0.8)

            # -----------------------------------------------------
            # 조건 3: Clamp (Nav2 명령과 보정값의 차이 제한)
            # -----------------------------------------------------
            # Nav2가 주려는 값(final_cmd)과 내가 계산한 값(calc)의 차이 계산
            
            # Linear Clamping
            lin_diff = calc_vx - final_cmd.linear.x
            # 차이를 max_linear_diff 안으로 자름
            lin_diff = max(min(lin_diff, self.max_linear_diff), -self.max_linear_diff)
            final_cmd.linear.x = final_cmd.linear.x + lin_diff

            # Angular Clamping
            ang_diff = calc_w - final_cmd.angular.z
            # 차이를 max_angular_diff 안으로 자름
            ang_diff = max(min(ang_diff, self.max_angular_diff), -self.max_angular_diff)
            final_cmd.angular.z = final_cmd.angular.z + ang_diff

            self.is_correcting = True

```

### ✨ 변경된 로직 요약

1. **XY Stable Bypass:** `xy_satisfied` 상태가 1초(`xy_stable_duration`) 이상 지속되면 `xy_completed`가 `True`가 되며, 이후 로직은 `return` 되어 Nav2의 원본 명령(`final_cmd`)이 그대로 나갑니다.
2. **All Satisfied Stop:** XY와 Yaw가 모두 만족되면 `0.0`을 강제로 내보냅니다.
3. **Difference Clamping:**
* `calc_vx`(내 계산값) - `final_cmd.linear.x`(Nav2값) = `diff`
* 이 `diff`를 `±max_linear_diff` 범위로 자릅니다.
* `최종 출력 = Nav2값 + 잘린 diff`
* 이렇게 하면 Nav2가 0.1m/s를 주고 내가 0.3m/s를 주고 싶어도, 차이가 허용치(0.05)보다 크면 0.15m/s까지만 변경되어 **급발진을 방지**합니다.
##################
#######################
#########################
네, ROS 2에서 여러 토픽을 동시에 원활하게 처리하고, 특히 `control_loop`와 데이터 수신(Callback)이 서로를 막지 않게 하려면 **`MultiThreadedExecutor`**와 **`ReentrantCallbackGroup`**을 사용하는 것이 정석입니다.

제안하신 대로 멀티스레딩을 적용하여 **"모든 콜백이 병렬로 막힘없이 실행되도록"** 개선한 전체 코드를 드립니다.

### 🚀 개선된 점

1. **`ReentrantCallbackGroup` 적용:** 모든 Subscriber와 Timer가 이 그룹을 공유하게 하여, 하나의 콜백이 실행 중이어도 다른 콜백(예: TF 업데이트, Path 수신)이 기다리지 않고 별도 스레드에서 즉시 실행됩니다.
2. **`MultiThreadedExecutor` 사용:** `main` 함수에서 노드를 실행할 때 여러 스레드 풀을 사용하여 병렬 처리를 수행합니다.

---

### 🛠️ 최종 완성 코드 (Multi-Thread 적용)

```python
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Path, Odometry
import tf2_ros
import math
import numpy as np
import tf_transformations

def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

class StrictLateralDockingNode(Node):
    def __init__(self):
        super().__init__('strict_lateral_docking_node')

        # --- [New] Callback Group for Multi-threading ---
        # Reentrant: 이 그룹 내의 콜백들은 서로 병렬 실행 가능
        self.callback_group = ReentrantCallbackGroup()

        # --- Parameters ---
        self.path_length_threshold = 2.0
        self.cte_enable_threshold = 0.025
        self.cte_disable_threshold = 0.010
        
        # Tolerance
        self.final_xy_tolerance = 0.01      
        self.final_yaw_tolerance = 0.05     
        
        # Stability & Clamping
        self.xy_stable_duration = 1.0       
        self.max_linear_diff = 0.05         
        self.max_angular_diff = 0.5         
        self.min_creep_speed = 0.02

        # --- State ---
        self.is_correcting = False
        self.xy_stable_start_time = None
        self.xy_completed = False

        self.latest_cmd_vel = Twist()
        self.latest_cmd_time = self.get_clock().now()
        self.pruned_path = None
        
        # --- TF Buffer ---
        self.tf_buffer = tf2_ros.Buffer()
        # TF Listener는 내부적으로 노드의 Executor를 사용하므로 MultiThreadedExecutor의 혜택을 받음
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # --- Subscribers (Callback Group 적용) ---
        self.create_subscription(
            Path, 
            '/plan_pruned', 
            self.pruned_path_callback, 
            10,
            callback_group=self.callback_group # 병렬 처리
        )
        self.create_subscription(
            Twist, 
            '/cmd_vel_smoothed', 
            self.cmd_callback, 
            10,
            callback_group=self.callback_group # 병렬 처리
        )
        # Odom은 TF 조회를 위해 필요할 수 있으나, 여기서는 더미로만 사용
        self.create_subscription(
            Odometry, 
            '/odom', 
            lambda msg: None, 
            10,
            callback_group=self.callback_group
        )
        
        # --- Publisher ---
        self.cmd_pub = self.create_publisher(
            Twist, 
            '/cmd_vel_input_monitor', 
            10,
            callback_group=self.callback_group
        )
        
        # --- Timer (Control Loop) ---
        # 제어 루프도 별도 스레드에서 돌 수 있도록 설정
        self.create_timer(
            0.05, 
            self.control_loop, 
            callback_group=self.callback_group
        )

    def pruned_path_callback(self, msg): 
        self.pruned_path = msg

    def cmd_callback(self, msg): 
        self.latest_cmd_vel = msg
        self.latest_cmd_time = self.get_clock().now()

    def get_path_length(self):
        if not self.pruned_path or len(self.pruned_path.poses) < 2: return 0.0
        coords = np.array([(p.pose.position.x, p.pose.position.y) for p in self.pruned_path.poses])
        return np.sum(np.linalg.norm(coords[1:] - coords[:-1], axis=1))

    def get_dist_to_global_goal(self, robot_pose):
        if not self.pruned_path or len(self.pruned_path.poses) == 0: return float('inf')
        goal_pt = self.pruned_path.poses[-1].pose.position
        return math.hypot(goal_pt.x - robot_pose.position.x, goal_pt.y - robot_pose.position.y)

    def transform_global_to_local(self, global_pt, robot_pose):
        dx = global_pt[0] - robot_pose.position.x
        dy = global_pt[1] - robot_pose.position.y
        
        import tf_transformations
        _, _, robot_yaw = tf_transformations.euler_from_quaternion(
            [robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w])

        local_x = dx * math.cos(robot_yaw) + dy * math.sin(robot_yaw)
        local_y = -dx * math.sin(robot_yaw) + dy * math.cos(robot_yaw)
        return local_x, local_y, robot_yaw

    def get_lookahead_point(self, robot_pose, lookahead_dist=0.4):
        if not self.pruned_path or len(self.pruned_path.poses) < 2: return None
        path_arr = np.array([(p.pose.position.x, p.pose.position.y) for p in self.pruned_path.poses])
        robot_xy = np.array([robot_pose.position.x, robot_pose.position.y])
        dists = np.linalg.norm(path_arr - robot_xy, axis=1)
        min_idx = np.argmin(dists)
        curr_dist = 0.0
        target_pt = path_arr[min_idx]
        for i in range(min_idx, len(path_arr) - 1):
            p1 = path_arr[i]; p2 = path_arr[i+1]
            seg_len = np.linalg.norm(p2 - p1)
            if curr_dist + seg_len >= lookahead_dist:
                ratio = (lookahead_dist - curr_dist) / seg_len
                return p1 + (p2 - p1) * ratio
            curr_dist += seg_len
            target_pt = p2
        return target_pt

    def control_loop(self):
        # Safety Check
        if (self.get_clock().now() - self.latest_cmd_time).nanoseconds > 0.5 * 1e9:
            self.cmd_pub.publish(Twist()); return

        final_cmd = Twist()
        final_cmd.linear = self.latest_cmd_vel.linear
        final_cmd.angular = self.latest_cmd_vel.angular

        # Robot Pose Lookup
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_pose = Pose()
            robot_pose.position.x = trans.transform.translation.x
            robot_pose.position.y = trans.transform.translation.y
            robot_pose.position.z = trans.transform.translation.z
            robot_pose.orientation = trans.transform.rotation
        except Exception:
            self.cmd_pub.publish(final_cmd); return

        if self.pruned_path is None: return

        path_len = self.get_path_length()
        dist_to_goal = self.get_dist_to_global_goal(robot_pose)

        # =========================================================
        # [Stage 2] Final Docking (0.05m 이내)
        # =========================================================
        if dist_to_goal < 0.05:
            
            # Goal Pose & Yaw Error
            goal_pose_global = self.pruned_path.poses[-1].pose
            _, _, goal_yaw = tf_transformations.euler_from_quaternion(
                [goal_pose_global.orientation.x, goal_pose_global.orientation.y, goal_pose_global.orientation.z, goal_pose_global.orientation.w])
            
            _, _, current_yaw = tf_transformations.euler_from_quaternion(
                [robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w])
            
            yaw_error = normalize_angle(goal_yaw - current_yaw)
            
            xy_satisfied = dist_to_goal < self.final_xy_tolerance
            yaw_satisfied = abs(yaw_error) < self.final_yaw_tolerance

            # 1. Complete Stop (All satisfied)
            if xy_satisfied and yaw_satisfied:
                final_cmd.linear.x = 0.0
                final_cmd.angular.z = 0.0
                self.cmd_pub.publish(final_cmd)
                self.is_correcting = True
                return

            # 2. Bypass Check (XY Stable)
            if xy_satisfied:
                if self.xy_stable_start_time is None:
                    self.xy_stable_start_time = self.get_clock().now()
                
                elapsed = (self.get_clock().now() - self.xy_stable_start_time).nanoseconds / 1e9
                if elapsed > self.xy_stable_duration:
                    self.xy_completed = True
            else:
                self.xy_stable_start_time = None
                self.xy_completed = False

            if self.xy_completed:
                self.is_correcting = False
                self.cmd_pub.publish(final_cmd) # Bypass
                return

            # 3. Correction & Clamping
            goal_pt_global = [goal_pose_global.position.x, goal_pose_global.position.y]
            local_x, local_y, _ = self.transform_global_to_local(goal_pt_global, robot_pose)
            
            # Calc Linear
            kp_dist = 1.5 
            calc_vx = kp_dist * local_x 
            speed_limit = 0.1 
            if abs(calc_vx) > speed_limit:
                calc_vx = math.copysign(speed_limit, calc_vx)
            if abs(calc_vx) < self.min_creep_speed:
                 calc_vx = math.copysign(self.min_creep_speed, calc_vx)

            # Calc Angular
            target_yaw_local = math.atan2(local_y, local_x)
            if local_x < 0:
                steering_error = normalize_angle(target_yaw_local - math.pi)
            else:
                steering_error = target_yaw_local

            calc_w = 2.5 * steering_error
            calc_w = max(min(calc_w, 0.8), -0.8)

            # [Clamping Logic]
            lin_diff = calc_vx - final_cmd.linear.x
            lin_diff = max(min(lin_diff, self.max_linear_diff), -self.max_linear_diff)
            final_cmd.linear.x = final_cmd.linear.x + lin_diff

            ang_diff = calc_w - final_cmd.angular.z
            ang_diff = max(min(ang_diff, self.max_angular_diff), -self.max_angular_diff)
            final_cmd.angular.z = final_cmd.angular.z + ang_diff

            self.is_correcting = True

        # =========================================================
        # [Stage 1] Approach (Look-ahead)
        # =========================================================
        elif path_len < self.path_length_threshold:
            
            lookahead_pt = self.get_lookahead_point(robot_pose, lookahead_dist=0.4)
            
            if lookahead_pt is not None:
                local_x, local_y, _ = self.transform_global_to_local(lookahead_pt, robot_pose)
                current_cte = abs(local_y)
                
                if not self.is_correcting:
                    if current_cte > self.cte_enable_threshold: self.is_correcting = True
                else:
                    if current_cte < self.cte_disable_threshold: self.is_correcting = False

                if self.is_correcting:
                    target_v = final_cmd.linear.x
                    
                    if abs(target_v) < self.min_creep_speed:
                         if abs(target_v) < 0.001:
                             target_v = self.min_creep_speed 
                         else:
                             target_v = math.copysign(self.min_creep_speed, target_v)

                    dist_sq = local_x**2 + local_y**2
                    curvature = 2.0 * local_y / dist_sq if dist_sq > 0 else 0
                    
                    final_cmd.linear.x = target_v
                    final_cmd.angular.z = target_v * curvature
                    final_cmd.angular.z = max(min(final_cmd.angular.z, 0.8), -0.8)

        else:
            self.is_correcting = False

        self.cmd_pub.publish(final_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = StrictLateralDockingNode()
    
    # [New] MultiThreadedExecutor 사용
    # 여러 스레드를 사용하여 CallbackGroup에 등록된 작업들을 병렬로 처리함
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

```
######################  
########################  
#########################  
사용자님의 아이디어는 로봇 제어에서 **"Timeout-based Relaxation (시간 초과 시 조건 완화)"** 전략이라고 불리며, 무한 루프나 진동(Oscillation)에 빠지는 것을 막는 아주 실용적인 안전장치입니다.

"3.5초 동안 노력했는데도 정밀한 목표(Strict)를 못 맞춘다면, 오실레이션이 발생하고 있다고 판단하고 허용 범위를 넓혀서(Relaxed) 일단 멈추게 하자"는 논리입니다.

코드 수정이 필요한 부분은 **`__init__` (파라미터 추가)**과 **`control_loop` 내의 Stage 2 진입 부분**입니다.

---

### 🛠️ 수정 1: `__init__` 함수 (변수 추가)

오실레이션 판단 기준 시간과, 완화된(Relaxed) 허용 오차 값을 정의합니다.

```python
    def __init__(self):
        super().__init__('strict_lateral_docking_node')

        # ... (기존 파라미터들 유지) ...
        self.path_length_threshold = 2.0
        self.cte_enable_threshold = 0.025
        self.cte_disable_threshold = 0.010
        
        # [기존] 엄격한(Strict) 정지 조건
        self.final_xy_tolerance = 0.01      # 1cm
        self.final_yaw_tolerance = 0.05     # 약 2.8도
        
        # [신규] 오실레이션 방지용 파라미터
        self.oscillation_timeout = 3.5      # 3.5초 이상 지체되면 조건 완화
        self.relaxed_xy_tolerance = 0.03    # 완화된 XY 오차 (3cm)
        self.relaxed_yaw_tolerance = 0.1    # 완화된 Yaw 오차 (약 5.7도)
        
        # [신규] Final Stage 진입 시간 기록용
        self.final_approach_start_time = None 

        # ... (나머지 초기화 유지) ...

```

---

### 🛠️ 수정 2: `control_loop` 함수 (로직 변경)

`control_loop` 함수 내부의 **`if dist_to_goal < 0.05:` (Stage 2)** 블록을 아래 코드로 교체해 주세요.

```python
        # =========================================================
        # [Stage 2] Final Docking (0.05m 이내)
        # =========================================================
        if dist_to_goal < 0.05:
            
            # [1] 시간 측정 로직 (Oscillation Timer)
            # Final Stage에 처음 진입했다면 시간 기록 시작
            if self.final_approach_start_time is None:
                self.final_approach_start_time = self.get_clock().now()
            
            # 경과 시간 계산
            elapsed_time = (self.get_clock().now() - self.final_approach_start_time).nanoseconds / 1e9
            
            # [2] Tolerance 결정 로직 (Dynamic Tolerance)
            # 3.5초가 지났다면? -> 완화된 기준(Relaxed) 적용
            if elapsed_time > self.oscillation_timeout:
                current_xy_tol = self.relaxed_xy_tolerance
                current_yaw_tol = self.relaxed_yaw_tolerance
                # (옵션) 로그 출력: 오실레이션 감지됨
                # self.get_logger().warn("Oscillation detected! Relaxing tolerance...", throttle_duration_sec=1.0)
            else:
                # 3.5초 이내라면? -> 엄격한 기준(Strict) 적용
                current_xy_tol = self.final_xy_tolerance
                current_yaw_tol = self.final_yaw_tolerance

            # [3] 목표 오차 계산 (기존 로직 유지)
            goal_pose_global = self.pruned_path.poses[-1].pose
            _, _, goal_yaw = tf_transformations.euler_from_quaternion(
                [goal_pose_global.orientation.x, goal_pose_global.orientation.y, goal_pose_global.orientation.z, goal_pose_global.orientation.w])
            
            _, _, current_yaw = tf_transformations.euler_from_quaternion(
                [robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w])
            
            yaw_error = normalize_angle(goal_yaw - current_yaw)

            # [4] 정지 조건 확인 (동적으로 결정된 Tolerance 사용)
            xy_satisfied = dist_to_goal < current_xy_tol
            yaw_satisfied = abs(yaw_error) < current_yaw_tol

            # [5] 조건 만족 시 정지
            if xy_satisfied and yaw_satisfied:
                final_cmd.linear.x = 0.0
                final_cmd.angular.z = 0.0
                self.cmd_pub.publish(final_cmd)
                self.is_correcting = True 
                
                # 성공했으므로 타이머 리셋 (다음에 다시 쓸 수 있게)
                self.final_approach_start_time = None 
                return

            # --- (Bypass 로직 및 보정 제어 로직은 기존과 동일) ---
            # ... (아래 내용은 기존 코드의 XY Stable Check 및 P-Control 부분 유지) ...
            
            # (중략: XY Stable Check 로직)

            # (중략: P-Control 계산 및 Clamping 로직)
            
            # ...
            
            self.is_correcting = True

        # [중요] Final Stage를 벗어나면 타이머 리셋
        # (예: 로봇이 밀려서 0.05m 밖으로 나갔다가 다시 들어오는 경우 대비)
        elif path_len < self.path_length_threshold:
            self.final_approach_start_time = None # 타이머 리셋
            
            # ... (Stage 1 로직 유지) ...
        else:
            self.final_approach_start_time = None # 타이머 리셋
            self.is_correcting = False

```

### 🧠 로직 동작 설명

1. **타이머 시작:** 로봇이 Goal 5cm 이내로 진입하는 순간 `self.final_approach_start_time`에 현재 시간을 찍습니다.
2. **엄격 모드 (0.0s ~ 3.5s):** 처음 3.5초 동안은 `current_xy_tol`이 **0.01m(1cm)**입니다. 로봇은 이 정밀한 기준을 맞추기 위해 열심히 미세 조정을 시도합니다.
3. **완화 모드 (3.5s 이후):** 만약 3.5초가 지났는데도 1cm를 못 맞춰서 계속 움직이고 있다면(Oscillation), `current_xy_tol`이 **0.03m(3cm)**로 변경됩니다.
4. **탈출:** 이때 로봇의 오차가 2cm 정도였다면, 엄격 모드에서는 실패였지만 완화 모드에서는 **"성공"**으로 판정됩니다. 즉시 `0.0` 명령을 내리고 정지하여 진동을 멈춥니다.
5. **리셋:** 만약 로봇이 다시 멀어지거나(Stage 1으로 복귀) 완전히 끝나면 타이머를 `None`으로 초기화합니다.
####################
#########################
##########################
사용자님의 의도는 **"단순히 목표 근처에 오래 머무는 것"**이 아니라, **"제어값(속도)의 부호가 바뀌는(Forward↔Backward, Left↔Right) 찐(Real) 오실레이션 현상이 지속될 때"**만 조건을 완화하고 싶다는 것이군요.

이를 구현하려면 **'부호 반전(Zero Crossing)'**을 감지하는 로직과, 그 반전이 **'지속되는 시간'**을 측정하는 로직이 필요합니다.

다음과 같은 알고리즘을 적용했습니다.

1. **플립(Flip) 감지:** 현재 계산된 속도의 부호가 이전 부호와 다르면(`+` ↔ `-`) "플립"으로 간주합니다.
2. **연속성 판단:** 마지막 플립 이후 **1.0초 이내**에 또 플립이 발생하면 "오실레이션 상태"가 유지되는 것으로 봅니다. (1초 이상 부호가 안 바뀌면 안정이 된 것으로 보고 타이머 리셋)
3. **지속 시간 체크:** 이 "오실레이션 상태"가 **3.5초 이상** 이어지면, 그때 Tolerance를 완화합니다.

수정이 필요한 코드 부분입니다.

---

### 🛠️ 1. `__init__` (변수 추가)

이전 값(부호)을 저장할 변수와 타이머 변수들을 추가합니다.

```python
    def __init__(self):
        super().__init__('strict_lateral_docking_node')

        # ... (기존 파라미터들 유지) ...
        
        # [Tolerance 설정]
        self.final_xy_tolerance = 0.01      # Strict
        self.final_yaw_tolerance = 0.05
        
        self.relaxed_xy_tolerance = 0.03    # Relaxed
        self.relaxed_yaw_tolerance = 0.1
        
        # [신규] 오실레이션 감지 설정
        self.oscillation_duration_threshold = 3.5  # 3.5초 이상 지속 시 완화
        self.flip_continuity_window = 1.0          # 1초 안에 다시 뒤집히면 연속된 오실레이션으로 간주
        
        # [신규] 상태 변수
        self.prev_lin_sign = 0.0    # 이전 선속도 부호
        self.prev_ang_sign = 0.0    # 이전 각속도 부호
        
        self.last_flip_time = None          # 가장 최근 부호가 바뀐 시각
        self.oscillation_start_time = None  # 오실레이션(불안정)이 시작된 시각

        # ... (나머지 초기화 유지) ...

```

---

### 🛠️ 2. `control_loop` 내부 (오실레이션 감지 로직 구현)

`control_loop` 내부의 **Stage 2 (Final Docking)** 계산 로직 사이에 아래 코드를 넣어주세요.
**순서가 중요합니다.** `calc_vx`, `calc_w`를 계산한 **직후**, 그리고 `final_cmd`에 할당하기 **직전**에 판단해야 합니다.

```python
        # =========================================================
        # [Stage 2] Final Docking (0.05m 이내)
        # =========================================================
        if dist_to_goal < 0.05:
            
            # 1. 목표 계산 (먼저 수행해야 부호 비교 가능)
            goal_pose_global = self.pruned_path.poses[-1].pose
            goal_pt_global = [goal_pose_global.position.x, goal_pose_global.position.y]
            local_x, local_y, _ = self.transform_global_to_local(goal_pt_global, robot_pose)
            
            # --- [Raw Control Value Calculation] ---
            # (속도 제한 전, 순수 P제어 계산값의 부호를 보는 것이 더 정확함)
            kp_dist = 1.5 
            raw_vx = kp_dist * local_x 
            
            target_yaw_local = math.atan2(local_y, local_x)
            if local_x < 0: steering_error = normalize_angle(target_yaw_local - math.pi)
            else: steering_error = target_yaw_local
            raw_w = 2.5 * steering_error

            # ----------------------------------------------------------------
            # [신규] 찐(Real) 오실레이션 감지 로직 (Sign Flip Detection)
            # ----------------------------------------------------------------
            current_time = self.get_clock().now()
            
            # 현재 부호 추출 (-1, 0, 1)
            # 0.001 같은 노이즈 제외를 위해 약간의 데드존을 줄 수도 있음
            curr_lin_sign = 1.0 if raw_vx > 1e-4 else (-1.0 if raw_vx < -1e-4 else 0.0)
            curr_ang_sign = 1.0 if raw_w > 1e-4 else (-1.0 if raw_w < -1e-4 else 0.0)
            
            # 플립(부호 반전) 발생 여부 확인
            # (이전 부호와 곱했을 때 음수면 반전된 것임. 0에서 변하는 건 오실레이션 아님)
            is_lin_flipped = (curr_lin_sign * self.prev_lin_sign < 0)
            is_ang_flipped = (curr_ang_sign * self.prev_ang_sign < 0)
            
            is_flipped = is_lin_flipped or is_ang_flipped

            if is_flipped:
                # 부호가 뒤집힘!
                self.last_flip_time = current_time
                
                # 오실레이션 타이머가 안 돌고 있었다면 시작
                if self.oscillation_start_time is None:
                    self.oscillation_start_time = current_time
            else:
                # 부호가 안 뒤집힘 (안정적 or 한 방향 주행 중)
                # 만약 마지막 플립으로부터 1초 이상 지났다면 -> "오실레이션 끝났다"고 판정
                if self.last_flip_time is not None:
                    elapsed_since_flip = (current_time - self.last_flip_time).nanoseconds / 1e9
                    if elapsed_since_flip > self.flip_continuity_window:
                        self.oscillation_start_time = None # 리셋
                        self.last_flip_time = None

            # 부호 상태 업데이트
            if abs(curr_lin_sign) > 0: self.prev_lin_sign = curr_lin_sign
            if abs(curr_ang_sign) > 0: self.prev_ang_sign = curr_ang_sign

            # ----------------------------------------------------------------
            # [Tolerance 결정] 오실레이션 지속 시간 체크
            # ----------------------------------------------------------------
            use_relaxed = False
            if self.oscillation_start_time is not None:
                oscillation_duration = (current_time - self.oscillation_start_time).nanoseconds / 1e9
                if oscillation_duration > self.oscillation_duration_threshold:
                    use_relaxed = True
                    # (옵션) 로그: "3.5초간 좌우로 흔들려서 기준 완화함"
            
            current_xy_tol = self.relaxed_xy_tolerance if use_relaxed else self.final_xy_tolerance
            current_yaw_tol = self.relaxed_yaw_tolerance if use_relaxed else self.final_yaw_tolerance

            # ----------------------------------------------------------------
            # [정지 조건 확인]
            # ----------------------------------------------------------------
            _, _, goal_yaw = tf_transformations.euler_from_quaternion(
                [goal_pose_global.orientation.x, goal_pose_global.orientation.y, goal_pose_global.orientation.z, goal_pose_global.orientation.w])
            _, _, current_yaw = tf_transformations.euler_from_quaternion(
                [robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w])
            yaw_error = normalize_angle(goal_yaw - current_yaw)

            if dist_to_goal < current_xy_tol and abs(yaw_error) < current_yaw_tol:
                final_cmd.linear.x = 0.0
                final_cmd.angular.z = 0.0
                self.cmd_pub.publish(final_cmd)
                self.is_correcting = True
                
                # 성공 시 상태 리셋
                self.oscillation_start_time = None
                self.last_flip_time = None
                return

            # ----------------------------------------------------------------
            # [실제 속도 명령 생성] (이전 로직과 동일 - 변수명만 raw_vx 사용)
            # ----------------------------------------------------------------
            # Calc Linear (Clamp & Creep)
            speed_limit = 0.1 
            calc_vx = raw_vx # 위에서 계산한 P제어 값 사용
            
            if abs(calc_vx) > speed_limit:
                calc_vx = math.copysign(speed_limit, calc_vx)
            if abs(calc_vx) < self.min_creep_speed:
                 calc_vx = math.copysign(self.min_creep_speed, calc_vx)

            # Calc Angular (Clamp)
            calc_w = raw_w # 위에서 계산한 값
            calc_w = max(min(calc_w, 0.8), -0.8)

            # Difference Clamping & Publish
            lin_diff = max(min(calc_vx - final_cmd.linear.x, self.max_linear_diff), -self.max_linear_diff)
            final_cmd.linear.x = final_cmd.linear.x + lin_diff

            ang_diff = max(min(calc_w - final_cmd.angular.z, self.max_angular_diff), -self.max_angular_diff)
            final_cmd.angular.z = final_cmd.angular.z + ang_diff

            self.is_correcting = True
        
        else:
            # Stage 2 벗어나면 오실레이션 상태 리셋
            self.oscillation_start_time = None
            self.last_flip_time = None
            self.prev_lin_sign = 0.0
            self.prev_ang_sign = 0.0
            
            # ... (Stage 1 로직 유지) ...

```

### 🧠 로직 상세 설명

1. **부호 비교 (`*` 연산):** `현재_부호 * 이전_부호 < 0` 이면 서로 부호가 다르다는 뜻이므로 **플립(Flip)**이 감지된 것입니다. (예: `1.0 * -1.0 = -1.0`)
2. **`flip_continuity_window` (1.0초):**
* 로봇이 `앞 -> 뒤`로 한 번 바꿨다고 바로 오실레이션은 아닙니다. (지나쳐서 돌아오는 걸 수도 있음)
* 하지만 `앞 -> 뒤`로 바꾸고 **1초 안에** 다시 `뒤 -> 앞`으로 바꾸면 이건 확실한 오실레이션(떨림)입니다.
* 이 윈도우 시간 내에 플립이 계속 발생하면 `oscillation_start_time`을 유지(누적)합니다.


3. **`oscillation_duration_threshold` (3.5초):**
* 위 상태가 3.5초 동안 이어지면, "아, 얘는 지금 1cm를 맞추려다가 계속 제자리에서 떨고 있구나"라고 판단합니다.
* 이때 `use_relaxed = True`가 되어 허용 오차를 1cm → 3cm로 넓혀줍니다.


4. **자동 멈춤:** 오차가 3cm로 넓어지는 순간, 현재 오차가 2cm였다면 즉시 조건 만족(`True`)이 되어 `0.0` 명령을 보내고 떨림을 멈춥니다.
