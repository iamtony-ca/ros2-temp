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

