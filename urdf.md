
```xml
<?xml version="1.0" ?>
<robot name="my_robot" xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:include filename="$(find nav2_minimal_tb4_description)/urdf/icreate/create3.urdf.xacro" />
  <xacro:include filename="$(find nav2_minimal_tb4_description)/urdf/sensors/rplidar.urdf.xacro" />
  <xacro:include filename="$(find nav2_minimal_tb4_description)/urdf/sensors/oakd.urdf.xacro" />
  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_link"/>
    <child link="base_footprint"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  <link name="base_footprint"/>

  <link name="chassis_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="light_grey">
        <color rgba="0.8 0.8 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>
    <!-- <inertial>
      <mass value="15.0"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.5" iyz="0.0" izz="0.5" />
    </inertial> -->
    <inertial>
      <mass value="15.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.1625" ixy="0.0" ixz="0.0"
               iyy="0.3625" iyz="0.0"
               izz="0.425" />
    </inertial>

  </link>

  <joint name="chassis_joint" type="fixed">
    <parent link="base_link"/>
    <child link="chassis_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

  <xacro:rplidar name="rplidar" parent_link="chassis_link" gazebo="$(arg gazebo)">
    <origin xyz="0.0 0.0 0.15" rpy="0 0 0"/>
  </xacro:rplidar>

  <xacro:oakd model="pro" parent_link="chassis_link">
    <origin xyz="0.25 0.0 0.05" rpy="0 0 0"/>
  </xacro:oakd>
  </robot>

```




```xml
<?xml version="1.0" ?>
<robot name="turtlebot4" xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:include filename="$(find nav2_minimal_tb4_description)/urdf/icreate/create3.urdf.xacro" />
  <xacro:include filename="$(find nav2_minimal_tb4_description)/urdf/sensors/rplidar.urdf.xacro" />
  <xacro:include filename="$(find nav2_minimal_tb4_description)/urdf/sensors/oakd.urdf.xacro" />
  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_link"/>
    <child link="base_footprint"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  <link name="base_footprint"/>

  <link name="chassis_link">
    <visual>
      <box size="0.5 0.3 0.2"/>
      <material name="light_grey"><color rgba="0.8 0.8 0.8 1.0"/></material>
    </visual>
    <collision>
      <geometry><box size="0.5 0.3 0.2"/></geometry>
    </collision>
    <!-- <inertial>
      <mass value="15.0"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.5" iyz="0.0" izz="0.5" />
    </inertial> -->
    <inertial>
      <mass value="15.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.1625" ixy="0.0" ixz="0.0"
               iyy="0.3625" iyz="0.0"
               izz="0.425" />
    </inertial>


  </link>

  <joint name="chassis_joint" type="fixed">
    <parent link="base_link"/>
    <child link="chassis_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>
  <xacro:rplidar name="front_rplidar" parent_link="chassis_link" gazebo="$(arg gazebo)">
    <origin xyz="0.20 0.10 0.12" rpy="0 0 ${0.25 * 3.14159}" />
    <xacro:property name="topic" value="front_scan" />
  </xacro:rplidar>

  <xacro:rplidar name="back_rplidar" parent_link="chassis_link" gazebo="$(arg gazebo)">
    <origin xyz="-0.20 -0.10 0.12" rpy="0 0 ${-0.75 * 3.14159}" />
    <xacro:property name="topic" value="back_scan" />
  </xacro:rplidar>

  <xacro:oakd model="pro" parent_link="chassis_link">
    <origin xyz="0.25 0.0 0.05" rpy="0 0 0"/>
  </xacro:oakd>
  </robot>

```
  
      footprint: "[[0.25, 0.15], [-0.25, 0.15], [-0.25, -0.15], [0.25, -0.15]]"





이 캡처면, **변환이 실패해서 RViz가 /scan을 자기 프레임(rplidar_link) 그대로 그려버린** 상황이랑 99% 같아요. 그러면 Fixed Frame이 map이어도 빨간 점들이 로봇 회전에 “붙어서” 같이 도는 것처럼 보입니다.

핵심 단서:

* `/scan`의 스탬프가 **181초**.
* `view_frames.pdf`엔 `map→odom`, `odom→base_link` 변환들의 **가장 최근/가장 오래된 시각이 210~214초대**로 나옵니다.
  → 즉, **TF 버퍼가 210~214초 구간**만 갖고 있는데, `/scan`은 **181초**로 들어와서, RViz가 `map ← rplidar_link` 변환을 **그 시각에** 못 구합니다. 못 구하면 RViz는 경고를 띄우거나(작게 보일 수 있음) **그냥 센서 프레임 좌표로 그림** → 로봇 돌면 같이 돎.

이건 URDF yaw 문제 이전에 **시간축(sim time) 불일치**가 먼저입니다.

---

# 바로 해결 체크리스트 (위에서부터 순서대로 실행)

1. 모두 `use_sim_time=true`

```bash
ros2 param get /rviz use_sim_time
ros2 param get /robot_state_publisher use_sim_time
ros2 param get /rplidar_node use_sim_time           # 실제 라이다 드라이버/플러그인 노드명
ros2 param get /<odometry_node> use_sim_time        # diff_drive, gazebo plugin 등
```

하나라도 `false`면:

```bash
ros2 param set /rviz use_sim_time true
ros2 param set /robot_state_publisher use_sim_time true
ros2 param set /rplidar_node use_sim_time true
ros2 param set /<odometry_node> use_sim_time true
```

2. **/clock 시각과 /scan 시각이 같은 축인지 확인**

```bash
ros2 topic echo /clock --once
ros2 topic echo /scan --once | head -n 20
```

* `/clock.clock.sec` 와 `/scan.header.stamp.sec` 가 **근접**해야 합니다(보통 오차 < 0.5s).
* 지금은 **181초 vs TF 210초대**로 30초 이상 어긋나 있음 → 변환 실패.

3. RViz 상태창/Displays에서 LaserScan에 **TF 경고** 뜨는지 확인

* “No transform from rplidar_link to map at time …” 같은 경고가 잠깐이라도 보이면 지금 케이스가 맞습니다.

4. 시뮬레이터/플러그인 쪽 설정

* Gazebo(Harmonic/Classic) 사용 시, 센서 플러그인이 **스탬프를 sim time(/clock)으로** 찍고 있는지 확인.

  * Gazebo ROS 플러그인 계열은 보통 자동으로 sim time을 쓰지만, 일부 드라이버 래퍼(예: rplidar 실제 드라이버)를 **시뮬 환경에서 그대로 실행**하면 시스템 시간으로 스탬프를 찍을 수 있습니다. 이 경우 반드시 `use_sim_time=true`.
* 노드를 재시작할 땐 `/clock`이 흐르는 상태에서 띄우세요(초기 0초/미래 시각 꼬임 방지).

5. 그래도 어긋나면(드물지만 있음)

* TF 버퍼가 너무 짧아서 과거 시각을 못 잡는 경우: TF 퍼블리셔의 버퍼 길이(예: `tf_buffer_duration`)를 늘리거나, 센서/TF의 타임스탬프 차이를 줄이세요. 보통은 10초면 충분합니다.

---

# URDF 정합(부모/각도)도 같이 잡기

시간 문제를 먼저 해결한 뒤에도 각도가 이상하면 아래도 적용하세요.

* TB4 기본값에 맞추기(권장):

```xml
<xacro:rplidar name="rplidar" parent_link="shell_link" gazebo="$(arg gazebo)">
  <origin xyz="0.0 0.0 0.15" rpy="0 0 ${pi/2}"/>
</xacro:rplidar>
```

* `chassis_link` 유지 시에도 **yaw=pi/2**는 맞추세요.

그리고 Nav2 costmap:

```yaml
scan:
  topic: /scan
  sensor_frame: rplidar_link   # /scan header.frame_id와 동일
  data_type: LaserScan
  clearing: true
  marking: true
  inf_is_valid: true
```

---

# 빠른 판별법 (한 번에 끝내기)

* 위 1)~3) 실행 후, 다시 캡처처럼 로봇을 빙글 돌려 보세요.

  * **정상**이면: 벽/지도 위에서 스캔이 고정되어 보이고, RViz LaserScan의 “Decay Time” 때문에 ‘자취’가 남아도 **지도에 정렬**됩니다.
  * **여전히 같이 돈다**면: `/scan`과 TF의 시각이 아직 안 맞습니다(또는 RViz Fixed Frame이 실수로 base_link).

필요하면 `/clock` 1줄, `/scan` 헤더 1줄, RViz의 LaserScan Status 메시지 한 줄만 더 보내줘—그걸로 정확히 어느 노드가 다른 시간축을 쓰는지 집어낼게요.


####################
맞아요—URDF만 바꿨다고 `use_sim_time` 플래그 값이 “자동으로 바뀌는” 일은 거의 없어요.
하지만 **URDF 변경이 “시간 불일치”를 유발**할 수 있는 경로는 몇 가지가 있습니다. 핵심은 “모두 `true`인가?”가 아니라 **“같은 시간축으로 찍히고 있나?”**예요. 앞서 로그에서 `/scan` 시각이 **181초**, TF는 **210~214초**였죠. 이건 전형적인 **시각 불일치**입니다.

## URDF만 바꿨는데 시간이 어긋날 수 있는 이유 (현실에서 자주 생김)

1. **센서 플러그인/드라이버가 바뀜**

   * TB4 기본 URDF의 `xacro:rplidar`는 Gazebo용 Ray Sensor 플러그인을 붙여 **sim time(/clock)** 으로 스탬프를 찍습니다.
   * 커스텀 URDF에서 `parent_link`, `gazebo` 인자, 매크로 인자 누락 등으로 **플러그인 분기가 달라지면**, 같은 `/scan` 토픽명이라도 **다른 노드**(혹은 시스템 시계로 찍는 노드)가 퍼블리시할 수 있어요. 결과: `/scan`은 **시스템 시간**, TF는 **sim time** → 변환 실패.

2. **노드/네임스페이스가 달라져 파라미터 주입이 빗나감**

   * 예: 이전에 `use_sim_time`를 `/**` 또는 `/rplidar_node`로 주입했는데, URDF/런치 변경으로 노드명이 `rplidar`→`rplidar_node`(또는 네임스페이스 변경)로 바뀌면 **그 노드에는 `use_sim_time`가 안 먹습니다.**

3. **재시작 타이밍으로 생기는 시각 격차**

   * URDF 바꾸면서 `robot_state_publisher`(TF)는 재시작되었는데 Gazebo 센서 플러그인은 **이전부터 돌던 시계 기준**으로 계속 퍼블리시 → `/scan` 첫 메시지 시각이 과거(181s)로 남고, TF는 최신(210s) → RViz가 변환 못 해서 **센서 프레임 그대로 그림**(붙어서 도는 증상).

4. **`gazebo` 인자 누락/기본값**

   * TB4 URDF는 `xacro:arg name="gazebo"`로 플러그인 포함을 토글합니다. 커스텀 URDF에서 이 인자가 없거나 기본값이 달라지면 플러그인 경로가 바뀌어 **스탬프 방식이 달라질** 수 있어요.

---

## 빠른 확증 테스트 (3줄로 끝)

아래 세 줄로 “누가 다른 시간축인지” 바로 잡힙니다.

```bash
# 1) /clock 현재 시각
ros2 topic echo /clock --once

# 2) /scan 헤더 시각
ros2 topic echo /scan --once | head -n 5

# 3) /tf 최신 시각(odom->base_link 같은 자주 나오는 변환)
ros2 topic echo /tf --once | head -n 20
```

* 세 값이 **서로 수 초 이내**로 비슷해야 정상입니다.
* 하나라도 **수십 초** 어긋나면 그 퍼블리셔(대개 `/scan`)가 범인.

---

## 바로 고치는 액션 플랜

1. **모든 관련 노드가 sim time을 쓰는지 강제 확인**

   ```bash
   ros2 param get /gazebo use_sim_time            # Gazebo(또는 gz-ros-bridge가 있다면 그쪽)
   ros2 param get /robot_state_publisher use_sim_time
   ros2 param get /rviz use_sim_time
   ros2 param get /rplidar_node use_sim_time      # 실제 센서 드라이버/플러그인 노드명에 맞추기
   ros2 param get /diff_drive_controller use_sim_time  # odom 퍼블리셔 쪽
   ```

   하나라도 `false`면 `ros2 param set ... true`.

2. **/scan 퍼블리셔 정체 확인**

   ```bash
   ros2 node info $(ros2 topic info /scan -v | grep 'Publisher count' -n; ros2 topic info /scan -v)
   # 또는: ros2 topic info /scan -v  만으로 퍼블리셔 노드명 확인
   ```

   * TB4 때와 **퍼블리셔 노드명이 다른지** 확인.
   * 퍼블리셔 노드에 `use_sim_time`가 들어갔는지 바로 `ros2 param get`으로 확인/설정.

3. **URDF의 rplidar 매크로 인자/부모/각도 정리**

   * 매크로 첫머리에 없으면 추가:

     ```xml
     <xacro:arg name="gazebo" default="true"/>
     ```
   * TB4와 동일하게(권장):

     ```xml
     <xacro:rplidar name="rplidar" parent_link="shell_link" gazebo="$(arg gazebo)">
       <origin xyz="0.0 0.0 0.15" rpy="0 0 ${pi/2}"/>
     </xacro:rplidar>
     ```

     (부모를 `chassis_link`로 유지하고 싶어도 **yaw=pi/2**는 맞추세요.)

4. **스택 통째로 재시작**

   * Gazebo(또는 시뮬), `robot_state_publisher`, 센서 퍼블리셔, RViz를 **모두 끄고 다시** 띄워 _동일한 시각 축_으로 시작하게 하세요.
   * 재시작 후 위 3줄 테스트를 다시 실행.

---

## 요약

* `use_sim_time=true` “값 자체”는 그대로일 수 있어도, **URDF 변경이 퍼블리셔/플러그인/네임스페이스/재시작 타이밍**을 바꾸면서 **시각 불일치**를 만들 수 있습니다.
* 지금 캡처와 숫자는 **그 불일치(181s vs 210+s)**를 정확히 가리키고 있어요.
* 위 액션 1–4만 수행하면, Fixed Frame이 `map`인 RViz에서 **빨간 점들이 지도에 고정**되어 보일 겁니다.
* 그 다음 단계로 각도(yaw=pi/2)까지 TB4와 맞추면 방향 정렬까지 깔끔하게 끝납니다.
############################



네, `depot.sdf` 파일에서 3D 맵 모델을 교체하는 것은 간단합니다. 핵심은 Gazebo Fuel에서 온라인으로 Depot 맵을 불러오는 `<include>` 블록을, **사용자의 로컬 STL 파일을 사용하는 새로운 `<include>` 블록으로 교체**하는 것입니다.

다만, STL 파일을 바로 월드에 넣기보다는 Gazebo의 표준 방식인 '모델(Model)'로 먼저 만들어주는 것이 좋습니다. 이렇게 하면 맵을 재사용하고 관리하기가 훨씬 편해집니다.

아래 3단계로 나누어 자세히 설명해 드리겠습니다.

-----

### \#\# 1단계: STL 파일을 Gazebo 모델로 만들기

먼저 사용자의 STL 파일을 Gazebo가 인식할 수 있는 '모델' 패키지 형태로 정리해야 합니다. 아래와 같은 폴더 구조를 만드는 것을 추천합니다.

  * `my_robot_gazebo` (사용자의 시뮬레이션 패키지)
      * `models/`
          * `my_map/` (새로 만들 맵 모델 폴더)
              * `meshes/`
                  * `my_map.stl` **(사용자의 STL 파일을 이곳에 복사)**
              * `model.config` **(아래 내용으로 새로 만들기)**
              * `model.sdf` **(아래 내용으로 새로 만들기)**
      * `worlds/`
          * `my_world.sdf` (곧 만들 커스텀 월드 파일)
      * `...`

#### \#\#\# `model.config` 파일 내용

맵 모델의 이름, 버전, 설명 등 메타데이터를 담는 파일입니다.

```xml
<?xml version="1.0"?>
<model>
  <name>My Custom Map</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Your Name</name>
    <email>your@email.com</email>
  </author>
  <description>
    A custom map model for my robot simulation.
  </description>
</model>
```

#### \#\#\# `model.sdf` 파일 내용

이 파일이 STL 파일을 실제 시뮬레이션 객체로 정의하는 역할을 합니다.

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="my_map_model">
    <static>true</static>
    <link name="map_link">
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>meshes/my_map.stl</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>meshes/my_map.stl</uri>
          </mesh>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

-----

### \#\# 2단계: 월드(`depot.sdf`) 파일 수정

이제 월드 파일을 수정하여 기존 Depot 맵 대신 방금 만든 `my_map` 모델을 불러오도록 하겠습니다.

1.  `depot.sdf` 파일을 `my_robot_gazebo/worlds/` 폴더에 `my_world.sdf` 라는 이름으로 복사합니다.

2.  `my_world.sdf` 파일을 열고, 아래 **`<include>` 블록 전체를 찾아서 삭제**합니다.

    **삭제할 부분:**

    ```xml
    <include>
      <uri>
        https://fuel.gazebosim.org/1.0/OpenRobotics/models/Depot
      </uri>
      <experimental:params>
        ...
      </experimental:params>
    </include>
    ```

3.  삭제한 위치에, 1단계에서 만든 **로컬 모델을 불러오는 새로운 `<include>` 블록을 추가**합니다.

    **추가할 부분:**

    ```xml
    <include>
      <uri>model://my_map</uri>
    </include>
    ```

-----

### \#\# 3단계: 런치 파일 수정

마지막으로, 시뮬레이션을 시작하는 메인 런치 파일이 새로 만든 `my_world.sdf`를 사용하고, `my_map` 모델을 찾을 수 있도록 경로를 수정해야 합니다.

`tb4_simulation_launch.py`(또는 복사본) 파일을 열어 두 부분을 수정합니다.

1.  **월드 파일 경로 변경:** `declare_world_cmd`의 `default_value`를 `my_world.sdf`로 변경합니다.
2.  **모델 경로 추가:** `set_env_vars_resources`에 `models` 폴더 경로를 추가하여 Gazebo가 `model://my_map`을 찾을 수 있게 합니다.

**수정 예시 (`tb4_simulation_launch.py`):**

```python
    ...
    sim_dir = get_package_share_directory('my_robot_gazebo') # 사용자 패키지 이름
    ...
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        # 1. 월드 파일 경로를 my_world.sdf로 변경
        default_value=os.path.join(sim_dir, 'worlds', 'my_world.sdf'),
        description='Full path to world model file to load',
    )
    ...
    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            # 2. 기존 worlds 경로 뒤에 models 폴더 경로도 추가
            os.path.join(sim_dir, 'worlds') + ':' + os.path.join(sim_dir, 'models')
    )
    ...
```

이제 이 3단계를 거치면 Gazebo 시뮬레이션에 사용자의 커스텀 3D 맵이 성공적으로 나타날 것입니다\!

###############  



네, 알겠습니다. 기존 `unique_multi_tb3_simulation_launch.py` 파일의 복잡하고 오래된 구조를 버리고, 최신 ROS 2와 Gazebo 방식에 맞춰 여러 로봇을 정상적으로 실행할 수 있도록 수정된 전체 코드를 제공해 드리겠습니다.

-----

### \#\# 핵심 수정 방향

새로운 런치 파일은 다음과 같은 명확한 구조를 따릅니다.

  * Gazebo 시뮬레이션 월드(서버)는 **단 한 번만 실행**합니다.
  * 정의된 로봇 목록(`robots` 리스트)을 순회(loop)하면서 각 로봇에 대해 아래 3가지 작업을 독립적으로 실행합니다.
    1.  각 로봇의 URDF를 로드하는 **`robot_state_publisher`** 실행
    2.  로봇을 시뮬레이션에 생성(spawn)하는 **`spawn_tb3.launch.py`** 실행
    3.  각 로봇을 제어할 **`Nav2 스택(bringup_launch.py)`** 실행
  * 모든 로봇 관련 노드들은 충돌을 피하기 위해 `robot1`, `robot2`와 같이 고유한 \*\*네임스페이스(namespace)\*\*를 사용하도록 설정합니다.
  * RViz는 모든 로봇을 한 번에 볼 수 있도록 한 번만 실행합니다.

-----

### \#\# 수정된 `unique_multi_tb3_simulation_launch.py` 전체 코드

기존 `nav2_bringup/launch/unique_multi_tb3_simulation_launch.py` 파일의 내용을 아래 코드로 **전체 교체** 하세요.

```python
# Copyright (c) 2018 Intel Corporation
# Copyright (c) 2024 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import tempfile
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 패키지 경로 설정
    bringup_dir = get_package_share_directory('nav2_bringup')
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # 시뮬레이션할 로봇들의 이름과 초기 위치 정의
    robots = [
        {'name': 'robot1', 'x_pose': 0.0, 'y_pose': 0.5, 'yaw': 0.0},
        {'name': 'robot2', 'x_pose': 0.0, 'y_pose': -0.5, 'yaw': 0.0}
    ]

    # 런치 파라미터 선언
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(sim_dir, 'worlds', 'tb3_sandbox.sdf.xacro'),
        description='Full path to world file to load')
    
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'tb3_sandbox.yaml'),
        description='Full path to map file to load')

    params_file_arg_1 = DeclareLaunchArgument(
        'robot1_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_multirobot_params_1.yaml'),
        description='Full path to the ROS2 parameters file to use for robot1')

    params_file_arg_2 = DeclareLaunchArgument(
        'robot2_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_multirobot_params_2.yaml'),
        description='Full path to the ROS2 parameters file to use for robot2')

    rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_namespaced_view.rviz'),
        description='Full path to the RVIZ config file to use.')

    # 1. Gazebo 서버를 단 한 번만 실행
    world_sdf = tempfile.mktemp(prefix='nav2_', suffix='.sdf')
    world_sdf_xacro = ExecuteProcess(
        cmd=['xacro', '-o', world_sdf, ['headless:=', 'False'], LaunchConfiguration('world')])
    
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_sdf],
        output='screen')

    remove_temp_sdf_file = RegisterEventHandler(event_handler=OnShutdown(
        on_shutdown=[OpaqueFunction(function=lambda _: os.remove(world_sdf))]))

    # TurtleBot3 Waffle URDF 파일 내용 읽기
    urdf = os.path.join(sim_dir, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()
    
    # 2. RViz를 단 한 번만 실행
    start_rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        launch_arguments={'use_namespace': 'True',
                          'rviz_config': LaunchConfiguration('rviz_config')}.items())

    # 3. 각 로봇에 대한 노드 그룹 생성 (루프)
    robots_actions = []
    for robot in robots:
        robot_name = robot['name']
        
        # 각 로봇의 Nav2 파라미터 파일 지정
        params_file = LaunchConfiguration(f'{robot_name}_params_file')
        
        # 각 로봇의 네임스페이스에 맞게 TF 리매핑
        remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

        # 각 로봇에 대한 GroupAction 생성
        robot_group = GroupAction([
            # 3-1. 각 로봇의 Robot State Publisher 실행
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=robot_name,
                output='screen',
                parameters=[{'use_sim_time': True,
                             'robot_description': robot_description,
                             'frame_prefix': f'{robot_name}/'}],
                remappings=remappings),

            # 3-2. 각 로봇을 Gazebo에 생성(spawn)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sim_dir, 'launch', 'spawn_tb3.launch.py')),
                launch_arguments={
                    'namespace': robot_name,
                    'robot_name': robot_name,
                    'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                    'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                    'yaw': TextSubstitution(text=str(robot['yaw'])),
                }.items()),

            # 3-3. 각 로봇의 Nav2 스택 실행
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'namespace': robot_name,
                    'use_namespace': 'True',
                    'map': LaunchConfiguration('map'),
                    'use_sim_time': 'True',
                    'params_file': params_file,
                    'autostart': 'true',
                }.items()),
        ])
        robots_actions.append(robot_group)

    # LaunchDescription 생성 및 액션 추가
    ld = LaunchDescription()
    
    # 런치 파라미터 선언 추가
    ld.add_action(world_arg)
    ld.add_action(map_arg)
    ld.add_action(params_file_arg_1)
    ld.add_action(params_file_arg_2)
    ld.add_action(rviz_arg)
    ld.add_action(rviz_config_arg)

    # Gazebo 관련 액션 추가
    ld.add_action(world_sdf_xacro)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(remove_temp_sdf_file)
    
    # RViz 액션 추가
    ld.add_action(start_rviz_cmd)

    # 각 로봇 그룹 액션 추가
    for group in robots_actions:
        ld.add_action(group)

    return ld

```

-----

### \#\# 사용 방법

1.  기존 `nav2_bringup/launch/unique_multi_tb3_simulation_launch.py` 파일의 내용을 위 코드로 완전히 교체하고 저장합니다.
2.  터미널에서 작업 공간의 루트(`~/turtlebot_ws`)로 이동하여, 수정 사항을 적용하기 위해 다시 빌드하고 소싱합니다.
    ```bash
    cd ~/turtlebot_ws
    colcon build --packages-select nav2_bringup
    source install/setup.bash
    ```
3.  수정된 런치 파일을 실행합니다.
    ```bash
    ros2 launch nav2_bringup unique_multi_tb3_simulation_launch.py
    ```

이제 Gazebo에 로봇 두 대가 각각의 초기 위치에 생성되고, 각 로봇을 제어하는 Nav2 스택이 독립적으로 실행될 것입니다. RViz에서도 두 로봇의 상태를 모두 확인할 수 있습니다.



죄송합니다. 제가 이전에 제공해 드린 수정된 런치 코드에 **Gazebo 모델 경로를 설정하는 중요한 부분이 누락**되었습니다. 문제가 발생한 것이 당연합니다.

`Unable to find uri[model://turtlebot3_world]` 오류는 이전 `my_map` 오류와 원인이 100% 동일합니다. Gazebo가 `turtlebot3_world`라는 모델을 찾아야 하는데, 어디서 찾아야 할지 경로를 모르기 때문에 발생하는 것입니다.

-----

### \#\# 문제 분석: 모델 경로 누락

`tb3_simulation_launch.py`와 `unique_multi_tb3_simulation_launch.py`는 모두 `tb3_sandbox.sdf.xacro`라는 월드 파일을 사용합니다. 이 월드 파일 안에는 `turtlebot3_world`라는 3D 모델(벽, 장애물 등)을 불러오는 `<include uri="model://turtlebot3_world">` 코드가 포함되어 있습니다.

이 `turtlebot3_world` 모델은 `nav2_minimal_tb3_sim/models/` 폴더 안에 있습니다.

제가 드린 수정 코드에서는 이 `models` 폴더의 경로를 Gazebo에게 알려주는 `AppendEnvironmentVariable` 부분이 빠져있었습니다.

-----

### \#\# 해결 방법: 런치 파일에 경로 추가

이전에 수정했던 `unique_multi_tb3_simulation_launch.py` 파일을 다시 열고, Gazebo 모델 경로를 추가하는 코드를 삽입해야 합니다.

아래는 **경로 설정 코드가 추가된, 최종 수정된 전체 코드**입니다. 이 내용으로 `unique_multi_tb3_simulation_launch.py` 파일을 다시 한번 덮어쓰고 저장해주세요.

```python
# Copyright (c) 2018 Intel Corporation
# Copyright (c) 2024 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import tempfile
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction,
                            AppendEnvironmentVariable) # AppendEnvironmentVariable 추가
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 패키지 경로 설정
    bringup_dir = get_package_share_directory('nav2_bringup')
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # 시뮬레이션할 로봇들의 이름과 초기 위치 정의
    robots = [
        {'name': 'robot1', 'x_pose': 0.0, 'y_pose': 0.5, 'yaw': 0.0},
        {'name': 'robot2', 'x_pose': 0.0, 'y_pose': -0.5, 'yaw': 0.0}
    ]

    # ==========================================================================================
    # [수정된 부분 START] Gazebo가 모델을 찾을 수 있도록 GZ_SIM_RESOURCE_PATH 환경 변수 설정
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(sim_dir, 'models')
    )
    # ==========================================================================================

    # 런치 파라미터 선언
    # ... (이전과 동일한 파라미터 선언 부분) ...
    world_arg = DeclareLaunchArgument(...)
    map_arg = DeclareLaunchArgument(...)
    params_file_arg_1 = DeclareLaunchArgument(...)
    params_file_arg_2 = DeclareLaunchArgument(...)
    rviz_arg = DeclareLaunchArgument(...)
    rviz_config_arg = DeclareLaunchArgument(...)

    # 1. Gazebo 서버를 단 한 번만 실행
    # ... (이전과 동일한 Gazebo 실행 부분) ...
    world_sdf = tempfile.mktemp(...)
    world_sdf_xacro = ExecuteProcess(...)
    start_gazebo_cmd = ExecuteProcess(...)
    remove_temp_sdf_file = RegisterEventHandler(...)
    
    # TurtleBot3 Waffle URDF 파일 내용 읽기
    # ... (이전과 동일) ...
    urdf = os.path.join(sim_dir, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()
    
    # 2. RViz를 단 한 번만 실행
    # ... (이전과 동일) ...
    start_rviz_cmd = IncludeLaunchDescription(...)

    # 3. 각 로봇에 대한 노드 그룹 생성 (루프)
    # ... (이전과 동일한 루프 부분) ...
    robots_actions = []
    for robot in robots:
        # ... (이전과 동일) ...
        robot_group = GroupAction([...])
        robots_actions.append(robot_group)

    # LaunchDescription 생성 및 액션 추가
    ld = LaunchDescription()
    
    # [수정된 부분] 환경 변수 설정 액션을 가장 먼저 추가
    ld.add_action(set_env_vars_resources)

    # 런치 파라미터 선언 추가
    # ... (이전과 동일) ...
    ld.add_action(world_arg)
    ld.add_action(map_arg)
    ld.add_action(params_file_arg_1)
    ld.add_action(params_file_arg_2)
    ld.add_action(rviz_arg)
    ld.add_action(rviz_config_arg)

    # Gazebo 관련 액션 추가
    # ... (이전과 동일) ...
    ld.add_action(world_sdf_xacro)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(remove_temp_sdf_file)
    
    # RViz 액션 추가
    ld.add_action(start_rviz_cmd)

    # 각 로봇 그룹 액션 추가
    for group in robots_actions:
        ld.add_action(group)

    return ld
```

*(코드의 생략된 부분(...)은 이전 답변의 내용과 동일합니다. 전체 코드를 다시 붙여넣으셔도 되고, `AppendEnvironmentVariable` 관련 부분만 추가/수정하셔도 됩니다.)*

-----

### \#\# (재확인) `CMakeLists.txt` 점검

이전 `my_map` 문제 때와 마찬가지로, `nav2_minimal_tb3_sim` 패키지 자체의 `CMakeLists.txt` 파일에 `models` 폴더가 `install` 목록에 포함되어 있는지도 다시 한번 확인해주세요.

**`~/turtlebot_ws/src/nav2_minimal_turtlebot_simulation/nav2_minimal_tb3_sim/CMakeLists.txt`** 파일을 열어 아래와 같이 `models`가 있는지 확인하고, 없다면 추가해주세요.

```cmake
install(DIRECTORY
  launch
  worlds
  models  # <-- 이 부분이 있는지 확인!
  urdf
  ...
  DESTINATION share/${PROJECT_NAME}
)
```

-----

### \#\# 최종 실행 순서

1.  위와 같이 `unique_multi_tb3_simulation_launch.py` 파일을 수정합니다.
2.  (필요하다면) `nav2_minimal_tb3_sim`의 `CMakeLists.txt`도 수정합니다.
3.  터미널에서 작업 공간을 **깨끗하게 다시 빌드**하고 실행합니다.
    ```bash
    cd ~/turtlebot_ws
    rm -rf install build log
    colcon build
    source install/setup.bash
    ros2 launch nav2_bringup unique_multi_tb3_simulation_launch.py
    ```

제 실수로 중요한 부분을 누락하여 혼란을 드린 점 사과드립니다. 위와 같이 수정하시면 Gazebo가 `turtlebot3_world` 모델을 정상적으로 찾을 수 있을 것입니다.
