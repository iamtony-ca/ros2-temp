
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
