
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
