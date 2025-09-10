```xacro

    <!-- ... 기존 코드 ... -->

    <!-- ======================= ZED CAMERA 매크로 호출 부분 =================================== -->

    <!-- 1. 전방 ZED 카메라 -->
    <xacro:zed_camera_from_macro prefix="${prefix}" parent="${prefix}base_link" name="zed_front" model="zed2i">
      <origin xyz="0.4 0.0 0.3" rpy="0 0 0" />
    </xacro:zed_camera_from_macro>

    <!-- 2. 후방 ZED 카메라 -->
    <xacro:zed_camera_from_macro prefix="${prefix}" parent="${prefix}base_link" name="zed_back" model="zed2i">
      <origin xyz="-0.4 0.0 0.3" rpy="0 0 ${pi}" />
    </xacro:zed_camera_from_macro>
    
    <!-- 3. 왼쪽 ZED 카메라 -->
    <xacro:zed_camera_from_macro prefix="${prefix}" parent="${prefix}base_link" name="zed_left" model="zed2i">
      <!-- 로봇 중심에서 왼쪽(Y축 +)을 바라보도록 90도 회전 -->
      <origin xyz="0.0 0.25 0.3" rpy="0 0 ${pi/2}" />
    </xacro:zed_camera_from_macro>

    <!-- 4. 오른쪽 ZED 카메라 -->
    <xacro:zed_camera_from_macro prefix="${prefix}" parent="${prefix}base_link" name="zed_right" model="zed2i">
      <!-- 로봇 중심에서 오른쪽(Y축 -)을 바라보도록 -90도 회전 -->
      <origin xyz="0.0 -0.25 0.3" rpy="0 0 -${pi/2}" />
    </xacro:zed_camera_from_macro>
    
    <!-- 5. 전방 45도 ZED 카메라 (예시) -->
    <xacro:zed_camera_from_macro prefix="${prefix}" parent="${prefix}base_link" name="zed_front_45" model="zed2i">
      <!-- 전방을 기준으로 45도 (pi/4) 회전 -->
      <origin xyz="0.35 0.15 0.3" rpy="0 0 ${pi/4}" />
    </xacro:zed_camera_from_macro>

    <!-- ======================================================================================= -->


    <!-- Ultrasound sensors -->
    <!-- ... 이하 기존 코드 ... -->

```


#########

```python

# ... 기존 브릿지 노드들 ...

# Bridge for the left camera
gz_bridge_left_camera = Node(
    package='ros_gz_bridge', executable='parameter_bridge',
    name='gz_bridge_left_camera',
    arguments=['/zed_left/image@sensor_msgs/msg/Image[ignition.msgs.Image',
               '/zed_left/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
               '/zed_left/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'],
    output='screen'
)

# Bridge for the right camera
gz_bridge_right_camera = Node(
    package='ros_gz_bridge', executable='parameter_bridge',
    name='gz_bridge_right_camera',
    arguments=['/zed_right/image@sensor_msgs/msg/Image[ignition.msgs.Image',
               # ... (depth_image, camera_info 추가)
              ],
    output='screen'
)

# Bridge for the front_45 camera
gz_bridge_front_45_camera = Node(
    package='ros_gz_bridge', executable='parameter_bridge',
    name='gz_bridge_front_45_camera',
    arguments=['/zed_front_45/image@sensor_msgs/msg/Image[ignition.msgs.Image',
               # ... (depth_image, camera_info 추가)
              ],
    output='screen'
)

# ... LaunchDescription에 모든 브릿지 노드 추가
launch_description.add_action(gz_bridge_left_camera)
launch_description.add_action(gz_bridge_right_camera)
launch_description.add_action(gz_bridge_front_45_camera)
```
