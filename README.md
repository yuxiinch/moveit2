# dynamixel_control

The [`ros2_control`](https://github.com/ros-controls/ros2_control) implementation for any kind of [ROBOTIS Dynamixel](https://emanual.robotis.com/docs/en/dxl/) robots.

- `dynamixel_hardware`: the [`SystemInterface`](https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/system_interface.hpp) implementation for the multiple ROBOTIS Dynamixel servos.

The `dynamixel_hardware` package is hopefully compatible any configuration of ROBOTIS Dynamixel servos thanks to the `ros2_control`'s flexible architecture.

## Set up

First [install ROS 2 Humble on Ubuntu 22.04](http://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). Then follow the instruction below.

```shell
$ vcs import src < src/tb4_arm_ros2/dynamixel_control.repos
$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
$ . install/setup.bash
```

## Demo with real ROBOTIS small arm

### Configure Dynamixel motor parameters

Update the `usb_port`, `baud_rate`, and `joint_ids` parameters on [`small_arm_description/urdf/open_manipulator_x.ros2_control.xacro`](https://github.com/youtalk/dynamixel_control/blob/main/open_manipulator_x_description/urdf/open_manipulator_x.ros2_control.xacro#L9-L12) to correctly communicate with Dynamixel motors.
The `use_dummy` parameter is required if you don't have a real small arm.

Note that `joint_ids` parameters must be splited by `,`.

```xml
<hardware>
  <plugin>dynamixel_hardware/DynamixelHardware</plugin>
  <param name="usb_port">/dev/ttyUSB0</param>
  <param name="baud_rate">1000000</param>
  <!-- <param name="use_dummy">true</param> -->
</hardware>
```

- Terminal 1

Launch the `ros2_control` manager for the OpenManipulator-X.

```shell
$ ros2 launch small_arm_description small_arm.launch.py
```

- Terminal 2

Start the `joint_trajectory_controller` and send a `/joint_trajectory_controller/follow_joint_trajectory` goal to move the small_arm.

```shell
$ ros2 control switch_controllers --activate joint_state_broadcaster --activate joint_trajectory_controller --deactivate velocity_controller
$ ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory -f "{
  trajectory: {
    joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],
    points: [
      { positions: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1], time_from_start: { sec: 2 } },
      { positions: [-0.1, -0.1, -0.1, -0.1, 0.1, 0], time_from_start: { sec: 4 } },
      { positions: [0, 0, 0, 0, 0, 0], time_from_start: { sec: 6 } }
    ]
  }
}"
```

If you would like to use the velocity control instead, switch to the `velocity_controller` and publish a `/velocity_controller/commands` message to move the small arm.

```shell
$ ros2 control switch_controllers --activate joint_state_broadcaster --deactivate joint_trajectory_controller --activate velocity_controller
$ ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [0.1, 0.1, 0.1, 0.1, 0]"
```

## Demo with dummy ROBOTIS small arm

The `use_dummy` parameter is required if you use the dummy small arm.

```diff
diff --git a/small_arm_description/urdf/small_arm.ros2_control.xacro b/small_arm_description/urdf/small_arm.ros2_control.ros2_control.xacro
index c6cdb74..111846d 100644
--- a/small_arm_description/urdf/small_arm.ros2_control.ros2_control.xacro
+++ b/small_arm_description/urdf/small_arm.ros2_control.ros2_control.xacro
@@ -9,7 +9,7 @@
         <param name="usb_port">/dev/ttyUSB0</param>
         <param name="baud_rate">1000000</param>
-        <!-- <param name="use_dummy">true</param> -->
+        <param name="use_dummy">true</param>
       </hardware>
       <joint name="joint1">
         <param name="id">11</param>
```

Then follow the same instruction of the real robot one.

Note that the dummy implementation has no interpolation so far.
If you sent a joint message, the robot would move directly to the joints without interpolation.
