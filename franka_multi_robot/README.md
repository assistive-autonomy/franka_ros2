# Humanoid ROS 2

ROS2 packages for controlling a Franka humanoid robot.

## Usage

If you want to start the example, run

```
ros2 launch franka_multi_robot_bringup multi_robot.launch.py robot_ips:="['192.168.1.21','192.168.1.22', '192.168.1.20']" use_rviz:=true arm_ids:="['fr3','fr3','tmr']" arm_prefixes:="['right','left','platform']" number_of_robots:="3"
```
