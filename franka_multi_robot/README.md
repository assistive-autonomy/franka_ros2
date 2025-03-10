# Quickstart for the Multi Robot setup: TMR + 2 FR3s

After following the installation instructions for the normal `franka_ros2` workspace, you can directly start to execute examples.
E.g. if you want to start a basic example which mostly shows the state of the robot, run
```
ros2 launch franka_multi_robot_bringup franka_multi_robot.launch.py use_rviz:=true
```

An example for running the platforms in gravity compensation mode (aka `joint effort`), run
```
ros2 launch franka_multi_robot_bringup franka_multi_robot_gravity_example_controller.launch.py use_rviz:=true
```

To see an example to command `joint velocities` to the joints, see
```
ros2 launch franka_multi_robot_bringup franka_multi_robot_joint_velocity_example_controller.launch.py use_rviz:=true
```
