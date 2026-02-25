franka_gazebo
=============

.. important::

    Minimum necessary `franka_description` version is 0.3.0.
    You can clone franka_description package from https://github.com/frankarobotics/franka_description.

A project integrating Franka ROS 2 with the Gazebo simulator.

Launch RVIZ + Gazebo
--------------------

Launch an example which spawns RVIZ and Gazebo showing the robot:


.. code-block:: shell

    ros2 launch franka_gazebo_bringup visualize_franka_robot.launch.py

If you want to display another robot, you can define the robot_type:

.. code-block:: shell

    ros2 launch franka_gazebo_bringup visualize_franka_robot.launch.py robot_type:=fp3

If you want to start the simulation including the franka_hand:

.. code-block:: shell

    ros2 launch franka_gazebo_bringup visualize_franka_robot.launch.py load_gripper:=true franka_hand:='franka_hand'

Joint Velocity Control Example with Gazebo
-------------------------------------------

Before starting, be sure to build `franka_example_controllers` and `franka_description` packages.
`franka_description` must have the minimum version of 0.3.0.


.. code-block:: shell

    colcon build --packages-select franka_example_controllers


Now you can launch the velocity example with Gazebo simulator.

.. code-block:: shell

    ros2 launch franka_gazebo_bringup gazebo_joint_velocity_controller_example.launch.py load_gripper:=true franka_hand:='franka_hand'


Keep in mind that the gripper joint has a bug with the joint velocity controller.
If you are interested in controlling the gripper please use joint position interface.


Joint Position Control Example with Gazebo
-------------------------------------------

To run the joint position control example you need to have the required software listed in the joint velocity control section.

Then you can run with the following command.

.. code-block:: shell

    ros2 launch franka_gazebo_bringup gazebo_joint_position_controller_example.launch.py load_gripper:=true franka_hand:='franka_hand'


Joint Impedance Control Example with Gazebo
--------------------------------------------

For running torque example:

.. code-block:: shell

    ros2 launch franka_gazebo_bringup gazebo_joint_impedance_controller_example.launch.py load_gripper:=true franka_hand:='franka_hand'


Multi-Robot Control Example with Gazebo
-------------------------------------------------------

For running multiple robots simultaneously in Gazebo, each in their own namespace with dedicated URDFs. The example is designed to be run with two `fr3v2` and one `tmrv0_2`.

First, configure the robots in `franka_bringup/config/franka.config.yaml`. Uncomment and set `robot_type` and `arm_prefix` for each robot you want to spawn. 
Multi-robot configurations (positions and controllers) are defined in `franka_gazebo_bringup/config/gazebo_multi_robot_configs.yaml`.
Ensure `franka_example_controllers` and `franka_description` are built.

.. code-block:: shell

    colcon build --packages-up-to franka_example_controllers

Then source your workspace.

.. code-block:: shell

    source install/setup.sh

Then you can run the multi-robot control example.

.. code-block:: shell

    ros2 launch franka_gazebo_bringup multi_robot_gazebo_example.launch.py 

This will launch Gazebo with the configured robots, each in their own namespace. Use ``with_sensors:=true`` to enable sensor-enhanced descriptions for ``tmrv0_2`` robots.


Troubleshooting
---------------

If you experience that Gazebo can't find your model files, try to include the workspace. E.g.


.. code-block:: shell

    export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:/workspaces/src/