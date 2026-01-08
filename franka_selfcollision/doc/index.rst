franka_selfcollision
==============

This package contains the library and the service for the FR3_duo self-collision check.

.. important::

    Minimum necessary `franka_description` version is 2.3.0.
    You can clone franka_description package from https://github.com/frankarobotics/franka_description.

Functionality
-------------

This controller node is spawned by fr3_duo.launch.py in the franka_bringup if `check_selfcollision` is enabled in the config.
The service node (`/check_self_collision`) checks joint configurations for self-collisions between FR3_duo-robot links.

Configuration
-------------

Parameters are defined in the config files:

* ``security_margin``: Safety margin around the robot links in meters for collision checking
* ``print_collisions``: Enable collision pair logging

Usage
-----

The self-collision controller is automatically started when you launch the robot if `check_selfcollision` is enabled:

.. code-block:: shell

    ros2 launch franka_bringup fr3_duo.launch.py \
        robot_config_file:=fr3_duo.config.yaml \
        controller_name:=<controller_name>

The self-collision service can be started standalone once the robot is started:

.. code-block:: shell

    ros2 run franka_selfcollision self_collision_service
