#  Copyright (c) 2024 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def robot_description_dependent_nodes_spawner(
    context: LaunchContext,
    robot_ip,
    mobile_robot_id,
    arm_id,
    use_fake_hardware,
    fake_sensor_commands,
):

    robot_ip_str = context.perform_substitution(robot_ip)
    mobile_robot_id_str = context.perform_substitution(mobile_robot_id)
    arm_id_str = context.perform_substitution(arm_id)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    fake_sensor_commands_str = context.perform_substitution(fake_sensor_commands)

    franka_xacro_filepath = os.path.join(
        get_package_share_directory("franka_description"),
        "robots",
        "tmr.urdf.xacro",
    )
    robot_description = xacro.process_file(
        franka_xacro_filepath,
        mappings={
            "ros2_control": "true",
            "mobile_robot_id": mobile_robot_id_str,
            "robot_ip": robot_ip_str,
            "use_fake_hardware": use_fake_hardware_str,
            "fake_sensor_commands": fake_sensor_commands_str,
        },
    ).toprettyxml(indent="  ")

    franka_controllers = PathJoinSubstitution(
        [FindPackageShare("franka_mobile_bringup"), "config", "controllers.yaml"]
    )

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                franka_controllers,
                {"robot_description": robot_description},
                {"mobile_robot_id": mobile_robot_id},
                {"arm_id": arm_id},
            ],
            remappings=[("joint_states", mobile_robot_id_str + "/joint_states")],
            output={
                "stdout": "screen",
                "stderr": "screen",
            },
            on_exit=Shutdown(),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            parameters=[{"source_list": [mobile_robot_id_str + "/joint_states"], "rate": 30}],
        ),
    ]


def generate_launch_description():
    mobile_robot_id_parameter_name = "mobile_robot_id"
    arm_id_parameter_name = "arm_id"
    robot_ip_parameter_name = "robot_ip"
    use_fake_hardware_parameter_name = "use_fake_hardware"
    fake_sensor_commands_parameter_name = "fake_sensor_commands"
    use_rviz_parameter_name = "use_rviz"

    mobile_robot_id = LaunchConfiguration(mobile_robot_id_parameter_name)
    arm_id = LaunchConfiguration(arm_id_parameter_name)
    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)

    rviz_file = os.path.join(get_package_share_directory("franka_description"), "rviz", "tmr.rviz")

    robot_description_dependent_nodes_spawner_opaque_function = OpaqueFunction(
        function=robot_description_dependent_nodes_spawner,
        args=[robot_ip, mobile_robot_id, arm_id, use_fake_hardware, fake_sensor_commands],
    )

    launch_description = LaunchDescription(
        [
            DeclareLaunchArgument(
                robot_ip_parameter_name,
                default_value="172.16.1.20",
                description="Hostname or IP address of the robot.",
            ),
            DeclareLaunchArgument(
                mobile_robot_id_parameter_name,
                default_value="tmr",
                description="Name of the robot.",
            ),
            DeclareLaunchArgument(
                arm_id_parameter_name,
                default_value="tmr",
                description="ID of the type of mobile robot used. Supported values: tmr",
            ),
            DeclareLaunchArgument(
                use_rviz_parameter_name,
                default_value="false",
                description="Visualize the robot in Rviz",
            ),
            DeclareLaunchArgument(
                use_fake_hardware_parameter_name,
                default_value="false",
                description="Use fake hardware",
            ),
            DeclareLaunchArgument(
                fake_sensor_commands_parameter_name,
                default_value="false",
                description='Fake sensor commands. Only valid when "{}" is true'.format(
                    use_fake_hardware_parameter_name
                ),
            ),
            robot_description_dependent_nodes_spawner_opaque_function,
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["--controller-manager-timeout", "2", "joint_state_broadcaster"],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["--display-config", rviz_file],
                condition=IfCondition(use_rviz),
            ),
        ]
    )

    return launch_description
