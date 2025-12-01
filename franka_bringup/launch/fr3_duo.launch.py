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

############################################################################
# Parameters:
# robot_types: Types of the robot arms as a string list (e.g., "['fr3','fr3']")
# arm_prefixes: Prefixes for the arms as a string list (e.g., "['right','left']")
# robot_ips: IP addresses of the robots as a string list (e.g., "['172.16.0.5','172.16.0.6']")
# load_gripper: Use Franka Gripper as end-effector (default: 'false')
# use_fake_hardware: Use fake hardware (default: 'false')
# fake_sensor_commands: Fake sensor commands (default: 'false')
# is_async: Use async hardware interface (default: 'true')
# joint_state_rate: Rate for joint state publishing in Hz (default: '30')
# namespace: Namespace for the robot (default: '')
#
# The fr3_duo.launch.py launch file provides a robust interface for launching
# a Franka Robotics dual-arm setup. It generates the robot description from the
# duo URDF xacro file and launches the necessary nodes for controlling both arms.
#
# NOTE: The franka_robot_state_broadcaster is NOT launched for duo setups as it
# is not supported for multi-arm configurations.
#
# This script is designed to be called (included) by example.launch.py when a
# duo configuration is detected in the config file.
############################################################################

import ast

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    Shutdown,
)

from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def parse_string_list(string_list_repr):
    """Parse a string representation of a list into an actual Python list.

    Handles formats like "['item1','item2']" or "['item1', 'item2']".
    """
    try:
        return ast.literal_eval(string_list_repr)
    except (ValueError, SyntaxError):
        # Fallback: try to parse manually if ast fails
        cleaned = string_list_repr.strip('[]').replace("'", '').replace('"', '')
        return [item.strip() for item in cleaned.split(',')]


def generate_robot_nodes(context):
    robot_ips_str = LaunchConfiguration('robot_ips').perform(context)
    robot_types_str = LaunchConfiguration('robot_types').perform(context)
    arm_prefixes_str = LaunchConfiguration('arm_prefixes').perform(context)
    use_fake_hardware_str = LaunchConfiguration('use_fake_hardware').perform(context)
    fake_sensor_commands_str = LaunchConfiguration('fake_sensor_commands').perform(
        context
    )
    load_gripper_str = LaunchConfiguration('load_gripper').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    controllers_yaml = LaunchConfiguration('controllers_yaml').perform(context)
    joint_state_rate = int(LaunchConfiguration('joint_state_rate').perform(context))

    # Parse string list representations into actual Python lists for ros2_control_node
    robot_types_list = parse_string_list(robot_types_str)
    arm_prefixes_list = parse_string_list(arm_prefixes_str)

    # Build URDF path based on the first robot type
    base_robot_type = robot_types_list[0]
    urdf_path = PathJoinSubstitution(
        [
            FindPackageShare('franka_description'),
            'robots',
            f'{base_robot_type}_duo',
            f'{base_robot_type}_duo.urdf.xacro',
        ]
    ).perform(context)

    robot_description = xacro.process_file(
        urdf_path,
        mappings={
            'ros2_control': 'true',
            'robot_types': robot_types_str,
            'robot_ips': robot_ips_str,
            'hand': load_gripper_str,
            'use_fake_hardware': use_fake_hardware_str,
            'fake_sensor_commands': fake_sensor_commands_str,
            'is_async': 'true',
        },
    ).toprettyxml(indent='  ')

    joint_state_publisher_sources = [
        'franka/joint_states',
        'franka_gripper/joint_states',
    ]

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace=namespace,
            parameters=[
                controllers_yaml,
                {'robot_description': robot_description},
                {'robot_types': robot_types_list},
                {'arm_prefixes': arm_prefixes_list},
            ],
            remappings=[('joint_states', 'franka/joint_states')],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
            on_exit=Shutdown(),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=namespace,
            parameters=[
                {
                    'source_list': joint_state_publisher_sources,
                    'rate': joint_state_rate,
                }
            ],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        # NOTE: franka_robot_state_broadcaster is NOT launched for duo setups
        # as it is not supported for multi-arm configurations.
        # TODO: Gripper feature is not implemented yet.
    ]

    return nodes


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'robot_ips',
            description='IP addresses or hostnames of the robots as a string list',
        ),
        DeclareLaunchArgument(
            'robot_types',
            description='Types of the robot arms as a string list (e.g., fr3, fr3v2, fer, fp3)',
        ),
        DeclareLaunchArgument(
            'arm_prefixes', description='Prefixes for the arms as a string list'
        ),
        DeclareLaunchArgument(
            'load_gripper',
            default_value='false',
            description='Use Franka Gripper as an end-effector',
        ),
        DeclareLaunchArgument(
            'use_fake_hardware', default_value='false', description='Use fake hardware'
        ),
        DeclareLaunchArgument(
            'fake_sensor_commands',
            default_value='false',
            description='Fake sensor commands',
        ),
        DeclareLaunchArgument(
            'joint_state_rate',
            default_value='30',
            description='Rate for joint state publishing (Hz)',
        ),
        DeclareLaunchArgument(
            'namespace', default_value='', description='Namespace for the robot'
        ),
        DeclareLaunchArgument(
            'controllers_yaml',
            default_value=PathJoinSubstitution(
                [FindPackageShare('franka_bringup'), 'config', 'controllers.yaml']
            ),
            description='Override the default controllers.yaml file',
        ),
    ]

    return LaunchDescription(
        launch_args + [OpaqueFunction(function=generate_robot_nodes)]
    )
