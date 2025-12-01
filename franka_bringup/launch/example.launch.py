#  Copyright (c) 2025 Franka Robotics GmbH
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
# controller_names: Comma-separated list of controller names to spawn (required, no default)
# robot_config_file: Configuration file name or path. If just a filename is
#                   provided (e.g., 'fr3_duo.config.yaml'), it will be
#                   looked up in franka_bringup/config/ directory.
#                   (default: franka.config.yaml)
#
# The example.launch.py launch file provides a flexible and unified interface
# for launching Franka Robotics example controllers via the 'controller_names'
# parameter, such as 'elbow_example_controller'.
# Example:
# ros2 launch franka_bringup example.launch.py controller_names:=elbow_example_controller
#
# This script "includes" franka.launch.py to declare core component nodes,
# including: robot_state_publisher, ros2_control_node, joint_state_publisher,
# joint_state_broadcaster, franka_robot_state_broadcaster, and optionally
# franka_gripper and rviz, with support for namespaced and non-namespaced
# environments as defined in franka.config.yaml. RViz is launched if
# 'use_rviz' is set to true in the configuration file.
#
# The default robot_config_file is franka.config.yaml in the
# franka_bringup/config directory. See that file for its own documentation.
#
# This approach improves upon the earlier individual launch scripts, which
# varied in structure and lacked namespace support, offering a more consistent
# and maintainable solution. While some may favor the older scripts for their
# specific use cases, example.launch.py enhances scalability and ease of use
# for a wide range of Franka Robotics applications.
#
# Ensure the specified controller_names match controllers defined in
# controllers.yaml to avoid runtime errors.
############################################################################

import ast
import importlib.util
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# constant for the controller name parameter
CONTROLLER_EXAMPLE = 'controller'


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

package_share = get_package_share_directory('franka_bringup')
utils_path = os.path.abspath(
    os.path.join(package_share, '..', '..', 'lib', 'franka_bringup', 'utils')
)
launch_utils_path = os.path.join(utils_path, 'launch_utils.py')

spec = importlib.util.spec_from_file_location('launch_utils', launch_utils_path)
launch_utils = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_utils)

load_yaml = launch_utils.load_yaml

# Iterates over the uncommented lines in file specified by the robot_config_file parameter.
# 'Includes' franka.launch.py for each active (uncommented) Robot.
# That file is well documented.
# The function also checks if the 'use_rviz' parameter is set to true in the YAML file.
# If so, it includes a node for RViz to visualize the robot's state.
# The function returns a list of nodes to be launched.


def generate_robot_nodes(context):
    config_file = LaunchConfiguration('robot_config_file').perform(context)
    
    # If config_file is just a filename (no path separators), look in franka_bringup/config/
    if not os.path.isabs(config_file) and os.path.sep not in config_file:
        config_file = os.path.join(package_share, 'config', config_file)
    
    controller_names = LaunchConfiguration('controller_names').perform(context)
    controller_names_vector = controller_names.split(',')
    configs = load_yaml(config_file)
    nodes = []

    for index, (_, config) in enumerate(configs.items()):
        namespace = config.get('namespace', '')
        # Detect duo setup by checking for plural keys (robot_types, robot_ips, arm_prefixes)
        # that are unique to multi-robot configurations
        duo_keys = {'robot_types', 'robot_ips', 'arm_prefixes'}
        is_duo = duo_keys.issubset(config.keys())

        if is_duo:
            # Validate that all duo arrays have the same length
            robot_types_list = parse_string_list(str(config['robot_types']))
            robot_ips_list = parse_string_list(str(config['robot_ips']))
            arm_prefixes_list = parse_string_list(str(config['arm_prefixes']))

            if not (len(robot_types_list) == len(robot_ips_list) == len(arm_prefixes_list)):
                print(
                    f'Error: Duo configuration arrays must have the same length.\n'
                    f'  robot_types:  {len(robot_types_list)} items: {robot_types_list}\n'
                    f'  robot_ips:    {len(robot_ips_list)} items: {robot_ips_list}\n'
                    f'  arm_prefixes: {len(arm_prefixes_list)} items: {arm_prefixes_list}\n'
                    f'Please check your configuration file and ensure all arrays '
                    f'have the same number of elements.'
                )
                sys.exit(1)

            # Validate that arm_prefixes are unique
            if len(arm_prefixes_list) != len(set(arm_prefixes_list)):
                duplicates = [p for p in arm_prefixes_list if arm_prefixes_list.count(p) > 1]
                print(
                    f'Error: arm_prefixes must be unique.\n'
                    f'  arm_prefixes: {arm_prefixes_list}\n'
                    f'  Duplicate values: {list(set(duplicates))}\n'
                    f'Each robot arm requires a unique prefix to avoid naming conflicts.'
                )
                sys.exit(1)

            # Duo configuration: use fr3_duo.launch.py for dual-arm setup
            nodes.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution(
                            [
                                FindPackageShare('franka_bringup'),
                                'launch',
                                'fr3_duo.launch.py',
                            ]
                        )
                    ),
                    launch_arguments={
                        'robot_types': str(config['robot_types']),
                        'arm_prefixes': str(config['arm_prefixes']),
                        'robot_ips': str(config['robot_ips']),
                        'namespace': str(namespace),
                        'load_gripper': str(config.get('load_gripper', 'false')),
                        'use_fake_hardware': str(config.get('use_fake_hardware', 'false')),
                        'fake_sensor_commands': str(
                            config.get('fake_sensor_commands', 'false')
                        ),
                        'joint_state_rate': str(config.get('joint_state_rate', 30)),
                    }.items(),
                )
            )
        else:
            # Single robot configuration: use franka.launch.py
            nodes.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution(
                            [
                                FindPackageShare('franka_bringup'),
                                'launch',
                                'franka.launch.py',
                            ]
                        )
                    ),
                    launch_arguments={
                        'robot_type': str(config['robot_type']),
                        'arm_prefix': str(config['arm_prefix']),
                        'namespace': str(namespace),
                        'robot_ip': str(config['robot_ip']),
                        'load_gripper': str(config['load_gripper']),
                        'use_fake_hardware': str(config['use_fake_hardware']),
                        'fake_sensor_commands': str(config['fake_sensor_commands']),
                        'joint_state_rate': str(config['joint_state_rate']),
                    }.items(),
                )
            )

        # Determine which controller to use for this config
        if controller_names_vector:
            if len(controller_names_vector) == len(configs):
                controller_name = controller_names_vector[index]
            else:
                print(
                    'Warning: Number of controller names does not match number of robot configs.'
                    ' Using the first controller for all robots.'
                )
                controller_name = controller_names_vector[0]
        else:
            print(
                'Error: No controller names provided. Please provide at least one controller name.'
            )
            sys.exit(1)

        if CONTROLLER_EXAMPLE in controller_name:
            # Spawn the example as ros2_control controller
            nodes.append(
                Node(
                    package='controller_manager',
                    executable='spawner',
                    namespace=namespace,
                    arguments=[controller_name, '--controller-manager-timeout', '30'],
                    parameters=[
                        PathJoinSubstitution(
                            [
                                FindPackageShare('franka_bringup'),
                                'config',
                                'controllers.yaml',
                            ]
                        )
                    ],
                    output='screen',
                )
            )
        else:
            # Spawn the example as node
            nodes.append(
                Node(
                    package='franka_example_controllers',
                    executable=controller_name,
                    namespace=namespace,
                    output='screen',
                )
            )

    if any(
        str(config.get('use_rviz', 'false')).lower() == 'true'
        for config in configs.values()
    ):
        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=[
                    '--display-config',
                    PathJoinSubstitution(
                        [
                            FindPackageShare('franka_description'),
                            'rviz',
                            'visualize_franka.rviz',
                        ]
                    ),
                ],
                output='screen',
            )
        )
    return nodes


# The generate_launch_description function is the entry point (like 'main')
# It is called by the ROS 2 launch system when the launch file is executed.
# via: ros2 launch franka_bringup example.launch.py ARGS...
# This function must return a LaunchDescription object containing nodes to be launched.
# it calls the generate_robot_nodes function to get the list of nodes to be launched.


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_config_file',
                default_value='franka.config.yaml',
                description='Config file name (looked up in franka_bringup/config/) or full path',
            ),
            DeclareLaunchArgument(
                'controller_names',
                description='Comma-separated list of controller names to spawn (required)',
            ),
            OpaqueFunction(function=generate_robot_nodes),
        ]
    )
