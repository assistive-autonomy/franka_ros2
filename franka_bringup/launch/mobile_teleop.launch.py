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

import sys
import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Add the path to the `utils` folder
package_share = get_package_share_directory('franka_bringup')
utils_path = os.path.join(package_share, '..', '..', 'lib', 'franka_bringup', 'utils')
sys.path.append(os.path.abspath(utils_path))

from launch_utils import load_yaml

def generate_robot_nodes(context):
    additional_nodes = []
    # Get the arguments from the launch configuration
    controller_names = LaunchConfiguration('controller_names').perform(context)
    robot_config_file = LaunchConfiguration('robot_config_file').perform(context)
    config_filepath = LaunchConfiguration('config_filepath').perform(context)

    # Include the existing example.launch.py file
    additional_nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('franka_bringup'), 'launch', 'example.launch.py'
                ])
            ),
            launch_arguments={
                'robot_config_file': robot_config_file,
                'controller_names': controller_names,
            }.items(),
        )
    )

    # Load the robot configuration file
    configs = load_yaml(robot_config_file)

    for _,config in configs.items():
        namespace = config['namespace']
        # Define the additional nodes
        additional_nodes.append(
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                namespace=namespace,
                parameters=[
                    {
                        "dev": "/dev/input/js0",
                        "deadzone": 0.3,
                        "autorepeat_rate": 20.0,
                    }
                ],
            ),
        )
        additional_nodes.append(
                Node(
                    package="teleop_twist_joy",
                    executable="teleop_node",
                    name="teleop_twist_joy_node",
                    namespace=namespace,
                    parameters=[config_filepath],
                    remappings=[("/" + namespace + "/cmd_vel", "/" + namespace + "/mobile_cartesian_velocity_controller/cmd_vel")],
                ),
            )
    return additional_nodes


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments and add additional ones if needed
        DeclareLaunchArgument(
            'controller_names',
            description='Name of the controller to be used'
        ),
        DeclareLaunchArgument(
            'robot_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('franka_bringup'), 'config', 'franka.config.yaml'
            ]),
            description='Path to the robot configuration file to load',
        ),
        DeclareLaunchArgument(
            "config_filepath",
            default_value=[
                launch.substitutions.TextSubstitution(
                    text=os.path.join(
                        get_package_share_directory("franka_bringup"), "config", ""
                    )
                ),
                launch.substitutions.TextSubstitution(text="xbox.config.yaml"),
            ],
        ),
        # Generate robot nodes
        OpaqueFunction(function=generate_robot_nodes),
    ])