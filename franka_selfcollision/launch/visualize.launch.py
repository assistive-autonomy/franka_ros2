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
# hand: Load the gripper (hand) for the robots (default: 'true')
#
# This launch file provides a standalone visualization environment for the
# FR3 Duo setup. It spawns:
# 1. The Robot State Publisher (URDF)
# 2. The Joint State Publisher GUI (Sliders to move the robot)
# 3. The Self-Collision Monitoring Node (To verify collisions)
# 4. RViz
#
# Usage:
#    ros2 launch <your_package> visualize_fr3_duo.launch.py hand:=true
############################################################################

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_robot_nodes(context):
    load_gripper = LaunchConfiguration('hand').perform(context)

    # --- 1. Process URDF (Robot Description) ---
    urdf_path = PathJoinSubstitution(
        [
            FindPackageShare('franka_description'),
            'robots',
            'fr3_duo',
            'fr3_duo.urdf.xacro',
        ]
    ).perform(context)

    # We hardcode IP/Types here for visualization purposes as we are not
    # connecting to real hardware.
    robot_description_xml = xacro.process_file(
        urdf_path,
        mappings={
            'ros2_control': 'false',  # No hardware control needed for viz
            'robot_types': "['fr3', 'fr3']",
            'robot_ips': "['0.0.0.0', '0.0.0.0']",
            'hand': load_gripper,
            'arm_prefixes': "['left', 'right']",
            'use_fake_hardware': 'true',
            'is_async': 'false',
        },
    ).toprettyxml(indent='  ')

    # --- 2. Process SRDF (Semantic Description for Collision) ---
    srdf_path = PathJoinSubstitution(
        [
            FindPackageShare('franka_description'),
            'robots',
            'fr3_duo',
            'fr3_duo.srdf.xacro',
        ]
    ).perform(context)

    robot_description_semantic_xml = xacro.process_file(
        srdf_path,
        mappings={
            'robot_types': "['fr3', 'fr3']",
            'arm_prefixes': "['left', 'right']",
            'hand': load_gripper,
        }
    ).toprettyxml(indent='  ')

    # --- 3. Define Nodes ---
    nodes = [
        # Publishes TF based on joint states
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_xml}],
        ),
        # GUI Sliders to manually move joints
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            # We map the gripper joints to the slider outputs if needed
            parameters=[{'source_list': ['franka/joint_states']}],
        ),
        # The Node under test: Checks for self-collisions
        Node(
            package='franka_selfcollision',
            executable='self_collision_node',
            name='fr3_duo_self_collision_node',
            output='screen',
            parameters=[
                {
                    'robot_description_semantic': robot_description_semantic_xml,
                    'print_collisions': True,  # Helpful for debugging
                    'security_margin': 0.001,
                }
            ],
        ),
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d',
                PathJoinSubstitution(
                    [
                        FindPackageShare('franka_selfcollision'),
                        'config',
                        'visualize_fr3.rviz',
                    ]
                ),
            ],
            output='screen',
        ),
    ]

    return nodes


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'hand',
            default_value='false',
            description='Load the gripper (hand) for the robots.',
        ),
    ]

    return LaunchDescription(
        launch_args + [OpaqueFunction(function=generate_robot_nodes)]
    )