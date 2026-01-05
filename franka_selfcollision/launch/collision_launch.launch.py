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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    robot_types_str = "['fr3', 'fr3']"
    arm_prefixes_str = "['left', 'right']"
    robot_ips_str = "['0.0.0.0', '0.0.0.0']"
    load_gripper_str = 'true'
    base_robot_type = 'fr3'

    franka_desc_share = get_package_share_directory('franka_description')

    urdf_path = os.path.join(
            franka_desc_share,
            'robots',
            f'{base_robot_type}_duo',
            f'{base_robot_type}_duo.urdf.xacro'

    )

    robot_description = xacro.process_file(
        urdf_path,
        mappings={
            'ros2_control': 'true',
            'robot_types': robot_types_str,
            'robot_ips': robot_ips_str,
            'hand': load_gripper_str,
            'use_fake_hardware': 'true',
            'fake_sensor_commands': 'false',
            'is_async': 'true',
        },
    ).toprettyxml(indent='  ')

    srdf_path = os.path.join(
            franka_desc_share,
            'robots',
            f'{base_robot_type}_duo',
            f'{base_robot_type}_duo.srdf.xacro',
    )

    robot_description_semantic = xacro.process_file(
        srdf_path,
        mappings={
            'robot_types': robot_types_str,
            'arm_prefixes': arm_prefixes_str,
            'hand': load_gripper_str,
        }
    ).toprettyxml(indent='  ')

    rviz_config_path = '/ros2_ws/src/franka_selfcollision/config/visualize_franka.rviz'
    monitor_script_path = '/ros2_ws/src/franka_selfcollision/scripts/monitor_collision.py'

    return LaunchDescription([

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{'robot_description': robot_description}],
            remappings=[('joint_states', '/joint_states')]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path]
        ),

        Node(
            package='franka_selfcollision',
            executable='self_collision_service',
            name='self_collision_service',
            output='screen',
            parameters=[{
                         'robot_description': robot_description,
                         'robot_description_semantic': robot_description_semantic,
                         'security_margin': 0.045,
                         'print_collisions': False
            }]
        ),

        ExecuteProcess(
            cmd=['python3', monitor_script_path],
            output='screen',
            emulate_tty=True,
            name='monitor_collision_script'
        )

    ])
