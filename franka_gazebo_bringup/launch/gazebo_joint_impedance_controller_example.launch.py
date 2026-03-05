# Copyright (c) 2024 Franka Robotics GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_robot_description(context: LaunchContext, robot_type, load_gripper, franka_hand):
    robot_type_str = context.perform_substitution(robot_type)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_gazebo_bringup'),
        'urdf', 'franka_arm.gazebo.xacro'
    )

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'robot_type': robot_type_str,
            'hand': load_gripper_str,
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': franka_hand_str,
            'gazebo_effort': 'true',
        }
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )]


def generate_launch_description():
    load_gripper = LaunchConfiguration('load_gripper')
    franka_hand = LaunchConfiguration('franka_hand')
    robot_type = LaunchConfiguration('robot_type')

    gazebo_bringup_share = get_package_share_directory('franka_gazebo_bringup')
    gz_world = os.path.join(gazebo_bringup_share, 'worlds', 'empty_no_gravity.sdf')

    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(
        get_package_share_directory('franka_description'))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {gz_world}'}.items(),
    )

    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    rviz_file = os.path.join(get_package_share_directory('franka_description'),
                             'rviz', 'visualize_franka.rviz')
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['--display-config', rviz_file, '-f', 'world'],
    )

    load_joint_state_broadcaster = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager-timeout', '120'],
        output='screen',
    )

    load_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_impedance_example_controller',
                   '--controller-manager-timeout', '120'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('load_gripper', default_value='false'),
        DeclareLaunchArgument('franka_hand', default_value='franka_hand'),
        DeclareLaunchArgument('robot_type', default_value='fr3v2'),
        gazebo,
        OpaqueFunction(function=get_robot_description,
                       args=[robot_type, load_gripper, franka_hand]),
        rviz,
        spawn,
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=spawn, on_exit=[load_joint_state_broadcaster])),
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster, on_exit=[load_controller])),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'source_list': ['joint_states'], 'rate': 30}],
        ),
    ])
