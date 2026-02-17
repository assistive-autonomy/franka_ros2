# Copyright (c) 2026 Franka Robotics GmbH
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration
from launch_ros.actions import Node

def get_robot_description(context: LaunchContext, load_gripper, franka_hand, with_sensors):
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)
    with_sensors_val = context.perform_substitution(with_sensors).lower()

    if with_sensors_val == 'true':
        selected_package = 'franka_mobile_sensors'
        franka_xacro_file = os.path.join(
            get_package_share_directory(selected_package),
            'robots',
            'mobile_fr3_duo_v0_2_with_sensors.urdf.xacro'
        )
    else:
        selected_package = 'franka_description'
        franka_xacro_file = os.path.join(
            get_package_share_directory(selected_package),
            'robots',
            'mobile_fr3_duo_v0_2',
            'mobile_fr3_duo_v0_2.urdf.xacro'
        )

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'robot_types': "['tmrv0_2', 'fr3v2', 'fr3v2']",
            'robot_prefixes': "['', 'left', 'right']",
            'hand': load_gripper_str,
            'ee_id': franka_hand_str,
            'ros2_control': 'true',
            'gazebo': 'true',
            'gazebo_effort': 'true'
        }
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
        ]
    )

    return [robot_state_publisher]

def set_gz_sim_resource_path(context, with_sensors):
    with_sensors_val = context.perform_substitution(with_sensors).lower()
    if with_sensors_val == 'true':
        sensors_share = os.path.dirname(get_package_share_directory('franka_mobile_sensors'))
        description_share = os.path.dirname(get_package_share_directory('franka_description'))
        oliv_module_descriptions_share = os.path.dirname(get_package_share_directory('olv_module_descriptions'))
        os.environ['GZ_SIM_RESOURCE_PATH'] = f"{sensors_share}:{description_share}:{oliv_module_descriptions_share}"
    else:
        description_share = os.path.dirname(get_package_share_directory('franka_description'))
        os.environ['GZ_SIM_RESOURCE_PATH'] = description_share
    return []

def generate_launch_description():
    load_gripper_name = 'load_gripper'
    franka_hand_name = 'franka_hand'
    namespace_name = 'namespace'
    with_sensors_name = 'with_sensors'

    load_gripper = LaunchConfiguration(load_gripper_name)
    franka_hand = LaunchConfiguration(franka_hand_name)
    namespace = LaunchConfiguration(namespace_name)
    with_sensors = LaunchConfiguration(with_sensors_name)

    load_gripper_launch_argument = DeclareLaunchArgument(
            load_gripper_name,
            default_value='true',
            description='true/false for activating the gripper')
    franka_hand_launch_argument = DeclareLaunchArgument(
            franka_hand_name,
            default_value='franka_hand',
            description='Default value: franka_hand')
    namespace_launch_argument = DeclareLaunchArgument(
        namespace_name,
        default_value='',
        description='Namespace for the robot. If not set, the robot will be launched in the root namespace.')
    with_sensors_launch_argument = DeclareLaunchArgument(
        with_sensors_name,
        default_value='false',
        description='If true, use sensor-enhanced description package (franka_mobile_sensors)')

    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[load_gripper, franka_hand, with_sensors])

    set_gz_sim_resource_path_action = OpaqueFunction(function=set_gz_sim_resource_path, args=[with_sensors])
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': 'empty.sdf -r', }.items(),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_franka.rviz')
    rviz = Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             namespace=namespace,
             arguments=['--display-config', rviz_file, '-f', 'world'],
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_state_broadcaster'],
        output='screen'
    )

    mobile_duo_ik_example_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'mobile_duo_with_ik_example_controller'],
        output='screen'
    )

    return LaunchDescription([
        load_gripper_launch_argument,
        franka_hand_launch_argument,
        namespace_launch_argument,
        with_sensors_launch_argument,
        set_gz_sim_resource_path_action,
        gazebo_empty_world,
        robot_state_publisher,
        rviz,
        spawn,
        bridge,
        RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn,
                    on_exit=[load_joint_state_broadcaster],
                )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[mobile_duo_ik_example_controller],
            )
        ),
    ])