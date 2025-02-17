# Copyright (c) 2023 Franka Robotics GmbH
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
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def robot_description_dependent_nodes_spawner(
        context: LaunchContext, robot_ips, arm_ids, arm_prefixes, use_fake_hardware):
    robot_ips_str = context.perform_substitution(robot_ips)
    arm_ids_str = context.perform_substitution(arm_ids)
    arm_prefixes_str = context.perform_substitution(arm_prefixes)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)


    gripper_config = os.path.join(
        get_package_share_directory('franka_gripper'), 'config', 'franka_gripper_node.yaml'
    )

    robot_ips_list = robot_ips_str.strip("[]").split(',')
    robot_ips_list = [ip.strip("'") for ip in robot_ips_list]
    arm_ids_list = arm_ids_str.strip("[]").split(',')
    arm_ids_list = [id.strip("'") for id in arm_ids_list]
    arm_prefixes_list = arm_prefixes_str.strip("[]").split(',')
    arm_prefixes_list = [prefix.strip("'") for prefix in arm_prefixes_list]

    default_joint_name_postfix = '_finger_joint'
    list_of_nodes = []

    for i in range(len(robot_ips_list)):

        joint_names = [
            arm_prefixes_list[i] + '_' + arm_ids_list[i] + default_joint_name_postfix + '1',
            arm_prefixes_list[i] + '_' + arm_ids_list[i] + default_joint_name_postfix + '2'
        ]

        list_of_nodes.append(Node(
            package='franka_gripper',
            executable='franka_gripper_node',
            name=[arm_prefixes_list[i], '_', arm_ids_list[i], '_gripper'],
            remappings=[(arm_prefixes_list[i] + '_' + arm_ids_list[i] + '_gripper/joint_states', 'franka_gripper/joint_states')],
            parameters=[{'robot_ip': robot_ips_list[i], 'joint_names': joint_names}, gripper_config],
            condition=UnlessCondition(use_fake_hardware_str),
        ))
        list_of_nodes.append(Node(
            package='franka_gripper',
            executable='fake_gripper_state_publisher.py',
            name=[arm_prefixes_list[i], '_', arm_ids_list[i], '_gripper'],
            remappings=[(arm_prefixes_list[i] + '_' + arm_ids_list[i] + '_gripper/joint_states', 'franka_gripper/joint_states')],
            parameters=[{'robot_ip': robot_ips_list[i], 'joint_names': joint_names}, gripper_config],
            condition=IfCondition(use_fake_hardware_str),
        ))

    return list_of_nodes

def generate_launch_description():
    robot_ips_parameter_name = 'robot_ips'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    arm_ids_parameter_name = 'arm_ids'
    arm_prefixes_parameter_name = 'arm_prefixes'
    robot_ips = LaunchConfiguration(robot_ips_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    arm_ids = LaunchConfiguration(arm_ids_parameter_name)
    arm_prefixes = LaunchConfiguration(arm_prefixes_parameter_name)

    robot_description_dependent_nodes_spawner_opaque_function = OpaqueFunction(
        function=robot_description_dependent_nodes_spawner,
        args=[robot_ips, arm_ids, arm_prefixes, use_fake_hardware])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                robot_ips_parameter_name, description='Hostname or IP address of the robot.'
            ),
            DeclareLaunchArgument(
                use_fake_hardware_parameter_name,
                default_value='false',
                description=(
                    'Publish fake gripper joint states without connecting to a real gripper'
                ),
            ),
            DeclareLaunchArgument(
                arm_ids_parameter_name,
                default_value='fr3',
                description=(
                    'Name of the arm in the URDF file. This is used to generate the joint '
                    'names of the gripper.'
                ),
            ),
            robot_description_dependent_nodes_spawner_opaque_function,
        ]
    )
