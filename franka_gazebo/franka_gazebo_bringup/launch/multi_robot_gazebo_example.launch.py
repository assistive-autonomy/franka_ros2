# Copyright (c) 2026 Franka Robotics GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
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
    RegisterEventHandler,
    OpaqueFunction,
    ExecuteProcess,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import franka_bringup.launch_utils as launch_utils


package_share = get_package_share_directory('franka_bringup')
load_yaml = launch_utils.load_yaml

def set_global_gz_sim_resource_path(context, with_sensors):
    with_sensors_val = context.perform_substitution(with_sensors).lower()
    description_share = os.path.dirname(get_package_share_directory('franka_description'))
    if with_sensors_val == 'true':
        sensors_share = os.path.dirname(get_package_share_directory('franka_mobile_sensors'))
        oliv_module_descriptions_share = os.path.dirname(get_package_share_directory('olv_module_descriptions'))
        gz_sim_resource_path = f"{sensors_share}:{description_share}:{oliv_module_descriptions_share}"
    else:
        gz_sim_resource_path = description_share
    return [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_sim_resource_path)]

def get_robot_nodes(context: LaunchContext, robot_cfg, multi_robot_configs, with_sensors):
    namespace = robot_cfg['namespace']
    robot_type = robot_cfg['robot_type']
    load_gripper = str(robot_cfg['load_gripper']).lower()
    arm_prefix = robot_cfg['arm_prefix']
    with_sensors_val = context.perform_substitution(with_sensors).lower()

    if with_sensors_val == 'true' and robot_type == 'tmrv0_2':
        selected_package = 'franka_mobile_sensors'
        franka_xacro_file = os.path.join(
            get_package_share_directory(selected_package),
            'robots',
            'tmrv0_2_with_sensors.urdf.xacro'
        )
    else:
        franka_xacro_file = os.path.join(
            get_package_share_directory('franka_description'),
            'robots', robot_type, f'{robot_type}.urdf.xacro'
        )

    config = multi_robot_configs.get(arm_prefix, multi_robot_configs.get('', {}))
    pos = config.get('position', {})
    controller = config.get('controller', '')

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'robot_type': robot_type,
            'hand': load_gripper,
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': 'franka_hand',
            'robot_namespace': namespace,
            'gazebo_effort': 'true',
        }
    )

    robot_description = {
        'robot_description': robot_description_config.toxml()
    }

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[robot_description],
        remappings=[
            ('/joint_states', f'/{namespace}/joint_states')
        ],
        output='screen'
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', f'/{namespace}/robot_description',
            '-name', namespace,
            '-x', str(pos['x']),
            '-y', str(pos['y']),
            '-z', str(pos['z']),
            '-R', str(pos['roll']),
            '-P', str(pos['pitch']),
            '-Y', str(pos['yaw']),
        ],
        output='screen'
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--controller-manager', f'/{namespace}/controller_manager',
            '--set-state', 'active',
            'joint_state_broadcaster'
        ],
        output='screen'
    )

    load_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--controller-manager', f'/{namespace}/controller_manager',
            '--set-state', 'active',
            controller
        ],
        output='screen'
    )

    load_joint_state_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(period=1.0, actions=[load_joint_state_broadcaster])
            ]
        )
    )

    load_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[
                TimerAction(period=0.5, actions=[load_controller])
            ]
        )
    )

    return [
        rsp_node,
        spawn_robot,
        load_joint_state_broadcaster_event,
        load_controller_event,
    ]

def generate_launch_description():
    with_sensors_name = 'with_sensors'
    with_sensors = LaunchConfiguration(with_sensors_name)

    robot_config_file_arg = DeclareLaunchArgument(
        'robot_config_file',
        default_value=os.path.join(
            package_share, 'config', 'franka.config.yaml'
        ),
        description='Multi-robot config file'
    )

    gazebo_config_file_arg = DeclareLaunchArgument(
        'gazebo_config_file',
        default_value=os.path.join(
            get_package_share_directory('franka_gazebo_bringup'), 'config', 'gazebo_multi_robot_configs.yaml'
        ),
        description='Gazebo multi-robot config file'
    )

    with_sensors_launch_argument = DeclareLaunchArgument(
        with_sensors_name,
        default_value='false',
        description='If true, use sensor-enhanced description package (franka_mobile_sensors)'
    )
 

    robots_yaml = load_yaml(
        os.path.join(package_share, 'config', 'franka.config.yaml')
    )

    gazebo_config_yaml = load_yaml(
        os.path.join(get_package_share_directory('franka_gazebo_bringup'), 'config', 'gazebo_multi_robot_configs.yaml')
    )

    arm_configs = gazebo_config_yaml

    robots = [
        cfg for key, cfg in robots_yaml.items()
        if key.startswith('ROBOT') and cfg.get('robot_type')
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items()
    )

    actions = [
        robot_config_file_arg,
        gazebo_config_file_arg,
        with_sensors_launch_argument,
        OpaqueFunction(function=set_global_gz_sim_resource_path, args=[with_sensors]),
        gazebo,
    ]

    for _, robot_cfg in enumerate(robots):
        actions.append(
            OpaqueFunction(
                function=get_robot_nodes,
                args=[robot_cfg, arm_configs, with_sensors]
            )
        )

    return LaunchDescription(actions)


