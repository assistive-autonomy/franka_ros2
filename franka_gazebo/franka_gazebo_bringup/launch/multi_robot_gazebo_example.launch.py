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
import importlib.util

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    OpaqueFunction,
    ExecuteProcess,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# --------------------------------------------------------
# Load YAML helper
# --------------------------------------------------------
package_share = get_package_share_directory('franka_bringup')
utils_path = os.path.abspath(
    os.path.join(package_share, '..', '..', 'lib', 'franka_bringup', 'utils')
)
launch_utils_path = os.path.join(utils_path, 'launch_utils.py')
spec = importlib.util.spec_from_file_location('launch_utils', launch_utils_path)
launch_utils = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_utils)
load_yaml = launch_utils.load_yaml


# --------------------------------------------------------
# Per-robot setup
# --------------------------------------------------------
def get_robot_nodes(context: LaunchContext, robot_cfg):
    namespace = robot_cfg['namespace']
    robot_type = robot_cfg['robot_type']
    load_gripper = str(robot_cfg['load_gripper']).lower()
    arm_prefix = robot_cfg['arm_prefix']

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', robot_type, f'{robot_type}.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'robot_type': robot_type,
            'hand': load_gripper,
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': 'franka_hand',
            'robot_namespace': namespace,
            'gazebo_effort': 'true'
        }
    )

    robot_description = {
        'robot_description': robot_description_config.toxml()
    }

    # Define positions for left and right robots
    positions = [
        {'x': 0.0369, 'y': 0.05018, 'z': 1.0, 'roll': -0.89334809, 'pitch': -0.17456074, 'yaw': -0.46334506},  # left
        {'x': 0.0369, 'y': -0.05018, 'z': 1.0, 'roll': 0.89334809, 'pitch': -0.17456074, 'yaw': 0.46334506},  # right
        {'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}  # default
    ]
    controllers =[
        'joint_impedance_example_controller',
        'joint_impedance_example_controller',
        'mobile_cartesian_velocity_with_ik_example_controller'
    ]
    index = {'left': 0, 'right': 1, '': 2}
    pos = positions[index[arm_prefix]]
    controller = controllers[index[arm_prefix]]

    # ----------------------------------------------------
    # Robot State Publisher (namespaced)
    # ----------------------------------------------------
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

    # ----------------------------------------------------
    # Spawn robot into Gazebo
    # (Gazebo creates ros2_control + controller_manager)
    # ----------------------------------------------------
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

    # ----------------------------------------------------
    # Load controllers AFTER spawn
    # ----------------------------------------------------
    load_jsb = ExecuteProcess(
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


    # ----------------------------------------------------
    # Event ordering
    # ----------------------------------------------------
    load_jsb_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(period=1.0, actions=[load_jsb])
            ]
        )
    )

    load_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_jsb,
            on_exit=[
                TimerAction(period=0.5, actions=[load_controller])
            ]
        )
    )

    return [
        rsp_node,
        spawn_robot,
        load_jsb_event,
        load_controller_event,
    ]


# --------------------------------------------------------
# Main launch
# --------------------------------------------------------
def generate_launch_description():

    robot_config_file_arg = DeclareLaunchArgument(
        'robot_config_file',
        default_value=os.path.join(
            package_share, 'config', 'franka.config.yaml'
        ),
        description='Multi-robot config file'
    )

    robots_yaml = load_yaml(
        os.path.join(package_share, 'config', 'franka.config.yaml')
    )

    robots = [
        cfg for key, cfg in robots_yaml.items()
        if key.startswith('ROBOT') and cfg.get('robot_type')
    ]

    # ----------------------------------------------------
    # Gazebo (single, global)
    # ----------------------------------------------------
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(
        get_package_share_directory('franka_description')
    )

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
        gazebo,
    ]

    for _, robot_cfg in enumerate(robots):
        actions.append(
            OpaqueFunction(
                function=get_robot_nodes,
                args=[robot_cfg]
            )
        )

    return LaunchDescription(actions)


