#!/usr/bin/env python3
#  Copyright (c) 2026 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the 'License');
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an 'AS IS' BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

"""
Generic integration test for Franka example controllers.

Usage:
    # Via launch_test
    launch_test generic_controller_test.py controller_name:=joint_impedance_example_controller

    # With RViz visualization
    launch_test generic_controller_test.py controller_name:=joint_impedance_example_controller \
        use_rviz:=true

    # Via ros2 run
    ros2 run integration_launch_testing generic_controller_test controller_name:=<name>

Supported controllers:
    - move_to_start_example_controller
    - joint_position_example_controller
    - joint_velocity_example_controller
    - joint_impedance_example_controller
    - cartesian_pose_example_controller
    - cartesian_velocity_example_controller
    - cartesian_orientation_example_controller
    - gravity_compensation_example_controller
"""

import os
import sys
import unittest

# Add the test directory to Python path for importing controller_test_utils
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from ament_index_python.packages import get_package_share_directory
    pkg_test = get_package_share_directory('integration_launch_testing')
    sys.path.insert(0, os.path.join(pkg_test, 'test'))
except Exception:
    pass

from launch import LaunchDescription  # noqa: E402
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription  # noqa: E402
from launch.conditions import IfCondition  # noqa: E402
from launch.launch_description_sources import PythonLaunchDescriptionSource  # noqa: E402
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution  # noqa: E402
from launch_ros.actions import Node  # noqa: E402
from launch_ros.substitutions import FindPackageShare  # noqa: E402
from launch_testing.actions import ReadyToTest  # noqa: E402

import rclpy  # noqa: E402, I100


# Default controller name - can be overridden via launch argument
DEFAULT_CONTROLLER_NAME = 'joint_impedance_example_controller'

# Get controller name from environment or use default
# This allows the test to be configured before generate_test_description is called
CONTROLLER_NAME = os.environ.get('CONTROLLER_NAME', DEFAULT_CONTROLLER_NAME)

# Check if we need to run move_to_start first (from environment variable)
# If true, we spawn the target controller in inactive mode so we can run move_to_start first
NEEDS_MOVE_TO_START = os.environ.get('NEEDS_MOVE_TO_START', 'false').lower() == 'true'


def generate_test_description():
    """
    Generate the test launch description.

    The controller name can be specified via:
    - Environment variable CONTROLLER_NAME
    - Launch argument controller_name:=<name>

    RViz can be enabled with use_rviz:=true
    """
    robot_ip_parameter_name = 'robot_ip'
    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    controller_name = LaunchConfiguration('controller_name')
    use_rviz = LaunchConfiguration('use_rviz')

    # Launch the franka robot with real hardware
    franka_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare('franka_bringup'),
                        'launch',
                        'franka.launch.py',
                    ]
                )
            ]
        ),
        launch_arguments={
            'robot_type': 'fr3',
            robot_ip_parameter_name: robot_ip,
        }.items(),
    )

    # Spawn the controller
    # If NEEDS_MOVE_TO_START is true, spawn in inactive mode so we can run move_to_start first
    # The test code will then activate this controller after move_to_start completes
    spawner_args = [controller_name, '--controller-manager-timeout', '30']
    if NEEDS_MOVE_TO_START and CONTROLLER_NAME != 'move_to_start_example_controller':
        spawner_args.append('--inactive')

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=spawner_args,
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

    # RViz for visualization (optional)
    rviz_node = Node(
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
        condition=IfCondition(use_rviz),
    )

    return (
        LaunchDescription(
            [
                DeclareLaunchArgument(
                    robot_ip_parameter_name,
                    default_value='172.16.0.2',
                    description='Hostname or IP address of the robot.',
                ),
                DeclareLaunchArgument(
                    'controller_name',
                    default_value=CONTROLLER_NAME,
                    description='Name of the controller to test.',
                ),
                DeclareLaunchArgument(
                    'use_rviz',
                    default_value='false',
                    description='Launch RViz for visualization.',
                ),
                franka_launch,
                controller_spawner,
                rviz_node,
                ReadyToTest(),
            ]
        ),
        {
            'franka_launch': franka_launch,
            'controller_spawner': controller_spawner,
        },
    )


class TestGenericController(unittest.TestCase):
    """Generic test class for any controller."""

    @classmethod
    def setUpClass(cls):
        """Initialize the ROS context for the test node."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown the ROS context."""
        rclpy.shutdown()

    def setUp(self):
        """Create a ROS node for tests."""
        self.link_node = rclpy.create_node('generic_controller_test_link')

    def tearDown(self):
        """Destroy the test node."""
        self.link_node.destroy_node()

    def test_controller(self):
        """
        Test that the controller runs successfully.

        The controller runs for a fixed duration (10 seconds). If it runs
        without errors and joint states are being published, the test passes.
        The controller is then stopped externally.

        If NEEDS_MOVE_TO_START environment variable is set to 'true', the
        move_to_start_example_controller will be run first to ensure the robot
        starts from a known position.
        """
        # Get the controller name from environment
        controller_name = os.environ.get('CONTROLLER_NAME', DEFAULT_CONTROLLER_NAME)

        # Check if we need to run move_to_start first
        # This is determined by the NEEDS_MOVE_TO_START environment variable
        needs_move_to_start = os.environ.get('NEEDS_MOVE_TO_START', 'false').lower() == 'true'

        # Import the helper functions
        from controller_test_utils import (  # noqa: E402
            run_controller_smoke_test,
            run_move_to_start_and_switch_to_target_controller,
            MOVE_TO_START_CONTROLLER,
        )

        # Run move_to_start first if needed (and we're not already testing move_to_start)
        if needs_move_to_start and controller_name != MOVE_TO_START_CONTROLLER:
            self.link_node.get_logger().info(
                f'Running {MOVE_TO_START_CONTROLLER} to move robot to start position '
                f'before testing {controller_name}...'
            )
            move_to_start_success = run_move_to_start_and_switch_to_target_controller(
                self.link_node,
                target_controller=controller_name,
                wait_duration_sec=30.0,
            )
            self.assertTrue(
                move_to_start_success,
                f'Failed to run {MOVE_TO_START_CONTROLLER} before testing {controller_name}',
            )
            self.link_node.get_logger().info(
                f'{MOVE_TO_START_CONTROLLER} completed. {controller_name} is now active.'
            )

        # Test duration - let controller run for this long
        # Use longer duration for move_to_start as it may take longer to reach target
        if 'move_to_start' in controller_name:
            test_duration_sec = 15.0
        else:
            test_duration_sec = 10.0

        self.link_node.get_logger().info(
            f'Testing controller: {controller_name} (will run for {test_duration_sec}s)'
        )

        run_controller_smoke_test(
            self,
            controller_name,
            test_duration_sec=test_duration_sec,
        )


if __name__ == '__main__':
    """Allow running this test directly via ros2 run or python."""
    import argparse
    import subprocess

    parser = argparse.ArgumentParser(description='Run generic controller integration test')
    parser.add_argument(
        'launch_args',
        nargs='*',
        help='Launch args as key:=value (e.g. controller_name:=joint_impedance)',
    )

    args = parser.parse_args()

    # Parse controller_name from launch args
    controller_name = DEFAULT_CONTROLLER_NAME
    for arg in args.launch_args:
        if arg.startswith('controller_name:='):
            controller_name = arg.split(':=')[1]
            break

    # Set environment variable for the test
    os.environ['CONTROLLER_NAME'] = controller_name

    # Find the test file in the share directory
    try:
        from ament_index_python.packages import get_package_share_directory
        test_file = os.path.join(
            get_package_share_directory('integration_launch_testing'),
            'test',
            'generic_controller_test.py'
        )
    except Exception:
        test_file = os.path.abspath(__file__)

    # Build command with all arguments
    cmd = ['launch_test', test_file] + args.launch_args
    sys.exit(subprocess.call(cmd))
