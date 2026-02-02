import unittest

import launch.launch_description_sources
import launch.substitutions
import launch_ros.substitutions
import launch_testing
import rclpy

initialize_step = {
    'controller_name': 'move_to_start_example_controller',
    'config_file_name': 'test_0.config.yaml',
}

parameters = [
    {
        'controller_name': 'cartesian_elbow_example_controller',
        'config_file_name': 'test_0.config.yaml',
    },
    {
        'controller_name': 'cartesian_orientation_example_controller',
        'config_file_name': 'test_0.config.yaml',
    },
    {
        'controller_name': 'cartesian_orientation_example_controller',
        'config_file_name': 'test_1.config.yaml',
    },
    {
        'controller_name': 'cartesian_pose_example_controller',
        'config_file_name': 'test_0.config.yaml',
    },
    {
        'controller_name': 'cartesian_pose_example_controller',
        'config_file_name': 'test_1.config.yaml',
    },
    {
        'controller_name': 'cartesian_velocity_example_controller',
        'config_file_name': 'test_0.config.yaml',
    },
    {
        'controller_name': 'elbow_example_controller',
        'config_file_name': 'test_0.config.yaml',
    },
    # {
    #     'controller_name': 'fr3_duo_joint_impedance_example_controller',
    #     'config_file_name': 'test_0.config.yaml',
    # },
    # {
    #     'controller_name': 'fr3_duo_self_collision_example_controller',
    #     'config_file_name': 'test_0.config.yaml',
    # },
    {
        'controller_name': 'gravity_compensation_example_controller',
        'config_file_name': 'test_0.config.yaml',
    },
    {
        'controller_name': 'gravity_compensation_example_controller',
        'config_file_name': 'test_1.config.yaml',
    },
    {
        'controller_name': 'gripper_example_controller',
        'config_file_name': 'test_0.config.yaml',
    },
    {
        'controller_name': 'joint_impedance_example_controller',
        'config_file_name': 'test_0.config.yaml',
    },
    {
        'controller_name': 'joint_impedance_with_ik_example_controller',
        'config_file_name': 'test_0.config.yaml',
    },
    {
        'controller_name': 'joint_impedance_with_ik_example_controller',
        'config_file_name': 'test_1.config.yaml',
    },
    {
        'controller_name': 'joint_position_example_controller',
        'config_file_name': 'test_0.config.yaml',
    },
    {
        'controller_name': 'joint_velocity_example_controller',
        'config_file_name': 'test_0.config.yaml',
    },
    {
        'controller_name': 'model_example_controller',
        'config_file_name': 'test_0.config.yaml',
    },
    {
        'controller_name': 'model_example_controller',
        'config_file_name': 'test_1.config.yaml',
    },
]

test_parameters = [
    element for name in parameters for element in [initialize_step, name]
][:-1]


@launch_testing.parametrize('test_parameter', test_parameters)
def generate_test_description(test_parameter):
    """Generate the test launch descriptions."""
    controller_name = test_parameter['controller_name']
    config_file_name = test_parameter['config_file_name']

    config_file = launch.substitutions.PathJoinSubstitution(
        [
            launch_ros.substitutions.FindPackageShare(
                'integration_launch_testing'
            ),
            'config',
            config_file_name,
        ]
    )

    example_launch_description = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            launch.substitutions.PathJoinSubstitution(
                [
                    launch_ros.substitutions.FindPackageShare(
                        'franka_bringup'
                    ),
                    'launch',
                    'example.launch.py',
                ]
            )
        ),
        launch_arguments={
            'robot_config_file': config_file,
            'controller_names': controller_name,
        }.items(),
    )

    test_description = (
        launch.LaunchDescription(
            [
                example_launch_description,
                launch.actions.TimerAction(
                    period=3.0, actions=[launch_testing.actions.ReadyToTest()]
                ),
            ],
        ),
        {'example_launch_description': example_launch_description},
    )
    print('Generated test description')
    return test_description


class TestExampleController(unittest.TestCase):
    """Class for testing an Example Controller."""

    @classmethod
    def setUpClass(cls):
        """Initialize the ROS context."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown the ROS context."""
        rclpy.shutdown()

    def test_has_no_error(self, proc_output):
        """Check if any error messages have been logged."""
        has_no_error = not proc_output.waitFor(
            'ERROR', timeout=5, stream='stderr'
        )

        assert has_no_error, 'Found [ERROR] log messages in launch output'
