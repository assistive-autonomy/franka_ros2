#  Copyright (c) 2024 Franka Robotics GmbH
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip_parameter_name = "robot_ip"
    mobile_robot_id_parameter_name = "mobile_robot_id"
    use_fake_hardware_parameter_name = "use_fake_hardware"
    fake_sensor_commands_parameter_name = "fake_sensor_commands"
    use_rviz_parameter_name = "use_rviz"

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    mobile_robot_id = LaunchConfiguration(mobile_robot_id_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                robot_ip_parameter_name,
                default_value="172.16.1.20",
                description="Hostname or IP address of the robot."
            ),
            DeclareLaunchArgument(
                mobile_robot_id_parameter_name,
                default_value="tmr",
                description="ID of the type of arm used. Supported values: tmr",
            ),
            DeclareLaunchArgument(
                use_rviz_parameter_name,
                default_value="false",
                description="Visualize the robot in Rviz",
            ),
            DeclareLaunchArgument(
                use_fake_hardware_parameter_name,
                default_value="false",
                description="Use fake hardware",
            ),
            DeclareLaunchArgument(
                fake_sensor_commands_parameter_name,
                default_value="false",
                description="Fake sensor commands. Only valid when '{}' is true".format(
                    use_fake_hardware_parameter_name
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [FindPackageShare("franka_mobile_bringup"), "launch", "mobile_robot.launch.py"]
                        )
                    ]
                ),
                launch_arguments={
                    robot_ip_parameter_name: robot_ip,
                    mobile_robot_id_parameter_name: mobile_robot_id,
                    use_fake_hardware_parameter_name: use_fake_hardware,
                    fake_sensor_commands_parameter_name: fake_sensor_commands,
                    use_rviz_parameter_name: use_rviz,
                }.items(),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_drive_controller"],
                output="screen",
            ),
        ]
    )
