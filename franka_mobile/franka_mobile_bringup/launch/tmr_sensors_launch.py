from ament_index_python.packages import get_package_share_directory
import os
import copy
from launch import LaunchDescription, LaunchContext
from launch.actions import OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
sys.path.append(os.path.join(
    get_package_share_directory('realsense2_camera'), 'launch'))
import rs_launch

# These parameters are declared to override the default values from the realsense2_camera package
camera_params = [
    {'name': 'camera_name1', 'default': 'camera_front_right', 'description': 'camera1 unique name'},
    {'name': 'camera_name2', 'default': 'camera_front_left', 'description': 'camera2 unique name'},
    {'name': 'camera_name3', 'default': 'camera_rear_right', 'description': 'camera3 unique name'},
    {'name': 'camera_name4', 'default': 'camera_rear_left', 'description': 'camera4 unique name'},
    {'name': 'camera_namespace1', 'default': 'camera_front_right', 'description':
     'camera1 namespace'},
    {'name': 'camera_namespace2', 'default': 'camera_front_left',
     'description': 'camera2 namespace'},
    {'name': 'camera_namespace3', 'default': 'camera_rear_right',
     'description': 'camera3 namespace'},
    {'name': 'camera_namespace4', 'default': 'camera_rear_left',
     'description': 'camera4 namespace'},
    # The default values of the usb_port_id parameters are the ports where the cameras are connected
    # to the host pc when it was delivered. If the cameras are connected to different ports, the
    # default values can be changed. If the false port is given, a warning will be raised
    # during execution and the ports where the cameras are connected will be shown.
    {'name': 'usb_port_id1', 'default': "'2-3.3.4'",
     'description': 'Usb port id of camera1 (by default camera_front_right)'},
    {'name': 'usb_port_id2', 'default': "'2-3.3.3'",
     'description': 'Usb port id of camera2 (by default camera_front_left)'},
    {'name': 'usb_port_id3', 'default': "'2-3.3.2'",
     'description': 'Usb port id of camera3 (by default camera_rear_right)'},
    {'name': 'usb_port_id4', 'default': "'2-3.3.1'",
     'description': 'Usb port id of camera4 (by default camera_rear_left)'}
]

# The indexes of this array are used for the parameter substitution in the launch of the lidar nodes
lidar_params = [
    {'name': 'lidar_namespace1', 'default': 'lidar_front',
     'description': 'lidar1 unique name that will be used as frame_id'},
    {'name': 'lidar_namespace2', 'default': 'lidar_rear',
     'description': 'lidar2 unique name that will be used as frame_id'},
    {'name': 'lidar_ip1', 'default': '192.168.1.11', 'description': 'unique ip of lidar 1'},
    {'name': 'lidar_ip2', 'default': '192.168.1.12', 'description': 'unique ip of the lidar 2'},
    {'name': 'host_ip', 'default': '192.168.1.9', 'description': 'unique ip of host pc'}
]


def duplicate_params(general_params, posix):
    local_params = copy.deepcopy(general_params)
    for param in local_params:
        param['original_name'] = param['name']
        param['name'] += posix
    return local_params


def set_configurable_parameters(parameters):
    return dict([(param['original_name'],
                  LaunchConfiguration(param['name'])) for param in parameters])


def generate_launch_description():
    launch_localization_node_arg = DeclareLaunchArgument(
        'launch_localization_node',
        default_value='False',
        description='Experimental feature: Bool to start robot_localization_node. Fuses IMU and ' +
            'wheel encoders to publish the filtered odometry data and updates the robot frame. ' +
            'By default, the wheel odom is based on the diff_drive_controller so this controller ' +
            'needs to be started. If another odom topic with wheel odometry data is available, ' +
            'it can be selected in the file config/ekf_node.yaml. Options: True or False')

    # Create a LaunchConfiguration from the argument
    launch_localization_node = LaunchConfiguration('launch_localization_node')

    # Determine the rviz_filename based on the launch_localization_node argument
    rviz_filename = PythonExpression([
        "'tmr_localization.rviz' if ", launch_localization_node, " else 'tmr_sensors.rviz'"
    ])

    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('franka_description'), 'launch'),
            '/visualize_tmr.launch.py']),
        launch_arguments={'rviz_filename': rviz_filename}.items()
    )

    # This node fuses IMU and joint encoders data to smooth the odometry, which is published in the
    # odometry/filtered topic. The transform odom -> base link is also published on the /tf topic.
    robot_localization_node = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('franka_mobile_bringup'),
            'config/ekf_node.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(launch_localization_node)
    )

    # Duplicate the parameters for each camera to be declared as arguments to the launch file by
    # adding a suffix. This allows to personalize all the parameters of the cameras.
    # The duplicated parameters can be checked in the rs_launch from the realsense2_camera package
    config_params_camera1 = duplicate_params(rs_launch.configurable_parameters, '1')
    config_params_camera2 = duplicate_params(rs_launch.configurable_parameters, '2')
    config_params_camera3 = duplicate_params(rs_launch.configurable_parameters, '3')
    config_params_camera4 = duplicate_params(rs_launch.configurable_parameters, '4')
    return LaunchDescription(
        # Declare configurable parameters from launch file before the duplicated parameters to get
        # the correct default values (declared in this launch files)
        rs_launch.declare_configurable_parameters(camera_params) +
        rs_launch.declare_configurable_parameters(lidar_params) +
        rs_launch.declare_configurable_parameters(config_params_camera1) +
        rs_launch.declare_configurable_parameters(config_params_camera2) +
        rs_launch.declare_configurable_parameters(config_params_camera3) +
        rs_launch.declare_configurable_parameters(config_params_camera4) +
        [
            OpaqueFunction(function=rs_launch.launch_setup,
                           kwargs={'params': set_configurable_parameters(config_params_camera1),
                                   'param_name_suffix': '1'}),
            OpaqueFunction(function=rs_launch.launch_setup,
                           kwargs={'params': set_configurable_parameters(config_params_camera2),
                                   'param_name_suffix': '2'}),
            OpaqueFunction(function=rs_launch.launch_setup,
                           kwargs={'params': set_configurable_parameters(config_params_camera3),
                                   'param_name_suffix': '3'}),
            OpaqueFunction(function=rs_launch.launch_setup,
                           kwargs={'params': set_configurable_parameters(config_params_camera4),
                                   'param_name_suffix': '4'}),
            Node(
                package="sick_safetyscanners2",
                executable="sick_safetyscanners2_node",
                name="sick_safetyscanners2_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "frame_id": LaunchConfiguration(lidar_params[0]['name']),
                        "sensor_ip": LaunchConfiguration(lidar_params[2]['name']),
                        "host_ip": LaunchConfiguration(lidar_params[4]['name']),
                        "interface_ip": "0.0.0.0",
                        "host_udp_port": 0,
                        "channel": 0,
                        "channel_enabled": True,
                        "skip": 0,
                        # Angle parameters are disregarded due to use_persistent_config being true
                        # The angles from the sensor (configured in Safety Designer) are loaded
                        "angle_start": 0.0,
                        "angle_end": 0.0,
                        "time_offset": 0.0,
                        "general_system_state": True,
                        "derived_settings": True,
                        "measurement_data": True,
                        "intrusion_data": True,
                        "application_io_data": True,
                        "use_persistent_config": True,
                        "min_intensities": 0.0
                    }
                ],
                remappings=[
                    ('scan', PythonExpression(["'", LaunchConfiguration(
                        lidar_params[0]['name']), "' + '/scan'"])),
                    ('extended_scan', PythonExpression(["'", LaunchConfiguration(
                        lidar_params[0]['name']), "' + '/extended_scan'"])),
                    ('output_paths', PythonExpression(["'", LaunchConfiguration(
                        lidar_params[0]['name']), "' + '/output_paths'"])),
                    ('raw_data', PythonExpression(
                        ["'", LaunchConfiguration(lidar_params[0]['name']), "' + '/raw_data'"]))
                ]
            ),
            Node(
                package="sick_safetyscanners2",
                executable="sick_safetyscanners2_node",
                name="sick_safetyscanners2_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "frame_id": LaunchConfiguration(lidar_params[1]['name']),
                        "sensor_ip": LaunchConfiguration(lidar_params[3]['name']),
                        "host_ip": LaunchConfiguration(lidar_params[4]['name']),
                        "interface_ip": "0.0.0.0",
                        "host_udp_port": 0,
                        "channel": 0,
                        "channel_enabled": True,
                        "skip": 0,
                        # Angle parameters are disregarded due to use_persistent_config being true
                        # The angles from the sensor (configured in Safety Designer) are loaded
                        "angle_start": 0.0,
                        "angle_end": 0.0,
                        "time_offset": 0.0,
                        "general_system_state": True,
                        "derived_settings": True,
                        "measurement_data": True,
                        "intrusion_data": True,
                        "application_io_data": True,
                        "use_persistent_config": True,
                        "min_intensities": 0.0
                    }
                ],
                remappings=[
                    ('scan', PythonExpression(["'", LaunchConfiguration(
                        lidar_params[1]['name']), "' + '/scan'"])),
                    ('extended_scan', PythonExpression(["'", LaunchConfiguration(
                        lidar_params[1]['name']), "' + '/extended_scan'"])),
                    ('output_paths', PythonExpression(["'", LaunchConfiguration(
                        lidar_params[1]['name']), "' + '/output_paths'"])),
                    ('raw_data', PythonExpression(
                        ["'", LaunchConfiguration(lidar_params[1]['name']), "' + '/raw_data'"]))
                ]
            ),
            launch_localization_node_arg,
            DeclareLaunchArgument(name='use_sim_time', default_value='True',
                description='Flag to enable use_sim_time in robot_localization_node'),
            robot_localization_node,
            rviz_node
        ])
