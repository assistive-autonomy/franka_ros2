import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    urdf_file_path = "/ros2_ws/src/franka_selfcollision/urdfs/fr3_duo.urdf"
    rviz_config_path = "/ros2_ws/src/franka_selfcollision/config/visualize_franka.rviz"
    monitor_script_path = "/ros2_ws/src/franka_selfcollision/scripts/monitor_collision.py"

    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            remappings=[('joint_states', '/joint_states')] 
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path]
        ),

        Node(
            package='franka_selfcollision',
            executable='self_collision_node',
            name='self_collision_node'
        ),

        ExecuteProcess(
            cmd=['python3', monitor_script_path],
            output='screen',
            emulate_tty=True,  
            name='monitor_collision_script'
        )

    ])