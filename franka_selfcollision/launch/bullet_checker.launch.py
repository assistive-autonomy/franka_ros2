import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    urdf_file_path = "/ros2_ws/src/franka_description/urdfs/fr3_duo.urdf"
    srdf_file_path = "/ros2_ws/src/franka_description/urdfs/fr3_duo_arms.srdf"

    with open(urdf_file_path, 'r') as f:
        robot_desc = f.read()

    with open(srdf_file_path, 'r') as f:
        semantic_content = f.read()

    moveit_params = {
        "robot_description": robot_desc,
        "robot_description_semanctic": semantic_content
    }

    return LaunchDescription([
        
        Node(
            package='franka_selfcollision',
            executable='self_collision_node_bullet',
            name='self_collision_node_bullet',
            output='screen',
            parameters=[moveit_params]
        )
    ])