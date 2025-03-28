from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Odometry 노드
        Node(
            package='auto_package_cpp',
            node_executable='odom_node',
            node_name='odom_node',
            output='screen'
        ),
        
        # mapping 노드
        Node(
            package='auto_package_cpp',
            node_executable='mapping_node',
            node_name='mapping_node',
            output='screen'
        ),
    ]) 