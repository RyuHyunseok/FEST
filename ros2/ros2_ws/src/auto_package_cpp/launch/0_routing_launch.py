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
        
        # make_path 노드
        Node(
            package='auto_package_cpp',
            node_executable='make_path',
            node_name='make_path',
            output='screen'
        ),
    ]) 