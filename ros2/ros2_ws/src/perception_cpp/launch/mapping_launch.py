from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. odom_unity 노드
        Node(
            package='perception_cpp',
            executable='odom_unity',
            name='odom_unity',
            output='screen'
        ),

        # 2. lidar_filter 노드
        Node(
            package='perception_cpp',
            executable='lidar_filter',
            name='lidar_filter',
            output='screen'
        ),

        # 3. mapping 노드
        Node(
            package='perception_cpp',
            executable='mapping',
            name='mapping',
            output='screen'
        ),
    ]) 