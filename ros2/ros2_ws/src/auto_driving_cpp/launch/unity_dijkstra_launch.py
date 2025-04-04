from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. global_dijkstra_path 노드
        Node(
            package='auto_driving_cpp',
            executable='global_dijkstra_path',
            name='global_dijkstra_path',
            output='screen'
        ),

        # 2. local_dijkstra_path 노드
        Node(
            package='auto_driving_cpp',
            executable='local_dijkstra_path',
            name='local_dijkstra_path',
            output='screen'
        ),

        # 3. follow_unity 노드
        Node(
            package='auto_driving_cpp',
            executable='follow_unity',
            name='follow_unity',
            output='screen'
        ),

        # # 4. goal_publisher_manual 노드
        # Node(
        #     package='auto_driving_cpp',
        #     executable='goal_publisher_manual',
        #     name='goal_publisher_manual',
        #     output='screen'
        # ),
    ]) 