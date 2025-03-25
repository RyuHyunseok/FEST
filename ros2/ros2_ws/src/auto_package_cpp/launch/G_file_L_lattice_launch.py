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
        
        # 2. global_path_publisher 노드
        Node(
            package='auto_package_cpp',
            node_executable='global_path_publisher',
            node_name='global_path_publisher',
            output='screen'
        ),

        # 2. local_path_planner 노드
        Node(
            package='auto_package_cpp',
            node_executable='local_path_planner',
            node_name='local_path_planner',
            output='screen'
        ),
        
        # 3. Follow The Carrot 노드
        Node(
            package='auto_package_cpp',
            node_executable='follow_the_carrot',
            node_name='follow_the_carrot',
            output='screen'
        ),
        
        # goal_publisher 노드
        # Node(
        #     package='auto_package_cpp',
        #     node_executable='goal_publisher',
        #     node_name='goal_publisher',
        #     output='screen'
        # ),
    ]) 