from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. odom_node 노드
        Node(
            package='auto_package_cpp',
            node_executable='odom_node',
            node_name='odom_node',
            output='screen'
        ),
        
        # 1. global_cost_map_generator 노드
        Node(
            package='auto_package_cpp',
            node_executable='global_cost_map_generator',
            node_name='global_cost_map_generator',
            output='screen'
        ),
        
        # 1. local_cost_map_generator 노드
        Node(
            package='auto_package_cpp',
            node_executable='local_cost_map_generator',
            node_name='local_cost_map_generator',
            output='screen'
        ),

        # 2. global_dijkstra_path 노드
        Node(
            package='auto_package_cpp',
            node_executable='global_dijkstra_path',
            node_name='global_dijkstra_path',
            output='screen'
        ),

        # 2. local_dijkstra_path 노드
        Node(
            package='auto_package_cpp',
            node_executable='local_dijkstra_path',
            node_name='local_dijkstra_path',
            output='screen'
        ),

        # 3. follow_the_carrot 노드
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