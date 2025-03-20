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
        
        # 2. map_visualizer 노드
        Node(
            package='auto_package_cpp',
            node_executable='map_visualizer',
            node_name='map_visualizer',
            output='screen'
        ),
        
        # 3. local_cost_map_generator 노드
        Node(
            package='auto_package_cpp',
            node_executable='local_cost_map_generator',
            node_name='local_cost_map_generator',
            output='screen'
        ),

        # 4. dijkstra_planner노드
        Node(
            package='auto_package_cpp',
            node_executable='dijkstra_planner',
            node_name='dijkstra_planner',
            output='screen'
        ),

        # 5. local_path_planner 노드
        Node(
            package='auto_package_cpp',
            node_executable='local_path_planner',
            node_name='local_path_planner',
            output='screen'
        ),

        # 6. follow_the_carrot 노드
        Node(
            package='auto_package_cpp',
            node_executable='follow_the_carrot',
            node_name='follow_the_carrot',
            output='screen'
        ),
    ]) 