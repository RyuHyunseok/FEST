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
        
        # 2. Path Publisher 노드
        Node(
            package='auto_package_cpp',
            node_executable='path_publisher',
            node_name='path_publisher',
            output='screen'
        ),
        
        # 3. Follow The Carrot 노드
        Node(
            package='auto_package_cpp',
            node_executable='follow_the_carrot',
            node_name='follow_the_carrot',
            output='screen'
        ),

        # Make Path 노드
        # Node(
        #     package='auto_package_cpp',
        #     node_executable='make_path',
        #     node_name='make_path',
        #     output='screen'
        # ),

        # 4. lattice planner 노드
        # Node(
        #     package='auto_package_cpp',
        #     node_executable='lattice_planner',
        #     node_name='lattice_planner',
        #     output='screen'
        # ),

        # 5. map visualizer 노드
        Node(
            package='auto_package_cpp',
            node_executable='map_visualizer',
            node_name='map_visualizer',
            output='screen'
        ),
    ]) 