from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. perception_cpp 패키지 노드들
        Node(
            package='perception_cpp',
            executable='odom_unity',
            name='odom_unity',
            output='screen'
        ),

        Node(
            package='perception_cpp',
            executable='lidar_filter',
            name='lidar_filter',
            output='screen'
        ),

        Node(
            package='perception_cpp',
            executable='cost_map_global',
            name='cost_map_global',
            output='screen'
        ),

        Node(
            package='perception_cpp',
            executable='cost_map_local',
            name='cost_map_local',
            output='screen'
        ),

        # 2. auto_driving_cpp 패키지 노드들
        Node(
            package='auto_driving_cpp',
            executable='global_dijkstra_path',
            name='global_dijkstra_path',
            output='screen'
        ),

        Node(
            package='auto_driving_cpp',
            executable='local_dijkstra_path',
            name='local_dijkstra_path',
            output='screen'
        ),

        Node(
            package='auto_driving_cpp',
            executable='follow_unity',
            name='follow_unity',
            output='screen'
        ),

        # goal_publisher_manual 노드는 원본에서 주석 처리되어 있어서 여기서도 주석 처리합니다
        # Node(
        #     package='auto_driving_cpp',
        #     executable='goal_publisher_manual',
        #     name='goal_publisher_manual',
        #     output='screen'
        # ),

        # 3. TCP 엔드포인트 노드
        Node(
            package="ros_tcp_endpoint",
            executable="default_server_endpoint",
            emulate_tty=True,
            parameters=[{"ROS_IP": "0.0.0.0"}, {"ROS_TCP_PORT": 10000}],
        ),
    ]) 