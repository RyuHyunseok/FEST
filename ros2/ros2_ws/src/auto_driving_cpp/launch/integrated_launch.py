from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. perception_cpp
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

        # 2. auto_driving_cpp
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

        # 3. MQTT
        Node(
            package='topic_bridge_py',
            executable='mqtt_ros2_bridge',
            name='mqtt_ros2_bridge',
            output='screen'
        ),

        Node(
            package='topic_bridge_py',
            executable='ros2_mqtt_bridge',
            name='ros2_mqtt_bridge',
            output='screen'
        ),

        # 4. fire_image
        Node(
            package='perception_py',
            executable='opencv_yolo',
            name='opencv_yolo',
            output='screen'
        ),

        Node(
            package='perception_py',
            executable='fire_image_subscriber',
            name='fire_image_subscriber',
            output='screen'
        ),

        Node(
            package='fire_control',
            executable='fire_suppression_node',
            name='fire_suppression_node',
            output='screen'
        ),

        # 5. TCP
        Node(
            package="ros_tcp_endpoint",
            executable="default_server_endpoint",
            emulate_tty=True,
            parameters=[{"ROS_IP": "0.0.0.0"}, {"ROS_TCP_PORT": 10000}],
        ),
    ]) 