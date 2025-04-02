from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ssafy_bridge',
            executable='udp_to_pub',
            name='udp_to_pub'
        ),
        Node(
            package='ssafy_bridge',
            executable='sub_to_udp',
            name='sub_to_udp'
        ),
        Node(
            package='ssafy_bridge',
            executable='udp_to_cam',
            name='udp_to_cam'
        ),

        Node(
            package='ssafy_bridge',
            executable='udp_to_laser',
            name='udp_to_laser'
        ),

        # Node(
        #     package='ssafy_bridge',
        #     node_executable='cam_viewer',
        #     node_name='cam_viewer'
        # ),
    ])



