from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 파라미터 선언
    map_resolution_arg = DeclareLaunchArgument(
        'map_resolution',
        default_value='0.05',
        description='Map resolution in meters/pixel'
    )
    
    map_width_arg = DeclareLaunchArgument(
        'map_width',
        default_value='2000',
        description='Map width in pixels'
    )
    
    map_height_arg = DeclareLaunchArgument(
        'map_height',
        default_value='2000',
        description='Map height in pixels'
    )

    # mapping_node 설정
    mapping_node = Node(
        package='perception_cpp',
        executable='mapping_node',
        name='mapping_node',
        parameters=[{
            'map_resolution': LaunchConfiguration('map_resolution'),
            'map_width': LaunchConfiguration('map_width'),
            'map_height': LaunchConfiguration('map_height'),
        }],
        output='screen'
    )

    return LaunchDescription([
        map_resolution_arg,
        map_width_arg,
        map_height_arg,
        mapping_node
    ]) 