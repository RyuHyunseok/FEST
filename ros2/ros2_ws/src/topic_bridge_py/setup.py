from setuptools import setup

package_name = 'topic_bridge_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/goal', ['goal/goal_list.txt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SSAFY',
    maintainer_email='ehddnr0929@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mqtt_command_subscriber = topic_bridge_py.mqtt_command_subscriber:main',
            'ros2_mqtt_bridge = topic_bridge_py.ros2_mqtt_bridge:main',
            'mqtt_ros2_bridge = topic_bridge_py.mqtt_ros2_bridge:main',
            'control_node = topic_bridge_py.4_control_no_mqtt:main',
            'goal_control_node = topic_bridge_py.4_goal_control:main',
        ],
    },
)
