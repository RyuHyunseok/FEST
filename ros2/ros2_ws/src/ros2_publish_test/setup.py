from setuptools import setup

package_name = 'ros2_publish_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SSAFY',
    maintainer_email='SSAFY@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'smooth_position_publisher = ros2_publish_test.smooth_position_publisher:main',
            'mqtt_command_subscriber = ros2_publish_test.mqtt_command_subscriber:main',
            'ros2_mqtt_bridge = ros2_publish_test.ros2_mqtt_bridge:main',
            'mqtt_ros2_bridge = ros2_publish_test.mqtt_ros2_bridge:main',
        ],
    },
)
