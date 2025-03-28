from setuptools import setup

package_name = 'test_topic_publisher_py'

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
    maintainer_email='ehddnr0929@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'smooth_position_publisher = test_topic_publisher_py.smooth_position_publisher:main',
            'test_scan_topic_publisher = test_topic_publisher_py.test_scan_topic_publisher:main',
            'test_turtlebot_status_topic_publisher = test_topic_publisher_py.test_turtlebot_status_topic_publisher:main',
        ],
    },
)
