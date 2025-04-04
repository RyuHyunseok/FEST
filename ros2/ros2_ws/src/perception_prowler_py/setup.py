from setuptools import setup
import os
from glob import glob

package_name = 'perception_prowler_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SSAFY',
    maintainer_email='SSAFY@todo.todo',
    description='Unity camera to ROS2 bridge package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'opencv_yolo = perception_prowler_py.opencv_yolo:main',
            'fire_particle_listener = perception_prowler_py.fire_particle_listener:main'
        ],
    },
)
