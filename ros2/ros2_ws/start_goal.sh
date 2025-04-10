#!/bin/bash

# ROS2 환경 설정
source /opt/ros/foxy/setup.bash

# 워크스페이스 설정
source ./install/local_setup.bash

# goal_control_node 실행
ros2 run topic_bridge_py goal_control_node 