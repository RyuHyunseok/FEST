#!/bin/bash

# pip3 설치
# if ! command -v pip3 &> /dev/null; then
#     echo "pip3가 설치되어 있지 않습니다. 설치를 시작합니다..."
#     sudo apt-get update
#     sudo apt-get install -y python3-pip
# fi

# # 필요한 Python 패키지 설치
# pip3 install opencv-python paho-mqtt

# # ROS2 cv-bridge 및 관련 패키지 설치
# sudo apt-get install -y ros-foxy-cv-bridge ros-foxy-vision-opencv

# ROS2 환경 설정
source /opt/ros/foxy/setup.bash

# 워크스페이스 설정
source ./install/local_setup.bash

# 통합 런치 파일 실행
ros2 launch auto_driving_cpp integrated_launch.py 