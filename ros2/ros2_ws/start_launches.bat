@echo off
rem ROS2 환경 설정
call C:\dev\ros2-foxy\setup.bat

@REM call C:\Users\SSAFY\Desktop\S12P21D106\ros2\ros2_ws\install\local_setup.bat
call .\install\local_setup.bat

rem 0. integrated_launch.py 실행(통합 런치치 실행)
rem start ros2 launch auto_driving_cpp integrated_launch.py



rem 1. perception_launch.py 실행(인지 실행)
start ros2 launch perception_cpp perception_launch.py

rem 2. unity_dijkstra_launch.py 실행(판단/제어 실행)
start ros2 launch auto_driving_cpp unity_dijkstra_launch.py


start ros2 run topic_bridge_py mqtt_ros2_bridge

start ros2 run topic_bridge_py ros2_mqtt_bridge

rem 3. endpoint.py 실행(unity와 연결 실행)
@REM start ros2 launch ros_tcp_endpoint endpoint.py
start ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1 -p ROS_TCP_PORT:=10000


start ros2 run perception_py opencv_yolo

start ros2 run perception_py fire_image_subscriber

start ros2 run fire_control fire_suppression_node



rem 4. goal_publisher 노드(좌표 입력을 쉽게 하기 위해 따로 실행)
@REM start ros2 run auto_driving_cpp goal_publisher  

pause