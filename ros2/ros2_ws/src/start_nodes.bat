@echo off
rem ROS2 환경 설정
call C:\dev\ros2-foxy\setup.bat
call C:\Users\SSAFY\Desktop\S12P21D106\ros2\ros2_ws\install\local_setup.bat

rem 1. perception_cpp 패키지 노드들
start ros2 run perception_cpp odom_unity
start ros2 run perception_cpp lidar_filter
start ros2 run perception_cpp cost_map_global
start ros2 run perception_cpp cost_map_local

rem 2. auto_driving_cpp 패키지 노드들
start ros2 run auto_driving_cpp global_dijkstra_path
start ros2 run auto_driving_cpp local_dijkstra_path
start ros2 run auto_driving_cpp follow_unity

rem 3. TCP 엔드포인트 노드
start ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000

rem 4. goal_publisher 노드(좌표 입력을 쉽게 하기 위해 따로 실행)
start ros2 run auto_driving_cpp goal_publisher  

echo 모든 노드가 시작되었습니다.
pause 