
## ROS2명령어(OpenCV_YOLO)
#### build
call C:\dev\ros2-foxy\setup.bat
cd C:\Users\SSAFY\Desktop\<git 받은 파일명>\S12P21D106\ros2\ros2_ws
colcon build --symlink-install

#### endpoint
call C:\dev\ros2-foxy\setup.bat
call C:\Users\SSAFY\Desktop\<git 받은 파일명>\S12P21D106\ros2\ros2_ws\install\local_setup.bat
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1 -p ROS_TCP_PORT:=10000

#### 일반 실행 (빌드 후)
call C:\dev\ros2-foxy\setup.bat
call C:\Users\SSAFY\Desktop\<git 받은 파일명>\S12P21D106\ros2\ros2_ws\install\local_setup.bat
cd C:\Users\SSAFY\Desktop\<git 받은 파일명>\S12P21D106\ros2\ros2_ws\src\unity_camera\unity_camera
ros2 run unity_camera opencv_yolo

#### OpenCV 창닫기
- q

