@echo off
rem ROS2 환경 설정
call C:\dev\ros2-foxy\setup.bat

@REM call C:\Users\SSAFY\Desktop\S12P21D106\ros2\ros2_ws\install\local_setup.bat
call .\install\local_setup.bat

rem 3. endpoint.py 실행(unity와 연결 실행)
@REM ros2 launch ros_tcp_endpoint endpoint.py
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1 -p ROS_TCP_PORT:=10000

rem 에러 메시지 확인을 위해 창 유지
echo.
echo error

pause