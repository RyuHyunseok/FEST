@echo off
rem ROS2 환경 설정
call C:\dev\ros2-foxy\setup.bat

@REM call C:\Users\SSAFY\Desktop\S12P21D106\ros2\ros2_ws\install\local_setup.bat
call .\install\local_setup.bat

rem 0. integrated_launch.py 실행(통합 런치치 실행)
ros2 launch auto_driving_cpp integrated_launch.py

pause