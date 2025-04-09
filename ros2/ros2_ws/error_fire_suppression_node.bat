@echo off
rem ROS2 환경 설정
call C:\dev\ros2-foxy\setup.bat

@REM call C:\Users\SSAFY\Desktop\S12P21D106\ros2\ros2_ws\install\local_setup.bat
call .\install\local_setup.bat


@REM start ros2 run topic_bridge_py control_node
ros2 run fire_control fire_suppression_node


pause