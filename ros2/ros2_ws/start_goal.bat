@echo off
rem ROS2 환경 설정
call C:\dev\ros2-foxy\setup.bat

@REM call C:\Users\SSAFY\Desktop\S12P21D106\ros2\ros2_ws\install\local_setup.bat
call .\install\local_setup.bat


@REM start ros2 run topic_bridge_py control_node
start ros2 run topic_bridge_py goal_control_node


rem 4. goal_publisher 노드(좌표 입력을 쉽게 하기 위해 따로 실행)
@REM start ros2 run auto_driving_cpp goal_publisher  

pause