@echo off
rem Visual Studio 2019 환경 설정 (경로는 VS 설치 위치에 따라 다를 수 있음)
@REM call "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvars32.bat"
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars32.bat"

rem ROS2 환경 설정
call C:\dev\ros2-foxy\setup.bat

rem 작업 디렉토리로 이동
@REM cd C:\Users\SSAFY\Desktop\S12P21D106\ros2\ros2_ws

rem colcon build 실행
colcon build

rem 빌드 완료 후 사용자에게 알림
echo.
echo 빌드가 완료되었습니다.
pause 