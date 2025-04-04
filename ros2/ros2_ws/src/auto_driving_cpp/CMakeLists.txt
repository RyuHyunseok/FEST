#=============================================================================
# 프로젝트 기본 설정
#=============================================================================
# CMake 최소 버전 요구사항 설정
cmake_minimum_required(VERSION 3.5)

# 프로젝트 이름 설정
project(auto_driving_cpp)

#=============================================================================
# 컴파일러 설정
#=============================================================================
# C99 표준 사용 - C 코드 컴파일 시 적용
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# C++14 표준 사용 - ROS2 Foxy 기본 C++ 표준
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# GCC나 Clang 컴파일러 사용 시 경고 메시지 활성화
# -Wall: 모든 일반적인 경고
# -Wextra: 추가 경고
# -Wpedantic: ISO C/C++ 표준 준수 여부 검사
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# MSVC 컴파일러를 위한 UTF-8 설정
if(MSVC)
  add_compile_options(/utf-8)
endif()

#=============================================================================
# ROS2 의존성 패키지 설정
#=============================================================================
# 기본 ROS2 패키지
find_package(ament_cmake REQUIRED)  # ROS2 CMake 빌드 시스템
find_package(rclcpp REQUIRED)      # ROS2 C++ 클라이언트 라이브러리
find_package(std_msgs REQUIRED)    # 표준 메시지 타입

# 네비게이션 관련 패키지
find_package(nav_msgs REQUIRED)          # 네비게이션 메시지 타입
find_package(geometry_msgs REQUIRED)     # 기하학적 메시지 타입
find_package(tf2_ros REQUIRED)          # 좌표 변환 라이브러리
find_package(sensor_msgs REQUIRED)       # 센서 데이터 메시지 타입
find_package(visualization_msgs REQUIRED) # 시각화 메시지 타입

# 사용자 정의 유틸리티 패키지
find_package(common_utils REQUIRED)      # 공통 유틸리티 기능

#=============================================================================
# ROS2 노드 실행 파일 생성
#=============================================================================
# 1. 전역 경로 계획 관련 노드들
# 1-1. global_dijkstra_path: 다익스트라 알고리즘을 사용한 전역 경로 계획
add_executable(global_dijkstra_path src/2_1_global_dijkstra_path.cpp)
ament_target_dependencies(global_dijkstra_path
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "visualization_msgs")

# 1-2. global_path_publisher: 계산된 전역 경로를 발행
add_executable(global_path_publisher src/2_1_global_path_publisher.cpp)
ament_target_dependencies(global_path_publisher
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "common_utils")

# 2. 지역 경로 계획 관련 노드들
# 2-1. local_dijkstra_path: 지역 환경에서의 다익스트라 경로 계획
add_executable(local_dijkstra_path src/2_2_local_dijkstra_path.cpp)
ament_target_dependencies(local_dijkstra_path
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "tf2_ros")

# 2-2. local_path_planner: 동적 장애물을 고려한 지역 경로 계획
add_executable(local_path_planner src/2_2_local_path_planner.cpp)
ament_target_dependencies(local_path_planner
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "tf2_ros"
  "visualization_msgs")

# 3. 로봇 제어 노드
# Unity 시뮬레이션 환경에서 로봇의 움직임을 제어
add_executable(follow_unity src/3_follow_unity.cpp)
ament_target_dependencies(follow_unity
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "tf2_ros"
  "std_msgs")

# 4. 목표점 관리 관련 노드들
# 4-1. goal_publisher: 수동/자동 모드 통합 목표점 관리
add_executable(goal_publisher src/goal_publisher.cpp)
ament_target_dependencies(goal_publisher
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "common_utils")

# 4-2. goal_publisher_manual: 사용자 입력에 의한 수동 목표점 설정
add_executable(goal_publisher_manual src/goal_publisher_manual.cpp)
ament_target_dependencies(goal_publisher_manual
  "rclcpp"
  "geometry_msgs")

# 4-3. goal_publisher_auto: 미리 정의된 경로에 따른 자동 목표점 관리
add_executable(goal_publisher_auto src/goal_publisher_auto.cpp)
ament_target_dependencies(goal_publisher_auto
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "common_utils")

#=============================================================================
# 설치 및 테스트 설정
#=============================================================================
# 실행 파일 설치 설정
# lib/${PROJECT_NAME} 디렉토리에 모든 실행 파일 설치
install(TARGETS
  global_dijkstra_path
  global_path_publisher
  local_dijkstra_path
  local_path_planner
  follow_unity
  goal_publisher
  goal_publisher_manual
  goal_publisher_auto
  DESTINATION lib/${PROJECT_NAME})

# 테스트 설정 (빌드 테스트가 활성화된 경우)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#=============================================================================
# 추가 파일 설치 설정
#=============================================================================
# launch 파일 설치
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

# 헤더 파일 설치
install(DIRECTORY
  include/
  DESTINATION include/)

# ROS2 패키지 설정 완료
ament_package()
