#=============================================================================
# 프로젝트 기본 설정
#=============================================================================
cmake_minimum_required(VERSION 3.5)
project(perception_cpp)

#=============================================================================
# 컴파일러 설정
#=============================================================================
# C99 표준 사용
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# C++14 표준 사용
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 컴파일러 경고 설정
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#=============================================================================
# ROS2 의존성 패키지 설정
#=============================================================================
# 기본 ROS2 패키지
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 메시지 타입 패키지
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# 외부 라이브러리
find_package(Eigen3 REQUIRED)
# find_package(OpenCV REQUIRED)
find_package(common_utils REQUIRED)

#=============================================================================
# ROS2 노드 실행 파일 생성
#=============================================================================
# 0. 기본 센서 데이터 처리 노드들
# 0-1. odom_unity: Unity 시뮬레이션의 위치 데이터를 Odometry로 변환
add_executable(odom_unity src/0_1_odom_unity.cpp)
ament_target_dependencies(odom_unity
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "tf2_ros"
)

# 0-2. lidar_filter: LiDAR 데이터 필터링 및 전처리
add_executable(lidar_filter src/0_2_lidar_filter.cpp)
ament_target_dependencies(lidar_filter
  "rclcpp"
  "sensor_msgs"
  "nav_msgs"
)

# 1. 비용 맵 생성 노드들
# 1-1. cost_map_global: 전역 비용 맵 생성
add_executable(cost_map_global src/1_1_cost_map_global.cpp)
ament_target_dependencies(cost_map_global
  "rclcpp"
  "nav_msgs"
  "common_utils"
)

# 1-2. cost_map_local: 지역 비용 맵 생성
add_executable(cost_map_local src/1_2_cost_map_local.cpp)
ament_target_dependencies(cost_map_local
  "rclcpp"
  "nav_msgs"
  "sensor_msgs"
  "tf2_ros"
)

# 2. 경로 생성 및 맵핑 노드들
# 2-1. make_path: 경로 기록 및 저장
add_executable(make_path src/make_path.cpp)
ament_target_dependencies(make_path
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "common_utils"
)

# 2-2. mapping: SLAM 기반 맵 생성
add_executable(mapping src/mapping.cpp)
ament_target_dependencies(mapping
  "rclcpp"
  "nav_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "tf2_ros"
  "Eigen3"
  "common_utils"
)

#=============================================================================
# 설치 설정
#=============================================================================
# 실행 파일 설치
install(TARGETS
  odom_unity
  lidar_filter
  cost_map_global
  cost_map_local
  make_path
  mapping
  DESTINATION lib/${PROJECT_NAME}
)

# 헤더 파일 설치 설정 추가
# install(
#   DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
# )

# 맵 파일 설치
install(
  DIRECTORY map/
  DESTINATION share/${PROJECT_NAME}/map
)

# 경로 파일 설치
install(
  DIRECTORY path/
  DESTINATION share/${PROJECT_NAME}/path
)

# 런치 파일 설치
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

#=============================================================================
# 테스트 설정
#=============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
