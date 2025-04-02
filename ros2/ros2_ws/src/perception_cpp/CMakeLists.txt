cmake_minimum_required(VERSION 3.5)
project(perception_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(common_utils REQUIRED)

# include 디렉토리 추가
# include_directories(include)

# 실행 파일 추가
add_executable(cost_map_global src/cost_map_global.cpp)
add_executable(cost_map_local src/cost_map_local.cpp)
add_executable(lidar_filter src/lidar_filter.cpp)
add_executable(make_path src/make_path.cpp)
add_executable(mapping src/mapping.cpp)
add_executable(mapping_node src/mapping_node.cpp)
add_executable(odom_slam src/odom_slam.cpp)

# 의존성 연결
ament_target_dependencies(cost_map_global
  rclcpp
  nav_msgs
  common_utils
)

ament_target_dependencies(cost_map_local
  rclcpp
  nav_msgs
  sensor_msgs
  tf2_ros
)

ament_target_dependencies(lidar_filter
  rclcpp
  sensor_msgs
  nav_msgs
)

ament_target_dependencies(make_path
  rclcpp
  nav_msgs
  geometry_msgs
  common_utils
)

ament_target_dependencies(mapping
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  Eigen3
  common_utils
)

ament_target_dependencies(mapping_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  OpenCV
  common_utils
)

ament_target_dependencies(odom_slam
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  Eigen3
)

# 설치 대상 추가
install(TARGETS 
  cost_map_global
  cost_map_local
  lidar_filter
  make_path
  mapping
  mapping_node
  odom_slam
  DESTINATION lib/${PROJECT_NAME}
)

# 헤더 파일 설치 설정 추가
# install(
#   DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
# )

# 맵 파일 설치 설정 추가
install(
  DIRECTORY map/
  DESTINATION share/${PROJECT_NAME}/map
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

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
