#=============================================================================
# Project Basic Settings
#=============================================================================
cmake_minimum_required(VERSION 3.10)
project(perception_cpp)

#=============================================================================
# Compiler Settings
#=============================================================================
# Use C99 standard
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Use C++14 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Compiler warning settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# UTF-8 settings for MSVC compiler
if(MSVC)
  add_compile_options(/utf-8)
endif()

#=============================================================================
# ROS2 Dependency Package Settings
#=============================================================================
# Basic ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Message type packages
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# External libraries
find_package(Eigen3 REQUIRED)
find_package(common_utils REQUIRED)
# find_package(OpenCV REQUIRED)

#=============================================================================
# ROS2 Node Execution File Creation
#=============================================================================
# 0. Basic sensor data processing nodes
# 0-1. odom_unity: Convert Unity simulation position data to Odometry
add_executable(odom_unity src/0_1_odom_unity.cpp)
ament_target_dependencies(odom_unity
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "tf2_ros"
)

# 0-2. lidar_filter: LiDAR data filtering and preprocessing
add_executable(lidar_filter src/0_2_lidar_filter.cpp)
ament_target_dependencies(lidar_filter
  "rclcpp"
  "sensor_msgs"
  "nav_msgs"
)

# 1. Cost map generation nodes
# 1-1. cost_map_global: Global cost map generation
add_executable(cost_map_global src/1_1_cost_map_global.cpp)
ament_target_dependencies(cost_map_global
  "rclcpp"
  "nav_msgs"
  "common_utils"
)

# 1-2. cost_map_local: Local cost map generation
add_executable(cost_map_local src/1_2_cost_map_local.cpp)
ament_target_dependencies(cost_map_local
  "rclcpp"
  "nav_msgs"
  "sensor_msgs"
  "tf2_ros"
)

# 2. Path generation and mapping nodes
# 2-1. make_path: Path recording and saving
add_executable(make_path src/make_path.cpp)
ament_target_dependencies(make_path
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "common_utils"
)

# mapping target creation
add_executable(mapping src/mapping.cpp)
target_include_directories(mapping PRIVATE ${EIGEN3_INCLUDE_DIRS})
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
# Installation Settings
#=============================================================================
# Execution file installation
install(TARGETS
  odom_unity
  lidar_filter
  cost_map_global
  cost_map_local
  make_path
  mapping
  DESTINATION lib/${PROJECT_NAME}
)

# Header file installation settings
# install(
#   DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
# )

# Map file installation
install(
  DIRECTORY map/
  DESTINATION share/${PROJECT_NAME}/map
)

# Path file installation
install(
  DIRECTORY path/
  DESTINATION share/${PROJECT_NAME}/path
)

# Launch file installation
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

#=============================================================================
# Test Settings
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
