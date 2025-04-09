#=============================================================================
# Project Basic Settings
#=============================================================================
# CMake minimum version requirement
cmake_minimum_required(VERSION 3.10)

# Project name setting
project(auto_driving_cpp)

#=============================================================================
# Compiler Settings
#=============================================================================
# Use C99 standard for C code compilation
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Use C++14 standard (ROS2 Foxy default C++ standard)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Enable warning messages for GCC or Clang compiler
# -Wall: All common warnings
# -Wextra: Extra warnings
# -Wpedantic: ISO C/C++ standard compliance check
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# UTF-8 settings for MSVC compiler
if(MSVC)
  add_compile_options(/utf-8)
  add_compile_definitions(_UNICODE UNICODE)
endif()

#=============================================================================
# ROS2 Dependency Package Settings
#=============================================================================
# Basic ROS2 packages
find_package(ament_cmake REQUIRED)  # ROS2 CMake build system
find_package(rclcpp REQUIRED)      # ROS2 C++ client library
find_package(std_msgs REQUIRED)    # Standard message type

# Navigation related packages
find_package(nav_msgs REQUIRED)          # Navigation message type
find_package(geometry_msgs REQUIRED)     # Geometric message type
find_package(tf2_ros REQUIRED)          # Coordinate transformation library
find_package(sensor_msgs REQUIRED)       # Sensor data message type
find_package(visualization_msgs REQUIRED) # Visualization message type

# User-defined utility packages
find_package(common_utils REQUIRED)      # Common utility functions

#=============================================================================
# ROS2 Node Execution File Creation
#=============================================================================
# 1. Global path planning related nodes
# 1-1. global_dijkstra_path: Global path planning using Dijkstra algorithm
add_executable(global_dijkstra_path src/2_1_global_dijkstra_path.cpp)
ament_target_dependencies(global_dijkstra_path
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "visualization_msgs")

# 1-2. global_path_publisher: Publish calculated global path
add_executable(global_path_publisher src/2_1_global_path_publisher.cpp)
ament_target_dependencies(global_path_publisher
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "common_utils")

# 2. Local path planning related nodes
# 2-1. local_dijkstra_path: Local path planning using Dijkstra algorithm
add_executable(local_dijkstra_path src/2_2_local_dijkstra_path.cpp)
ament_target_dependencies(local_dijkstra_path
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "tf2_ros")

# 2-2. local_path_planner: Local path planning considering dynamic obstacles
add_executable(local_path_planner src/2_2_local_path_planner.cpp)
ament_target_dependencies(local_path_planner
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "tf2_ros"
  "visualization_msgs")

# 3. Robot control node
# Control the movement of the robot in Unity simulation environment
add_executable(follow_unity src/3_follow_unity.cpp)
ament_target_dependencies(follow_unity
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "tf2_ros"
  "std_msgs")

# 4. Goal management related nodes
# 4-1. goal_publisher: Integrated goal management for manual/automatic modes
add_executable(goal_publisher src/goal_publisher.cpp)
ament_target_dependencies(goal_publisher
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "common_utils")

# 4-2. goal_publisher_manual: Manual goal setting based on user input
add_executable(goal_publisher_manual src/goal_publisher_manual.cpp)
ament_target_dependencies(goal_publisher_manual
  "rclcpp"
  "geometry_msgs")

# 4-3. goal_publisher_auto: Automatic goal management based on predefined path
add_executable(goal_publisher_auto src/goal_publisher_auto.cpp)
ament_target_dependencies(goal_publisher_auto
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "common_utils")

#=============================================================================
# Installation and Test Settings
#=============================================================================
# Execution file installation settings
# Install all execution files to lib/${PROJECT_NAME} directory
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

# Test settings (enabled if build test is activated)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#=============================================================================
# Additional File Installation Settings
#=============================================================================
# launch file installation
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

# Header file installation
install(DIRECTORY
  include/
  DESTINATION include/)

# ROS2 package settings completed
ament_package()
