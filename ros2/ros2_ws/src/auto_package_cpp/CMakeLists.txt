cmake_minimum_required(VERSION 3.10)
project(auto_package_cpp)

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
find_package(ament_cmake_target_dependencies REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# 의존성 패키지 추가
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ssafy_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 기본 노드 실행 파일
add_executable(my_node src/my_node.cpp)
target_include_directories(my_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(my_node
  "rclcpp"
  "std_msgs"
)

# odom_node 추가
add_executable(odom_node src/1_odom_node.cpp)
ament_target_dependencies(odom_node
  "rclcpp"
  "ssafy_msgs"
  "nav_msgs"
  "geometry_msgs"
  "tf2_ros"
)

# path_publisher 노드 추가
add_executable(path_publisher src/2_path_publisher.cpp)
ament_target_dependencies(path_publisher
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
)

# follow_the_carrot 노드 추가
add_executable(follow_the_carrot src/3_follow_the_carrot.cpp)
ament_target_dependencies(follow_the_carrot
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "ssafy_msgs"
  "sensor_msgs"
  "tf2_ros"
)

# make_path 노드 추가
add_executable(make_path src/make_path.cpp)
ament_target_dependencies(make_path
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
)

# lattice_planner 노드 추가
add_executable(lattice_planner src/2_lattice_planner.cpp)
ament_target_dependencies(lattice_planner
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "tf2_ros"
  "visualization_msgs"
  "ssafy_msgs"
)

# mapping_node 추가
if(WIN32)
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "C:/dev/ros2-windows/share/eigen3_cmake_module/cmake")
  find_package(Eigen3 REQUIRED)
  set(EIGEN3_INCLUDE_DIR "C:/dev/ros2-windows/include/eigen3")
endif()

add_executable(mapping_node src/mapping.cpp)
target_include_directories(mapping_node PRIVATE
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(mapping_node
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "Eigen3"
)

# map_visualizer 노드 추가
add_executable(map_visualizer src/1_map_visualizer.cpp)
ament_target_dependencies(map_visualizer
  "rclcpp"
  "nav_msgs"
)

# a_star_planner 노드 추가
# add_executable(a_star_planner src/a_star_planner.cpp)
# ament_target_dependencies(a_star_planner
#   "rclcpp"
#   "nav_msgs"
#   "geometry_msgs"
#   "ssafy_msgs"
#   "visualization_msgs"
# )

# dijkstra_planner 노드 추가
add_executable(dijkstra_planner src/2_dijkstra_planner.cpp)
ament_target_dependencies(dijkstra_planner
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "ssafy_msgs"
  "visualization_msgs"
)

# local_cost_map_generator 노드 추가
add_executable(local_cost_map_generator src/1_local_cost_map_generator.cpp)
ament_target_dependencies(local_cost_map_generator
  "rclcpp"
  "nav_msgs"
  "sensor_msgs"
  "tf2_ros"
)

# local_path_planner 노드 추가
add_executable(local_path_planner src/2_local_path_planner.cpp)
ament_target_dependencies(local_path_planner
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "tf2_ros"
)

# goal_publisher 노드 추가
add_executable(goal_publisher src/goal_publisher.cpp)
ament_target_dependencies(goal_publisher
  "rclcpp"
  "geometry_msgs"
)

# 설치 대상 추가
install(TARGETS 
  my_node 
  odom_node 
  path_publisher 
  follow_the_carrot 
  make_path 
  lattice_planner 
  mapping_node 
  map_visualizer 
  dijkstra_planner 
  local_cost_map_generator
  local_path_planner
  goal_publisher
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

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

# 런치 파일 설치
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

ament_package()
