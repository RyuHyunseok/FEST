#=============================================================================
# 프로젝트 기본 설정
#=============================================================================
cmake_minimum_required(VERSION 3.10)
project(auto_package_cpp)

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

# GCC나 Clang 컴파일러 사용 시 경고 메시지 활성화
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#=============================================================================
# ROS2 의존성 패키지 설정
#=============================================================================
# 기본 ROS2 패키지
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_target_dependencies REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# nlohmann/json 라이브러리 찾기
# find_package(nlohmann_json REQUIRED)

# 커스텀 메시지 및 네비게이션 관련 패키지
find_package(ssafy_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 헤더 파일 포함 디렉토리 설정
include_directories(include)

#=============================================================================
# MQTT 라이브러리 설정
#=============================================================================
# Windows 환경에서 mosquitto 라이브러리 경로 설정
if(WIN32)
  if(DEFINED ENV{MOSQUITTO_DIR})
    if(EXISTS "$ENV{MOSQUITTO_DIR}/devel")
      set(MOSQUITTO_DIR "$ENV{MOSQUITTO_DIR}/devel")
    else()
      set(MOSQUITTO_DIR $ENV{MOSQUITTO_DIR})
    endif()
  else()
    set(POSSIBLE_PATHS
      "C:/Program Files/mosquitto/devel"
      "C:/Program Files (x86)/mosquitto/devel"
      "C:/mosquitto/devel"
      "${CMAKE_CURRENT_SOURCE_DIR}/mosquitto/devel"
      "C:/Program Files/mosquitto"
      "C:/Program Files (x86)/mosquitto"
      "C:/mosquitto"
      "C:/opt/mosquitto"
      "C:/opt/mosquitto/devel"
      "C:/msys64/mingw64/include"
      "C:/msys64/mingw64/lib"
      "C:/msys64/mingw64"
    )

    foreach(PATH ${POSSIBLE_PATHS})
      if(EXISTS "${PATH}/mosquitto.h")
        set(MOSQUITTO_DIR ${PATH})
        break()
      endif()
    endforeach()
  endif()
endif()

#=============================================================================
# 유틸리티 라이브러리 생성
#=============================================================================
# 파일 경로 처리 라이브러리
add_library(file_path src/utils/file_path.cpp)
target_include_directories(file_path PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(file_path "ament_index_cpp")

# MQTT 통신 처리 라이브러리
add_library(mqtt_handler src/utils/mqtt_handler.cpp)
target_include_directories(mqtt_handler PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# MQTT 라이브러리 링크 설정
if(WIN32)
  target_include_directories(mqtt_handler PRIVATE 
    "${MOSQUITTO_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/include")
  target_link_directories(mqtt_handler PRIVATE "${MOSQUITTO_DIR}")
  target_link_libraries(mqtt_handler mosquitto)
else()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(MOSQUITTO REQUIRED mosquitto)
  target_include_directories(mqtt_handler PRIVATE ${MOSQUITTO_INCLUDE_DIRS})
  target_link_libraries(mqtt_handler ${MOSQUITTO_LIBRARIES})
endif()

#=============================================================================
# ROS2 노드 실행 파일 생성
#=============================================================================
# 1. odom_node: 로봇의 위치 정보를 발행하는 노드
add_executable(odom_node src/1_odom_node.cpp)
ament_target_dependencies(odom_node
  "rclcpp"
  "ssafy_msgs"
  "nav_msgs"
  "geometry_msgs"
  "tf2_ros")

# 2. global_cost_map_generator: 전역 비용 지도 생성 노드
add_executable(global_cost_map_generator src/1_global_cost_map_generator.cpp)
ament_target_dependencies(global_cost_map_generator
  "rclcpp"
  "nav_msgs")
target_link_libraries(global_cost_map_generator file_path)

# 3. local_cost_map_generator: 지역 비용 지도 생성 노드
add_executable(local_cost_map_generator src/1_local_cost_map_generator.cpp)
ament_target_dependencies(local_cost_map_generator
  "rclcpp"
  "nav_msgs"
  "sensor_msgs"
  "tf2_ros")
target_link_libraries(local_cost_map_generator file_path)

# 4. global_path_publisher: 전역 경로 발행 노드
add_executable(global_path_publisher src/2_global_path_publisher.cpp)
ament_target_dependencies(global_path_publisher
  "rclcpp"
  "nav_msgs"
  "geometry_msgs")
target_link_libraries(global_path_publisher file_path)

# 5. local_path_planner: 지역 경로 계획 노드
add_executable(local_path_planner src/2_local_path_planner.cpp)
ament_target_dependencies(local_path_planner
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "tf2_ros"
  "visualization_msgs"
  "ssafy_msgs")

# 6. global_dijkstra_path: 전역 다익스트라 경로 계획 노드
add_executable(global_dijkstra_path src/2_global_dijkstra_path.cpp)
ament_target_dependencies(global_dijkstra_path
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "ssafy_msgs"
  "visualization_msgs")

# MQTT 통신을 위한 설정
if(WIN32)
  target_include_directories(global_dijkstra_path PRIVATE 
    "${MOSQUITTO_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/include")
  target_link_directories(global_dijkstra_path PRIVATE "${MOSQUITTO_DIR}")
  target_link_libraries(global_dijkstra_path
    mqtt_handler
    file_path
    mosquitto)
else()
  target_link_libraries(global_dijkstra_path
    mqtt_handler
    file_path)
endif()

# 7. local_dijkstra_path: 지역 다익스트라 경로 계획 노드
add_executable(local_dijkstra_path src/2_local_dijkstra_path.cpp)
ament_target_dependencies(local_dijkstra_path
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "tf2_ros")

# 8. follow_the_carrot: carrot following 알고리즘 구현 노드
add_executable(follow_the_carrot src/3_follow_the_carrot.cpp)
ament_target_dependencies(follow_the_carrot
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "ssafy_msgs"
  # "sensor_msgs"
  "tf2_ros")

# 9. mapping_node: SLAM 매핑 노드
add_executable(mapping_node src/mapping.cpp)
target_include_directories(mapping_node PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(mapping_node
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "Eigen3")
target_link_libraries(mapping_node file_path)

# 10. make_path: 경로 생성 노드
add_executable(make_path src/make_path.cpp)
ament_target_dependencies(make_path
  "rclcpp"
  "nav_msgs"
  "geometry_msgs")
target_link_libraries(make_path file_path)

# 11. goal_publisher: 목표점 발행 노드 (수동, 자동 모드 선택)
add_executable(goal_publisher src/goal_publisher.cpp)
ament_target_dependencies(goal_publisher
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "ament_index_cpp")
target_link_libraries(goal_publisher file_path)

# 11-1. goal_publisher_manual: 수동 목표점 발행 노드
add_executable(goal_publisher_manual src/goal_publisher_manual.cpp)
ament_target_dependencies(goal_publisher_manual
  "rclcpp"
  "geometry_msgs")

# 11-2. goal_publisher_auto: 자동 목표점 관리 노드
add_executable(goal_publisher_auto src/goal_publisher_auto.cpp)
ament_target_dependencies(goal_publisher_auto
  "rclcpp"
  "geometry_msgs"
  "nav_msgs")
target_link_libraries(goal_publisher_auto file_path)

# 11-3. test_odom_publisher: 테스트용 odom 발행 노드
add_executable(test_odom_publisher src/test_odom_publisher.cpp)
ament_target_dependencies(test_odom_publisher
  "rclcpp"
  "nav_msgs")

# 0. my_node: 기본 노드 (현재 비활성화)
# add_executable(my_node src/my_node.cpp)
# target_include_directories(my_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# ament_target_dependencies(my_node
#   "rclcpp"
#   "std_msgs")

#=============================================================================
# 설치 및 테스트 설정
#=============================================================================
# 실행 파일 설치 설정
install(TARGETS 
  file_path
  mqtt_handler
  odom_node
  global_cost_map_generator
  local_cost_map_generator
  global_path_publisher
  local_path_planner
  global_dijkstra_path
  local_dijkstra_path
  follow_the_carrot
  mapping_node
  make_path
  goal_publisher
  goal_publisher_manual
  goal_publisher_auto
  test_odom_publisher
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# 테스트 설정
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 추가 파일 설치 설정
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  include/
  DESTINATION include/)

install(DIRECTORY path/ DESTINATION share/${PROJECT_NAME}/path)

ament_package()
