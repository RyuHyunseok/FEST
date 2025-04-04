# 각 노드 토픽 정보 정리

이 문서는 `perception_cpp` 패키지 내의 각 노드가 사용하는 토픽 정보를 정리합니다.

## 0. 센서 데이터 처리 노드

### 0_1_odom_unity

(센서) Unity 환경에서 로봇의 위치 정보를 처리하고 변환하는 노드

- **구독 토픽**:
  - `/robots/fest_1/position` (`geometry_msgs/msg/Pose`): Unity에서의 로봇 위치
  - `/cmd_vel` (`geometry_msgs/msg/Twist`): 로봇 제어 명령

- **발행 토픽**:
  - `/odom` (`nav_msgs/msg/Odometry`): 변환된 오도메트리 정보

- **TF 변환**:
  - map → base_link
  - base_link → laser

### 0_2_lidar_filter

(센서) 라이다 데이터를 필터링하여 노이즈를 제거하는 노드

- **구독 토픽**:
  - `/laser` (`sensor_msgs/msg/LaserScan`): 원본 라이다 데이터

- **발행 토픽**:
  - `/laser_filtered` (`sensor_msgs/msg/LaserScan`): 필터링된 라이다 데이터

## 1. 맵 생성 관련 노드

### 1_1_cost_map_global

(맵) PGM 맵 파일을 읽어서 전역 비용 맵을 생성하는 노드

- **발행 토픽**:
  - `/map_file` (`nav_msgs/msg/OccupancyGrid`): 기본 맵 데이터
  - `/cost_map` (`nav_msgs/msg/OccupancyGrid`): 비용이 할당된 전역 맵

### 1_2_cost_map_local

(맵) 라이다 데이터를 기반으로 로컬 비용 맵을 생성하는 노드

- **구독 토픽**:
  - `/laser_filtered` (`sensor_msgs/msg/LaserScan`): 필터링된 라이다 데이터
  - `/odom` (`nav_msgs/msg/Odometry`): 로봇의 위치 정보

- **발행 토픽**:
  - `/local_cost_map` (`nav_msgs/msg/OccupancyGrid`): 로컬 비용 맵

### mapping

(맵) 라이다 데이터를 기반으로 환경 맵을 생성하는 SLAM 노드

- **구독 토픽**:
  - `/laser_filtered` (`sensor_msgs/msg/LaserScan`): 필터링된 라이다 데이터
  - `/odom` (`nav_msgs/msg/Odometry`): 로봇의 위치 정보

- **발행 토픽**:
  - `/map` (`nav_msgs/msg/OccupancyGrid`): 생성된 맵 데이터

### make_path

(맵) 로봇의 이동 경로를 기록하고 저장하는 노드

- **구독 토픽**:
  - `/odom` (`nav_msgs/msg/Odometry`): 로봇의 위치 정보

- **발행 토픽**:
  - `/global_path` (`nav_msgs/msg/Path`): 기록된 경로

## ROS2 메시지 타입 헤더 파일 정리

이 섹션에서는 프로젝트에서 사용하는 ROS2 메시지 타입 헤더 파일들을 분류하여 정리합니다.

### 표준 메시지 타입 헤더

ROS2에서 기본적으로 제공하는 표준 메시지 타입 헤더 파일들:

1. **geometry_msgs**: 로봇의 기하학적 데이터 표현 메시지
   - `geometry_msgs/msg/pose.hpp`: 3D 공간에서의 위치와 방향
   - `geometry_msgs/msg/twist.hpp`: 선속도와 각속도를 포함한 로봇 제어 명령

2. **nav_msgs**: 내비게이션 관련 메시지
   - `nav_msgs/msg/odometry.hpp`: 로봇의 위치와 속도 추정치
   - `nav_msgs/msg/path.hpp`: 일련의 위치로 구성된 경로
   - `nav_msgs/msg/occupancy_grid.hpp`: 2D 격자 맵 표현

3. **sensor_msgs**: 센서 데이터 관련 메시지
   - `sensor_msgs/msg/laser_scan.hpp`: 2D 라이다 스캔 데이터 