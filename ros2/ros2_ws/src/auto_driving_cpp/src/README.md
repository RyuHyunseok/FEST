# 각 노드 토픽 정보 정리

이 문서는 `auto_driving_cpp` 패키지 내의 각 노드가 사용하는 토픽 정보를 정리합니다.

## 2. 경로 계획 관련 노드

### 2_1_global_dijkstra_path

(판단) 다익스트라 알고리즘을 사용하여 전역 경로를 계획하는 노드

- **구독 토픽**:
  - `/cost_map` (`nav_msgs/msg/OccupancyGrid`): 전역 비용 맵
  - `/odom` (`nav_msgs/msg/Odometry`): 로봇의 현재 위치
  - `/goal_point` (`geometry_msgs/msg/Point`): 목표점 좌표

- **발행 토픽**:
  - `/global_path` (`nav_msgs/msg/Path`): 계획된 전역 경로
  - `/planning_points` (`visualization_msgs/msg/MarkerArray`): 경로 계획 시각화

### 2_1_global_path_publisher

(판단) 파일에서 읽은 경로를 주기적으로 발행하는 노드

- **구독 토픽**:
  - `/odom` (`nav_msgs/msg/Odometry`): 로봇의 위치 정보

- **발행 토픽**:
  - `/global_path` (`nav_msgs/msg/Path`): 파일에서 읽은 전역 경로

### 2_2_local_dijkstra_path

(판단) 장애물 회피를 위한 지역 경로를 계획하는 노드

- **구독 토픽**:
  - `/global_path` (`nav_msgs/msg/Path`): 전역 경로
  - `/local_cost_map` (`nav_msgs/msg/OccupancyGrid`): 지역 비용 맵
  - `/odom` (`nav_msgs/msg/Odometry`): 로봇의 위치 정보

- **발행 토픽**:
  - `/local_path` (`nav_msgs/msg/Path`): 계획된 지역 경로

### 2_2_local_path_planner

(판단) 격자 기반 후보 경로를 생성하고 최적 경로를 선택하는 노드

- **구독 토픽**:
  - `/global_path` (`nav_msgs/msg/Path`): 전역 경로
  - `/odom` (`nav_msgs/msg/Odometry`): 로봇의 위치 정보
  - `/scan` (`sensor_msgs/msg/LaserScan`): 라이다 스캔 데이터

- **발행 토픽**:
  - `/local_path` (`nav_msgs/msg/Path`): 선택된 지역 경로
  - `/candidate_paths` (`nav_msgs/msg/Path`): 생성된 후보 경로들

### 3_follow_unity

(제어) Unity 환경에서 로봇을 제어하는 노드

- **구독 토픽**:
  - `/local_path` (`nav_msgs/msg/Path`): 지역 경로
  - `/odom` (`nav_msgs/msg/Odometry`): 로봇의 위치 정보
  - `/goal_point` (`geometry_msgs/msg/Point`): 목표점 정보

- **발행 토픽**:
  - `/cmd_vel` (`geometry_msgs/msg/Twist`): 로봇 제어 명령
  - `/goal_reached` (`std_msgs/msg/Bool`): 목표 도달 상태

### goal_publisher_auto

(제어) 파일에서 읽은 목표점을 자동으로 순차 발행하는 노드

- **구독 토픽**:
  - `/odom` (`nav_msgs/msg/Odometry`): 로봇의 위치 정보

- **발행 토픽**:
  - `/goal_point` (`geometry_msgs/msg/Point`): 목표점 정보

### goal_publisher_manual

(제어) 사용자 입력을 통해 수동으로 목표점을 발행하는 노드

- **발행 토픽**:
  - `/goal_point` (`geometry_msgs/msg/Point`): 목표점 정보

### goal_publisher

(제어) 자동/수동 모드를 지원하는 하이브리드 목표점 발행 노드

- **구독 토픽**:
  - `/odom` (`nav_msgs/msg/Odometry`): 로봇의 위치 정보

- **발행 토픽**:
  - `/goal_point` (`geometry_msgs/msg/Point`): 목표점 정보

## ROS2 메시지 타입 헤더 파일 정리

이 섹션에서는 프로젝트에서 사용하는 ROS2 메시지 타입 헤더 파일들을 분류하여 정리합니다.

### 표준 메시지 타입 헤더

ROS2에서 기본적으로 제공하는 표준 메시지 타입 헤더 파일들:

1. **geometry_msgs**: 로봇의 기하학적 데이터 표현 메시지
   - `geometry_msgs/msg/twist.hpp`: 선속도와 각속도를 포함한 로봇 제어 명령
   - `geometry_msgs/msg/point.hpp`: 3D 공간의 점 좌표(x, y, z)

2. **nav_msgs**: 내비게이션 관련 메시지
   - `nav_msgs/msg/odometry.hpp`: 로봇의 위치와 속도 추정치
   - `nav_msgs/msg/path.hpp`: 일련의 위치로 구성된 경로
   - `nav_msgs/msg/occupancy_grid.hpp`: 2D 격자 맵 표현

3. **sensor_msgs**: 센서 데이터 관련 메시지
   - `sensor_msgs/msg/laser_scan.hpp`: 2D 라이다 스캔 데이터

4. **std_msgs**: 기본 데이터 타입 메시지
   - `std_msgs/msg/bool.hpp`: 불리언 값

5. **visualization_msgs**: 시각화 관련 메시지
   - `visualization_msgs/msg/marker_array.hpp`: 시각화 마커들의 배열 