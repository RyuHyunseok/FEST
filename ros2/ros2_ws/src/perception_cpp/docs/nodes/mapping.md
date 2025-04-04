# SLAM Mapping Node

## 개요
라이다 데이터를 기반으로 환경 맵을 생성하는 SLAM 노드입니다.

## 주요 기능
- 라이다 기반 맵핑
- 로봇 위치 추적
- 맵 업데이트 및 저장
- PGM 파일 형식 지원

## 토픽
### 구독
- `/laser_filtered` (sensor_msgs/msg/LaserScan): 필터링된 라이다 데이터
- `/odom` (nav_msgs/msg/Odometry): 로봇의 위치 정보

### 발행
- `/map` (nav_msgs/msg/OccupancyGrid): 생성된 맵 데이터

## 주요 매개변수
- MAP_RESOLUTION: 0.2m/pixel
- MAP_WIDTH: 120.0m
- MAP_HEIGHT: 120.0m
- OCCUPANCY_UP: 0.1
- OCCUPANCY_DOWN: 0.01 