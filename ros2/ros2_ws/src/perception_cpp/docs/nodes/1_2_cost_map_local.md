# Local Cost Map Generator Node

## 개요
라이다 데이터를 기반으로 로컬 비용 맵을 생성하는 노드입니다.

## 주요 기능
- 실시간 장애물 감지
- 동적 비용 맵 생성
- 로봇 주변 영역 모니터링
- 장애물 팽창 처리

## 토픽
### 구독
- `/laser_filtered` (sensor_msgs/msg/LaserScan): 필터링된 라이다 데이터
- `/odom` (nav_msgs/msg/Odometry): 로봇의 위치 정보

### 발행
- `/local_cost_map` (nav_msgs/msg/OccupancyGrid): 로컬 비용 맵

## 주요 매개변수
- resolution: 0.2m/pixel
- width: 200 cells (40m)
- height: 200 cells (40m)
- inflation_radius: 2.0m 