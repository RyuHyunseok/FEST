# Unity Robot Controller

## 개요
Unity 환경에서 로봇을 제어하는 노드입니다.

## 주요 기능
- 지역 경로 추종
- Unity 좌표계 변환 처리
- 목표점 도달 확인
- 속도 및 방향 제어

## 토픽
### 구독
- `/local_path` (nav_msgs/msg/Path): 지역 경로
- `/odom` (nav_msgs/msg/Odometry): 로봇의 위치 정보
- `/goal_point` (geometry_msgs/msg/Point): 목표점 정보

### 발행
- `/cmd_vel` (geometry_msgs/msg/Twist): 로봇 제어 명령
- `/goal_reached` (std_msgs/msg/Bool): 목표 도달 상태

## 주요 매개변수
- linear_speed: 3.0 m/s
- max_angular_speed: 90.0 deg/s
- goal_tolerance: 2.0m
- 제어 주기: 50ms

## 제어 알고리즘
1. 경로 추종
   - 현재 위치에서 목표점까지의 거리 계산
   - 목표 방향 계산
   - 선속도 및 각속도 결정

2. Unity 좌표계 변환
   - 각도 변환 (라디안 ↔ 도)
   - 좌표계 회전 보정 