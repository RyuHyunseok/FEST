# Local Path Planner (Lattice)

## 개요
격자 기반 후보 경로를 생성하고 최적 경로를 선택하는 노드입니다.

## 주요 기능
- 격자 기반 후보 경로 생성
- 장애물 회피를 위한 경로 선택
- 라이다 데이터 기반 충돌 검사

## 토픽
### 구독
- `/global_path` (nav_msgs/msg/Path): 전역 경로
- `/odom` (nav_msgs/msg/Odometry): 로봇의 위치 정보
- `/scan` (sensor_msgs/msg/LaserScan): 라이다 스캔 데이터

### 발행
- `/local_path` (nav_msgs/msg/Path): 선택된 지역 경로
- `/candidate_paths` (nav_msgs/msg/Path): 생성된 후보 경로들

## 주요 매개변수
- local_path_size: 15 (지역 경로 포인트 수)
- robot_radius: 0.3m (로봇 반경)
- num_paths: 7 (생성할 후보 경로 수)
- max_offset: 0.5m (최대 측면 간격)

## 알고리즘 상세
1. 경로 생성 과정
   - 현재 위치에서 가장 가까운 전역 경로 지점 찾기
   - 측면 오프셋 기반 후보 경로 생성
   - 장애물 충돌 검사
   - 최적 경로 선택

2. 경로 선택 기준
   - 장애물과의 거리
   - 중앙 경로와의 거리
   - 이전 경로와의 연속성 