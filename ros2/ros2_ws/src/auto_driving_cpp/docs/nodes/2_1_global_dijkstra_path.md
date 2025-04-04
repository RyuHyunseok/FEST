# Global Dijkstra Path Planner

## 개요
다익스트라 알고리즘을 사용하여 전역 경로를 계획하는 노드입니다.

## 주요 기능
- 비용 맵 기반 최적 경로 생성
- 시작점과 목표점 간의 최단 경로 계산
- 장애물 회피를 위한 비용 패널티 적용
- 경로 계획 시각화 마커 제공

## 토픽
### 구독
- `/cost_map` (nav_msgs/msg/OccupancyGrid): 전역 비용 맵
- `/odom` (nav_msgs/msg/Odometry): 로봇의 현재 위치
- `/goal_point` (geometry_msgs/msg/Point): 목표점 좌표

### 발행
- `/global_path` (nav_msgs/msg/Path): 계획된 전역 경로
- `/planning_points` (visualization_msgs/msg/MarkerArray): 경로 계획 시각화

## 주요 매개변수
- 비용 패널티 계산 방식
  - 낮은 비용 (0-1): 0.1 패널티
  - 중간 비용: 비례적 패널티 계산
  - 높은 비용 (>=10): 통과 불가

## 알고리즘 상세
1. 다익스트라 노드 구조
   - 위치 (x, y)
   - 누적 비용
   - 부모 노드 포인터

2. 경로 계획 과정
   - 시작점에서 목표점까지 최소 비용 경로 탐색
   - 8방향 이웃 노드 탐색
   - 비용 맵 기반 이동 비용 계산
   - 휴리스틱 비용 추가로 탐색 효율성 향상 