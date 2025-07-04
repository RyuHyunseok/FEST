# Local Dijkstra Path Planner

## 개요
장애물 회피를 위한 지역 경로를 계획하는 노드입니다.

## 주요 기능
- 전역 경로 기반 지역 경로 생성
- 실시간 장애물 회피 경로 계획
- 다익스트라 알고리즘 기반 우회 경로 생성
- 동적 장애물 대응
- 경로 재계획 최적화

## 토픽
### 구독
- `/global_path` (nav_msgs/msg/Path): 전역 경로
- `/local_cost_map` (nav_msgs/msg/OccupancyGrid): 지역 비용 맵
- `/odom` (nav_msgs/msg/Odometry): 로봇의 위치 정보

### 발행
- `/local_path` (nav_msgs/msg/Path): 계획된 지역 경로

## 주요 매개변수
- lookahead_distance: 10.0m (전방 고려 거리)
- collision_threshold: 50 (충돌 판단 임계값)
- 경로 계획 주기: 100ms
- 재계획 대기 시간: 2.0초
- 최대 반복 횟수: 1,000,000회

## 알고리즘 상세
1. 경로 계획 과정
   - 현재 위치에서 가장 가까운 전역 경로 지점 찾기
   - 전방 거리 내 경로점 추출
   - 장애물 충돌 검사
   - 필요시 우회 경로 계획
   - 경로 부드러움 최적화

2. 충돌 회피 전략
   - 그리드 기반 경로 탐색
   - 비용 맵 기반 이동 비용 계산
   - 8방향 이웃 노드 탐색
   - 휴리스틱 비용 적용
   - 재계획 제한 조건 적용

3. 성능 최적화
   - 방문 노드 추적
   - 비용 맵 캐싱
   - 효율적인 경로 재구성 