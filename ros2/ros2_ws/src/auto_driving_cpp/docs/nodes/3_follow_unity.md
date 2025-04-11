# Unity Robot Controller

## 개요
Unity 환경에서 로봇을 제어하는 노드입니다.

## 주요 기능
- 지역 경로 추종
- Unity 좌표계 변환 처리
- 목표점 도달 확인
- 속도 및 방향 제어
- 부드러운 가속/감속 제어
- 장애물 회피를 위한 속도 조절
- 실시간 경로 모니터링

## 토픽
### 구독
- `/local_path` (nav_msgs/msg/Path): 지역 경로
- `/odom` (nav_msgs/msg/Odometry): 로봇의 위치 정보
- `/goal_point` (geometry_msgs/msg/Point): 목표점 정보

### 발행
- `/cmd_vel` (geometry_msgs/msg/Twist): 로봇 제어 명령
- `/goal_reached` (std_msgs/msg/Bool): 목표 도달 상태

## 주요 매개변수
- linear_speed: 3.0 m/s (최대 선속도)
- max_angular_speed: 90.0 deg/s (최대 각속도)
- goal_tolerance: 2.0m (목표 도달 판단 거리)
- 제어 주기: 50ms
- 가속/감속 계수: 0.1
- 회전 반경: 1.0m
- 장애물 감지 거리: 2.0m
- 긴급 정지 거리: 1.0m

## 제어 알고리즘
1. 경로 추종
   - 현재 위치에서 목표점까지의 거리 계산
   - 목표 방향 계산
   - 선속도 및 각속도 결정
   - 부드러운 가속/감속 적용
   - 회전 반경 고려한 속도 조절

2. Unity 좌표계 변환
   - 각도 변환 (라디안 ↔ 도)
   - 좌표계 회전 보정
   - 좌표계 스케일 조정

3. 안전 제어
   - 장애물 감지 및 대응
   - 긴급 정지 상황 처리
   - 속도 제한 적용
   - 경로 이탈 감지

4. 성능 최적화
   - 효율적인 제어 주기 관리
   - 메모리 사용 최적화
   - 실시간 모니터링 