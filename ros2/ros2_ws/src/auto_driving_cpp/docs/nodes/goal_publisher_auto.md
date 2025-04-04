# Automatic Goal Publisher

## 개요
파일에서 읽은 목표점을 자동으로 순차 발행하는 노드입니다.

## 주요 기능
- 파일에서 목표점 좌표 읽기
- 로봇 위치 모니터링
- 목표점 도달 감지 및 자동 다음 목표점 발행

## 토픽
### 구독
- `/odom` (nav_msgs/msg/Odometry): 로봇의 위치 정보

### 발행
- `/goal_point` (geometry_msgs/msg/Point): 목표점 정보

## 파일 입출력
### 입력 파일
- `path/goal_list.txt`: 목표점 좌표 리스트
  - 형식: "x y" (각 줄)
  - 단위: 미터

## 주요 매개변수
- GOAL_THRESHOLD: 2.0m (목표점 도달 판단 거리)
- 초기화 주기: 100ms 