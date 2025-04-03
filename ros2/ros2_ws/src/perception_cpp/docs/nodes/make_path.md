# Path Recording Node

## 개요
로봇의 이동 경로를 기록하고 저장하는 노드입니다.

## 주요 기능
- 실시간 경로 기록
- 웨이포인트 생성
- 경로 파일 저장
- 경로 발행

## 토픽
### 구독
- `/odom` (nav_msgs/msg/Odometry): 로봇의 위치 정보

### 발행
- `/global_path` (nav_msgs/msg/Path): 기록된 경로

## 주요 매개변수
- 웨이포인트 간격: 0.1m
- 파일 형식: txt (x, y 좌표) 