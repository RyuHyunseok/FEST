# Unity Odometry Node

## 개요
Unity 환경에서 로봇의 위치 정보를 처리하고 변환하는 노드입니다.

## 주요 기능
- Unity 좌표계를 ROS2 좌표계로 변환
- 로봇의 위치와 방향 정보 처리
- TF 브로드캐스팅
- 속도 명령 처리

## 토픽
### 구독
- `/robots/fest_1/position` (geometry_msgs/msg/Pose): Unity에서의 로봇 위치
- `/cmd_vel` (geometry_msgs/msg/Twist): 로봇 제어 명령

### 발행
- `/odom` (nav_msgs/msg/Odometry): 변환된 오도메트리 정보

## TF 변환
- map → base_link
- base_link → laser 