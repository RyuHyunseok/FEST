# LiDAR Filter Node

## 개요
라이다 데이터를 필터링하여 노이즈를 제거하고 데이터를 안정화하는 노드입니다.

## 주요 기능
- 자기 감지 제거 (로봇 자신의 감지 제거)
- 이동 평균 필터 적용
- 급격한 변화 필터링
- 데이터 안정화

## 토픽
### 구독
- `/laser` (sensor_msgs/msg/LaserScan): 원본 라이다 데이터

### 발행
- `/laser_filtered` (sensor_msgs/msg/LaserScan): 필터링된 라이다 데이터

## 주요 매개변수
- self_radius: 0.4m (로봇 반경)
- window_size: 3 (이동 평균 윈도우 크기)
- noise_threshold: 0.1m (노이즈 임계값) 