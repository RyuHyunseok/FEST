# Perception CPP 패키지

자율주행 로봇을 위한 ROS2 C++ 인식 패키지입니다. 이 패키지는 센서 데이터 처리, 맵핑, 비용 맵 생성 등의 기능을 제공합니다.

## 런치 파일
```bash
# 인식 모드
call C:\dev\ros2-foxy\setup.bat
call C:\Users\SSAFY\Desktop\S12P21D106\ros2\ros2_ws\install\local_setup.bat
ros2 launch perception_cpp perception_launch.py

# 경로 생성 모드
ros2 launch perception_cpp make_path_launch.py

# 맵핑 모드
ros twice launch perception_cpp mapping_launch.py
```

## 패키지 구조

### 1. 센서 데이터 처리 노드
- **기본 센서 처리**
  - `0_1_odom_unity`: Unity 시뮬레이션의 위치 데이터를 Odometry로 변환
  - `0_2_lidar_filter`: LiDAR 데이터 필터링 및 전처리

### 2. 비용 맵 생성 노드
- **비용 맵 처리**
  - `1_1_cost_map_global`: 전역 비용 맵 생성 및 관리
  - `1_2_cost_map_local`: 실시간 지역 비용 맵 생성

### 3. 맵핑 및 경로 생성 노드
- `make_path`: 로봇 경로 기록 및 저장
- `mapping`: SLAM 기반 환경 맵 생성

## 주요 기능

### 1. 센서 데이터 처리
- Unity 시뮬레이션 데이터 변환
- LiDAR 데이터 필터링
- 자기 감지 제거
- 노이즈 필터링

### 2. 비용 맵 생성
- 전역 비용 맵: PGM 파일 기반 정적 맵 생성
- 지역 비용 맵: 실시간 장애물 감지
- 장애물 팽창 처리
- 동적 비용 할당

### 3. 맵핑
- SLAM 기반 환경 맵 생성
- 실시간 맵 업데이트
- PGM 파일 저장
- 경로 기록 및 저장

## 의존성

### ROS2 메시지
- geometry_msgs
- nav_msgs
- sensor_msgs
- tf2_ros

### 외부 라이브러리
- Eigen3
- common_utils

## 설정 파일

### 맵 파일
- `map/map.pgm`: 기본 맵 데이터
- `map/cost_map.pgm`: 비용 맵 데이터
- `map/map_save.pgm`: 생성된 맵 저장

### 경로 파일
- `path/test_save.txt`: 기록된 경로 데이터

## 실행 방법

1. **센서 데이터 처리**
```bash
# Unity Odometry 변환
ros2 run perception_cpp odom_unity

# LiDAR 필터링
ros2 run perception_cpp lidar_filter
```

2. **비용 맵 생성**
```bash
# 전역 비용 맵
ros2 run perception_cpp cost_map_global

# 지역 비용 맵
ros2 run perception_cpp cost_map_local
```

3. **맵핑 및 경로 생성**
```bash
# 경로 기록
ros2 run perception_cpp make_path

# 맵 생성
ros2 run perception_cpp mapping
```

## 토픽 구조

### 주요 입력 토픽
- `/robots/fest_1/position`: Unity 로봇 위치
- `/laser`: 원본 라이다 데이터
- `/odom`: 로봇 위치 정보

### 주요 출력 토픽
- `/laser_filtered`: 필터링된 라이다 데이터
- `/map`: 생성된 맵 데이터
- `/cost_map`: 전역 비용 맵
- `/local_cost_map`: 지역 비용 맵
- `/global_path`: 기록된 경로

## 개발 가이드라인

### 코드 스타일
- C++14 표준 준수
- ROS2 코딩 컨벤션 준수
- 클래스 기반 노드 구현

### 디버깅
- RCLCPP_INFO/DEBUG/WARN/ERROR 매크로 활용
- PGM 파일 저장 및 확인

## 주의사항
- 맵 크기 및 해상도 설정 확인
- 좌표계 변환 주의
- 파일 경로는 common_utils 사용
