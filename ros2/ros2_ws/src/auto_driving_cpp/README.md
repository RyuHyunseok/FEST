# Auto Driving CPP 패키지

자율주행 로봇을 위한 ROS2 C++ 패키지입니다. 이 패키지는 경로 계획, 장애물 회피, 로봇 제어 등의 기능을 제공합니다.

## 런치 파일 (파일 경로는 필요에 따라 변경)
```bash
call C:\dev\ros2-foxy\setup.bat
call C:\Users\SSAFY\Desktop\S12P21D106\ros2\ros2_ws\install\local_setup.bat
ros2 launch auto_driving_cpp unity_dijkstra_launch.py
```

## 패키지 구조

### 1. 경로 계획 노드
- **Global Path Planning**
  - `2_1_global_dijkstra_path`: 다익스트라 알고리즘 기반 전역 경로 계획
  - `2_1_global_path_publisher`: 저장된 경로 파일 기반 전역 경로 발행

- **Local Path Planning**
  - `2_2_local_dijkstra_path`: 장애물 회피를 위한 지역 경로 계획
  - `2_2_local_path_planner`: 격자 기반 지역 경로 생성

### 2. 로봇 제어 노드
- `3_follow_unity`: Unity 환경에서의 로봇 제어

### 3. 목표점 관리 노드
- `goal_publisher`: 하이브리드 목표점 관리 (자동/수동)
- `goal_publisher_auto`: 자동 목표점 순차 발행
- `goal_publisher_manual`: 수동 목표점 입력 및 발행

## 주요 기능

### 1. 경로 계획
- 전역 경로 계획: 비용 맵 기반 최적 경로 생성
- 지역 경로 계획: 실시간 장애물 회피
- 격자 기반 경로 생성: 다중 후보 경로 생성 및 평가

### 2. 로봇 제어
- Unity 환경 적응형 제어
- 목표점 추적 및 도달 판단
- 안전한 속도 및 방향 제어

### 3. 목표점 관리
- 파일 기반 자동 경로 추종
- 수동 목표점 입력 지원
- 하이브리드 모드 운영

## 의존성

### ROS2 메시지
- geometry_msgs
- nav_msgs
- sensor_msgs
- visualization_msgs
- std_msgs

### 외부 라이브러리
- tf2
- common_utils (자체 개발 유틸리티)

## 설정 파일

### 경로 파일
- `path/path.txt`: 전역 경로 데이터
- `path/goal_list.txt`: 목표점 리스트
- `path/map.pgm`: 환경 맵 데이터

## 실행 방법

1. **전역 경로 계획**
```bash
# 다익스트라 기반 경로 계획
ros2 run auto_driving_cpp global_dijkstra_path

# 파일 기반 경로 발행
ros2 run auto_driving_cpp global_path_publisher
```

2. **지역 경로 계획**
```bash
# 장애물 회피 경로 계획
ros2 run auto_driving_cpp local_dijkstra_path

# 격자 기반 경로 생성
ros2 run auto_driving_cpp local_path_planner
```

3. **로봇 제어**
```bash
ros2 run auto_driving_cpp follow_unity
```

4. **목표점 관리**
```bash
# 하이브리드 모드
ros2 run auto_driving_cpp goal_publisher

# 자동 모드
ros2 run auto_driving_cpp goal_publisher_auto

# 수동 모드
ros2 run auto_driving_cpp goal_publisher_manual
```

## 토픽 구조

### 주요 입력 토픽
- `/cost_map`: 전역 비용 맵
- `/local_cost_map`: 지역 비용 맵
- `/odom`: 로봇 위치 정보
- `/scan`: 라이다 데이터

### 주요 출력 토픽
- `/global_path`: 전역 경로
- `/local_path`: 지역 경로
- `/cmd_vel`: 로봇 제어 명령
- `/goal_point`: 목표점 정보

## 개발 가이드라인

### 코드 스타일
- C++14 표준 준수
- ROS2 코딩 컨벤션 준수
- 클래스 기반 노드 구현

### 디버깅
- RCLCPP_INFO/DEBUG/WARN/ERROR 매크로 활용
- 시각화 도구 (RViz2) 지원

## 주의사항
- Unity 환경에서의 좌표계 변환 주의
- 파일 경로는 common_utils 사용
- 메모리 관리를 위한 스마트 포인터 활용 