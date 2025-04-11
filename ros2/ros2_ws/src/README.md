# ROS2 자율주행 시스템 패키지 설명서

## 실행 방법

### 통합 실행
모든 노드를 한 번에 실행하려면:
```bash
ros2 launch auto_driving_cpp integrated_launch.py
```

integrated_launch의 주석 해제 또는 아래 명령어 실행
(좌표 입력을 쉽게 하기 위해 따로 실행)
```bash
ros2 run auto_driving_cpp goal_publisher
```

### 개별 실행
각 패키지의 노드를 개별적으로 실행하려면:
```bash
ros2 launch <패키지_이름> <런치파일_이름>.py
```

## 1. auto_driving_cpp
자율주행 경로 계획 및 차량 제어를 담당하는 C++ 패키지

### 주요 노드
- `global_dijkstra_path`: 전역 경로 계획을 위한 다익스트라 알고리즘 구현
- `global_path_publisher`: 파일에서 읽은 경로를 주기적으로 발행
- `local_dijkstra_path`: 지역 경로 계획을 위한 다익스트라 알고리즘 구현
- `local_path_planner`: 격자 기반 후보 경로 생성 및 최적 경로 선택
- `follow_unity`: Unity 시뮬레이션 환경에서 차량 제어
- `goal_publisher`: 자동/수동 모드를 지원하는 목표점 발행 노드 (선택적적)
- `goal_publisher_auto`: 파일에서 읽은 목표점을 자동으로 순차 발행 (선택적)
- `goal_publisher_manual`: 수동 목표점 설정 노드 (선택적)

## 2. perception_cpp
환경 인식 및 센서 데이터 처리를 담당하는 C++ 패키지

### 주요 노드
- `odom_unity`: Unity 시뮬레이션의 차량 위치 정보 처리
- `lidar_filter`: LiDAR 센서 데이터 필터링
- `cost_map_global`: 전역 비용 지도 생성
- `cost_map_local`: 지역 비용 지도 생성

### 주요 디렉토리
- `map/`: 맵 데이터 저장
- `path/`: 경로 데이터 저장

## 3. common_utils
여러 패키지에서 공통으로 사용되는 유틸리티 기능을 제공하는 패키지

### 주요 기능
- 공통 데이터 구조
- 유틸리티 함수
- 공유 라이브러리

## 4. ros2_tcp_endpoint
Unity 시뮬레이션과 ROS2 시스템 간의 통신을 담당하는 패키지

### 주요 기능
- TCP 서버 엔드포인트 제공
- Unity-ROS2 메시지 변환
- 네트워크 통신 관리

### 설정
- IP: 0.0.0.0
- Port: 10000

## 5. topic_bridge_py
토픽 간 브리지 역할을 수행하는 Python 패키지

### 주요 기능
- 토픽 간 데이터 변환
- 메시지 포맷 변환
- 토픽 중계

## 의존성
- ROS2 Humble
- Python 3.8 이상
- C++17 (GCC 9.3 이상)
- Unity 2021.3 LTS 이상
- CMake 3.16 이상

## 시스템 구조
1. Unity 시뮬레이션에서 센서 데이터 수집 (ros2_tcp_endpoint)
   - 토픽: `/odom`, `/scan`
   - 데이터: 위치, 방향, 라이다 스캔

2. 센서 데이터 처리 및 환경 인식 (perception_cpp)
   - 토픽: `/cost_map_global`, `/cost_map_local`
   - 데이터: 비용 지도, 장애물 정보

3. 경로 계획 수립 (auto_driving_cpp)
   - 토픽: `/global_path`, `/local_path`
   - 데이터: 전역/지역 경로, 목표점

4. 차량 제어 명령 생성 (auto_driving_cpp)
   - 토픽: `/cmd_vel`
   - 데이터: 선속도, 각속도

5. Unity 시뮬레이션으로 제어 명령 전송 (ros2_tcp_endpoint)
   - 토픽: `/cmd_vel`
   - 데이터: 제어 명령 실행

## 주의사항
- Unity 시뮬레이션이 실행 중이어야 합니다
- TCP 엔드포인트는 다른 노드들보다 먼저 실행되어야 합니다
- 올바른 맵 데이터가 perception_cpp/map 디렉토리에 있어야 합니다
- 네트워크 설정이 올바르게 구성되어 있어야 합니다
- 시스템 메모리 사용량을 모니터링해야 합니다
- 실시간 성능을 위해 CPU 사용률을 관리해야 합니다
- 네트워크 지연이 발생하지 않도록 주의해야 합니다