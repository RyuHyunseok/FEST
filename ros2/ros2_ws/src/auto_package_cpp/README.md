# 각 노드 토픽 정보 정리

이 문서는 `auto_package_cpp` 패키지 내의 각 노드가 사용하는 토픽 정보를 정리합니다.

## 1. 오돔 및 맵 관련 노드

### 1_odom_node

(인지) 터틀봇의 정보를 가져와서 변환후 다시 발행하는 노드

- **구독 토픽**:
  - `/turtlebot_status` (`ssafy_msgs/msg/TurtlebotStatus`): 터틀봇의 상태 정보

- **발행 토픽**:
  - `odom` (`nav_msgs/msg/Odometry`): 로봇의 위치 및 속도 정보

### 1_global_cost_map_generator

(인지) 저장된 맵 데이터를 가져와서 global 코스트 맵을 작성후 발행하는 노드

- **구독 토픽**: 없음

- **발행 토픽**:
  - `/map` (`nav_msgs/msg/OccupancyGrid`): 글로벌 지도 정보
  - `/cost_map` (`nav_msgs/msg/OccupancyGrid`): 글로벌 비용 지도 정보

- **파일 입출력**:
  - 읽기: `path/map.pgm` - 맵 파일 로드
  - 쓰기: `path/cost_map.pgm` - 생성된 비용 지도 저장

### 1_local_cost_map_generator

(인지) 라이다 데이터를 가져와서 장애물을 반영한 local 코스트 맵을 작성후 발행하는 노드

- **구독 토픽**:
  - `/scan` (`sensor_msgs/msg/LaserScan`): 라이다 스캔 데이터
  - `/odom` (`nav_msgs/msg/Odometry`): 로봇의 위치 및 속도 정보

- **발행 토픽**:
  - `/local_cost_map` (`nav_msgs/msg/OccupancyGrid`): 로컬 비용 지도 정보

## 2. 경로 계획 관련 노드

### 2_global_path_publisher

(판단) 저장된 경로 파일을 가져와서 global 경로를 발행하는 노드

- **구독 토픽**:
  - `/odom` (`nav_msgs/msg/Odometry`): 로봇의 위치 및 속도 정보

- **발행 토픽**:
  - `global_path` (`nav_msgs/msg/Path`): 미리 정의된 전역 경로

- **파일 입출력**:
  - 읽기: `path/test.txt` - 미리 정의된 경로 포인트 로드

### 2_global_dijkstra_path

(판단) global 코스트 맵에서 현재 위치에서 목표점까지의 최단 경로를 global 경로로 발행하는 노드

- **구독 토픽**:
  - `/cost_map` (`nav_msgs/msg/OccupancyGrid`): 글로벌 지도 정보
  - `/goal_point` (`geometry_msgs/msg/Point`): 목표 지점 정보
  - `/odom` (`nav_msgs/msg/Odometry`): 로봇의 위치 및 속도 정보

- **발행 토픽**:
  - `/global_path` (`nav_msgs/msg/Path`): 다익스트라 알고리즘으로 계산된 전역 경로
  - `/planning_points` (`visualization_msgs/msg/MarkerArray`): 경로 계획 시각화 포인트

### 2_local_path_planner

(판단) 현재 위치에서 가장 가까운 global 경로를 local 경로로 만들어서 발행하는 노드 (장애물 회피를 위한 예비 경로 작성 내용도 포함)

- **구독 토픽**:
  - `global_path` (`nav_msgs/msg/Path`): 전역 경로 정보
  - `/odom` (`nav_msgs/msg/Odometry`): 로봇의 위치 및 속도 정보
  - `/scan` (`sensor_msgs/msg/LaserScan`): 라이다 스캔 데이터

- **발행 토픽**:
  - `local_path` (`nav_msgs/msg/Path`): 로컬 경로 정보
  - `candidate_paths` (`nav_msgs/msg/Path`): 후보 경로 정보

### 2_local_dijkstra_path

(판단) global 경로 에서 장애물을 감지하면 local 코스트 맵의 정보로 장애물을 회피하는 local 경로를 발행하는 노드

- **구독 토픽**:
  - `global_path` (`nav_msgs/msg/Path`): 전역 경로 정보
  - `/local_cost_map` (`nav_msgs/msg/OccupancyGrid`): 로컬 비용 지도 정보
  - `/odom` (`nav_msgs/msg/Odometry`): 로봇의 위치 및 속도 정보

- **발행 토픽**:
  - `local_path` (`nav_msgs/msg/Path`): 다익스트라 알고리즘으로 계산된 로컬 경로

## 3. 경로 추종 및 제어 노드

### 3_follow_the_carrot

(제어) local 경로를 따라가도록 로봇의 제어를 발행하는 노드

- **구독 토픽**:
  - `/odom` (`nav_msgs/msg/Odometry`): 로봇의 위치 및 속도 정보
  - `/turtlebot_status` (`ssafy_msgs/msg/TurtlebotStatus`): 터틀봇 상태 정보
  - `local_path` (`nav_msgs/msg/Path`): 로컬 경로 정보

- **발행 토픽**:
  - `cmd_vel` (`geometry_msgs/msg/Twist`): 로봇 제어 명령(속도, 회전)
  - `robot_state` (`std_msgs/msg/Int32`): 로봇 상태 정보

## 기타 노드

### goal_publisher

목표점을 토픽으로 발행하기 위한 노드

- **구독 토픽**:
  - `robot_state` (`std_msgs/msg/Int32`): 로봇 상태 정보

- **발행 토픽**:
  - `/goal_point` (`geometry_msgs/msg/Point`): 목표 지점 정보

- **파일 입출력**:
  - 읽기: `path/hoom2_path.txt` - 목표 지점 경로 파일 로드

### mapping

라이다 데이터를 직접 움직여서 pgm파일로 맵 데이터를 저장하는 노드

- **구독 토픽**:
  - `/scan` (`sensor_msgs/msg/LaserScan`): 라이다 스캔 데이터
  - `/odom` (`nav_msgs/msg/Odometry`): 로봇의 위치 및 속도 정보

- **발행 토픽**:
  - `/map` (`nav_msgs/msg/OccupancyGrid`): 맵핑을 통해 생성된 지도 정보

- **파일 입출력**:
  - 쓰기: `path/map.pgm` - 생성된 맵 저장

### make_path

로봇을 직접 움직인 경로를 txt파일로 저장하는 노드

- **구독 토픽**:
  - `/odom` (`nav_msgs/msg/Odometry`): 로봇의 위치 및 속도 정보

- **발행 토픽**:
  - `global_path` (`nav_msgs/msg/Path`): 생성된 전역 경로

- **파일 입출력**:
  - 쓰기: `path/test_save.txt` - 생성된 경로 포인트 저장

## 파일 경로 유틸리티

패키지 내의 노드들은 플랫폼 독립적인 파일 경로를 사용하기 위해 공통 유틸리티 함수를 사용합니다:

```cpp
// file_path.cpp
std::string create_file_path(const std::string& package_name, const std::string& relative_path)
```

이 함수는 패키지 이름과 상대 경로를 입력으로 받아 운영체제에 맞는 절대 경로를 반환합니다. 

## ROS2 메시지 타입 헤더 파일 정리

이 섹션에서는 프로젝트에서 사용하는 ROS2 메시지 타입 헤더 파일들을 분류하여 정리합니다.

### 표준 메시지 타입 헤더

ROS2에서 기본적으로 제공하는 표준 메시지 타입 헤더 파일들:

1. **geometry_msgs**: 로봇의 기하학적 데이터 표현 메시지
   - `geometry_msgs/msg/twist.hpp`: 선속도와 각속도를 포함한 로봇 제어 명령
   - `geometry_msgs/msg/point.hpp`: 3D 공간의 점 좌표(x, y, z)
   - `geometry_msgs/msg/pose_stamped.hpp`: 타임스탬프가 포함된 위치와 방향 정보
   - `geometry_msgs/msg/transform_stamped.hpp`: 타임스탬프가 포함된 좌표 변환 정보

2. **nav_msgs**: 내비게이션 관련 메시지
   - `nav_msgs/msg/odometry.hpp`: 로봇의 위치와 속도 추정치
   - `nav_msgs/msg/path.hpp`: 일련의 위치로 구성된 경로
   - `nav_msgs/msg/occupancy_grid.hpp`: 2D 격자 맵 표현

3. **sensor_msgs**: 센서 데이터 관련 메시지
   - `sensor_msgs/msg/laser_scan.hpp`: 2D 라이다 스캔 데이터

4. **std_msgs**: 기본 데이터 타입 메시지
   - `std_msgs/msg/int32.hpp`: 32비트 정수값

5. **visualization_msgs**: 시각화 관련 메시지
   - `visualization_msgs/msg/marker_array.hpp`: 시각화 마커들의 배열

6. **builtin_interfaces**: 내장 인터페이스 메시지
   - `builtin_interfaces/msg/time.hpp`: 시간 표현

### 사용자 정의 메시지 타입 헤더

프로젝트에서 별도로 정의하여 사용하는 메시지 타입 헤더 파일들:

1. **ssafy_msgs**: SSAFY 프로젝트를 위해 정의된 메시지
   - `ssafy_msgs/msg/turtlebot_status.hpp`: 터틀봇의 상태 정보를 담는 메시지

### 메시지 타입 사용 방법

메시지 타입 헤더 파일은 다음과 같이 노드에서 사용됩니다:

1. **헤더 파일 포함**: 노드 소스 코드 상단에 필요한 메시지 타입 헤더 파일을 포함
   ```cpp
   #include "geometry_msgs/msg/twist.hpp"
   #include "nav_msgs/msg/path.hpp"
   ```

2. **퍼블리셔/서브스크라이버 생성**: 메시지 타입을 템플릿 매개변수로 지정
   ```cpp
   // Publisher 예시
   cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
   
   // Subscriber 예시
   odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
       "/odom", 10, 
       std::bind(&ClassName::callback_function, this, std::placeholders::_1));
   ```

3. **메시지 송수신**: 해당 타입의 메시지 객체를 생성하여 데이터 전송
   ```cpp
   // 메시지 발행 예시
   auto msg = geometry_msgs::msg::Twist();
   msg.linear.x = 0.5;  // 선속도 설정
   msg.angular.z = 0.2; // 각속도 설정
   cmd_pub_->publish(msg);
   ```

## ssafy_msgs 메시지 정의 및 사용 상세

이 섹션에서는 프로젝트에서 정의하고 사용하는 사용자 정의 메시지 패키지인 `ssafy_msgs`의 세부 내용을 설명합니다.

### TurtlebotStatus 메시지

`ssafy_msgs/msg/TurtlebotStatus`는 터틀봇 로봇의 현재 상태 정보를 담고 있는 메시지 타입입니다.

#### 메시지 구조

TurtlebotStatus 메시지는 다음과 같은 주요 필드들을 포함합니다:

```
# TurtlebotStatus.msg 예상 구조
geometry_msgs/Twist twist      # 로봇의 속도 및 위치 정보
  linear:                      # 선형 움직임 관련 값
    x: float64                 # 전진/후진 선속도
    y: float64                 # 측면 선속도 (사용 안 함)
    z: float64                 # 로봇 회전 각도(도)
  angular:                     # 회전 움직임 관련 값
    x: float64                 # 로봇의 절대 x 좌표
    y: float64                 # 로봇의 절대 y 좌표
    z: float64                 # 로봇의 회전 각속도
# 그 외 추가 상태 정보가 있을 수 있음
```

#### 사용 예시

1. **1_odom_node.cpp에서의 사용**:
   - 터틀봇의 상태 정보를 구독하여 오도메트리 메시지로 변환
   - 로봇의 절대 위치(x, y)와 방향 정보를 추출하여 사용
   ```cpp
   void status_callback(const ssafy_msgs::msg::TurtlebotStatus::SharedPtr msg) {
     // 로봇의 절대 x, y 좌표 사용
     x_ = msg->twist.angular.x - x1_;  // 로봇의 절대 x 좌표
     y_ = msg->twist.angular.y - y1_;  // 로봇의 절대 y 좌표
     
     // 로봇의 방향(각도) 정보 사용 - 라디안으로 변환
     theta_ = msg->twist.linear.z * (M_PI / 180.0);
     
     // 이 정보로 오도메트리 메시지 및 TF 변환 업데이트
   }
   ```

2. **3_follow_the_carrot.cpp에서의 사용**:
   - 로봇의 현재 상태 정보를 받아 경로 추종 알고리즘에 활용
   - 경로 추종 시 로봇의 속도 제어와 상태 관리에 필요한 정보로 사용
   ```cpp
   void status_callback(const ssafy_msgs::msg::TurtlebotStatus::SharedPtr msg) {
     is_status_ = true;
     status_msg_ = *msg;
     // 이 정보를 바탕으로 로봇의 상태를 판단하고 제어 명령 생성
   }
   ```

#### 활용 방법

TurtlebotStatus 메시지는 다음과 같은 목적으로 활용됩니다:

1. **로봇 위치 추적**: 
   - 로봇의 절대 좌표와 방향 정보를 이용하여 현재 위치 추적
   - 오도메트리 메시지와 TF(Transform) 생성의 기준 데이터로 활용

2. **로봇 상태 관리**:
   - 로봇의 현재 주행 상태(정지, 주행 중, 도착 등) 판단
   - 목적지 도달 여부 확인 및 다음 목표점 설정에 활용

3. **속도 제어**:
   - 경로 추종 시 로봇의 선속도와 각속도 결정에 참고
   - 안전한 주행을 위한 상태 정보로 활용 