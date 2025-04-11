# Global Path Publisher

## 개요
파일에서 읽은 경로를 주기적으로 발행하는 노드입니다.

## 주요 기능
- 파일에서 경로 좌표 읽기
- 주기적인 경로 발행
- 로봇 위치 모니터링
- 경로 데이터 유효성 검사

## 토픽
### 구독
- `/odom` (nav_msgs/msg/Odometry): 로봇의 위치 정보

### 발행
- `/global_path` (nav_msgs/msg/Path): 파일에서 읽은 전역 경로

## 파일 입출력
### 입력 파일
- `path/path.txt`: 전역 경로 좌표 데이터
  - 형식: "x y" (각 줄)
  - 단위: 미터
  - 좌표계: 맵 좌표계 (map frame)
  - 파일 경로: `perception_cpp/path/path.txt`

## 주요 매개변수
- 발행 주기: 20ms
- 경로 발행 간격: 10 사이클마다 1회
- 파일 읽기 재시도: 파일 열기 실패 시 자동 재시도 