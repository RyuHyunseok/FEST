# Global Cost Map Generator Node

## 개요
PGM 맵 파일을 읽어서 전역 비용 맵을 생성하는 노드입니다.

## 주요 기능
- PGM 맵 파일 로드
- 장애물 주변 비용 할당
- 비용 맵 생성 및 발행
- 비용 맵 저장

## 토픽
### 발행
- `/map_file` (nav_msgs/msg/OccupancyGrid): 기본 맵 데이터
- `/cost_map` (nav_msgs/msg/OccupancyGrid): 비용이 할당된 전역 맵

## 주요 매개변수
- inflation_radius: 2.0m
- cost_scaling_factor: 1.0
- map_resolution: 0.2m/pixel 