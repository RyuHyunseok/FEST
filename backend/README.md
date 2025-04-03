
# 화재 진압 로봇 프로젝트 아키텍처

## 🏗️ 시스템 구성요소

1. **화재 감지 시스템** (Unity 시뮬레이션):
    - 화재 발생 및 센서 감지 시뮬레이션
    - 화재 위치 및 정보를 MQTT로 중앙 관제 시스템에 전송
2. **중앙 관제 시스템** (FastAPI 백엔드):
    - 화재 정보 수신 및 분석
    - 로봇 상태 모니터링
    - 최적 로봇 선택 및 명령 하달(우리는 일단 로봇 하나만 정해져있음)
    - 데이터 저장 및 프론트엔드에 실시간 정보 제공
3. **로봇 제어 시스템** (ROS2):
    - 중앙 관제 시스템으로부터 명령 수신
    - 자율 주행 및 화재 진압 작업 실행
    - 로봇 상태 및 위치 정보 전송
4. **모니터링 시스템** (Vue.js 프론트엔드):
    - 화재 및 로봇 실시간 현황 시각화
    - 과거 데이터 조회 및 분석

## 📊 역할 분담

### 백엔드(중앙 관제 시스템)의 역할:

- 화재 발생 위치 정보 수집
- 가용 로봇 중 최적의 로봇 선택
- 선택된 로봇에게 "이 위치(x, y)로 이동해서 화재를 진압하라"는 높은 수준의 명령 전달
- 전체 상황 모니터링 및 기록

### 로봇(ROS 기반) 자체의 역할:

- 목표 위치로의 자율 주행 경로 계획 및 실행
- 장애물 회피 등 주행 중 실시간 판단
- 현장 도착 후 화재 정확한 위치 감지
- 소화기 분사 등 실제 화재 진압 작업 수행
- 작업 상태 보고

## 🔄 데이터 흐름 및 통신 방법

```
Unity(화재+센서) -> MQTT -> 백엔드(중앙관제) -> MQTT -> ROS2(로봇제어) -> Unity(로봇행동)
                     |                |
                     |                v
                     |           PostgreSQL/Redis(데이터저장)
                     |                |
                     v                v
                 프론트엔드 <------ WebSocket
                (모니터링)

```

### 1. 화재 감지 → 중앙 관제

- **통신 방법**: MQTT
- **데이터**: 화재 위치, 심각도, 감지 시간
- **흐름**: Unity 시뮬레이션 → MQTT 브로커 → 백엔드
- **토픽 예시**: `sensors/{sensor_id}/alarm`, `incidents/new`

### 2. 중앙 관제 → 로봇 제어

- **통신 방법**: MQTT
- **데이터**: 로봇 ID, 목표 위치, 작업 유형
- **흐름**: 백엔드 → MQTT 브로커 → ROS2
- **토픽 예시**: `robots/{robot_id}/command`

### 3. 로봇 → 중앙 관제

- **통신 방법**: MQTT
- **데이터**: 로봇 위치, 배터리 상태, 작업 상태
- **흐름**: ROS2 → MQTT 브로커 → 백엔드
- **토픽 예시**: `robots/{robot_id}/position`, `robots/{robot_id}/status`

### 4. 중앙 관제 → 모니터링

- **통신 방법**: WebSocket
- **데이터**: 로봇 상태, 화재 정보, 작업 현황
- **흐름**: 백엔드 → WebSocket → 프론트엔드
- **엔드포인트 예시**: `/ws/robots`, `/ws/incidents`

### 5. 데이터 저장 및 처리

- **실시간 데이터**: Redis (캐싱)
    - 로봇의 최신 위치를 `robot:{robot_id}:location` 형식으로 저장
    - 활성화된 화재 정보를 `incident:{incident_id}` 형식으로 저장
    - WebSocket으로 실시간 전송을 위한 데이터 준비
- **영구 저장**: PostgreSQL + PostGIS (이력 및 분석용)
    - `robot_positions` 테이블에 위치 및 시간 기록
    - `incidents` 테이블에 화재 정보 기록
    - `firefighting_missions` 테이블에 임무 정보 기록
    - PostGIS 확장을 통해 공간 데이터 쿼리 사용하여 경로 분석

### 6. 다수 로봇 데이터 처리

- 각 로봇은 고유 토픽으로 MQTT 브로커에 데이터 전송
    - 예: `robots/fest_1/position`, `robots/fest_2/position`
- FastAPI는 와일드카드 구독으로 모든 로봇 데이터 수신
    - 예: `robots/+/position`
- 로봇 ID별 데이터 처리 및 저장



## 💾 데이터베이스 구조

### 1. PostgreSQL + PostGIS (영구 저장)

- 로봇 정보, 화재 이력, 작업 기록 등 저장
- 로봇 경로 및 위치 데이터 공간 분석
- 통계 분석 및 보고서 생성에 활용

### 2. Redis (실시간 데이터)

- 현재 활성화된 화재 정보
- 로봇 실시간 위치 및 상태
- WebSocket을 통한 빠른 데이터 전달

## 🛠️ 기술 스택 및 역할

| 기술 | 역할 |
| --- | --- |
| **Unity + ROS2** | 로봇 시뮬레이션 및 센서 데이터 생성 |
| **FastAPI** | 백엔드 서버 (ROS2 데이터 수집 & WebSocket 데이터 전송) |
| **Redis** | 최신 로봇 위치 저장 (빠른 조회) |
| **PostgreSQL + PostGIS** | 과거 경로 저장 (위치 데이터 분석) |
| **MQTT** | 로봇 데이터 송수신 (메시지 브로커) |
| **Vue.js** | 프론트엔드 (실시간 위치 모니터링 & 경로 시각화) |

## 🔧 구현 계획

### 1️⃣ Unity + ROS2 시뮬레이션

- **Unity** 환경에서 로봇의 움직임과 화재 발생 구현
- **ROS2**를 통해 로봇 및 센서 데이터를 생성하여 서버로 전송

### 2️⃣ FastAPI 서버 구축

- **ROS2 데이터 수집 API** 개발
- **MQTT**를 통해 다수의 로봇 데이터 및 센서 데이터 수집
- **Redis**에 실시간 위치 및 화재 정보 저장
- **PostgreSQL**에 과거 위치 기록 및 화재 대응 분석

### 3️⃣ WebSocket을 이용한 실시간 데이터 전송

- **FastAPI**에서 WebSocket을 통해 **Vue.js** 프론트엔드로 실시간 로봇 위치 및 화재 정보 전송

### 4️⃣ Vue.js 프론트엔드 개발

- **WebSocket**을 통해 실시간 로봇 위치 및 화재 정보 수신
- **PostgreSQL** 데이터를 사용하여 로봇의 과거 경로 및 화재 대응 시각화

## 🔄 시스템 워크플로우

1. Unity에서 화재 발생 시뮬레이션 및 센서 감지
2. 감지 정보가 MQTT를 통해 백엔드로 전송
3. 백엔드에서 화재 정보 분석 및 가용 로봇 확인
4. 최적 로봇 선택 및 화재 진압 명령 발행
5. 로봇이 명령을 수신하고 자율적으로 화재 위치로 이동
6. 로봇이 화재 위치에 도착하면 자체적으로 화재 진압 작업 실행
7. 모든 과정이 실시간으로 프론트엔드에 표시





## 데이터베이스 구조

### PostgreSQL + PostGIS 테이블

```sql
-- 로봇 정보 테이블
CREATE TABLE robots (
    robot_id VARCHAR(50) PRIMARY KEY,
    name VARCHAR(100),
    battery INTEGER
);

-- 화재 사고 테이블
CREATE TABLE incidents (
    incident_id VARCHAR(50) PRIMARY KEY,
    location GEOMETRY(POINT, 4326),
    detected_at TIMESTAMP,
    extinguished_at TIMESTAMP,
    status VARCHAR(20) DEFAULT 'active' -- 'active', 'extinguished' # 이건 화재의 현재 상태
);

-- 로봇 위치 이력 테이블
CREATE TABLE robot_positions (
    id SERIAL PRIMARY KEY,
    robot_id VARCHAR(50) REFERENCES robots(robot_id),
    position GEOMETRY(POINT, 4326),
    orientation FLOAT,
    recorded_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);


CREATE TABLE firefighting_missions (
    mission_id VARCHAR(50) PRIMARY KEY,
    robot_id VARCHAR(50) REFERENCES robots(robot_id),
    incident_id VARCHAR(50) REFERENCES incidents(incident_id),
    status VARCHAR(20) DEFAULT 'assigned',  -- assigned, in_progress, completed, failed # 이건 로봇의 임무 수행 상태
    assigned_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    arrived_at TIMESTAMP NULL,
    completed_at TIMESTAMP NULL
);

```

### Redis 데이터 (실시간 캐싱)
- 로봇의 최신 위치: `robot:{robot_id}:position`
- 로봇의 최신 상태: `robot:{robot_id}:status`
- 활성화된 화재 정보: `incident:{incident_id}`

## MQTT 토픽 구조

### 로봇 관련 토픽
```
robots/{robot_id}/position     # 로봇 위치 정보 (x, y, orientation)
robots/{robot_id}/status       # 로봇 상태 정보 (배터리, 상태 등)
robots/{robot_id}/command      # 로봇에 보내는 명령 (이동, 진화 등)
```

### 화재 관련 토픽
```
incidents/new                  # 새 화재 발생 알림
incidents/{incident_id}/status # 화재 상태 업데이트
```

## MQTT 메시지 포맷

### 로봇 위치 메시지 (robots/{robot_id}/position)
```json
{
  "x": 15.3,
  "y": 22.1,
  "orientation": 45.2
}
```

### 로봇 상태 메시지 (robots/{robot_id}/status)
```json
{
  "battery": 75,
  "water": 100,
  "status": "idle"  // idle, moving, fighting_fire
}
```

### 로봇 명령 메시지 (robots/{robot_id}/command)
```json
{
    // "type": "move_to", // move_to, extinguish
    "target": {"x": 25.5, "y": 30.2},
    "mission_id": "mission_id",  // 미션 ID 포함
    "incident_id": "incident_id"  // 화재 ID 포함
}
```

### 화재 발생 메시지 (incidents/new)
```json
{
  "incident_id": "fire_123",
  "location": {"x": 25.5, "y": 30.2},
  "detected_at": 1646406000000
}
```

### 화재 상태 메시지 (incidents/{incident_id}/status)
```json
{
    "incident_id": "fire_id",
    "status": "extinguished",
    "extinguished_at": "current_time"
}
```

## REST API 구조

### 로봇 관련 API
- `GET /api/v1/robots` - 모든 로봇 목록 조회
- `GET /api/v1/robots/{robot_id}` - 특정 로봇 정보 조회
- `GET /api/v1/robots/{robot_id}/status` - 로봇 상태 조회
- `GET /api/v1/robots/{robot_id}/position` - 로봇 현재 위치 조회
<!-- - `POST /api/v1/robots/{robot_id}/command` - 로봇에 명령 전송 -->

### 화재 알람 관련 API
- `GET /api/v1/incidents` - 모든 화재 사고 목록
- `GET /api/v1/incidents/{incident_id}` - 특정 화재 사고 상세 정보
- `GET /api/v1/incidents/active` - 현재 진행 중인 화재 목록
<!-- - `POST /api/v1/incidents` - 새 화재 생성 (테스트용) -->

## WebSocket 엔드포인트
- `/ws/robots` - 모든 로봇 상태 업데이트 수신
- `/ws/incidents` - 화재 상태 업데이트 수신

# FastAPI 구조
```
backend/
├── main.py                     # FastAPI 애플리케이션 엔트리포인트
├── core/                       # 설정, 보안, 유틸성 모듈
│   ├── config.py               # 환경 변수 로드, 전역 설정  
│   └── security.py             # 인증, JWT 로직 등
├── db/                         # 데이터베이스 관련 코드
│   ├── database.py             # SQLAlchemy 설정, 연결 코드
│   └── base.py                 # 모델 베이스 클래스 및 모델 임포트
├── models/                     # SQLAlchemy 모델 정의
│   ├── robot.py                # 로봇 모델
│   ├── incident.py             # 화재 사고 모델
│   └── position.py             # 로봇 위치 기록 모델
├── schemas/                    # Pydantic 스키마
│   ├── robot.py                # 로봇 관련 스키마
│   └── incident.py             # 화재 사고 관련 스키마
├── api/
│   └── v1/                     # 버전별 API
│       ├── endpoints/          # 실제 라우트(엔드포인트)
│       │   ├── robots.py       # 로봇 관련 API
│       │   └── incidents.py    # 화재 사고 관련 API
│       ├── services/           # 비즈니스 로직 구현체
│       │   └── mqtt_service.py # MQTT 서비스 로직
│       └── routers/            # API 라우터
│           └── routers.py      # v1 라우터들을 등록하는 모듈
└── infra/                      # 인프라 관련 모듈
    ├── mqtt/
    │   └── mqtt_client.py      # MQTT 클라이언트 관련 코드
    └── websocket/
        └── websocket.py        # 웹소켓 관련 코드
```

# Vue 구조
```
src/
├── components/
│   ├── layout/
│   │   ├── Sidebar.vue         # 사이드바 컴포넌트
│   │   └── Topbar.vue          # 상단바 컴포넌트
│   ├── monitoring/
│   │   ├── RobotMap.vue        # 로봇 위치 맵 컴포넌트
│   │   ├── FireIncidentList.vue # 화재 사고 목록 컴포넌트
│   │   ├── RobotPathViewer.vue # 로봇 경로 시각화 컴포넌트
│   │   └── MissionTimer.vue    # 미션 시간 컴포넌트
│   └── common/
│       ├── StatusBadge.vue     # 상태 표시 배지 컴포넌트
│       └── AlertBox.vue        # 알림 박스 컴포넌트
├── services/
│   ├── api.js                  # API 호출 함수 모음
│   └── websocket.js            # WebSocket 연결 관리
├── views/
│   ├── Login.vue               # 로그인 화면
│   ├── Dashboard.vue           # 메인 대시보드 화면
│   ├── RobotDetail.vue         # 로봇 상세 화면
│   └── IncidentHistory.vue     # 화재 사고 이력 화면
└── store/
    ├── index.js                # Vuex 스토어
    ├── modules/
    │   ├── auth.js             # 인증 관련 상태 관리
    │   ├── robots.js           # 로봇 관련 상태 관리
    │   └── incidents.js        # 화재 사고 관련 상태 관리
```

### 프론트에 보여줄 기능

~~1. 로그인(관리자만 모니터링 가능하게)~~
~~2. 현재 로봇의 실시간 위치 표시~~
~~3. 화재 발생 시 경고 알림~~
~~4. 화재 발생 시 로봇에게 명령 전송~~
~~5. 화재 발생 위치 표시, 화재 상태 실시간 표시~~
~~6. 로봇 상태 조회(목록 조회)~~
~~7. 과거 화재 기록 조회~~
8. 화재 발생부터 진압 시 까지 로봇의 경로 표시
9. 시작 지점에서 화재 지점까지 가는데 걸린 시간 및 진압 시간 확인