-- 데이터베이스가 없으면 생성
CREATE DATABASE fest_db;

-- 데이터베이스에 연결
\c fest_db;

-- PostGIS 확장 활성화
CREATE EXTENSION IF NOT EXISTS postgis;
CREATE EXTENSION IF NOT EXISTS postgis_topology;

-- 사용자가 없으면 생성하고 권한 부여
CREATE USER ssafy WITH PASSWORD 'ssafy';
GRANT ALL PRIVILEGES ON DATABASE fest_db TO ssafy;
ALTER ROLE ssafy WITH LOGIN;

-- 스키마 생성
-- 로봇 정보 테이블
CREATE TABLE IF NOT EXISTS robots (
    robot_id VARCHAR(50) PRIMARY KEY,
    name VARCHAR(100),
    battery INTEGER
);

INSERT INTO robots (robot_id, name, battery) VALUES ('fest_1', 'fest_1', 100);

-- 화재 사고 테이블
CREATE TABLE IF NOT EXISTS incidents (
    incident_id VARCHAR(50) PRIMARY KEY,
    location GEOMETRY(POINT, 4326),
    detected_at TIMESTAMP,
    extinguished_at TIMESTAMP,
    status VARCHAR(20) DEFAULT 'active'
);

-- 로봇 위치 이력 테이블
CREATE TABLE IF NOT EXISTS robot_positions (
    id SERIAL PRIMARY KEY,
    robot_id VARCHAR(50) REFERENCES robots(robot_id),
    position GEOMETRY(POINT, 4326),
    orientation FLOAT,
    recorded_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- 화재 진압 미션 테이블
CREATE TABLE IF NOT EXISTS firefighting_missions (
    mission_id VARCHAR(50) PRIMARY KEY,
    robot_id VARCHAR(50) REFERENCES robots(robot_id),
    incident_id VARCHAR(50) REFERENCES incidents(incident_id),
    status VARCHAR(20) DEFAULT 'assigned',
    assigned_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    arrived_at TIMESTAMP NULL,
    completed_at TIMESTAMP NULL
);

-- 테이블에 대한 권한 부여
GRANT ALL PRIVILEGES ON ALL TABLES IN SCHEMA public TO ssafy;
GRANT USAGE, SELECT ON ALL SEQUENCES IN SCHEMA public TO ssafy;