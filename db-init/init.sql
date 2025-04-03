-- 🚀 1️⃣ `postgres` 슈퍼유저로 실행
-- `fest_db` 데이터베이스가 없으면 생성
DO
$$
BEGIN
    IF NOT EXISTS (SELECT FROM pg_database WHERE datname = 'fest_db') THEN
        CREATE DATABASE fest_db;
    END IF;
END
$$;

-- 🚀 2️⃣ `fest_db`에 접속하여 PostGIS 확장 설치
\c fest_db postgres;

CREATE EXTENSION IF NOT EXISTS postgis;
CREATE EXTENSION IF NOT EXISTS postgis_topology;

-- 🚀 3️⃣ `ssafy` 유저 생성 (이미 존재하면 스킵)
DO
$$
BEGIN
    IF NOT EXISTS (SELECT FROM pg_roles WHERE rolname = 'ssafy') THEN
        CREATE USER ssafy WITH PASSWORD 'ssafy';
    END IF;
END
$$;

-- 🚀 4️⃣ `ssafy` 유저에게 권한 부여
GRANT CONNECT ON DATABASE fest_db TO ssafy;
GRANT USAGE, CREATE ON SCHEMA public TO ssafy;
GRANT ALL PRIVILEGES ON ALL TABLES IN SCHEMA public TO ssafy;
GRANT USAGE, SELECT ON ALL SEQUENCES IN SCHEMA public TO ssafy;

-- PostGIS 관련 권한도 부여
GRANT ALL ON geometry_columns TO ssafy;
GRANT ALL ON spatial_ref_sys TO ssafy;

-- 🚀 5️⃣ `ssafy` 계정으로 전환 (이후 모든 작업은 `ssafy`로 진행)
\c fest_db ssafy;

-- 🚀 6️⃣ 테이블 및 데이터 생성
CREATE TABLE IF NOT EXISTS robots (
    robot_id VARCHAR(50) PRIMARY KEY,
    name VARCHAR(100),
    battery INTEGER
);

INSERT INTO robots (robot_id, name, battery) VALUES ('fest_1', 'fest_1', 100);

CREATE TABLE IF NOT EXISTS incidents (
    incident_id VARCHAR(50) PRIMARY KEY,
    location GEOMETRY(POINT, 4326),
    detected_at TIMESTAMP,
    extinguished_at TIMESTAMP,
    status VARCHAR(20) DEFAULT 'active'
);

CREATE TABLE IF NOT EXISTS robot_positions (
    id SERIAL PRIMARY KEY,
    robot_id VARCHAR(50) REFERENCES robots(robot_id),
    position GEOMETRY(POINT, 4326),
    orientation FLOAT,
    recorded_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE TABLE IF NOT EXISTS firefighting_missions (
    mission_id VARCHAR(50) PRIMARY KEY,
    robot_id VARCHAR(50) REFERENCES robots(robot_id),
    incident_id VARCHAR(50) REFERENCES incidents(incident_id),
    status VARCHAR(20) DEFAULT 'assigned',
    assigned_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    arrived_at TIMESTAMP NULL,
    completed_at TIMESTAMP NULL
);
