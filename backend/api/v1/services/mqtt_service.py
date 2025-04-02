import json
import redis
import uuid
import time
from infra.mqtt.mqtt_client import mqtt_client, connect_mqtt  # mqtt_client import
from paho.mqtt.client import Client
from db.database import SessionLocal
from models.position import RobotPosition
from models.robot import Robot
from models.incident import Incident
from sqlalchemy import func
from geoalchemy2.functions import ST_MakePoint
import datetime

from models.mission import FirefightingMission


# Redis 연결 설정
redis_client = redis.Redis(host='localhost', port=6379, db=0, decode_responses=True)

# 위치 데이터 샘플링을 위한 설정
POSITION_SAMPLE_INTERVAL = 1  # 1초마다 위치 데이터 저장
last_position_save = {}        # 로봇별 마지막 저장 시간 추적용 딕셔너리

# 위치 데이터를 샘플링할지 결정하는 함수
def should_sample_position(robot_id):
    current_time = time.time()
    last_save_time = last_position_save.get(robot_id, 0)
    
    # 마지막 저장 후 일정 시간(POSITION_SAMPLE_INTERVAL)이 지났는지 확인
    if current_time - last_save_time >= POSITION_SAMPLE_INTERVAL:
        last_position_save[robot_id] = current_time
        return True
    
    return False

# MQTT 메시지 수신 콜백 함수
def on_message(client, userdata, msg):
    try:
        message = json.loads(msg.payload.decode())  # JSON 데이터를 dictionary 객체로 변환
        topic = msg.topic

        # print(f"Received message on topic {topic}: {message}")

        # 위치 정보 처리 (topic: robots/{robot_id}/position)
        if "position" in topic:
            # 토픽에서 로봇 ID 추출 (형식: robots/{robot_id}/position)
            parts = topic.split('/')
            if len(parts) >= 3:
                robot_id = parts[1]
                
                # Redis에 최신 위치 저장 (로봇별)
                redis_client.set(f"robot:{robot_id}:position", json.dumps(message))
                
                # 데이터베이스에 위치 기록 저장 (샘플링 적용)
                # 성능을 위해 모든 데이터를 저장하지 않고 일부만 저장
                should_save = should_sample_position(robot_id)
                if should_save:
                    save_position_to_db(robot_id, message)
        
        # 상태 정보 처리 (topic: robots/{robot_id}/status)
        elif topic.startswith("robots/") and topic.endswith("/status"):
            # 토픽에서 로봇 ID 추출 (형식: robots/{robot_id}/status)
            parts = topic.split('/')
            if len(parts) >= 3:
                robot_id = parts[1]
                
                # Redis에 최신 상태 저장 (로봇별)
                redis_client.set(f"robot:{robot_id}:status", json.dumps(message))
                print('redis에 저장ㅁㅎㅁ')
                # 참고: 상태 정보는 Redis에만 저장하고 PostgreSQL에는 저장하지 않음
                # 실시간 모니터링에만 사용하므로 영구 저장은 하지 않음
        
        # 화재 정보 처리 (topic: incidents/new)
        elif topic == "incidents/new":
            # Redis에 최신 화재 정보 저장
            incident_id = message.get("incident_id", f"fire_{uuid.uuid4().hex[:8]}")
            redis_client.set(f"incident:{incident_id}", json.dumps(message))
            
            # 데이터베이스에 화재 정보 저장
            create_new_incident(incident_id, message)


        # 화재 상태 업데이트 처리 (topic: incidents/{incident_id}/status)
        elif topic.startswith("incidents/") and topic.endswith("/status"):
            # 토픽에서 화재 ID 추출 (형식: incidents/{incident_id}/status)

            # 토픽 메시지 정보
            """
            update_data = {
                'incident_id': fire_id,
                'status': 'extinguished',
                'extinguished_at': current_time
                }
            """

            parts = topic.split('/')
            if len(parts) >= 3:
                incident_id = parts[1]
                
                # incident_id가 message에 없다면 추가해줍니다
                if "incident_id" not in message:
                    message["incident_id"] = incident_id
                
                # Redis에 최신 상태 저장
                redis_client.set(f"incident:{incident_id}", json.dumps(message))
                
                # 데이터베이스에 상태 업데이트 저장
                update_incident_status(incident_id, message)

        # 미션 상태 업데이트 처리 (topic: mission/{mission_id}/update)
        elif topic.startswith("mission/") and topic.endswith("/update"):
            # 토픽에서 미션 ID 추출 (형식: mission/{mission_id}/update)
            parts = topic.split('/')
            if len(parts) >= 3:
                mission_id = parts[1]
                
                # 데이터베이스에 미션 상태 업데이트
                update_mission_status(mission_id, message)

        elif topic == "prowler/new":
            # redis에 침입자 정보 저장
            prowler_id = message.get("prowler_id", f"prowler_{uuid.uuid4().hex[:8]}")
            redis_client.set(f"prowler:{prowler_id}", json.dumps(message), ex=30) # 30초 후 자동 삭제

    except Exception as e:
        print(f"Error processing message: {e}")



def save_position_to_db(robot_id, position_data):
    """로봇 위치 데이터를 데이터베이스에 저장"""
    try:
        db = SessionLocal()
        
        # 먼저 로봇이 존재하는지 확인
        robot = db.query(Robot).filter(Robot.robot_id == robot_id).first()
        if not robot:
            # 로봇이 없으면 새로 생성
            robot = Robot(robot_id=robot_id, name=f"Robot {robot_id}")
            db.add(robot)
            db.commit()
        
        # 위치 데이터 생성
        x = position_data.get("x", 0)
        y = position_data.get("y", 0)
        orientation = position_data.get("orientation", 0)
        
        # PostGIS 포인트 생성 (ST_MakePoint)
        point = func.ST_SetSRID(ST_MakePoint(x, y), 4326)
        
        # 로봇 위치 저장 (x, y는 position에 포함, orientation은 별도 저장)
        position = RobotPosition(
            robot_id=robot_id,
            position=point,
            orientation=orientation
        )
        
        db.add(position)
        db.commit()
        print(f"Saved position data to database: {position}")
        
    except Exception as e:
        print(f"Error saving position to database: {e}")
    finally:
        db.close()

def create_new_incident(incident_id, incident_data):
    """새로운 화재 사고를 데이터베이스에 저장하고, 로봇에게 미션 할당"""
    try:
        db = SessionLocal()
        
        # 위치 데이터 추출
        location = incident_data.get("location", {})
        x = location.get("x", 0)
        y = location.get("y", 0)
        
        # PostGIS 포인트 생성
        point = func.ST_SetSRID(ST_MakePoint(x, y), 4326)
        
        # 새 화재 사고 생성
        incident = Incident(
            incident_id=incident_id,
            location=point,
            status=incident_data.get("status", "active")
        )
        db.add(incident)
        db.commit()
        
        # 가용 로봇 찾기 (가장 가까운 로봇)
        # 실제 구현에서는 좀 더 복잡한 알고리즘을 사용할 수 있음
        # 여기서는 단순히 위치 정보가 있는 모든 로봇에게 미션 부여
        robot_keys = redis_client.keys("robot:*:position")
        
        for key in robot_keys:
            parts = key.split(':')
            if len(parts) >= 3:
                robot_id = parts[1]
                
                # 로봇 위치 정보 가져오기
                position_data = redis_client.get(key)
                if position_data:
                    # 미션 ID 생성
                    mission_id = f"mission_{uuid.uuid4().hex[:8]}"
                    
                    # 미션 생성
                    from models.mission import FirefightingMission
                    mission = FirefightingMission(
                        mission_id=mission_id,
                        robot_id=robot_id,
                        incident_id=incident_id,
                        status="assigned"
                    )
                    db.add(mission)
                    
                    # 로봇 메시지 생성
                    command = {
                        'type': 'move_to',
                        'target': location,
                        'mission_id': mission_id,  # 미션 ID 포함
                        'incident_id': incident_id  # 화재 ID 포함
                    }
                    
                    # MQTT로 로봇에게 명령 publish
                    _mqtt_client.publish(f'robots/{robot_id}/command', json.dumps(command))
                    print(f'화재 발생: 로봇 {robot_id}에게 미션 {mission_id} 할당')

                    # PostgreSQL 로봇 상태 업데이트 (기존 코드 유지)
                    robot = db.query(Robot).filter(Robot.robot_id == robot_id).first()
                    if robot:
                        robot.status = "on_mission"

                    # Redis에 별도 키로 미션 상태 저장
                    redis_client.set(f"robot:{robot_id}:mission_status", "on_mission")
                    print(f"Mission status set to on_mission for robot {robot_id}")


        db.commit()
        print(f"새로운 화재 db 저장완료: {incident}")
        return True
        
    except Exception as e:
        print(f"Error creating incident in database: {e}")
        return False
    finally:
        db.close()

def update_incident_status(incident_id, status_data):
    """화재 사고 상태를 업데이트"""
    try:
        db = SessionLocal()
        
        # 기존 화재 사고 조회
        incident = db.query(Incident).filter(Incident.incident_id == incident_id).first()
        
        if not incident:
            print(f"Cannot update incident status: Incident {incident_id} not found")
            return False
        
        # 상태 업데이트
        if "status" in status_data:
            incident.status = status_data["status"]
        
        # extinguished_at 처리
        if "extinguished_at" in status_data:
            # 타임스탬프를 datetime으로 변환
            try:
                extinguished_ts = status_data["extinguished_at"]
                # 밀리초 타임스탬프라면 초 단위로 변환
                if extinguished_ts > 1000000000000:  # 밀리초 타임스탬프 체크
                    extinguished_ts = extinguished_ts / 1000
                incident.extinguished_at = datetime.datetime.fromtimestamp(extinguished_ts)
            except Exception as e:
                print(f"Error converting extinguished_at: {e}")
        
        # 화재가 진압된 경우(status = extinguished), 관련 미션도 완료로 처리
        if "status" in status_data and status_data["status"] == "extinguished":
            # 해당 incident_id에 연결된 미션 조회
            from models.mission import FirefightingMission
            mission = db.query(FirefightingMission).filter(
                FirefightingMission.incident_id == incident_id,
                FirefightingMission.status != "completed"  # 아직 완료되지 않은 미션만
            ).first()
            
            if mission:
                # 미션 상태 업데이트
                mission.status = "completed"
                mission.completed_at = incident.extinguished_at or datetime.datetime.now()
                
                # PostgreSQL 로봇 상태 업데이트
                robot = db.query(Robot).filter(Robot.robot_id == mission.robot_id).first()
                if robot:
                    robot.status = "idle"
                
                # Redis에 별도 키로 미션 상태 저장
                robot_id = mission.robot_id
                redis_client.set(f"robot:{robot_id}:mission_status", "idle")
                print(f"Mission status set to idle for robot {robot_id}")
        
        db.commit()
        print(f"화재 상태 업데이트: {incident}")
        return True
        
    except Exception as e:
        print(f"Error updating incident in database: {e}")
        return False
    finally:
        db.close()

def update_mission_status(mission_id, status_data):
    """화재 진압 미션 상태를 업데이트"""
    try:
        db = SessionLocal()
        
        # 미션 모델 임포트
        from models.mission import FirefightingMission
        
        # 기존 미션 조회
        mission = db.query(FirefightingMission).filter(FirefightingMission.mission_id == mission_id).first()
        
        if not mission:
            print(f"Cannot update mission status: Mission {mission_id} not found")
            return False
        
        # 상태 업데이트
        if "status" in status_data:
            mission.status = status_data["status"]
        
        # arrived_at 처리
        if "arrived_at" in status_data:
            # 타임스탬프를 datetime으로 변환
            try:
                arrived_ts = status_data["arrived_at"]
                # 밀리초 타임스탬프라면 초 단위로 변환
                if arrived_ts > 1000000000000:  # 밀리초 타임스탬프 체크
                    arrived_ts = arrived_ts / 1000
                mission.arrived_at = datetime.datetime.fromtimestamp(arrived_ts)
            except Exception as e:
                print(f"Error converting arrived_at: {e}")
        
        # completed_at 처리
        if "completed_at" in status_data:
            # 타임스탬프를 datetime으로 변환
            try:
                completed_ts = status_data["completed_at"]
                # 밀리초 타임스탬프라면 초 단위로 변환
                if completed_ts > 1000000000000:  # 밀리초 타임스탬프 체크
                    completed_ts = completed_ts / 1000
                mission.completed_at = datetime.datetime.fromtimestamp(completed_ts)
            except Exception as e:
                print(f"Error converting completed_at: {e}")
        
        db.commit()
        print(f"미션 상태 업데이트: {mission}")
        return True
        
    except Exception as e:
        print(f"Error updating mission in database: {e}")
        return False
    finally:
        db.close()

# MQTT 클라이언트 참조
_mqtt_client = None

# MQTT 클라이언트 설정 및 시작
def start_mqtt_listener():
    global _mqtt_client
    _mqtt_client = connect_mqtt()  # MQTT 연결 설정
    _mqtt_client.on_message = on_message  # 메시지 수신 시 호출될 콜백 함수 설정

    # 메시지 구독
    _mqtt_client.subscribe("robots/+/position")  # 모든 로봇 위치 정보 (+ 는 와일드카드)
    _mqtt_client.subscribe("robots/+/status")    # 모든 로봇 상태 정보
    _mqtt_client.subscribe("incidents/new")      # 새 화재 발생 정보
    _mqtt_client.subscribe("incidents/+/status") # 모든 화재 상태 정보
    _mqtt_client.subscribe("mission/+/update")   # 모든 미션 상태 업데이트
    _mqtt_client.subscribe("prowler/new")        # 침입자 발생 정보보

    _mqtt_client.loop_start()  # 비동기 구독 시작
    print("MQTT 리스너가 시작되었습니다.")

# MQTT 클라이언트 종료
def stop_mqtt_listener():
    global _mqtt_client
    if _mqtt_client:
        print("MQTT 리스너 종료 중...")
        _mqtt_client.loop_stop()
        print("MQTT 리스너가 종료되었습니다.")