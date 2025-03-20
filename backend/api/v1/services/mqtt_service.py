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


# Redis 연결 설정
redis_client = redis.Redis(host='localhost', port=6379, db=0, decode_responses=True)

# 위치 데이터 샘플링을 위한 설정
POSITION_SAMPLE_INTERVAL = 10  # 10초마다 위치 데이터 저장
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
                
                # 참고: 상태 정보는 Redis에만 저장하고 PostgreSQL에는 저장하지 않음
                # 실시간 모니터링에만 사용하므로 영구 저장은 하지 않음
        
        # 화재 정보 처리 (topic: incidents/new)
        elif topic == "incidents/new":
            # Redis에 최신 화재 정보 저장
            incident_id = message.get("incident_id", f"fire_{uuid.uuid4().hex[:8]}")
            redis_client.set(f"incident:{incident_id}", json.dumps(message))
            
            # 데이터베이스에 화재 정보 저장
            create_new_incident(incident_id, message)

            # 일단 모든 로봇에게 명령
            # 나중에 최단 거리 로봇에게 명령하도록 구현
            robot_keys = redis_client.keys("robot:*:position")
            for key in robot_keys:
                parts = key.split(':')
                if len(parts) >= 3:
                    robot_id = parts[1]

                    # 로봇 메시지 생성
                    command = {
                        'type': 'move_to',
                        'target': message['location']
                    }

                    # MQTT로 로봇에게 명령 topic publish
                    _mqtt_client.publish(f'robots/{robot_id}/command', json.dumps(command))
                    print(f'화재 발생: 로봇 {robot_id}에게 화재 위치로 이동하라 명령')


        # 화재 상태 업데이트 처리 (topic: incidents/{incident_id}/status)
        elif topic.startswith("incidents/") and topic.endswith("/status"):
            # 토픽에서 화재 ID 추출 (형식: incidents/{incident_id}/status)
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
    """새로운 화재 사고를 데이터베이스에 저장"""
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
        
        db.commit()
        print(f"화재 상태 업데이트: {incident}")
        return True
        
    except Exception as e:
        print(f"Error updating incident in database: {e}")
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
    _mqtt_client.subscribe("incidents/new")       # 새 화재 발생 정보
    _mqtt_client.subscribe("incidents/+/status") # 모든 화재 상태 정보

    _mqtt_client.loop_start()  # 비동기 구독 시작
    print("MQTT 리스너가 시작되었습니다.")

# MQTT 클라이언트 종료
def stop_mqtt_listener():
    global _mqtt_client
    if _mqtt_client:
        print("MQTT 리스너 종료 중...")
        _mqtt_client.loop_stop()
        print("MQTT 리스너가 종료되었습니다.")