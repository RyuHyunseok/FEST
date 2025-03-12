import json
import redis
from core.mqtt_client import mqtt_client, connect_mqtt  # mqtt_client import
from paho.mqtt.client import Client

# Redis 연결 설정
redis_client = redis.Redis(host='localhost', port=6379, db=0, decode_responses=True)

# MQTT 메시지 수신 콜백 함수
def on_message(client, userdata, msg):
    try:
        message = json.loads(msg.payload.decode())  # JSON 데이터 변환
        topic = msg.topic

        print(f"Received message on topic {topic}: {message}")

        # Redis에 저장 (예: 최신 위치 데이터를 저장)
        redis_client.set("latest_position", json.dumps(message))

    except Exception as e:
        print(f"Error processing message: {e}")

# MQTT 클라이언트 설정 및 시작
def start_mqtt_listener():
    mqtt_client = connect_mqtt()  # MQTT 연결 설정
    mqtt_client.on_message = on_message  # 메시지 수신 시 호출될 콜백 함수 설정

    # 메시지 구독
    mqtt_client.subscribe("robot/position")  # ROS2에서 publish한 topic

    mqtt_client.loop_start()  # 비동기 구독 시작
