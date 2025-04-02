import paho.mqtt.client as mqtt
import atexit

import os
from dotenv import load_dotenv

# .env 파일 로드
load_dotenv()

# MQTT 클라이언트 생성
mqtt_client = mqtt.Client()
is_connected = False

def connect_mqtt():
    """브로커에 연결하는 함수"""
    global is_connected
    try:
        # 환경 변수에서 호스트와 포트 가져오기, 기본값 설정
        mqtt_host = os.getenv('MQTT_HOST', 'localhost')
        mqtt_port = int(os.getenv('MQTT_PORT', 1883))
        
        mqtt_client.connect(mqtt_host, mqtt_port, 60)
        
        is_connected = True
        print("MQTT 브로커에 연결되었습니다.")
    except Exception as e:
        print(f"MQTT 브로커 연결 실패: {e}")
    return mqtt_client

def disconnect_mqtt():
    """브로커 연결을 종료하는 함수"""
    global is_connected
    if is_connected:
        print("MQTT 연결 종료 중...")
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        is_connected = False
        print("MQTT 연결이 종료되었습니다.")

# 프로그램 종료 시 MQTT 연결 종료
atexit.register(disconnect_mqtt)