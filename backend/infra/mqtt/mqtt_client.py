import paho.mqtt.client as mqtt
import atexit

# MQTT 클라이언트 생성
mqtt_client = mqtt.Client()
is_connected = False

def connect_mqtt():
    """브로커에 연결하는 함수"""
    global is_connected
    try:
        mqtt_client.connect("localhost", 1883, 60)
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