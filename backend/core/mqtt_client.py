import paho.mqtt.client as mqtt

# MQTT 클라이언트 생성
mqtt_client = mqtt.Client()

def connect_mqtt():
    """브로커에 연결하는 함수"""
    mqtt_client.connect("localhost", 1883, 60)
    return mqtt_client
