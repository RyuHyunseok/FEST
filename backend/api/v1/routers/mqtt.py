from fastapi import APIRouter
from paho.mqtt.client import Client
from backend.core import mqtt_client # mqtt 설정 가져오기

router = APIRouter()

# ros2 에서 토픽을 발행하는걸 테스트하는 용
@router.post("/publish/")
async def publish_message(topic: str, message: str):
    """
    MQTT 메시지 발행 API
    """

    mqtt_client.mqtt_client.publish(topic, message)
    return {"message": f"Published `{message}` to `{topic}`"}

