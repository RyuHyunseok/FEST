import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import random
import math
import json

class SmoothPositionPublisher(Node):
    def __init__(self):
        super().__init__('smooth_position_publisher')

        # MQTT 설정
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect('localhost', 1883, 60)  # MQTT 브로커 연결
        self.mqtt_topic = "robot/position"  # 퍼블리시할 MQTT 토픽

        # 로봇의 초기 위치 및 이동 속도 설정
        self.x = random.uniform(0, 5)
        self.y = random.uniform(0, 5)
        self.orientation = random.uniform(0, 360)  # 초기 방향 (도)
        self.speed = 0.1  # 이동 속도 (한 번에 0.1m 이동)

        self.timer = self.create_timer(0.5, self.publish_position)  # 0.5초마다 발행

    def publish_position(self):
        # 현재 방향(각도)을 라디안으로 변환
        radian = math.radians(self.orientation)

        # 방향을 따라 조금씩 이동
        self.x += self.speed * math.cos(radian)
        self.y += self.speed * math.sin(radian)

        # 이동 중 약간 방향 변경 (5도 이내로 랜덤 회전)
        self.orientation += random.uniform(-5, 5)
        self.orientation = self.orientation % 360  # 0~360도 유지

        # MQTT 메시지 생성
        position_data = {
            "x": round(self.x, 2),
            "y": round(self.y, 2),
            "orientation": round(self.orientation, 2)
        }

        # MQTT 메시지 발행
        self.mqtt_client.publish(self.mqtt_topic, json.dumps(position_data))
        self.get_logger().info(f'Publishing to MQTT: {position_data}')

def main(args=None):
    rclpy.init(args=args)
    node = SmoothPositionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()