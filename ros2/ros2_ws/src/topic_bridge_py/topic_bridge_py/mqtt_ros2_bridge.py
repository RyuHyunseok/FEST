"""
MQTT 토픽과 ROS2 토픽 간의 변환 관계:

MQTT 토픽                    ->  ROS2 토픽
--------------------------------------------------------
robots/fest_1/command       ->  goal_point_mqtt (Point)
incidents/new               ->  goal_point_mqtt (Point)
"""

import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
from geometry_msgs.msg import Point
import json

class MqttToRos2Bridge(Node):
    def __init__(self):
        super().__init__('mqtt_to_ros2_bridge')
        
        # ROS2 퍼블리셔 생성
        self.incident_pub = self.create_publisher(
            Point,
            'goal_point_mqtt',
            10
        )
        
        # MQTT 클라이언트 설정
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        # MQTT 브로커 연결
        # self.mqtt_client.connect('j12d106.p.ssafy.io', 1883, 60)
        self.mqtt_client.connect('localhost', 1883, 60)
        self.mqtt_client.loop_start()

    def on_mqtt_connect(self, client, userdata, flags, rc):
        # 관련 MQTT 토픽 구독
        self.mqtt_client.subscribe([
            ("robots/fest_1/command", 0),
            # ("incidents/new", 0)
        ])
        self.get_logger().info('Connected to MQTT broker and subscribed to topics')

    def on_mqtt_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            topic = msg.topic
            
            # 로봇 명령 메시지 처리
            if topic == "robots/fest_1/command":
                location_msg = Point()
                location_msg.x = float(payload['target']['y'])
                location_msg.y = - float(payload['target']['x'])
                location_msg.z = 0.0  # z 좌표는 사용하지 않음
                self.incident_pub.publish(location_msg)
                self.get_logger().info(f'Published command target as goal point: {location_msg}')
            
            # 새로운 화재 발생 메시지 처리
            # elif topic == "incidents/new":
            #     # 화재 위치 발행
            #     location_msg = Point()
            #     location_msg.x = float(payload['location']['x'])
            #     location_msg.y = float(payload['location']['y'])
            #     self.incident_pub.publish(location_msg)
            #     self.get_logger().info(f'Published new incident: {location_msg}')
                
        except Exception as e:
            self.get_logger().error(f'Error processing message: {e}')

def main(args=None):
    rclpy.init(args=args)
    bridge = MqttToRos2Bridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 처리
        bridge.mqtt_client.loop_stop()
        bridge.mqtt_client.disconnect()
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
