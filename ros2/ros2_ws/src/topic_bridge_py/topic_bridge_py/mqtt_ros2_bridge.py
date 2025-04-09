"""
MQTT 토픽과 ROS2 토픽 간의 변환 관계:

MQTT 토픽                    ->  ROS2 토픽
--------------------------------------------------------
robots/fest_1/position      ->  robot/position (Pose2D)
robots/fest_1/status        ->  robot/battery (Float32)
                             ->  robot/status (String)
robots/fest_1/command       ->  goal_point (Point)
incidents/new               ->  goal_point (Point)
incidents/+/status         ->  incidents/status (String)
missions/+/update          ->  missions/status (String)
"""

import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
from geometry_msgs.msg import Pose2D, Point  # 로봇 위치용, 화재 위치용
from std_msgs.msg import Float32, String  # 배터리, 상태용
from diagnostic_msgs.msg import DiagnosticStatus  # 로봇 상태용
import json
from datetime import datetime

class MqttToRos2Bridge(Node):
    def __init__(self):
        super().__init__('mqtt_to_ros2_bridge')
        
        # ROS2 퍼블리셔 생성
        self.position_pub = self.create_publisher(
            Pose2D,
            'robot/position',
            10
        )
        
        self.battery_pub = self.create_publisher(
            Float32,
            'robot/battery',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            'robot/status',
            10
        )
        
        self.incident_pub = self.create_publisher(
            Point,
            # 'incidents/new',
            'goal_point_mqtt',
            10
        )
        
        self.incident_status_pub = self.create_publisher(
            String,
            'incidents/status',
            10
        )
        
        self.mission_status_pub = self.create_publisher(
            String,
            'missions/status',
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
        # 모든 관련 MQTT 토픽 구독
        self.mqtt_client.subscribe([
            ("robots/fest_1/position", 0),
            ("robots/fest_1/status", 0),
            ("robots/fest_1/command", 0),
            ("incidents/new", 0),
            ("incidents/+/status", 0),
            ("missions/+/update", 0)
        ])
        self.get_logger().info('Connected to MQTT broker and subscribed to topics')

    def on_mqtt_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            topic = msg.topic
            
            # 로봇 위치 메시지 처리
            if topic == "robots/fest_1/position":
                ros_msg = Pose2D()
                ros_msg.x = float(payload['x'])
                ros_msg.y = float(payload['y'])
                ros_msg.theta = float(payload['orientation'])
                self.position_pub.publish(ros_msg)
                self.get_logger().info(f'Published robot position: {ros_msg}')
            
            # 로봇 명령 메시지 처리
            elif topic == "robots/fest_1/command":
                location_msg = Point()
                location_msg.x = float(payload['target']['y'])
                location_msg.y = - float(payload['target']['x'])
                location_msg.z = 0.0  # z 좌표는 사용하지 않음
                self.incident_pub.publish(location_msg)
                self.get_logger().info(f'Published command target as goal point: {location_msg}')
            
            # 로봇 상태 메시지 처리
            elif topic == "robots/fest_1/status":
                # 배터리 정보 발행
                battery_msg = Float32()
                battery_msg.data = float(payload['battery'])
                self.battery_pub.publish(battery_msg)
                
                # 상태 정보 발행
                status_msg = String()
                status_msg.data = payload['status']
                self.status_pub.publish(status_msg)
                
                self.get_logger().info(f'Published robot status: battery={battery_msg.data}, status={status_msg.data}')
            
            # 새로운 화재 발생 메시지 처리
            elif topic == "incidents/new":
                # 화재 위치 발행
                location_msg = Point()
                location_msg.x = float(payload['location']['x'])
                location_msg.y = float(payload['location']['y'])
                self.incident_pub.publish(location_msg)
                
                self.get_logger().info(f'Published new incident: {location_msg}')
            
            # 화재 상태 업데이트 메시지 처리
            elif "incidents" in topic and "status" in topic:
                status_msg = String()
                status_msg.data = json.dumps(payload)
                self.incident_status_pub.publish(status_msg)
                
                self.get_logger().info(f'Published incident status update: {status_msg.data}')
            
            # 미션 상태 업데이트 메시지 처리
            elif "missions" in topic and "update" in topic:
                status_msg = String()
                status_msg.data = json.dumps(payload)
                self.mission_status_pub.publish(status_msg)
                
                self.get_logger().info(f'Published mission status update: {status_msg.data}')
                
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
