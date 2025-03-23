#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import paho.mqtt.client as mqtt
from geometry_msgs.msg import Pose
import math

class ROS2ToMQTTBridge(Node):
    def __init__(self):
        super().__init__('ros2_to_mqtt_bridge')
        
        # 로봇 ID 설정
        self.robot_id = "fest_1"
        self.get_logger().info(f"Starting ROS2 to MQTT bridge for robot: {self.robot_id}")
        
        # MQTT 설정
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.position_topic = f"robots/{self.robot_id}/position"
        
        # MQTT 브로커 연결
        try:
            self.mqtt_client.connect('localhost', 1883, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info("Connected to MQTT broker")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT broker: {e}")
        
        # ROS2 토픽 구독 설정 (Pose 타입)
        self.subscription = self.create_subscription(
            Pose,
            f'/robots/{self.robot_id}/position',
            self.position_callback,
            10  # QoS 프로파일
        )
        
        # 타이머 설정 (MQTT 연결 상태 체크)
        self.timer = self.create_timer(5.0, self.check_connection)
    
    def on_connect(self, client, userdata, flags, rc):
        """MQTT 연결 성공 시 호출될 콜백"""
        self.get_logger().info(f"Connected to MQTT broker with result code: {rc}")
    
    def check_connection(self):
        """정기적으로 MQTT 연결 상태 확인"""
        if self.mqtt_client.is_connected():
            self.get_logger().debug("MQTT connection is active")
        else:
            self.get_logger().warn("MQTT connection lost. Attempting to reconnect...")
            try:
                self.mqtt_client.reconnect()
            except Exception as e:
                self.get_logger().error(f"Failed to reconnect: {e}")
    
    def quaternion_to_euler_yaw(self, x, y, z, w):
        """쿼터니언에서 오일러 각의 Yaw(Z축 회전)만 추출"""
        # 쿼터니언에서 yaw 추출 (Z축 회전)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # 라디안을 도(degree)로 변환
        return math.degrees(yaw)
    
    def position_callback(self, msg):
        """ROS2 위치 토픽 메시지 수신 시 호출되는 콜백"""
        try:
            # Pose 타입 메시지에서 필요한 정보 추출
            pos_x = msg.position.x
            pos_y = msg.position.y
            
            # 쿼터니언을 yaw 각도로 변환
            orientation = self.quaternion_to_euler_yaw(
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            )
            
            # MQTT 메시지 포맷 (기존 MQTT 메시지 형식에 맞춤)
            position_data = {
                "x": round(float(pos_x), 2),
                "y": round(float(pos_y), 2),
                "orientation": round(float(orientation), 2)
            }
            
            # JSON으로 변환하여 MQTT로 발행
            mqtt_msg = json.dumps(position_data)
            self.mqtt_client.publish(self.position_topic, mqtt_msg)
            
            self.get_logger().info(f'Published position to MQTT: {position_data}')
        except Exception as e:
            self.get_logger().error(f'Error processing position data: {e}')
    
    def on_shutdown(self):
        """노드 종료 시 호출될 메소드"""
        self.get_logger().info("Shutting down ROS2 to MQTT bridge...")
        if self.mqtt_client.is_connected():
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            self.get_logger().info("MQTT client disconnected")

def main(args=None):
    rclpy.init(args=args)
    node = ROS2ToMQTTBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()