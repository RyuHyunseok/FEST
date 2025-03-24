import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json

class MQTTCommandSubscriber(Node):
    def __init__(self):
        super().__init__('mqtt_command_subscriber')
        
        # 로봇 ID 설정 (실제 프로젝트에서는 파라미터로 받을 수 있음)
        self.robot_id = "fest_1"
        self.get_logger().info(f"Starting MQTT command subscriber for robot: {self.robot_id}")
        
        # MQTT 설정
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        
        # MQTT 브로커 연결
        try:
            self.mqtt_client.connect('localhost', 1883, 60)
            self.get_logger().info("Connected to MQTT broker")
            
            # 루프 시작
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT broker: {e}")
        
        # 타이머 설정 (정기적으로 상태 체크를 위해)
        self.create_timer(5.0, self.check_connection)
    
    def on_connect(self, client, userdata, flags, rc):
        """MQTT 연결 성공 시 호출될 콜백"""
        self.get_logger().info(f"Connected to MQTT broker with result code: {rc}")
        
        # 명령 토픽 구독
        command_topic = f"robots/{self.robot_id}/command"
        self.mqtt_client.subscribe(command_topic)
        self.get_logger().info(f"Subscribed to topic: {command_topic}")
    
    def on_message(self, client, userdata, msg):
        """MQTT 메시지 수신 시 호출될 콜백"""
        try:
            # 메시지 디코딩 및 파싱
            payload = msg.payload.decode()
            self.get_logger().info(f"Received message on topic {msg.topic}: {payload}")
            
            # JSON 파싱
            command = json.loads(payload)
            command_type = command.get("type")
            target = command.get("target")
            mission_id = command.get("mission_id")  # 미션 ID 추출
            incident_id = command.get("incident_id")  # 화재 ID 추출
            
            # 명령 타입에 따른 처리
            if command_type == "move_to":
                self.get_logger().info(f"[COMMAND] Moving to location: X={target['x']}, Y={target['y']}")
                if mission_id:
                    self.get_logger().info(f"[MISSION] Mission ID: {mission_id}")
                    # 실제 구현에서는 여기서 미션 ID를 저장하고, 현장 도착 시 미션 상태 업데이트
                    # 이 예제에서는 로봇 코드에 mission_id를 저장하는 로직은 포함하지 않음
            elif command_type == "extinguish":
                self.get_logger().info(f"[COMMAND] Starting fire extinguishing at: {target}")
                if mission_id:
                    self.get_logger().info(f"[MISSION] Mission ID: {mission_id}")
            else:
                self.get_logger().info(f"[COMMAND] Unknown command type: {command_type}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")


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
    
    def on_shutdown(self):
        """노드 종료 시 호출될 메소드"""
        self.get_logger().info("Shutting down MQTT command subscriber...")
        if self.mqtt_client.is_connected():
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            self.get_logger().info("MQTT client disconnected")

def main(args=None):
    rclpy.init(args=args)
    node = MQTTCommandSubscriber()
    
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