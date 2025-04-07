"""
ROS2 토픽과 MQTT 토픽 간의 변환 관계:

ROS2 토픽                    ->  MQTT 토픽
--------------------------------------------------------
/robots/fest_1/position     ->  robots/fest_1/position
                             (Pose -> {x, y, orientation})
"""

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import paho.mqtt.client as mqtt
from geometry_msgs.msg import Pose
import math
from std_msgs.msg import String

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
        self.robot_status_topic = f"robots/{self.robot_id}/status"
        self.prowler_topic = 'prowler/new'

        # MQTT 브로커 연결
        try:
            # self.mqtt_client.connect('j12d106.p.ssafy.io', 1883, 60)
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

        self.subscription = self.create_subscription(
            String,
            f'/robots/{self.robot_id}/status',
            self.robot_status_callback,
            10
        )

        self.fire_subscription = self.create_subscription(
            String,
            '/incidents/new',  # ROS2 토픽 경로
            self.fire_callback,
            10  # QoS 프로파일
        )

                # 화재 상태 업데이트 구독
        self.fire_status_subscription = self.create_subscription(
            String,
            '/incidents/status',  # ROS2 토픽 경로
            self.fire_status_callback,
            10  # QoS 프로파일
        )

        self.prowler_subscription = self.create_subscription(
            String,
            '/prowler/new',
            self.prowler_callback,
            10
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
    
    def prowler_callback(self, msg):
        
        prowler_data = json.loads(msg.data)

        prowler_info = {
            "prowler_id": prowler_data['prowler_id'],
            "count": prowler_data['count'],
            # prowler_data['detected_at']
        }

        serialized_prowler = json.dumps(prowler_info)

        self.mqtt_client.publish(self.prowler_topic, serialized_prowler)



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
    
    def robot_status_callback(self, msg):

        try: 
            status_data = json.loads(msg.data) # json 역직렬화

            # status_data 에서 원하는 정보만 추출
            robot_id = status_data['robot_id']
            battery = status_data['battery']
            water = status_data['water']

            status_info = {
                'battery': battery,
                'water': water
            }

            # 해당 정보 직렬화
            serialized_status_info = json.dumps(status_info)

            # 직렬화된 json mqtt 브로커로 publish
            self.mqtt_client.publish(self.robot_status_topic, serialized_status_info)

            self.get_logger().info(f'Published status to MQTT:  {status_info}')
        except Exception as e:
            self.get_logger().error(f'Error processing status data: {e}')

    def fire_callback(self, msg):
        """ROS2 화재 발생 토픽 메시지 수신 시 호출되는 콜백"""
        try:
            # 메시지에서 JSON 데이터 파싱
            fire_data = json.loads(msg.data)
            
            # 위치 데이터 형식 확인 및 조정
            if 'location' in fire_data and isinstance(fire_data['location'], dict):
                location = fire_data['location']
                
                # Unity에서 전송된 x, z 좌표 사용
                if 'x' in location and 'z' in location:
                    unity_x = float(location['x'])
                    unity_z = float(location['z'])
                    
                    # 로봇 위치와 동일한 형식으로 매핑 (x, y 속성만 사용)
                    new_location = {
                        "x": round(unity_x, 2),  # 화재의 x 좌표 그대로 사용
                        "y": round(unity_z, 2)   # 화재의 z 좌표를 y 좌표로 사용
                    }
                    
                    # 새 location으로 교체
                    fire_data['location'] = new_location
            
            # 변환된 데이터를 JSON으로 다시 직렬화
            mqtt_msg = json.dumps(fire_data)
            self.mqtt_client.publish("incidents/new", mqtt_msg)
            
            self.get_logger().info(f'Published fire incident to MQTT: {fire_data}')
        except Exception as e:
            self.get_logger().error(f'Error processing fire incident data: {e}')

    def fire_status_callback(self, msg):
        """ROS2 화재 상태 업데이트 토픽 메시지 수신 시 호출되는 콜백"""
        try:
            # 메시지에서 JSON 데이터 파싱
            status_data = json.loads(msg.data)
            
            # incident_id 확인
            if 'incident_id' in status_data:
                incident_id = status_data['incident_id']

                # 동적으로 MQTT 토픽 생성
                mqtt_topic = f"incidents/{incident_id}/status"
                
                # 데이터를 JSON으로 직렬화하여 MQTT로 발행
                mqtt_msg = json.dumps(status_data)
                self.mqtt_client.publish(mqtt_topic, mqtt_msg)
                
                self.get_logger().info(f'Published fire status update to MQTT: {mqtt_topic} - {status_data}')
            else:
                self.get_logger().warn(f'Missing incident_id in fire status update: {status_data}')
                
        except Exception as e:
            self.get_logger().error(f'Error processing fire status update: {e}')


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
