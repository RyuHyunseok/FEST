import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class FireExtinguisher:
    def __init__(self, parent_node):
        # 부모 노드 참조 저장
        self.node = parent_node
        
        # 소화액 분사 명령 발행
        self.spray_publisher = self.node.create_publisher(String, '/extinguisher/spray', 10)
        self.is_spraying = False  # 분사 상태
        self.complete_publisher = self.node.create_publisher(String, '/extinguisher/complete', 10)

    def start_spray(self):
        """소화액 분사 시작"""
        if not self.is_spraying:  
            self.is_spraying = True
            spray_msg = String()
            spray_msg.data = "spraying_start"
            self.spray_publisher.publish(spray_msg)
            self.node.get_logger().info("Starting spray!")

    def stop_spray(self):
        """소화액 분사 중지"""
        if self.is_spraying:  
            self.is_spraying = False
            spray_msg = String()
            spray_msg.data = "spraying_stop"
            self.spray_publisher.publish(spray_msg)
            self.node.get_logger().info("Stopping spray!")

            # 소화 완료 메시지 발행
            complete_msg = String(data="fire_extinguished")
            self.complete_publisher.publish(complete_msg)