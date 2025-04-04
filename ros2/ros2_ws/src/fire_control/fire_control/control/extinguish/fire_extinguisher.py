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