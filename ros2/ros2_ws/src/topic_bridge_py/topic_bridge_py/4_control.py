import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import signal
import time

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # 프로세스 저장을 위한 변수들
        self.auto_goal_publisher = None
        self.manual_goal_publisher = None
        
        # 토픽 구독자 생성
        self.goal_point_sub = self.create_subscription(
            String,
            '/goal_point_2',
            self.goal_point_callback,
            10)
        
        self.extinguisher_sub = self.create_subscription(
            String,
            '/extinguisher/complete',
            self.extinguisher_callback,
            10)
        
        self.start_sub = self.create_subscription(
            String,
            '/start',
            self.start_callback,
            10)
        
        # 초기 상태 설정
        self.is_auto_mode = True
        
        # 초기 노드 실행
        self.start_auto_mode()
        
    def start_auto_mode(self):
        """자동 모드 시작"""
        if self.is_auto_mode:
            return
            
        # 수동 모드 프로세스 종료
        if self.manual_goal_publisher:
            self.manual_goal_publisher.terminate()
            self.manual_goal_publisher = None
            
        # 자동 모드 프로세스 시작
        self.auto_goal_publisher = subprocess.Popen(
            ['ros2', 'run', 'auto_driving_cpp', 'goal_publisher_auto'],
            preexec_fn=os.setsid
        )
        
        self.is_auto_mode = True
        self.get_logger().info('Switched to auto mode')
        
    def start_manual_mode(self):
        """수동 모드 시작"""
        if not self.is_auto_mode:
            return
            
        # 자동 모드 프로세스 종료
        if self.auto_goal_publisher:
            self.auto_goal_publisher.terminate()
            self.auto_goal_publisher = None
            
        # 수동 모드 프로세스 시작
        self.manual_goal_publisher = subprocess.Popen(
            ['ros2', 'run', 'auto_driving_cpp', 'goal_publisher_manual'],
            preexec_fn=os.setsid
        )
        
        self.is_auto_mode = False
        self.get_logger().info('Switched to manual mode')
        
    def goal_point_callback(self, msg):
        """/goal_point_2 토픽 콜백"""
        if self.auto_goal_publisher:
            self.auto_goal_publisher.terminate()
            self.auto_goal_publisher = None
            self.get_logger().info('Stopped auto goal publisher')
            
    def extinguisher_callback(self, msg):
        """/extinguisher/complete 토픽 콜백"""
        if msg.data == 'fire_extinguished':
            self.start_manual_mode()
            
    def start_callback(self, msg):
        """/start 토픽 콜백"""
        self.start_auto_mode()
        
    def __del__(self):
        """소멸자에서 프로세스 정리"""
        if self.auto_goal_publisher:
            self.auto_goal_publisher.terminate()
        if self.manual_goal_publisher:
            self.manual_goal_publisher.terminate()

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

