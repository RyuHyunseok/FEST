"""
로봇의 오도메트리와 라이다 스캔 데이터 발행 테스트 노드

이 노드는 로봇의 위치(오도메트리)와 라이다 스캔 데이터를 시뮬레이션하여 발행합니다.
주요 기능:
- 사용자 입력을 통한 로봇 위치 설정
- 오도메트리 데이터 발행 (위치, 방향, 속도)
- 라이다 스캔 데이터 시뮬레이션 및 발행
- 실시간 사용자 입력 처리
"""

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import random
import math
import threading
import time

class TestTopicPublisher(Node):
    def __init__(self):
        super().__init__('test_topic_publisher')
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        
        # Timer for publishing
        self.timer = self.create_timer(0.1, self.publish_topics)  # 10Hz
        
        # Current position
        self.current_x = 0.0
        self.current_y = 0.0
        
        # Scan data
        self.scan_data = None
        
        # Thread for user input
        self.input_thread = threading.Thread(target=self.get_user_input)
        self.input_thread.daemon = True
        self.input_thread.start()
        
        self.get_logger().info('Test Topic Publisher initialized')
        self.get_logger().info('Enter coordinates in format: x y')

    def get_user_input(self):
        while rclpy.ok():
            try:
                user_input = input('Enter x y coordinates (or "q" to quit): ')
                if user_input.lower() == 'q':
                    rclpy.shutdown()
                    break
                
                x, y = map(float, user_input.split())
                self.current_x = x
                self.current_y = y
                self.get_logger().info(f'Updated position to: ({x:.2f}, {y:.2f})')
                
                # Generate new random scan data when position changes
                self.generate_scan_data()
                
            except ValueError:
                self.get_logger().error('Invalid input. Please enter two numbers.')
            except Exception as e:
                self.get_logger().error(f'Error: {str(e)}')

    def generate_scan_data(self):
        scan = LaserScan()
        scan.header.frame_id = 'base_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 0.01  # 360 points
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 10.0
        
        # Generate random ranges
        scan.ranges = []
        for _ in range(360):
            if random.random() < 0.1:  # 10% chance of invalid reading
                scan.ranges.append(float('inf'))
            else:
                scan.ranges.append(random.uniform(0.1, 10.0))
        
        self.scan_data = scan

    def publish_topics(self):
        # Publish odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = self.current_x
        odom.pose.pose.position.y = self.current_y
        odom.pose.pose.position.z = 0.0
        
        # Random orientation
        yaw = random.uniform(-math.pi, math.pi)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(yaw/2)
        odom.pose.pose.orientation.w = math.cos(yaw/2)
        
        # Random velocity
        odom.twist.twist.linear.x = random.uniform(-1.0, 1.0)
        odom.twist.twist.linear.y = random.uniform(-1.0, 1.0)
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = random.uniform(-1.0, 1.0)
        
        self.odom_pub.publish(odom)
        
        # Publish scan data if available
        if self.scan_data is not None:
            self.scan_data.header.stamp = self.get_clock().now().to_msg()
            self.scan_pub.publish(self.scan_data)

def main(args=None):
    rclpy.init(args=args)
    node = TestTopicPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
