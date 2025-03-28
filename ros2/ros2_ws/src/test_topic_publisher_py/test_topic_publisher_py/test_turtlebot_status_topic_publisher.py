#!/usr/bin/env python3

"""
Turtlebot Status Topic Publisher Node

이 노드는 테스트를 위한 가상의 터틀봇 상태 데이터를 생성하여 발행합니다.
로봇의 현재 상태, 배터리 상태, 팔의 상태 등을 시뮬레이션합니다.

발행 데이터 구조 (ssafy_msgs/msg/TurtlebotStatus):
- twist: 로봇의 현재 속도 정보 (geometry_msgs/msg/Twist)
  - linear: 선속도 (x, y, z) [-2 ~ 2 m/s]
  - angular: 각속도 (x, y, z) [-2 ~ 2 rad/s]
- power_supply_status: 배터리 상태 (uint8)
  - 0: 배터리 부족
  - 1: 충전 중
  - 2: 완전 충전
- battery_percentage: 배터리 잔량 (float32) [0 ~ 100%]
- can_use_hand: 손 사용 가능 여부 (bool)
- can_put: 물건 놓기 가능 여부 (bool)
- can_lift: 물건 들어올리기 가능 여부 (bool)

특징:
- 10Hz로 데이터 발행
- 모든 상태값은 랜덤하게 생성
- 로봇 팔의 상태는 True/False로 랜덤하게 결정
- 배터리 상태는 3가지 상태 중 하나로 랜덤하게 결정
"""

import rclpy
from rclpy.node import Node
from ssafy_msgs.msg import TurtlebotStatus
from geometry_msgs.msg import Twist
import random

class TurtlebotStatusPublisher(Node):
    def __init__(self):
        super().__init__('turtlebot_status_publisher')
        self.publisher_ = self.create_publisher(
            TurtlebotStatus,
            '/turtlebot_status',
            10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz로 발행
        self.get_logger().info('Turtlebot Status Publisher has been started')

    def timer_callback(self):
        msg = TurtlebotStatus()
        
        # Twist 메시지 생성 (랜덤한 속도값)
        twist = Twist()
        twist.linear.x = random.uniform(-2.0, 2.0)  # -2 ~ 2 m/s
        twist.linear.y = random.uniform(-2.0, 2.0)
        twist.linear.z = random.uniform(-2.0, 2.0)
        twist.angular.x = random.uniform(-2.0, 2.0)  # -2 ~ 2 rad/s
        twist.angular.y = random.uniform(-2.0, 2.0)
        twist.angular.z = random.uniform(-2.0, 2.0)
        msg.twist = twist
        
        # 배터리 상태 (0: 배터리 부족, 1: 충전 중, 2: 완전 충전)
        msg.power_supply_status = random.randint(0, 2)
        
        # 배터리 잔량 (0 ~ 100%)
        msg.battery_percentage = random.uniform(0.0, 100.0)
        
        # 로봇 팔 상태 (랜덤하게 True/False)
        msg.can_use_hand = random.choice([True, False])
        msg.can_put = random.choice([True, False])
        msg.can_lift = random.choice([True, False])
        
        self.publisher_.publish(msg)
        self.get_logger().debug('Published turtlebot status data')

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotStatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
