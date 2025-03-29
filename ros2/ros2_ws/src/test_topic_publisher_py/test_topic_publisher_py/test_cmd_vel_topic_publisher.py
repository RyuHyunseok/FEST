#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # 타이머 추가 (10Hz = 0.1초 주기)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        self.get_logger().info('cmd_vel 토픽 발행자가 시작되었습니다.')

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = float(self.current_linear_x)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(self.current_angular_z)
        
        self.publisher.publish(msg)

    def update_velocity(self, linear_x, angular_z):
        self.current_linear_x = linear_x
        self.current_angular_z = angular_z
        self.get_logger().info(f'속도 업데이트 - 선속도: {linear_x}, 각속도: {angular_z}')

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_publisher = CmdVelPublisher()
    
    # spin을 별도 스레드에서 실행
    spin_thread = threading.Thread(target=rclpy.spin, args=(cmd_vel_publisher,))
    spin_thread.start()

    try:
        while True:
            try:
                linear_x = input("선속도 (m/s)를 입력하세요 (종료: q): ")
                if linear_x.lower() == 'q':
                    break
                
                angular_z = input("각속도 (rad/s)를 입력하세요: ")
                cmd_vel_publisher.update_velocity(float(linear_x), float(angular_z))
                
            except ValueError as e:
                cmd_vel_publisher.get_logger().error('올바른 숫자를 입력해주세요.')
            
    except KeyboardInterrupt:
        # 종료 시 로봇을 정지
        cmd_vel_publisher.update_velocity(0.0, 0.0)
        
    # 종료 처리
    cmd_vel_publisher.update_velocity(0.0, 0.0)
    rclpy.shutdown()
    spin_thread.join()
    cmd_vel_publisher.destroy_node()

if __name__ == '__main__':
    main()
