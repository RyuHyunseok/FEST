#!/usr/bin/env python3

"""
Scan Topic Publisher Node

이 노드는 테스트를 위한 가상의 라이다 스캔 데이터를 생성하여 발행합니다.
실제 라이다 센서처럼 2D 스캔 데이터를 시뮬레이션합니다.

발행 데이터 구조 (sensor_msgs/msg/LaserScan):
- header: 메시지의 타임스탬프와 프레임 정보
- angle_min: 스캔 시작 각도 (-π rad)
- angle_max: 스캔 종료 각도 (π rad)
- angle_increment: 각도 간격 (π/180 rad)
- time_increment: 각 측정 사이의 시간 간격
- scan_time: 전체 스캔에 걸리는 시간
- range_min: 최소 측정 거리 (0.3m)
- range_max: 최대 측정 거리 (8.0m)
- ranges: 각도별 거리 측정값 배열 (360개 포인트)
- intensities: 각도별 강도값 배열 (360개 포인트)

특징:
- 10Hz로 데이터 발행
- 360도 스캔 (1도 간격)
- 30% 확률로 무한대 값(장애물 없음), 70% 확률로 랜덤 거리값
- 모든 강도값은 0~100 사이의 랜덤값
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import random
import math

class ScanTopicPublisher(Node):
    def __init__(self):
        super().__init__('scan_topic_publisher')
        self.publisher_ = self.create_publisher(
            LaserScan,
            '/scan',
            10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz로 발행
        self.get_logger().info('Scan Topic Publisher has been started')

    def timer_callback(self):
        msg = LaserScan()
        
        # 헤더 정보 설정
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # 스캔 파라미터 설정
        msg.angle_min = -math.pi  # -180도
        msg.angle_max = math.pi   # 180도
        msg.angle_increment = math.pi / 180.0  # 1도 간격
        msg.time_increment = 0.0
        msg.scan_time = 0.1  # 10Hz
        msg.range_min = 0.3  # 30cm
        msg.range_max = 8.0  # 8m
        
        # 랜덤한 거리값 생성 (360개 포인트)
        msg.ranges = []
        for _ in range(360):
            # 30% 확률로 무한대 값(장애물 없음), 70% 확률로 랜덤 거리값
            if random.random() < 0.3:
                msg.ranges.append(float('inf'))
            else:
                msg.ranges.append(random.uniform(msg.range_min, msg.range_max))
        
        # 강도값도 랜덤하게 생성
        msg.intensities = [random.uniform(0.0, 100.0) for _ in range(360)]
        
        self.publisher_.publish(msg)
        self.get_logger().debug('Published scan data')

def main(args=None):
    rclpy.init(args=args)
    node = ScanTopicPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
