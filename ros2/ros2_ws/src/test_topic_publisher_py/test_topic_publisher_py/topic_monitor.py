import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry

class TopicMonitor(Node):
    def __init__(self):
        super().__init__('topic_monitor')
        
        # 로봇 위치 구독
        self.pose_sub = self.create_subscription(
            Pose,
            '/robots/fest_1/position',
            self.pose_callback,
            10)
            
        # Odometry 구독
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
            
        # cmd_vel 구독
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
            
        self.get_logger().info('토픽 모니터링을 시작합니다...')

    def pose_callback(self, msg):
        self.get_logger().info('\n=== 로봇 위치 (/robots/fest_1/position) ===')
        self.get_logger().info(f'위치: x={msg.position.x:.3f}, y={msg.position.y:.3f}, z={msg.position.z:.3f}')
        self.get_logger().info(f'방향: x={msg.orientation.x:.3f}, y={msg.orientation.y:.3f}, z={msg.orientation.z:.3f}, w={msg.orientation.w:.3f}')

    def odom_callback(self, msg):
        self.get_logger().info('\n=== Odometry 정보 (/odom) ===')
        self.get_logger().info(f'위치: x={msg.pose.pose.position.x:.3f}, y={msg.pose.pose.position.y:.3f}, z={msg.pose.pose.position.z:.3f}')
        self.get_logger().info(f'선속도: x={msg.twist.twist.linear.x:.3f}, y={msg.twist.twist.linear.y:.3f}, z={msg.twist.twist.linear.z:.3f}')
        self.get_logger().info(f'각속도: x={msg.twist.twist.angular.x:.3f}, y={msg.twist.twist.angular.y:.3f}, z={msg.twist.twist.angular.z:.3f}')

    def cmd_vel_callback(self, msg):
        self.get_logger().info('\n=== 제어 명령 (/cmd_vel) ===')
        self.get_logger().info(f'선속도: x={msg.linear.x:.3f}, y={msg.linear.y:.3f}, z={msg.linear.z:.3f}')
        self.get_logger().info(f'각속도: x={msg.angular.x:.3f}, y={msg.angular.y:.3f}, z={msg.angular.z:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = TopicMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
