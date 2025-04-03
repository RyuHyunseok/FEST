import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point

class FireParticleListener(Node):
    def __init__(self):
        super().__init__('fire_particle_listener')
        
        # 화재 이벤트 구독
        self.fire_event_subscription = self.create_subscription(
            String,
            '/unity/fire_event',
            self.fire_event_callback,
            10
        )
        
        # 화재 위치 구독
        self.fire_location_subscription = self.create_subscription(
            Point,
            '/unity/fire_location',
            self.fire_location_callback,
            10
        )

    def fire_event_callback(self, msg):
        # 화재 이벤트 처리 로직
        self.get_logger().info(f'Fire Particle Event: {msg.data}')

    def fire_location_callback(self, msg):
        # 화재 위치 처리 로직
        self.get_logger().info(f'Fire Particle Location - X: {msg.x}, Y: {msg.y}, Z: {msg.z}')

def main(args=None):
    rclpy.init(args=args)
    node = FireParticleListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()