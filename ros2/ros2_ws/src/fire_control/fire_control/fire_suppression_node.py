import rclpy
from rclpy.node import Node
from fire_control.decision.fire_decision_maker import FireDecisionMaker
from fire_control.control.robot.fire_motion_controller import FireMotionController

class FireSuppressionNode(Node):
    def __init__(self):
        super().__init__('fire_suppression_node')
        
        # 의사결정 모듈과 모션 컨트롤러 초기화
        self.decision_maker = FireDecisionMaker(self)
        self.motion_controller = FireMotionController(self)
        
        # 로깅 타이머
        self.create_timer(1.0, self.log_status)
        
        self.get_logger().info('Fire Suppression Node initialized')

    def log_status(self):
        """주기적인 상태 로깅"""
        if self.decision_maker.fire_extinguisher.is_spraying:
            self.get_logger().info('Spraying fire extinguisher...')
        elif self.decision_maker.object_detected:
            self.get_logger().info('Tracking fire...')
        else:
            self.get_logger().info('Waiting for fire detection...')

def main(args=None):
    rclpy.init(args=args)
    node = FireSuppressionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()