import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from fire_control.decision.fire_decision_maker import FireDecisionMaker
from fire_control.control.robot.fire_motion_controller import FireMotionController

class FireSuppressionNode(Node):
    def __init__(self):
        super().__init__('fire_suppression_node')
        self.decision_maker = FireDecisionMaker(self)
        self.motion_controller = FireMotionController(self)
        self.create_timer(1.0, self.loop)
        self.get_logger().info('FireSuppressionNode initialized.')

    def loop(self):
        if self.decision_maker.fire_extinguisher.is_spraying:
            self.get_logger().info('Spraying fire extinguisher...')
        elif self.decision_maker.object_detected:
            self.get_logger().info('Tracking fire...')
        else:
            self.get_logger().info('Waiting for fire detection...')


class FireSuppressionManager(Node):
    def __init__(self):
        super().__init__('fire_suppression_manager')
        self.fire_node = None  # FireSuppressionNode 인스턴스
        self.create_subscription(Bool, '/goal_reached', self.goal_callback, 10)
        self.get_logger().info('FireSuppressionManager is running...')

    def goal_callback(self, msg: Bool):
        if msg.data and self.fire_node is None:
            self.get_logger().info('Goal reached! Starting fire suppression node...')
            self.fire_node = FireSuppressionNode()
            rclpy.get_default_context().on_shutdown(self.fire_node.destroy_node)
            rclpy.get_default_context().executor.add_node(self.fire_node)
        elif not msg.data and self.fire_node is not None:
            self.get_logger().warn('Goal lost. Stopping fire suppression node...')
            self.fire_node.destroy_node()
            rclpy.get_default_context().executor.remove_node(self.fire_node)
            self.fire_node = None


def main(args=None):
    rclpy.init(args=args)
    manager = FireSuppressionManager()
    executor = rclpy.executors.MultiThreadedExecutor()
    rclpy.get_default_context().executor = executor  # 전역 executor 등록
    executor.add_node(manager)
    executor.spin()

    if manager.fire_node is not None:
        manager.fire_node.destroy_node()
    manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()