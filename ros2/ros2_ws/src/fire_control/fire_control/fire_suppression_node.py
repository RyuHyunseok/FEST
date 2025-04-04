import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from fire_control.decision.fire_decision_maker import FireDecisionMaker
from fire_control.control.robot.fire_motion_controller import FireMotionController

def wait_for_goal_reached():
    """
    'goal_reached' 토픽의 값을 기다려서 True일 때만 노드를 실행
    """
    rclpy.init()
    node = rclpy.create_node('goal_checker')

    future = rclpy.Future()  # 비동기 결과 저장

    def callback(msg):
        if msg.data:
            node.get_logger().info('Goal reached! Proceeding to start FireSuppressionNode...')
            future.set_result(True)  # True일 때 실행
        else:
            node.get_logger().info('Waiting for goal_reached to be True...')

    subscription = node.create_subscription(
        Bool,
        '/goal_reached',
        callback,
        10
    )

    # goal_reached가 True가 될 때까지 기다린다
    rclpy.spin_until_future_complete(node, future)

    node.destroy_node()
    rclpy.shutdown()
    return future.result()

class FireSuppressionNode(Node):
    def __init__(self):
        super().__init__('fire_suppression_node')

        # 의사결정 모듈과 모션 컨트롤러 초기화
        self.decision_maker = FireDecisionMaker(self)
        self.motion_controller = FireMotionController(self)

        # 주기적인 상태 로깅
        self.create_timer(1.0, self.log_status)

        self.get_logger().info('Fire Suppression Node initialized!')

        # 화재 진압 시작
        self.motion_controller.start_suppression()

    def log_status(self):
        """주기적인 상태 로깅"""
        if self.decision_maker.fire_extinguisher.is_spraying:
            self.get_logger().info('Spraying fire extinguisher...')
        elif self.decision_maker.object_detected:
            self.get_logger().info('Tracking fire...')
        else:
            self.get_logger().info('Waiting for fire detection...')

def main(args=None):
    # goal_reached가 True일 때만 실행
    if wait_for_goal_reached():
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
