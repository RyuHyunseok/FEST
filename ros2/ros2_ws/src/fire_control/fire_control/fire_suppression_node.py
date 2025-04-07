# ROS2 Python 클라이언트 라이브러리 import
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# 사용자 정의 모듈 import (불 인식 및 제어 로직)
from fire_control.decision.fire_decision_maker import FireDecisionMaker
from fire_control.control.robot.fire_motion_controller import FireMotionController


# 실제로 화재 진압을 수행하는 노드 정의
class FireSuppressionNode(Node):
    def __init__(self):
        super().__init__('fire_suppression_node')  # 노드 이름 설정
        self.decision_maker = FireDecisionMaker(self)  # 화재 인식 및 판단 모듈 초기화
        self.motion_controller = FireMotionController(self)  # 로봇 모션 제어 모듈 초기화

        self.create_timer(1.0, self.loop)  # 주기적으로 loop 함수 호출 (1초 간격)
        self.get_logger().info('FireSuppressionNode initialized.')  # 초기화 로그 출력

    # 주기적으로 화재 상태를 확인하고 적절한 동작 수행
    def loop(self):
        if self.decision_maker.fire_extinguisher.is_spraying:
            self.get_logger().info('Spraying fire extinguisher...')  # 소화기 작동 중
        elif self.decision_maker.object_detected:
            self.get_logger().info('Tracking fire...')  # 불이 감지되어 추적 중
        else:
            self.get_logger().info('Waiting for fire detection...')  # 아무 일 없음, 대기 중


# 화재 진압 노드를 동적으로 생성/종료하는 관리자 노드 정의
class FireSuppressionManager(Node):
    def __init__(self):
        super().__init__('fire_suppression_manager')  # 관리자 노드 이름 설정
        self.fire_node = None  # 실제 진압 노드 인스턴스를 저장할 변수

        # '/goal_reached' 토픽을 구독하여 목표 지점 도달 여부 판단
        self.create_subscription(Bool, '/goal_reached', self.goal_callback, 10)
        self.get_logger().info('FireSuppressionManager is running...')

    # 목표 지점 도달 여부 콜백 함수
    def goal_callback(self, msg: Bool):
        if msg.data and self.fire_node is None:
            # 목표 지점에 도달했고 아직 fire_node가 없다면 새로 생성
            self.get_logger().info('Goal reached! Starting fire suppression node...')
            self.fire_node = FireSuppressionNode()  # 노드 인스턴스 생성

            # ROS2 종료 시 해당 노드도 정리되도록 등록
            rclpy.get_default_context().on_shutdown(self.fire_node.destroy_node)

            # Executor에 fire_node 추가하여 스케줄링 가능하게 함
            rclpy.get_default_context().executor.add_node(self.fire_node)
        elif not msg.data and self.fire_node is not None:
            # 목표 지점을 벗어나고 fire_node가 존재한다면 제거
            self.get_logger().warn('Goal lost. Stopping fire suppression node...')
            self.fire_node.destroy_node()  # 노드 종료
            rclpy.get_default_context().executor.remove_node(self.fire_node)  # Executor에서 제거
            self.fire_node = None  # 인스턴스 초기화


# 프로그램 진입점
def main(args=None):
    rclpy.init(args=args)  # rclpy 초기화
    manager = FireSuppressionManager()  # 관리자 노드 생성

    # 멀티스레드 Executor 생성 (여러 노드 동시 실행 가능)
    executor = rclpy.executors.MultiThreadedExecutor()

    # 전역 context에 executor 등록 (콜백에서 접근 가능하게 하기 위해)
    rclpy.get_default_context().executor = executor

    # manager 노드를 executor에 등록
    executor.add_node(manager)

    # 모든 노드가 실행되도록 spin (콜백 대기)
    executor.spin()

    # 종료 시 자식 노드까지 정리
    if manager.fire_node is not None:
        manager.fire_node.destroy_node()
    manager.destroy_node()
    rclpy.shutdown()  # ROS 종료


# Python으로 직접 실행할 경우 main() 실행
if __name__ == '__main__':
    main()
