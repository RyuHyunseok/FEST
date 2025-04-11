import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String, Bool
import os
from ament_index_python.packages import get_package_share_directory

class GoalControlNode(Node):
    def __init__(self):
        super().__init__('goal_control_node')
        
        # 발행자 생성
        self.goal_pub = self.create_publisher(Point, 'goal_point', 10)
        
        # 구독자 생성
        self.mqtt_goal_sub = self.create_subscription(
            Point,
            'goal_point_mqtt',
            self.mqtt_goal_callback,
            10)
            
        self.extinguisher_sub = self.create_subscription(
            String,
            '/extinguisher/complete',
            self.extinguisher_callback,
            10)
            
        self.goal_reached_sub = self.create_subscription(
            Bool,
            'goal_reached',
            self.goal_reached_callback,
            10)
        
        # 상태 변수
        self.is_auto_mode = True
        self.current_goal_index = 0
        self.goals = []
        self.waiting_for_extinguisher = False  # 소화기 완료 대기 상태 추가
        
        # 목표점 파일 로드
        self.load_goals()
        
        # 초기 목표점 발행
        self.publish_next_goal()
        
    def load_goals(self):
        """goal_list.txt에서 목표점 로드"""
        package_share_dir = get_package_share_directory('topic_bridge_py')
        file_path = os.path.join(package_share_dir, 'goal', 'goal_list.txt')
        try:
            with open(file_path, 'r') as file:
                for line in file:
                    x, y = map(float, line.strip().split())
                    point = Point()
                    point.x = y
                    point.y = -x
                    point.z = 0.0
                    self.goals.append(point)
            self.get_logger().info(f'Loaded {len(self.goals)} goals from file')
        except Exception as e:
            self.get_logger().error(f'Failed to load goals: {e}')
            
    def publish_next_goal(self):
        """다음 목표점 발행"""
        if not self.goals:
            return
            
        if self.current_goal_index >= len(self.goals):
            self.current_goal_index = 0
            
        goal = self.goals[self.current_goal_index]
        self.goal_pub.publish(goal)
        self.get_logger().info(f'Published goal: ({goal.x}, {goal.y})')
        
    def mqtt_goal_callback(self, msg):
        """MQTT 목표점 콜백"""
        # MQTT 좌표는 언제나 받을 수 있음
        self.is_auto_mode = False
        self.waiting_for_extinguisher = False  # 새로운 MQTT 좌표를 받으면 이전 대기 상태 취소
        self.get_logger().info('Switched to MQTT goal mode')
        
        # 새로운 Point 메시지 생성
        goal = Point()
        goal.x = msg.x
        goal.y = msg.y
        goal.z = 0.0
        
        # goal_point 토픽으로 발행
        self.goal_pub.publish(goal)
        self.get_logger().info(f'Published MQTT goal: ({goal.x}, {goal.y})')
            
    def extinguisher_callback(self, msg):
        """소화기 완료 콜백"""
        if msg.data == 'fire_extinguished':
            if self.waiting_for_extinguisher:  # MQTT 목표점 도달 후 대기 중일 때만
                # (0, 5) 좌표로 이동
                goal = Point()
                goal.x = 5.0
                goal.y = 0.0
                goal.z = 0.0
                self.goal_pub.publish(goal)
                self.get_logger().info('Published extinguisher goal: (0, 5)')
                self.waiting_for_extinguisher = False  # 대기 상태 해제
            
    def goal_reached_callback(self, msg):
        """목표 도달 콜백"""
        if msg.data:
            if not self.is_auto_mode:
                # MQTT 모드에서 목표점 도달 시
                self.waiting_for_extinguisher = True  # 소화기 완료 대기 상태로 전환
                self.get_logger().info('Reached MQTT goal, waiting for extinguisher complete')
            elif not self.waiting_for_extinguisher:  # 자동 모드이고 소화기 대기 중이 아닐 때만
                # 자동 모드에서 목표점 도달 시
                self.current_goal_index += 1
                self.publish_next_goal()

def main(args=None):
    rclpy.init(args=args)
    node = GoalControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
