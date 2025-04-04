from geometry_msgs.msg import Twist

class FireMotionController:
    def __init__(self, parent_node):
        # 부모 노드 참조 저장
        self.node = parent_node
        
        # 로봇 이동 명령 발행
        self.cmd_vel_publisher = parent_node.create_publisher(
            Twist, 
            '/cmd_vel', 
            10
        )
        
        # PD 제어 관련 변수
        self.proportional_gain = 0.05
        self.derivative_gain = 0.02
        self.max_rotation_speed = 10.0
        self.previous_error = 0
        self.deadzone = 30
    
    def control_robot(self, angle_error, object_detected):
        """로봇 이동 제어 함수"""
        if not object_detected:
            return
            
        # Twist 메시지 생성
        twist = Twist()
        
        # 화재가 중앙에 위치한 경우 (데드존 내에 있음)
        if abs(angle_error) <= self.deadzone:
            twist.angular.y = 0.0
        else:
            # 이동 명령 계산
            p_term = angle_error * self.proportional_gain
            d_term = (angle_error - self.previous_error) * self.derivative_gain
            rotation_speed = p_term + d_term
            
            # 최대/최소 회전 속도 제한
            rotation_speed = max(min(rotation_speed, self.max_rotation_speed), -self.max_rotation_speed)
            
            # 회전 속도 적용
            twist.angular.y = rotation_speed
            self.previous_error = angle_error  # 오차 업데이트
            
            # 로깅
            self.node.get_logger().info(f'Offset: {angle_error}, Rotation Speed: {rotation_speed}')
        
        # 이동 명령 발행
        self.cmd_vel_publisher.publish(twist)
    
    def stop_robot(self):
        """로봇을 정지시키는 함수"""
        twist = Twist()
        twist.angular.y = 0.0
        self.cmd_vel_publisher.publish(twist)