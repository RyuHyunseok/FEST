from rclpy.node import Node
from std_msgs.msg import Float32
from fire_control.control.extinguish.fire_extinguisher import FireExtinguisher

class FireDecisionMaker:
    def __init__(self, parent_node):
        # 부모 노드 참조 저장
        self.node = parent_node
        
        # 소화액 분사 컨트롤러 초기화
        self.fire_extinguisher = FireExtinguisher(parent_node)
        
        # 감지 상태 플래그 및 안정화 파라미터
        self.object_detected = False
        self.stable_counter = 0
        self.stability_threshold = 5  # 5회 연속으로 안정화되면 분사 시작
        self.deadzone = 30  # 중앙 정렬 간주 영역
        
        # Offset 구독
        self.offset_subscription = parent_node.create_subscription(
            Float32, 
            '/red_area/offset', 
            self.offset_callback, 
            10
        )

    def offset_callback(self, msg):
        """화재 감지 오프셋 콜백"""
        angle_error = msg.data

        # 화재 감지 여부 판단
        if angle_error == -9999:  
            self.handle_no_fire()
        else:
            self.handle_fire_detected(angle_error)
    
    def handle_no_fire(self):
        """화재가 감지되지 않은 경우 처리"""
        self.object_detected = False
        self.node.get_logger().warn("No fire detected! Stopping spray.")
        self.fire_extinguisher.stop_spray()
        self.stable_counter = 0  # 카운터 리셋

    def handle_fire_detected(self, angle_error):
        """화재가 감지된 경우 처리"""
        self.object_detected = True
        self.node.get_logger().info(f"Fire detected at offset: {angle_error}")
        
        # 중앙 정렬 여부 판단
        if abs(angle_error) <= self.deadzone:
            self.handle_centered_fire()
        else:
            self.handle_offset_fire()
            
        # 모션 컨트롤러에 제어 명령 전달
        self.node.motion_controller.control_robot(angle_error, self.object_detected)
    
    def handle_centered_fire(self):
        """화재가 중앙에 위치한 경우 처리"""
        # 안정화 카운터 증가
        self.stable_counter += 1
        self.node.get_logger().info(f'Stable position: {self.stable_counter}/{self.stability_threshold}')
        
        # 안정화 완료 후 분사 시작
        if self.stable_counter >= self.stability_threshold and not self.fire_extinguisher.is_spraying:
            self.fire_extinguisher.start_spray()
    
    def handle_offset_fire(self):
        """화재가 중앙에서 벗어난 경우 처리"""
        self.stable_counter = 0  # 안정화 카운터 리셋