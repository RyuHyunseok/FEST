import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import threading
import time
from ultralytics import YOLO
import numpy as np
import json

class OpenCVYOLO(Node):
    def __init__(self):
        super().__init__('opencv_yolo')
        self.bridge = CvBridge()
        # YOLOv8 모델 로드 및 사람 클래스만 탐지하도록 설정
        self.model = YOLO('yolov8n.pt')
        self.model.to('cpu')  # GPU 사용 설정
        self.conf_threshold = 0.5  # 신뢰도 임계치
        self.model.classes = [0]  # COCO 데이터셋에서 person 클래스는 0번
        
        # ROS2 publisher(침입자 감지 알림) 설정
        self.publisher = self.create_publisher(String, '/prowler/new', 10)
        # 바운딩 박스 정보를 위한 새로운 publisher
        self.bbox_publisher = self.create_publisher(String, '/detection/bbox', 10)
        
        # 객체 탐지 상태 추적을 위한 변수
        self.has_published = False  # 메시지 발행 여부를 추적하는 플래그

        # ROS2 subscriber(카메라 이미지) 설정
        self.subscription = self.create_subscription(
            Image,
            '/unity_camera/image_raw',
            self.image_callback,
            10)
        self.latest_image = None
        self.running = True  # 프로그램 실행 상태 플래그
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        
        self.display_thread.start()
        # 디스플레이 관련 설정
        self.display_width = 640  # 표시할 이미지 너비
        # 이미지 처리 최적화를 위한 변수
        self.last_process_time = 0
        self.process_interval = 0.033  # 30 FPS로 처리 제한
        
        # 초기화 로그
        self.get_logger().info('OpenCVYOLO node initialized')
        self.get_logger().info(f'Using YOLO model: yolov8n.pt')
        self.get_logger().info(f'Confidence threshold: {self.conf_threshold}')
        self.get_logger().info(f'Detecting class: person (0)')
        
    def image_callback(self, msg):
        try:
            # ROS 메시지 → OpenCV 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            
            # 이미지 크기 조절
            height, width = cv_image.shape[:2]
            scale = self.display_width / width
            new_height = int(height * scale)
            cv_image = cv2.resize(cv_image, (self.display_width, new_height))
            
            # 이미지 반전
            flipped_vertical = cv2.flip(cv_image, 0)  # 상하 반전
            
            # YOLOv8 객체 탐지 수행
            results = self.model(flipped_vertical, conf=self.conf_threshold, verbose=False)[0]
            
            # 실제 사람으로 탐지된 객체만 필터링
            detected_boxes = []  # 탐지된 모든 박스 정보를 저장할 리스트
            for box in results.boxes:
                cls = int(box.cls[0])
                if cls == 0:  # person class만 처리
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = box.conf[0].cpu().numpy() * 100  # 신뢰도를 백분율로 변환
                    label = f'Person {conf:.1f}%'  # 라벨에 백분율로 표시
                    
                    # 바운딩 박스 정보 저장
                    box_info = {
                        "x1": float(x1),
                        "y1": float(y1),
                        "x2": float(x2),
                        "y2": float(y2),
                        "confidence": float(conf),
                        "label": label
                    }
                    detected_boxes.append(box_info)
                    
                    # 바운딩 박스 그리기
                    cv2.rectangle(flipped_vertical, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    # 라벨 표시
                    cv2.putText(flipped_vertical, label, (int(x1), int(y1) - 10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # 객체가 탐지되고 아직 메시지를 발행하지 않았다면
                    if not self.has_published:
                        # 객체가 탐지되면 ROS2 메시지 발행(Web에서 사용 )
                        message_data = {
                            "prowler_id": 1
                        }
                        msg = String()
                        msg.data = json.dumps(message_data)
                        self.publisher.publish(msg)
                        self.has_published = True  # 발행 상태 업데이트
                        self.get_logger().info(f'Published message: {msg.data} (Confidence: {conf:.1f}%)')
            
            # 바운딩 박스 정보 전송(Unity에서 사용)
            # if detected_boxes:
            bbox_msg = String()
            bbox_data = {
                "timestamp": time.time(),
                "image_width": self.display_width,
                "image_height": new_height,
                "boxes": detected_boxes
            }
            bbox_msg.data = json.dumps(bbox_data)
            self.bbox_publisher.publish(bbox_msg)
            self.get_logger().info(f'Published {len(detected_boxes)} bounding boxes')
            
            # 실제 탐지된 사람 수 표시 (신뢰도 임계값을 통과한 사람만 카운트)
            cv2.putText(flipped_vertical, f'People detected: {len(detected_boxes)}', (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            self.latest_image = flipped_vertical
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def display_loop(self):
        while self.running:
            if self.latest_image is not None:
                cv2.imshow('Vertical Flip', self.latest_image)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):  # 'q' 키를 누르면 종료
                    self.running = False
                    break
            time.sleep(0.001)  # CPU 사용량 감소를 위한 짧은 대기

def main(args=None):
    rclpy.init(args=args)
    viewer = OpenCVYOLO()
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass  # 종료 메시지 출력 제거
    finally:
        viewer.running = False  # 프로그램 종료 플래그 설정
        viewer.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
