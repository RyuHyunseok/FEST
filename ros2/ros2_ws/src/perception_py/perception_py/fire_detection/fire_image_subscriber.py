import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2
import numpy as np
from cv_bridge import CvBridge

class FireImageSubscriber(Node):
    def __init__(self):
        super().__init__('fire_image_subscriber')
        
        # ROS2 subscribers and publishers
        self.image_subscription = self.create_subscription(
            Image, 
            '/unity_camera_image',  
            self.image_callback, 
            10
        )
        
        # Publish red area offset
        self.offset_publisher = self.create_publisher(
            Float32, 
            '/red_area/offset', 
            10
        )
        
        # OpenCV bridge for converting ROS images
        self.bridge = CvBridge()
        
        # Parameters for red color detection
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])

    def image_callback(self, msg):
        # Convert ROS image to OpenCV format
        try:
            # ROS에서 받은 이미지를 BGR로 변환 (OpenCV 기본 형식)
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 이미지 상하 반전 (0은 상하 반전을 의미)
            cv_image = cv2.flip(cv_image, 0)
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')
            return

        # Process image and get red area offset
        offset, processed_image = self.detect_red_area(cv_image)
        
        # Publish offset if detected
        if offset is not None:
            offset_msg = Float32()
            offset_msg.data = float(offset)
            self.offset_publisher.publish(offset_msg)
        
        # 처리된 이미지 표시
        # if processed_image is not None:
        #     # 창 크기 조정 및 전체 이미지 표시
        #     cv2.namedWindow('Red Detection', cv2.WINDOW_NORMAL)
        #     cv2.resizeWindow('Red Detection', processed_image.shape[1], processed_image.shape[0])
        #     cv2.imshow('Red Detection', processed_image)
        #     cv2.waitKey(1)  # 1ms 대기

    def detect_red_area(self, image):
        # Convert to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create red color mask (handles two red color ranges)
        red_mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        red_mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        
        # Find contours
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 이미지 복사본 생성 (원본 이미지 수정 방지)
        processed_image = image.copy()
        
        # 불이 감지된 경우
        if contours:
            # Find largest red area
            largest_contour = max(contours, key=cv2.contourArea)
            
            # 컨투어 그리기
            cv2.drawContours(processed_image, [largest_contour], -1, (0, 255, 0), 2)
            
            # Get image center and contour center
            image_height, image_width = image.shape[:2]
            image_center_x = image_width // 2
            
            # 이미지 중앙에 세로선 그리기
            cv2.line(processed_image, 
                    (image_center_x, 0), 
                    (image_center_x, image_height), 
                    (255, 0, 0), 2)
            
            # Calculate contour moments
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                
                # 컨투어 중심점에 원 그리기
                cv2.circle(processed_image, (cx, int(M["m01"] / M["m00"])), 5, (0, 0, 255), -1)
                
                # Calculate offset from image center
                return cx - image_center_x, processed_image
        
        # 불이 감지되지 않으면 -9999 반환
        return -9999, processed_image

def main(args=None):
    rclpy.init(args=args)
    node = FireImageSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # cv2.destroyAllWindows()  # OpenCV 창 닫기

if __name__ == '__main__':
    main()