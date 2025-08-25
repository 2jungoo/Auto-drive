import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import math

# 기존의 Autoracer 클래스
class Autoracer(Node):
    def __init__(self):
        super().__init__('autoracer_node')
        self.qos_profile = QoSProfile(depth=10)
        self.qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        # 이미지 구독자 설정
        self.image_subscriber = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            self.qos_profile
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # 차량 제어 메시지
        self.twist_msg = Twist()
        self.twist_msg.linear.x = 0.0  # 출발하지 않도록 속도를 0으로 설정
        self.twist_msg.angular.z = 0.0  # 조향각을 0으로 설정

        self.last_image_time = time.time()
        self.camera_fps = 0
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)

    def publish_cmd_vel(self):
        """차량 제어 명령을 발행 (정지 상태 유지)"""
        self.cmd_vel_publisher.publish(self.twist_msg)

    def image_callback(self, msg):
        """
        카메라 이미지를 처리하는 콜백 함수.
        """
        # 이미지 디코딩
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # 초록불 감지 함수 호출
        is_detected, processed_image = self.detect_traffic_light(image)
        
        # 감지 여부에 따라 텍스트 출력
        if is_detected:
            print("✅ 초록불이 감지되었습니다! (출발 대기 중...)")
        else:
            print("❌ 초록불이 감지되지 않았습니다. 대기 중...")

        # (선택 사항) 웹 대시보드에 처리된 이미지 전송
        # self.web_viewer_server.send_image(processed_image)

    def detect_traffic_light(self, image):
        """
        초록불 신호등을 감지하는 함수 (대회 환경 최적화).
        """
        height, width, _ = image.shape
        roi = image[0:int(height / 3), 0:width]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_green = np.array([35, 100, 100])
        upper_green = np.array([85, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        is_green_light_detected = False
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 200:
                is_green_light_detected = True
        
        return is_green_light_detected, image

# 기존 메인 함수
def main(args=None):
    rclpy.init(args=args)
    autoracer = Autoracer()
    rclpy.spin(autoracer)
    autoracer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
