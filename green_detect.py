#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import threading
import time
import socket
import math
from http.server import BaseHTTPRequestHandler, HTTPServer
from enum import Enum

qos_profile = QoSProfile(depth=10)
qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

class DriveMode(Enum):
    RUBBERCON_AVOIDANCE = "RUBBERCON_AVOID"
    LANE_FOLLOWING = "LANE_FOLLOW"
    EMERGENCY_STOP = "EMERGENCY_STOP"

class WebViewer(BaseHTTPRequestHandler):
    def __init__(self, autoracer_node, *args, **kwargs):
        self.autoracer = autoracer_node
        super().__init__(*args, **kwargs)

    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            html = """
            <html>
            <head>
                <title>🚗 Autoracer 2025 Contest</title>
                <style>
                    body { background: linear-gradient(135deg, #1e1e1e..."""
            
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
        self.current_mode = DriveMode.EMERGENCY_STOP # 초기 모드를 정지 상태로 설정
        self.rubbercon_passed = False
        self.lidar_data = None
        self.current_speed = 0.0
        self.current_steering = 0.0
        
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
        is_detected, _ = self.detect_traffic_light(image)
        
        # 감지 여부에 따라 텍스트 출력
        if is_detected:
            print("✅ 초록불이 감지되었습니다! (출발 대기 중...)")
        else:
            print("❌ 초록불이 감지되지 않았습니다. 대기 중...")
            
    def detect_traffic_light(self, image):
        """
        초록불 신호등을 감지하는 함수 (대회 환경 최적화).
        """
        # 이미지의 상단 1/3을 관심 영역(ROI)으로 설정
        height, width, _ = image.shape
        roi = image[0:int(height / 3), 0:width]
        
        # BGR을 HSV 색공간으로 변환
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # 초록색 신호등에 최적화된 HSV 범위 설정
        lower_green = np.array([35, 100, 100])
        upper_green = np.array([85, 255, 255])
        
        # 초록색 마스크 생성
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # 노이즈 제거 (모폴로지 연산)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # 컨투어(윤곽선) 찾기
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        is_green_light_detected = False
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 200:
                is_green_light_detected = True
        
        return is_green_light_detected, image

    def get_stats(self):
        """웹 대시보드용 통계 데이터 반환"""
        lidar_distance = "N/A"
        if self.lidar_data is not None and len(self.lidar_data.ranges) > 0:
            center = len(self.lidar_data.ranges) // 2
            front_range = min(15, len(self.lidar_data.ranges) // 12)
            front_ranges = self.lidar_data.ranges[center-front_range:center+front_range]
            valid_ranges = [r for r in front_ranges if 0.05 < r < 10.0]
            if valid_ranges:
                lidar_distance = f"{min(valid_ranges):.2f}"
        
        return {
            "current_mode": self.current_mode.value,
            "rubbercon_status": "✅ PASSED" if self.rubbercon_passed else ("🚧 AVOIDING" if getattr(self, 'rubbercon_avoidance_active', False) else "🔍 SEARCHING"),
            "lane_status": "✅ DETECTED" if getattr(self, 'lane_detected', False) else "🔍 SEARCHING",
            "camera_fps": self.camera_fps,
            "lidar_distance": lidar_distance,
            "speed": f"{self.current_speed:.2f}",
            "steering_angle": f"{math.degrees(self.current_steering):.1f}",
        }

def main(args=None):
    rclpy.init(args=args)
    autoracer = Autoracer()
    rclpy.spin(autoracer)
    autoracer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
