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
            html = f"""
            <html>
            <head>
                <title>🚗 Autoracer 2025 Contest</title>
                <style>
                    body {{ background: linear-gradient(135deg, #1e1e1e, #000000); color: #f0f0f0; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 0; padding: 20px; }}
                    .container {{ max-width: 900px; margin: auto; background-color: #2a2a2a; padding: 20px; border-radius: 10px; box-shadow: 0 0 15px rgba(0,0,0,0.5); }}
                    h1, h2 {{ color: #4CAF50; border-bottom: 2px solid #4CAF50; padding-bottom: 5px; }}
                    .stats-grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 20px; margin-top: 20px; }}
                    .stat-box {{ background-color: #333; padding: 15px; border-radius: 8px; text-align: center; }}
                    .stat-value {{ font-size: 2em; font-weight: bold; color: #f0f0f0; }}
                    .stat-label {{ color: #aaa; font-size: 0.9em; margin-top: 5px; }}
                    #image-feed {{ margin-top: 20px; text-align: center; border: 2px solid #555; border-radius: 8px; }}
                </style>
            </head>
            <body>
                <div class="container">
                    <h1>🚗 Autoracer 2025 Dashboard</h1>
                    <div class="stats-grid">
                        <div class="stat-box"><div class="stat-value" id="current_mode">N/A</div><div class="stat-label">Current Mode</div></div>
                        <div class="stat-box"><div class="stat-value" id="rubbercon_status">N/A</div><div class="stat-label">Rubbercon</div></div>
                        <div class="stat-box"><div class="stat-value" id="lane_status">N/A</div><div class="stat-label">Lane Status</div></div>
                        <div class="stat-box"><div class="stat-value" id="traffic_light_status">N/A</div><div class="stat-label">Traffic Light</div></div>
                    </div>
                    <h2>Camera Feed</h2>
                    <img id="image-feed" src="/image_feed" alt="Camera Feed" width="100%">
                </div>
                <script>
                    function fetchStats() {{
                        fetch('/stats')
                            .then(response => response.json())
                            .then(data => {{
                                document.getElementById('current_mode').innerText = data.current_mode;
                                document.getElementById('rubbercon_status').innerText = data.rubbercon_status;
                                document.getElementById('lane_status').innerText = data.lane_status;
                                document.getElementById('traffic_light_status').innerText = data.traffic_light_status;
                            }});
                    }}
                    setInterval(fetchStats, 500);
                </script>
            </body>
            </html>
            """
            self.wfile.write(html.encode('utf-8'))
        elif self.path == '/image_feed':
            if self.autoracer.last_image is not None:
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(self.autoracer.last_image)))
                self.end_headers()
                self.wfile.write(self.autoracer.last_image)
            else:
                self.send_response(404)
                self.end_headers()
        elif self.path == '/stats':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            stats = self.autoracer.get_stats()
            self.wfile.write(bytes(str(stats).replace("'", '"'), 'utf-8'))
        else:
            self.send_response(404)
            self.end_headers()

class Autoracer(Node):
    def __init__(self):
        super().__init__('autoracer_node')
        self.qos_profile = QoSProfile(depth=10)
        self.qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        self.image_subscriber = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            self.qos_profile
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.twist_msg = Twist()
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0

        self.last_image = None
        self.last_image_time = time.time()
        self.camera_fps = 0
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)
        self.current_mode = DriveMode.EMERGENCY_STOP
        self.rubbercon_passed = False
        self.lidar_data = None
        self.current_speed = 0.0
        self.current_steering = 0.0
        self.detected_color = "NONE" # 신호등 감지 색상 저장 변수

        # 웹 대시보드 서버 시작
        def run_server():
            server_address = ('', 8000)
            httpd = HTTPServer(server_address, lambda *args, **kwargs: WebViewer(self, *args, **kwargs))
            print("웹 대시보드 서버 시작. http://localhost:8000/ 에 접속하세요.")
            httpd.serve_forever()

        self.web_thread = threading.Thread(target=run_server)
        self.web_thread.daemon = True
        self.web_thread.start()

    def publish_cmd_vel(self):
        """차량 제어 명령을 발행 (정지 상태 유지)"""
        self.cmd_vel_publisher.publish(self.twist_msg)

    def image_callback(self, msg):
        """
        카메라 이미지를 처리하는 콜백 함수.
        """
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        self.detected_color, processed_image = self.detect_traffic_light(image)
        
        # 감지된 색상에 따라 콘솔 출력
        if self.detected_color == "RED":
            print("🔴 빨간불이 감지되었습니다! (대기 중...)")
        elif self.detected_color == "ORANGE":
            print("🟠 주황불이 감지되었습니다! (대기 중...)")
        elif self.detected_color == "GREEN":
            print("🟢 초록불이 감지되었습니다! (출발 준비...)")
        else:
            print("⚫ 신호등이 감지되지 않았습니다. (대기 중...)")

        # 웹 대시보드에 이미지 전송
        ret, jpeg = cv2.imencode('.jpg', processed_image)
        self.last_image = jpeg.tobytes()

    def detect_traffic_light(self, image):
        """
        빨강, 주황, 초록 신호등을 감지하는 함수.
        """
        height, width, _ = image.shape
        roi = image[0:int(height / 3), 0:width]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # 빨강, 주황, 초록색의 HSV 범위
        # 빨강색은 H값이 0 근처와 180 근처에 분포
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        # 주황색 범위
        lower_orange = np.array([10, 100, 100])
        upper_orange = np.array([25, 255, 255])
        
        # 초록색 범위
        lower_green = np.array([35, 100, 100])
        upper_green = np.array([85, 255, 255])
        
        masks = {
            "RED": cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1), cv2.inRange(hsv, lower_red2, upper_red2)),
            "ORANGE": cv2.inRange(hsv, lower_orange, upper_orange),
            "GREEN": cv2.inRange(hsv, lower_green, upper_green),
        }
        
        detected_color = "NONE"
        processed_image = image.copy()
        
        for color, mask in masks.items():
            # 노이즈 제거
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 200:
                    x, y, w, h = cv2.boundingRect(contour)
                    # 감지된 색상에 따라 사각형과 텍스트 표시
                    if color == "RED":
                        cv2.rectangle(processed_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
                        cv2.putText(processed_image, "RED LIGHT", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        detected_color = "RED"
                    elif color == "ORANGE":
                        cv2.rectangle(processed_image, (x, y), (x + w, y + h), (0, 165, 255), 2)
                        cv2.putText(processed_image, "ORANGE LIGHT", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
                        detected_color = "ORANGE"
                    elif color == "GREEN":
                        cv2.rectangle(processed_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.putText(processed_image, "GREEN LIGHT", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        detected_color = "GREEN"
                    
                    return detected_color, processed_image

        return detected_color, processed_image

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
            "traffic_light_status": self.detected_color, # 감지된 색상 정보 추가
            "camera_fps": self.camera_fps,
            "lidar_distance": lidar_distance,
            "speed": f"{self.current_speed:.2f}",
            "steering_angle": f"{math.degrees(self.current_steering):.1f}",
        }

def main(args=None):
    rclpy.init(args=args)
    autoracer = Autoracer()
    try:
        rclpy.spin(autoracer)
    except KeyboardInterrupt:
        pass
    finally:
        autoracer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
