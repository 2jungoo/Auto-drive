#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import threading
import time
import math
import json
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
                <title>Autoracer Dashboard</title>
                <style>
                    body { font-family: sans-serif; margin: 0; padding: 20px; background-color: #333; color: #fff; }
                    .container { max-width: 800px; margin: auto; padding: 20px; background-color: #444; border-radius: 10px; }
                    h1 { color: #4CAF50; }
                    #stats { display: flex; justify-content: space-around; margin-bottom: 20px; }
                    .stat { text-align: center; }
                    .stat-value { font-size: 2em; font-weight: bold; }
                    .stat-label { font-size: 0.8em; color: #ccc; }
                    #image-feed { width: 100%; border-radius: 5px; }
                </style>
            </head>
            <body>
                <div class="container">
                    <h1>Autoracer Status</h1>
                    <div id="stats">
                        <div class="stat"><div class="stat-value" id="current_mode"></div><div class="stat-label">Mode</div></div>
                        <div class="stat"><div class="stat-value" id="traffic_light"></div><div class="stat-label">Traffic Light</div></div>
                    </div>
                    <h2>Camera Feed</h2>
                    <img id="image-feed" src="/image_feed">
                </div>
                <script>
                    function updateStats() {
                        fetch('/stats').then(response => response.json()).then(data => {
                            document.getElementById('current_mode').innerText = data.current_mode;
                            document.getElementById('traffic_light').innerText = data.traffic_light_status;
                        });
                    }
                    setInterval(updateStats, 500);
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
            self.wfile.write(json.dumps(stats).encode('utf-8'))
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
        self.detected_color = "NONE"

        def run_server():
            server_address = ('', 8000)
            httpd = HTTPServer(server_address, lambda *args, **kwargs: WebViewer(self, *args, **kwargs))
            print("웹 대시보드 서버 시작. http://localhost:8000/ 에 접속하세요.")
            httpd.serve_forever()

        self.web_thread = threading.Thread(target=run_server)
        self.web_thread.daemon = True
        self.web_thread.start()

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        self.detected_color, processed_image = self.detect_traffic_light(image)
        
        if self.detected_color == "RED":
            print("🔴 빨간불이 감지되었습니다! (대기 중...)")
        elif self.detected_color == "ORANGE":
            print("🟠 주황불이 감지되었습니다! (대기 중...)")
        elif self.detected_color == "GREEN":
            # 초록불 감지 시 멘트 변경
            print("🟢 출발!")
        else:
            print("⚫ 신호등이 감지되지 않았습니다. (대기 중...)")

        ret, jpeg = cv2.imencode('.jpg', processed_image)
        self.last_image = jpeg.tobytes()

    def detect_traffic_light(self, image):
        height, width, _ = image.shape
        roi = image[0:int(height / 3), 0:width]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        lower_orange = np.array([10, 100, 100])
        upper_orange = np.array([25, 255, 255])
        
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
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 200:
                    x, y, w, h = cv2.boundingRect(contour)
                    if color == "RED":
                        cv2.rectangle(processed_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
                        cv2.putText(processed_image, "RED", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        detected_color = "RED"
                    elif color == "ORANGE":
                        cv2.rectangle(processed_image, (x, y), (x + w, y + h), (0, 165, 255), 2)
                        cv2.putText(processed_image, "ORANGE", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
                        detected_color = "ORANGE"
                    elif color == "GREEN":
                        cv2.rectangle(processed_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.putText(processed_image, "GREEN", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        detected_color = "GREEN"
                    
                    return detected_color, processed_image

        return detected_color, processed_image

    def get_stats(self):
        return {
            "current_mode": "Traffic Light Detection",
            "traffic_light_status": self.detected_color,
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
