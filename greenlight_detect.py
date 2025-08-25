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
    WAITING_FOR_GREEN = "WAITING_FOR_GREEN"
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
                    <h1>🚦 Traffic Light Detection Dashboard</h1>
                    <div class="stats-grid">
                        <div class="stat-box"><div class="stat-value" id="current_mode">N/A</div><div class="stat-label">Current Mode</div></div>
                        <div class="stat-box"><div class="stat-value" id="traffic_light_status">N/A</div><div class="stat-label">Traffic Light</div></div>
                        <div class="stat-box"><div class="stat-value" id="confidence">N/A</div><div class="stat-label">Confidence</div></div>
                        <div class="stat-box"><div class="stat-value" id="fps">N/A</div><div class="stat-label">FPS</div></div>
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
                                document.getElementById('traffic_light_status').innerText = data.traffic_light_status;
                                document.getElementById('confidence').innerText = data.confidence + '%';
                                document.getElementById('fps').innerText = data.fps;
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

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')
        self.qos_profile = QoSProfile(depth=10)
        self.qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        # ROS2 구독자
        self.image_subscriber = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            self.qos_profile
        )
        
        # 제어 발행자 (실제로는 사용하지 않음)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # 상태 변수
        self.last_image = None
        self.last_processed_image = None
        self.detected_color = "NONE"
        self.confidence = 0.0
        self.frame_count = 0
        self.fps = 0.0
        self.last_time = time.time()
        
        # 신호등 검출 상태
        self.green_detected = False
        self.consecutive_green_frames = 0
        self.red_detected = False
        self.orange_detected = False
        
        # 현재 모드
        self.current_mode = DriveMode.WAITING_FOR_GREEN

        # OpenCV 윈도우 표시를 위한 스레드
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()

        # 웹 대시보드 서버 시작
        def run_server():
            server_address = ('', 8000)
            httpd = HTTPServer(server_address, lambda *args, **kwargs: WebViewer(self, *args, **kwargs))
            print("🌐 웹 대시보드: http://localhost:8000/")
            httpd.serve_forever()

        self.web_thread = threading.Thread(target=run_server, daemon=True)
        self.web_thread.start()

        self.get_logger().info("🚦 Traffic Light Detector with GStreamer Display Started!")

    def image_callback(self, msg):
        """카메라 이미지 처리 콜백"""
        try:
            # 이미지 디코딩
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if image is None:
                return

            # 신호등 검출 및 이미지 처리
            processed_image = self.process_image(image)
            
            # FPS 계산
            self.frame_count += 1
            current_time = time.time()
            if current_time - self.last_time >= 1.0:
                self.fps = self.frame_count / (current_time - self.last_time)
                self.frame_count = 0
                self.last_time = current_time

            # 처리된 이미지 저장 (웹과 gstreamer 모두 사용)
            ret, jpeg = cv2.imencode('.jpg', processed_image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ret:
                self.last_image = jpeg.tobytes()
                self.last_processed_image = processed_image.copy()

            # 콘솔 출력
            self.print_detection_status()

        except Exception as e:
            self.get_logger().error(f'이미지 처리 오류: {e}')

    def process_image(self, image):
        """이미지 처리 및 신호등 검출"""
        processed = image.copy()
        height, width = image.shape[:2]
        
        # 상태 헤더 그리기
        self.draw_status_overlay(processed)
        
        # 신호등 검출
        self.detected_color, self.confidence = self.detect_traffic_light(image, processed)
        
        # 검출 결과에 따른 상태 업데이트
        self.update_detection_state()
        
        return processed

    def draw_status_overlay(self, image):
        """상태 정보 오버레이"""
        height, width = image.shape[:2]
        
        # 반투명 배경
        overlay = image.copy()
        cv2.rectangle(overlay, (0, 0), (width, 120), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.75, image, 0.25, 0, image)
        
        # 상태에 따른 색상
        if self.detected_color == "GREEN":
            status_color = (0, 255, 0)
            status_text = "🟢 GREEN LIGHT DETECTED!"
        elif self.detected_color == "RED":
            status_color = (0, 0, 255)
            status_text = "🔴 RED LIGHT DETECTED"
        elif self.detected_color == "ORANGE":
            status_color = (0, 165, 255)
            status_text = "🟠 ORANGE LIGHT DETECTED"
        else:
            status_color = (128, 128, 128)
            status_text = "⚫ NO TRAFFIC LIGHT"
        
        # 텍스트 정보
        cv2.putText(image, f'🚦 Traffic Light Detection | {status_text}', 
                   (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        cv2.putText(image, f'Mode: {self.current_mode.value} | FPS: {self.fps:.1f}', 
                   (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(image, f'Confidence: {self.confidence:.1f}% | Consecutive Green: {self.consecutive_green_frames}', 
                   (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # 신호등 LED 시뮬레이션 (우상단)
        self.draw_traffic_light_led(image)

    def draw_traffic_light_led(self, image):
        """신호등 LED 시뮬레이션 그리기"""
        height, width = image.shape[:2]
        led_size = 25
        led_x = width - 60
        spacing = 40
        
        # 빨간불
        red_color = (0, 0, 255) if self.detected_color == "RED" else (50, 0, 0)
        cv2.circle(image, (led_x, 30), led_size, red_color, -1)
        cv2.circle(image, (led_x, 30), led_size, (255, 255, 255), 2)
        
        # 주황불
        orange_color = (0, 165, 255) if self.detected_color == "ORANGE" else (50, 50, 0)
        cv2.circle(image, (led_x, 30 + spacing), led_size, orange_color, -1)
        cv2.circle(image, (led_x, 30 + spacing), led_size, (255, 255, 255), 2)
        
        # 초록불
        green_color = (0, 255, 0) if self.detected_color == "GREEN" else (0, 50, 0)
        cv2.circle(image, (led_x, 30 + spacing * 2), led_size, green_color, -1)
        cv2.circle(image, (led_x, 30 + spacing * 2), led_size, (255, 255, 255), 2)

    def detect_traffic_light(self, image, processed_image):
        """개선된 신호등 검출"""
        height, width, _ = image.shape
        
        # ROI 설정 (화면 상단 1/3)
        roi = image[0:int(height / 3), 0:width]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # 색상별 HSV 범위 (더 정확하게 조정)
        color_ranges = {
            "RED": [
                (np.array([0, 120, 120]), np.array([10, 255, 255])),     # 빨강 1
                (np.array([170, 120, 120]), np.array([180, 255, 255]))   # 빨강 2
            ],
            "ORANGE": [
                (np.array([10, 150, 150]), np.array([25, 255, 255]))     # 주황
            ],
            "GREEN": [
                (np.array([40, 120, 120]), np.array([80, 255, 255])),    # 기본 녹색
                (np.array([50, 150, 180]), np.array([70, 255, 255]))     # LED 녹색
            ]
        }
        
        best_detection = {"color": "NONE", "confidence": 0.0, "area": 0}
        
        for color, ranges in color_ranges.items():
            # 여러 범위 마스크 결합
            combined_mask = None
            for lower, upper in ranges:
                mask = cv2.inRange(hsv, lower, upper)
                if combined_mask is None:
                    combined_mask = mask
                else:
                    combined_mask = cv2.bitwise_or(combined_mask, mask)
            
            # 노이즈 제거
            kernel = np.ones((5, 5), np.uint8)
            combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel, iterations=2)
            combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel, iterations=1)
            
            # 컨투어 찾기
            contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 150:  # 최소 면적
                    x, y, w, h = cv2.boundingRect(contour)
                    aspect_ratio = float(w) / h
                    
                    # 원형도 계산
                    perimeter = cv2.arcLength(contour, True)
                    if perimeter > 0:
                        circularity = 4 * math.pi * area / (perimeter * perimeter)
                    else:
                        circularity = 0
                    
                    # 신호등 조건 검사
                    if (0.5 < aspect_ratio < 2.0 and    # 원형/정사각형
                        circularity > 0.3 and           # 원형에 가까움
                        area > 200):                     # 충분한 면적
                        
                        # 밝기 확인
                        roi_brightness = cv2.mean(roi[y:y+h, x:x+w])[0]
                        confidence = area * circularity * (roi_brightness / 255.0) * 0.1
                        
                        # 가장 좋은 검출 결과 업데이트
                        if confidence > best_detection["confidence"]:
                            best_detection = {
                                "color": color,
                                "confidence": confidence,
                                "area": area,
                                "bbox": (x, y, w, h),
                                "brightness": roi_brightness
                            }
        
        # 검출 결과 시각화
        if best_detection["color"] != "NONE":
            x, y, w, h = best_detection["bbox"]
            
            # 색상에 따른 표시
            if best_detection["color"] == "RED":
                color_bgr = (0, 0, 255)
                text = "RED LIGHT"
            elif best_detection["color"] == "ORANGE":
                color_bgr = (0, 165, 255)
                text = "ORANGE LIGHT"
            elif best_detection["color"] == "GREEN":
                color_bgr = (0, 255, 0)
                text = "GREEN LIGHT"
            
            # 사각형과 텍스트 그리기
            cv2.rectangle(processed_image, (x, y), (x + w, y + h), color_bgr, 3)
            cv2.putText(processed_image, text, (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)
            
            # 신뢰도 표시
            cv2.putText(processed_image, f'Conf: {best_detection["confidence"]:.1f}', 
                       (x, y + h + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 1)
            
            # 중심점 표시
            center_x, center_y = x + w//2, y + h//2
            cv2.circle(processed_image, (center_x, center_y), 5, color_bgr, -1)
        
        return best_detection["color"], best_detection["confidence"]

    def update_detection_state(self):
        """검출 상태 업데이트"""
        if self.detected_color == "GREEN":
            self.consecutive_green_frames += 1
            if self.consecutive_green_frames >= 5:  # 5프레임 연속 검출
                self.green_detected = True
                self.current_mode = DriveMode.RUBBERCON_AVOIDANCE
        else:
            self.consecutive_green_frames = max(0, self.consecutive_green_frames - 1)
            if self.consecutive_green_frames == 0:
                self.green_detected = False
        
        # 다른 색상 검출
        self.red_detected = (self.detected_color == "RED")
        self.orange_detected = (self.detected_color == "ORANGE")

    def display_loop(self):
        """GStreamer 창 표시 루프"""
        self.get_logger().info("🖼️ OpenCV 디스플레이 창 시작...")
        
        while rclpy.ok():
            try:
                if self.last_processed_image is not None:
                    # 창 크기 조정
                    display_image = cv2.resize(self.last_processed_image, (1024, 768))
                    cv2.imshow('🚦 Traffic Light Detection', display_image)
                    
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q') or key == 27:  # 'q' 또는 ESC
                        break
                    elif key == ord('r'):  # 리셋
                        self.reset_detection()
                        
                time.sleep(0.033)  # ~30 FPS
                
            except Exception as e:
                self.get_logger().error(f'디스플레이 오류: {e}')
                break
        
        cv2.destroyAllWindows()

    def reset_detection(self):
        """검출 상태 리셋"""
        self.detected_color = "NONE"
        self.confidence = 0.0
        self.consecutive_green_frames = 0
        self.green_detected = False
        self.red_detected = False
        self.orange_detected = False
        self.current_mode = DriveMode.WAITING_FOR_GREEN
        self.get_logger().info("🔄 검출 상태 리셋!")

    def print_detection_status(self):
        """검출 상태 콘솔 출력 (5초마다)"""
        if not hasattr(self, 'last_print_time'):
            self.last_print_time = 0
        
        current_time = time.time()
        if current_time - self.last_print_time >= 5.0:
            if self.detected_color == "RED":
                self.get_logger().info("🔴 빨간불 감지됨 - 대기 중...")
            elif self.detected_color == "ORANGE":
                self.get_logger().info("🟠 주황불 감지됨 - 대기 중...")
            elif self.detected_color == "GREEN":
                self.get_logger().info("🟢 초록불 감지됨 - 출발 준비!")
            else:
                self.get_logger().info("⚫ 신호등을 찾고 있습니다...")
            
            self.last_print_time = current_time

    def get_stats(self):
        """웹 대시보드용 통계 반환"""
        return {
            "current_mode": self.current_mode.value,
            "traffic_light_status": self.detected_color,
            "confidence": f"{self.confidence:.1f}",
            "fps": f"{self.fps:.1f}",
        }

def main(args=None):
    rclpy.init(args=args)
    
    try:
        detector = TrafficLightDetector()
        
        print("""
╔═══════════════════════════════════════════════╗
║        🚦 TRAFFIC LIGHT DETECTOR               ║
╠═══════════════════════════════════════════════╣
║  • OpenCV 창과 웹 대시보드에서 결과 확인       ║
║  • 'q' 또는 ESC: 종료                         ║  
║  • 'r': 검출 상태 리셋                        ║
║  • 웹 대시보드: http://localhost:8000/        ║
╚═══════════════════════════════════════════════╝
        """)
        
        rclpy.spin(detector)
        
    except KeyboardInterrupt:
        print("\n🏁 Traffic Light Detector 종료!")
    finally:
        if 'detector' in locals():
            detector.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
