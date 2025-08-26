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
import json
from http.server import BaseHTTPRequestHandler, HTTPServer
from enum import Enum

qos_profile = QoSProfile(depth=10)
qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

class DriveMode(Enum):
    TRAFFIC_LIGHT_WAIT = "TRAFFIC_LIGHT_WAIT"
    RUBBERCON_AVOIDANCE = "RUBBERCON_AVOID"
    LANE_FOLLOWING = "LANE_FOLLOW"
    OBSTACLE_CAR_AVOIDANCE = "OBSTACLE_CAR_AVOID"
    EMERGENCY_STOP = "EMERGENCY_STOP"

class WebViewer(BaseHTTPRequestHandler):
    # (WebViewer 클래스 코드는 변경하지 않았습니다. 기존 코드를 그대로 사용하시면 됩니다.)
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
                <title>🏁 Autoracer 2025 Contest - Enhanced with Traffic Light & Obstacle Avoidance</title>
                <style>
                    body { background: linear-gradient(135deg, #1e1e1e, #2d2d30); color: #fff; font-family: 'Segoe UI', Arial; margin: 0; padding: 20px; }
                    .container { display: flex; gap: 20px; max-width: 1400px; margin: 0 auto; }
                    .panel { background: rgba(255,255,255,0.1); border-radius: 12px; padding: 20px; backdrop-filter: blur(10px); }
                    .status-card { background: rgba(0,255,0,0.1); border-left: 4px solid #00ff00; margin: 10px 0; padding: 15px; border-radius: 8px; }
                    .metric { display: flex; justify-content: space-between; margin: 8px 0; }
                    .metric-value { font-weight: bold; color: #00ff88; }
                    h1 { text-align: center; color: #00ff88; text-shadow: 0 0 20px #00ff88; }
                    h3 { color: #00ccff; margin-top: 0; }
                    .progress-bar { width: 100%; height: 20px; background: #333; border-radius: 10px; overflow: hidden; margin: 10px 0; }
                    .progress-fill { height: 100%; background: linear-gradient(90deg, #00ff00, #00ccff); transition: width 0.3s; }
                </style>
            </head>
            <body>
                <h1>🏆 Autoracer 2025 Contest - Enhanced with Traffic Light & Car Avoidance</h1>
                <div class="container">
                    <div class="panel" style="flex: 2;">
                        <h3>📹 Live Camera Feed</h3>
                        <img src="/stream.mjpg" width="800" height="600" style="border: 2px solid #444; border-radius: 8px; width: 100%; max-width: 800px;">
                    </div>
                    <div class="panel" style="flex: 1;">
                        <h3>🎯 Mission Control</h3>
                        <div class="status-card">
                            <div class="metric"><span>Current Mode:</span><span id="mode" class="metric-value">Loading...</span></div>
                            <div class="metric"><span>Traffic Light:</span><span id="traffic_light" class="metric-value">Searching...</span></div>
                            <div class="metric"><span>Rubbercon Status:</span><span id="rubbercon" class="metric-value">Searching...</span></div>
                            <div class="metric"><span>Lane Status:</span><span id="lane" class="metric-value">Detecting...</span></div>
                            <div class="metric"><span>Obstacle Car:</span><span id="obstacle_car" class="metric-value">None...</span></div>
                            <div class="metric"><span>Detection Confidence:</span><span id="confidence" class="metric-value">0%</span></div>
                        </div>
                        
                        <h3>📊 Vehicle Telemetry</h3>
                        <div class="status-card">
                            <div class="metric"><span>Camera FPS:</span><span id="camera_fps" class="metric-value">0</span></div>
                            <div class="metric"><span>Lidar Distance:</span><span id="lidar_dist" class="metric-value">N/A</span> m</div>
                            <div class="metric"><span>Speed:</span><span id="speed" class="metric-value">0</span> m/s</div>
                            <div class="metric"><span>Steering:</span><span id="steering" class="metric-value">0</span>°</div>
                            <div class="metric"><span>Mission Time:</span><span id="mission_time" class="metric-value">00:00</span></div>
                        </div>
                        
                        <h3>🏁 Mission Progress</h3>
                        <div class="progress-bar">
                            <div id="progress" class="progress-fill" style="width: 0%;"></div>
                        </div>
                        <div style="font-size: 14px;">
                            <p id="mission1" style="color: #ffaa00;">🚦 Mission 1: Traffic Light Wait</p>
                            <p id="mission2" style="color: #666;">🔴 Mission 2: Rubbercon Avoidance</p>
                            <p id="mission3" style="color: #666;">🛣️ Mission 3: Lane Following</p>
                            <p id="mission4" style="color: #666;">🚗 Mission 4: Obstacle Car Avoidance</p>
                        </div>
                        
                        <h3>⚠️ System Alerts</h3>
                        <div id="alerts" style="background: rgba(255,0,0,0.1); border-radius: 8px; padding: 15px; min-height: 50px;">
                            <span style="color: #aaa;">System Normal</span>
                        </div>
                    </div>
                </div>
                
                <script>
                let startTime = Date.now();
                setInterval(() => {
                    fetch('/stats')
                    .then(r => r.json())
                    .then(data => {
                        document.getElementById('mode').textContent = data.current_mode;
                        document.getElementById('traffic_light').textContent = data.traffic_light_status;
                        document.getElementById('rubbercon').textContent = data.rubbercon_status;
                        document.getElementById('lane').textContent = data.lane_status;
                        document.getElementById('obstacle_car').textContent = data.obstacle_car_status;
                        document.getElementById('confidence').textContent = data.detection_confidence + '%';
                        document.getElementById('camera_fps').textContent = data.camera_fps;
                        document.getElementById('lidar_dist').textContent = data.lidar_distance;
                        document.getElementById('speed').textContent = data.speed;
                        document.getElementById('steering').textContent = data.steering_angle;
                        
                        let elapsed = Math.floor((Date.now() - startTime) / 1000);
                        let mins = Math.floor(elapsed / 60);
                        let secs = elapsed % 60;
                        document.getElementById('mission_time').textContent = 
                            `${mins.toString().padStart(2,'0')}:${secs.toString().padStart(2,'0')}`;
                        
                        let progress = 0;
                        if (data.current_mode.includes('TRAFFIC')) progress = 0;
                        else if (data.current_mode.includes('RUBBERCON')) progress = 25;
                        else if (data.current_mode.includes('LANE')) progress = 50;
                        else if (data.current_mode.includes('OBSTACLE')) progress = 75;
                        if(data.lap_counter > 0) progress = 100;
                        document.getElementById('progress').style.width = progress + '%';
                        
                        const missions = ['mission1', 'mission2', 'mission3', 'mission4'];
                        missions.forEach((m, i) => {
                            const elem = document.getElementById(m);
                            if (i * 25 < progress) elem.style.color = '#00ff00';
                            else if (i * 25 === progress - 25) elem.style.color = '#ffaa00';
                        });
                        
                        let alerts = [];
                        if (data.lidar_distance < 0.3 && data.lidar_distance !== 'N/A') alerts.push('⚠️ Obstacle Too Close');
                        if (data.camera_fps < 5 && data.camera_fps > 0) alerts.push('📹 Low Camera FPS');
                        if (Math.abs(data.steering_angle) > 20) alerts.push('🎯 Sharp Turn');
                        
                        document.getElementById('alerts').innerHTML = 
                            alerts.length > 0 ? alerts.join('<br>') : '<span style="color: #aaa;">System Normal</span>';
                    }).catch(e => console.log('Stats error:', e));
                }, 500);
                </script>
            </body>
            </html>
            """
            self.wfile.write(html.encode())
            
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            
            try:
                while True:
                    frame = self.autoracer.get_processed_frame()
                    if frame is not None:
                        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                        
                        self.wfile.write(b'--frame\r\n')
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', str(len(buffer)))
                        self.end_headers()
                        self.wfile.write(buffer)
                        self.wfile.write(b'\r\n')
                    
                    time.sleep(0.033)  # ~30 FPS
                    
            except Exception as e:
                self.autoracer.get_logger().error(f'Streaming error: {e}')
                
        elif self.path == '/stats':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            
            stats = self.autoracer.get_stats()
            self.wfile.write(json.dumps(stats).encode())
        else:
            self.send_response(404)
            self.end_headers()

class Autoracer(Node):
    def __init__(self):
        super().__init__('Autoracer')
        
        # 이미지 및 센서 데이터
        self.current_image = None
        self.processed_frame = None
        self.lidar_data = None
        self.image_lock = threading.Lock()
        
        # 미션 상태 관리 - 2025년 확장
        self.current_mode = DriveMode.TRAFFIC_LIGHT_WAIT  # 신호등 대기로 시작
        self.traffic_light_passed = False
        self.rubbercon_passed = False
        self.lane_following_started = False
        self.lane_detected = False
        self.one_lap_completed = False
        
        # 제어 변수
        self.current_speed = 0.0
        self.current_steering = 0.0
        self.target_speed = 0.0
        self.target_steering = 0.0
        
        # PID 제어 변수
        self.prev_error = 0.0
        self.integral_error = 0.0
        self.last_lane_center = 320
        
        # 신호등 상태 - NEW for 2025
        self.traffic_light_state = "SEARCHING"
        self.green_light_detected_time = None
        self.green_light_detection_count = 0
        self.traffic_light_confidence = 0.0
        
        # 방해차량 회피 상태 - NEW for 2025
        self.obstacle_car_detected = False
        self.obstacle_car_position = None  # "left", "right", "center"
        self.obstacle_avoidance_active = False
        self.obstacle_avoidance_start_time = None
        self.obstacle_car_confidence = 0.0
        
        # 라바콘 회피 상태
        self.rubbercon_detection_count = 0
        self.rubbercon_avoidance_active = False
        self.rubbercon_clear_count = 0
        self.rubbercon_detection_flag = 0
        self.no_rubbercon_frames = 0
        self.rubbercon_status = "SEARCHING"

        # 통계 및 성능 데이터
        self.frame_count = 0
        self.start_time = time.time()
        self.last_camera_time = 0
        self.camera_fps = 0
        self.mission_start_time = time.time()
        self.detection_confidence = 0.0
        
        # Bird's Eye View 변환 행렬
        self.bev_matrix = None
        self.inv_bev_matrix = None
        self.setup_bev_transform()
        
        # 차선 검출용 슬라이딩 윈도우
        self.left_lane_pixels = []
        self.right_lane_pixels = []
        self.lane_confidence = 0.0
        self.prev_left_base = 160
        self.prev_right_base = 480
        
        # 주행 횟수 카운터 - NEW for 2025
        self.lap_counter = 0
        self.start_line_passed = False
        
        # ROS2 구독자
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10
        )
        
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile
        )
        
        # ROS2 발행자
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # 웹 서버 시작
        self.web_port = 8080
        self.start_web_server()
        
        # 제어 루프 타이머
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20Hz
        
        self.get_logger().info('🏆 Autoracer 2025 Contest Started - Enhanced with Traffic Light & Obstacle Avoidance!')

    def setup_bev_transform(self):
        """Bird's Eye View 변환 행렬 설정 - 2023년 최적화"""
        src_points = np.float32([
            [80, 480],    # 좌하단  
            [560, 480],   # 우하단
            [240, 280],   # 좌상단 - 더 넓은 시야각
            [400, 280]    # 우상단 - 더 넓은 시야각
        ])
        
        dst_points = np.float32([
            [150, 480],   # 좌하단
            [490, 480],   # 우하단
            [150, 0],     # 좌상단
            [490, 0]      # 우상단
        ])
        
        self.bev_matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        self.inv_bev_matrix = cv2.getPerspectiveTransform(dst_points, src_points)

    def get_ip_address(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return "localhost"

    def start_web_server(self):
        def create_handler(*args, **kwargs):
            return WebViewer(self, *args, **kwargs)
        
        def run_server():
            for port in range(8080, 8090):
                try:
                    server = HTTPServer(('0.0.0.0', port), create_handler)
                    self.web_port = port
                    self.get_logger().info(f'🌐 Web server is running on http://{self.get_ip_address()}:{port}/')
                    server.serve_forever()
                    break
                except OSError as e:
                    if e.errno == 98: # Address already in use
                        self.get_logger().warn(f'Port {port} is busy, trying next one.')
                        continue
                    else:
                        self.get_logger().error(f'Web server error: {e}')
                        break
        
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if image is not None:
                with self.image_lock:
                    self.current_image = image.copy()
                
                self.process_image(image)
                
                # FPS 계산
                self.frame_count += 1
                current_time = time.time()
                if self.last_camera_time > 0:
                    fps = 1.0 / (current_time - self.last_camera_time)
                    self.camera_fps = round(fps, 1)
                self.last_camera_time = current_time
                
        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

    def process_image(self, image):
        processed = image.copy()
        
        # 미션별 처리
        if self.current_mode == DriveMode.TRAFFIC_LIGHT_WAIT:
            self.detect_traffic_light(processed)
        elif self.current_mode == DriveMode.RUBBERCON_AVOIDANCE:
            self.detect_and_avoid_rubbercon_enhanced(processed)
        elif self.current_mode == DriveMode.LANE_FOLLOWING:
            self.detect_lane_enhanced_2023(processed)
            # 차선 추종 중 방해차량 감지
            if self.detect_obstacle_car(processed):
                self.current_mode = DriveMode.OBSTACLE_CAR_AVOIDANCE
                self.get_logger().info('🚗 Obstacle Car Detected! Switching to Avoidance Mode.')
        elif self.current_mode == DriveMode.OBSTACLE_CAR_AVOIDANCE:
            self.avoid_obstacle_car(processed)
        
        # 라이다 오버레이
        if self.lidar_data is not None:
            self.draw_lidar_overlay(processed)
            
        # 상태 정보 헤더 (가장 마지막에 그려야 다른 그림들을 덮지 않음)
        self.draw_status_header(processed)

        with self.image_lock:
            self.processed_frame = processed.copy()

    def draw_status_header(self, image):
        """상태 정보 헤더 그리기"""
        height, width = image.shape[:2]
        
        # 반투명 배경
        overlay = image.copy()
        cv2.rectangle(overlay, (0, 0), (width, 140), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)
        
        # 미션 경과 시간
        elapsed = time.time() - self.mission_start_time
        time_str = f"{int(elapsed//60):02d}:{int(elapsed%60):02d}"
        
        # 텍스트 정보
        cv2.putText(image, f'🏁 Enhanced 2025 | Mode: {self.current_mode.value}', 
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(image, f'⏱️ Time: {time_str} | Frame: {self.frame_count} | FPS: {self.camera_fps}', 
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(image, f'🚗 Speed: {self.current_speed:.2f} | 🎯 Steer: {math.degrees(self.current_steering):.1f}° | Conf: {self.detection_confidence:.1f}%', 
                    (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(image, f'🏁 Laps: {self.lap_counter} | 🚦 Light: {self.traffic_light_state or "None"} | 🚗 Obstacle: {self.obstacle_car_position or "None"}', 
                    (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

    def detect_traffic_light(self, image):
        """신호등 녹색불 감지 - NEW for 2025"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 녹색 신호등 HSV 범위 설정
        lower_green1 = np.array([40, 50, 50])
        upper_green1 = np.array([80, 255, 255])
        lower_green2 = np.array([35, 100, 100])
        upper_green2 = np.array([85, 255, 255])
        
        # 마스크 생성 및 결합
        green_mask1 = cv2.inRange(hsv, lower_green1, upper_green1)
        green_mask2 = cv2.inRange(hsv, lower_green2, upper_green2)
        green_mask = cv2.bitwise_or(green_mask1, green_mask2)
        
        # 노이즈 제거
        kernel = np.ones((5, 5), np.uint8)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel, iterations=2)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        
        # 관심 영역 설정 (화면 상단 1/3)
        height, width = image.shape[:2]
        roi_mask = np.zeros_like(green_mask)
        roi_mask[0:height//3, :] = 255
        green_mask = cv2.bitwise_and(green_mask, roi_mask)
        
        # 컨투어 검출
        contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        green_lights = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 50:  # 최소 면적
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w) / h
                
                if 0.7 < aspect_ratio < 1.3 and area > 100:
                    perimeter = cv2.arcLength(contour, True)
                    if perimeter > 0:
                        circularity = 4 * math.pi * area / (perimeter * perimeter)
                        if circularity > 0.5:
                            green_lights.append({'x': x, 'y': y, 'w': w, 'h': h, 'area': area, 'confidence': area * circularity})
                            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 3)
                            cv2.putText(image, f'GREEN LIGHT({area:.0f})', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if len(green_lights) > 0:
            best_light = max(green_lights, key=lambda x: x['confidence'])
            self.traffic_light_confidence = min(100, best_light['confidence'] / 20)
            self.green_light_detection_count += 1
            
            if self.green_light_detection_count > 15:
                if self.green_light_detected_time is None:
                    self.green_light_detected_time = time.time()
                    self.get_logger().info('🟢 Green light confirmed! Starting sequence...')
                
                if time.time() - self.green_light_detected_time > 1.0:
                    self.traffic_light_passed = True
                    self.current_mode = DriveMode.RUBBERCON_AVOIDANCE
                    self.traffic_light_state = "GREEN_PASSED"
                    self.get_logger().info('🚦 Traffic light mission passed! Moving to rubbercon avoidance.')
            
            self.traffic_light_state = "GREEN"
        else:
            self.green_light_detection_count = max(0, self.green_light_detection_count - 1)
            self.traffic_light_confidence = 0
            if self.green_light_detection_count == 0:
                self.traffic_light_state = "RED_OR_SEARCHING"
        
        if not self.traffic_light_passed:
            self.target_speed = 0.0
            self.target_steering = 0.0

    def detect_obstacle_car(self, image):
        """방해차량 감지 - NEW for 2025"""
        height, width = image.shape[:2]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 다양한 차량 색상 감지
        lower_white = np.array([0, 0, 180])
        upper_white = np.array([180, 30, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        
        lower_dark = np.array([0, 0, 0])
        upper_dark = np.array([180, 255, 80])
        dark_mask = cv2.inRange(hsv, lower_dark, upper_dark)

        # 모든 마스크 결합
        car_mask = cv2.bitwise_or(white_mask, dark_mask)
        
        kernel_large = np.ones((7, 7), np.uint8)
        car_mask = cv2.morphologyEx(car_mask, cv2.MORPH_CLOSE, kernel_large, iterations=2)
        
        # 관심 영역 설정
        roi_mask = np.zeros_like(car_mask)
        roi_mask[height//3:height*3//4, width//4:width*3//4] = 255
        car_mask = cv2.bitwise_and(car_mask, roi_mask)
        
        contours, _ = cv2.findContours(car_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        obstacle_cars = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000: # 최소 면적
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w) / h
                
                if 0.8 < aspect_ratio < 4.0 and w > 40 and h > 30:
                    center_x = x + w // 2
                    position = "center"
                    if center_x < width // 3: position = "left"
                    elif center_x > width * 2 // 3: position = "right"
                    
                    obstacle_cars.append({'x': x, 'y': y, 'w': w, 'h': h, 'center_x': center_x, 'area': area, 'position': position, 'confidence': area})
                    
                    color = (0, 0, 255)
                    cv2.rectangle(image, (x, y), (x + w, y + h), color, 3)
                    cv2.putText(image, f'CAR-{position.upper()}({area:.0f})', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        if len(obstacle_cars) > 0:
            best_car = max(obstacle_cars, key=lambda x: x['confidence'])
            self.obstacle_car_confidence = min(100, best_car['confidence'] / 100)
            
            if self.obstacle_car_confidence > 50:
                self.obstacle_car_detected = True
                self.obstacle_car_position = best_car['position']
                self.detection_confidence = self.obstacle_car_confidence
                return True
        
        self.obstacle_car_confidence = 0
        self.obstacle_car_detected = False
        self.obstacle_car_position = None
        return False

    def avoid_obstacle_car(self, image):
        """방해차량 회피 주행 - NEW for 2025"""
        if not self.obstacle_avoidance_active:
            self.obstacle_avoidance_active = True
            self.obstacle_avoidance_start_time = time.time()
            self.get_logger().info(f'🚗 Starting car avoidance maneuver. Car position: {self.obstacle_car_position}')
        
        # 회피 중에도 차선과 차량을 계속 감지
        self.detect_lane_enhanced_2023(image)
        car_still_detected = self.detect_obstacle_car(image)
        
        # 회피 조향 로직
        avoid_steer = -0.5  # 기본적으로 왼쪽으로 회피
        if self.obstacle_car_position == "left":
            avoid_steer = 0.5 # 좌측에 있으면 우측으로
        
        self.target_steering = avoid_steer
        self.target_speed = 0.25 # 감속하여 회피
        
        cv2.putText(image, f"AVOIDING CAR: Steering {avoid_steer}", (10, image.shape[0]-30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # 회피 종료 조건
        if self.obstacle_avoidance_active:
            elapsed_time = time.time() - self.obstacle_avoidance_start_time
            # 차량이 안보이고, 최소 회피 시간을 넘겼을 때
            if not car_still_detected and elapsed_time > 3.0:
                self.obstacle_avoidance_active = False
                self.current_mode = DriveMode.LANE_FOLLOWING
                self.get_logger().info('✅ Obstacle avoidance complete. Resuming lane following.')


    def detect_and_avoid_rubbercon_enhanced(self, image):
        """라바콘 검출 및 회피 - 2023년 우승 알고리즘 기반 개선"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        lower_orange1 = np.array([5, 100, 100])
        upper_orange1 = np.array([25, 255, 255])
        orange_mask = cv2.inRange(hsv, lower_orange1, upper_orange1)
        
        kernel_large = np.ones((7, 7), np.uint8)
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel_large, iterations=2)
        
        height, width = image.shape[:2]
        roi_mask = np.zeros_like(orange_mask)
        roi_mask[int(height*0.4):height, :] = 255
        orange_mask = cv2.bitwise_and(orange_mask, roi_mask)
        
        contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        rubbercons = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 200:
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w) / h
                if 0.3 < aspect_ratio < 1.5 and h > 20:
                    center_x = x + w // 2
                    rubbercons.append({'center_x': center_x, 'area': area})
                    cv2.rectangle(image, (x, y), (x+w, y+h), (0, 165, 255), 2)
                    cv2.putText(image, f'CONE({area:.0f})', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)

        if len(rubbercons) > 0:
            self.no_rubbercon_frames = 0
            self.rubbercon_status = f"DETECTED: {len(rubbercons)}"
            self.detection_confidence = min(100, len(rubbercons) * 25)

            left_cones = [c for c in rubbercons if c['center_x'] < width // 2]
            right_cones = [c for c in rubbercons if c['center_x'] > width // 2]

            target_x = width // 2
            if left_cones and right_cones:
                left_most = min(left_cones, key=lambda c: c['center_x'])
                right_most = max(right_cones, key=lambda c: c['center_x'])
                target_x = (left_most['center_x'] + right_most['center_x']) // 2
            elif left_cones:
                # 가장 오른쪽에 있는 왼쪽 콘을 기준으로 오른쪽으로 회피
                closest_cone = max(left_cones, key=lambda c: c['center_x'])
                target_x = closest_cone['center_x'] + 150  
            elif right_cones:
                # 가장 왼쪽에 있는 오른쪽 콘을 기준으로 왼쪽으로 회피
                closest_cone = min(right_cones, key=lambda c: c['center_x'])
                target_x = closest_cone['center_x'] - 150
            
            error = target_x - (width // 2)
            self.calculate_steering_control(error, "RUBBERCON")
            cv2.circle(image, (target_x, height-50), 10, (255,0,255), -1)
            cv2.line(image, (width//2, height), (target_x, height-50), (255,0,255), 2)

        else:
            self.no_rubbercon_frames += 1
            self.detection_confidence = 0
            # 10프레임 이상 라바콘이 안보이면 통과로 간주
            if self.no_rubbercon_frames > 10:
                self.rubbercon_passed = True
                self.current_mode = DriveMode.LANE_FOLLOWING
                self.rubbercon_status = "PASSED"
                self.get_logger().info('✅ Rubbercon mission passed! Switching to lane following.')

    # AuTURBO팀의 차선 인식 로직을 반영하여 수정
    def detect_lane_enhanced_2023(self, image):
        """AuTURBO팀 로직을 적용하여 차선 인식 개선"""
        if self.bev_matrix is None:
            bev_image = image.copy()
        else:
            bev_image = cv2.warpPerspective(image, self.bev_matrix, (640, 480))
        
        # HSV + Adaptive Thresholding 기반 차선 마스크 생성
        hsv_bev = cv2.cvtColor(bev_image, cv2.COLOR_BGR2HSV)
        
        # 흰색 차선 감지 (HSV)
        lower_white = np.array([0, 0, 180])
        upper_white = np.array([180, 30, 255])
        white_mask = cv2.inRange(hsv_bev, lower_white, upper_white)
        
        # 노란색 차선 감지 (HSV)
        lower_yellow = np.array([20, 80, 80])
        upper_yellow = np.array([35, 255, 255])
        yellow_mask = cv2.inRange(hsv_bev, lower_yellow, upper_yellow)
        
        # 그레이스케일 및 Adaptive Thresholding (조명 변화에 강인)
        gray_bev = cv2.cvtColor(bev_image, cv2.COLOR_BGR2GRAY)
        adaptive_mask = cv2.adaptiveThreshold(gray_bev, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                              cv2.THRESH_BINARY, 15, -10)
        
        # 모든 마스크 결합
        lane_mask = cv2.bitwise_or(white_mask, yellow_mask)
        lane_mask = cv2.bitwise_or(lane_mask, adaptive_mask)
        
        # Morphology를 사용한 노이즈 제거 및 차선 연결
        kernel = np.ones((5, 5), np.uint8)
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_OPEN, kernel)
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, kernel)
        
        left_line, right_line, lane_center = self.sliding_window_lane_detection_2023(lane_mask)
        
        self.draw_lane_overlay_2023(image, bev_image, lane_mask, left_line, right_line, lane_center)
        
        # 차선 중심 계산 및 제어
        if lane_center is not None:
            self.lane_detected = True
            image_center = image.shape[1] // 2
            steering_error = lane_center - image_center
            self.last_lane_center = lane_center
            self.calculate_steering_control(steering_error, "LANE")
        else:
            self.lane_detected = False
            steering_error = self.last_lane_center - (image.shape[1] // 2)
            self.calculate_steering_control(steering_error * 0.8, "LANE_LOST")

    # AuTURBO팀의 슬라이딩 윈도우 로직으로 개선
    def sliding_window_lane_detection_2023(self, binary_image):
        """AuTURBO팀의 개선된 슬라이딩 윈도우 로직"""
        height, width = binary_image.shape
        
        # 히스토그램 생성 및 스무딩
        histogram = np.sum(binary_image[height * 2 // 3:, :], axis=0)
        histogram = np.convolve(histogram, np.ones(10)/10, mode='same')
        
        midpoint = width // 2
        
        # 차선 시작점 찾기 (이전 위치 기반)
        left_base = self.find_lane_base(histogram[:midpoint], self.prev_left_base, True)
        right_base = self.find_lane_base(histogram[midpoint:], self.prev_right_base - midpoint, False) + midpoint
        
        nwindows = 12
        window_height = height // nwindows
        margin = 60
        minpix = 30
        
        leftx_current = left_base
        rightx_current = right_base
        
        left_lane_inds_list = []
        right_lane_inds_list = []
        
        # Nonzero 픽셀 한번만 계산
        nonzero = binary_image.nonzero()
        nonzero_y = np.array(nonzero[0])
        nonzero_x = np.array(nonzero[1])
        
        for window in range(nwindows):
            win_y_low = height - (window + 1) * window_height
            win_y_high = height - window * window_height
            
            win_xleft_low = max(0, leftx_current - margin)
            win_xleft_high = min(width, leftx_current + margin)
            
            win_xright_low = max(0, rightx_current - margin)
            win_xright_high = min(width, rightx_current + margin)
            
            good_left_inds = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
                              (nonzero_x >= win_xleft_low) & (nonzero_x < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
                               (nonzero_x >= win_xright_low) & (nonzero_x < win_xright_high)).nonzero()[0]
            
            if len(good_left_inds) > 0:
                left_lane_inds_list.append(good_left_inds)
                if len(good_left_inds) > minpix:
                    leftx_current = int(np.mean(nonzero_x[good_left_inds]))
                    
            if len(good_right_inds) > 0:
                right_lane_inds_list.append(good_right_inds)
                if len(good_right_inds) > minpix:
                    rightx_current = int(np.mean(nonzero_x[good_right_inds]))
        
        self.prev_left_base = leftx_current
        self.prev_right_base = rightx_current
        
        left_line, right_line, lane_center = None, None, None
        
        if left_lane_inds_list:
            left_lane_inds = np.concatenate(left_lane_inds_list)
            left_x = nonzero_x[left_lane_inds]
            left_y = nonzero_y[left_lane_inds]
            if len(left_x) > 100:
                left_line = np.polyfit(left_y, left_x, 2)
        
        if right_lane_inds_list:
            right_lane_inds = np.concatenate(right_lane_inds_list)
            right_x = nonzero_x[right_lane_inds]
            right_y = nonzero_y[right_lane_inds]
            if len(right_x) > 100:
                right_line = np.polyfit(right_y, right_x, 2)
        
        y_eval = height * 0.9
        
        if left_line is not None and right_line is not None:
            left_x_eval = left_line[0] * y_eval**2 + left_line[1] * y_eval + left_line[2]
            right_x_eval = right_line[0] * y_eval**2 + right_line[1] * y_eval + right_line[2]
            lane_center = (left_x_eval + right_x_eval) / 2
            total_pixels = len(left_x) + len(right_x)
            self.detection_confidence = self.lane_confidence = min(100, total_pixels / 100)
        elif left_line is not None:
            left_x_eval = left_line[0] * y_eval**2 + left_line[1] * y_eval + left_line[2]
            lane_center = left_x_eval + (width / 2) # 차선 폭을 절반으로 가정
            self.detection_confidence = self.lane_confidence = min(100, len(left_x) / 80)
        elif right_line is not None:
            right_x_eval = right_line[0] * y_eval**2 + right_line[1] * y_eval + right_line[2]
            lane_center = right_x_eval - (width / 2)
            self.detection_confidence = self.lane_confidence = min(100, len(right_x) / 80)
        else:
            self.detection_confidence = self.lane_confidence = 0
            
        return left_line, right_line, lane_center
    
    def find_lane_base(self, histogram, prev_base, is_left):
        """AuTURBO팀의 히스토그램 기반 차선 시작점 찾기 로직"""
        search_range = 50
        start_idx = max(0, int(prev_base) - search_range)
        end_idx = min(len(histogram), int(prev_base) + search_range)
        
        local_max_idx = np.argmax(histogram[start_idx:end_idx])
        local_max_val = histogram[start_idx + local_max_idx]
        
        if local_max_val > 2000:
            return start_idx + local_max_idx
        
        if np.max(histogram) > 2000:
            return np.argmax(histogram)
        
        return int(prev_base)

    def draw_lane_overlay_2023(self, image, bev_image, lane_mask, left_line, right_line, lane_center):
        """차선 검출 결과 시각화"""
        plot_y = np.linspace(0, bev_image.shape[0] - 1, bev_image.shape[0])
        lane_img = np.zeros_like(bev_image)
        
        if left_line is not None:
            left_fit_x = left_line[0] * plot_y**2 + left_line[1] * plot_y + left_line[2]
            pts_left = np.array([np.transpose(np.vstack([left_fit_x, plot_y]))])
            cv2.polylines(lane_img, np.int_([pts_left]), isClosed=False, color=(255, 0, 0), thickness=10)

        if right_line is not None:
            right_fit_x = right_line[0] * plot_y**2 + right_line[1] * plot_y + right_line[2]
            pts_right = np.array([np.transpose(np.vstack([right_fit_x, plot_y]))])
            cv2.polylines(lane_img, np.int_([pts_right]), isClosed=False, color=(0, 0, 255), thickness=10)

        if left_line is not None and right_line is not None:
            left_fit_x = left_line[0] * plot_y**2 + left_line[1] * plot_y + left_line[2]
            right_fit_x = right_line[0] * plot_y**2 + right_line[1] * plot_y + right_line[2]
            pts = np.hstack((pts_left, pts_right[:, ::-1, :]))
            cv2.fillPoly(lane_img, np.int_([pts]), (0, 255, 0))

        warped_back = cv2.warpPerspective(lane_img, self.inv_bev_matrix, (image.shape[1], image.shape[0]))
        result = cv2.addWeighted(image, 1, warped_back, 0.3, 0)
        
        mask_colored = cv2.cvtColor(lane_mask, cv2.COLOR_GRAY2BGR)
        bev_with_lines = cv2.addWeighted(bev_image, 1, lane_img, 0.5, 0)
        
        combined_debug = np.vstack([
            cv2.resize(bev_with_lines, (160, 120)),
            cv2.resize(mask_colored, (160, 120))
        ])
        result[150:150+240, 10:10+160] = combined_debug
        
        cv2.putText(result, "BEV View", (20, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv2.putText(result, "Lane Mask", (20, 290), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        np.copyto(image, result)

    def lidar_callback(self, msg):
        self.lidar_data = msg.ranges

    def draw_lidar_overlay(self, image):
        height, width = image.shape[:2]
        center_x, center_y = width // 2, height

        if self.lidar_data is None:
            return

        num_ranges = len(self.lidar_data)
        front_ranges = self.lidar_data[num_ranges // 4 : 3 * num_ranges // 4]
        
        for i, dist in enumerate(front_ranges):
            if math.isinf(dist) or math.isnan(dist) or dist > 2.0:
                continue

            angle = (i / len(front_ranges) - 0.5) * math.pi
            
            x = int(center_x + math.sin(angle) * dist * 200)
            y = int(center_y - math.cos(angle) * dist * 150)

            if 0 <= x < width and 0 <= y < height:
                color = (0, 255, 0)
                if dist < 0.5:
                    color = (0, 0, 255)
                elif dist < 1.0:
                    color = (0, 255, 255)
                cv2.circle(image, (x, y), 3, color, -1)

    def calculate_steering_control(self, error, mode):
        """PID 제어를 사용하여 조향각 계산 (AuTURBO팀 게인 적용)"""
        # AuTURBO팀의 PID 게인 값으로 튜닝
        if mode == "LANE":
            kp, ki, kd = 0.0025, 0.00005, 0.0018
            self.target_speed = 0.7 if abs(math.degrees(self.current_steering)) < 10 else 0.4
        elif mode == "RUBBERCON":
            kp, ki, kd = 0.0035, 0.00008, 0.002
            self.target_speed = 0.35
        elif mode == "LANE_LOST":
            kp, ki, kd = 0.002, 0.0001, 0.001
            self.target_speed = 0.2
        else:
            kp, ki, kd = 0.002, 0.0001, 0.001
            self.target_speed = 0.2

        p_term = kp * error
        
        # 적분 windup 방지
        integral_limit = 800
        self.integral_error += error
        if abs(self.integral_error) > integral_limit:
            self.integral_error = integral_limit * np.sign(self.integral_error)
        i_term = ki * self.integral_error
        
        derivative = error - self.prev_error
        d_term = kd * derivative
        self.prev_error = error
        
        steering = p_term + i_term + d_term
        self.target_steering = np.clip(steering, -0.6, 0.6)

    def control_loop(self):
        twist = Twist()
        
        self.current_speed = self.current_speed * 0.9 + self.target_speed * 0.1
        self.current_steering = self.current_steering * 0.9 + self.target_steering * 0.1

        twist.linear.x = self.current_speed
        twist.angular.z = self.current_steering
        
        self.cmd_pub.publish(twist)

    def get_processed_frame(self):
        with self.image_lock:
            if self.processed_frame is not None:
                return self.processed_frame.copy()
        return None

    def get_stats(self):
        min_lidar_dist = "N/A"
        if self.lidar_data:
            front_dist = self.lidar_data[0]
            if not (math.isinf(front_dist) or math.isnan(front_dist)):
                min_lidar_dist = round(front_dist, 2)

        return {
            "current_mode": self.current_mode.value,
            "traffic_light_status": self.traffic_light_state,
            "rubbercon_status": self.rubbercon_status,
            "lane_status": "DETECTED" if self.lane_detected else "SEARCHING",
            "obstacle_car_status": self.obstacle_car_position or "None",
            "detection_confidence": round(self.detection_confidence, 1),
            "camera_fps": self.camera_fps,
            "lidar_distance": min_lidar_dist,
            "speed": round(self.current_speed, 2),
            "steering_angle": round(math.degrees(self.current_steering), 1),
            "lap_counter": self.lap_counter
        }

def main(args=None):
    rclpy.init(args=args)
    autoracer_node = Autoracer()
    try:
        rclpy.spin(autoracer_node)
    except KeyboardInterrupt:
        autoracer_node.get_logger().info('🛑 Node stopped cleanly.')
    finally:
        autoracer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
