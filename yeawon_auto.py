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
    AR_CURVE_CONTROL = "AR_CURVE_CONTROL"
    RUBBERCON_AVOIDANCE = "RUBBERCON_AVOID"
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
                <title>🏁 Autoracer 2025 Contest - Enhanced Dashboard</title>
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
                    
                    time.sleep(0.066)
                    
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
        
        # 미션 상태 관리
        self.current_mode = DriveMode.AR_CURVE_CONTROL
        self.rubbercon_passed = False
        self.lane_detected = False
        self.lane_search_count = 0
        
        # 제어 변수
        self.current_speed = 0.0
        self.current_steering = 0.0
        self.target_speed = 0.0
        self.target_steering = 0.0
        
        # PID 제어 변수
        self.prev_error = 0.0
        self.integral_error = 0.0
        self.last_lane_center = 320
        
        # 라바콘 회피 상태
        self.rubbercon_detection_count = 0
        self.rubbercon_avoidance_active = False
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
        self.lane_confidence = 0.0
        self.prev_left_base = 160
        self.prev_right_base = 480
        
        self.lap_counter = 0
        
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
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('🏆 Autoracer 2025 Contest Started!')

    def setup_bev_transform(self):
        src_points = np.float32([
            [80, 480],    
            [560, 480],   
            [240, 280],   
            [400, 280]    
        ])
        
        dst_points = np.float32([
            [150, 480],   
            [490, 480],   
            [150, 0],     
            [490, 0]      
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
                    if e.errno == 98:
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
            
            if image is None:
                self.get_logger().error('Failed to decode image, skipping frame.')
                return
            
            with self.image_lock:
                self.current_image = image.copy()
            
            self.process_image(image)
            
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
        
        if self.current_mode == DriveMode.RUBBERCON_AVOIDANCE:
            self.detect_and_avoid_rubbercon_enhanced(processed)
        elif self.current_mode == DriveMode.AR_CURVE_CONTROL:
            self.ar_curve_control(processed)
        
        if self.lidar_data is not None:
            self.draw_lidar_overlay(processed)
            
        self.draw_status_header(processed)

        with self.image_lock:
            self.processed_frame = processed.copy()

    def draw_status_header(self, image):
        height, width = image.shape[:2]
        
        overlay = image.copy()
        cv2.rectangle(overlay, (0, 0), (width, 140), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)
        
        elapsed = time.time() - self.mission_start_time
        time_str = f"{int(elapsed//60):02d}:{int(elapsed%60):02d}"
        
        cv2.putText(image, f'🏁 Autoracer 2025 | Mode: {self.current_mode.value}', 
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(image, f'⏱️ Time: {time_str} | Frame: {self.frame_count} | FPS: {self.camera_fps}', 
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(image, f'🚗 Speed: {self.current_speed:.2f} | 🎯 Steer: {math.degrees(self.current_steering):.1f}° | Conf: {self.detection_confidence:.1f}%', 
                    (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(image, f'🏁 Laps: {self.lap_counter} | Rubbercon Status: {self.rubbercon_status}', 
                    (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

    def ar_curve_control(self, image):
        left_line, right_line, lane_center = self.detect_lane_enhanced_2023(image)

        rubbercons = self.find_rubbercons(image)
        if len(rubbercons) > 0:
            self.get_logger().info('🔴 Rubbercon detected, switching to avoidance mode.')
            self.current_mode = DriveMode.RUBBERCON_AVOIDANCE
            self.rubbercon_status = "DETECTED"
            self.integral_error = 0.0
            self.prev_error = 0.0
            return

        if lane_center is not None:
            self.lane_detected = True
            image_center = image.shape[1] // 2
            steering_error = lane_center - image_center
            self.last_lane_center = lane_center
            self.calculate_steering_control(steering_error, "LANE")
        else:
            self.lane_detected = False
            steering_error = self.last_lane_center - (image.shape[1] // 2)
            self.calculate_steering_control(steering_error, "LANE_LOST")

    def find_rubbercons(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        lower_orange = np.array([5, 100, 100])
        upper_orange = np.array([25, 255, 255])
        orange_mask = cv2.inRange(hsv, lower_orange, upper_orange)
        
        kernel = np.ones((5, 5), np.uint8)
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, kernel, iterations=2)
        
        height, width = image.shape[:2]
        contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        rubbercons = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 200:
                x, y, w, h = cv2.boundingRect(contour)
                center_y = y + h // 2
                distance_from_bottom = height - center_y
                if distance_from_bottom < 150:
                    rubbercons.append({'center_x': x + w // 2, 'area': area, 'distance': distance_from_bottom})
        return rubbercons

    def detect_and_avoid_rubbercon_enhanced(self, image):
        height, width = image.shape[:2]
        
        rubbercons = self.find_rubbercons(image)

        if len(rubbercons) > 0:
            self.rubbercon_detection_flag = 1
            self.no_rubbercon_frames = 0
            self.rubbercon_status = f"DETECTED: {len(rubbercons)}"
            
            left_cones = [c for c in rubbercons if c['center_x'] < width // 2]
            right_cones = [c for c in rubbercons if c['center_x'] > width // 2]

            target_x = width // 2
            
            if left_cones and right_cones:
                left_most = min(left_cones, key=lambda c: c['center_x'])
                right_most = max(right_cones, key=lambda c: c['center_x'])
                
                target_x = (left_most['center_x'] + right_most['center_x']) // 2
                
                error_from_left = target_x - (left_most['center_x'] + 100)
                error_from_right = target_x - (right_most['center_x'] - 100)
                error_sum = error_from_left + error_from_right
                
                error = target_x - (width // 2) + error_sum * 0.1
            
            elif left_cones:
                closest_cone = max(left_cones, key=lambda c: c['center_x'])
                target_x = closest_cone['center_x'] + 150
                error = target_x - (width // 2)
            
            elif right_cones:
                closest_cone = min(right_cones, key=lambda c: c['center_x'])
                target_x = closest_cone['center_x'] - 150
                error = target_x - (width // 2)
            
            self.calculate_steering_control(error, "RUBBERCON")
            cv2.circle(image, (int(target_x), height - 50), 10, (255, 0, 255), -1)
            cv2.line(image, (width // 2, height), (int(target_x), height - 50), (255, 0, 255), 2)
            
        elif self.rubbercon_detection_flag == 1:
            self.no_rubbercon_frames += 1
            if self.no_rubbercon_frames > 10:
                self.rubbercon_passed = True
                self.current_mode = DriveMode.AR_CURVE_CONTROL
                self.rubbercon_detection_flag = 0
                self.get_logger().info('✅ Rubbercon avoidance complete. Switching to AR Curve Control.')
        else:
            self.ar_curve_control(image)

    def detect_lane_enhanced_2023(self, image):
        if self.bev_matrix is None:
            bev_image = image.copy()
        else:
            bev_image = cv2.warpPerspective(image, self.bev_matrix, (640, 480))
        
        hsv_bev = cv2.cvtColor(bev_image, cv2.COLOR_BGR2HSV)
        
        lower_white = np.array([0, 0, 180])
        upper_white = np.array([180, 30, 255])
        white_mask = cv2.inRange(hsv_bev, lower_white, upper_white)
        
        lower_yellow = np.array([20, 80, 80])
        upper_yellow = np.array([35, 255, 255])
        yellow_mask = cv2.inRange(hsv_bev, lower_yellow, upper_yellow)
        
        gray_bev = cv2.cvtColor(bev_image, cv2.COLOR_BGR2GRAY)
        adaptive_mask = cv2.adaptiveThreshold(gray_bev, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                              cv2.THRESH_BINARY, 15, -10)
        
        lane_mask = cv2.bitwise_or(white_mask, yellow_mask)
        lane_mask = cv2.bitwise_or(lane_mask, adaptive_mask)
        
        kernel = np.ones((5, 5), np.uint8)
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_OPEN, kernel)
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, kernel)
        
        left_line, right_line, lane_center = self.sliding_window_lane_detection_2023(lane_mask)
        
        self.draw_lane_overlay_2023(image, bev_image, lane_mask, left_line, right_line, lane_center)
        
        return left_line, right_line, lane_center

    def sliding_window_lane_detection_2023(self, binary_image):
        height, width = binary_image.shape
        
        histogram = np.sum(binary_image[height * 2 // 3:, :], axis=0)
        histogram = np.convolve(histogram, np.ones(10)/10, mode='same')
        
        midpoint = width // 2
        
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
            lane_center = left_x_eval + (width / 2)
            self.detection_confidence = self.lane_confidence = min(100, len(left_x) / 80)
        elif right_line is not None:
            right_x_eval = right_line[0] * y_eval**2 + right_line[1] * y_eval + right_line[2]
            lane_center = right_x_eval - (width / 2)
            self.detection_confidence = self.lane_confidence = min(100, len(right_x) / 80)
        else:
            self.detection_confidence = self.lane_confidence = 0
            
        return left_line, right_line, lane_center
    
    def find_lane_base(self, histogram, prev_base, is_left):
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
        if mode == "LANE":
            kp, ki, kd = 0.0018, 0.00004, 0.0015
            self.target_speed = 0.3 if abs(math.degrees(self.current_steering)) < 10 else 0.2
        elif mode == "RUBBERCON":
            kp, ki, kd = 0.0025, 0.00006, 0.0018
            self.target_speed = 0.15
        elif mode == "LANE_LOST":
            kp, ki, kd = 0.0015, 0.00008, 0.0008
            self.target_speed = 0.1
        else:
            kp, ki, kd = 0.0015, 0.00008, 0.0008
            self.target_speed = 0.1

        p_term = kp * error
        
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
            "rubbercon_status": self.rubbercon_status,
            "lane_status": "DETECTED" if self.lane_detected else "SEARCHING",
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
