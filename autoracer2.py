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
    WAITING_FOR_GREEN = "WAITING_GREEN"
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
                <h1>🏆 Autoracer 2025 Contest Dashboard</h1>
                <div class="container">
                    <div class="panel" style="flex: 2;">
                        <h3>📹 Live Camera Feed</h3>
                        <img src="/stream.mjpg" width="800" height="600" style="border: 2px solid #444; border-radius: 8px; width: 100%; max-width: 800px;">
                    </div>
                    <div class="panel" style="flex: 1;">
                        <h3>🎯 Mission Control</h3>
                        <div class="status-card">
                            <div class="metric"><span>Current Mode:</span><span id="mode" class="metric-value">Loading...</span></div>
                            <div class="metric"><span>Traffic Light:</span><span id="traffic_light" class="metric-value">Detecting...</span></div>
                            <div class="metric"><span>Rubbercon Status:</span><span id="rubbercon" class="metric-value">Searching...</span></div>
                            <div class="metric"><span>Lane Status:</span><span id="lane" class="metric-value">Detecting...</span></div>
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
                            <div id="progress" class="progress-fill" style="width: 33%;"></div>
                        </div>
                        <div style="font-size: 14px;">
                            <p id="mission1" style="color: #00ff00;">✅ Mission 1: Green Light Detection</p>
                            <p id="mission2" style="color: #ffaa00;">🔄 Mission 2: Rubbercon Avoidance</p>
                            <p id="mission3" style="color: #666;">⏳ Mission 3: Lane Following</p>
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
                        document.getElementById('camera_fps').textContent = data.camera_fps;
                        document.getElementById('lidar_dist').textContent = data.lidar_distance;
                        document.getElementById('speed').textContent = data.speed;
                        document.getElementById('steering').textContent = data.steering_angle;
                        
                        // Mission time
                        let elapsed = Math.floor((Date.now() - startTime) / 1000);
                        let mins = Math.floor(elapsed / 60);
                        let secs = elapsed % 60;
                        document.getElementById('mission_time').textContent = 
                            `${mins.toString().padStart(2,'0')}:${secs.toString().padStart(2,'0')}`;
                        
                        // Progress bar update
                        let progress = 0;
                        if (data.current_mode.includes('RUBBERCON')) progress = 33;
                        else if (data.current_mode.includes('LANE')) progress = 66;
                        else if (data.current_mode.includes('COMPLETE')) progress = 100;
                        document.getElementById('progress').style.width = progress + '%';
                        
                        // Mission status colors
                        const missions = ['mission1', 'mission2', 'mission3'];
                        missions.forEach((m, i) => {
                            const elem = document.getElementById(m);
                            if (i * 33 < progress) elem.style.color = '#00ff00';
                            else if (i * 33 === progress - 33) elem.style.color = '#ffaa00';
                        });
                        
                        // Alerts
                        let alerts = [];
                        if (data.lidar_distance < 0.3) alerts.push('⚠️ Obstacle Too Close');
                        if (data.camera_fps < 5) alerts.push('📹 Low Camera FPS');
                        if (data.steering_angle > 0.8) alerts.push('🎯 Sharp Turn');
                        
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
            import json
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
        self.current_mode = DriveMode.WAITING_FOR_GREEN
        self.green_light_detected = False
        self.rubbercon_passed = False
        self.lane_following_started = False
        
        # 제어 변수
        self.current_speed = 0.0
        self.current_steering = 0.0
        self.target_speed = 0.0
        self.target_steering = 0.0
        
        # Pure Pursuit 제어 변수
        self.prev_error = 0.0
        self.integral_error = 0.0
        self.last_lane_center = 320
        
        # 라바콘 회피 상태
        self.rubbercon_detection_count = 0
        self.rubbercon_avoidance_active = False
        self.rubbercon_clear_count = 0
        
        # 통계 및 성능 데이터
        self.frame_count = 0
        self.start_time = time.time()
        self.last_camera_time = 0
        self.camera_fps = 0
        self.mission_start_time = time.time()
        
        # Bird's Eye View 변환 행렬
        self.bev_matrix = None
        self.inv_bev_matrix = None
        self.setup_bev_transform()
        
        # 차선 검출용 슬라이딩 윈도우
        self.left_lane_pixels = []
        self.right_lane_pixels = []
        self.lane_confidence = 0.0
        
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
        
        self.get_logger().info('🏆 Autoracer 2025 Contest Started!')
        self.get_logger().info(f'📊 Dashboard: http://{self.get_ip_address()}:{self.web_port}/')

    def setup_bev_transform(self):
        """Bird's Eye View 변환 행렬 설정"""
        # 원본 이미지의 사각형 영역 (차선이 보이는 영역)
        src_points = np.float32([
            [100, 480],   # 좌하단
            [540, 480],   # 우하단  
            [280, 200],   # 좌상단
            [360, 200]    # 우상단
        ])
        
        # 변환될 BEV 이미지의 사각형 영역
        dst_points = np.float32([
            [100, 480],   # 좌하단
            [540, 480],   # 우하단
            [100, 0],     # 좌상단
            [540, 0]      # 우상단
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
                    self.get_logger().info(f'🌐 Web server: http://{self.get_ip_address()}:{port}/')
                    server.serve_forever()
                    break
                except OSError as e:
                    if e.errno == 98:
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
        height, width = image.shape[:2]
        
        # 상태 정보 헤더
        self.draw_status_header(processed)
        
        # 미션별 처리
        if self.current_mode == DriveMode.WAITING_FOR_GREEN:
            self.detect_traffic_light(processed)
        elif self.current_mode == DriveMode.RUBBERCON_AVOIDANCE:
            self.detect_and_avoid_rubbercon(processed)
        elif self.current_mode == DriveMode.LANE_FOLLOWING:
            self.detect_lane_advanced(processed)
        
        # 라이다 오버레이
        if self.lidar_data is not None:
            self.draw_lidar_overlay(processed)
        
        with self.image_lock:
            self.processed_frame = processed.copy()

    def draw_status_header(self, image):
        """상태 정보 헤더 그리기"""
        height, width = image.shape[:2]
        
        # 반투명 배경
        overlay = image.copy()
        cv2.rectangle(overlay, (0, 0), (width, 120), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)
        
        # 미션 경과 시간
        elapsed = time.time() - self.mission_start_time
        time_str = f"{int(elapsed//60):02d}:{int(elapsed%60):02d}"
        
        # 텍스트 정보
        cv2.putText(image, f'🏆 Contest 2025 | Mode: {self.current_mode.value}', 
                   (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(image, f'⏱️ Time: {time_str} | Frame: {self.frame_count} | FPS: {self.camera_fps}', 
                   (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(image, f'🚗 Speed: {self.current_speed:.2f} m/s | 🎯 Steer: {math.degrees(self.current_steering):.1f}°', 
                   (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # 미션 진행도 바
        progress_width = width - 20
        progress_height = 10
        cv2.rectangle(image, (10, 100), (10 + progress_width, 100 + progress_height), (100, 100, 100), -1)
        
        progress = 0
        if self.current_mode == DriveMode.RUBBERCON_AVOIDANCE:
            progress = 33
        elif self.current_mode == DriveMode.LANE_FOLLOWING:
            progress = 66
        
        if progress > 0:
            fill_width = int((progress / 100) * progress_width)
            cv2.rectangle(image, (10, 100), (10 + fill_width, 100 + progress_height), (0, 255, 0), -1)

    def detect_traffic_light(self, image):
        """고급 신호등 검출 - 형태 및 색상 기반"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 초록색 범위를 더 정밀하게 설정
        lower_green1 = np.array([35, 40, 40])
        upper_green1 = np.array([85, 255, 255])
        
        lower_green2 = np.array([35, 40, 40])
        upper_green2 = np.array([85, 255, 255])
        
        # 두 개의 마스크를 합침
        green_mask1 = cv2.inRange(hsv, lower_green1, upper_green1)
        green_mask2 = cv2.inRange(hsv, lower_green2, upper_green2)
        green_mask = cv2.bitwise_or(green_mask1, green_mask2)
        
        # 노이즈 제거 및 형태학적 연산
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
        
        # 원형 검출 (Hough Circle)
        circles = cv2.HoughCircles(
            green_mask,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=30,
            param1=50,
            param2=25,
            minRadius=8,
            maxRadius=80
        )
        
        green_detected = False
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                # 신호등 위치 필터링 (화면 상단 중앙 영역)
                if y < image.shape[0] * 0.6 and r > 10:
                    # 원형도 검사
                    mask_roi = green_mask[max(0, y-r):min(green_mask.shape[0], y+r), 
                                        max(0, x-r):min(green_mask.shape[1], x+r)]
                    if mask_roi.size > 0:
                        circle_ratio = cv2.countNonZero(mask_roi) / (math.pi * r * r)
                        
                        if circle_ratio > 0.3:  # 원형도 임계값
                            cv2.circle(image, (x, y), r, (0, 255, 0), 3)
                            cv2.circle(image, (x, y), 2, (0, 255, 0), -1)
                            cv2.putText(image, '🟢 GREEN LIGHT!', (x-60, y-r-15), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                            green_detected = True
                            break
        
        # 상태 업데이트
        if green_detected and not self.green_light_detected:
            self.green_light_detected = True
            self.current_mode = DriveMode.RUBBERCON_AVOIDANCE
            self.get_logger().info('🟢 Green light detected! Switching to rubbercon avoidance mode')
        
        # 상태 표시
        status_text = "🟢 GREEN DETECTED - STARTING MISSION!" if green_detected else "🔴 WAITING FOR GREEN LIGHT..."
        cv2.putText(image, status_text, (10, image.shape[0]-30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if green_detected else (0, 100, 255), 2)

    def detect_and_avoid_rubbercon(self, image):
        """라바콘 검출 및 회피 - 2023년 알고리즘 개선"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 주황색/빨간색 라바콘 검출 (더 넓은 범위)
        lower_orange1 = np.array([0, 120, 120])
        upper_orange1 = np.array([15, 255, 255])
        
        lower_orange2 = np.array([160, 120, 120])
        upper_orange2 = np.array([180, 255, 255])
        
        orange_mask1 = cv2.inRange(hsv, lower_orange1, upper_orange1)
        orange_mask2 = cv2.inRange(hsv, lower_orange2, upper_orange2)
        orange_mask = cv2.bitwise_or(orange_mask1, orange_mask2)
        
        # 노이즈 제거
        kernel = np.ones((5, 5), np.uint8)
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, kernel)
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel)
        
        # 컨투어 검출
        contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        rubbercons = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 300:  # 최소 면적
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w) / h
                
                # 라바콘 형태 필터링 (세로가 더 길고 하단에 위치)
                if 0.2 < aspect_ratio < 1.2 and h > 20 and y > image.shape[0] * 0.3:
                    # 중심점이 화면 하단 60%에 있어야 함
                    center_y = y + h // 2
                    if center_y > image.shape[0] * 0.4:
                        rubbercons.append({
                            'x': x, 'y': y, 'w': w, 'h': h,
                            'center_x': x + w // 2,
                            'center_y': center_y,
                            'area': area,
                            'distance': image.shape[0] - center_y  # 대략적인 거리
                        })
                        
                        # 시각화
                        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 165, 255), 2)
                        cv2.putText(image, f'CONE({area:.0f})', (x, y - 10), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
                        cv2.circle(image, (x + w // 2, center_y), 5, (0, 165, 255), -1)
        
        # 라바콘 회피 로직 (2023년 알고리즘 기반)
        self.process_rubbercon_avoidance(rubbercons, image)
        
        # 디버그 정보
        cv2.putText(image, f"🎯 RUBBERCONS: {len(rubbercons)} detected", 
                   (10, image.shape[0]-60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

    def process_rubbercon_avoidance(self, rubbercons, image):
        """라바콘 회피 처리 로직"""
        if len(rubbercons) >= 2:
            # 라바콘이 2개 이상 검출되면 회피 모드 활성화
            self.rubbercon_detection_count += 1
            self.rubbercon_avoidance_active = True
            self.rubbercon_clear_count = 0
            
            # 거리순으로 정렬 (가까운 것부터)
            rubbercons.sort(key=lambda x: x['distance'], reverse=True)
            
            # 좌우 라바콘 구분
            image_center = image.shape[1] // 2
            left_cones = [cone for cone in rubbercons if cone['center_x'] < image_center]
            right_cones = [cone for cone in rubbercons if cone['center_x'] >= image_center]
            
            if len(left_cones) > 0 and len(right_cones) > 0:
                # 가장 가까운 좌우 라바콘
                closest_left = min(left_cones, key=lambda x: x['distance'])
                closest_right = min(right_cones, key=lambda x: x['distance'])
                
                # 두 라바콘 사이의 중점 계산
                target_x = (closest_left['center_x'] + closest_right['center_x']) // 2
                target_y = min(closest_left['center_y'], closest_right['center_y'])
                
                # 목표점 시각화
                cv2.circle(image, (target_x, target_y), 10, (255, 0, 255), -1)
                cv2.putText(image, '🎯 TARGET', (target_x - 30, target_y - 15), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
                
                # 조향각 계산 (Pure Pursuit 기반)
                steering_error = target_x - image_center
                self.calculate_steering_control(steering_error, "RUBBERCON")
                
                # 연결선 그리기
                cv2.line(image, (closest_left['center_x'], closest_left['center_y']),
                        (closest_right['center_x'], closest_right['center_y']), (255, 255, 0), 2)
                cv2.line(image, (image_center, image.shape[0]), 
                        (target_x, target_y), (255, 0, 255), 2)
            
        elif self.rubbercon_avoidance_active and len(rubbercons) < 2:
            # 라바콘이 줄어들면 클리어 카운트 증가
            self.rubbercon_clear_count += 1
            
            # 5프레임 연속으로 라바콘이 적게 검출되면 통과 완료
            if self.rubbercon_clear_count > 5:
                self.rubbercon_passed = True
                self.current_mode = DriveMode.LANE_FOLLOWING
                self.get_logger().info('🎯 Rubbercon avoidance completed! Switching to lane following')
        
        # 회피 상태 표시
        if self.rubbercon_avoidance_active:
            status = f"🚧 AVOIDING - Clear: {self.rubbercon_clear_count}"
            cv2.putText(image, status, (10, image.shape[0]-90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 165, 0), 2)

    def detect_lane_advanced(self, image):
        """고급 차선 검출 - Bird's Eye View + Sliding Window"""
        # Bird's Eye View 변환
        if self.bev_matrix is not None:
            bev_image = cv2.warpPerspective(image, self.bev_matrix, (640, 480))
        else:
            bev_image = image
        
        # HSV 변환 후 흰색 차선 검출
        hsv_bev = cv2.cvtColor(bev_image, cv2.COLOR_BGR2HSV)
        
        # 흰색 범위 (밝기 기반)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])
        
        white_mask = cv2.inRange(hsv_bev, lower_white, upper_white)
        
        # 추가적으로 밝기 기반 검출
        gray_bev = cv2.cvtColor(bev_image, cv2.COLOR_BGR2GRAY)
        _, bright_mask = cv2.threshold(gray_bev, 200, 255, cv2.THRESH_BINARY)
        
        # 두 마스크 결합
        lane_mask = cv2.bitwise_or(white_mask, bright_mask)
        
        # 노이즈 제거
        kernel = np.ones((3, 3), np.uint8)
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_OPEN, kernel)
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, kernel)
        
        # 슬라이딩 윈도우로 차선 검출
        left_line, right_line, lane_center = self.sliding_window_lane_detection(lane_mask)
        
        # 결과를 원본 이미지에 오버레이
        self.draw_lane_overlay(image, bev_image, lane_mask, left_line, right_line, lane_center)
        
        # 차선 추종 제어
        if lane_center is not None:
            self.lane_detected = True
            image_center = image.shape[1] // 2
            steering_error = lane_center - image_center
            self.calculate_steering_control(steering_error, "LANE")
        else:
            self.lane_detected = False

    def sliding_window_lane_detection(self, binary_image):
        """슬라이딩 윈도우 기반 차선 검출"""
        height, width = binary_image.shape
        
        # 히스토그램으로 차선의 시작점 찾기
        histogram = np.sum(binary_image[height//2:, :], axis=0)
        
        # 좌우 반분으로 나누어 최대값 찾기
        midpoint = width // 2
        left_base = np.argmax(histogram[:midpoint]) if np.max(histogram[:midpoint]) > 50 else 0
        right_base = np.argmax(histogram[midpoint:]) + midpoint if np.max(histogram[midpoint:]) > 50 else width
        
        # 슬라이딩 윈도우 설정
        nwindows = 9
        window_height = height // nwindows
        margin = 50
        minpix = 50
        
        # 현재 윈도우 중심
        leftx_current = left_base
        rightx_current = right_base
        
        # 차선 픽셀 저장할 리스트
        left_lane_inds = []
        right_lane_inds = []
        
        # 윈도우별로 검색
        for window in range(nwindows):
            # 윈도우 경계 계산
            win_y_low = height - (window + 1) * window_height
            win_y_high = height - window * window_height
            
            # 좌측 윈도우
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            
            # 우측 윈도우
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            
            # 윈도우 내의 픽셀 찾기
            good_left_inds = ((binary_image[win_y_low:win_y_high, win_xleft_low:win_xleft_high] > 0).nonzero())
            good_right_inds = ((binary_image[win_y_low:win_y_high, win_xright_low:win_xright_high] > 0).nonzero())
            
            if len(good_left_inds[0]) > 0:
                good_left_inds = (good_left_inds[0] + win_y_low, good_left_inds[1] + win_xleft_low)
                left_lane_inds.append(good_left_inds)
                
                # 다음 윈도우 중심 업데이트
                if len(good_left_inds[1]) > minpix:
                    leftx_current = int(np.mean(good_left_inds[1]))
            
            if len(good_right_inds[0]) > 0:
                good_right_inds = (good_right_inds[0] + win_y_low, good_right_inds[1] + win_xright_low)
                right_lane_inds.append(good_right_inds)
                
                # 다음 윈도우 중심 업데이트
                if len(good_right_inds[1]) > minpix:
                    rightx_current = int(np.mean(good_right_inds[1]))
        
        # 차선 피팅
        left_line = None
        right_line = None
        lane_center = None
        
        if len(left_lane_inds) > 3:
            # 좌측 차선 픽셀 합치기
            left_y = np.concatenate([inds[0] for inds in left_lane_inds])
            left_x = np.concatenate([inds[1] for inds in left_lane_inds])
            
            if len(left_x) > 100:  # 충분한 픽셀이 있을 때만
                left_line = np.polyfit(left_y, left_x, 2)
        
        if len(right_lane_inds) > 3:
            # 우측 차선 픽셀 합치기
            right_y = np.concatenate([inds[0] for inds in right_lane_inds])
            right_x = np.concatenate([inds[1] for inds in right_lane_inds])
            
            if len(right_x) > 100:  # 충분한 픽셀이 있을 때만
                right_line = np.polyfit(right_y, right_x, 2)
        
        # 차선 중심 계산
        if left_line is not None and right_line is not None:
            y_eval = height * 3 // 4  # 화면 하단 3/4 지점에서 평가
            left_x_eval = left_line[0] * y_eval**2 + left_line[1] * y_eval + left_line[2]
            right_x_eval = right_line[0] * y_eval**2 + right_line[1] * y_eval + right_line[2]
            lane_center = (left_x_eval + right_x_eval) // 2
        elif left_line is not None:
            y_eval = height * 3 // 4
            left_x_eval = left_line[0] * y_eval**2 + left_line[1] * y_eval + left_line[2]
            lane_center = left_x_eval + 150  # 대략적인 차선 폭의 절반
        elif right_line is not None:
            y_eval = height * 3 // 4
            right_x_eval = right_line[0] * y_eval**2 + right_line[1] * y_eval + right_line[2]
            lane_center = right_x_eval - 150  # 대략적인 차선 폭의 절반
        
        return left_line, right_line, lane_center

    def draw_lane_overlay(self, original_image, bev_image, lane_mask, left_line, right_line, lane_center):
        """차선 검출 결과를 원본 이미지에 오버레이"""
        height, width = bev_image.shape[:2]
        
        # BEV 결과를 컬러로 변환
        bev_colored = cv2.cvtColor(lane_mask, cv2.COLOR_GRAY2BGR)
        
        # 차선 곡선 그리기
        plot_y = np.linspace(0, height-1, height)
        
        if left_line is not None:
            left_fitx = left_line[0] * plot_y**2 + left_line[1] * plot_y + left_line[2]
            left_points = np.array(list(zip(left_fitx, plot_y)), dtype=np.int32)
            cv2.polylines(bev_colored, [left_points], False, (255, 0, 0), 10)
        
        if right_line is not None:
            right_fitx = right_line[0] * plot_y**2 + right_line[1] * plot_y + right_line[2]
            right_points = np.array(list(zip(right_fitx, plot_y)), dtype=np.int32)
            cv2.polylines(bev_colored, [right_points], False, (0, 0, 255), 10)
        
        # 차선 중심선 그리기
        if lane_center is not None:
            cv2.line(bev_colored, (int(lane_center), 0), (int(lane_center), height), (0, 255, 255), 5)
        
        # 역변환하여 원본 이미지에 오버레이
        if self.inv_bev_matrix is not None:
            lane_overlay = cv2.warpPerspective(bev_colored, self.inv_bev_matrix, 
                                             (original_image.shape[1], original_image.shape[0]))
            
            # 원본 이미지와 합성
            mask = lane_overlay.astype(bool)
            original_image[mask] = cv2.addWeighted(original_image, 0.7, lane_overlay, 0.3, 0)[mask]
        
        # 차선 상태 표시
        if lane_center is not None:
            cv2.putText(original_image, f"🛣️ LANE CENTER: {int(lane_center)}", 
                       (10, original_image.shape[0]-120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        else:
            cv2.putText(original_image, "🔍 SEARCHING FOR LANE...", 
                       (10, original_image.shape[0]-120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 100, 255), 2)

    def calculate_steering_control(self, error, mode):
        """PID 기반 조향 제어"""
        # 모드별 제어 게인
        if mode == "RUBBERCON":
            kp, ki, kd = 0.004, 0.0001, 0.002
            max_steering = 0.6
        elif mode == "LANE":
            kp, ki, kd = 0.003, 0.00005, 0.0015
            max_steering = 0.4
        else:
            kp, ki, kd = 0.002, 0.0001, 0.001
            max_steering = 0.3
        
        # PID 계산
        self.integral_error += error * 0.05  # dt = 0.05 (20Hz)
        derivative_error = (error - self.prev_error) / 0.05
        
        # 적분 windup 방지
        if abs(self.integral_error) > 1000:
            self.integral_error = 1000 * np.sign(self.integral_error)
        
        # PID 출력
        steering_output = -(kp * error + ki * self.integral_error + kd * derivative_error)
        
        # 조향각 제한
        self.target_steering = max(-max_steering, min(max_steering, steering_output))
        self.prev_error = error

    def draw_lidar_overlay(self, image):
        """라이다 데이터 시각화"""
        if self.lidar_data is None:
            return
        
        height, width = image.shape[:2]
        ranges = self.lidar_data.ranges
        total_points = len(ranges)
        
        if total_points == 0:
            return
        
        center = total_points // 2
        
        # 전방 영역 거리 계산 (±15도)
        front_range = min(30, total_points // 12)
        front_ranges = ranges[center-front_range:center+front_range]
        valid_ranges = [r for r in front_ranges if 0.05 < r < 10.0]
        
        if valid_ranges:
            avg_distance = sum(valid_ranges) / len(valid_ranges)
            min_distance = min(valid_ranges)
            
            # 거리에 따른 색상 및 상태
            if min_distance < 0.2:
                color = (0, 0, 255)  # 빨강
                status = "⚠️ CRITICAL"
            elif min_distance < 0.5:
                color = (0, 255, 255)  # 노랑
                status = "⚠️ WARNING"
            else:
                color = (0, 255, 0)  # 초록
                status = "✅ CLEAR"
            
            # 라이다 정보 패널
            panel_width, panel_height = 280, 120
            panel_x, panel_y = width - panel_width - 10, height - panel_height - 10
            
            # 반투명 배경
            overlay = image.copy()
            cv2.rectangle(overlay, (panel_x, panel_y), (panel_x + panel_width, panel_y + panel_height), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)
            
            # 텍스트 정보
            cv2.putText(image, "📡 LIDAR STATUS", (panel_x + 10, panel_y + 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(image, f"Avg Distance: {avg_distance:.2f}m", (panel_x + 10, panel_y + 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            cv2.putText(image, f"Min Distance: {min_distance:.2f}m", (panel_x + 10, panel_y + 70), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            cv2.putText(image, status, (panel_x + 10, panel_y + 95), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # 라이다 점들을 미니맵에 표시
        self.draw_lidar_minimap(image, ranges)

    def draw_lidar_minimap(self, image, ranges):
        """라이다 데이터 미니맵"""
        minimap_size = 150
        minimap_x = 20
        minimap_y = 150
        
        # 미니맵 배경
        cv2.rectangle(image, (minimap_x, minimap_y), 
                     (minimap_x + minimap_size, minimap_y + minimap_size), (50, 50, 50), -1)
        cv2.rectangle(image, (minimap_x, minimap_y), 
                     (minimap_x + minimap_size, minimap_y + minimap_size), (255, 255, 255), 2)
        
        # 중심점 (차량 위치)
        center_x = minimap_x + minimap_size // 2
        center_y = minimap_y + minimap_size // 2
        cv2.circle(image, (center_x, center_y), 3, (0, 255, 0), -1)
        
        # 라이다 점들 그리기
        total_points = len(ranges)
        if total_points > 0:
            angle_increment = 2 * math.pi / total_points
            
            for i, distance in enumerate(ranges):
                if 0.05 < distance < 3.0:  # 유효한 거리만
                    angle = i * angle_increment - math.pi  # -π to π
                    
                    # 좌표 변환
                    x = int(center_x + (distance * minimap_size / 6) * math.cos(angle))
                    y = int(center_y - (distance * minimap_size / 6) * math.sin(angle))
                    
                    # 화면 내에 있는 점만 그리기
                    if minimap_x <= x <= minimap_x + minimap_size and minimap_y <= y <= minimap_y + minimap_size:
                        color = (0, 255, 255) if distance < 1.0 else (255, 255, 255)
                        cv2.circle(image, (x, y), 1, color, -1)
        
        # 미니맵 제목
        cv2.putText(image, "LIDAR MAP", (minimap_x, minimap_y - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    def lidar_callback(self, msg):
        """라이다 콜백 - 장애물 검출 및 응급정지"""
        self.lidar_data = msg
        
        if len(msg.ranges) == 0:
            return
        
        # 전방 장애물 검사
        total_points = len(msg.ranges)
        center = total_points // 2
        front_range = min(20, total_points // 15)  # 전방 ±12도 정도
        
        front_ranges = msg.ranges[center-front_range:center+front_range]
        valid_ranges = [r for r in front_ranges if 0.05 < r < 10.0]
        
        if valid_ranges:
            min_distance = min(valid_ranges)
            
            # 응급정지 조건 (20cm 이내)
            if min_distance < 0.20:
                if self.current_mode != DriveMode.EMERGENCY_STOP:
                    self.current_mode = DriveMode.EMERGENCY_STOP
                    self.emergency_start_time = time.time()
                    self.get_logger().warn('🚨 EMERGENCY STOP - Obstacle too close!')

    def control_loop(self):
        """메인 제어 루프 - 20Hz"""
        cmd = Twist()
        
        # 부드러운 제어를 위한 저역 통과 필터
        alpha = 0.3  # 필터 상수
        self.current_speed = alpha * self.target_speed + (1 - alpha) * self.current_speed
        self.current_steering = alpha * self.target_steering + (1 - alpha) * self.current_steering
        
        if self.current_mode == DriveMode.WAITING_FOR_GREEN:
            # 신호등 대기
            self.target_speed = 0.0
            self.target_steering = 0.0
            
        elif self.current_mode == DriveMode.RUBBERCON_AVOIDANCE:
            # 라바콘 회피 - 중간 속도
            self.target_speed = 0.4
            # steering은 detect_and_avoid_rubbercon에서 설정
            
        elif self.current_mode == DriveMode.LANE_FOLLOWING:
            # 차선 추종 - 빠른 속도
            if self.lane_detected:
                self.target_speed = 0.6
                # steering은 detect_lane_advanced에서 설정
            else:
                # 차선을 찾지 못한 경우
                self.target_speed = 0.2
                self.target_steering = 0.0
                
        elif self.current_mode == DriveMode.EMERGENCY_STOP:
            # 응급정지
            self.target_speed = 0.0
            self.target_steering = 0.0
            
            # 3초 후 이전 모드로 복귀
            if hasattr(self, 'emergency_start_time'):
                if time.time() - self.emergency_start_time > 3.0:
                    if self.green_light_detected and not self.rubbercon_passed:
                        self.current_mode = DriveMode.RUBBERCON_AVOIDANCE
                    elif self.rubbercon_passed:
                        self.current_mode = DriveMode.LANE_FOLLOWING
                    else:
                        self.current_mode = DriveMode.WAITING_FOR_GREEN
                    
                    delattr(self, 'emergency_start_time')
                    self.get_logger().info('🔄 Emergency stop cleared - resuming mission')
        
        # 최종 명령 설정
        cmd.linear.x = float(self.current_speed)
        cmd.angular.z = float(self.current_steering)
        
        # 안전 제한
        cmd.linear.x = max(0.0, min(1.0, cmd.linear.x))
        cmd.angular.z = max(-1.0, min(1.0, cmd.angular.z))
        
        # 명령 발행
        self.cmd_pub.publish(cmd)

    def get_processed_frame(self):
        """웹 스트리밍용 처리된 프레임 반환"""
        with self.image_lock:
            return self.processed_frame.copy() if self.processed_frame is not None else None

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
            "traffic_light_status": "✅ DETECTED" if self.green_light_detected else "🔍 SEARCHING",
            "rubbercon_status": "✅ PASSED" if self.rubbercon_passed else ("🚧 AVOIDING" if self.rubbercon_avoidance_active else "🔍 SEARCHING"),
            "lane_status": "✅ DETECTED" if self.lane_detected else "🔍 SEARCHING",
            "camera_fps": self.camera_fps,
            "lidar_distance": lidar_distance,
            "speed": f"{self.current_speed:.2f}",
            "steering_angle": f"{math.degrees(self.current_steering):.1f}",
        }

def main(args=None):
    rclpy.init(args=args)
    
    try:
        autoracer = Autoracer()
        rclpy.spin(autoracer)
    except KeyboardInterrupt:
        print("\n🏁 Autoracer 2025 Contest Ended!")
    finally:
        if 'autoracer' in locals():
            autoracer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
