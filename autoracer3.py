
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
    TRAFFIC_LIGHT_WAIT = "TRAFFIC_LIGHT_WAIT"
    RUBBERCON_AVOIDANCE = "RUBBERCON_AVOID"
    LANE_FOLLOWING = "LANE_FOLLOW"
    OBSTACLE_CAR_AVOIDANCE = "OBSTACLE_CAR_AVOID"
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
                        
                        // Mission time
                        let elapsed = Math.floor((Date.now() - startTime) / 1000);
                        let mins = Math.floor(elapsed / 60);
                        let secs = elapsed % 60;
                        document.getElementById('mission_time').textContent = 
                            `${mins.toString().padStart(2,'0')}:${secs.toString().padStart(2,'0')}`;
                        
                        // Progress bar update
                        let progress = 0;
                        if (data.current_mode.includes('TRAFFIC')) progress = 25;
                        else if (data.current_mode.includes('RUBBERCON')) progress = 50;
                        else if (data.current_mode.includes('LANE')) progress = 75;
                        else if (data.current_mode.includes('OBSTACLE')) progress = 100;
                        document.getElementById('progress').style.width = progress + '%';
                        
                        // Mission status colors
                        const missions = ['mission1', 'mission2', 'mission3', 'mission4'];
                        missions.forEach((m, i) => {
                            const elem = document.getElementById(m);
                            if (i * 25 < progress) elem.style.color = '#00ff00';
                            else if (i * 25 === progress - 25) elem.style.color = '#ffaa00';
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
        
        # Pure Pursuit 제어 변수
        self.prev_error = 0.0
        self.integral_error = 0.0
        self.last_lane_center = 320
        
        # 신호등 상태 - NEW for 2025
        self.traffic_light_state = None
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
        self.get_logger().info(f'📊 Dashboard: http://{self.get_ip_address()}:{self.web_port}/')

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
        if self.current_mode == DriveMode.TRAFFIC_LIGHT_WAIT:
            self.detect_traffic_light(processed)
        elif self.current_mode == DriveMode.RUBBERCON_AVOIDANCE:
            self.detect_and_avoid_rubbercon_enhanced(processed)
        elif self.current_mode == DriveMode.LANE_FOLLOWING:
            self.detect_lane_enhanced_2023(processed)
            # 차선 추종 중 방해차량 감지
            if self.detect_obstacle_car(processed):
                self.current_mode = DriveMode.OBSTACLE_CAR_AVOIDANCE
        elif self.current_mode == DriveMode.OBSTACLE_CAR_AVOIDANCE:
            self.avoid_obstacle_car(processed)
        
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
        
        # 미션 진행도 바
        progress_width = width - 20
        progress_height = 10
        cv2.rectangle(image, (10, 125), (10 + progress_width, 125 + progress_height), (100, 100, 100), -1)
        
        progress = 0
        if self.current_mode == DriveMode.TRAFFIC_LIGHT_WAIT:
            progress = 25
        elif self.current_mode == DriveMode.RUBBERCON_AVOIDANCE:
            progress = 50
        elif self.current_mode == DriveMode.LANE_FOLLOWING:
            progress = 75
        elif self.current_mode == DriveMode.OBSTACLE_CAR_AVOIDANCE:
            progress = 100
        
        if progress > 0:
            fill_width = int((progress / 100) * progress_width)
            cv2.rectangle(image, (10, 125), (10 + fill_width, 125 + progress_height), (0, 255, 0), -1)

def detect_traffic_light(self, image):
    """신호등 녹색불 감지 - 단순화된 방식"""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # autoracer5.py의 단순한 녹색 범위
    lower_green = np.array([35, 100, 100])
    upper_green = np.array([85, 255, 255])
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    
    # 노이즈 제거
    kernel = np.ones((3, 3), np.uint8)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel, iterations=2)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    
    # 녹색 영역의 총 픽셀 수 계산 (autoracer5 방식)
    green_pixel_count = np.sum(green_mask)
    
    # 시각화
    height, width = image.shape[:2]
    
    # 마스크를 컬러로 변환해서 표시
    mask_resized = cv2.resize(green_mask, (160, 120))
    mask_colored = cv2.applyColorMap(mask_resized, cv2.COLORMAP_JET)
    image[10:130, 10:170] = mask_colored
    
    # 녹색 영역이 충분히 크면 신호등으로 인식
    if green_pixel_count > 100000:  # autoracer5의 임계값 사용
        self.traffic_light_state = "GREEN"
        self.green_light_detection_count += 1
        self.traffic_light_confidence = min(100, green_pixel_count / 2000)
        
        # 일정 횟수 이상 감지되면 출발
        if self.green_light_detection_count > 10:  # 기존보다 낮춤
            if self.green_light_detected_time is None:
                self.green_light_detected_time = time.time()
                self.get_logger().info('🟢 Green light detected! Starting to move...')
            
            # 녹색불 감지 후 1초 후 출발
            if time.time() - self.green_light_detected_time > 1.0:
                self.traffic_light_passed = True
                self.current_mode = DriveMode.RUBBERCON_AVOIDANCE
                self.traffic_light_state = "GREEN_PASSED"
                self.get_logger().info('🚦 Traffic light passed! Moving to rubbercon avoidance')
        
        # 녹색 영역 시각화
        cv2.putText(image, f"GREEN DETECTED! Pixels: {int(green_pixel_count)}", 
                   (10, height-30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    else:
        self.green_light_detection_count = max(0, self.green_light_detection_count - 1)
        self.traffic_light_confidence = 0
        self.traffic_light_state = "RED_OR_YELLOW"
        
        cv2.putText(image, f"WAITING... Pixels: {int(green_pixel_count)}", 
                   (10, height-30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    
    # 신호등 대기 중 정지
    if not self.traffic_light_passed:
        self.target_speed = 0.0
        self.target_steering = 0.0
    
    # 디버그 정보
    cv2.putText(image, f"🚦 TRAFFIC LIGHT: {self.traffic_light_state} | Count: {self.green_light_detection_count}", 
               (10, image.shape[0]-60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    def detect_obstacle_car(self, image):
        """방해차량 감지 - NEW for 2025"""
        height, width = image.shape[:2]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 다양한 차량 색상 감지 (흰색, 검은색, 회색, 빨간색, 파란색)
        # 흰색 차량
        lower_white = np.array([0, 0, 180])
        upper_white = np.array([180, 30, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        
        # 검은색/어두운 회색 차량
        lower_dark = np.array([0, 0, 0])
        upper_dark = np.array([180, 255, 80])
        dark_mask = cv2.inRange(hsv, lower_dark, upper_dark)
        
        # 빨간색 차량
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])
        red_mask = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1),
                                 cv2.inRange(hsv, lower_red2, upper_red2))
        
        # 파란색 차량
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([130, 255, 255])
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        
        # 모든 마스크 결합
        car_mask = cv2.bitwise_or(white_mask, dark_mask)
        car_mask = cv2.bitwise_or(car_mask, red_mask)
        car_mask = cv2.bitwise_or(car_mask, blue_mask)
        
        # 노이즈 제거
        kernel_small = np.ones((3, 3), np.uint8)
        kernel_large = np.ones((7, 7), np.uint8)
        car_mask = cv2.morphologyEx(car_mask, cv2.MORPH_OPEN, kernel_small, iterations=2)
        car_mask = cv2.morphologyEx(car_mask, cv2.MORPH_CLOSE, kernel_large, iterations=2)
        
        # 관심 영역 설정 (전방 중앙 영역)
        roi_mask = np.zeros_like(car_mask)
        roi_mask[height//3:height*3//4, width//4:width*3//4] = 255
        car_mask = cv2.bitwise_and(car_mask, roi_mask)
        
        # 컨투어 검출
        contours, _ = cv2.findContours(car_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 차량 후보 필터링
        obstacle_cars = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # 최소 면적 (차량 크기 고려)
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w) / h
                
                # 차량 형태 검증 (가로가 세로보다 길거나 비슷)
                if 0.8 < aspect_ratio < 4.0 and w > 30 and h > 20:
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    # 차량 위치 분류
                    position = "center"
                    if center_x < width // 3:
                        position = "left"
                    elif center_x > width * 2 // 3:
                        position = "right"
                    
                    obstacle_cars.append({
                        'x': x, 'y': y, 'w': w, 'h': h,
                        'center_x': center_x,
                        'center_y': center_y,
                        'area': area,
                        'position': position,
                        'confidence': area * (1.0 / abs(aspect_ratio - 1.5))  # 이상적 비율에 가까울수록 높은 점수
                    })
                    
                    # 시각화
                    color = (0, 0, 255) if position == "center" else (0, 255, 255)
                    cv2.rectangle(image, (x, y), (x + w, y + h), color, 3)
                    cv2.putText(image, f'CAR-{position.upper()}({area:.0f})', (x, y - 10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    cv2.circle(image, (center_x, center_y), 5, color, -1)
        
        # 가장 신뢰도 높은 차량 선택
        if len(obstacle_cars) > 0:
            obstacle_cars.sort(key=lambda x: x['confidence'], reverse=True)
            best_car = obstacle_cars[0]
            
            # 신뢰도 계산
            self.obstacle_car_confidence = min(100, best_car['confidence'] / 50)
            
            # 일정 신뢰도 이상일 때만 방해차량으로 인식
            if self.obstacle_car_confidence > 60:
                self.obstacle_car_detected = True
                self.obstacle_car_position = best_car['position']
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
            self.get_logger().info(f'차량 회피 시작 - Position: {self.obstacle_car_position}')
        
        # 계속해서 방해차량 감지 시도
        car_detected = self.detect_obstacle_car(image)
        
        if car_detected and self.obstacle_car_position:
            # 방해차량 위치에 따른 회피 전략
            if self.obstacle_car_position == "left":
                # 좌측에 차량이 있으면 우측으로 회피
                self.target_steering = 0.3
                self.target_speed = 0.3
            elif self.obstacle_car_position == "right":
                # 우측에 차량이 있으면 좌측으로 회피
                self.target_steering = -0.3
                self.target_speed = 0.3
            elif self.obstacle_car_position == "center":
                # 중앙에 차량이 있으면 좌측으로 회피 (기본 전략)
                self.target_steering = -0.4
                self.target_speed = 0.25
            
            cv2.putText(image, f"AVOIDING CAR - {self.obstacle_car_position.upper()}", 
                       (10, image.shape[0]-90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            # 방해차량이 더 이상 감지되지 않으면
            if self.obstacle_avoidance_active:
                elapsed_time = time.time() - self.obstacle_avoidance_start_time
                
                # 최소 3초간 회피 행동 유지 후 차선 복귀
                if elapsed_time > 3.0:
                    self.obstacle_avoidance_active = False
                    self.current_mode = DriveMode.LANE_FOLLOWING
                    self.get_logger().info('차량 회피 완료 - 차선 추종으로 복귀')
                else:
                    # 아직 회피 중이면 이전 회피 동작 유지
                    pass
        
        # 차선 검출도 동시에 수행하여 기본 주행 유지
        self.detect_lane_enhanced_2023(image)

    def detect_and_avoid_rubbercon_enhanced(self, image):
        """라바콘 검출 및 회피 - 2023년 우승 알고리즘 기반 개선"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 2023년 우승팀 HSV 범위 - 더 정확한 주황색 검출
        lower_orange1 = np.array([5, 100, 100])
        upper_orange1 = np.array([25, 255, 255])
        
        lower_orange2 = np.array([165, 100, 100])
        upper_orange2 = np.array([180, 255, 255])
        
        lower_orange3 = np.array([25, 150, 150])
        upper_orange3 = np.array([35, 255, 255])
        
        # 마스크 생성 및 결합
        orange_mask1 = cv2.inRange(hsv, lower_orange1, upper_orange1)
        orange_mask2 = cv2.inRange(hsv, lower_orange2, upper_orange2) 
        orange_mask3 = cv2.inRange(hsv, lower_orange3, upper_orange3)
        
        orange_mask = cv2.bitwise_or(orange_mask1, orange_mask2)
        orange_mask = cv2.bitwise_or(orange_mask, orange_mask3)
        
        # 노이즈 제거
        kernel_small = np.ones((3, 3), np.uint8)
        kernel_large = np.ones((7, 7), np.uint8)
        
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, kernel_small, iterations=2)
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel_large, iterations=1)
        
        # 관심 영역 설정
        height, width = image.shape[:2]
        roi_mask = np.zeros_like(orange_mask)
        roi_mask[int(height*0.3):height, :] = 255
        orange_mask = cv2.bitwise_and(orange_mask, roi_mask)
        
        # 컨투어 검출
        contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 라바콘 후보 필터링
        rubbercons = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 200:
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w) / h
                center_y = y + h // 2
                solidity = area / (w * h)
                
                if (0.3 < aspect_ratio < 1.5 and
                    h > 15 and w > 10 and
                    center_y > height * 0.4 and
                    solidity > 0.4):
                    
                    perimeter = cv2.arcLength(contour, True)
                    if perimeter > 0:
                        compactness = 4 * math.pi * area / (perimeter * perimeter)
                        if compactness > 0.3:
                            rubbercons.append({
                                'x': x, 'y': y, 'w': w, 'h': h,
                                'center_x': x + w // 2,
                                'center_y': center_y,
                                'area': area,
                                'distance': height - center_y,
                                'confidence': area * solidity * compactness
                            })
                            
                            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 165, 255), 2)
                            cv2.putText(image, f'CONE({area:.0f})', (x, y - 10), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
                            cv2.circle(image, (x + w // 2, center_y), 5, (0, 165, 255), -1)
        
        # 신뢰도 높은 라바콘만 선택
        rubbercons.sort(key=lambda x: x['confidence'], reverse=True)
        rubbercons = rubbercons[:4]
        
        # 신뢰도 계산
        if len(rubbercons) > 0:
            total_confidence = sum([cone['confidence'] for cone in rubbercons])
            self.detection_confidence = min(100, total_confidence / 10)
        else:
            self.detection_confidence = 0
        
        # 라바콘 회피 처리
        self.process_rubbercon_avoidance_2023(rubbercons, image)
        
        # 디버그 정보
        cv2.putText(image, f"RUBBERCONS: {len(rubbercons)} detected | Flag: {self.rubbercon_detection_flag}", 
                   (10, image.shape[0]-60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
        
        # 마스크 시각화
        mask_resized = cv2.resize(orange_mask, (160, 120))
        mask_colored = cv2.applyColorMap(mask_resized, cv2.COLORMAP_JET)
        image[10:130, 10:170] = mask_colored

    def process_rubbercon_avoidance_2023(self, rubbercons, image):
        """2023년 우승팀 방식: flag 기반 라바콘 회피"""
        image_center = image.shape[1] // 2
        
        close_rubbercons = [cone for cone in rubbercons if cone['distance'] < 200]
        
        if len(close_rubbercons) >= 2:
            self.rubbercon_detection_flag = 1
            self.rubbercon_avoidance_active = True
            self.no_rubbercon_frames = 0
            
            left_cones = [cone for cone in close_rubbercons if cone['center_x'] < image_center - 50]
            right_cones = [cone for cone in close_rubbercons if cone['center_x'] > image_center + 50]
            
            if len(left_cones) > 0 and len(right_cones) > 0:
                best_left = max(left_cones, key=lambda x: x['confidence'])
                best_right = max(right_cones, key=lambda x: x['confidence'])
                
                left_distance = abs(best_left['center_x'] - (image_center - 100))
                right_distance = abs(best_right['center_x'] - (image_center + 100))
                
                error_sum = left_distance + right_distance
                target_x = (best_left['center_x'] + best_right['center_x']) // 2
                target_y = min(best_left['center_y'], best_right['center_y'])
                
                center_error = target_x - image_center
                error_correction = error_sum * 0.001
                
                if left_distance < right_distance:
                    center_error += error_correction
                else:
                    center_error -= error_correction
                
                self.calculate_steering_control(center_error, "RUBBERCON")
                
                # 시각화
                cv2.circle(image, (target_x, target_y), 10, (255, 0, 255), -1)
                cv2.putText(image, 'TARGET', (target_x - 30, target_y - 15), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
                cv2.line(image, (best_left['center_x'], best_left['center_y']),
                        (best_right['center_x'], best_right['center_y']), (255, 255, 0), 2)
                cv2.line(image, (image_center, image.shape[0]), 
                        (target_x, target_y), (255, 0, 255), 2)
                
            elif len(left_cones) > 0:
                best_left = max(left_cones, key=lambda x: x['confidence'])
                target_x = best_left['center_x'] + 120
                center_error = target_x - image_center
                self.calculate_steering_control(center_error, "RUBBERCON")
                
            elif len(right_cones) > 0:
                best_right = max(right_cones, key=lambda x: x['confidence'])
                target_x = best_right['center_x'] - 120
                center_error = target_x - image_center
                self.calculate_steering_control(center_error, "RUBBERCON")
                
        elif self.rubbercon_detection_flag == 1:
            self.no_rubbercon_frames += 1
            
            if self.no_rubbercon_frames > 10:
                self.rubbercon_passed = True
                self.current_mode = DriveMode.LANE_FOLLOWING
                self.rubbercon_detection_flag = 0
                self.get_logger().info('라바콘 회피 완료! 차선 추종으로 전환')
        
        # 회피 상태 표시
        if self.rubbercon_avoidance_active:
            status = f"AVOIDING - Flag: {self.rubbercon_detection_flag} | NoDetect: {self.no_rubbercon_frames}"
            cv2.putText(image, status, (10, image.shape[0]-90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 165, 0), 2)

    def detect_lane_enhanced_2023(self, image):
        """2023년 우승팀 차선 검출 - HSV 필터링 + Bird's Eye View + 슬라이딩 윈도우"""
        
        # Bird's Eye View 변환
        if self.bev_matrix is not None:
            bev_image = cv2.warpPerspective(image, self.bev_matrix, (640, 480))
        else:
            bev_image = image.copy()
        
        # HSV 기반 흰색 차선 검출
        hsv_bev = cv2.cvtColor(bev_image, cv2.COLOR_BGR2HSV)
        
        # 다중 마스크
        lower_white1 = np.array([0, 0, 200])
        upper_white1 = np.array([180, 25, 255])
        white_mask1 = cv2.inRange(hsv_bev, lower_white1, upper_white1)
        
        lower_white2 = np.array([0, 0, 160])
        upper_white2 = np.array([180, 40, 200])
        white_mask2 = cv2.inRange(hsv_bev, lower_white2, upper_white2)
        
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        yellow_mask = cv2.inRange(hsv_bev, lower_yellow, upper_yellow)
        
        # 밝기 기반 추가 검출
        gray_bev = cv2.cvtColor(bev_image, cv2.COLOR_BGR2GRAY)
        _, bright_mask = cv2.threshold(gray_bev, 180, 255, cv2.THRESH_BINARY)
        
        adaptive_mask = cv2.adaptiveThreshold(gray_bev, 255, cv2.ADAPTIVE_THRESH_MEAN_C, 
                                            cv2.THRESH_BINARY, 15, -10)
        
        # 모든 마스크 결합
        lane_mask = cv2.bitwise_or(white_mask1, white_mask2)
        lane_mask = cv2.bitwise_or(lane_mask, bright_mask)
        lane_mask = cv2.bitwise_or(lane_mask, adaptive_mask)
        lane_mask = cv2.bitwise_or(lane_mask, yellow_mask)
        
        # 노이즈 제거
        kernel_small = np.ones((3, 3), np.uint8)
        kernel_medium = np.ones((5, 5), np.uint8)
        
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_OPEN, kernel_small, iterations=1)
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, kernel_medium, iterations=2)
        
        # 슬라이딩 윈도우로 차선 검출
        left_line, right_line, lane_center = self.sliding_window_lane_detection_2023(lane_mask)
        
        # 결과를 원본 이미지에 오버레이
        self.draw_lane_overlay_2023(image, bev_image, lane_mask, left_line, right_line, lane_center)
        
        # 차선 중심 계산 및 제어
        if lane_center is not None:
            self.lane_detected = True
            image_center = image.shape[1] // 2
            
            steering_error = lane_center - image_center
            
            # 각도 기반 예외처리
            if left_line is not None and right_line is not None:
                height = lane_mask.shape[0]
                y_eval = height * 3 // 4
                
                left_slope = 2 * left_line[0] * y_eval + left_line[1]
                right_slope = 2 * right_line[0] * y_eval + right_line[1]
                
                slope_diff = abs(left_slope - right_slope)
                if slope_diff > 0.01:
                    correction = slope_diff * 50
                    if left_slope > right_slope:
                        steering_error += correction
                    else:
                        steering_error -= correction
            
            self.calculate_steering_control(steering_error, "LANE")
        else:
            self.lane_detected = False
            if hasattr(self, 'prev_error'):
                self.calculate_steering_control(self.prev_error * 0.5, "LANE")

    def sliding_window_lane_detection_2023(self, binary_image):
        """2023년 우승팀 슬라이딩 윈도우"""
        height, width = binary_image.shape
        
        # 히스토그램으로 차선의 시작점 찾기
        histogram = np.sum(binary_image[height*2//3:, :], axis=0)
        histogram = np.convolve(histogram, np.ones(10)/10, mode='same')
        
        midpoint = width // 2
        
        left_base = self.find_lane_base(histogram[:midpoint], self.prev_left_base, True)
        right_base = self.find_lane_base(histogram[midpoint:], self.prev_right_base - midpoint, False) + midpoint
        
        # 슬라이딩 윈도우 설정
        nwindows = 12
        window_height = height // nwindows
        margin = 60
        minpix = 30
        
        leftx_current = left_base
        rightx_current = right_base
        
        left_lane_inds = []
        right_lane_inds = []
        
        left_confidence = 0
        right_confidence = 0
        
        # 윈도우별로 검색
        for window in range(nwindows):
            win_y_low = height - (window + 1) * window_height
            win_y_high = height - window * window_height
            
            win_xleft_low = max(0, leftx_current - margin)
            win_xleft_high = min(width, leftx_current + margin)
            
            win_xright_low = max(0, rightx_current - margin)
            win_xright_high = min(width, rightx_current + margin)
            
            nonzero = binary_image[win_y_low:win_y_high, :].nonzero()
            nonzero_y = np.array(nonzero[0]) + win_y_low
            nonzero_x = np.array(nonzero[1])
            
            good_left_inds = ((nonzero_x >= win_xleft_low) & (nonzero_x < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzero_x >= win_xright_low) & (nonzero_x < win_xright_high)).nonzero()[0]
            
            if len(good_left_inds) > 0:
                left_lane_inds.append((nonzero_y[good_left_inds], nonzero_x[good_left_inds]))
                left_confidence += len(good_left_inds)
                
                if len(good_left_inds) > minpix:
                    leftx_current = int(np.mean(nonzero_x[good_left_inds]))
                    
            if len(good_right_inds) > 0:
                right_lane_inds.append((nonzero_y[good_right_inds], nonzero_x[good_right_inds]))
                right_confidence += len(good_right_inds)
                
                if len(good_right_inds) > minpix:
                    rightx_current = int(np.mean(nonzero_x[good_right_inds]))
        
        # 이전 프레임 정보 업데이트
        self.prev_left_base = leftx_current
        self.prev_right_base = rightx_current
        
        # 차선 피팅
        left_line = None
        right_line = None
        lane_center = None
        
        if len(left_lane_inds) > 5 and left_confidence > 200:
            left_y = np.concatenate([inds[0] for inds in left_lane_inds])
            left_x = np.concatenate([inds[1] for inds in left_lane_inds])
            
            if len(left_x) > 100:
                try:
                    left_line = np.polyfit(left_y, left_x, 2)
                except:
                    pass
        
        if len(right_lane_inds) > 5 and right_confidence > 200:
            right_y = np.concatenate([inds[0] for inds in right_lane_inds])
            right_x = np.concatenate([inds[1] for inds in right_lane_inds])
            
            if len(right_x) > 100:
                try:
                    right_line = np.polyfit(right_y, right_x, 2)
                except:
                    pass
        
        # 차선 중심 계산
        if left_line is not None and right_line is not None:
            y_eval = height * 3 // 4
            left_x_eval = left_line[0] * y_eval**2 + left_line[1] * y_eval + left_line[2]
            right_x_eval = right_line[0] * y_eval**2 + right_line[1] * y_eval + right_line[2]
            
            lane_width = abs(right_x_eval - left_x_eval)
            if 100 < lane_width < 400:
                lane_center = (left_x_eval + right_x_eval) / 2
                self.lane_confidence = min(100, (left_confidence + right_confidence) / 20)
            else:
                self.lane_confidence = 0
                
        elif left_line is not None:
            y_eval = height * 3 // 4
            left_x_eval = left_line[0] * y_eval**2 + left_line[1] * y_eval + left_line[2]
            lane_center = left_x_eval + 160
            self.lane_confidence = min(100, left_confidence / 15)
            
        elif right_line is not None:
            y_eval = height * 3 // 4
            right_x_eval = right_line[0] * y_eval**2 + right_line[1] * y_eval + right_line[2]
            lane_center = right_x_eval - 160
            self.lane_confidence = min(100, right_confidence / 15)
        else:
            self.lane_confidence = 0
        
        return left_line, right_line, lane_center

    def find_lane_base(self, histogram, prev_base, is_left):
        """이전 프레임 정보를 활용한 차선 베이스 찾기"""
        if prev_base > 0 and prev_base < len(histogram):
            search_range = 50
            start_idx = max(0, prev_base - search_range)
            end_idx = min(len(histogram), prev_base + search_range)
            
            local_max_idx = np.argmax(histogram[start_idx:end_idx])
            local_max_val = histogram[start_idx + local_max_idx]
            
            if local_max_val > 100:
                return start_idx + local_max_idx
        
        # 전체 영역에서 최댓값 찾기
        max_idx = np.argmax(histogram)
        if histogram[max_idx] > 50:
            return max_idx
        
        # 기본값 반환
        return len(histogram) // 2 if is_left else len(histogram) // 2

    def draw_lane_overlay_2023(self, original_image, bev_image, lane_mask, left_line, right_line, lane_center):
        """2023년 방식 차선 검출 결과 오버레이"""
        height, width = bev_image.shape[:2]
        
        # BEV 결과를 컬러로 변환
        bev_colored = cv2.cvtColor(lane_mask, cv2.COLOR_GRAY2BGR)
        
        # 차선 곡선 그리기
        plot_y = np.linspace(0, height-1, height).astype(int)
        
        if left_line is not None:
            left_fitx = (left_line[0] * plot_y**2 + left_line[1] * plot_y + left_line[2]).astype(int)
            left_fitx = np.clip(left_fitx, 0, width-1)
            left_points = np.array(list(zip(left_fitx, plot_y)), dtype=np.int32)
            cv2.polylines(bev_colored, [left_points], False, (255, 100, 100), 8)
        
        if right_line is not None:
            right_fitx = (right_line[0] * plot_y**2 + right_line[1] * plot_y + right_line[2]).astype(int)
            right_fitx = np.clip(right_fitx, 0, width-1)
            right_points = np.array(list(zip(right_fitx, plot_y)), dtype=np.int32)
            cv2.polylines(bev_colored, [right_points], False, (100, 100, 255), 8)
        
        # 차선 사이 영역 채우기
        if left_line is not None and right_line is not None:
            left_fitx = (left_line[0] * plot_y**2 + left_line[1] * plot_y + left_line[2]).astype(int)
            right_fitx = (right_line[0] * plot_y**2 + right_line[1] * plot_y + right_line[2]).astype(int)
            left_fitx = np.clip(left_fitx, 0, width-1)
            right_fitx = np.clip(right_fitx, 0, width-1)
            
            pts_left = np.array([np.transpose(np.vstack([left_fitx, plot_y]))])
            pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, plot_y])))])
            pts = np.hstack((pts_left, pts_right))
            cv2.fillPoly(bev_colored, np.int_([pts]), (0, 100, 0))
        
        # 차선 중심선 그리기
        if lane_center is not None:
            center_int = int(np.clip(lane_center, 0, width-1))
            cv2.line(bev_colored, (center_int, height//2), (center_int, height), (0, 255, 255), 6)
        
        # 역변환하여 원본 이미지에 오버레이
        if self.inv_bev_matrix is not None:
            lane_overlay = cv2.warpPerspective(bev_colored, self.inv_bev_matrix, 
                                             (original_image.shape[1], original_image.shape[0]))
            
            mask = (lane_overlay.sum(axis=2) > 0).astype(np.uint8)
            for c in range(3):
                original_image[:,:,c] = np.where(mask, 
                    cv2.addWeighted(original_image[:,:,c], 0.6, lane_overlay[:,:,c], 0.4, 0),
                    original_image[:,:,c])
        
        # BEV 마스크 미니뷰
        mask_resized = cv2.resize(lane_mask, (200, 150))
        mask_colored_mini = cv2.applyColorMap(mask_resized, cv2.COLORMAP_RAINBOW)
        x_offset = original_image.shape[1] - 210
        y_offset = 130
        original_image[y_offset:y_offset+150, x_offset:x_offset+200] = mask_colored_mini
        cv2.putText(original_image, "BEV MASK", (x_offset, y_offset-10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # 차선 상태 표시
        if lane_center is not None:
            cv2.putText(original_image, f"LANE CENTER: {int(lane_center)} | Conf: {self.lane_confidence:.1f}%", 
                       (10, original_image.shape[0]-120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        else:
            cv2.putText(original_image, "SEARCHING FOR LANE... | Applying previous control", 
                       (10, original_image.shape[0]-120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 100, 255), 2)

    def calculate_steering_control(self, error, mode):
        """2023년 우승팀 Pure Pursuit 제어 방식"""
        # 모드별 제어 게인
        if mode == "RUBBERCON":
            kp, ki, kd = 0.0035, 0.00008, 0.002
            max_steering = 0.5
            max_speed_factor = 0.8
        elif mode == "LANE":
            kp, ki, kd = 0.0025, 0.00005, 0.0018
            max_steering = 0.35
            max_speed_factor = 1.0
        else:
            kp, ki, kd = 0.002, 0.0001, 0.001
            max_steering = 0.3
            max_speed_factor = 0.7
        
        # PID 계산
        dt = 0.05
        self.integral_error += error * dt
        derivative_error = (error - self.prev_error) / dt
        
        # 적분 windup 방지
        integral_limit = 800
        if abs(self.integral_error) > integral_limit:
            self.integral_error = integral_limit * np.sign(self.integral_error)
        
        # PID 출력
        steering_output = -(kp * error + ki * self.integral_error + kd * derivative_error)
        
        # 속도 연동 조향 제한
        speed_factor = max(0.5, 1.0 - abs(self.current_speed) * 0.3)
        max_steering *= speed_factor
        
        # 조향각 제한
        self.target_steering = max(-max_steering, min(max_steering, steering_output))
        self.prev_error = error
        
        # 속도 조정
        steering_magnitude = abs(self.target_steering)
        if steering_magnitude > 0.3:
            speed_reduction = (steering_magnitude - 0.3) * 2
            max_speed_factor *= max(0.4, 1.0 - speed_reduction)
        
        # 목표 속도 업데이트
        if mode == "RUBBERCON":
            base_speed = 0.35
        elif mode == "LANE":
            if steering_magnitude < 0.1:
                base_speed = 0.5
            elif steering_magnitude < 0.25:
                base_speed = 0.25
            else:
                base_speed = 0.1
        else:
            base_speed = 0.2
            
        self.target_speed = base_speed * max_speed_factor

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
        
        # 전방 영역 거리 계산
        front_range = min(40, total_points // 9)
        front_ranges = ranges[center-front_range:center+front_range]
        valid_ranges = [r for r in front_ranges if 0.05 < r < 10.0]
        
        if valid_ranges:
            avg_distance = sum(valid_ranges) / len(valid_ranges)
            min_distance = min(valid_ranges)
            
            # 거리에 따른 색상 및 상태
            if min_distance < 0.15:
                color = (0, 0, 255)
                status = "CRITICAL"
            elif min_distance < 0.3:
                color = (0, 255, 255)
                status = "WARNING"
            else:
                color = (0, 255, 0)
                status = "CLEAR"
            
            # 라이다 정보 패널
            panel_width, panel_height = 280, 120
            panel_x, panel_y = width - panel_width - 10, height - panel_height - 10
            
            # 반투명 배경
            overlay = image.copy()
            cv2.rectangle(overlay, (panel_x, panel_y), (panel_x + panel_width, panel_y + panel_height), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)
            
            # 텍스트 정보
            cv2.putText(image, "LIDAR STATUS", (panel_x + 10, panel_y + 25), 
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
        minimap_size = 120
        minimap_x = 20
        minimap_y = 320
        
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
                if 0.05 < distance < 3.0:
                    angle = i * angle_increment - math.pi
                    
                    x = int(center_x + (distance * minimap_size / 6) * math.cos(angle))
                    y = int(center_y - (distance * minimap_size / 6) * math.sin(angle))
                    
                    if minimap_x <= x <= minimap_x + minimap_size and minimap_y <= y <= minimap_y + minimap_size:
                        color = (0, 255, 255) if distance < 1.0 else (255, 255, 255)
                        cv2.circle(image, (x, y), 1, color, -1)
        
        # 미니맵 제목
        cv2.putText(image, "LIDAR", (minimap_x, minimap_y - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    def lidar_callback(self, msg):
        """라이다 콜백 - 장애물 검출 및 응급정지"""
        self.lidar_data = msg
        
        if len(msg.ranges) == 0:
            return
        
        # 전방 장애물 검사
        total_points = len(msg.ranges)
        center = total_points // 2
        front_range = min(25, total_points // 12)
        
        front_ranges = msg.ranges[center-front_range:center+front_range]
        valid_ranges = [r for r in front_ranges if 0.05 < r < 10.0]
        
        if valid_ranges:
            min_distance = min(valid_ranges)
            
            # 응급정지 조건 (15cm 이내)
            if min_distance < 0.15:
                if self.current_mode != DriveMode.EMERGENCY_STOP:
                    self.current_mode = DriveMode.EMERGENCY_STOP
                    self.emergency_start_time = time.time()
                    self.get_logger().warn('EMERGENCY STOP - Obstacle detected!')

    def control_loop(self):
        """메인 제어 루프 - 20Hz"""
        cmd = Twist()
        
        # 부드러운 제어를 위한 저역 통과 필터
        alpha_speed = 0.4
        alpha_steering = 0.6
        
        self.current_speed = alpha_speed * self.target_speed + (1 - alpha_speed) * self.current_speed
        self.current_steering = alpha_steering * self.target_steering + (1 - alpha_steering) * self.current_steering
        
        if self.current_mode == DriveMode.TRAFFIC_LIGHT_WAIT:
            # 신호등 대기 - 정지 상태 유지
            if not self.traffic_light_passed:
                self.target_speed = 0.0
                self.target_steering = 0.0
            
        elif self.current_mode == DriveMode.RUBBERCON_AVOIDANCE:
            # 라바콘 회피
            if not hasattr(self, 'target_speed') or self.target_speed == 0:
                self.target_speed = 0.35
            
        elif self.current_mode == DriveMode.LANE_FOLLOWING:
            # 차선 추종
            if getattr(self, 'lane_detected', False):
                if not hasattr(self, 'target_speed') or self.target_speed == 0:
                    steering_magnitude = abs(self.current_steering)
                    if steering_magnitude < 0.1:
                        self.target_speed = 0.5
                    elif steering_magnitude < 0.25:
                        self.target_speed = 0.25
                    else:
                        self.target_speed = 0.1
            else:
                # 차선을 찾지 못한 경우
                self.target_speed = 0.2
                if not hasattr(self, 'lane_search_count'):
                    self.lane_search_count = 0
                self.lane_search_count += 1
                
                if self.lane_search_count > 40:
                    search_steering = 0.1 * math.sin(self.lane_search_count * 0.2)
                    self.target_steering = search_steering
                    if self.lane_search_count > 100:
                        self.lane_search_count = 0
        
        elif self.current_mode == DriveMode.OBSTACLE_CAR_AVOIDANCE:
            # 방해차량 회피 - avoid_obstacle_car에서 처리됨
            pass
                
        elif self.current_mode == DriveMode.EMERGENCY_STOP:
            # 응급정지
            self.target_speed = 0.0
            self.target_steering = 0.0
            
            # 2초 후 이전 모드로 복귀
            if hasattr(self, 'emergency_start_time'):
                if time.time() - self.emergency_start_time > 2.0:
                    if not self.traffic_light_passed:
                        self.current_mode = DriveMode.TRAFFIC_LIGHT_WAIT
                    elif not self.rubbercon_passed:
                        self.current_mode = DriveMode.RUBBERCON_AVOIDANCE
                    else:
                        self.current_mode = DriveMode.LANE_FOLLOWING
                    
                    delattr(self, 'emergency_start_time')
                    self.get_logger().info('Emergency cleared - resuming mission')
        
        # 1회 주행 완료 체크
        if (self.current_mode == DriveMode.LANE_FOLLOWING and 
            self.traffic_light_passed and self.rubbercon_passed):
            
            # 시작 지점 근처로 돌아왔는지 체크 (간단한 예시)
            # 실제로는 GPS나 특정 마커를 사용해야 함
            elapsed_time = time.time() - self.mission_start_time
            if elapsed_time > 180 and not self.one_lap_completed:  # 3분 후 (예시)
                self.one_lap_completed = True
                self.target_speed = 0.0
                self.target_steering = 0.0
                self.get_logger().info('1회 주행 완료! 미션 성공!')
        
        # 최종 명령 설정
        cmd.linear.x = float(self.current_speed)
        cmd.angular.z = float(self.current_steering)
        
        # 안전 제한
        cmd.linear.x = max(0.0, min(0.8, cmd.linear.x))
        cmd.angular.z = max(-0.6, min(0.6, cmd.angular.z))
        
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
            front_range = min(20, len(self.lidar_data.ranges) // 10)
            front_ranges = self.lidar_data.ranges[center-front_range:center+front_range]
            valid_ranges = [r for r in front_ranges if 0.05 < r < 10.0]
            if valid_ranges:
                lidar_distance = f"{min(valid_ranges):.2f}"
        
        # 신호등 상태
        traffic_light_status = "SEARCHING"
        if self.traffic_light_passed:
            traffic_light_status = "PASSED"
        elif self.traffic_light_state == "GREEN":
            traffic_light_status = f"GREEN DETECTED ({self.green_light_detection_count})"
        elif self.traffic_light_state == "RED_OR_YELLOW":
            traffic_light_status = "RED/YELLOW"
        
        # 라바콘 상태
        rubbercon_status = "SEARCHING"
        if self.rubbercon_passed:
            rubbercon_status = "PASSED"
        elif getattr(self, 'rubbercon_avoidance_active', False):
            rubbercon_status = f"AVOIDING (Flag:{self.rubbercon_detection_flag})"
        elif self.detection_confidence > 50:
            rubbercon_status = "DETECTED"
        
        # 차선 상태
        lane_status = "SEARCHING"
        if getattr(self, 'lane_detected', False):
            lane_status = f"FOLLOWING (Conf:{self.lane_confidence:.1f}%)"
        elif hasattr(self, 'lane_confidence') and self.lane_confidence > 30:
            lane_status = "DETECTED"
        
        # 방해차량 상태
        obstacle_car_status = "NONE"
        if self.obstacle_car_detected:
            obstacle_car_status = f"DETECTED-{self.obstacle_car_position.upper()}"
        elif getattr(self, 'obstacle_avoidance_active', False):
            obstacle_car_status = "AVOIDING"
        
        return {
            "current_mode": self.current_mode.value,
            "traffic_light_status": traffic_light_status,
            "rubbercon_status": rubbercon_status,
            "lane_status": lane_status,
            "obstacle_car_status": obstacle_car_status,
            "detection_confidence": f"{self.detection_confidence:.1f}",
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
            print("\nAutoracer 2025 Contest Ended!")
        finally:
            if 'autoracer' in locals():
                autoracer.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
