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
import json

qos_profile = QoSProfile(depth=10)
qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

class DriveMode(Enum):
    WAITING_FOR_GREEN = "WAITING_FOR_GREEN"
    RUBBERCON_AVOIDANCE = "RUBBERCON_AVOID"
    LANE_FOLLOWING = "LANE_FOLLOW"
    ZIGZAG_MODE = "ZIGZAG"
    EMERGENCY_STOP = "EMERGENCY_STOP"

class WebViewer(BaseHTTPRequestHandler):
    """
    HTTP Request Handler for the web dashboard.
    Serves the main HTML page and live streams camera and telemetry data.
    """
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
                <title>🚗 Autoracer 2025 Contest - Enhanced</title>
                <style>
                    body { background: linear-gradient(135deg, #1e1e1e, #2d2d30); color: #fff; font-family: 'Segoe UI', Arial; margin: 0; padding: 20px; }
                    .container { display: flex; flex-wrap: wrap; gap: 20px; max-width: 1400px; margin: 0 auto; }
                    .panel { background: rgba(255,255,255,0.1); border-radius: 12px; padding: 20px; backdrop-filter: blur(10px); flex: 1; min-width: 300px; }
                    .main-feed { flex: 2; min-width: 600px; }
                    .status-card { background: rgba(0,255,0,0.1); border-left: 4px solid #00ff00; margin: 10px 0; padding: 15px; border-radius: 8px; }
                    .metric { display: flex; justify-content: space-between; margin: 8px 0; }
                    .metric-value { font-weight: bold; color: #00ff88; }
                    h1 { text-align: center; color: #00ff88; text-shadow: 0 0 20px #00ff88; }
                    h3 { color: #00ccff; margin-top: 0; }
                    .progress-bar { width: 100%; height: 20px; background: #333; border-radius: 10px; overflow: hidden; margin: 10px 0; }
                    .progress-fill { height: 100%; background: linear-gradient(90deg, #00ff00, #00ccff); transition: width 0.3s; }
                    .alert-box { color: #ff0000; font-weight: bold; margin-top: 15px; }
                    @media (max-width: 1000px) {
                        .container { flex-direction: column; }
                    }
                </style>
            </head>
            <body>
                <h1>🏆 Autoracer 2025 Contest - Enhanced Detection</h1>
                <div class="container">
                    <div class="panel main-feed">
                        <h3>📹 Live Camera Feed</h3>
                        <img src="/stream.mjpg" width="800" height="600" style="border: 2px solid #444; border-radius: 8px; width: 100%; max-width: 800px;">
                    </div>
                    <div class="panel">
                        <h3>🎯 Mission Control</h3>
                        <div class="status-card">
                            <div class="metric"><span>Current Mode:</span><span id="mode" class="metric-value">Loading...</span></div>
                            <div class="metric"><span>Rubbercon Status:</span><span id="rubbercon" class="metric-value">Searching...</span></div>
                            <div class="metric"><span>Lane Status:</span><span id="lane" class="metric-value">Detecting...</span></div>
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
                            <p id="mission1" style="color: #ffaa00;">🔄 Mission 1: Rubbercon Avoidance</p>
                            <p id="mission2" style="color: #666;">⏳ Mission 2: Lane Following</p>
                        </div>
                        <div id="alerts" class="alert-box"></div>
                    </div>
                </div>
                
                <script>
                let startTime = Date.now();
                setInterval(() => {
                    fetch('/stats')
                    .then(r => r.json())
                    .then(data => {
                        document.getElementById('mode').textContent = data.current_mode;
                        document.getElementById('rubbercon').textContent = data.rubbercon_status;
                        document.getElementById('lane').textContent = data.lane_status;
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
                        if (data.current_mode.includes('RUBBERCON')) progress = 50;
                        else if (data.current_mode.includes('LANE')) progress = 100;
                        document.getElementById('progress').style.width = progress + '%';
                        
                        // Mission status colors
                        const missions = ['mission1', 'mission2'];
                        missions.forEach((m, i) => {
                            const elem = document.getElementById(m);
                            if (i * 50 < progress) elem.style.color = '#00ff00';
                            else if (i * 50 === progress - 50) elem.style.color = '#ffaa00';
                        });
                        
                        // Alerts
                        let alerts = [];
                        if (data.lidar_distance !== 'N/A' && parseFloat(data.lidar_distance) < 0.3) alerts.push('⚠️ Obstacle Too Close');
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
                        self.wfile.write(buffer.tobytes())
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
    """
    Main ROS2 node for the Autoracer, managing sensors,
    image processing, and vehicle control logic.
    """
    def __init__(self):
        super().__init__('Autoracer')
        
        # Image and Sensor Data
        self.current_image = None
        self.processed_frame = None
        self.lidar_data = None
        self.image_lock = threading.Lock()
        
        # Mission State Management
        self.current_mode = DriveMode.RUBBERCON_AVOIDANCE
        self.rubbercon_passed = False
        self.lane_following_started = False
        self.lane_detected = False
        self.lane_search_count = 0
        
        # Control Variables
        self.current_speed = 0.0
        self.current_steering = 0.0
        self.target_speed = 0.0
        self.target_steering = 0.0
        
        # PID Control Variables for Lane Following
        self.prev_error = 0.0
        self.integral_error = 0.0
        self.last_lane_center = 320
        
        # Rubbercon Avoidance State
        self.rubbercon_detection_flag = 0
        self.no_rubbercon_frames = 0
        
        # Statistics and Performance
        self.frame_count = 0
        self.start_time = time.time()
        self.last_camera_time = 0
        self.camera_fps = 0
        self.mission_start_time = time.time()
        self.detection_confidence = 0.0
        
        # Bird's Eye View (BEV) Transform
        self.bev_matrix = None
        self.inv_bev_matrix = None
        self.setup_bev_transform()
        
        # Lane Detection Sliding Window
        self.prev_left_base = 160
        self.prev_right_base = 480
        
        # ROS2 Subscriptions
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
        
        # ROS2 Publishers
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Start Web Server
        self.web_port = 8080
        self.start_web_server()
        
        # Control Loop Timer (20Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('🏆 Autoracer 2025 Contest Started - Enhanced!')
        self.get_logger().info(f'📊 Dashboard: http://{self.get_ip_address()}:{self.web_port}/')

    def setup_bev_transform(self):
        """Sets up the perspective transform matrix for Bird's Eye View."""
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
        """Fetches the local IP address for dashboard access."""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "localhost"

    def start_web_server(self):
        """Starts a web server in a separate thread for the dashboard."""
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
                    if e.errno == 98:  # Address already in use
                        continue
                    else:
                        self.get_logger().error(f'Web server error: {e}')
                        break
        
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()

    def image_callback(self, msg):
        """Processes incoming camera images."""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if image is not None:
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
        """Routes image processing based on the current drive mode."""
        processed = image.copy()
        
        self.draw_status_header(processed)
        
        if self.current_mode == DriveMode.RUBBERCON_AVOIDANCE:
            self.detect_and_avoid_rubbercon_enhanced(processed)
        elif self.current_mode == DriveMode.LANE_FOLLOWING:
            self.detect_lane_enhanced_2023(processed)
        
        if self.lidar_data is not None:
            self.draw_lidar_overlay(processed)
        
        with self.image_lock:
            self.processed_frame = processed.copy()

    def draw_status_header(self, image):
        """Draws a semi-transparent header with status information."""
        height, width = image.shape[:2]
        overlay = image.copy()
        cv2.rectangle(overlay, (0, 0), (width, 120), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)
        
        elapsed = time.time() - self.mission_start_time
        time_str = f"{int(elapsed//60):02d}:{int(elapsed%60):02d}"
        
        cv2.putText(image, f'🏆 Enhanced 2025 | Mode: {self.current_mode.value}', 
                   (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(image, f'⏱️ Time: {time_str} | Frame: {self.frame_count} | FPS: {self.camera_fps}', 
                   (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(image, f'🚗 Speed: {self.current_speed:.2f} | 🎯 Steer: {math.degrees(self.current_steering):.1f}° | Conf: {self.detection_confidence:.1f}%', 
                   (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        progress_width = width - 20
        progress_height = 10
        cv2.rectangle(image, (10, 100), (10 + progress_width, 100 + progress_height), (100, 100, 100), -1)
        
        progress = 0
        if self.current_mode == DriveMode.RUBBERCON_AVOIDANCE:
            progress = 50
        elif self.current_mode == DriveMode.LANE_FOLLOWING:
            progress = 100
        
        if progress > 0:
            fill_width = int((progress / 100) * progress_width)
            cv2.rectangle(image, (10, 100), (10 + fill_width, 100 + progress_height), (0, 255, 0), -1)

    def detect_and_avoid_rubbercon_enhanced(self, image):
        """Detects and avoids rubbercones using improved HSV filtering and contour analysis."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        lower_orange1 = np.array([5, 100, 100])
        upper_orange1 = np.array([25, 255, 255])
        
        lower_orange2 = np.array([165, 100, 100])
        upper_orange2 = np.array([180, 255, 255])
        
        lower_orange3 = np.array([25, 150, 150])
        upper_orange3 = np.array([35, 255, 255])
        
        orange_mask1 = cv2.inRange(hsv, lower_orange1, upper_orange1)
        orange_mask2 = cv2.inRange(hsv, lower_orange2, upper_orange2)
        orange_mask3 = cv2.inRange(hsv, lower_orange3, upper_orange3)
        
        orange_mask = cv2.bitwise_or(orange_mask1, orange_mask2)
        orange_mask = cv2.bitwise_or(orange_mask, orange_mask3)
        
        kernel_small = np.ones((3, 3), np.uint8)
        kernel_large = np.ones((7, 7), np.uint8)
        
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, kernel_small, iterations=2)
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel_large, iterations=1)
        
        height, width = image.shape[:2]
        roi_mask = np.zeros_like(orange_mask)
        roi_mask[int(height * 0.3):height, :] = 255
        orange_mask = cv2.bitwise_and(orange_mask, roi_mask)
        
        contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
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
        
        rubbercons.sort(key=lambda x: x['confidence'], reverse=True)
        rubbercons = rubbercons[:4]
        
        if len(rubbercons) > 0:
            total_confidence = sum([cone['confidence'] for cone in rubbercons])
            self.detection_confidence = min(100, total_confidence / 10)
        else:
            self.detection_confidence = 0
        
        self.process_rubbercon_avoidance_2023(rubbercons, image)
        
        cv2.putText(image, f"🎯 RUBBERCONS: {len(rubbercons)} detected | Flag: {self.rubbercon_detection_flag}",
                   (10, height - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
        
        mask_resized = cv2.resize(orange_mask, (160, 120))
        mask_colored = cv2.applyColorMap(mask_resized, cv2.COLORMAP_JET)
        image[10:130, 10:170] = mask_colored

    def process_rubbercon_avoidance_2023(self, rubbercons, image):
        """Implements a flag-based rubbercone avoidance strategy."""
        image_center = image.shape[1] // 2
        
        close_rubbercons = [cone for cone in rubbercons if cone['distance'] < 200]
        
        if len(close_rubbercons) >= 2:
            self.rubbercon_detection_flag = 1
            self.rubbercon_avoidance_active = True
            self.no_rubbercon_frames = 0
            
            left_cones = [cone for cone in close_rubbercons if cone['center_x'] < image_center - 50]
            right_cones = [cone for cone in close_rubbercons if cone['center_x'] > image_center + 50]
            
            if left_cones and right_cones:
                best_left = max(left_cones, key=lambda x: x['confidence'])
                best_right = max(right_cones, key=lambda x: x['confidence'])
                
                target_x = (best_left['center_x'] + best_right['center_x']) // 2
                target_y = min(best_left['center_y'], best_right['center_y'])
                
                center_error = target_x - image_center
                
                self.calculate_steering_control(center_error, "RUBBERCON")
                
                cv2.circle(image, (target_x, target_y), 10, (255, 0, 255), -1)
                cv2.putText(image, '🎯 TARGET', (target_x - 30, target_y - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
                cv2.line(image, (best_left['center_x'], best_left['center_y']),
                         (best_right['center_x'], best_right['center_y']), (255, 255, 0), 2)
                cv2.line(image, (image_center, image.shape[0]),
                         (target_x, target_y), (255, 0, 255), 2)
                
            elif left_cones:
                best_left = max(left_cones, key=lambda x: x['confidence'])
                target_x = best_left['center_x'] + 120
                center_error = target_x - image_center
                self.calculate_steering_control(center_error, "RUBBERCON")
            
            elif right_cones:
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
                self.get_logger().info('🎯 Rubbercon avoidance completed! Switching to lane following')
        
        if self.rubbercon_avoidance_active:
            status = f"🚧 AVOIDING - Flag: {self.rubbercon_detection_flag} | NoDetect: {self.no_rubbercon_frames}"
            cv2.putText(image, status, (10, image.shape[0] - 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 165, 0), 2)

    def detect_lane_enhanced_2023(self, image):
        """
        Detects lanes using a robust combination of HSV, adaptive thresholding,
        and a sliding window approach on a Bird's Eye View image.
        """
        if self.bev_matrix is None:
            bev_image = image.copy()
        else:
            bev_image = cv2.warpPerspective(image, self.bev_matrix, (640, 480))
        
        hsv_bev = cv2.cvtColor(bev_image, cv2.COLOR_BGR2HSV)
        
        lower_white1 = np.array([0, 0, 200])
        upper_white1 = np.array([180, 25, 255])
        white_mask1 = cv2.inRange(hsv_bev, lower_white1, upper_white1)
        
        lower_white2 = np.array([0, 0, 160])
        upper_white2 = np.array([180, 40, 200])
        white_mask2 = cv2.inRange(hsv_bev, lower_white2, upper_white2)
        
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        yellow_mask = cv2.inRange(hsv_bev, lower_yellow, upper_yellow)
        
        gray_bev = cv2.cvtColor(bev_image, cv2.COLOR_BGR2GRAY)
        _, bright_mask = cv2.threshold(gray_bev, 180, 255, cv2.THRESH_BINARY)
        
        adaptive_mask = cv2.adaptiveThreshold(gray_bev, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                                cv2.THRESH_BINARY, 15, -10)
        
        lane_mask = cv2.bitwise_or(white_mask1, white_mask2)
        lane_mask = cv2.bitwise_or(lane_mask, bright_mask)
        lane_mask = cv2.bitwise_or(lane_mask, adaptive_mask)
        lane_mask = cv2.bitwise_or(lane_mask, yellow_mask)
        
        kernel_small = np.ones((3, 3), np.uint8)
        kernel_medium = np.ones((5, 5), np.uint8)
        
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_OPEN, kernel_small, iterations=1)
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, kernel_medium, iterations=2)
        
        left_line, right_line, lane_center = self.sliding_window_lane_detection_2023(lane_mask)
        
        self.draw_lane_overlay_2023(image, bev_image, lane_mask, left_line, right_line, lane_center)
        
        if lane_center is not None:
            self.lane_detected = True
            image_center = image.shape[1] // 2
            steering_error = lane_center - image_center
            
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
            self.calculate_steering_control(self.prev_error * 0.5, "LANE")
            
    def sliding_window_lane_detection_2023(self, binary_image):
        """
        Uses a sliding window approach to find and fit lane lines.
        Returns the fitted polynomial lines and the calculated lane center.
        """
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
        
        left_lane_inds = []
        right_lane_inds = []
        
        left_confidence = 0
        right_confidence = 0
        
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
                left_lane_inds.extend(zip(nonzero_y[good_left_inds], nonzero_x[good_left_inds]))
                left_confidence += len(good_left_inds)
                if len(good_left_inds) > minpix:
                    leftx_current = int(np.mean(nonzero_x[good_left_inds]))
                    
            if len(good_right_inds) > 0:
                right_lane_inds.extend(zip(nonzero_y[good_right_inds], nonzero_x[good_right_inds]))
                right_confidence += len(good_right_inds)
                if len(good_right_inds) > minpix:
                    rightx_current = int(np.mean(nonzero_x[good_right_inds]))
        
        self.prev_left_base = leftx_current
        self.prev_right_base = rightx_current
        
        left_line = None
        right_line = None
        lane_center = None
        
        if len(left_lane_inds) > 5 and left_confidence > 200:
            left_y, left_x = zip(*left_lane_inds)
            left_y = np.array(left_y)
            left_x = np.array(left_x)
            if len(left_x) > 100:
                try:
                    left_line = np.polyfit(left_y, left_x, 2)
                except Exception:
                    pass
        
        if len(right_lane_inds) > 5 and right_confidence > 200:
            right_y, right_x = zip(*right_lane_inds)
            right_y = np.array(right_y)
            right_x = np.array(right_x)
            if len(right_x) > 100:
                try:
                    right_line = np.polyfit(right_y, right_x, 2)
                except Exception:
                    pass
        
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
        """Helps find the starting point of a lane line using a histogram."""
        search_range = 50
        start_idx = max(0, prev_base - search_range)
        end_idx = min(len(histogram), prev_base + search_range)
        
        local_max_idx = np.argmax(histogram[start_idx:end_idx])
        local_max_val = histogram[start_idx + local_max_idx]
        
        if local_max_val > 100:
            return start_idx + local_max_idx
        
        max_idx = np.argmax(histogram)
        if histogram[max_idx] > 50:
            return max_idx
        
        return len(histogram) // 2 if is_left else len(histogram) // 2

    def draw_lane_overlay_2023(self, original_image, bev_image, lane_mask, left_line, right_line, lane_center):
        """Overlays lane detection results onto the original image."""
        height, width = bev_image.shape[:2]
        bev_colored = cv2.cvtColor(lane_mask, cv2.COLOR_GRAY2BGR)
        plot_y = np.linspace(0, height - 1, height).astype(int)
        
        if left_line is not None:
            left_fitx = (left_line[0] * plot_y**2 + left_line[1] * plot_y + left_line[2]).astype(int)
            left_fitx = np.clip(left_fitx, 0, width - 1)
            left_points = np.array(list(zip(left_fitx, plot_y)), dtype=np.int32)
            cv2.polylines(bev_colored, [left_points], False, (255, 100, 100), 8)
        
        if right_line is not None:
            right_fitx = (right_line[0] * plot_y**2 + right_line[1] * plot_y + right_line[2]).astype(int)
            right_fitx = np.clip(right_fitx, 0, width - 1)
            right_points = np.array(list(zip(right_fitx, plot_y)), dtype=np.int32)
            cv2.polylines(bev_colored, [right_points], False, (100, 100, 255), 8)
        
        if left_line is not None and right_line is not None:
            left_fitx = np.clip((left_line[0] * plot_y**2 + left_line[1] * plot_y + left_line[2]).astype(int), 0, width - 1)
            right_fitx = np.clip((right_line[0] * plot_y**2 + right_line[1] * plot_y + right_line[2]).astype(int), 0, width - 1)
            pts_left = np.array([np.transpose(np.vstack([left_fitx, plot_y]))])
            pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, plot_y])))])
            pts = np.hstack((pts_left, pts_right))
            cv2.fillPoly(bev_colored, np.int_([pts]), (0, 100, 0))
        
        if lane_center is not None:
            center_int = int(np.clip(lane_center, 0, width - 1))
            cv2.line(bev_colored, (center_int, height // 2), (center_int, height), (0, 255, 255), 6)
        
        if self.inv_bev_matrix is not None:
            lane_overlay = cv2.warpPerspective(bev_colored, self.inv_bev_matrix,
                                               (original_image.shape[1], original_image.shape[0]))
            
            mask = (lane_overlay.sum(axis=2) > 0).astype(np.uint8)
            for c in range(3):
                original_image[:,:,c] = np.where(mask,
                    cv2.addWeighted(original_image[:,:,c], 0.6, lane_overlay[:,:,c], 0.4, 0),
                    original_image[:,:,c])
        
        mask_resized = cv2.resize(lane_mask, (200, 150))
        mask_colored_mini = cv2.applyColorMap(mask_resized, cv2.COLORMAP_RAINBOW)
        x_offset = original_image.shape[1] - 210
        y_offset = 130
        original_image[y_offset:y_offset + 150, x_offset:x_offset + 200] = mask_colored_mini
        cv2.putText(original_image, "BEV MASK", (x_offset, y_offset - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        if lane_center is not None:
            cv2.putText(original_image, f"🛣️ LANE CENTER: {int(lane_center)} | Conf: {self.lane_confidence:.1f}%",
                        (10, original_image.shape[0] - 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        else:
            cv2.putText(original_image, "🔍 SEARCHING FOR LANE... | Applying previous control",
                        (10, original_image.shape[0] - 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 100, 255), 2)

    def calculate_steering_control(self, error, mode):
        """Applies PID control with mode-specific gains."""
        if mode == "RUBBERCON":
            kp, ki, kd = 0.0035, 0.00008, 0.002
            max_steering = 0.5
            base_speed = 0.35
        elif mode == "LANE":
            kp, ki, kd = 0.0025, 0.00005, 0.0018
            max_steering = 0.35
            base_speed = 0.7 if abs(self.current_steering) < 0.1 else 0.4
        else:
            kp, ki, kd = 0.002, 0.0001, 0.001
            max_steering = 0.3
            base_speed = 0.2
        
        dt = 0.05
        self.integral_error += error * dt
        derivative_error = (error - self.prev_error) / dt
        
        integral_limit = 800
        if abs(self.integral_error) > integral_limit:
            self.integral_error = integral_limit * np.sign(self.integral_error)
        
        steering_output = -(kp * error + ki * self.integral_error + kd * derivative_error)
        
        speed_factor = max(0.5, 1.0 - abs(self.current_speed) * 0.3)
        max_steering *= speed_factor
        
        self.target_steering = max(-max_steering, min(max_steering, steering_output))
        self.prev_error = error
        
        steering_magnitude = abs(self.target_steering)
        if steering_magnitude > 0.3:
            speed_reduction = (steering_magnitude - 0.3) * 2
            base_speed *= max(0.4, 1.0 - speed_reduction)
        
        self.target_speed = base_speed

    def draw_lidar_overlay(self, image):
        """Draws a LIDAR status panel and a minimap on the image."""
        if self.lidar_data is None:
            return
        
        height, width = image.shape[:2]
        ranges = self.lidar_data.ranges
        total_points = len(ranges)
        
        if total_points == 0:
            return
        
        center = total_points // 2
        front_range = min(25, total_points // 12)
        front_ranges = ranges[center-front_range:center+front_range]
        valid_ranges = [r for r in front_ranges if 0.05 < r < 10.0]
        
        if valid_ranges:
            avg_distance = sum(valid_ranges) / len(valid_ranges)
            min_distance = min(valid_ranges)
            
            if min_distance < 0.15:
                color = (0, 0, 255)
                status = "⚠️ CRITICAL"
            elif min_distance < 0.3:
                color = (0, 255, 255)
                status = "⚠️ WARNING"
            else:
                color = (0, 255, 0)
                status = "✅ CLEAR"
            
            panel_width, panel_height = 280, 120
            panel_x, panel_y = width - panel_width - 10, height - panel_height - 10
            
            overlay = image.copy()
            cv2.rectangle(overlay, (panel_x, panel_y), (panel_x + panel_width, panel_y + panel_height), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)
            
            cv2.putText(image, "📡 LIDAR STATUS", (panel_x + 10, panel_y + 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(image, f"Avg Distance: {avg_distance:.2f}m", (panel_x + 10, panel_y + 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            cv2.putText(image, f"Min Distance: {min_distance:.2f}m", (panel_x + 10, panel_y + 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            cv2.putText(image, status, (panel_x + 10, panel_y + 95),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        self.draw_lidar_minimap(image, ranges)

    def draw_lidar_minimap(self, image, ranges):
        """Draws a mini-map of LIDAR points."""
        minimap_size = 120
        minimap_x = 20
        minimap_y = 320
        
        cv2.rectangle(image, (minimap_x, minimap_y),
                      (minimap_x + minimap_size, minimap_y + minimap_size), (50, 50, 50), -1)
        cv2.rectangle(image, (minimap_x, minimap_y),
                      (minimap_x + minimap_size, minimap_y + minimap_size), (255, 255, 255), 2)
        
        center_x = minimap_x + minimap_size // 2
        center_y = minimap_y + minimap_size // 2
        cv2.circle(image, (center_x, center_y), 3, (0, 255, 0), -1)
        
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
        
        cv2.putText(image, "LIDAR", (minimap_x, minimap_y - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    def lidar_callback(self, msg):
        """Processes incoming LIDAR data for obstacle detection and emergency stop."""
        self.lidar_data = msg
        if not msg.ranges:
            return
        
        total_points = len(msg.ranges)
        center = total_points // 2
        front_range = min(25, total_points // 12)
        
        front_ranges = msg.ranges[center - front_range:center + front_range]
        valid_ranges = [r for r in front_ranges if 0.05 < r < 10.0]
        
        if valid_ranges:
            min_distance = min(valid_ranges)
            
            if min_distance < 0.15:
                if self.current_mode != DriveMode.EMERGENCY_STOP:
                    self.current_mode = DriveMode.EMERGENCY_STOP
                    self.emergency_start_time = time.time()
                    self.get_logger().warn('🚨 EMERGENCY STOP - Obstacle detected!')

    def control_loop(self):
        """Main control loop that publishes vehicle commands."""
        cmd = Twist()
        
        # Low-pass filter for smooth control
        alpha_speed = 0.4
        alpha_steering = 0.6
        
        self.current_speed = alpha_speed * self.target_speed + (1 - alpha_speed) * self.current_speed
        self.current_steering = alpha_steering * self.target_steering + (1 - alpha_steering) * self.current_steering
        
        if self.current_mode == DriveMode.RUBBERCON_AVOIDANCE:
            if not hasattr(self, 'target_speed') or self.target_speed == 0:
                self.target_speed = 0.35
        elif self.current_mode == DriveMode.LANE_FOLLOWING:
            if not self.lane_detected:
                self.target_speed = 0.2
                self.lane_search_count += 1
                if self.lane_search_count > 40:
                    search_steering = 0.1 * math.sin(self.lane_search_count * 0.2)
                    self.target_steering = search_steering
                    if self.lane_search_count > 100:
                        self.lane_search_count = 0
            else:
                self.lane_search_count = 0
        elif self.current_mode == DriveMode.EMERGENCY_STOP:
            self.target_speed = 0.0
            self.target_steering = 0.0
            
            if hasattr(self, 'emergency_start_time'):
                if time.time() - self.emergency_start_time > 2.0:
                    if not self.rubbercon_passed:
                        self.current_mode = DriveMode.RUBBERCON_AVOIDANCE
                    else:
                        self.current_mode = DriveMode.LANE_FOLLOWING
                    delattr(self, 'emergency_start_time')
                    self.get_logger().info('🔄 Emergency cleared - resuming mission')
        
        cmd.linear.x = float(max(0.0, min(0.8, self.current_speed)))
        cmd.angular.z = float(max(-0.6, min(0.6, self.current_steering)))
        
        self.cmd_pub.publish(cmd)

    def get_processed_frame(self):
        """Returns the processed frame for web streaming."""
        with self.image_lock:
            if self.processed_frame is None:
                return None
            return self.processed_frame.copy()

    def get_stats(self):
        """Returns telemetry data for the web dashboard."""
        lidar_distance = "N/A"
        if self.lidar_data is not None and len(self.lidar_data.ranges) > 0:
            center = len(self.lidar_data.ranges) // 2
            front_range = min(20, len(self.lidar_data.ranges) // 10)
            front_ranges = self.lidar_data.ranges[center - front_range:center + front_range]
            valid_ranges = [r for r in front_ranges if 0.05 < r < 10.0]
            if valid_ranges:
                lidar_distance = f"{min(valid_ranges):.2f}"
        
        rubbercon_status = "🔍 SEARCHING"
        if self.rubbercon_passed:
            rubbercon_status = "✅ PASSED"
        elif getattr(self, 'rubbercon_avoidance_active', False):
            rubbercon_status = f"🚧 AVOIDING (Flag:{self.rubbercon_detection_flag})"
        elif self.detection_confidence > 50:
            rubbercon_status = "🎯 DETECTED"
        
        lane_status = "🔍 SEARCHING"
        if getattr(self, 'lane_detected', False):
            lane_status = f"✅ FOLLOWING (Conf:{self.lane_confidence:.1f}%)"
        elif hasattr(self, 'lane_confidence') and self.lane_confidence > 30:
            lane_status = "🎯 DETECTED"
        
        return {
            "current_mode": self.current_mode.value,
            "rubbercon_status": rubbercon_status,
            "lane_status": lane_status,
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
        print("\n🏁 Autoracer 2025 Contest Ended!")
    finally:
        if 'autoracer' in locals():
            autoracer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
