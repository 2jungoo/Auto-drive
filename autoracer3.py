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
from collections import deque
import json

# QoS Configuration
qos_profile = QoSProfile(depth=10)
qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

class DriveMode(Enum):
    """Mission states for the autonomous racing challenge"""
    TRAFFIC_LIGHT_WAIT = "TRAFFIC_LIGHT"
    RUBBERCON_NAVIGATION = "RUBBERCON"
    LANE_FOLLOWING = "LANE_FOLLOW"
    OBSTACLE_AVOIDANCE = "OBSTACLE_AVOID"
    EMERGENCY_STOP = "E_STOP"
    MISSION_COMPLETE = "COMPLETE"

class ControlParameters:
    """Centralized control parameters for different driving modes"""
    PARAMS = {
        DriveMode.TRAFFIC_LIGHT_WAIT: {
            'kp': 0.0, 'ki': 0.0, 'kd': 0.0,
            'max_speed': 0.0, 'max_steering': 0.0
        },
        DriveMode.RUBBERCON_NAVIGATION: {
            'kp': 0.0035, 'ki': 0.00008, 'kd': 0.002,
            'max_speed': 0.35, 'max_steering': 0.5
        },
        DriveMode.LANE_FOLLOWING: {
            'kp': 0.0025, 'ki': 0.00005, 'kd': 0.0018,
            'max_speed': 0.5, 'max_steering': 0.35
        },
        DriveMode.OBSTACLE_AVOIDANCE: {
            'kp': 0.003, 'ki': 0.0001, 'kd': 0.002,
            'max_speed': 0.3, 'max_steering': 0.4
        },
        DriveMode.EMERGENCY_STOP: {
            'kp': 0.0, 'ki': 0.0, 'kd': 0.0,
            'max_speed': 0.0, 'max_steering': 0.0
        }
    }

class WebViewer(BaseHTTPRequestHandler):
    """Web interface for real-time monitoring and debugging"""
    
    def __init__(self, autoracer_node, *args, **kwargs):
        self.autoracer = autoracer_node
        super().__init__(*args, **kwargs)
    
    def log_message(self, format, *args):
        """Suppress default HTTP logging"""
        pass
    
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(self._generate_html().encode())
            
        elif self.path == '/stream.mjpg':
            self._handle_stream()
            
        elif self.path == '/stats':
            self._handle_stats()
            
        else:
            self.send_error(404)
    
    def _generate_html(self):
        """Generate the monitoring dashboard HTML"""
        return """
        <!DOCTYPE html>
        <html>
        <head>
            <title>Autoracer 2025 - Mission Control</title>
            <style>
                * { margin: 0; padding: 0; box-sizing: border-box; }
                body { 
                    background: linear-gradient(135deg, #0f0f1e, #1a1a2e); 
                    color: #fff; 
                    font-family: 'Inter', system-ui, -apple-system, sans-serif; 
                    padding: 20px;
                }
                .header {
                    text-align: center;
                    padding: 20px 0;
                    background: rgba(255,255,255,0.05);
                    border-radius: 12px;
                    margin-bottom: 20px;
                }
                h1 { 
                    font-size: 2.5em;
                    background: linear-gradient(90deg, #00ff88, #00ccff);
                    -webkit-background-clip: text;
                    -webkit-text-fill-color: transparent;
                }
                .container { 
                    display: grid; 
                    grid-template-columns: 2fr 1fr;
                    gap: 20px; 
                    max-width: 1600px; 
                    margin: 0 auto; 
                }
                .panel { 
                    background: rgba(255,255,255,0.08); 
                    border-radius: 16px; 
                    padding: 24px; 
                    backdrop-filter: blur(20px);
                    border: 1px solid rgba(255,255,255,0.1);
                }
                .video-container {
                    position: relative;
                    background: #000;
                    border-radius: 12px;
                    overflow: hidden;
                }
                .video-container img {
                    width: 100%;
                    height: auto;
                    display: block;
                }
                .mission-indicator {
                    display: grid;
                    grid-template-columns: repeat(4, 1fr);
                    gap: 10px;
                    margin: 20px 0;
                }
                .mission-card {
                    background: rgba(255,255,255,0.05);
                    padding: 15px;
                    border-radius: 8px;
                    text-align: center;
                    border: 2px solid rgba(255,255,255,0.1);
                    transition: all 0.3s;
                }
                .mission-card.active {
                    background: rgba(0,255,136,0.2);
                    border-color: #00ff88;
                    transform: scale(1.05);
                }
                .mission-card.complete {
                    background: rgba(0,200,255,0.2);
                    border-color: #00ccff;
                }
                .metrics {
                    display: grid;
                    gap: 15px;
                }
                .metric-group {
                    background: rgba(255,255,255,0.05);
                    padding: 15px;
                    border-radius: 8px;
                }
                .metric-row {
                    display: flex;
                    justify-content: space-between;
                    padding: 8px 0;
                    border-bottom: 1px solid rgba(255,255,255,0.1);
                }
                .metric-row:last-child {
                    border-bottom: none;
                }
                .metric-label {
                    color: #888;
                    font-size: 0.9em;
                }
                .metric-value {
                    font-weight: 600;
                    color: #00ff88;
                    font-family: 'Courier New', monospace;
                }
                .alert-box {
                    background: rgba(255,50,50,0.1);
                    border: 1px solid rgba(255,50,50,0.3);
                    border-radius: 8px;
                    padding: 15px;
                    margin-top: 20px;
                    min-height: 60px;
                }
                .status-indicator {
                    display: inline-block;
                    width: 12px;
                    height: 12px;
                    border-radius: 50%;
                    margin-right: 8px;
                    animation: pulse 2s infinite;
                }
                .status-green { background: #00ff88; }
                .status-yellow { background: #ffaa00; }
                .status-red { background: #ff3366; }
                @keyframes pulse {
                    0%, 100% { opacity: 1; }
                    50% { opacity: 0.5; }
                }
                .progress-bar {
                    width: 100%;
                    height: 30px;
                    background: rgba(255,255,255,0.1);
                    border-radius: 15px;
                    overflow: hidden;
                    margin: 20px 0;
                }
                .progress-fill {
                    height: 100%;
                    background: linear-gradient(90deg, #00ff88, #00ccff);
                    transition: width 0.5s ease;
                    display: flex;
                    align-items: center;
                    justify-content: center;
                    font-weight: bold;
                }
            </style>
        </head>
        <body>
            <div class="header">
                <h1>AUTORACER 2025 MISSION CONTROL</h1>
                <p style="color: #888; margin-top: 10px;">Real-time Autonomous Vehicle Monitoring System</p>
            </div>
            
            <div class="mission-indicator">
                <div class="mission-card" id="mission1">
                    <div style="font-size: 1.5em;">🚦</div>
                    <div>Mission 1</div>
                    <small>Traffic Light</small>
                </div>
                <div class="mission-card" id="mission2">
                    <div style="font-size: 1.5em;">🔶</div>
                    <div>Mission 2</div>
                    <small>Rubbercons</small>
                </div>
                <div class="mission-card" id="mission3">
                    <div style="font-size: 1.5em;">🛣️</div>
                    <div>Mission 3</div>
                    <small>Lane Following</small>
                </div>
                <div class="mission-card" id="mission4">
                    <div style="font-size: 1.5em;">🚗</div>
                    <div>Mission 4</div>
                    <small>Obstacle Avoid</small>
                </div>
            </div>
            
            <div class="container">
                <div class="panel">
                    <h3 style="margin-bottom: 15px;">📹 Live Camera Feed</h3>
                    <div class="video-container">
                        <img src="/stream.mjpg" alt="Camera Stream">
                    </div>
                    <div class="progress-bar">
                        <div id="progress" class="progress-fill" style="width: 0%;">
                            <span id="progress-text">0%</span>
                        </div>
                    </div>
                </div>
                
                <div class="panel">
                    <h3 style="margin-bottom: 15pangularControllerServiceImplpx;">📊 System Metrics</h3>
                    
                    <div class="metric-group">
                        <h4 style="margin-bottom: 10px; color: #00ccff;">Vehicle Status</h4>
                        <div class="metric-row">
                            <span class="metric-label">Mode</span>
                            <span id="mode" class="metric-value">INIT</span>
                        </div>
                        <div class="metric-row">
                            <span class="metric-label">Speed</span>
                            <span id="speed" class="metric-value">0.00 m/s</span>
                        </div>
                        <div class="metric-row">
                            <span class="metric-label">Steering</span>
                            <span id="steering" class="metric-value">0.0°</span>
                        </div>
                        <div class="metric-row">
                            <span class="metric-label">Runtime</span>
                            <span id="runtime" class="metric-value">00:00</span>
                        </div>
                    </div>
                    
                    <div class="metric-group">
                        <h4 style="margin-bottom: 10px; color: #00ccff;">Detection Status</h4>
                        <div class="metric-row">
                            <span class="metric-label">Traffic Light</span>
                            <span id="traffic" class="metric-value">SCANNING</span>
                        </div>
                        <div class="metric-row">
                            <span class="metric-label">Rubbercons</span>
                            <span id="rubbercon" class="metric-value">NONE</span>
                        </div>
                        <div class="metric-row">
                            <span class="metric-label">Lane</span>
                            <span id="lane" class="metric-value">NONE</span>
                        </div>
                        <div class="metric-row">
                            <span class="metric-label">Obstacle</span>
                            <span id="obstacle" class="metric-value">CLEAR</span>
                        </div>
                    </div>
                    
                    <div class="metric-group">
                        <h4 style="margin-bottom: 10px; color: #00ccff;">Sensors</h4>
                        <div class="metric-row">
                            <span class="metric-label">Camera FPS</span>
                            <span id="fps" class="metric-value">0</span>
                        </div>
                        <div class="metric-row">
                            <span class="metric-label">LiDAR Range</span>
                            <span id="lidar" class="metric-value">N/A</span>
                        </div>
                        <div class="metric-row">
                            <span class="metric-label">Confidence</span>
                            <span id="confidence" class="metric-value">0%</span>
                        </div>
                    </div>
                    
                    <div class="alert-box">
                        <h4 style="margin-bottom: 10px;">⚠️ System Alerts</h4>
                        <div id="alerts">
                            <span class="status-indicator status-green"></span>
                            <span>All systems operational</span>
                        </div>
                    </div>
                </div>
            </div>
            
            <script>
                let startTime = Date.now();
                
                function updateDashboard() {
                    fetch('/stats')
                        .then(r => r.json())
                        .then(data => {
                            // Update metrics
                            document.getElementById('mode').textContent = data.mode;
                            document.getElementById('speed').textContent = data.speed + ' m/s';
                            document.getElementById('steering').textContent = data.steering + '°';
                            document.getElementById('fps').textContent = data.fps;
                            document.getElementById('lidar').textContent = data.lidar;
                            document.getElementById('confidence').textContent = data.confidence + '%';
                            
                            // Update detection status
                            document.getElementById('traffic').textContent = data.traffic_status;
                            document.getElementById('rubbercon').textContent = data.rubbercon_status;
                            document.getElementById('lane').textContent = data.lane_status;
                            document.getElementById('obstacle').textContent = data.obstacle_status;
                            
                            // Update runtime
                            let elapsed = Math.floor((Date.now() - startTime) / 1000);
                            let mins = Math.floor(elapsed / 60);
                            let secs = elapsed % 60;
                            document.getElementById('runtime').textContent = 
                                `${mins.toString().padStart(2,'0')}:${secs.toString().padStart(2,'0')}`;
                            
                            // Update progress
                            document.getElementById('progress').style.width = data.progress + '%';
                            document.getElementById('progress-text').textContent = data.progress + '%';
                            
                            // Update mission cards
                            for (let i = 1; i <= 4; i++) {
                                let card = document.getElementById('mission' + i);
                                card.className = 'mission-card';
                                if (i <= data.mission_stage) {
                                    card.classList.add('complete');
                                }
                                if (i === data.current_mission) {
                                    card.classList.add('active');
                                }
                            }
                            
                            // Update alerts
                            let alertsDiv = document.getElementById('alerts');
                            if (data.alerts && data.alerts.length > 0) {
                                alertsDiv.innerHTML = data.alerts.map(alert => {
                                    let color = alert.level === 'error' ? 'red' : 
                                               alert.level === 'warning' ? 'yellow' : 'green';
                                    return `<div><span class="status-indicator status-${color}"></span>${alert.message}</div>`;
                                }).join('');
                            } else {
                                alertsDiv.innerHTML = '<span class="status-indicator status-green"></span>All systems operational';
                            }
                        })
                        .catch(e => console.error('Stats error:', e));
                }
                
                setInterval(updateDashboard, 100);
            </script>
        </body>
        </html>
        """
    
    def _handle_stream(self):
        """Handle MJPEG stream"""
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        
        try:
            while True:
                frame = self.autoracer.get_processed_frame()
                if frame is not None:
                    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    self.wfile.write(b'--frame\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', str(len(buffer)))
                    self.end_headers()
                    self.wfile.write(buffer)
                    self.wfile.write(b'\r\n')
                time.sleep(0.033)
        except:
            pass
    
    def _handle_stats(self):
        """Handle stats request"""
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        stats = self.autoracer.get_stats()
        self.wfile.write(json.dumps(stats).encode())

class Autoracer(Node):
    """Main autonomous racing node"""
    
    def __init__(self):
        super().__init__('autoracer_2025')
        
        # Initialize state variables
        self._init_state_variables()
        
        # Setup computer vision
        self._setup_vision()
        
        # Setup ROS2 interfaces
        self._setup_ros2()
        
        # Start web server
        self._start_web_server()
        
        self.get_logger().info('🏁 Autoracer 2025 initialized - Ready for missions!')
    
    def _init_state_variables(self):
        """Initialize all state variables"""
        # Image processing
        self.current_image = None
        self.processed_frame = None
        self.image_lock = threading.Lock()
        
        # Mission state
        self.current_mode = DriveMode.TRAFFIC_LIGHT_WAIT
        self.mission_stage = 0
        self.mission_complete = False
        
        # Mission flags
        self.traffic_light_passed = False
        self.rubbercon_passed = False
        self.lane_following_active = False
        self.obstacle_avoided = False
        
        # Control variables
        self.current_speed = 0.0
        self.current_steering = 0.0
        self.target_speed = 0.0
        self.target_steering = 0.0
        
        # PID control
        self.pid_error = 0.0
        self.pid_integral = 0.0
        self.pid_derivative = 0.0
        self.prev_error = 0.0
        
        # Detection states
        self.traffic_light_state = None
        self.green_detection_count = 0
        self.rubbercon_positions = []
        self.lane_center = None
        self.obstacle_position = None
        
        # Performance metrics
        self.frame_count = 0
        self.fps = 0
        self.last_fps_time = time.time()
        self.confidence_score = 0.0
        
        # Sensor data
        self.lidar_data = None
        self.min_lidar_distance = float('inf')
        
        # Alerts system
        self.alerts = []
        
    def _setup_vision(self):
        """Setup computer vision parameters"""
        # BEV transformation matrix
        src_pts = np.float32([
            [100, 480],   # bottom-left
            [540, 480],   # bottom-right  
            [260, 300],   # top-left
            [380, 300]    # top-right
        ])
        
        dst_pts = np.float32([
            [160, 480],
            [480, 480],
            [160, 0],
            [480, 0]
        ])
        
        self.bev_matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
        self.inv_bev_matrix = cv2.getPerspectiveTransform(dst_pts, src_pts)
        
        # Color ranges for detection
        self.color_ranges = {
            'green': {
                'lower': np.array([40, 100, 100]),
                'upper': np.array([80, 255, 255])
            },
            'orange': {
                'lower': np.array([5, 100, 100]),
                'upper': np.array([25, 255, 255])
            },
            'white': {
                'lower': np.array([0, 0, 200]),
                'upper': np.array([180, 30, 255])
            },
            'yellow': {
                'lower': np.array([20, 100, 100]),
                'upper': np.array([35, 255, 255])
            }
        }
        
    def _setup_ros2(self):
        """Setup ROS2 publishers and subscribers"""
        # Subscribers
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
        
        # Publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Control timer (20Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
    def _start_web_server(self):
        """Start web monitoring server"""
        def create_handler(*args, **kwargs):
            return WebViewer(self, *args, **kwargs)
        
        def run_server():
            server = HTTPServer(('0.0.0.0', 8080), create_handler)
            self.get_logger().info(f'📡 Web interface: http://{self._get_ip()}:8080')
            server.serve_forever()
        
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()
    
    def _get_ip(self):
        """Get local IP address"""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return "localhost"
    
    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Decode image
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if image is not None:
                with self.image_lock:
                    self.current_image = image.copy()
                
                # Process based on current mission
                self.process_mission(image)
                
                # Calculate FPS
                self.frame_count += 1
                current_time = time.time()
                if current_time - self.last_fps_time > 1.0:
                    self.fps = self.frame_count
                    self.frame_count = 0
                    self.last_fps_time = current_time
                    
        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')
    
    def process_mission(self, image):
        """Process image based on current mission"""
        processed = image.copy()
        
        # Add debug overlay
        self._add_debug_overlay(processed)
        
        # Mission-specific processing
        if self.current_mode == DriveMode.TRAFFIC_LIGHT_WAIT:
            self._process_traffic_light(processed)
            
        elif self.current_mode == DriveMode.RUBBERCON_NAVIGATION:
            self._process_rubbercon(processed)
            
        elif self.current_mode == DriveMode.LANE_FOLLOWING:
            self._process_lane_following(processed)
            # Check for obstacles during lane following
            if self._detect_obstacle_vehicle(processed):
                self.current_mode = DriveMode.OBSTACLE_AVOIDANCE
                self.get_logger().info('🚗 Obstacle detected - initiating avoidance')
                
        elif self.current_mode == DriveMode.OBSTACLE_AVOIDANCE:
            self._process_obstacle_avoidance(processed)
        
        # Store processed frame
        with self.image_lock:
            self.processed_frame = processed
    
    def _add_debug_overlay(self, image):
        """Add debug information overlay"""
        h, w = image.shape[:2]
        
        # Semi-transparent header
        overlay = image.copy()
        cv2.rectangle(overlay, (0, 0), (w, 100), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)
        
        # Mission info
        cv2.putText(image, f'MODE: {self.current_mode.value}', 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(image, f'FPS: {self.fps} | SPEED: {self.current_speed:.2f} | STEER: {math.degrees(self.current_steering):.1f}°', 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(image, f'CONFIDENCE: {self.confidence_score:.1f}% | LIDAR: {self.min_lidar_distance:.2f}m', 
                   (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    
def _process_traffic_light(self, image):
    """Mission 1: Traffic light detection - Fixed for top-mounted traffic light"""
    h, w = image.shape[:2]
    
    # Define ROI for traffic light at the top of the frame
    roi_y_start = 0
    roi_y_end = h // 3  # Top third of the image
    roi_x_start = w // 4  # Center portion horizontally
    roi_x_end = w * 3 // 4
    
    # Extract ROI
    roi = image[roi_y_start:roi_y_end, roi_x_start:roi_x_end]
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    # Detect green light with adjusted parameters for better detection
    green_mask = cv2.inRange(hsv_roi, 
                            np.array([40, 50, 50]),   # Lower threshold - more permissive
                            np.array([80, 255, 255])) # Upper threshold
    
    # Noise reduction
    kernel = np.ones((3, 3), np.uint8)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
    
    # Find contours to identify circular green light
    contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    green_detected = False
    largest_green_area = 0
    
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 100:  # Minimum area for green light
            # Check if contour is roughly circular
            perimeter = cv2.arcLength(contour, True)
            if perimeter > 0:
                circularity = 4 * np.pi * area / (perimeter * perimeter)
                
                if circularity > 0.3:  # Reasonably circular
                    x, y, w_box, h_box = cv2.boundingRect(contour)
                    aspect_ratio = float(w_box) / h_box if h_box > 0 else 0
                    
                    # Traffic light should be roughly square/circular
                    if 0.5 < aspect_ratio < 2.0:
                        green_detected = True
                        largest_green_area = max(largest_green_area, area)
                        
                        # Draw detection on original image
                        actual_x = roi_x_start + x
                        actual_y = roi_y_start + y
                        cv2.rectangle(image, (actual_x, actual_y), 
                                    (actual_x + w_box, actual_y + h_box), 
                                    (0, 255, 0), 3)
                        cv2.putText(image, "GREEN LIGHT", 
                                  (actual_x, actual_y - 10), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # Draw ROI rectangle for debugging
    cv2.rectangle(image, (roi_x_start, roi_y_start), 
                 (roi_x_end, roi_y_end), (255, 255, 0), 2)
    cv2.putText(image, "TRAFFIC LIGHT ROI", 
               (roi_x_start, roi_y_start - 10), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    
    # Visualization of detection mask (smaller display)
    roi_display = cv2.resize(green_mask, (120, 90))
    roi_colored = cv2.applyColorMap(roi_display, cv2.COLORMAP_JET)
    image[10:100, w-130:w-10] = roi_colored
    
    # Detection logic with improved stability
    if green_detected and largest_green_area > 200:  # Minimum significant area
        self.green_detection_count += 1
        self.traffic_light_state = "GREEN"
        self.confidence_score = min(100, (largest_green_area / 10))  # Scale confidence
        
        cv2.putText(image, f"GREEN DETECTED! Count: {self.green_detection_count}", 
                   (10, h-60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(image, f"Area: {int(largest_green_area)} | Confidence: {self.confidence_score:.1f}%", 
                   (10, h-30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Start after consistent detection (reduced threshold for faster response)
        if self.green_detection_count > 8:  # Reduced from 15 to 8
            self.traffic_light_passed = True
            self.current_mode = DriveMode.RUBBERCON_NAVIGATION
            self.mission_stage = 1
            self.get_logger().info('✅ Mission 1 complete - Green light detected!')
    else:
        # Decay counter more slowly to avoid flickering
        self.green_detection_count = max(0, self.green_detection_count - 1)
        self.traffic_light_state = "WAITING"
        self.confidence_score *= 0.9  # Gradual confidence decay
        
        cv2.putText(image, "WAITING FOR GREEN LIGHT", 
                   (10, h-60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.putText(image, f"Scanning... Count: {self.green_detection_count}", 
                   (10, h-30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
    def _process_rubbercon(self, image):
        """Mission 2: Rubbercon navigation"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, w = image.shape[:2]
        
        # Detect orange rubbercons
        orange_mask = cv2.inRange(hsv, self.color_ranges['orange']['lower'],
                                 self.color_ranges['orange']['upper'])
        
        # Focus on lower region
        roi_mask = np.zeros_like(orange_mask)
        roi_mask[h//2:, :] = 255
        orange_mask = cv2.bitwise_and(orange_mask, roi_mask)
        
        # Noise reduction
        kernel = np.ones((5, 5), np.uint8)
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, kernel)
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter and track rubbercons
        rubbercons = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 300:  # Minimum area threshold
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w) / h if h > 0 else 0
                
                # Filter by shape
                if 0.3 < aspect_ratio < 2.0:
                    center_x = x + w // 2
                    center_y = y + h // 2
                    rubbercons.append({
                        'x': center_x,
                        'y': center_y,
                        'area': area,
                        'bbox': (x, y, w, h)
                    })
                    
                    # Draw detection
                    cv2.rectangle(image, (x, y), (x+w, y+h), (0, 165, 255), 2)
                    cv2.circle(image, (center_x, center_y), 5, (0, 255, 255), -1)
        
        # Navigation logic
        if len(rubbercons) >= 2:
            # Find path between cones
            rubbercons.sort(key=lambda c: c['x'])
            
            # Calculate midpoint
            if len(rubbercons) >= 2:
                left_cone = rubbercons[0]
                right_cone = rubbercons[-1]
                
                target_x = (left_cone['x'] + right_cone['x']) // 2
                target_y = (left_cone['y'] + right_cone['y']) // 2
                
                # Draw navigation path
                cv2.line(image, (left_cone['x'], left_cone['y']),
                        (right_cone['x'], right_cone['y']), (255, 255, 0), 2)
                cv2.circle(image, (target_x, target_y), 10, (255, 0, 255), -1)
                
                # Calculate steering
                center_error = target_x - w // 2
                self._calculate_pid_control(center_error, DriveMode.RUBBERCON_NAVIGATION)
                
                self.confidence_score = min(100, len(rubbercons) * 20)
        else:
            # No cones detected - check if passed
            if hasattr(self, 'no_cone_frames'):
                self.no_cone_frames += 1
            else:
                self.no_cone_frames = 0
            
            if self.no_cone_frames > 30:
                self.rubbercon_passed = True
                self.current_mode = DriveMode.LANE_FOLLOWING
                self.mission_stage = 2
                self.get_logger().info('✅ Mission 2 complete - Rubbercon section passed!')
        
        # Status display
        cv2.putText(image, f"RUBBERCONS: {len(rubbercons)} detected", 
                   (10, h-30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
    
    def _process_lane_following(self, image):
        """Mission 3: Lane following with white and yellow line detection"""
        h, w = image.shape[:2]
        
        # Apply BEV transformation
        bev_image = cv2.warpPerspective(image, self.bev_matrix, (w, h))
        
        # Detect lanes
        lane_mask = self._detect_lanes(bev_image)
        
        # Find lane center using sliding window
        left_lane, right_lane, lane_center = self._sliding_window_search(lane_mask)
        
        # Draw lane overlay
        self._draw_lane_overlay(image, bev_image, left_lane, right_lane, lane_center)
        
        # Control based on lane center
        if lane_center is not None:
            center_error = lane_center - w // 2
            self._calculate_pid_control(center_error, DriveMode.LANE_FOLLOWING)
            self.lane_center = lane_center
            self.confidence_score = 80
            
            cv2.putText(image, f"LANE CENTER: {int(lane_center)}", 
                       (10, h-30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            # Use previous steering if lane not found
            if hasattr(self, 'prev_error'):
                self._calculate_pid_control(self.prev_error * 0.7, DriveMode.LANE_FOLLOWING)
            
            cv2.putText(image, "SEARCHING FOR LANE", 
                       (10, h-30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 100, 255), 2)
    
    def _detect_lanes(self, bev_image):
        """Detect white and yellow lane lines"""
        hsv = cv2.cvtColor(bev_image, cv2.COLOR_BGR2HSV)
        
        # White lanes
        white_mask = cv2.inRange(hsv, self.color_ranges['white']['lower'],
                                self.color_ranges['white']['upper'])
        
        # Yellow lanes
        yellow_mask = cv2.inRange(hsv, self.color_ranges['yellow']['lower'],
                                 self.color_ranges['yellow']['upper'])
        
        # Combine masks
        lane_mask = cv2.bitwise_or(white_mask, yellow_mask)
        
        # Also use adaptive thresholding for better detection
        gray = cv2.cvtColor(bev_image, cv2.COLOR_BGR2GRAY)
        _, bright = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
        lane_mask = cv2.bitwise_or(lane_mask, bright)
        
        # Noise reduction
        kernel = np.ones((3, 3), np.uint8)
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_OPEN, kernel)
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, kernel)
        
        return lane_mask
    
    def _sliding_window_search(self, binary_image):
        """Sliding window search for lane detection"""
        h, w = binary_image.shape
        
        # Histogram for base points
        histogram = np.sum(binary_image[h*2//3:, :], axis=0)
        midpoint = w // 2
        
        # Find peaks
        left_base = np.argmax(histogram[:midpoint])
        right_base = np.argmax(histogram[midpoint:]) + midpoint
        
        # Sliding window parameters
        nwindows = 10
        window_height = h // nwindows
        margin = 50
        minpix = 50
        
        # Current positions
        leftx_current = left_base
        rightx_current = right_base
        
        # Collect lane pixels
        left_lane_inds = []
        right_lane_inds = []
        
        for window in range(nwindows):
            # Window boundaries
            win_y_low = h - (window + 1) * window_height
            win_y_high = h - window * window_height
            
            win_xleft_low = max(0, leftx_current - margin)
            win_xleft_high = min(w, leftx_current + margin)
            win_xright_low = max(0, rightx_current - margin)
            win_xright_high = min(w, rightx_current + margin)
            
            # Find nonzero pixels
            nonzero = binary_image[win_y_low:win_y_high, :].nonzero()
            nonzeroy = np.array(nonzero[0]) + win_y_low
            nonzerox = np.array(nonzero[1])
            
            # Identify lane pixels
            good_left = ((nonzerox >= win_xleft_low) & 
                        (nonzerox < win_xleft_high)).nonzero()[0]
            good_right = ((nonzerox >= win_xright_low) & 
                         (nonzerox < win_xright_high)).nonzero()[0]
            
            if len(good_left) > 0:
                left_lane_inds.append((nonzeroy[good_left], nonzerox[good_left]))
                if len(good_left) > minpix:
                    leftx_current = int(np.mean(nonzerox[good_left]))
            
            if len(good_right) > 0:
                right_lane_inds.append((nonzeroy[good_right], nonzerox[good_right]))
                if len(good_right) > minpix:
                    rightx_current = int(np.mean(nonzerox[good_right]))
        
        # Fit polynomials
        left_fit = None
        right_fit = None
        lane_center = None
        
        if len(left_lane_inds) > 3:
            lefty = np.concatenate([pts[0] for pts in left_lane_inds])
            leftx = np.concatenate([pts[1] for pts in left_lane_inds])
            if len(leftx) > 100:
                left_fit = np.polyfit(lefty, leftx, 2)
        
        if len(right_lane_inds) > 3:
            righty = np.concatenate([pts[0] for pts in right_lane_inds])
            rightx = np.concatenate([pts[1] for pts in right_lane_inds])
            if len(rightx) > 100:
                right_fit = np.polyfit(righty, rightx, 2)
        
        # Calculate lane center
        if left_fit is not None and right_fit is not None:
            y_eval = h * 3 // 4
            left_x = left_fit[0]*y_eval**2 + left_fit[1]*y_eval + left_fit[2]
            right_x = right_fit[0]*y_eval**2 + right_fit[1]*y_eval + right_fit[2]
            lane_center = (left_x + right_x) / 2
        elif left_fit is not None:
            y_eval = h * 3 // 4
            left_x = left_fit[0]*y_eval**2 + left_fit[1]*y_eval + left_fit[2]
            lane_center = left_x + 160  # Assume lane width
        elif right_fit is not None:
            y_eval = h * 3 // 4
            right_x = right_fit[0]*y_eval**2 + right_fit[1]*y_eval + right_fit[2]
            lane_center = right_x - 160
        
        return left_fit, right_fit, lane_center
    
    def _draw_lane_overlay(self, original, bev_image, left_lane, right_lane, center):
        """Draw lane detection overlay"""
        h, w = bev_image.shape[:2]
        overlay = np.zeros_like(bev_image)
        
        # Draw lane lines
        ploty = np.linspace(0, h-1, h)
        
        if left_lane is not None:
            left_fitx = left_lane[0]*ploty**2 + left_lane[1]*ploty + left_lane[2]
            pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
            cv2.polylines(overlay, np.int32([pts_left]), False, (255, 0, 0), 5)
        
        if right_lane is not None:
            right_fitx = right_lane[0]*ploty**2 + right_lane[1]*ploty + right_lane[2]
            pts_right = np.array([np.transpose(np.vstack([right_fitx, ploty]))])
            cv2.polylines(overlay, np.int32([pts_right]), False, (0, 0, 255), 5)
        
        # Fill lane area
        if left_lane is not None and right_lane is not None:
            left_fitx = left_lane[0]*ploty**2 + left_lane[1]*ploty + left_lane[2]
            right_fitx = right_lane[0]*ploty**2 + right_lane[1]*ploty + right_lane[2]
            
            pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
            pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
            pts = np.hstack((pts_left, pts_right))
            cv2.fillPoly(overlay, np.int32([pts]), (0, 100, 0))
        
        # Draw center line
        if center is not None:
            cv2.line(overlay, (int(center), h//2), (int(center), h), (0, 255, 255), 3)
        
        # Inverse transform back to original perspective
        warped_back = cv2.warpPerspective(overlay, self.inv_bev_matrix, 
                                         (original.shape[1], original.shape[0]))
        result = cv2.addWeighted(original, 1, warped_back, 0.3, 0)
        
        original[:] = result
    
    def _detect_obstacle_vehicle(self, image):
        """Mission 4: Detect obstacle vehicles"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, w = image.shape[:2]
        
        # Detect various car colors (simplified)
        # Dark colors (black/gray cars)
        lower_dark = np.array([0, 0, 0])
        upper_dark = np.array([180, 255, 100])
        dark_mask = cv2.inRange(hsv, lower_dark, upper_dark)
        
        # Bright colors (white/silver cars)
        lower_bright = np.array([0, 0, 180])
        upper_bright = np.array([180, 30, 255])
        bright_mask = cv2.inRange(hsv, lower_bright, upper_bright)
        
        # Combine masks
        car_mask = cv2.bitwise_or(dark_mask, bright_mask)
        
        # ROI - focus on road ahead
        roi_mask = np.zeros_like(car_mask)
        roi_mask[h//3:h*2//3, w//4:w*3//4] = 255
        car_mask = cv2.bitwise_and(car_mask, roi_mask)
        
        # Find contours
        contours, _ = cv2.findContours(car_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000:  # Significant size
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w) / h if h > 0 else 0
                
                # Car-like aspect ratio
                if 1.0 < aspect_ratio < 3.0:
                    center_x = x + w // 2
                    
                    # Determine position
                    if center_x < image.shape[1] // 3:
                        self.obstacle_position = "LEFT"
                    elif center_x > image.shape[1] * 2 // 3:
                        self.obstacle_position = "RIGHT"
                    else:
                        self.obstacle_position = "CENTER"
                    
                    # Draw detection
                    cv2.rectangle(image, (x, y), (x+w, y+h), (0, 0, 255), 3)
                    cv2.putText(image, f"VEHICLE-{self.obstacle_position}", 
                               (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    
                    return True
        
        return False
    
    def _process_obstacle_avoidance(self, image):
        """Mission 4: Obstacle avoidance"""
        h, w = image.shape[:2]
        
        # Continue detecting obstacle
        obstacle_present = self._detect_obstacle_vehicle(image)
        
        if obstacle_present and self.obstacle_position:
            # Avoidance strategy based on obstacle position
            if self.obstacle_position == "LEFT":
                self.target_steering = 0.3  # Turn right
            elif self.obstacle_position == "RIGHT":
                self.target_steering = -0.3  # Turn left
            else:  # CENTER
                self.target_steering = -0.4  # Default to left
            
            self.target_speed = 0.3  # Slow down
            
            cv2.putText(image, f"AVOIDING {self.obstacle_position} OBSTACLE", 
                       (10, h-30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            # No obstacle detected - return to lane
            if not hasattr(self, 'avoidance_timer'):
                self.avoidance_timer = 0
            
            self.avoidance_timer += 1
            
            if self.avoidance_timer > 40:  # After 2 seconds
                self.obstacle_avoided = True
                self.current_mode = DriveMode.LANE_FOLLOWING
                self.mission_stage = 3
                self.avoidance_timer = 0
                self.get_logger().info('✅ Mission 4 complete - Obstacle avoided!')
                
                # Check if all missions complete
                if self.traffic_light_passed and self.rubbercon_passed and self.obstacle_avoided:
                    self.mission_complete = True
                    self.current_mode = DriveMode.MISSION_COMPLETE
                    self.get_logger().info('🏆 ALL MISSIONS COMPLETE!')
    
    def _calculate_pid_control(self, error, mode):
        """Calculate PID control output"""
        params = ControlParameters.PARAMS[mode]
        
        # PID calculation
        dt = 0.05
        self.pid_integral += error * dt
        self.pid_integral = np.clip(self.pid_integral, -1000, 1000)  # Anti-windup
        
        derivative = (error - self.prev_error) / dt if hasattr(self, 'prev_error') else 0
        
        # Control output
        output = -(params['kp'] * error + 
                  params['ki'] * self.pid_integral + 
                  params['kd'] * derivative)
        
        # Apply limits
        self.target_steering = np.clip(output, -params['max_steering'], params['max_steering'])
        
        # Speed control based on steering
        if abs(self.target_steering) > 0.3:
            self.target_speed = params['max_speed'] * 0.5
        elif abs(self.target_steering) > 0.15:
            self.target_speed = params['max_speed'] * 0.7
        else:
            self.target_speed = params['max_speed']
        
        self.prev_error = error
    
    def lidar_callback(self, msg):
        """Process LiDAR data"""
        self.lidar_data = msg
        
        if len(msg.ranges) > 0:
            # Check front obstacles
            center = len(msg.ranges) // 2
            front_range = min(30, len(msg.ranges) // 10)
            
            front_ranges = msg.ranges[center-front_range:center+front_range]
            valid_ranges = [r for r in front_ranges if 0.1 < r < 10.0]
            
            if valid_ranges:
                self.min_lidar_distance = min(valid_ranges)
                
                # Emergency stop if too close
                if self.min_lidar_distance < 0.2:
                    if self.current_mode != DriveMode.EMERGENCY_STOP:
                        self.current_mode = DriveMode.EMERGENCY_STOP
                        self.get_logger().warn('⚠️ EMERGENCY STOP - Obstacle detected!')
    
    def control_loop(self):
        """Main control loop"""
        cmd = Twist()
        
        # Smooth control with low-pass filter
        alpha_speed = 0.3
        alpha_steer = 0.5
        
        self.current_speed = alpha_speed * self.target_speed + (1-alpha_speed) * self.current_speed
        self.current_steering = alpha_steer * self.target_steering + (1-alpha_steer) * self.current_steering
        
        # Mode-specific control
        if self.current_mode == DriveMode.TRAFFIC_LIGHT_WAIT:
            if not self.traffic_light_passed:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
        
        elif self.current_mode == DriveMode.EMERGENCY_STOP:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
            # Resume after 2 seconds
            if not hasattr(self, 'emergency_timer'):
                self.emergency_timer = 0
            self.emergency_timer += 1
            
            if self.emergency_timer > 40:
                self.current_mode = DriveMode.LANE_FOLLOWING
                self.emergency_timer = 0
        
        elif self.current_mode == DriveMode.MISSION_COMPLETE:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        
        else:
            cmd.linear.x = float(self.current_speed)
            cmd.angular.z = float(self.current_steering)
        
        # Safety limits
        cmd.linear.x = np.clip(cmd.linear.x, 0.0, 0.6)
        cmd.angular.z = np.clip(cmd.angular.z, -0.5, 0.5)
        
        # Publish command
        self.cmd_pub.publish(cmd)
    
    def get_processed_frame(self):
        """Get processed frame for streaming"""
        with self.image_lock:
            return self.processed_frame.copy() if self.processed_frame is not None else None
    
    def get_stats(self):
        """Get system statistics"""
        # Calculate progress
        progress = 0
        current_mission = 0
        
        if self.traffic_light_passed:
            progress += 25
            current_mission = 2
        if self.rubbercon_passed:
            progress += 25
            current_mission = 3
        if self.lane_following_active:
            progress += 25
            current_mission = 3
        if self.obstacle_avoided:
            progress += 25
            current_mission = 4
        
        if self.current_mode == DriveMode.TRAFFIC_LIGHT_WAIT:
            current_mission = 1
        
        # Determine alerts
        alerts = []
        if self.min_lidar_distance < 0.3:
            alerts.append({'level': 'warning', 'message': 'Close proximity detected'})
        if self.fps < 10:
            alerts.append({'level': 'warning', 'message': 'Low FPS'})
        if self.current_mode == DriveMode.EMERGENCY_STOP:
            alerts.append({'level': 'error', 'message': 'EMERGENCY STOP ACTIVE'})
        
        return {
            'mode': self.current_mode.value,
            'speed': f'{self.current_speed:.2f}',
            'steering': f'{math.degrees(self.current_steering):.1f}',
            'fps': self.fps,
            'lidar': f'{self.min_lidar_distance:.2f}m' if self.min_lidar_distance < float('inf') else 'N/A',
            'confidence': int(self.confidence_score),
            'progress': progress,
            'current_mission': current_mission,
            'mission_stage': self.mission_stage,
            'traffic_status': 'PASSED' if self.traffic_light_passed else self.traffic_light_state or 'WAITING',
            'rubbercon_status': 'PASSED' if self.rubbercon_passed else 'ACTIVE' if len(getattr(self, 'rubbercon_positions', [])) > 0 else 'NONE',
            'lane_status': 'TRACKING' if self.lane_center else 'SEARCHING',
            'obstacle_status': self.obstacle_position if self.obstacle_position else 'CLEAR',
            'alerts': alerts
        }

def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = Autoracer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🏁 Autoracer 2025 shutdown")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
