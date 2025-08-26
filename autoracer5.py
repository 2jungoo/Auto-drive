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
import tf_transformations

qos_profile = QoSProfile(depth=10)
qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

class DriveMode(Enum):
    TRAFFIC_LIGHT_WAIT = "TRAFFIC_LIGHT_WAIT"
    RUBBERCON_AVOIDANCE = "RUBBERCON_AVOID"
    LANE_FOLLOWING = "LANE_FOLLOW"
    OVERTAKING = "OVERTAKING"
    EMERGENCY_STOP = "EMERGENCY_STOP"

class PIDController:
    def __init__(self, kp, ki, kd):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.last_error = 0.0
        self.integral = 0.0

    def calculate(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.last_error = error
        return output

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
                    body { background: linear-gradient(135deg, #1e1e1e, #2d2d30);
                        color: #fff; font-family: 'Segoe UI', Arial; margin: 0; padding: 20px;
                    }
                    .container { display: flex;
                        gap: 20px; max-width: 1400px; margin: 0 auto;
                    }
                    .panel { background: rgba(255,255,255,0.1);
                        border-radius: 12px; padding: 20px; backdrop-filter: blur(10px);
                    }
                    .status-card { background: rgba(0,255,0,0.1);
                        border-left: 4px solid #00ff00; margin: 10px 0; padding: 15px; border-radius: 8px;
                    }
                    .metric { display: flex;
                        justify-content: space-between; margin: 8px 0;
                    }
                    .metric-value { font-weight: bold;
                        color: #00ff88;
                    }
                    h1 { text-align: center;
                        color: #00ff88; text-shadow: 0 0 20px #00ff88;
                    }
                    h3 { color: #00ccff;
                        margin-top: 0;
                    }
                    .progress-bar { width: 100%;
                        height: 20px; background: #333; border-radius: 10px; overflow: hidden; margin: 10px 0;
                    }
                    .progress-fill { height: 100%;
                        background: linear-gradient(90deg, #00ff00, #00ccff); transition: width 0.3s;
                    }
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
                            <div id="progress" class="progress-fill" style="width: 50%;"></div>
                        </div>
                        <div style="font-size: 14px;">
                            <p id="mission1" style="color: #ffaa00;">🔄 Mission 1: Rubbercon Avoidance</p>
                            <p id="mission2" style="color: #666;">⏳ Mission 2: Lane Following</p>
                        </div>
                        
                        <h3>⚠️ System Alerts</h3>
                        <div id="alerts" style="background: rgba(255,0,0,0.1);
                            border-radius: 8px; padding: 15px; min-height: 50px;">
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
                        if (data.lidar_distance < 0.3) alerts.push('⚠️ Obstacle Too Close');
                        if (data.camera_fps < 5) alerts.push('📹 Low Camera FPS');
                        if (data.steering_angle > 0.8) alerts.push('🎯 Sharp Turn');
                        document.getElementById('alerts').innerHTML = alerts.length > 0 ? alerts.join('<br>') : '<span style="color: #aaa;">System Normal</span>';
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
                        time.sleep(0.033) # ~30 FPS
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
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, qos_profile)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile)
        
        # 이미지 및 센서 데이터
        self.current_image = None
        self.processed_frame = None
        self.lidar_data = None
        self.image_lock = threading.Lock()
        
        # 미션 상태 관리
        self.current_mode = DriveMode.TRAFFIC_LIGHT_WAIT
        self.rubbercon_passed = False
        self.lane_following_started = False
        self.overtaking_active = False
        
        # 제어 변수
        self.current_speed = 0.0
        self.current_steering = 0.0
        self.target_speed = 0.0
        self.target_steering = 0.0
        self.overtake_target_steer = 0.0
        
        # PID 제어기 (차선 주행)
        self.pid_controller = PIDController(kp=0.5, ki=0.01, kd=0.01)
        self.last_time = time.time()
        
        # FPS 계산
        self.last_frame_time = time.time()
        self.camera_fps = 0
        
        # 웹 서버 스레드
        self.web_server_thread = threading.Thread(target=self.start_web_server)
        self.web_server_thread.daemon = True
        self.web_server_thread.start()
        
        # 메인 주행 로직 타이머
        self.create_timer(0.05, self.run_mission_control)
        
    def start_web_server(self):
        try:
            server = HTTPServer(('0.0.0.0', 8000), lambda *args, **kwargs: WebViewer(self, *args, **kwargs))
            self.get_logger().info('Starting web server on port 8000')
            server.serve_forever()
        except Exception as e:
            self.get_logger().error(f"Failed to start web server: {e}")

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.current_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        with self.image_lock:
            # FPS 계산
            current_time = time.time()
            self.camera_fps = 1.0 / (current_time - self.last_frame_time) if (current_time - self.last_frame_time) > 0 else 0
            self.last_frame_time = current_time
            
            # 주행 모드에 따라 이미지 처리
            if self.current_mode == DriveMode.TRAFFIC_LIGHT_WAIT:
                self.processed_frame = self.process_traffic_light()
            elif self.current_mode in [DriveMode.LANE_FOLLOWING, DriveMode.OVERTAKING]:
                self.processed_frame = self.process_lane_lines()
            else:
                self.processed_frame = self.current_image

    def lidar_callback(self, msg):
        self.lidar_data = msg

    def process_traffic_light(self):
        hsv = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2HSV)
        lower_green = np.array([35, 100, 100])
        upper_green = np.array([85, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # 초록색 영역이 충분히 넓으면 출발
        if np.sum(mask) > 100000:
            self.get_logger().info("Green light detected! Starting race.")
            self.current_mode = DriveMode.RUBBERCON_AVOIDANCE
            self.start_time = time.time()
            
        return cv2.bitwise_and(self.current_image, self.current_image, mask=mask)

    def process_lane_lines(self):
        # ROI 설정
        height, width, _ = self.current_image.shape
        roi_vertices = [
            (0, height),
            (width / 2 - 150, height / 2 + 50),
            (width / 2 + 150, height / 2 + 50),
            (width, height)
        ]
        roi_mask = np.zeros_like(self.current_image)
        cv2.fillPoly(roi_mask, [np.array(roi_vertices, np.int32)], (255, 255, 255))
        
        # 흑백 이미지 및 가우시안 블러
        gray = cv2.cvtColor(cv2.bitwise_and(self.current_image, roi_mask), cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Canny Edge Detection
        edges = cv2.Canny(blur, 50, 150)
        
        # Hough Line Transform
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, maxLineGap=20, minLineLength=20)
        
        left_lines = []
        right_lines = []
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1 + 1e-6)
                
                # 경사도를 기준으로 좌/우 차선 분류
                if abs(slope) > 0.5:
                    if slope < 0:
                        left_lines.append(line)
                    else:
                        right_lines.append(line)
                        
        # 좌/우 차선 평균 계산
        left_avg_line = self.calculate_avg_line(left_lines)
        right_avg_line = self.calculate_avg_line(right_lines)
        
        # 중앙점 계산
        center_x = width // 2
        lane_center_x = None
        
        if left_avg_line is not None and right_avg_line is not None:
            left_x1, _, left_x2, _ = left_avg_line[0]
            right_x1, _, right_x2, _ = right_avg_line[0]
            
            lane_center_x = (left_x2 + right_x2) // 2
        elif left_avg_line is not None:
            x1, _, x2, _ = left_avg_line[0]
            lane_center_x = x2 + 150 # 한쪽 차선만 보이면 가상 차선 생성
        elif right_avg_line is not None:
            x1, _, x2, _ = right_avg_line[0]
            lane_center_x = x2 - 150 # 한쪽 차선만 보이면 가상 차선 생성
            
        # 조향 제어 (PID)
        if lane_center_x is not None:
            error = center_x - lane_center_x
            dt = time.time() - self.last_time
            self.target_steering = self.pid_controller.calculate(error, dt)
            self.last_time = time.time()
            self.target_steering = np.clip(self.target_steering, -1.0, 1.0)
        
        # 시각화
        processed = self.current_image.copy()
        if left_avg_line is not None:
            x1, y1, x2, y2 = left_avg_line[0]
            cv2.line(processed, (x1, y1), (x2, y2), (0, 255, 0), 5)
        if right_avg_line is not None:
            x1, y1, x2, y2 = right_avg_line[0]
            cv2.line(processed, (x1, y1), (x2, y2), (0, 255, 0), 5)
        if lane_center_x is not None:
            cv2.circle(processed, (lane_center_x, height - 100), 5, (255, 0, 0), -1)
        
        return processed

    def calculate_avg_line(self, lines):
        if not lines:
            return None
        
        x_coords = []
        y_coords = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            x_coords.extend([x1, x2])
            y_coords.extend([y1, y2])
            
        poly = np.poly1d(np.polyfit(y_coords, x_coords, deg=1))
        
        min_y = min(y_coords)
        max_y = max(y_coords)
        
        start_x = int(poly(max_y))
        end_x = int(poly(min_y))
        
        return np.array([[[start_x, max_y, end_x, min_y]]], dtype=np.int32)
        
    def run_mission_control(self):
        # 주행 모드에 따라 로직 실행
        if self.current_mode == DriveMode.TRAFFIC_LIGHT_WAIT:
            self.target_speed = 0.0
            self.target_steering = 0.0
            self.get_logger().info("Waiting for green light...")

        elif self.current_mode == DriveMode.RUBBERCON_AVOIDANCE:
            self.target_speed = 0.5
            self.lidar_avoidance_logic()
            if self.rubbercon_passed:
                self.current_mode = DriveMode.LANE_FOLLOWING
                self.get_logger().info("Rubbercon avoidance completed. Switching to lane following.")
                
        elif self.current_mode == DriveMode.LANE_FOLLOWING:
            self.target_speed = 0.8
            self.overtake_check_logic()
            if self.overtaking_active:
                self.current_mode = DriveMode.OVERTAKING
                self.get_logger().info("Obstacle detected. Initiating overtaking maneuver.")
                
        elif self.current_mode == DriveMode.OVERTAKING:
            self.target_speed = 1.0 # 추월 시 가속
            self.overtaking_logic()
            
        # 제어 명령 발행
        twist = Twist()
        self.current_speed += (self.target_speed - self.current_speed) * 0.1
        self.current_steering += (self.target_steering - self.current_steering) * 0.1
        twist.linear.x = self.current_speed
        twist.angular.z = self.current_steering
        self.pub.publish(twist)

    def lidar_avoidance_logic(self):
        if self.lidar_data is None:
            return
        
        ranges = self.lidar_data.ranges
        num_ranges = len(ranges)
        
        # 전방 장애물 감지
        min_dist = min(ranges[num_ranges//4 : num_ranges//4*3] + ranges[num_ranges//4*3 : num_ranges//4] if num_ranges > 0 else [10.0])
        
        if min_dist < 0.5:
            # 장애물이 매우 가까우면 회피
            self.target_steering = -1.0 if ranges[num_ranges//2 - 10] > ranges[num_ranges//2 + 10] else 1.0
            self.target_speed = 0.3
        else:
            # 라바콘 구간 통과 완료
            if min(ranges) > 2.0 and not self.rubbercon_passed:
                self.get_logger().info("All rubber cones are passed.")
                self.rubbercon_passed = True
            
    def overtake_check_logic(self):
        if self.lidar_data is None:
            return
        
        ranges = self.lidar_data.ranges
        num_ranges = len(ranges)
        
        # 전방 차량 감지
        front_range = min(ranges[num_ranges // 2 - 50: num_ranges // 2 + 50] + ranges[num_ranges // 2 + 50 : num_ranges // 2 - 50] if num_ranges > 0 else [10.0])
        
        if front_range < 2.0 and front_range > 0.5:
            # 추월 모드 활성화
            self.overtaking_active = True
            
    def overtaking_logic(self):
        if self.lidar_data is None:
            return
            
        ranges = self.lidar_data.ranges
        num_ranges = len(ranges)
        
        # 전방 차량 감지 및 추월 완료 체크
        front_range = min(ranges[num_ranges // 2 - 50: num_ranges // 2 + 50] + ranges[num_ranges // 2 + 50 : num_ranges // 2 - 50] if num_ranges > 0 else [10.0])
        
        if front_range > 3.0:
            self.overtaking_active = False
            self.current_mode = DriveMode.LANE_FOLLOWING
            self.get_logger().info("Overtaking completed. Returning to lane following.")
            return

        # 좌우 차선에 공간이 있는지 확인 (예: 왼쪽이 1차선)
        left_dist = min(ranges[num_ranges // 2 + 10 : num_ranges // 2 + 60] + ranges[num_ranges // 2 + 60 : num_ranges // 2 + 10] if num_ranges > 0 else [10.0])
        right_dist = min(ranges[num_ranges // 2 - 60 : num_ranges // 2 - 10] + ranges[num_ranges // 2 - 10 : num_ranges // 2 - 60] if num_ranges > 0 else [10.0])
        
        # 추월 방향 결정 (더 넓은 공간으로)
        if left_dist > right_dist:
            # 왼쪽으로 추월
            self.target_steering = -0.5
        else:
            # 오른쪽으로 추월
            self.target_steering = 0.5
            
        self.target_speed = 1.5

    def get_processed_frame(self):
        with self.image_lock:
            return self.processed_frame

    def get_stats(self):
        """웹 대시보드용 통계 데이터 반환"""
        lidar_distance = "N/A"
        if self.lidar_data is not None and len(self.lidar_data.ranges) > 0:
            center = len(self.lidar_data.ranges) // 2
            front_range_indices = slice(center - 15, center + 15)
            front_ranges = self.lidar_data.ranges[front_range_indices]
            valid_ranges = [r for r in front_ranges if 0.05 < r < 10.0]
            if valid_ranges:
                lidar_distance = f"{min(valid_ranges):.2f}"
        
        return {
            "current_mode": self.current_mode.value,
            "rubbercon_status": "✅ PASSED" if self.rubbercon_passed else "🔍 SEARCHING",
            "lane_status": "✅ ACTIVE" if self.current_mode in [DriveMode.LANE_FOLLOWING, DriveMode.OVERTAKING] else "🔍 SEARCHING",
            "camera_fps": f"{self.camera_fps:.1f}",
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
