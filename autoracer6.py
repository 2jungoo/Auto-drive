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
import math # 수학 연산을 위해 추가
from http.server import BaseHTTPRequestHandler, HTTPServer
from enum import Enum
import json
import tf_transformations

# QoS 프로파일 설정
qos_profile = QoSProfile(depth=10)
qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

# 주행 모드 정의
class DriveMode(Enum):
    TRAFFIC_LIGHT_WAIT = "TRAFFIC_LIGHT_WAIT"
    RUBBERCON_AVOIDANCE = "RUBBERCON_AVOID"
    LANE_FOLLOWING = "LANE_FOLLOW"
    OVERTAKING = "OVERTAKING"
    EMERGENCY_STOP = "EMERGENCY_STOP"

# PID 제어기 클래스
class PIDController:
    def __init__(self, kp, ki, kd):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.last_error = 0.0
        self.integral = 0.0

    def calculate(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.last_error = error
        return output

# 웹 대시보드 서버 클래스
class WebViewer(BaseHTTPRequestHandler):
    def __init__(self, autoracer_node, *args, **kwargs):
        self.autoracer = autoracer_node
        super().__init__(*args, **kwargs)

    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
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
                        <h3>📹 실시간 카메라 영상</h3>
                        <img src="/stream.mjpg" width="800" height="600" style="border: 2px solid #444; border-radius: 8px; width: 100%; max-width: 800px;">
                    </div>
                    <div class="panel" style="flex: 1;">
                        <h3>🎯 미션 제어</h3>
                        <div class="status-card">
                            <div class="metric"><span>현재 모드:</span><span id="mode" class="metric-value">로딩중...</span></div>
                            <div class="metric"><span>라바콘 상태:</span><span id="rubbercon" class="metric-value">탐색중...</span></div>
                            <div class="metric"><span>차선 상태:</span><span id="lane" class="metric-value">탐색중...</span></div>
                        </div>
                        
                        <h3>📊 차량 정보</h3>
                        <div class="status-card">
                            <div class="metric"><span>카메라 FPS:</span><span id="camera_fps" class="metric-value">0</span></div>
                            <div class="metric"><span>라이다 거리:</span><span id="lidar_dist" class="metric-value">N/A</span> m</div>
                            <div class="metric"><span>속도:</span><span id="speed" class="metric-value">0</span> m/s</div>
                            <div class="metric"><span>조향각:</span><span id="steering" class="metric-value">0</span>°</div>
                            <div class="metric"><span>미션 시간:</span><span id="mission_time" class="metric-value">00:00</span></div>
                        </div>
                        
                        <h3>🏁 미션 진행도</h3>
                        <div class="progress-bar">
                            <div id="progress" class="progress-fill" style="width: 0%;"></div>
                        </div>
                        <div style="font-size: 14px;">
                            <p id="mission1" style="color: #ffaa00;">🔄 미션 1: 라바콘 회피</p>
                            <p id="mission2" style="color: #666;">⏳ 미션 2: 차선 주행</p>
                        </div>
                        
                        <h3>⚠️ 시스템 경고</h3>
                        <div id="alerts" style="background: rgba(255,0,0,0.1);
                            border-radius: 8px; padding: 15px; min-height: 50px;">
                            <span style="color: #aaa;">시스템 정상</span>
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
                        
                        let elapsed = Math.floor((Date.now() - startTime) / 1000);
                        let mins = Math.floor(elapsed / 60);
                        let secs = elapsed % 60;
                        document.getElementById('mission_time').textContent = 
                            `${mins.toString().padStart(2,'0')}:${secs.toString().padStart(2,'0')}`;
                        
                        let progress = 0;
                        if (data.rubbercon_status.includes('PASSED')) {
                            progress = 100;
                        } else if (data.current_mode.includes('RUBBERCON')) {
                            progress = 50;
                        }
                        document.getElementById('progress').style.width = progress + '%';

                        const mission1 = document.getElementById('mission1');
                        const mission2 = document.getElementById('mission2');
                        if (progress == 100) {
                            mission1.style.color = '#00ff00'; // Green for passed
                            mission2.style.color = '#00ff00'; // Green for active/next
                        } else if (progress == 50) {
                            mission1.style.color = '#ffaa00'; // Yellow for in-progress
                            mission2.style.color = '#666';   // Gray for pending
                        }

                        let alerts = [];
                        if (parseFloat(data.lidar_distance) < 0.3) alerts.push('⚠️ 장애물 근접');
                        if (parseFloat(data.camera_fps) < 10) alerts.push('📹 카메라 FPS 낮음');
                        if (Math.abs(parseFloat(data.steering_angle)) > 25) alerts.push('🎯 급격한 조향');
                        document.getElementById('alerts').innerHTML = alerts.length > 0 ? alerts.join('<br>') : '<span style="color: #aaa;">시스템 정상</span>';
                    }).catch(e => console.log('Stats error:', e));
                }, 500);
                </script>
            </body>
            </html>
            """
            self.wfile.write(html.encode('utf-8'))
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

# 메인 Autoracer 노드 클래스
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
        self.start_time = time.time() # 미션 시작 시간 기록
        
        # 제어 변수
        self.current_speed = 0.0
        self.current_steering = 0.0
        self.target_speed = 0.0
        self.target_steering = 0.0
        
        # PID 제어기 (차선 주행용)
        self.pid_controller = PIDController(kp=0.004, ki=0.0001, kd=0.001)
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
            self.get_logger().info('웹 서버 시작 (포트 8000)')
            server.serve_forever()
        except Exception as e:
            self.get_logger().error(f"웹 서버 시작 실패: {e}")

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.current_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        with self.image_lock:
            # FPS 계산
            current_time = time.time()
            dt = current_time - self.last_frame_time
            self.camera_fps = 1.0 / dt if dt > 0 else 0
            self.last_frame_time = current_time
            
            # 주행 모드에 따른 이미지 처리
            if self.current_mode == DriveMode.TRAFFIC_LIGHT_WAIT:
                self.processed_frame = self.process_traffic_light()
            elif self.current_mode in [DriveMode.LANE_FOLLOWING, DriveMode.OVERTAKING]:
                self.processed_frame = self.process_lane_lines()
            elif self.current_mode == DriveMode.RUBBERCON_AVOIDANCE:
                # 라바콘 회피는 라이다 기반이므로 원본 이미지를 표시
                self.processed_frame = self.current_image.copy()
            else:
                self.processed_frame = self.current_image

    def lidar_callback(self, msg):
        self.lidar_data = msg

    def process_traffic_light(self):
        img_copy = self.current_image.copy()
        hsv = cv2.cvtColor(img_copy, cv2.COLOR_BGR2HSV)
        lower_green = np.array([35, 100, 100])
        upper_green = np.array([85, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # 초록색이 충분히 감지되면 출발
        if np.sum(mask) > 5000:
            self.get_logger().info("✅ 초록불 감지! 주행을 시작합니다.")
            self.current_mode = DriveMode.RUBBERCON_AVOIDANCE
            self.start_time = time.time() # 미션 시작 시간 초기화
            
        return cv2.bitwise_and(img_copy, img_copy, mask=mask)

    def process_lane_lines(self):
        # ROI 설정
        height, width, _ = self.current_image.shape
        roi_vertices = [
            (0, height),
            (width / 2 - 250, height / 2 + 100),
            (width / 2 + 250, height / 2 + 100),
            (width, height)
        ]
        
        # 이미지 전처리
        gray = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        
        # ROI 마스크 적용
        mask = np.zeros_like(edges)
        cv2.fillPoly(mask, [np.array(roi_vertices, np.int32)], 255)
        masked_edges = cv2.bitwise_and(edges, mask)
        
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, 50, maxLineGap=50, minLineLength=50)
        
        left_lines, right_lines = [], []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x2 - x1 == 0: continue
                slope = (y2 - y1) / (x2 - x1)
                if abs(slope) > 0.3:
                    if slope < 0: left_lines.append(line)
                    else: right_lines.append(line)
                            
        left_avg_line = self.calculate_avg_line(left_lines)
        right_avg_line = self.calculate_avg_line(right_lines)
        
        center_x = width // 2
        lane_center_x = None
        
        if left_avg_line is not None and right_avg_line is not None:
            _, _, left_x2, _ = left_avg_line[0]
            _, _, right_x2, _ = right_avg_line[0]
            lane_center_x = (left_x2 + right_x2) // 2
        elif left_avg_line is not None:
            _, _, x2, _ = left_avg_line[0]
            lane_center_x = x2 + 320 # 한쪽 차선만 보일 때 중앙 예측
        elif right_avg_line is not None:
            _, _, x2, _ = right_avg_line[0]
            lane_center_x = x2 - 320 # 한쪽 차선만 보일 때 중앙 예측
            
        # PID 조향 제어
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
            cv2.line(processed, (x1, y1), (x2, y2), (0, 255, 0), 10)
        if right_avg_line is not None:
            x1, y1, x2, y2 = right_avg_line[0]
            cv2.line(processed, (x1, y1), (x2, y2), (0, 255, 0), 10)
        if lane_center_x is not None:
            cv2.line(processed, (lane_center_x, height), (center_x, int(height*0.7)), (0, 0, 255), 3)

        return processed

    def calculate_avg_line(self, lines):
        if not lines: return None
        
        x_coords, y_coords = [], []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            x_coords.extend([x1, x2])
            y_coords.extend([y1, y2])
        
        poly = np.poly1d(np.polyfit(y_coords, x_coords, deg=1))
        
        height, _, _ = self.current_image.shape
        start_y = height
        end_y = int(height * 0.7)
        
        start_x = int(poly(start_y))
        end_x = int(poly(end_y))
        
        return np.array([[[start_x, start_y, end_x, end_y]]], dtype=np.int32)
        
    def run_mission_control(self):
        if self.current_mode == DriveMode.TRAFFIC_LIGHT_WAIT:
            self.target_speed = 0.0
            self.target_steering = 0.0
        
        elif self.current_mode == DriveMode.RUBBERCON_AVOIDANCE:
            self.target_speed = 0.5 # 라바콘 구간은 일정한 속도로
            self.lidar_avoidance_logic() # 회피 로직 실행
            if self.rubbercon_passed:
                self.current_mode = DriveMode.LANE_FOLLOWING
                self.get_logger().info("✅ 라바콘 통과. 차선 주행 모드로 전환합니다.")
                
        elif self.current_mode == DriveMode.LANE_FOLLOWING:
            self.target_speed = 0.8
            self.overtake_check_logic()
            if self.overtaking_active:
                self.current_mode = DriveMode.OVERTAKING
                self.get_logger().info("⚠️ 장애물 감지. 추월을 시작합니다.")
                
        elif self.current_mode == DriveMode.OVERTAKING:
            self.target_speed = 1.2 # 추월 시 가속
            self.overtaking_logic()
            
        # 제어 명령 발행 (부드러운 가감속/조향)
        twist = Twist()
        self.current_speed += (self.target_speed - self.current_speed) * 0.1
        self.current_steering += (self.target_steering - self.current_steering) * 0.2
        twist.linear.x = self.current_speed
        twist.angular.z = self.current_steering
        self.pub.publish(twist)

    def lidar_avoidance_logic(self):
        """
        [핵심 수정] 라바콘 사이의 가장 넓고 안전한 경로(gap)를 찾아 주행하는 로직
        """
        if self.lidar_data is None or len(self.lidar_data.ranges) == 0:
            return
        
        ranges = np.array(self.lidar_data.ranges)
        num_ranges = len(ranges)
        
        # 전방 180도 영역을 탐색
        start_idx = num_ranges // 4
        end_idx = 3 * num_ranges // 4
        front_ranges = ranges[start_idx:end_idx]
        
        # 가장 긴 연속적인 '안전' 구간 찾기
        safe_distance = 0.8  # 장애물과 유지할 최소 거리 (m)
        max_gap_len = 0
        best_gap_start_idx = -1
        
        current_gap_len = 0
        for i, distance in enumerate(front_ranges):
            # 거리가 안전거리보다 멀거나, 무한대(측정불가)인 경우 안전한 것으로 간주
            if distance > safe_distance or math.isinf(distance):
                current_gap_len += 1
            else:
                # 안전 구간이 끝났을 때, 지금까지의 최대 길이와 비교
                if current_gap_len > max_gap_len:
                    max_gap_len = current_gap_len
                    best_gap_start_idx = i - current_gap_len
                current_gap_len = 0
        # 마지막 구간까지 확인
        if current_gap_len > max_gap_len:
            max_gap_len = current_gap_len
            best_gap_start_idx = len(front_ranges) - current_gap_len

        # --- 조향 로직 ---
        if max_gap_len > 15:  # 주행 가능한 충분히 넓은 공간이 있을 경우
            # 가장 넓은 공간의 중앙을 목표 지점으로 설정
            target_idx = best_gap_start_idx + max_gap_len // 2
            view_center_idx = len(front_ranges) // 2
            
            # 목표 지점과 현재 중앙의 차이(오차) 계산
            error = view_center_idx - target_idx
            
            # 오차에 비례하여 조향값 결정 (P제어)
            steering_gain = 0.01
            self.target_steering = -steering_gain * error
            self.target_steering = np.clip(self.target_steering, -1.0, 1.0)
        else:
            # 안전한 공간이 없을 경우: 속도를 줄이고 가장 가까운 장애물로부터 회피
            self.get_logger().warn("안전한 경로 없음! 비상 회피 기동.")
            self.target_speed = 0.2
            closest_idx = np.argmin(front_ranges)
            view_center_idx = len(front_ranges) // 2
            if closest_idx < view_center_idx:
                self.target_steering = 0.8  # 장애물이 왼쪽에 있으면 오른쪽으로 조향
            else:
                self.target_steering = -0.8 # 장애물이 오른쪽에 있으면 왼쪽으로 조향

        # --- 모드 종료 조건 ---
        # 전방이 일정 거리 이상 깨끗하고, 모드 진입 후 일정 시간이 지났으면 통과로 간주
        center_view_start = num_ranges // 2 - 15
        center_view_end = num_ranges // 2 + 15
        min_front_dist = np.min(ranges[center_view_start:center_view_end])
        
        time_in_mode = time.time() - self.start_time
        if min_front_dist > 3.5 and time_in_mode > 5.0:
            self.rubbercon_passed = True

    def overtake_check_logic(self):
        if self.lidar_data is None or len(self.lidar_data.ranges) == 0: return
        
        ranges = self.lidar_data.ranges
        num_ranges = len(ranges)
        
        # 전방의 좁은 영역만 확인하여 차량 감지
        front_view_start = num_ranges // 2 - 10
        front_view_end = num_ranges // 2 + 10
        front_range = min(ranges[front_view_start:front_view_end])
        
        if 0.5 < front_range < 2.0:
            self.overtaking_active = True
            
    def overtaking_logic(self):
        if self.lidar_data is None or len(self.lidar_data.ranges) == 0: return

        ranges = self.lidar_data.ranges
        num_ranges = len(ranges)
        
        # 추월 완료 확인 (전방이 깨끗해졌는지)
        front_view_start = num_ranges // 2 - 10
        front_view_end = num_ranges // 2 + 10
        front_range = min(ranges[front_view_start:front_view_end])
        
        if front_range > 3.0:
            self.overtaking_active = False
            self.current_mode = DriveMode.LANE_FOLLOWING
            self.get_logger().info("✅ 추월 완료. 차선 주행으로 복귀합니다.")
            return

        # 좌/우 공간을 확인하여 추월 방향 결정
        left_view_start = num_ranges // 2 + 20
        left_view_end = num_ranges // 2 + 70
        right_view_start = num_ranges // 2 - 70
        right_view_end = num_ranges // 2 - 20
        
        left_dist = min(ranges[left_view_start:left_view_end])
        right_dist = min(ranges[right_view_start:right_view_end])
        
        # 더 넓은 공간으로 추월
        if left_dist > right_dist and left_dist > 2.0:
            self.target_steering = -0.6 # 좌회전
        elif right_dist > 1.5:
            self.target_steering = 0.6  # 우회전
        else: # 공간이 없으면 일단 차선 유지
            self.current_mode = DriveMode.LANE_FOLLOWING 

    def get_processed_frame(self):
        with self.image_lock:
            return self.processed_frame

    def get_stats(self):
        lidar_distance = "N/A"
        if self.lidar_data is not None and len(self.lidar_data.ranges) > 0:
            center = len(self.lidar_data.ranges) // 2
            front_range_indices = slice(center - 5, center + 5)
            front_ranges = self.lidar_data.ranges[front_range_indices]
            valid_ranges = [r for r in front_ranges if 0.05 < r < 10.0]
            if valid_ranges:
                lidar_distance = f"{min(valid_ranges):.2f}"
        
        return {
            "current_mode": self.current_mode.value,
            "rubbercon_status": "✅ PASSED" if self.rubbercon_passed else "🚧 NAVIGATING",
            "lane_status": "✅ ACTIVE" if self.current_mode in [DriveMode.LANE_FOLLOWING, DriveMode.OVERTAKING] else "STANDBY",
            "camera_fps": f"{self.camera_fps:.1f}",
            "lidar_distance": lidar_distance,
            "speed": f"{self.current_speed:.2f}",
            "steering_angle": f"{math.degrees(self.current_steering):.1f}",
        }

# 메인 함수
def main(args=None):
    rclpy.init(args=args)
    autoracer = Autoracer()
    rclpy.spin(autoracer)
    autoracer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
