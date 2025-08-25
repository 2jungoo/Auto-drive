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
from http.server import BaseHTTPRequestHandler, HTTPServer

qos_profile = QoSProfile(depth=10)
qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

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
            <head><title>Autoracer Dashboard</title></head>
            <body style="background-color: #1e1e1e; color: white; font-family: Arial;">
                <h1>🚗 Autoracer Dashboard</h1>
                <div style="display: flex;">
                    <div>
                        <h3>📹 Camera Feed</h3>
                        <img src="/stream.mjpg" width="640" height="480" style="border: 2px solid #444;">
                        <p>Resolution: <span id="resolution">Loading...</span></p>
                    </div>
                    <div style="margin-left: 20px;">
                        <h3>🎯 Sensor Data</h3>
                        <p>Camera FPS: <span id="camera_fps">Loading...</span></p>
                        <p>Lidar Status: <span id="lidar_status">Loading...</span></p>
                        <p>Center Distance: <span id="center_dist">Loading...</span></p>
                        <p>Control Mode: <span id="control_mode">Loading...</span></p>
                        
                        <h3>📊 Statistics</h3>
                        <p>Processed Frames: <span id="frame_count">0</span></p>
                        <p>Lidar Scans: <span id="lidar_count">0</span></p>
                        <p>Uptime: <span id="uptime">0</span>s</p>
                    </div>
                </div>
                
                <script>
                setInterval(() => {
                    fetch('/stats')
                    .then(r => r.json())
                    .then(data => {
                        document.getElementById('camera_fps').textContent = data.camera_fps;
                        document.getElementById('lidar_status').textContent = data.lidar_status;
                        document.getElementById('center_dist').textContent = data.center_distance + 'm';
                        document.getElementById('control_mode').textContent = data.control_mode;
                        document.getElementById('frame_count').textContent = data.frame_count;
                        document.getElementById('lidar_count').textContent = data.lidar_count;
                        document.getElementById('uptime').textContent = Math.floor(data.uptime);
                    }).catch(e => console.log('Stats fetch error:', e));
                }, 1000);
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
                        # JPEG로 인코딩
                        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                        
                        self.wfile.write(b'--frame\r\n')
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', str(len(buffer)))
                        self.end_headers()
                        self.wfile.write(buffer)
                        self.wfile.write(b'\r\n')
                    
                    time.sleep(0.033)  # ~30 FPS
                    
            except Exception as e:
                self.autoracer.get_logger().error(f'스트리밍 에러: {e}')
                
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
        
        # 현재 데이터 저장
        self.current_image = None
        self.processed_frame = None
        self.lidar_data = None
        self.image_lock = threading.Lock()
        
        # 통계 데이터
        self.frame_count = 0
        self.lidar_count = 0
        self.start_time = time.time()
        self.last_camera_time = 0
        self.camera_fps = 0
        self.control_mode = "MANUAL"
        
        # ROS2 구독자들
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
        
        # ROS2 발행자 (제어 명령용)
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # 웹 서버 시작
        self.start_web_server()
        
        self.get_logger().info('🚗 Autoracer 시작됨!')
        self.get_logger().info('📊 대시보드: http://{}:8080/'.format(self.get_ip_address()))
    
    def get_ip_address(self):
        """현재 IP 주소 가져오기"""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return "localhost"
    
    def start_web_server(self):
        """웹 서버 시작"""
        def create_handler(*args, **kwargs):
            return WebViewer(self, *args, **kwargs)
        
        def run_server():
            try:
                server = HTTPServer(('0.0.0.0', 8080), create_handler)
                server.serve_forever()
            except Exception as e:
                self.get_logger().error(f'웹 서버 에러: {e}')
        
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()

    def image_callback(self, msg):
        """카메라 이미지 콜백"""
        try:
            # 압축된 이미지 디코딩
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if image is not None:
                with self.image_lock:
                    self.current_image = image.copy()
                
                # 이미지 처리 및 분석
                self.process_image(image)
                
                # 통계 업데이트
                self.frame_count += 1
                current_time = time.time()
                if self.last_camera_time > 0:
                    fps = 1.0 / (current_time - self.last_camera_time)
                    self.camera_fps = round(fps, 1)
                self.last_camera_time = current_time
                
                # 주기적 로그 (매 100프레임마다)
                if self.frame_count % 100 == 0:
                    self.get_logger().info(f'📸 처리된 프레임: {self.frame_count}, FPS: {self.camera_fps}')
                    
        except Exception as e:
            self.get_logger().error(f'이미지 처리 에러: {e}')

    def process_image(self, image):
        """이미지 처리 및 분석"""
        # 원본 이미지 복사
        processed = image.copy()
        
        # 이미지에 정보 표시
        height, width = image.shape[:2]
        
        # 헤더 정보
        cv2.rectangle(processed, (0, 0), (width, 80), (0, 0, 0), -1)
        cv2.putText(processed, f'Autoracer - Frame: {self.frame_count}', 
                   (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(processed, f'FPS: {self.camera_fps} | Mode: {self.control_mode}', 
                   (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # 중앙선 표시
        center_x = width // 2
        cv2.line(processed, (center_x, 0), (center_x, height), (0, 255, 255), 2)
        
        # 라이다 데이터가 있으면 표시
        if self.lidar_data is not None:
            self.draw_lidar_overlay(processed)
        
        # 컴퓨터 비전 처리
        self.detect_lanes(processed)
        self.detect_obstacles(processed)
        
        # 처리된 프레임 저장
        with self.image_lock:
            self.processed_frame = processed.copy()
    
    def detect_lanes(self, image):
        """차선 검출"""
        # 간단한 차선 검출 예시
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        
        # 관심 영역 설정 (하단 절반)
        height, width = image.shape[:2]
        mask = np.zeros_like(edges)
        roi_vertices = np.array([[(0, height), (width//2, height//2), (width, height)]], dtype=np.int32)
        cv2.fillPoly(mask, roi_vertices, 255)
        masked_edges = cv2.bitwise_and(edges, mask)
        
        # 허프 변환으로 직선 검출
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, threshold=50, 
                               minLineLength=50, maxLineGap=150)
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(image, (x1, y1), (x2, y2), (255, 0, 0), 3)
    
    def detect_obstacles(self, image):
        """장애물 검출 (간단한 예시)"""
        # 간단한 색상 기반 장애물 검출
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 빨간색 객체 검출 (예: 정지 신호)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        
        contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) > 500:  # 일정 크기 이상만
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(image, 'OBSTACLE', (x, y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    def draw_lidar_overlay(self, image):
        """라이다 데이터를 이미지에 오버레이"""
        if self.lidar_data is None:
            return
        
        height, width = image.shape[:2]
        
        # 라이다 데이터 중앙 부분
        total_points = len(self.lidar_data.ranges)
        center = total_points // 2
        
        # 전방 거리 표시
        front_distances = self.lidar_data.ranges[center-10:center+10]
        valid_distances = [d for d in front_distances if 0.1 < d < 10.0]
        
        if valid_distances:
            avg_distance = sum(valid_distances) / len(valid_distances)
            
            # 거리에 따른 색상 (가까우면 빨강, 멀면 초록)
            if avg_distance < 0.5:
                color = (0, 0, 255)  # 빨강
                status = "DANGER"
            elif avg_distance < 1.0:
                color = (0, 255, 255)  # 노랑
                status = "WARNING"
            else:
                color = (0, 255, 0)  # 초록
                status = "SAFE"
            
            # 전방 거리 표시
            cv2.rectangle(image, (10, height-80), (300, height-10), (0, 0, 0), -1)
            cv2.putText(image, f'Front: {avg_distance:.2f}m - {status}', 
                       (15, height-50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.putText(image, f'Lidar Points: {len(valid_distances)}', 
                       (15, height-25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    def lidar_callback(self, msg):
        """라이다 데이터 콜백"""
        self.lidar_data = msg
        self.lidar_count += 1
        
        # 전방 거리 계산
        total_points = len(msg.ranges)
        center = total_points // 2
        center_ranges = msg.ranges[center-5:center+5]
        valid_ranges = [r for r in center_ranges if 0.1 < r < 10.0]
        
        if valid_ranges:
            avg_distance = sum(valid_ranges) / len(valid_ranges)
            
            # 자동 제어 로직 (예시)
            if avg_distance < 0.3:
                self.control_mode = "EMERGENCY_STOP"
                self.publish_stop_command()
            elif avg_distance < 1.0:
                self.control_mode = "OBSTACLE_AVOIDANCE"
                self.publish_slow_command()
            else:
                self.control_mode = "NORMAL"
        
        # 주기적 로그 (매 50번마다)
        if self.lidar_count % 50 == 0:
            formatted_ranges = [f"{r:.3f}" for r in center_ranges[:5]]
            self.get_logger().info(f'🎯 중앙 거리: {formatted_ranges}')

    def publish_stop_command(self):
        """정지 명령 발행"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def publish_slow_command(self):
        """감속 명령 발행"""
        twist = Twist()
        twist.linear.x = 0.1  # 느린 속도
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def get_processed_frame(self):
        """처리된 프레임 반환 (웹 스트리밍용)"""
        with self.image_lock:
            if self.processed_frame is not None:
                return self.processed_frame.copy()
            else:
                # 더미 프레임 생성
                dummy = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(dummy, 'No Camera Data', (200, 240), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                return dummy

    def get_stats(self):
        """통계 데이터 반환"""
        current_time = time.time()
        uptime = current_time - self.start_time
        
        center_distance = "N/A"
        lidar_status = "No Data"
        
        if self.lidar_data is not None:
            total_points = len(self.lidar_data.ranges)
            center = total_points // 2
            center_ranges = self.lidar_data.ranges[center-5:center+5]
            valid_ranges = [r for r in center_ranges if 0.1 < r < 10.0]
            
            if valid_ranges:
                center_distance = f"{sum(valid_ranges) / len(valid_ranges):.2f}"
                lidar_status = "Active"
        
        return {
            'camera_fps': self.camera_fps,
            'lidar_status': lidar_status,
            'center_distance': center_distance,
            'control_mode': self.control_mode,
            'frame_count': self.frame_count,
            'lidar_count': self.lidar_count,
            'uptime': uptime
        }

def main(args=None):
    rclpy.init(args=args)
    
    try:
        autoracer = Autoracer()
        rclpy.spin(autoracer)
    except KeyboardInterrupt:
        pass
    finally:
        if 'autoracer' in locals():
            autoracer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
