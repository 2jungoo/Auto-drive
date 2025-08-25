#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np
import threading
import time
import socket
from http.server import BaseHTTPRequestHandler, HTTPServer
import urllib.parse

qos_profile = QoSProfile(depth=10)
qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

class StreamingHandler(BaseHTTPRequestHandler):
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
            <head><title>Autoracer Camera</title></head>
            <body>
                <h1>Autoracer Camera Stream</h1>
                <img src="/stream.mjpg" width="640" height="480">
                <br><br>
                <p>Camera Status: <span id="status">Connected</span></p>
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
                    frame = self.autoracer.get_current_frame()
                    if frame is not None:
                        # 프레임 리사이즈 (웹 표시용)
                        frame_resized = cv2.resize(frame, (640, 480))
                        
                        # 정보 텍스트 추가
                        cv2.putText(frame_resized, f'Autoracer - {time.strftime("%H:%M:%S")}', 
                                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        cv2.putText(frame_resized, f'Resolution: {frame.shape[1]}x{frame.shape[0]}', 
                                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                        
                        # JPEG로 인코딩
                        _, buffer = cv2.imencode('.jpg', frame_resized, [cv2.IMWRITE_JPEG_QUALITY, 80])
                        
                        # 스트림 전송
                        self.wfile.write(b'--frame\r\n')
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', str(len(buffer)))
                        self.end_headers()
                        self.wfile.write(buffer)
                        self.wfile.write(b'\r\n')
                    
                    time.sleep(0.03)  # ~33 FPS
                    
            except Exception as e:
                self.autoracer.get_logger().error(f'스트리밍 에러: {e}')
        else:
            self.send_response(404)
            self.end_headers()

class Autoracer(Node):
    def __init__(self):
        super().__init__('Autoracer')
        
        # 현재 프레임 저장
        self.current_frame = None
        self.frame_lock = threading.Lock()
        
        # 라이다 구독
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile
        )
        self.lidar_sub
        
        # 카메라 초기화
        self.cap = None
        self.init_camera()
        
        # 카메라 스레드 시작
        if self.cap is not None:
            self.camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
            self.camera_thread.start()
        
        # 웹 서버 시작
        self.start_web_server()
        
        self.get_logger().info('Autoracer 노드가 시작되었습니다.')
        
    def get_ip_address(self):
        """현재 IP 주소 가져오기"""
        try:
            # 외부와 연결해서 로컬 IP 확인
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
            return StreamingHandler(self, *args, **kwargs)
        
        def run_server():
            try:
                server = HTTPServer(('0.0.0.0', 8080), create_handler)
                ip_addr = self.get_ip_address()
                self.get_logger().info(f'🌐 웹 서버 시작됨!')
                self.get_logger().info(f'📹 카메라 스트림: http://{ip_addr}:8080/')
                self.get_logger().info(f'📹 또는: http://localhost:8080/')
                server.serve_forever()
            except Exception as e:
                self.get_logger().error(f'웹 서버 에러: {e}')
        
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()
    
    def init_camera(self):
        """카메라 초기화"""
        
        # CSI 카메라 파이프라인들
        pipelines = [
            ("CSI 고해상도", 'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=3280, height=2464, format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink'),
            ("CSI 중해상도", 'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'),
            ("CSI 저해상도", 'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink')
        ]
        
        # CSI 카메라 시도
        for name, pipeline in pipelines:
            try:
                self.get_logger().info(f'{name} 카메라 시도 중...')
                self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
                if self.cap.isOpened():
                    ret, frame = self.cap.read()
                    if ret and frame is not None:
                        self.get_logger().info(f'✅ {name} 카메라 연결 성공 - 해상도: {frame.shape}')
                        return
                    else:
                        self.cap.release()
                        self.get_logger().warn(f'{name} 카메라 열렸지만 프레임 읽기 실패')
                else:
                    self.get_logger().warn(f'{name} 카메라 열기 실패')
                    if self.cap:
                        self.cap.release()
            except Exception as e:
                self.get_logger().warn(f'{name} 카메라 연결 실패: {e}')
                if self.cap:
                    self.cap.release()
        
        # USB 카메라 시도
        for i in range(4):
            try:
                self.get_logger().info(f'USB 카메라 {i}번 포트 시도 중...')
                self.cap = cv2.VideoCapture(i)
                if self.cap.isOpened():
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    self.cap.set(cv2.CAP_PROP_FPS, 30)
                    
                    ret, frame = self.cap.read()
                    if ret and frame is not None:
                        self.get_logger().info(f'✅ USB 카메라 {i}번 연결 성공 - 해상도: {frame.shape}')
                        return
                    else:
                        self.cap.release()
                        self.get_logger().warn(f'USB 카메라 {i}번 열렸지만 프레임 읽기 실패')
                else:
                    self.get_logger().warn(f'USB 카메라 {i}번 열기 실패')
                    if self.cap:
                        self.cap.release()
            except Exception as e:
                self.get_logger().warn(f'USB 카메라 {i}번 연결 실패: {e}')
                if self.cap:
                    self.cap.release()
        
        self.get_logger().error('❌ 모든 카메라 연결 실패')
        self.cap = None
    
    def camera_loop(self):
        """카메라 프레임 읽기 루프"""
        consecutive_failures = 0
        
        while rclpy.ok():
            try:
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    consecutive_failures = 0
                    with self.frame_lock:
                        self.current_frame = frame.copy()
                else:
                    consecutive_failures += 1
                    if consecutive_failures <= 3:
                        self.get_logger().warn(f'프레임 읽기 실패 ({consecutive_failures}번째)')
                    
                    if consecutive_failures >= 10:
                        self.get_logger().error('카메라 재연결 시도...')
                        self.reconnect_camera()
                        consecutive_failures = 0
                    
                    time.sleep(0.1)
                    
            except Exception as e:
                self.get_logger().error(f'카메라 루프 에러: {e}')
                time.sleep(1)
    
    def get_current_frame(self):
        """현재 프레임 반환 (웹 스트리밍용)"""
        with self.frame_lock:
            if self.current_frame is not None:
                return self.current_frame.copy()
            else:
                # 카메라 없을 때 더미 프레임
                dummy = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(dummy, 'No Camera Connected', (150, 240), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                return dummy
    
    def reconnect_camera(self):
        """카메라 재연결"""
        if self.cap is not None:
            self.cap.release()
            time.sleep(1)
        self.init_camera()

    def lidar_callback(self, msg):
        """라이다 데이터 콜백"""
        total_points = len(msg.ranges)
        center = total_points // 2
        half_width = 5
        
        start_idx = max(0, center - half_width)
        end_idx = min(total_points, center + half_width)
        
        center_ranges = msg.ranges[start_idx:end_idx]
        formatted_ranges = [f"{r:.3f}" for r in center_ranges]
        
        self.get_logger().info(f"Center 10 ranges: {formatted_ranges}")
    
    def __del__(self):
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = Autoracer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C - 종료')
    finally:
        if hasattr(node, 'cap') and node.cap is not None:
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
