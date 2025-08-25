#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
import time

qos_profile = QoSProfile(depth=10)
qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

class StreamHandler(BaseHTTPRequestHandler):
    def __init__(self, frame_getter, *args, **kwargs):
        self.frame_getter = frame_getter
        super().__init__(*args, **kwargs)
    
    def do_GET(self):
        if self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            
            while True:
                try:
                    frame = self.frame_getter()
                    if frame is not None:
                        _, buffer = cv2.imencode('.jpg', frame)
                        self.wfile.write(b'--frame\r\n')
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', len(buffer))
                        self.end_headers()
                        self.wfile.write(buffer)
                        self.wfile.write(b'\r\n')
                    time.sleep(0.033)  # ~30 FPS
                except Exception as e:
                    print(f"스트리밍 에러: {e}")
                    break
        else:
            self.send_response(404)
            self.end_headers()

class Autoracer(Node):
    def __init__(self):
        super().__init__('Autoracer')
        
        # 현재 프레임을 저장할 변수
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
        self.init_camera()
        
        # HTTP 서버 시작
        self.start_http_server()
        
        # 카메라 프레임 읽기 스레드 시작
        self.camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
        self.camera_thread.start()
        
        self.get_logger().info('Autoracer 노드가 시작되었습니다.')
        self.get_logger().info('카메라 스트림: http://젯슨IP주소:8080/stream')
    
    def init_camera(self):
        """카메라 초기화 - CSI, USB 순서대로 시도"""
        self.cap = None
        
        # CSI 카메라 시도
        csi_pipeline = (
            'nvarguscamerasrc ! '
            'video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! '
            'nvvidconv ! video/x-raw, format=BGRx ! '
            'videoconvert ! video/x-raw, format=BGR ! appsink'
        )
        
        try:
            self.cap = cv2.VideoCapture(csi_pipeline, cv2.CAP_GSTREAMER)
            if self.cap.isOpened():
                self.get_logger().info('✅ CSI 카메라 연결 성공')
                return
        except Exception as e:
            self.get_logger().warn(f'CSI 카메라 연결 실패: {e}')
        
        # USB 카메라 시도
        try:
            self.cap = cv2.VideoCapture(0)
            if self.cap.isOpened():
                self.get_logger().info('✅ USB 카메라 연결 성공')
                return
        except Exception as e:
            self.get_logger().warn(f'USB 카메라 연결 실패: {e}')
        
        # 모든 카메라 연결 실패
        self.get_logger().error('❌ 모든 카메라 연결 실패')
        self.cap = None
    
    def camera_loop(self):
        """카메라 프레임을 지속적으로 읽는 루프"""
        if self.cap is None:
            return
            
        while rclpy.ok():
            try:
                ret, frame = self.cap.read()
                if ret:
                    with self.frame_lock:
                        self.current_frame = frame.copy()
                else:
                    self.get_logger().warn('프레임 읽기 실패')
                    time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f'카메라 루프 에러: {e}')
                time.sleep(1)
    
    def get_current_frame(self):
        """현재 프레임 반환 (HTTP 서버용)"""
        with self.frame_lock:
            if self.current_frame is not None:
                return self.current_frame.copy()
            else:
                # 기본 이미지 생성 (카메라 없을 때)
                dummy_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(dummy_frame, 'No Camera', (200, 240), 
                           cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 3)
                return dummy_frame
    
    def start_http_server(self):
        """HTTP 스트리밍 서버 시작"""
        def create_handler(*args, **kwargs):
            return StreamHandler(self.get_current_frame, *args, **kwargs)
        
        def run_server():
            try:
                server = HTTPServer(('0.0.0.0', 8080), create_handler)
                self.get_logger().info('HTTP 서버 시작: http://0.0.0.0:8080/stream')
                server.serve_forever()
            except Exception as e:
                self.get_logger().error(f'HTTP 서버 에러: {e}')
        
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()

    def lidar_callback(self, msg):
        """라이다 데이터 콜백"""
        total_points = len(msg.ranges)
        
        # 가운데 10개 인덱스 계산
        center = total_points // 2
        half_width = 10 // 2
        
        start_idx = max(0, center - half_width)
        end_idx = min(total_points, center + half_width)
        
        center_ranges = msg.ranges[start_idx:end_idx]
        formatted_ranges = [f"{r:.3f}" for r in center_ranges]
        
        self.get_logger().info(f"Center 10 ranges: {formatted_ranges}")
    
    def __del__(self):
        """소멸자 - 카메라 해제"""
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = Autoracer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'cap') and node.cap is not None:
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
