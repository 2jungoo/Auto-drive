#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np
import threading
import time

qos_profile = QoSProfile(depth=10)
qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

class Autoracer(Node):
    def __init__(self):
        super().__init__('Autoracer')
        
        # 라이다 구독
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile
        )
        self.lidar_sub
        
        # 카메라 초기화 및 스레드 시작
        self.cap = None
        self.init_camera()
        
        if self.cap is not None:
            self.camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
            self.camera_thread.start()
        
        self.get_logger().info('Autoracer 노드가 시작되었습니다.')
        if self.cap is not None:
            self.get_logger().info('카메라 창을 확인하세요. ESC키로 종료 가능합니다.')
    
    def init_camera(self):
        """카메라 초기화 - 여러 방법으로 시도"""
        
        # 방법 1: CSI 카메라 (높은 해상도) - cvcam.py와 동일
        csi_pipeline_high = (
            'nvarguscamerasrc ! '
            'video/x-raw(memory:NVMM), width=3280, height=2464, format=(string)NV12, framerate=(fraction)20/1 ! '
            'nvvidconv ! video/x-raw, format=(string)BGRx ! '
            'videoconvert ! video/x-raw, format=(string)BGR ! appsink'
        )
        
        # 방법 2: CSI 카메라 (중간 해상도) - csicam.py와 동일
        csi_pipeline_mid = (
            'nvarguscamerasrc ! '
            'video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! '
            'nvvidconv ! video/x-raw, format=BGRx ! '
            'videoconvert ! video/x-raw, format=BGR ! appsink'
        )
        
        # 방법 3: CSI 카메라 (저해상도)
        csi_pipeline_low = (
            'nvarguscamerasrc ! '
            'video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! '
            'nvvidconv ! video/x-raw, format=BGRx ! '
            'videoconvert ! video/x-raw, format=BGR ! appsink'
        )
        
        pipelines = [
            ("CSI 고해상도", csi_pipeline_high),
            ("CSI 중해상도", csi_pipeline_mid), 
            ("CSI 저해상도", csi_pipeline_low)
        ]
        
        # CSI 카메라들 순차 시도
        for name, pipeline in pipelines:
            try:
                self.get_logger().info(f'{name} 카메라 시도 중...')
                self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
                if self.cap.isOpened():
                    # 실제 프레임 읽기 테스트
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
        
        # USB 카메라들 시도 (0~3번 포트) - usbcam.py와 동일
        for i in range(4):
            try:
                self.get_logger().info(f'USB 카메라 {i}번 포트 시도 중...')
                self.cap = cv2.VideoCapture(i)
                if self.cap.isOpened():
                    # 해상도 설정
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    self.cap.set(cv2.CAP_PROP_FPS, 30)
                    
                    # 실제 프레임 읽기 테스트
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
        
        # 모든 카메라 연결 실패
        self.get_logger().error('❌ 모든 카메라 연결 실패')
        self.cap = None
    
    def camera_loop(self):
        """카메라 프레임을 지속적으로 읽고 화면에 표시"""
        consecutive_failures = 0
        max_failures = 10
        
        while rclpy.ok():
            try:
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    consecutive_failures = 0
                    
                    # 프레임에 정보 표시
                    cv2.putText(frame, 'Autoracer Camera', (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(frame, f'Resolution: {frame.shape[1]}x{frame.shape[0]}', (10, 60), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                    cv2.putText(frame, 'Press ESC to exit', (10, frame.shape[0]-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                    
                    # 화면에 표시
                    cv2.imshow('Autoracer Camera', frame)
                    
                    # ESC 키로 종료
                    key = cv2.waitKey(1) & 0xFF
                    if key == 27:  # ESC key
                        self.get_logger().info('ESC 키 입력 - 카메라 종료')
                        break
                        
                else:
                    consecutive_failures += 1
                    if consecutive_failures <= 3:
                        self.get_logger().warn(f'프레임 읽기 실패 ({consecutive_failures}번째)')
                    
                    if consecutive_failures >= max_failures:
                        self.get_logger().error('연속 프레임 읽기 실패. 카메라 재연결 시도...')
                        self.reconnect_camera()
                        consecutive_failures = 0
                        if self.cap is None:
                            break
                    
                    time.sleep(0.1)
                    
            except Exception as e:
                consecutive_failures += 1
                if consecutive_failures <= 3:
                    self.get_logger().error(f'카메라 루프 에러: {e}')
                time.sleep(1)
        
        # 정리
        cv2.destroyAllWindows()
    
    def reconnect_camera(self):
        """카메라 재연결 시도"""
        if self.cap is not None:
            self.cap.release()
            time.sleep(1)
        
        self.get_logger().info('카메라 재연결 중...')
        self.init_camera()

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
        """소멸자 - 리소스 정리"""
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = Autoracer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C 입력 - 프로그램 종료')
    finally:
        if hasattr(node, 'cap') and node.cap is not None:
            node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
