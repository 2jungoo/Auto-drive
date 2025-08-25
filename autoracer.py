#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
import subprocess
import threading
import time
import cv2
import numpy as np

class GstCamPublisher(Node):
    def __init__(self):
        super().__init__('gstcam_pub_node')
        
        # CompressedImage 퍼블리셔 생성
        self.image_publisher = self.create_publisher(
            CompressedImage,
            '/image_raw/compressed',
            10
        )
        
        # GStreamer 파이프라인으로 직접 카메라 열기
        self.init_gstreamer_camera()
        
        if self.cap is not None and self.cap.isOpened():
            # 타이머로 주기적 발행 (30 FPS)
            self.timer = self.create_timer(1.0/30.0, self.publish_image)
            self.get_logger().info('✅ GStreamer 카메라 시작됨!')
        else:
            self.get_logger().error('❌ GStreamer 카메라 초기화 실패')
    
    def init_gstreamer_camera(self):
        """GStreamer로 CSI 카메라 초기화"""
        # 작동하는 것으로 확인된 GStreamer 파이프라인
        pipeline = (
            'nvarguscamerasrc ! '
            'video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! '
            'nvvidconv ! '
            'video/x-raw, format=BGRx ! '
            'videoconvert ! '
            'video/x-raw, format=BGR ! '
            'appsink'
        )
        
        try:
            self.get_logger().info('🔍 GStreamer CSI 카메라 초기화 중...')
            self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            
            if self.cap.isOpened():
                # 초기화 시간
                time.sleep(2)
                
                # 테스트 프레임 읽기
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    self.get_logger().info(f'✅ GStreamer 카메라 성공! 해상도: {frame.shape}')
                else:
                    self.get_logger().error('❌ GStreamer 카메라 프레임 읽기 실패')
                    self.cap = None
            else:
                self.get_logger().error('❌ GStreamer 카메라 열기 실패')
                self.cap = None
                
        except Exception as e:
            self.get_logger().error(f'❌ GStreamer 카메라 에러: {e}')
            self.cap = None
    
    def publish_image(self):
        """이미지 읽기 및 ROS2 토픽 발행"""
        if self.cap is None or not self.cap.isOpened():
            return
        
        try:
            ret, frame = self.cap.read()
            if ret and frame is not None:
                # CompressedImage 메시지 생성
                msg = CompressedImage()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_frame'
                msg.format = 'jpeg'
                
                # JPEG로 압축
                encode_param = [cv2.IMWRITE_JPEG_QUALITY, 90]
                _, encoded_image = cv2.imencode('.jpg', frame, encode_param)
                msg.data = encoded_image.tobytes()
                
                # ROS2 토픽으로 발행
                self.image_publisher.publish(msg)
                
            else:
                self.get_logger().warn('GStreamer 프레임 읽기 실패')
                
        except Exception as e:
            self.get_logger().error(f'이미지 발행 에러: {e}')
    
    def destroy_node(self):
        """노드 종료 시 정리"""
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        gstcam_publisher = GstCamPublisher()
        rclpy.spin(gstcam_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if 'gstcam_publisher' in locals():
            gstcam_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
