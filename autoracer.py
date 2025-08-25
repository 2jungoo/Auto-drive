#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
import subprocess
import threading
import time
import signal
import os

class GstCamPublisher(Node):
    def __init__(self):
        super().__init__('gstcam_pub_node')
        
        # CompressedImage 퍼블리셔 생성
        self.image_publisher = self.create_publisher(
            CompressedImage,
            '/image_raw/compressed',
            10
        )
        
        # GStreamer 프로세스
        self.gst_process = None
        self.running = False
        
        # GStreamer로 JPEG 스트림 생성
        self.init_gstreamer_process()
        
        if self.gst_process is not None:
            # 스레드로 이미지 읽기 시작
            self.running = True
            self.capture_thread = threading.Thread(target=self.capture_images, daemon=True)
            self.capture_thread.start()
            self.get_logger().info('✅ GStreamer 프로세스 시작됨!')
        else:
            self.get_logger().error('❌ GStreamer 프로세스 시작 실패')
    
    def init_gstreamer_process(self):
        """GStreamer 프로세스로 JPEG 스트림 생성"""
        try:
            # GStreamer 파이프라인 - JPEG으로 stdout에 출력
            gst_pipeline = [
                'gst-launch-1.0',
                'nvarguscamerasrc',
                '!', 'video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1',
                '!', 'nvvidconv',
                '!', 'video/x-raw, format=BGRx',
                '!', 'videoconvert',
                '!', 'video/x-raw, format=BGR',
                '!', 'jpegenc', 'quality=90',
                '!', 'multifilesink', 'location=/tmp/camera_frame.jpg'
            ]
            
            self.get_logger().info('🔍 GStreamer 프로세스 시작 중...')
            
            # GStreamer 프로세스 실행
            self.gst_process = subprocess.Popen(
                gst_pipeline,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # 프로세스 그룹 생성
            )
            
            # 2초 대기 후 프로세스 상태 확인
            time.sleep(2)
            if self.gst_process.poll() is None:
                self.get_logger().info('✅ GStreamer 프로세스 정상 실행 중')
            else:
                stderr_output = self.gst_process.stderr.read().decode()
                self.get_logger().error(f'❌ GStreamer 프로세스 종료됨: {stderr_output}')
                self.gst_process = None
                
        except Exception as e:
            self.get_logger().error(f'❌ GStreamer 프로세스 시작 에러: {e}')
            self.gst_process = None
    
    def capture_images(self):
        """이미지 캡처 및 발행 스레드"""
        frame_count = 0
        
        while self.running and rclpy.ok():
            try:
                # 파일이 생성될 때까지 대기
                if os.path.exists('/tmp/camera_frame.jpg'):
                    # 파일 읽기
                    with open('/tmp/camera_frame.jpg', 'rb') as f:
                        jpeg_data = f.read()
                    
                    if len(jpeg_data) > 0:
                        # CompressedImage 메시지 생성
                        msg = CompressedImage()
                        msg.header = Header()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = 'camera_frame'
                        msg.format = 'jpeg'
                        msg.data = jpeg_data
                        
                        # ROS2 토픽으로 발행
                        self.image_publisher.publish(msg)
                        
                        frame_count += 1
                        if frame_count % 100 == 0:
                            self.get_logger().info(f'📸 {frame_count}번째 프레임 발행됨')
                
                time.sleep(1.0/30.0)  # 30 FPS
                
            except Exception as e:
                self.get_logger().error(f'이미지 캡처 에러: {e}')
                time.sleep(1)
    
    def destroy_node(self):
        """노드 종료 시 정리"""
        self.running = False
        
        if self.gst_process is not None:
            try:
                # 프로세스 그룹 전체 종료
                os.killpg(os.getpgid(self.gst_process.pid), signal.SIGTERM)
                self.gst_process.wait(timeout=5)
            except:
                pass
        
        # 임시 파일 삭제
        if os.path.exists('/tmp/camera_frame.jpg'):
            os.remove('/tmp/camera_frame.jpg')
            
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
