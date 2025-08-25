#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Autoracer with CSI Camera (ROS2, gscam)
"""

# ================================
# 1. 기본 라이브러리 임포트
# ================================
import threading, time, math
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, LaserScan, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String, Bool
from cv_bridge import CvBridge

import cv2
import numpy as np
from sklearn.cluster import DBSCAN


# ================================
# 2. MJPEG 스트리머 (디버그 뷰)
# ================================
class FrameStore:
    def __init__(self):
        self.lock = threading.Lock()
        self.jpg = None

    def update_bgr(self, bgr, quality=80):
        ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if ok:
            with self.lock:
                self.jpg = buf.tobytes()

    def get_jpg(self):
        with self.lock:
            return self.jpg

frame_store = FrameStore()

class MjpegHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.startswith('/stream'):
            self.send_response(200)
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    jpg = frame_store.get_jpg()
                    if jpg is not None:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(b'Content-Length: ' + str(len(jpg)).encode() + b'\r\n\r\n')
                        self.wfile.write(jpg)
                        self.wfile.write(b'\r\n')
                    time.sleep(0.01)
            except Exception:
                pass
        else:
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            html = (b"<html><body><h3>MJPEG Stream</h3>"
                    b"<img src='/stream' />"
                    b"</body></html>")
            self.wfile.write(html)

class MjpegServer(threading.Thread):
    def __init__(self, host='0.0.0.0', port=8080):
        super().__init__(daemon=True)
        self.httpd = HTTPServer((host, port), MjpegHandler)
    def run(self):
        self.httpd.serve_forever()


# ================================
# 3. Autoracer ROS2 노드
# ================================
class Autoracer(Node):
    def __init__(self):
        super().__init__('Autoracer')

        # --- 구독: CSI 카메라 republish 토픽 ---
        self.img_sub = self.create_subscription(
            CompressedImage, '/image_raw/compressed',
            self.image_callback, qos_profile_sensor_data)

        # --- 퍼블리셔 ---
        self.cmd_pub   = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/debug/image', 10)

        # 상태
        self.bridge = CvBridge()
        self.image = None
        self.img_width, self.img_height = 1280, 720
        self.JPEG_QUALITY = 60

        # MJPEG 서버 시작
        self.server = MjpegServer(port=8080)
        self.server.start()
        self.get_logger().info("MJPEG Stream: http://0.0.0.0:8080/stream")

        # 제어 루프
        self.timer = self.create_timer(1.0/30.0, self.control_loop)

    # ---------- 콜백 ----------
    def image_callback(self, msg: CompressedImage):
        try:
            img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if img is None: return
            if (img.shape[1], img.shape[0]) != (self.img_width, self.img_height):
                img = cv2.resize(img, (self.img_width, self.img_height), interpolation=cv2.INTER_AREA)
            self.image = img

            # ✅ 프레임을 스트림 서버로 전달
            frame_store.update_bgr(self.image, quality=self.JPEG_QUALITY)
        except Exception as e:
            self.get_logger().error(f"Image cb: {e}")

    # ---------- 제어 루프 ----------
    def control_loop(self):
        if self.image is None:
            return
        # 단순히 프리뷰를 퍼블리시
        msg = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
        self.debug_pub.publish(msg)


# ================================
# 4. main 함수
# ================================
def main(args=None):
    rclpy.init(args=args)
    node = Autoracer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ================================
# 5. 실행 시작점
# ================================
if __name__ == '__main__':
    main()

