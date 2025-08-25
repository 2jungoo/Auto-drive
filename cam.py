#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2, numpy as np

class CompressedCameraPublisher(Node):
    def __init__(self):
        super().__init__('compressed_camera_publisher')
        self.declare_parameter('src', 'csi')   # 'csi' or 'usb'
        self.declare_parameter('device', '/dev/video0')

        src = self.get_parameter('src').get_parameter_value().string_value
        dev = self.get_parameter('device').get_parameter_value().string_value

        if src == 'csi':
            gst = (
                "nvarguscamerasrc ! "
                "video/x-raw(memory:NVMM), width=640, height=480, framerate=30/1 ! "
                "nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! "
                "video/x-raw, format=BGR ! appsink drop=true max-buffers=1 sync=false"
            )
            self.get_logger().info('Using CSI camera pipeline')
            self.cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
        else:
            gst = (
                f"v4l2src device={dev} ! video/x-raw, width=640, height=480, framerate=30/1 ! "
                "videoconvert ! video/x-raw, format=BGR ! appsink drop=true max-buffers=1 sync=false"
            )
            self.get_logger().info(f'Using USB camera pipeline: {dev}')
            self.cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera pipeline')
        self.pub = self.create_publisher(CompressedImage, '/image_raw/compressed', 1)
        self.timer = self.create_timer(1.0/30.0, self.tick)

    def tick(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn('Failed to capture frame')
            return
        ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            self.get_logger().warn('imencode failed')
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        msg.data = np.asarray(buf).tobytes()
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            if hasattr(self, 'cap'): self.cap.release()
        finally:
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CompressedCameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
