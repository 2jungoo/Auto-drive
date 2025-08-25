def timer_callback(self):
    ret, frame = self.cap.read()
    if not ret:
        self.get_logger().warn("Failed to capture frame")
        return

    # JPEG 인코딩
    ret, jpeg_data = cv2.imencode('.jpg', frame)
    if not ret:
        self.get_logger().warn("Failed to encode frame")
        return

    msg = CompressedImage()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.format = "jpeg"
    msg.data = jpeg_data.tobytes()
    self.publisher.publish(msg)
