#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
# ROS2 rclpy 기반 자율주행 노드 (시뮬레이션 v2 → 실제 RC카용 변환)
#   • 신호등: 빨강/주황 → 정지, 초록/청록 → 출발
#   • 차선 인식 + PID 제어 + 실선 준수 (기본 로직 유지)
#   • LaserScan 시각화(선택)
# ------------------------------------------------------------
# 토픽 (필요시 변경):
#   - Sub: /image_raw/compressed (sensor_msgs/CompressedImage)
#   - Sub: /scan               (sensor_msgs/LaserScan)
#   - Pub: /xycar_motor        (xycar_msgs/XycarMotor)
# ------------------------------------------------------------
# 의존 패키지:
#   pip install numpy opencv-python
#   apt install ros-<distro>-xycar-msgs (or custom)
# ============================================================

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import numpy as np
import cv2
import time
import math

from sensor_msgs.msg import CompressedImage, LaserScan
from xycar_msgs.msg import XycarMotor

# ---------------- QoS 설정 ----------------
qos_profile = QoSProfile(depth=10)
qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

# ============================================================
# 메인 클래스
# ============================================================
class Autoracer(Node):
    def __init__(self):
        super().__init__('autoracer_node')

        # ---- 퍼블리셔 ----
        self.motor_pub = self.create_publisher(XycarMotor, '/xycar_motor', 10)

        # ---- 서브스크립션 ----
        self.img_sub = self.create_subscription(
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

        # ---- 타이머(50ms) ----
        self.timer = self.create_timer(0.05, self.control_loop)

        # ---- 상태 변수 ----
        self.image = np.empty(shape=[0])
        self.ranges = None
        self.started = False
        self.Fix_Speed = 12

        # PID
        self.prev_error = 0.0
        self.integral = 0.0

        self.solid_line_violation_count = 0

        # 신호등 타임아웃
        self.traffic_light_attempts = 0
        self.max_attempts = 100

        # 라이다 플롯(원하면 사용)
        self.use_lidar_plot = False
        if self.use_lidar_plot:
            import matplotlib.pyplot as plt
            self.plt = plt
            self.fig, self.ax = plt.subplots(figsize=(8, 8))
            self.ax.set_xlim(-120, 120)
            self.ax.set_ylim(-120, 120)
            self.ax.set_aspect('equal')
            (self.lidar_points,) = self.ax.plot([], [], 'bo')

        self.get_logger().info("Autoracer node initialized.")

    # --------------------------------------------------------
    # 콜백: 카메라
    # --------------------------------------------------------
    def image_callback(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is not None:
            self.image = img

    # --------------------------------------------------------
    # 콜백: 라이다
    # --------------------------------------------------------
    def lidar_callback(self, msg: LaserScan):
        self.ranges = np.array(msg.ranges[:360], dtype=np.float32)

        # (디버그) 중앙 10개 범위 출력
        # total_points = len(msg.ranges)
        # center = total_points // 2
        # half_width = 10 // 2
        # start_idx = max(0, center - half_width)
        # end_idx = min(total_points, center + half_width)
        # center_ranges = msg.ranges[start_idx:end_idx]
        # formatted = [f"{r:.3f}" for r in center_ranges]
        # self.get_logger().info(f"Center 10 ranges: {formatted}")

    # --------------------------------------------------------
    # 메인 제어 루프(타이머)
    # --------------------------------------------------------
    def control_loop(self):
        if self.image.size == 0:
            return

        # 원본 출력
        cv2.imshow("Original", self.image)

        # 1) 신호등 판단 단계
        if not self.started:
            status = self.detect_traffic_light(self.image)

            if status == "green":
                self.started = True
                self.traffic_light_attempts = 0
                self.get_logger().info("🟢 초록/청록 불 감지! 출발합니다!")

            elif status == "red":
                # 빨강/주황 → 정지
                self.traffic_light_attempts = 0
                self.drive(0.0, 0.0)
                cv2.waitKey(1)
                return

            else:  # unknown
                self.traffic_light_attempts += 1
                if self.traffic_light_attempts > self.max_attempts:
                    # 필요하다면 강제 출발/계속 대기 선택 가능
                    self.started = True
                    self.get_logger().warn("⚠️ 신호등 인식 실패(타임아웃) → 안전 출발")
                else:
                    self.drive(0.0, 0.0)
                    cv2.waitKey(1)
                    return

        # 2) 차선 추종 (PID)
        try:
            error = self.detect_lane_error(self.image)
            steer = self.compute_steering_pid(error)

            # 속도 조절
            if abs(steer) > 35 or self.solid_line_violation_count > 0:
                speed = self.Fix_Speed - 4
            else:
                speed = self.Fix_Speed

            self.drive(steer, speed)
            self.get_logger().info(f"[주행] err:{error:.1f}, steer:{steer:.1f}, speed:{speed}")

        except Exception as e:
            self.get_logger().error(f"주행 중 오류: {e}")
            self.drive(0.0, 5)

        # 3) 라이다 시각화(선택)
        if self.use_lidar_plot and self.ranges is not None:
            angles = np.linspace(0, 2 * np.pi, len(self.ranges)) + np.pi / 2
            x = self.ranges * np.cos(angles)
            y = self.ranges * np.sin(angles)
            self.lidar_points.set_data(x, y)
            self.fig.canvas.draw_idle()
            self.plt.pause(0.001)

        cv2.waitKey(1)

    # --------------------------------------------------------
    # 신호등 인식
    # --------------------------------------------------------
    def detect_traffic_light(self, img):
        """
        반환: 'green' | 'red' | 'unknown'
        """
        h, w = img.shape[:2]
        roi = img[0:int(h * 0.4), int(w * 0.3):int(w * 0.7)]
        if roi.size == 0:
            return "unknown"

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # GO: H 40~85 (초록/청록)
        mask_go = cv2.inRange(hsv, (40, 40, 40), (85, 255, 255))
        # STOP: 빨강(0~12, 170~180) + 주황(12~32)
        mask_stop = (cv2.inRange(hsv, (0, 120, 50), (12, 255, 255)) |
                     cv2.inRange(hsv, (12,120,50), (32,255,255))   |
                     cv2.inRange(hsv, (170,120,50),(180,255,255)))

        kernel = np.ones((5,5), np.uint8)
        mask_go   = cv2.morphologyEx(mask_go,   cv2.MORPH_OPEN, kernel)
        mask_stop = cv2.morphologyEx(mask_stop, cv2.MORPH_OPEN, kernel)

        go_pct   = np.count_nonzero(mask_go)   / mask_go.size   * 100
        stop_pct = np.count_nonzero(mask_stop) / mask_stop.size * 100

        # 디버깅 창
        cv2.imshow("TL_GO",   mask_go)
        cv2.imshow("TL_STOP", mask_stop)

        if go_pct >= 0.7 and stop_pct < 0.4:
            return "green"
        elif stop_pct >= 0.4:
            return "red"
        else:
            return "unknown"

    # --------------------------------------------------------
    # 차선 인식 (기존 로직 이식)
    # --------------------------------------------------------
    def detect_lane_error(self, img):
        height, width = img.shape[:2]
        roi_height = 120
        roi = img[height - roi_height:height, :]

        blurred = cv2.GaussianBlur(roi, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        white_lower = np.array([0, 0, 170])
        white_upper = np.array([180, 40, 255])
        white_mask = cv2.inRange(hsv, white_lower, white_upper)

        yellow_lower = np.array([10, 100, 100])
        yellow_upper = np.array([40, 255, 255])
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

        kernel = np.ones((3, 3), np.uint8)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)

        combined_mask = cv2.bitwise_or(white_mask, yellow_mask)

        mid_point = width // 2
        left_region  = combined_mask[:, :mid_point]
        right_region = combined_mask[:, mid_point:]

        lane_info = self.analyze_lane_structure(left_region, right_region, white_mask, yellow_mask, width)
        error, _ = self.calculate_driving_path(lane_info, width, mid_point)

        # 디버깅 이미지 생성
        _ = self.create_debug_image(roi, lane_info, error, width, roi_height)

        return error

    def analyze_lane_structure(self, left_region, right_region, white_mask, yellow_mask, width):
        lane_info = {
            'left_line': None,
            'right_line': None,
            'left_type': None,
            'right_type': None,
            'lane_width': None
        }

        # 왼쪽 차선
        left_contours, _ = cv2.findContours(left_region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if left_contours:
            largest_left = max(left_contours, key=cv2.contourArea)
            if cv2.contourArea(largest_left) > 50:
                M = cv2.moments(largest_left)
                if M['m00'] != 0:
                    left_x = int(M['m10'] / M['m00'])
                    lane_info['left_line'] = left_x
                    lane_info['left_type'] = self.determine_line_type(left_region, left_x)

        # 오른쪽 차선
        right_contours, _ = cv2.findContours(right_region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if right_contours:
            largest_right = max(right_contours, key=cv2.contourArea)
            if cv2.contourArea(largest_right) > 50:
                M = cv2.moments(largest_right)
                if M['m00'] != 0:
                    right_x = int(M['m10'] / M['m00']) + width // 2
                    lane_info['right_line'] = right_x
                    lane_info['right_type'] = self.determine_line_type(right_region, right_x - width // 2)

        if lane_info['left_line'] and lane_info['right_line']:
            lane_info['lane_width'] = lane_info['right_line'] - lane_info['left_line']

        return lane_info

    def determine_line_type(self, mask, center_x):
        if center_x is None or center_x < 0 or center_x >= mask.shape[1]:
            return None
        vertical_line = mask[:, max(0, center_x - 2):min(mask.shape[1], center_x + 3)]
        non_zero_ratio = np.count_nonzero(vertical_line) / vertical_line.size
        return 'solid' if non_zero_ratio > 0.7 else 'dashed'

    def calculate_driving_path(self, lane_info, width, mid_point):
        # (기본 버전: 실선 위반 로직 간소화, 필요시 확장)
        error = 0
        if lane_info['left_line'] and lane_info['right_line']:
            lane_center = (lane_info['left_line'] + lane_info['right_line']) // 2
            error = lane_center - mid_point
        return error, "normal"

    def create_debug_image(self, roi, lane_info, error, width, roi_height):
        dbg = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
        mid_point = width // 2

        if lane_info['left_line']:
            color = (255, 0, 0) if lane_info['left_type'] == 'solid' else (255, 100, 100)
            cv2.line(dbg, (lane_info['left_line'], 0), (lane_info['left_line'], roi_height), color, 3)
            cv2.putText(dbg, f"L:{lane_info['left_type']}",
                        (lane_info['left_line'] - 30, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        if lane_info['right_line']:
            color = (255, 255, 0) if lane_info['right_type'] == 'solid' else (255, 255, 100)
            cv2.line(dbg, (lane_info['right_line'], 0), (lane_info['right_line'], roi_height), color, 3)
            cv2.putText(dbg, f"R:{lane_info['right_type']}",
                        (lane_info['right_line'] - 30, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        target_x = mid_point + error
        cv2.line(dbg, (target_x, 0), (target_x, roi_height), (0, 255, 0), 3)
        cv2.line(dbg, (mid_point, 0), (mid_point, roi_height), (0, 0, 255), 2)

        cv2.putText(dbg, f"Error: {error:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(dbg, f"Violations: {self.solid_line_violation_count}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        cv2.imshow("Lane Analysis Debug", dbg)
        return dbg

    # --------------------------------------------------------
    # PID 제어 계산
    # --------------------------------------------------------
    def compute_steering_pid(self, error):
        Kp, Ki, Kd = 0.7, 0.01, 0.3
        self.integral += error
        self.integral = max(-100, min(100, self.integral))
        derivative = error - self.prev_error
        self.prev_error = error

        steer = Kp * error + Ki * self.integral + Kd * derivative
        steer = max(-50, min(50, steer))
        return steer

    # --------------------------------------------------------
    # 모터 제어
    # --------------------------------------------------------
    def drive(self, angle, speed):
        motor_msg = XycarMotor()
        motor_msg.angle = float(angle)
        motor_msg.speed = float(speed)
        self.motor_pub.publish(motor_msg)

# ============================================================
# 메인
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = Autoracer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        if node.use_lidar_plot:
            node.plt.close('all')

if __name__ == '__main__':
    main()
