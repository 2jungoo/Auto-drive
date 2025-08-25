#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# =========================================================
# [SOFTWARE RENDERING FORCE]  << 반드시 최상단에 위치해야 함 >>
# OpenGL을 소프트웨어 렌더링으로 강제해 GLX/FBConfig 문제 회피
# =========================================================
import os
os.environ["QT_OPENGL"] = "software"
os.environ["QT_QUICK_BACKEND"] = "software"
os.environ["QT_XCB_FORCE_SOFTWARE_OPENGL"] = "1"
os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
try:
    try:
        from PyQt5.QtCore import QCoreApplication, Qt  # noqa
    except Exception:
        from PySide6.QtCore import QCoreApplication, Qt  # noqa
    QCoreApplication.setAttribute(Qt.AA_UseSoftwareOpenGL, True)
except Exception:
    pass
# =========================================================

import threading, time
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, LaserScan, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String, Bool
from cv_bridge import CvBridge

import cv2
import numpy as np
import math
from scipy import ndimage
from sklearn.cluster import DBSCAN


# =========================================================
# 누락 상수/파라미터 (기본값 합리화)
# =========================================================
LIDAR_CONE_MIN_DIST = 0.20        # m, 너무 가까운 노이즈 제거
LIDAR_CONE_MAX_DIST = 3.50        # m, 너무 먼 포인트 제외
LIDAR_CONE_MIN_POINTS = 3         # 클러스터 최소 포인트
LIDAR_CONE_MAX_GAP = 0.18         # DBSCAN eps (m)
CONE_PATH_SAFETY_MARGIN = 0.15    # 차선 중앙 계산시 좌/우 분리 경계(m)

STEERING_SMOOTH_STRAIGHT = 0.10   # 직선에서 조향 스무딩
STEERING_SMOOTH_CURVE = 0.20      # 코너/회피에서 더 완만하게

# =========================
# MJPEG 스트리머 (디버그 뷰)
# =========================
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
            self.send_header('Age', '0')
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
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


# =========================
# ROS2 Autoracer 노드
# =========================
sensor_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST
)

class Autoracer(Node):
    def __init__(self):
        super().__init__('Autoracer')

        # Sub / Pub
        self.img_sub = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.image_callback, sensor_qos)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, sensor_qos)

        self.cmd_pub    = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub  = self.create_publisher(Image, '/debug/image', 10)
        self.steer_pub  = self.create_publisher(Float32, '/cmd/steering', 10)
        self.throt_pub  = self.create_publisher(Float32, '/cmd/throttle', 10)
        
        self.traffic_state_pub = self.create_publisher(String, '/traffic_light_state', 10)
        self.cone_detected_pub = self.create_publisher(Bool, '/cone_mode_active', 10)

        # 상태
        self.bridge = CvBridge()
        self.image  = None
        self.lidar_ranges = None
        self.lidar_angle_min = None
        self.lidar_angle_increment = None
        self.front_distance = float('inf')
        self.started = False
        self.traffic_light_checked_once = False

        # 입력(원본) 해상도
        self.img_width  = 1280
        self.img_height = 720

        # ⚙️ 런타임/성능 튜닝 파라미터 (통합)
        self.FIXED_THROTTLE = 0.45
        self.FIXED_SPEED_MPS = 0.4
        self.CURVE_SPEED_MPS = 0.25
        self.EMER_STOP_DIST = 0.45
        self.PROC_SCALE = 0.8
        self.JPEG_QUALITY = 60
        self.DEBUG_EVERY = 3
        self.LOCK_TTL_FRAMES = 15
        self.REDETECT_MAX_PERIOD = 40
        self.USE_BOTTOM_CROP = False
        
        # 내부 처리 해상도
        self.proc_w = int(self.img_width * self.PROC_SCALE)
        self.proc_h = int(self.img_height * self.PROC_SCALE)

        # 로워 ROI(본네트) 제외
        self.bottom_crop_ratio = 5.0/22.0
        self.proc_blocked_height = int(self.proc_h * self.bottom_crop_ratio) if self.USE_BOTTOM_CROP else 0

        # --- 조향 파라미터 ---
        self.max_steering = 1.0
        self.prev_steering = 0.0
        self.steer_smooth = STEERING_SMOOTH_STRAIGHT
        self.max_steer_delta = 0.15

        # 차선 폭 추정(px, BEV 기준)
        self.lane_width_px = 420 * self.PROC_SCALE

        # BEV 변환(처리 해상도 기준)
        self.M, self.Minv = self._compute_perspective(self.proc_w, self.proc_h)

        # MJPEG 디버그
        self.preview_w, self.preview_h = 640, 360
        self.jpeg_quality = 60
        self.server = MjpegServer(port=8080)
        self.server.start()
        self.get_logger().info('=== Enhanced Autoracer | MJPEG http://0.0.0.0:8080/stream ===')

        # Lock-Follow 상태
        self.frame_idx = 0
        self.lock_rem = 0
        self.last_lane_center = None
        self.prev_left_fit = None
        self.prev_right_fit = None
        self.prev_yellow_fit = None

        # 신호등 & 라바콘 상태 (통합)
        self.traffic_light_checked = False
        self.initial_green_detected = False
        self.cone_mode_active = False
        self.lidar_cone_path = None
        self.cone_centers_lidar = [] # 디버깅용
        self.current_curvature = 0.0
        
        # 색상 마스크 파라미터 (통합)
        self.WHITE_L_MIN = 180
        self.WHITE_S_MAX = 110
        self.LAB_L_MIN = 200
        self.LAB_A_DEV = 12
        self.LAB_B_DEV = 12
        self.YELLOW_H_MIN, self.YELLOW_H_MAX = 15, 40
        self.YELLOW_S_MIN = 80
        self.YELLOW_L_MIN = 80
        self.YELLOW_PIX_MIN_WARP = 400
        self.RED_LOWER1 = np.array([0, 120, 120])
        self.RED_UPPER1 = np.array([10, 255, 255])
        self.RED_LOWER2 = np.array([170, 120, 120])
        self.RED_UPPER2 = np.array([180, 255, 255])
        self.GREEN_LOWER = np.array([40, 60, 60])
        self.GREEN_UPPER = np.array([80, 255, 255])
        self.ORANGE_LOWER = np.array([8, 150, 150])
        self.ORANGE_UPPER = np.array([25, 255, 255])
        self.MIN_GREEN_AREA = 800

        self.timer = self.create_timer(1.0/30.0, self.control_loop)

    # ---------- 콜백 ----------
    def image_callback(self, msg: CompressedImage):
        try:
            img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                return
            if (img.shape[1], img.shape[0]) != (self.img_width, self.img_height):
                img = cv2.resize(img, (self.img_width, self.img_height), interpolation=cv2.INTER_AREA)
            self.image = img
        except Exception as e:
            self.get_logger().error(f"Image cb: {e}")

    def lidar_callback(self, msg: LaserScan):
        try:
            self.lidar_ranges = np.array(msg.ranges, dtype=float)
            self.lidar_angle_min = float(msg.angle_min)
            self.lidar_angle_increment = float(msg.angle_increment)

            # 전방 거리 계산: |각도| < 10도 내 최소거리
            n = len(self.lidar_ranges)
            if n == 0 or not np.isfinite(self.lidar_angle_increment):
                self.front_distance = float('inf')
                return
            angles = self.lidar_angle_min + np.arange(n) * self.lidar_angle_increment
            mask = (np.abs(angles) < np.deg2rad(10.0))
            seg = self.lidar_ranges[mask]
            seg = seg[(seg > 0.05) & (seg < 20.0) & np.isfinite(seg)]
            self.front_distance = float(seg.min()) if seg.size else float('inf')
        except Exception as e:
            self.get_logger().error(f"Lidar cb: {e}")

    # ---------- 신호등 검출 (1회만) ----------
    def detect_traffic_light_once(self, img):
        """신호등을 1회만 검사하여 초록불 확인 -> 'GREEN_START'/'WAITING_GREEN'/'PROCEED'"""
        if self.traffic_light_checked:
            return "PROCEED"
        h, w = img.shape[:2]
        roi = img[0:h//3, w//4:3*w//4]
        if roi.size == 0:
            return "WAITING_GREEN"
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv, self.GREEN_LOWER, self.GREEN_UPPER)
        green_area = cv2.countNonZero(green_mask)
        if green_area > self.MIN_GREEN_AREA:
            self.initial_green_detected = True
            self.traffic_light_checked = True
            self.traffic_state_pub.publish(String(data="GREEN"))
            return "GREEN_START"
        self.traffic_state_pub.publish(String(data="WAIT"))
        return "WAITING_GREEN"

    # ---------- 라이다 기반 라바콘 검출 ----------
    def detect_lidar_cones(self):
        """라이다 데이터로 라바콘(장애물) 검출 및 경로 계산"""
        if self.lidar_ranges is None:
            return False, None, []
        ranges = self.lidar_ranges
        n = len(ranges)
        if n < 5:
            return False, None, []

        # 각도 배열 (가능하면 실제 각도 사용)
        if self.lidar_angle_min is not None and self.lidar_angle_increment is not None:
            angles = self.lidar_angle_min + np.arange(n) * self.lidar_angle_increment
        else:
            # fallback: 정면이 0으로 가정
            angles = np.linspace(-np.pi, np.pi, n, endpoint=False)

        valid_mask = (ranges > LIDAR_CONE_MIN_DIST) & (ranges < LIDAR_CONE_MAX_DIST) & np.isfinite(ranges)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        if valid_ranges.size < LIDAR_CONE_MIN_POINTS:
            return False, None, []

        # 좌/우 = x(좌우), 전/후 = y(전방) 로 좌표계 설정
        x = valid_ranges * np.sin(valid_angles)  # 좌(-) / 우(+)
        y = valid_ranges * np.cos(valid_angles)  # 전방(+)
        front_mask = y > 0.3
        if not np.any(front_mask):
            return False, None, []

        cone_x = x[front_mask]
        cone_y = y[front_mask]
        points = np.column_stack([cone_x, cone_y])
        if len(points) < LIDAR_CONE_MIN_POINTS:
            return False, None, []

        clustering = DBSCAN(eps=LIDAR_CONE_MAX_GAP, min_samples=LIDAR_CONE_MIN_POINTS).fit(points)
        labels = clustering.labels_

        cone_centers = []
        for label in set(labels):
            if label == -1:
                continue
            cluster_points = points[labels == label]
            center = np.mean(cluster_points, axis=0)
            cone_centers.append(center)

        if len(cone_centers) >= 2:
            cone_centers = np.array(cone_centers)
            left_cones = cone_centers[cone_centers[:, 0] < -CONE_PATH_SAFETY_MARGIN]
            right_cones = cone_centers[cone_centers[:, 0] > CONE_PATH_SAFETY_MARGIN]
            if len(left_cones) > 0 and len(right_cones) > 0:
                # 가장 가까운 좌/우 콘 선택 (전방 y가 작은 것)
                closest_left = left_cones[np.argmin(left_cones[:, 1])]
                closest_right = right_cones[np.argmin(right_cones[:, 1])]
                path_center = (closest_left + closest_right) / 2
                return True, path_center, cone_centers

        return False, None, cone_centers

    # ---------- 곡률 계산 ----------
    def calculate_curvature(self, left_fit, right_fit):
        if left_fit is None and right_fit is None:
            return 0.0
        y_eval = self.proc_h - 1
        curvatures = []
        if left_fit is not None:
            left_curverad = abs(2 * left_fit[0] * y_eval + left_fit[1])
            curvatures.append(left_curverad)
        if right_fit is not None:
            right_curverad = abs(2 * right_fit[0] * y_eval + right_fit[1])
            curvatures.append(right_curverad)
        return np.mean(curvatures) if curvatures else 0.0

    # ---------- 투영/ROI ----------
    def _compute_perspective(self, w, h):
        rb = h - self.proc_blocked_height - 10
        rt = int(rb * 0.58)
        src = np.float32([
            [w*0.05, rb],
            [w*0.40, rt],
            [w*0.60, rt],
            [w*0.95, rb],
        ])
        dst = np.float32([
            [w*0.20, h-1],
            [w*0.20, 0],
            [w*0.80, 0],
            [w*0.80, h-1],
        ])
        M    = cv2.getPerspectiveTransform(src, dst)
        Minv = cv2.getPerspectiveTransform(dst, src)
        return M, Minv

    def _roi_mask_lane_only(self, gray_or_binary):
        h, w = gray_or_binary.shape[:2]
        rb = h - self.proc_blocked_height - 10
        rt = int(rb * 0.58)
        pts = np.array([[
            [int(w*0.05), rb],
            [int(w*0.40), rt],
            [int(w*0.60), rt],
            [int(w*0.95), rb]
        ]], dtype=np.int32)
        mask = np.zeros_like(gray_or_binary)
        cv2.fillPoly(mask, pts, 255)
        return cv2.bitwise_and(gray_or_binary, mask)

    # ---------- 색 마스크 ----------
    def _white_mask(self, bgr):
        hls = cv2.cvtColor(bgr, cv2.COLOR_BGR2HLS)
        mask_hls = cv2.inRange(hls, (0, self.WHITE_L_MIN, 0), (180, 255, self.WHITE_S_MAX))
        lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
        L2, A, B = cv2.split(lab)
        mask_labL = cv2.inRange(L2, self.LAB_L_MIN, 255)
        mask_a = cv2.inRange(A, 128-self.LAB_A_DEV, 128+self.LAB_A_DEV)
        mask_b = cv2.inRange(B, 128-self.LAB_B_DEV, 128+self.LAB_B_DEV)
        mask_lab = cv2.bitwise_and(mask_labL, cv2.bitwise_and(mask_a, mask_b))
        mask = cv2.bitwise_or(mask_hls, mask_lab)
        hls_y = cv2.inRange(hls, (self.YELLOW_H_MIN, self.YELLOW_L_MIN, self.YELLOW_S_MIN),
                                     (self.YELLOW_H_MAX, 255, 255))
        mask = cv2.bitwise_and(mask, cv2.bitwise_not(hls_y))
        k5 = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k5, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k5, iterations=1)
        return self._roi_mask_lane_only(mask)

    def _yellow_mask(self, bgr):
        hls = cv2.cvtColor(bgr, cv2.COLOR_BGR2HLS)
        yellow = cv2.inRange(hls, (self.YELLOW_H_MIN, self.YELLOW_L_MIN, self.YELLOW_S_MIN),
                                     (self.YELLOW_H_MAX, 255, 255))
        k3 = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN,  k3, iterations=1)
        yellow = cv2.morphologyEx(yellow, cv2.MORPH_CLOSE, k3, iterations=1)
        return self._roi_mask_lane_only(yellow)

    # ---------- BEV ----------
    def _warp_binary(self, binary):
        return cv2.warpPerspective(binary, self.M, (self.proc_w, self.proc_h), flags=cv2.INTER_NEAREST)

    # ---------- 피팅(좌/우 흰선) ----------
    def _fit_two_white_lanes(self, bw):
        histogram = np.sum(bw[bw.shape[0]//2:,:], axis=0)
        midpoint = histogram.shape[0] // 2
        leftx_base  = int(np.argmax(histogram[:midpoint]))
        rightx_base = int(np.argmax(histogram[midpoint:])) + midpoint

        nwindows = 9
        window_height = bw.shape[0] // nwindows
        margin, minpix = 90, 60

        nonzero = bw.nonzero()
        nonzeroy = np.array(nonzero[0]); nonzerox = np.array(nonzero[1])

        leftx_current, rightx_current = leftx_base, rightx_base
        left_inds, right_inds = [], []

        for win in range(nwindows):
            wy_low  = bw.shape[0] - (win+1)*window_height
            wy_high = bw.shape[0] - win*window_height
            wl_low  = leftx_current - margin
            wl_high = leftx_current + margin
            wr_low  = rightx_current - margin
            wr_high = rightx_current + margin

            good_left  = ((nonzeroy >= wy_low) & (nonzeroy < wy_high) &
                          (nonzerox >= wl_low) & (nonzerox < wl_high)).nonzero()[0]
            good_right = ((nonzeroy >= wy_low) & (nonzeroy < wy_high) &
                          (nonzerox >= wr_low) & (nonzerox < wr_high)).nonzero()[0]

            left_inds.append(good_left); right_inds.append(good_right)

            if good_left.size  > minpix: leftx_current  = int(np.mean(nonzerox[good_left]))
            if good_right.size > minpix: rightx_current = int(np.mean(nonzerox[good_right]))

        left_inds  = np.concatenate(left_inds)  if left_inds  else np.array([])
        right_inds = np.concatenate(right_inds) if right_inds else np.array([])

        left_fit = right_fit = None
        if left_inds.size  > 400: left_fit  = np.polyfit(nonzeroy[left_inds],  nonzerox[left_inds],  2)
        if right_inds.size > 400: right_fit = np.polyfit(nonzeroy[right_inds], nonzerox[right_inds], 2)

        if left_fit  is None: left_fit  = self.prev_left_fit
        if right_fit is None: right_fit = self.prev_right_fit

        self.prev_left_fit, self.prev_right_fit = left_fit, right_fit
        return left_fit, right_fit

    def _search_around_poly(self, bw, left_fit, right_fit):
        margin = 80
        nonzero = bw.nonzero()
        nonzeroy = np.array(nonzero[0]); nonzerox = np.array(nonzero[1])
        new_left = new_right = None
        if left_fit is not None:
            left_inds = ((nonzerox > (left_fit[0]*nonzeroy**2 + left_fit[1]*nonzeroy + left_fit[2] - margin)) &
                         (nonzerox < (left_fit[0]*nonzeroy**2 + left_fit[1]*nonzeroy + left_fit[2] + margin)))
            if np.count_nonzero(left_inds) > 300:
                new_left = np.polyfit(nonzeroy[left_inds], nonzerox[left_inds], 2)
        if right_fit is not None:
            right_inds = ((nonzerox > (right_fit[0]*nonzeroy**2 + right_fit[1]*nonzeroy + right_fit[2] - margin)) &
                          (nonzerox < (right_fit[0]*nonzeroy**2 + right_fit[1]*nonzeroy + right_fit[2] + margin)))
            if np.count_nonzero(right_inds) > 300:
                new_right = np.polyfit(nonzeroy[right_inds], nonzerox[right_inds], 2)
        return new_left, new_right

    def _fit_yellow_center(self, bw):
        ys, xs = np.nonzero(bw)
        if xs.size < 300:
            return self.prev_yellow_fit, xs.size
        fit = np.polyfit(ys, xs, 2)
        self.prev_yellow_fit = fit
        return fit, xs.size

    def _lane_center_from_fits(self, left_fit, right_fit, yellow_fit=None, prefer_yellow_pixels=0):
        y_points = [self.proc_h - 1, int(self.proc_h * 0.7), int(self.proc_h * 0.5)]
        weights  = [0.5, 0.3, 0.2]
        center_points = []
        for y, _w in zip(y_points, weights):
            if (yellow_fit is not None) and (prefer_yellow_pixels >= self.YELLOW_PIX_MIN_WARP):
                x = np.polyval(yellow_fit, y)
                center_points.append(x)
            else:
                lx = np.polyval(left_fit,  y) if left_fit  is not None else None
                rx = np.polyval(right_fit, y) if right_fit is not None else None
                if lx is not None and rx is not None:
                    center_points.append((lx + rx) / 2.0)
                elif lx is not None:
                    center_points.append(lx + self.lane_width_px / 2.0)
                elif rx is not None:
                    center_points.append(rx - self.lane_width_px / 2.0)
                else:
                    center_points.append(self.proc_w / 2.0)
        weighted_center = np.average(center_points, weights=weights)
        if np.isnan(weighted_center) or np.isinf(weighted_center):
            return self.proc_w / 2.0
        return float(weighted_center)

    def calc_steering_angle(self, lane_center_bev):
        img_center = self.proc_w / 2.0
        offset = lane_center_bev - img_center
        # 간단한 편향 + 이전 곡률 기반 보정
        curve_bias = 0.0
        if self.prev_left_fit is not None:
            curve_bias = self.prev_left_fit[0] * 200
        elif self.prev_right_fit is not None:
            curve_bias = self.prev_right_fit[0] * 200
        raw_angle = (offset / (self.proc_w / 2.0)) * self.max_steering + curve_bias
        # 프레임간 변화량 제한
        steer_delta = raw_angle - self.prev_steering
        if abs(steer_delta) > self.max_steer_delta:
            raw_angle = self.prev_steering + self.max_steer_delta * np.sign(steer_delta)
        raw_angle = float(np.clip(raw_angle, -self.max_steering, self.max_steering))
        steer = self.prev_steering * (1 - self.steer_smooth) + raw_angle * self.steer_smooth
        self.prev_steering = float(steer)
        return float(steer)

    def _adjust_speed_based_on_curvature(self):
        curvature = 0.0
        if self.prev_left_fit is not None:
            curvature = self.prev_left_fit[0]
        elif self.prev_right_fit is not None:
            curvature = self.prev_right_fit[0]
        speed_factor = 1.0 - abs(curvature) * 15.0
        speed_factor = float(np.clip(speed_factor, 0.0, 1.0))
        return max(self.FIXED_SPEED_MPS * speed_factor, self.FIXED_SPEED_MPS * 0.5)

    def steering_to_normalized(self, steering_angle):
        if self.max_steering <= 1e-6: return 0.0
        return float(np.clip(steering_angle / self.max_steering, -1.0, 1.0))

    def publish_debug_image(self, debug_bgr, steering, speed_mps, src_used, cone_centers=[]):
        if (self.frame_idx % self.DEBUG_EVERY) != 0:
            return
        preview = debug_bgr.copy()
        if (debug_bgr.shape[1], debug_bgr.shape[0]) != (self.preview_w, self.preview_h):
            preview = cv2.resize(debug_bgr, (self.preview_w, self.preview_h), interpolation=cv2.INTER_AREA)

        vis = self._draw_result(preview, self.prev_left_fit, self.prev_right_fit, self.prev_yellow_fit, self.last_lane_center)

        if self.cone_mode_active and cone_centers is not None:
            for cone in cone_centers:
                cx, cy = self.convert_lidar_to_image_coords(cone)
                cv2.circle(vis, (cx, cy), 10, (0, 165, 255), -1)

        cv2.putText(vis, f"Status: {'Driving' if self.started else 'Waiting'} | Speed: {speed_mps:.2f}",
                    (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
        cv2.putText(vis, f"mode={src_used} | steer={math.degrees(steering):.1f}deg",
                    (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
        cv2.putText(vis, f"front={self.front_distance:.2f}m",
                    (10,90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)

        msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
        self.debug_pub.publish(msg)
        frame_store.update_bgr(vis, quality=self.JPEG_QUALITY)

    def _draw_result(self, base_bgr, left_fit, right_fit, yellow_fit, lane_center_bev):
        h, w = base_bgr.shape[:2]
        ploty = np.linspace(0, h-1, h)
        warp_canvas = np.zeros((h, w, 3), dtype=np.uint8)

        if left_fit is not None:
            leftx = np.polyval(left_fit, ploty)
            pts = np.int32(np.transpose(np.vstack([leftx, ploty])))
            cv2.polylines(warp_canvas, [pts], False, (255,255,255), 6)
        if right_fit is not None:
            rightx = np.polyval(right_fit, ploty)
            pts = np.int32(np.transpose(np.vstack([rightx, ploty])))
            cv2.polylines(warp_canvas, [pts], False, (255,255,255), 6)
        if yellow_fit is not None:
            yx = np.polyval(yellow_fit, ploty)
            pts = np.int32(np.transpose(np.vstack([yx, ploty])))
            cv2.polylines(warp_canvas, [pts], False, (0,255,255), 4)

        yb = h - 150
        if lane_center_bev is None:
            lane_center_bev = w / 2
        cv2.line(warp_canvas, (int(w/2), yb), (int(w/2), yb-40), (0,255,0), 2)
        cv2.line(warp_canvas, (int(lane_center_bev), yb), (int(lane_center_bev), yb-40), (0,255,255), 2)

        unwarp = cv2.warpPerspective(warp_canvas, self.Minv, (w, h))
        out = cv2.addWeighted(base_bgr, 1.0, unwarp, 0.9, 0)
        return out

    def convert_lidar_to_image_coords(self, lidar_point):
        # LiDAR 좌표를 BEV 이미지 좌표로 단순 변환 (시각화용)
        x_lateral, y_forward = float(lidar_point[0]), float(lidar_point[1])
        x_bev = x_lateral * 100.0 + self.proc_w / 2.0
        y_bev = -y_forward * 100.0 + self.proc_h
        return int(np.clip(x_bev, 0, self.proc_w - 1)), int(np.clip(y_bev, 0, self.proc_h - 1))

    # ---------- 메인 루프 ----------
    def control_loop(self):
        if self.image is None:
            return
        try:
            self.frame_idx += 1

            # 신호등 체크 (1회)
            tl_state = self.detect_traffic_light_once(self.image)

            steering = 0.0
            speed_mps = 0.0
            src_used = "WAIT"
            cones = []

            if not self.started:
                self.get_logger().info(f"Status: WAITING | Signal: {tl_state}")
                if tl_state in ("GREEN_START", "PROCEED"):
                    self.get_logger().info("🟢 초록불 감지! 출발합니다.")
                    self.started = True
                    self.traffic_light_checked_once = True
            else:
                self.get_logger().info("Status: DRIVING")

                cone_detected, path_center_lidar, cones = self.detect_lidar_cones()
                self.cone_detected_pub.publish(Bool(data=cone_detected))
                self.cone_mode_active = cone_detected

                if self.cone_mode_active:
                    self.get_logger().info("Status: CONE AVOIDANCE")
                    self.steer_smooth = STEERING_SMOOTH_CURVE
                    if path_center_lidar is not None:
                        path_center_pixels = self.convert_lidar_to_image_coords(path_center_lidar)
                        steering = self.calc_steering_angle(path_center_pixels[0])  # x 좌표 기준
                        speed_mps = self.CURVE_SPEED_MPS
                    else:
                        steering = self.prev_steering
                        speed_mps = self.CURVE_SPEED_MPS
                    src_used = "CONE_AVOID"
                else:
                    self.get_logger().info("Status: LANE FOLLOWING")
                    self.steer_smooth = STEERING_SMOOTH_STRAIGHT

                    base = self.image if self.PROC_SCALE == 1.0 else cv2.resize(
                        self.image, (self.proc_w, self.proc_h), interpolation=cv2.INTER_AREA)

                    force_redetect = (self.frame_idx % self.REDETECT_MAX_PERIOD) == 0

                    if (self.lock_rem > 0) and (self.last_lane_center is not None) and (not force_redetect):
                        lane_center_bev = float(self.last_lane_center)
                        src_used = "LOCK"
                        self.lock_rem -= 1
                    else:
                        white_mask_img  = self._white_mask(base)
                        yellow_mask_img = self._yellow_mask(base)
                        white_warp  = self._warp_binary(white_mask_img)
                        yellow_warp = self._warp_binary(yellow_mask_img)

                        lf, rf = None, None
                        if (self.prev_left_fit is not None) or (self.prev_right_fit is not None):
                            lf, rf = self._search_around_poly(white_warp, self.prev_left_fit, self.prev_right_fit)
                        if lf is None and rf is None:
                            lf, rf = self._fit_two_white_lanes(white_warp)

                        yfit, ypix = self._fit_yellow_center(yellow_warp)

                        lane_center_bev = self._lane_center_from_fits(lf, rf, yfit, ypix)

                        if yfit is not None and ypix >= self.YELLOW_PIX_MIN_WARP:
                            src_used = "YELLOW"
                        elif lf is not None and rf is not None:
                            src_used = "WHITE_MID"
                        elif lf is not None:
                            src_used = "LEFT_ONLY"
                        elif rf is not None:
                            src_used = "RIGHT_ONLY"
                        else:
                            src_used = "FALLBACK"

                        # BUGFIX: yf -> yfit
                        self.prev_left_fit, self.prev_right_fit, self.prev_yellow_fit = lf, rf, yfit
                        self.last_lane_center = float(lane_center_bev)

                        good_white  = (lf is not None and rf is not None)
                        good_yellow = (ypix >= self.YELLOW_PIX_MIN_WARP)
                        if good_white or good_yellow:
                            self.lock_rem = self.LOCK_TTL_FRAMES
                        else:
                            self.lock_rem = 0

                    steering   = self.calc_steering_angle(lane_center_bev)
                    speed_mps  = self._adjust_speed_based_on_curvature()

            # 안전 정지
            if self.front_distance < self.EMER_STOP_DIST:
                steering = 0.0
                speed_mps = 0.0
                src_used = f"{src_used}|EMER_STOP"

            steer_norm = self.steering_to_normalized(steering)
            throttle = float(self.FIXED_THROTTLE)

            cmd = Twist(); cmd.linear.x = speed_mps; cmd.angular.z = steering
            self.cmd_pub.publish(cmd)
            self.steer_pub.publish(Float32(data=steer_norm))
            self.throt_pub.publish(Float32(data=throttle))

            # 디버그 출력
            if (self.frame_idx % self.DEBUG_EVERY) == 0:
                vis = self.image.copy() if self.PROC_SCALE == 1.0 else cv2.resize(
                    self.image, (self.proc_w, self.proc_h), interpolation=cv2.INTER_AREA)
                if self.started:
                    vis = self._draw_result(vis, self.prev_left_fit, self.prev_right_fit, self.prev_yellow_fit, self.last_lane_center)
                if self.cone_mode_active and cones:
                    for cone in cones:
                        cx, cy = self.convert_lidar_to_image_coords(cone)
                        cv2.circle(vis, (cx, cy), 10, (0, 165, 255), -1)
                self.publish_debug_image(vis, steering, speed_mps, src_used, cones)

        except Exception as e:
            self.get_logger().error(f"Control err: {e}")
            cmd = Twist()
            self.cmd_pub.publish(cmd)

    def start(self):
        self.timer = self.create_timer(1.0/30.0, self.control_loop)

# =========================
# main
# =========================
def main(args=None):
    rclpy.init(args=args)
    node = Autoracer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt로 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
