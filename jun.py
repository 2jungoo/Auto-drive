#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading, time
from http.server import BaseHTTPRequestHandler, HTTPServer
import warnings

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

# NumPy 경고 억제
warnings.filterwarnings("ignore", category=UserWarning, module="numpy")
warnings.filterwarnings("ignore", message=".*smallest_subnormal.*")

# sklearn 대신 간단한 클러스터링 구현
try:
    from sklearn.cluster import DBSCAN
    USE_SKLEARN = True
except ImportError:
    USE_SKLEARN = False
    print("sklearn not available, using simple clustering")


# =========================
# ⚙️ 런타임/성능 튜닝 파라미터
# =========================
FIXED_THROTTLE     = 0.45      # [0.0~1.0] 참고값: 0.40~0.55
FIXED_SPEED_MPS    = 0.4       # [m/s]
CURVE_SPEED_MPS    = 0.25      # [m/s] 곡선 구간 속도
EMER_STOP_DIST     = 0.45      # [m] 전방 아주 가까우면 즉시 정지
USE_BOTTOM_CROP    = False      # 본네트 영역 제외(카메라 각도 좋으면 False 권장)

# 🔒 Lock-Follow & 디버그 스로틀링
LOCK_TTL_FRAMES      = 15    # 잠금 유지 프레임 수(≈0.5s @30Hz). 커브 많으면 8~10
REDETECT_MAX_PERIOD = 40    # 이 주기마다 최소 1번은 풀 재검출
DEBUG_EVERY         = 3     # 디버그/스트림 전송 간격(프레임). 3~5 권장
JPEG_QUALITY        = 60    # MJPEG 품질(낮출수록 CPU↓)

# (선택) 내부 처리 해상도 축소: 1.0=원본, 0.75 권장(지연↓)
PROC_SCALE          = 0.8

# =========================
# 신호등 & 라바콘 관련 파라미터
# =========================
# 신호등 상태 (1회만 검사)
TRAFFIC_LIGHT_CHECK_FRAMES = 90  # 초기 3초간만 신호등 검사
MIN_GREEN_AREA = 800          # 초록불 최소 픽셀 영역

# 라이다 기반 라바콘 검출
LIDAR_CONE_MIN_DIST = 0.3     # 최소 거리 (m)
LIDAR_CONE_MAX_DIST = 3.5     # 최대 거리 (m)
LIDAR_CONE_MIN_POINTS = 3     # 클러스터 최소 포인트
LIDAR_CONE_MAX_GAP = 0.15     # 연속 포인트 최대 간격 (m)
CONE_PATH_SAFETY_MARGIN = 0.4 # 라바콘 경로 안전 마진 (m)

# 곡선 감지 파라미터
CURVE_THRESHOLD = 0.3         # 곡률 임계값
STEERING_SMOOTH_CURVE = 0.15  # 곡선에서 조향 스무딩
STEERING_SMOOTH_STRAIGHT = 0.08 # 직선에서 조향 스무딩

# HSV 색상 범위 (비전 기반 보조 검출용)
RED_LOWER1 = np.array([0, 120, 120])
RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([170, 120, 120])
RED_UPPER2 = np.array([180, 255, 255])
GREEN_LOWER = np.array([40, 60, 60])
GREEN_UPPER = np.array([80, 255, 255])

# 주황색 라바콘 (HSV) - 보조 검출용
ORANGE_LOWER = np.array([8, 150, 150])
ORANGE_UPPER = np.array([25, 255, 255])

# =========================
# 색상 마스크 튜닝(현장 조도에 맞게 소폭 조정)
# =========================
# 흰 실선: HLS(밝고 저채도) + LAB(고휘도 중성색) 결합
WHITE_L_MIN = 180
WHITE_S_MAX = 110
LAB_L_MIN   = 200
LAB_A_DEV   = 12
LAB_B_DEV   = 12

# 노란 점선(HLS)
YELLOW_H_MIN, YELLOW_H_MAX = 15, 40
YELLOW_S_MIN = 80
YELLOW_L_MIN = 80

# 노란선 신뢰 픽셀 임계 (BEV에서)
YELLOW_PIX_MIN_WARP = 400


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
        
        # 신호등 & 라바콘 상태 퍼블리셔
        self.traffic_state_pub = self.create_publisher(String, '/traffic_light_state', 10)
        self.cone_detected_pub = self.create_publisher(Bool, '/cone_mode_active', 10)

        # 상태
        self.bridge = CvBridge()
        self.image  = None
        self.lidar_ranges = None
        self.front_distance = float('inf')

        # 입력(원본) 해상도
        self.img_width  = 1280
        self.img_height = 720

        # 내부 처리 해상도
        self.proc_w = int(self.img_width  * PROC_SCALE)
        self.proc_h = int(self.img_height * PROC_SCALE)

        # 로워 ROI(본네트) 제외
        self.bottom_crop_ratio     = 5.0/22.0
        self.proc_blocked_height   = int(self.proc_h * self.bottom_crop_ratio) if USE_BOTTOM_CROP else 0

        # --- 조향 파라미터 (수정) ---
        self.max_steering = 1.0  # 조향 최대각을 늘려 급커브에 대응
        self.prev_steering = 0.0
        self.steer_smooth  = STEERING_SMOOTH_STRAIGHT  # 동적으로 변경

        # 차선 폭 추정(px, BEV 기준)
        self.lane_width_px = 420 * PROC_SCALE

        # BEV 변환(처리 해상도 기준)
        self.M, self.Minv = self._compute_perspective(self.proc_w, self.proc_h)

        # MJPEG 디버그
        self.preview_w, self.preview_h = 640, 360
        self.jpeg_quality = JPEG_QUALITY
        self.server = MjpegServer(port=8080)
        self.server.start()
        self.get_logger().info('=== Enhanced Autoracer | MJPEG http://0.0.0.0:8080/stream ===')

        # Lock-Follow 상태
        self.frame_idx = 0
        self.lock_rem  = 0
        self.last_lane_center = None
        self.prev_left_fit  = None
        self.prev_right_fit = None
        self.prev_yellow_fit= None

        # 신호등 & 라바콘 상태
        self.traffic_light_checked = False
        self.initial_green_detected = False
        self.cone_mode_active = False
        self.lidar_cone_path = None
        self.current_curvature = 0.0
        
        self.started = True
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
            # 안전한 방식으로 ranges 처리
            ranges_list = list(msg.ranges)
            self.lidar_ranges = np.array(ranges_list, dtype=np.float64)
            
            # 무한값과 NaN을 안전한 값으로 대체
            self.lidar_ranges = np.nan_to_num(self.lidar_ranges, 
                                            nan=0.0, 
                                            posinf=10.0, 
                                            neginf=0.0)
            
            # 전방 거리 계산 (중앙 ±15도)
            total_ranges = len(self.lidar_ranges)
            if total_ranges > 0:
                c = total_ranges // 2
                start_idx = max(0, c - 15)
                end_idx = min(total_ranges, c + 15)
                
                front_seg = self.lidar_ranges[start_idx:end_idx]
                # 유효한 거리만 선택 (0.1m ~ 10m)
                valid_seg = front_seg[(front_seg > 0.1) & (front_seg < 10.0)]
                
                if len(valid_seg) > 0:
                    self.front_distance = float(np.min(valid_seg))
                else:
                    self.front_distance = float('inf')
            else:
                self.front_distance = float('inf')
                
        except Exception as e:
            self.get_logger().error(f"Lidar cb error: {e}")
            self.front_distance = float('inf')

    # ---------- 신호등 검출 (1회만) ----------
    def detect_traffic_light_once(self, img):
        """신호등을 1회만 검사하여 초록불 확인"""
        if self.traffic_light_checked:
            return "PROCEED"  # 이미 검사 완료
            
        # 신호등이 있을 것으로 예상되는 상단 영역만 검사
        h, w = img.shape[:2]
        roi = img[0:h//3, w//4:3*w//4]  # 상단 1/3, 가운데 절반 영역
        
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # 초록불 검출
        green_mask = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)
        
        # 노이즈 제거
        kernel = np.ones((5,5), np.uint8)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        
        # 면적으로 판단
        green_area = cv2.countNonZero(green_mask)
        
        if green_area > MIN_GREEN_AREA:
            self.initial_green_detected = True
            self.traffic_light_checked = True
            return "GREEN_START"
        
        return "WAITING_GREEN"

    # ---------- 간단한 클러스터링 (sklearn 대체) ----------
    def simple_clustering(self, points, eps=0.15, min_samples=3):
        """sklearn 없이 간단한 클러스터링"""
        if len(points) < min_samples:
            return np.array([-1] * len(points))
        
        labels = np.array([-1] * len(points))
        cluster_id = 0
        
        for i, point in enumerate(points):
            if labels[i] != -1:  # 이미 클러스터에 속함
                continue
                
            # 현재 점에서 eps 거리 내의 점들 찾기
            distances = np.linalg.norm(points - point, axis=1)
            neighbors = np.where(distances <= eps)[0]
            
            if len(neighbors) >= min_samples:
                # 새 클러스터 생성
                labels[neighbors] = cluster_id
                cluster_id += 1
                
        return labels

    # ---------- 라이다 기반 라바콘 검출 ----------
    def detect_lidar_cones(self):
        """라이다 데이터로 라바콘(장애물) 검출 및 경로 계산"""
        if self.lidar_ranges is None:
            return False, None, []
            
        ranges = self.lidar_ranges
        # 무한값과 NaN 처리
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=10.0, neginf=0.0)
        
        angles = np.linspace(0, 2*np.pi, len(ranges), endpoint=False)
        
        # 유효한 거리 데이터만 필터링
        valid_mask = (ranges > LIDAR_CONE_MIN_DIST) & (ranges < LIDAR_CONE_MAX_DIST)
        
        if np.sum(valid_mask) < 5:
            return False, None, []
            
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        # 극좌표를 직교좌표로 변환
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        
        # 전방 120도 영역만 관심 (라바콘은 전방에 있음)
        front_mask = (valid_angles > -np.pi/3) & (valid_angles < np.pi/3)
        
        if not np.any(front_mask):
            return False, None, []
            
        front_x = x[front_mask]
        front_y = y[front_mask]
        
        # 라바콘은 보통 0.5m 이상 전방에 있음
        forward_mask = front_y > 0.5
        if not np.any(forward_mask):
            return False, None, []
            
        cone_x = front_x[forward_mask]
        cone_y = front_y[forward_mask]
        
        # 클러스터링으로 라바콘 그룹 찾기
        points = np.column_stack([cone_x, cone_y])
        if len(points) < LIDAR_CONE_MIN_POINTS:
            return False, None, []
        
        # sklearn 사용 가능하면 DBSCAN, 아니면 간단한 클러스터링
        if USE_SKLEARN:
            clustering = DBSCAN(eps=LIDAR_CONE_MAX_GAP, min_samples=LIDAR_CONE_MIN_POINTS).fit(points)
            labels = clustering.labels_
        else:
            labels = self.simple_clustering(points, eps=LIDAR_CONE_MAX_GAP, min_samples=LIDAR_CONE_MIN_POINTS)
        
        # 각 클러스터의 중심점 계산
        cone_centers = []
        unique_labels = set(labels)
        if -1 in unique_labels:
            unique_labels.remove(-1)  # 노이즈 제외
            
        for label in unique_labels:
            cluster_points = points[labels == label]
            if len(cluster_points) >= LIDAR_CONE_MIN_POINTS:
                center = np.mean(cluster_points, axis=0)
                cone_centers.append(center)
        
        if len(cone_centers) >= 2:
            # 좌우 라바콘 분리 (x좌표 기준)
            cone_centers = np.array(cone_centers)
            left_cones = cone_centers[cone_centers[:, 0] < -CONE_PATH_SAFETY_MARGIN]
            right_cones = cone_centers[cone_centers[:, 0] > CONE_PATH_SAFETY_MARGIN]
            
            if len(left_cones) > 0 and len(right_cones) > 0:
                # 가장 가까운 좌우 라바콘 선택
                closest_left = left_cones[np.argmin(left_cones[:, 1])]
                closest_right = right_cones[np.argmin(right_cones[:, 1])]
                
                # 두 라바콘 사이의 중점을 경로로 설정
                path_center = (closest_left + closest_right) / 2
                return True, path_center, cone_centers
        
        return False, None, cone_centers

    # ---------- 곡률 계산 ----------
    def calculate_curvature(self, left_fit, right_fit):
        """차선의 곡률 계산"""
        if left_fit is None and right_fit is None:
            return 0.0
            
        # 이미지 하단에서의 곡률 계산
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
        mask_hls = cv2.inRange(hls, (0, WHITE_L_MIN, 0), (180, 255, WHITE_S_MAX))
        lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
        L2, A, B = cv2.split(lab)
        mask_labL = cv2.inRange(L2, LAB_L_MIN, 255)
        mask_a = cv2.inRange(A, 128-LAB_A_DEV, 128+LAB_A_DEV)
        mask_b = cv2.inRange(B, 128-LAB_B_DEV, 128+LAB_B_DEV)
        mask_lab = cv2.bitwise_and(mask_labL, cv2.bitwise_and(mask_a, mask_b))
        mask = cv2.bitwise_or(mask_hls, mask_lab)
        hls_y = cv2.inRange(hls, (YELLOW_H_MIN, YELLOW_L_MIN, YELLOW_S_MIN),
                                     (YELLOW_H_MAX, 255, 255))
        mask = cv2.bitwise_and(mask, cv2.bitwise_not(hls_y))
        k5 = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k5, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k5, iterations=1)
        return self._roi_mask_lane_only(mask)

    def _yellow_mask(self, bgr):
        hls = cv2.cvtColor(bgr, cv2.COLOR_BGR2HLS)
        yellow = cv2.inRange(hls, (YELLOW_H_MIN, YELLOW_L_MIN, YELLOW_S_MIN),
                                     (YELLOW_H_MAX, 255, 255))
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

        # 이전값 폴백
        if left_fit  is None: left_fit  = self.prev_left_fit
        if right_fit is None: right_fit = self.prev_right_fit

        self.prev_left_fit, self.prev_right_fit = left_fit, right_fit
        return left_fit, right_fit

    def _search_around_poly(self, bw, left_fit, right_fit):
        margin = 80  # 약간 넓혀서 안정성 향상
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

    # ---------- 피팅(노란 중심) ----------
    def _fit_yellow_center(self, bw):
        ys, xs = np.nonzero(bw)
        if xs.size < 300:
            return self.prev_yellow_fit, xs.size
        fit = np.polyfit(ys, xs, 2)  # x = a*y^2 + b*y + c
        self.prev_yellow_fit = fit
        return fit, xs.size

    # ---------- 중앙 결정 (수정) ----------
    def _lane_center_from_fits(self, left_fit, right_fit, yellow_fit=None, prefer_yellow_pixels=0):
        # 3지점 가중치 평균을 사용하여 차선 중앙을 계산
        y_points = [self.proc_h - 1, int(self.proc_h * 0.7), int(self.proc_h * 0.5)]
        weights  = [0.5, 0.3, 0.2]
        
        center_points = []
        for y, w in zip(y_points, weights):
            if (yellow_fit is not None) and (prefer_yellow_pixels >= YELLOW_PIX_MIN_WARP):
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
        
        # 가중치 평균 계산
        weighted_center = np.average(center_points, weights=weights)
        
        return float(weighted_center)

    # ---------- 조향 ----------
    def calc_steering_angle(self, lane_center_bev, cone_mode=False, is_curve=False):
        img_center = self.proc_w / 2.0
        offset = lane_center_bev - img_center
        ratio  = offset / (self.proc_w / 2.0)
        
        # 곡선 구간에서 조향 스무딩 조정
        if is_curve:
            self.steer_smooth = STEERING_SMOOTH_CURVE
            sensitivity = 1.3  # 곡선에서 더 민감하게
        else:
            self.steer_smooth = STEERING_SMOOTH_STRAIGHT
            sensitivity = 1.0
        
        # 라바콘 모드에서는 더 민감한 조향
        if cone_mode:
            sensitivity *= 1.2
        
        # --- 조향각 계산 로직 보강 ---
        # 차선 곡률(2차 방정식의 1차항 계수)를 조향에 반영
        curve_bias = 0.0
        if self.prev_left_fit is not None:
            curve_bias += self.prev_left_fit[0] * 150
        if self.prev_right_fit is not None:
            curve_bias += self.prev_right_fit[0] * 150
            
        # 곡선에서 추가 조향 보정
        if is_curve:
            curve_bias *= 1.5

        raw_angle = np.clip(-ratio * self.max_steering * sensitivity + curve_bias, 
                           -self.max_steering, self.max_steering)
        steer   = self.prev_steering * (1 - self.steer_smooth) + raw_angle * self.steer_smooth
        self.prev_steering = float(steer)
        return float(steer)

    def steering_to_normalized(self, steering_angle):
        if self.max_steering <= 1e-6: return 0.0
        return float(np.clip(steering_angle / self.max_steering, -1.0, 1.0))

    # ---------- 디스플레이 ----------
    def _draw_result(self, base_bgr, left_fit, right_fit, yellow_fit, lane_center_bev, lidar_cones=None):
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

        yb = h - self.proc_blocked_height - 50
        cv2.line(warp_canvas, (int(w/2), yb), (int(w/2), yb-40), (0,255,0), 2)
        cv2.line(warp_canvas, (int(lane_center_bev), yb), (int(lane_center_bev), yb-40), (0,255,255), 2)

        unwarp = cv2.warpPerspective(warp_canvas, self.Minv, (w, h))
        out = cv2.addWeighted(base_bgr, 1.0, unwarp, 0.9, 0)
        
        # 라이다 기반 라바콘 표시
        if lidar_cones is not None and len(lidar_cones) > 0:
            for cone in lidar_cones:
                # 라이다 좌표를 이미지 좌표로 변환 (간단한 투영)
                # x, y는 차량 기준 좌표계 (전방이 +y, 좌측이 +x)
                if len(cone) >= 2:
                    cone_x, cone_y = cone[0], cone[1]
                    # 이미지 좌표로 변환 (대략적)
                    img_x = int(w/2 - cone_x * 100)  # 좌우 반전, 스케일링
                    img_y = int(h - cone_y * 100)    # 전방이 이미지 하단
                    
                    if 0 <= img_x < w and 0 <= img_y < h:
                        cv2.circle(out, (img_x, img_y), 8, (0, 165, 255), -1)  # 주황색
                        cv2.putText(out, f"C({cone_x:.1f},{cone_y:.1f})", 
                                   (img_x-30, img_y-15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        return out

    def _draw_traffic_light_status(self, img, state):
        """신호등 상태를 이미지에 표시"""
        if state == "GREEN_START":
            color = (0, 255, 0)
            text = "GREEN - GO!"
        elif state == "WAITING_GREEN":
            color = (0, 0, 255)
            text = "WAITING GREEN"
        elif state == "PROCEED":
            color = (128, 255, 128)
            text = "PROCEED"
        else:
            color = (128, 128, 128)
            text = f"TRAFFIC: {state}"
            
        cv2.rectangle(img, (10, 60), (250, 100), color, -1)
        cv2.putText(img, text, (15, 85), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    def publish_debug_image(self, debug_bgr):
        if (self.frame_idx % DEBUG_EVERY) != 0:
            return
        preview = debug_bgr
        if (debug_bgr.shape[1], debug_bgr.shape[0]) != (self.preview_w, self.preview_h):
            preview = cv2.resize(debug_bgr, (self.preview_w, self.preview_h), interpolation=cv2.INTER_AREA)
        msg = self.bridge.cv2_to_imgmsg(preview, encoding='bgr8')
        self.debug_pub.publish(msg)
        frame_store.update_bgr(preview, quality=self.jpeg_quality)

    # ---------- 메인 루프 ----------
    def control_loop(self):
        if self.image is None:
            return
        try:
            self.frame_idx += 1

            base = self.image if PROC_SCALE == 1.0 else cv2.resize(
                self.image, (self.proc_w, self.proc_h), interpolation=cv2.INTER_AREA)
            dbg = base.copy()

            # 신호등 검출 (초기에만)
            traffic_state = "PROCEED"
            if self.frame_idx <= TRAFFIC_LIGHT_CHECK_FRAMES:
                traffic_state = self.detect_traffic_light_once(base)
            elif self.traffic_light_checked:
                traffic_state = "PROCEED"
            
            # 라이다 기반 라바콘 검출
            cone_detected, lidar_path, lidar_cones = self.detect_lidar_cones()
            self.cone_mode_active = cone_detected
            if cone_detected:
                self.lidar_cone_path = lidar_path

            # 주행 제어 로직
            should_stop = False
            lane_center_bev = self.proc_w / 2.0  # 기본값
            src_used = "FALLBACK"
            is_curve = False

            # 1. 신호등 체크 - 초록불 전까지 대기
            if traffic_state == "WAITING_GREEN":
                should_stop = True
                src_used = "WAITING_GREEN"
            # 2. 라이다 기반 라바콘 모드
            elif cone_detected and self.lidar_cone_path is not None:
                # 라이다 경로를 이미지 좌표로 변환
                path_x, path_y = self.lidar_cone_path
                # 간단한 변환: 차량 중심 기준으로 이미지 중심에 매핑
                lane_center_bev = self.proc_w / 2.0 - path_x * 100  # 좌우 반전, 스케일링
                lane_center_bev = np.clip(lane_center_bev, 0, self.proc_w)
                src_used = "LIDAR_CONE"
            # 3. 일반 차선 추종
            else:
                force_redetect = (self.frame_idx % REDETECT_MAX_PERIOD) == 0

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

                    lane_center_bev = self._lane_center_from_fits(lf, rf, yfit, prefer_yellow_pixels=ypix)
                    
                    # 곡률 계산 및 곡선 감지
                    self.current_curvature = self.calculate_curvature(lf, rf)
                    is_curve = self.current_curvature > CURVE_THRESHOLD
                    
                    # 차선 종류에 따라 조향 중심을 조정하여 코너링 성능 향상
                    if yfit is not None and ypix >= YELLOW_PIX_MIN_WARP:
                        src_used = "YELLOW"
                    elif lf is not None and rf is not None:
                        src_used = "WHITE_MID"
                    elif lf is not None:
                        src_used = "LEFT_ONLY"
                    elif rf is not None:
                        src_used = "RIGHT_ONLY"
                    else:
                        src_used = "FALLBACK"
                    
                    if is_curve:
                        src_used += "_CURVE"
                    
                    self.prev_left_fit, self.prev_right_fit, self.prev_yellow_fit = lf, rf, yfit
                    self.last_lane_center = float(lane_center_bev)

                    good_white  = (lf is not None and rf is not None)
                    good_yellow = (ypix >= YELLOW_PIX_MIN_WARP)

                    if good_white or good_yellow:
                        self.lock_rem = LOCK_TTL_FRAMES
                    else:
                        self.lock_rem = 0

            # 조향각 계산
            steering = self.calc_steering_angle(lane_center_bev, 
                                              cone_mode=self.cone_mode_active, 
                                              is_curve=is_curve)
            steer_norm = self.steering_to_normalized(steering)
            
            # 속도 제어 - 곡선 구간에서 감속
            if should_stop:
                throttle = 0.0
                speed_mps = 0.0
            elif self.front_distance < EMER_STOP_DIST:
                throttle = 0.0
                speed_mps = 0.0
                src_used += "_LIDAR_STOP"
            elif is_curve:
                # 곡선에서 속도 감소
                throttle = float(FIXED_THROTTLE * 0.6)
                speed_mps = float(CURVE_SPEED_MPS)
                self.get_logger().info(f"Curve detected: curvature={self.current_curvature:.3f}")
            elif self.cone_mode_active:
                # 라바콘 구간에서는 속도 감소
                throttle = float(FIXED_THROTTLE * 0.75)
                speed_mps = float(FIXED_SPEED_MPS * 0.75)
            else:
                throttle = float(FIXED_THROTTLE)
                speed_mps = float(FIXED_SPEED_MPS)

            # 명령 발행
            cmd = Twist()
            cmd.linear.x = speed_mps
            cmd.angular.z = steering
            self.cmd_pub.publish(cmd)
            self.steer_pub.publish(Float32(data=steer_norm))
            self.throt_pub.publish(Float32(data=throttle))
            
            # 상태 발행
            self.traffic_state_pub.publish(String(data=traffic_state))
            self.cone_detected_pub.publish(Bool(data=self.cone_mode_active))

            # 디버그 이미지 생성 및 발행
            if (self.frame_idx % DEBUG_EVERY) == 0:
                vis = dbg.copy()
                
                # 차선 및 라바콘 표시
                if (self.prev_left_fit is not None) or (self.prev_right_fit is not None) or (self.prev_yellow_fit is not None):
                    vis = self._draw_result(vis, self.prev_left_fit, self.prev_right_fit, 
                                          self.prev_yellow_fit, lane_center_bev, lidar_cones)
                
                # 신호등 상태 표시
                self._draw_traffic_light_status(vis, traffic_state)
                
                # 라이다 경로 중심점 표시
                if cone_detected and self.lidar_cone_path is not None:
                    path_x, path_y = self.lidar_cone_path
                    img_x = int(vis.shape[1]/2 - path_x * 100)
                    img_y = int(vis.shape[0] - path_y * 100)
                    if 0 <= img_x < vis.shape[1] and 0 <= img_y < vis.shape[0]:
                        cv2.circle(vis, (img_x, img_y), 15, (255, 0, 255), -1)
                        cv2.putText(vis, "LIDAR PATH", 
                                   (img_x-50, img_y-20),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                
                # 원본 해상도로 복원
                if PROC_SCALE != 1.0:
                    vis = cv2.resize(vis, (self.img_width, self.img_height), interpolation=cv2.INTER_LINEAR)
                
                # 상태 정보 텍스트
                status_text = (f"Mode: {src_used} | Traffic: {traffic_state} | "
                             f"LidarCones: {len(lidar_cones) if lidar_cones else 0} | "
                             f"Curve: {self.current_curvature:.3f} | Lock: {self.lock_rem}")
                
                speed_text = (f"Throttle: {throttle:.2f} | Speed: {speed_mps:.2f} | "
                            f"Steer: {math.degrees(steering):.1f}deg | "
                            f"Front: {self.front_distance:.2f}m")
                
                cv2.putText(vis, status_text, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                cv2.putText(vis, speed_text, (10, 50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                
                self.publish_debug_image(vis)

        except Exception as e:
            self.get_logger().error(f"Control err: {e}")
            # 에러 시 안전하게 정지
            self.cmd_pub.publish(Twist())
            self.steer_pub.publish(Float32(data=0.0))
            self.throt_pub.publish(Float32(data=0.0))

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
