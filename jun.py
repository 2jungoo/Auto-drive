#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AuTURBO 스타일 완전 자율주행 시스템
- 신호등 인식 (1회)
- 차선 추종 (Pure Pursuit)
- 라바콘 회피
- 다양한 주행 모드
- 곡선 최적화
"""

import threading
import time
import math
import warnings

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String, Bool
from cv_bridge import CvBridge

import cv2
import numpy as np

# 경고 억제
warnings.filterwarnings("ignore")

# =========================
# 🎯 AuTURBO 스타일 파라미터
# =========================

# 주행 모드별 속도 설정
SPEEDS = {
    'STRAIGHT': 0.50,      # 직선 고속 주행
    'CURVE': 0.25,         # 곡선 최적 주행  
    'ZIGZAG': 0.30,        # 지그재그 최적 주행
    'CONE': 0.20,          # 라바콘 회피
    'PARKING': 0.15,       # 주차 모드
    'STOP': 0.0            # 정지
}

THROTTLES = {
    'STRAIGHT': 0.55,
    'CURVE': 0.35,
    'ZIGZAG': 0.40,
    'CONE': 0.30,
    'PARKING': 0.25,
    'STOP': 0.0
}

# Pure Pursuit 파라미터
LOOKAHEAD_DISTANCE = 1.5   # m
WHEELBASE = 0.3           # m (차축 거리)
MAX_STEER_ANGLE = 0.8     # rad

# 차선 검출 파라미터
LANE_WIDTH_M = 0.6        # m (실제 차선 폭)
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

# 곡률 기반 모드 전환
STRAIGHT_THRESHOLD = 0.1   # 직선 판단 임계값
CURVE_THRESHOLD = 0.3      # 곡선 판단 임계값
ZIGZAG_THRESHOLD = 0.6     # 지그재그 판단 임계값

# 라바콘 검출 (라이다)
CONE_DETECT_RANGE = 2.0    # m
CONE_SAFETY_MARGIN = 0.5   # m
CONE_MODE_TIMEOUT = 5.0    # s

# 색상 범위 (HSV)
WHITE_LOWER = np.array([0, 0, 200])
WHITE_UPPER = np.array([180, 30, 255])
YELLOW_LOWER = np.array([20, 80, 80])
YELLOW_UPPER = np.array([40, 255, 255])
GREEN_LOWER = np.array([40, 50, 50])
GREEN_UPPER = np.array([80, 255, 255])

# =========================
# 🚗 AuTURBO 스타일 자율주행 노드
# =========================

class AuTURBOAutoracer(Node):
    def __init__(self):
        super().__init__('auturbo_autoracer')
        
        # QoS 설정
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.steer_pub = self.create_publisher(Float32, '/cmd/steering', 10)
        self.throt_pub = self.create_publisher(Float32, '/cmd/throttle', 10)
        self.mode_pub = self.create_publisher(String, '/driving_mode', 10)
        self.debug_pub = self.create_publisher(String, '/debug_info', 10)
        
        # Subscribers
        self.img_sub = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.image_callback, sensor_qos)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, sensor_qos)
        
        # 상태 변수
        self.bridge = CvBridge()
        self.current_image = None
        self.lidar_ranges = None
        self.front_distance = float('inf')
        
        # 주행 상태
        self.driving_mode = 'INIT'
        self.frame_count = 0
        self.start_time = time.time()
        self.last_cone_time = 0
        
        # 차선 추종 상태
        self.lane_center_error = 0.0
        self.prev_error = 0.0
        self.lane_curvature = 0.0
        self.left_lane_detected = False
        self.right_lane_detected = False
        
        # 신호등 상태
        self.traffic_light_checked = False
        self.green_light_detected = False
        
        # Bird's Eye View 변환 매트릭스
        self.M, self.Minv = self._setup_bird_eye_view()
        
        # 제어 타이머 (30Hz)
        self.control_timer = self.create_timer(1.0/30.0, self.control_loop)
        
        self.get_logger().info("🚗 AuTURBO 스타일 자율주행 시스템 시작!")
        self.get_logger().info("📊 모니터링: ros2 topic echo /driving_mode")
        
    def image_callback(self, msg):
        """카메라 콜백"""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.current_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # 해상도 조정
            if self.current_image is not None:
                self.current_image = cv2.resize(self.current_image, (IMAGE_WIDTH, IMAGE_HEIGHT))
        except Exception as e:
            self.get_logger().error(f"Image callback error: {e}")
    
    def lidar_callback(self, msg):
        """라이다 콜백"""
        try:
            self.lidar_ranges = np.array(msg.ranges)
            self.lidar_ranges = np.nan_to_num(self.lidar_ranges, nan=0.0, posinf=10.0)
            
            # 전방 거리 계산
            total_points = len(self.lidar_ranges)
            center = total_points // 2
            front_segment = self.lidar_ranges[max(0, center-15):min(total_points, center+15)]
            valid_distances = front_segment[(front_segment > 0.1) & (front_segment < 10.0)]
            
            if len(valid_distances) > 0:
                self.front_distance = float(np.min(valid_distances))
            else:
                self.front_distance = float('inf')
                
        except Exception as e:
            self.get_logger().error(f"LiDAR callback error: {e}")
    
    def _setup_bird_eye_view(self):
        """Bird's Eye View 변환 설정"""
        # 소스 포인트 (원근 보정 전)
        src_points = np.float32([
            [IMAGE_WIDTH * 0.1, IMAGE_HEIGHT],
            [IMAGE_WIDTH * 0.4, IMAGE_HEIGHT * 0.6],
            [IMAGE_WIDTH * 0.6, IMAGE_HEIGHT * 0.6],
            [IMAGE_WIDTH * 0.9, IMAGE_HEIGHT]
        ])
        
        # 목적지 포인트 (Bird's Eye View)
        dst_points = np.float32([
            [IMAGE_WIDTH * 0.2, IMAGE_HEIGHT],
            [IMAGE_WIDTH * 0.2, 0],
            [IMAGE_WIDTH * 0.8, 0],
            [IMAGE_WIDTH * 0.8, IMAGE_HEIGHT]
        ])
        
        M = cv2.getPerspectiveTransform(src_points, dst_points)
        Minv = cv2.getPerspectiveTransform(dst_points, src_points)
        
        return M, Minv
    
    def detect_traffic_light_once(self, image):
        """신호등 1회 검출"""
        if self.traffic_light_checked:
            return True  # 이미 체크 완료
            
        # 상단 영역에서 초록불 검출
        h, w = image.shape[:2]
        roi = image[0:h//3, w//4:3*w//4]
        
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)
        
        green_area = cv2.countNonZero(green_mask)
        
        if green_area > 500:  # 충분한 초록색 영역
            self.green_light_detected = True
            self.traffic_light_checked = True
            self.get_logger().info("✅ 초록불 감지! 주행 시작")
            return True
            
        return False
    
    def detect_lanes_hsv(self, image):
        """HSV 기반 차선 검출"""
        # Bird's Eye View 변환
        bev_image = cv2.warpPerspective(image, self.M, (IMAGE_WIDTH, IMAGE_HEIGHT))
        
        # HSV 변환
        hsv = cv2.cvtColor(bev_image, cv2.COLOR_BGR2HSV)
        
        # 흰색과 노란색 차선 마스크
        white_mask = cv2.inRange(hsv, WHITE_LOWER, WHITE_UPPER)
        yellow_mask = cv2.inRange(hsv, YELLOW_LOWER, YELLOW_UPPER)
        
        # 합성 마스크
        lane_mask = cv2.bitwise_or(white_mask, yellow_mask)
        
        # 노이즈 제거
        kernel = np.ones((5,5), np.uint8)
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, kernel)
        
        return lane_mask, bev_image
    
    def sliding_window_lane_detection(self, binary_image):
        """Sliding Window 차선 검출"""
        # 히스토그램으로 차선 시작점 찾기
        histogram = np.sum(binary_image[binary_image.shape[0]//2:, :], axis=0)
        
        # 좌우 차선 시작점
        midpoint = histogram.shape[0] // 2
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint
        
        # Sliding window 파라미터
        nwindows = 9
        window_height = binary_image.shape[0] // nwindows
        margin = 80
        minpix = 50
        
        # 비영 픽셀 찾기
        nonzero = binary_image.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        
        # 현재 위치
        leftx_current = leftx_base
        rightx_current = rightx_base
        
        left_lane_inds = []
        right_lane_inds = []
        
        # 윈도우별 처리
        for window in range(nwindows):
            win_y_low = binary_image.shape[0] - (window + 1) * window_height
            win_y_high = binary_image.shape[0] - window * window_height
            
            # 좌측 윈도우
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            
            # 우측 윈도우
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            
            # 윈도우 내 픽셀 찾기
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                            (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                             (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            
            # 다음 윈도우 중심 업데이트
            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))
        
        # 인덱스 결합
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
        
        # 차선 피팅
        left_fit = None
        right_fit = None
        
        if len(left_lane_inds) > 100:
            leftx = nonzerox[left_lane_inds]
            lefty = nonzeroy[left_lane_inds]
            left_fit = np.polyfit(lefty, leftx, 2)
            self.left_lane_detected = True
        else:
            self.left_lane_detected = False
            
        if len(right_lane_inds) > 100:
            rightx = nonzerox[right_lane_inds]
            righty = nonzeroy[right_lane_inds]
            right_fit = np.polyfit(righty, rightx, 2)
            self.right_lane_detected = True
        else:
            self.right_lane_detected = False
            
        return left_fit, right_fit
    
    def calculate_lane_center_and_curvature(self, left_fit, right_fit):
        """차선 중앙과 곡률 계산"""
        y_eval = IMAGE_HEIGHT - 1
        
        # 차선 중앙 계산
        if left_fit is not None and right_fit is not None:
            # 양쪽 차선 모두 있는 경우
            left_x = np.polyval(left_fit, y_eval)
            right_x = np.polyval(right_fit, y_eval)
            lane_center = (left_x + right_x) / 2
            
            # 곡률 계산
            left_curverad = abs(2 * left_fit[0])
            right_curverad = abs(2 * right_fit[0])
            self.lane_curvature = (left_curverad + right_curverad) / 2
            
        elif left_fit is not None:
            # 왼쪽 차선만 있는 경우
            left_x = np.polyval(left_fit, y_eval)
            lane_center = left_x + (LANE_WIDTH_M * IMAGE_WIDTH / 4)  # 추정
            self.lane_curvature = abs(2 * left_fit[0])
            
        elif right_fit is not None:
            # 오른쪽 차선만 있는 경우
            right_x = np.polyval(right_fit, y_eval)
            lane_center = right_x - (LANE_WIDTH_M * IMAGE_WIDTH / 4)  # 추정
            self.lane_curvature = abs(2 * right_fit[0])
            
        else:
            # 차선이 없는 경우
            lane_center = IMAGE_WIDTH / 2
            self.lane_curvature = 0.0
        
        # 중앙 오차 계산 (정규화: -1.0 ~ 1.0)
        self.lane_center_error = (lane_center - IMAGE_WIDTH/2) / (IMAGE_WIDTH/2)
        
        return lane_center, self.lane_curvature
    
    def detect_cones_lidar(self):
        """라이다 기반 라바콘 검출"""
        if self.lidar_ranges is None:
            return False, 0.0, 0.0
        
        # 전방 120도 범위
        total_points = len(self.lidar_ranges)
        center = total_points // 2
        front_range = total_points // 6  # ±30도
        
        left_segment = self.lidar_ranges[center-front_range:center]
        right_segment = self.lidar_ranges[center:center+front_range]
        
        # 유효한 거리 필터링
        valid_left = left_segment[(left_segment > 0.2) & (left_segment < CONE_DETECT_RANGE)]
        valid_right = right_segment[(right_segment > 0.2) & (right_segment < CONE_DETECT_RANGE)]
        
        left_min = np.min(valid_left) if len(valid_left) > 0 else float('inf')
        right_min = np.min(valid_right) if len(valid_right) > 0 else float('inf')
        
        # 라바콘 검출 조건: 양쪽에 장애물이 있고 적당한 간격
        if (left_min < CONE_DETECT_RANGE and right_min < CONE_DETECT_RANGE and 
            abs(left_min - right_min) < 1.0):
            self.last_cone_time = time.time()
            return True, left_min, right_min
            
        return False, left_min, right_min
    
    def determine_driving_mode(self):
        """AuTURBO 스타일 주행 모드 결정"""
        current_time = time.time()
        
        # 1. 초기 신호등 체크
        if not self.traffic_light_checked and self.current_image is not None:
            if self.detect_traffic_light_once(self.current_image):
                self.driving_mode = 'STRAIGHT'
            else:
                self.driving_mode = 'STOP'
                return
        
        # 2. 라바콘 모드 체크
        cone_detected, left_dist, right_dist = self.detect_cones_lidar()
        if cone_detected:
            self.driving_mode = 'CONE'
            return
        elif (current_time - self.last_cone_time) < CONE_MODE_TIMEOUT:
            self.driving_mode = 'CONE'  # 라바콘 후 잠시 유지
            return
        
        # 3. 곡률 기반 모드 결정
        if self.lane_curvature < STRAIGHT_THRESHOLD:
            self.driving_mode = 'STRAIGHT'
        elif self.lane_curvature < CURVE_THRESHOLD:
            self.driving_mode = 'CURVE'
        elif self.lane_curvature < ZIGZAG_THRESHOLD:
            self.driving_mode = 'ZIGZAG'
        else:
            self.driving_mode = 'CURVE'  # 급커브는 커브 모드
    
    def pure_pursuit_control(self):
        """Pure Pursuit 제어 알고리즘"""
        # 목표점까지의 거리
        ld = LOOKAHEAD_DISTANCE
        
        # 횡방향 오차를 이용한 조향각 계산
        alpha = math.atan2(2 * WHEELBASE * self.lane_center_error, ld)
        
        # 조향각 제한
        steering_angle = np.clip(alpha, -MAX_STEER_ANGLE, MAX_STEER_ANGLE)
        
        # 곡률에 따른 추가 보정
        if self.driving_mode == 'CURVE':
            # 곡선에서 조향 강화
            steering_angle *= 1.2
        elif self.driving_mode == 'ZIGZAG':
            # 지그재그에서 조향 강화
            steering_angle *= 1.5
        
        return steering_angle
    
    def control_loop(self):
        """메인 제어 루프"""
        if self.current_image is None:
            return
            
        try:
            self.frame_count += 1
            
            # 차선 검출
            lane_mask, bev_image = self.detect_lanes_hsv(self.current_image)
            left_fit, right_fit = self.sliding_window_lane_detection(lane_mask)
            
            # 차선 중앙과 곡률 계산
            lane_center, curvature = self.calculate_lane_center_and_curvature(left_fit, right_fit)
            
            # 주행 모드 결정
            self.determine_driving_mode()
            
            # Pure Pursuit 제어
            steering_angle = self.pure_pursuit_control()
            
            # 속도 설정
            target_speed = SPEEDS.get(self.driving_mode, 0.0)
            target_throttle = THROTTLES.get(self.driving_mode, 0.0)
            
            # 긴급 정지 체크
            if self.front_distance < 0.3:
                target_speed = 0.0
                target_throttle = 0.0
                self.driving_mode = 'EMERGENCY_STOP'
            
            # 명령 발행
            cmd_vel = Twist()
            cmd_vel.linear.x = float(target_speed)
            cmd_vel.angular.z = float(steering_angle)
            
            self.cmd_pub.publish(cmd_vel)
            self.steer_pub.publish(Float32(data=float(steering_angle)))
            self.throt_pub.publish(Float32(data=float(target_throttle)))
            self.mode_pub.publish(String(data=self.driving_mode))
            
            # 디버그 정보 (1초마다)
            if self.frame_count % 30 == 0:
                debug_msg = (f"Mode: {self.driving_mode} | "
                           f"Speed: {target_speed:.2f} | "
                           f"Steer: {math.degrees(steering_angle):.1f}° | "
                           f"Curve: {self.lane_curvature:.3f} | "
                           f"Error: {self.lane_center_error:.3f} | "
                           f"Front: {self.front_distance:.2f}m | "
                           f"L/R Lane: {self.left_lane_detected}/{self.right_lane_detected}")
                
                self.get_logger().info(debug_msg)
                self.debug_pub.publish(String(data=debug_msg))
                
                # 주행 시작 확인 메시지
                if not self.traffic_light_checked:
                    self.get_logger().info("🚦 신호등 대기 중... (초록불을 찾고 있습니다)")
                elif self.driving_mode != 'STOP':
                    self.get_logger().info(f"🚗 주행 중: {self.driving_mode} 모드")
            
        except Exception as e:
            self.get_logger().error(f"Control loop error: {e}")
            # 에러 시 안전 정지
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)


def main(args=None):
    print("\n" + "="*60)
    print("🏎️  AuTURBO 스타일 자율주행 시스템")
    print("="*60)
    print("📋 구현된 기능:")
    print("  ✅ 신호등 인식 (1회)")
    print("  ✅ HSV 기반 차선 검출")
    print("  ✅ Bird's Eye View 변환")
    print("  ✅ Sliding Window 차선 추적")
    print("  ✅ Pure Pursuit 제어")
    print("  ✅ 다중 주행 모드 (직선/곡선/지그재그/라바콘)")
    print("  ✅ 라이다 기반 라바콘 검출")
    print("  ✅ 곡률 기반 속도 조절")
    print("="*60)
    print("🔧 모니터링 명령어:")
    print("  ros2 topic echo /driving_mode")
    print("  ros2 topic echo /debug_info")
    print("="*60)
    print()
    
    rclpy.init(args=args)
    node = AuTURBOAutoracer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 시스템 종료")
    finally:
        # 안전 정지
        stop_cmd = Twist()
        node.cmd_pub.publish(stop_cmd)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
