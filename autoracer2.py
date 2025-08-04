#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================
# ROS2 대회용 자율주행 코드 - 신호등 인식 + 차선 인식 주행
# 해상도: 1280x720, 하단 1/5 제거하여 차선 인식
# =============================================

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import time
import math

qos_profile = QoSProfile(depth=10)
qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

class Autoracer(Node):
    def __init__(self):
        super().__init__('Autoracer')
        
        # 구독자 설정
        self.img_sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10)
        
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile)
        
        # 발행자 설정 (차량 제어용)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 전역 변수 초기화
        self.image = None
        self.ranges = None
        self.started = False  # 신호등 인식 후 출발
        self.Fix_Speed = 0.3  # ROS2용 속도 (m/s)
        
        # PID 제어를 위한 변수들
        self.prev_error = 0
        self.integral = 0
        
        # 차선 변경 관련 변수들
        self.lane_change_mode = False
        self.lane_change_direction = 0  # -1: 왼쪽, 1: 오른쪽
        self.lane_change_start_time = 0
        self.current_lane = "right"  # "left", "right", "center"
        self.solid_line_violation_count = 0
        
        # 신호등 인식 시도 횟수
        self.traffic_light_attempts = 0
        self.max_attempts = 100  # 10초 후 강제 시작
        
        # 타이머 설정 (메인 루프용)
        self.timer = self.create_timer(0.05, self.main_loop)  # 20Hz
        
        self.get_logger().info("=== ROS2 자율주행 시스템 초기화 완료 ===")

    def image_callback(self, msg):
        """이미지 콜백 함수"""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"이미지 디코딩 오류: {e}")

    def lidar_callback(self, msg):
        """라이다 콜백 함수"""
        self.ranges = np.array(msg.ranges)
        
        # 중앙 10개 포인트 로깅 (디버깅용)
        total_points = len(msg.ranges)
        center = total_points // 2
        half_width = 5
        start_idx = max(0, center - half_width)
        end_idx = min(total_points, center + half_width)
        center_ranges = msg.ranges[start_idx:end_idx]
        formatted_ranges = [f"{r:.3f}" for r in center_ranges]
        self.get_logger().info(f"Center ranges: {formatted_ranges}")

    def drive(self, angle, speed):
        """차량 제어 명령 발행"""
        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(angle)
        self.cmd_pub.publish(cmd)

    def detect_traffic_light(self, img):
        """
        1280x720 해상도에 최적화된 신호등 인식
        """
        if img is None:
            return "unknown"
            
        height, width = img.shape[:2]
        
        # 1280x720 해상도에 맞는 ROI 영역 설정
        roi_areas = [
            img[60:240, 400:880],   # 중앙 상단 (더 넓은 영역)
            img[80:280, 360:920],   # 약간 더 넓은 영역
            img[40:200, 500:780],   # 더 위쪽 영역
        ]

        green_detected = False
        red_detected = False

        for i, roi in enumerate(roi_areas):
            if roi.size == 0:
                continue

            # HSV 변환
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            # 초록불 검출 (여러 범위로 시도)
            green_ranges = [
                ((40, 50, 50), (80, 255, 255)),   # 기본 초록색
                ((35, 40, 40), (85, 255, 255)),   # 더 넓은 범위
                ((45, 80, 80), (75, 255, 255)),   # 더 진한 초록색
            ]

            for lower, upper in green_ranges:
                green_mask = cv2.inRange(hsv, lower, upper)
                green_pixels = cv2.countNonZero(green_mask)

                if green_pixels > 100:  # 1280x720에 맞춘 임계값
                    green_detected = True
                    break

            # 빨간불 검출
            red_ranges = [
                ((0, 50, 50), (10, 255, 255)),     # 빨간색 범위1
                ((170, 50, 50), (180, 255, 255)),  # 빨간색 범위2
            ]

            for lower, upper in red_ranges:
                red_mask = cv2.inRange(hsv, lower, upper)
                red_pixels = cv2.countNonZero(red_mask)

                if red_pixels > 100:
                    red_detected = True
                    break

            # 디버깅용 이미지 표시 (첫 번째 ROI만)
            if i == 0:
                cv2.imshow(f"Traffic Light ROI", roi)
                if green_detected:
                    cv2.imshow("Green Mask", green_mask)

        # 결과 판단
        if green_detected and not red_detected:
            return "green"
        elif red_detected and not green_detected:
            return "red"
        else:
            return "yellow"

    def detect_lane_error(self, img):
        """
        1280x720 해상도에 최적화된 차선 인식
        하단 1/5 (144픽셀) 제거하여 라이더와 본넷 영역 배제
        """
        if img is None:
            return 0
            
        height, width = img.shape[:2]  # 720, 1280
        
        # 하단 1/5 제거 (144픽셀 제거)
        usable_height = int(height * 4/5)  # 576픽셀
        img_cropped = img[:usable_height, :]
        
        # ROI 설정 (크롭된 이미지의 하단 200픽셀)
        roi_height = 200
        roi_start = max(0, usable_height - roi_height)
        roi = img_cropped[roi_start:usable_height, :]

        # 가우시안 블러 적용 (노이즈 제거)
        blurred = cv2.GaussianBlur(roi, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 1280x720 해상도에 최적화된 색상 범위
        # 흰색 차선 (실선/점선 모두)
        white_lower = np.array([0, 0, 180])
        white_upper = np.array([180, 30, 255])
        white_mask = cv2.inRange(hsv, white_lower, white_upper)

        # 노란색 차선 (점선/실선)
        yellow_lower = np.array([15, 100, 120])
        yellow_upper = np.array([35, 255, 255])
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

        # 모폴로지 연산으로 노이즈 제거
        kernel = np.ones((3, 3), np.uint8)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)

        # 전체 차선 마스크 결합
        combined_mask = cv2.bitwise_or(white_mask, yellow_mask)

        # 좌우 영역 분할하여 차선 검출
        mid_point = width // 2  # 640
        left_region = combined_mask[:, :mid_point]
        right_region = combined_mask[:, mid_point:]

        # 차선 정보 분석
        lane_info = self.analyze_lane_structure(left_region, right_region, 
                                               white_mask, yellow_mask, width)

        # 현재 차량 위치 판단 및 주행 경로 계산
        error, lane_status = self.calculate_driving_path(lane_info, width, mid_point)

        # 디버깅용 이미지 생성
        self.create_debug_image(roi, lane_info, error, width, roi_height)

        return error

    def analyze_lane_structure(self, left_region, right_region, white_mask, yellow_mask, width):
        """차선 구조를 분석하여 실선/점선 정보와 위치를 파악"""
        lane_info = {
            'left_line': None,
            'right_line': None,
            'left_type': None,  # 'solid', 'dashed', None
            'right_type': None,
            'lane_width': None
        }

        # 왼쪽 차선 검출
        left_contours, _ = cv2.findContours(left_region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if left_contours:
            largest_left = max(left_contours, key=cv2.contourArea)
            if cv2.contourArea(largest_left) > 100:  # 1280x720에 맞춘 최소 크기
                M = cv2.moments(largest_left)
                if M['m00'] != 0:
                    lane_info['left_line'] = int(M['m10'] / M['m00'])
                    lane_info['left_type'] = self.determine_line_type(left_region, lane_info['left_line'])

        # 오른쪽 차선 검출
        right_contours, _ = cv2.findContours(right_region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if right_contours:
            largest_right = max(right_contours, key=cv2.contourArea)
            if cv2.contourArea(largest_right) > 100:
                M = cv2.moments(largest_right)
                if M['m00'] != 0:
                    lane_info['right_line'] = int(M['m10'] / M['m00']) + width // 2
                    lane_info['right_type'] = self.determine_line_type(right_region, 
                                                                     lane_info['right_line'] - width // 2)

        # 차선 폭 계산
        if lane_info['left_line'] and lane_info['right_line']:
            lane_info['lane_width'] = lane_info['right_line'] - lane_info['left_line']

        return lane_info

    def determine_line_type(self, mask, center_x):
        """차선의 연속성을 분석하여 실선/점선 판단"""
        if center_x is None or center_x < 0 or center_x >= mask.shape[1]:
            return None

        # 세로 방향으로 연속성 체크
        vertical_line = mask[:, max(0, center_x - 3):min(mask.shape[1], center_x + 4)]
        non_zero_ratio = np.count_nonzero(vertical_line) / vertical_line.size

        # 연속성이 높으면 실선, 낮으면 점선
        if non_zero_ratio > 0.7:
            return 'solid'
        elif non_zero_ratio > 0.3:
            return 'dashed'
        else:
            return None

    def calculate_driving_path(self, lane_info, width, mid_point):
        """차선 정보를 바탕으로 주행 경로 계산 (실선 준수)"""
        error = 0
        lane_status = "normal"

        # 양쪽 차선이 모두 검출된 경우
        if lane_info['left_line'] and lane_info['right_line']:
            lane_center = (lane_info['left_line'] + lane_info['right_line']) // 2
            error = lane_center - mid_point

            # 실선 위반 체크 (1280x720 해상도에 맞춘 여유 공간)
            vehicle_position = mid_point

            # 왼쪽 실선 위반 체크
            if (lane_info['left_type'] == 'solid' and
                    vehicle_position < lane_info['left_line'] + 40):  # 여유 공간 40픽셀
                self.solid_line_violation_count += 1
                lane_status = "left_solid_violation"
                error = max(error, 40)  # 오른쪽으로 강제 조정

            # 오른쪽 실선 위반 체크
            elif (lane_info['right_type'] == 'solid' and
                  vehicle_position > lane_info['right_line'] - 40):
                self.solid_line_violation_count += 1
                lane_status = "right_solid_violation"
                error = min(error, -40)  # 왼쪽으로 강제 조정

            # 정상 주행
            else:
                self.solid_line_violation_count = max(0, self.solid_line_violation_count - 1)

        # 한쪽 차선만 검출된 경우
        elif lane_info['left_line'] and not lane_info['right_line']:
            # 왼쪽 차선 기준으로 주행 (1280x720 해상도에 맞춘 거리)
            if lane_info['left_type'] == 'solid':
                target_position = lane_info['left_line'] + 160  # 실선에서 안전거리
            else:
                target_position = lane_info['left_line'] + 120  # 점선이면 조금 더 가까이
            error = target_position - mid_point

        elif lane_info['right_line'] and not lane_info['left_line']:
            # 오른쪽 차선 기준으로 주행
            if lane_info['right_type'] == 'solid':
                target_position = lane_info['right_line'] - 160  # 실선에서 안전거리
            else:
                target_position = lane_info['right_line'] - 120  # 점선이면 조금 더 가까이
            error = target_position - mid_point

        # 차선이 전혀 안 보이는 경우
        else:
            error = 0  # 직진
            lane_status = "no_lane_detected"

        return error, lane_status

    def create_debug_image(self, roi, lane_info, error, width, roi_height):
        """디버깅용 이미지 생성"""
        debug_img = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
        mid_point = width // 2

        # 차선 표시
        if lane_info['left_line']:
            color = (255, 0, 0) if lane_info['left_type'] == 'solid' else (255, 100, 100)
            cv2.line(debug_img, (lane_info['left_line'], 0),
                     (lane_info['left_line'], roi_height), color, 3)
            cv2.putText(debug_img, f"L:{lane_info['left_type']}",
                        (lane_info['left_line'] - 50, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        if lane_info['right_line']:
            color = (255, 255, 0) if lane_info['right_type'] == 'solid' else (255, 255, 100)
            cv2.line(debug_img, (lane_info['right_line'], 0),
                     (lane_info['right_line'], roi_height), color, 3)
            cv2.putText(debug_img, f"R:{lane_info['right_type']}",
                        (lane_info['right_line'] - 50, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # 목표 경로 표시
        target_x = mid_point + error
        cv2.line(debug_img, (target_x, 0), (target_x, roi_height), (0, 255, 0), 3)

        # 차량 중심선 표시
        cv2.line(debug_img, (mid_point, 0), (mid_point, roi_height), (0, 0, 255), 2)

        # 정보 표시 (1280x720에 맞춘 폰트 크기)
        cv2.putText(debug_img, f"Error: {error:.1f}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(debug_img, f"Violations: {self.solid_line_violation_count}", (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

        cv2.imshow("Lane Analysis Debug", debug_img)

    def compute_steering_pid(self, error, lane_status="normal"):
        """실선 위반 방지를 포함한 PID 제어"""
        # 실선 위반 시 더 강한 보정
        if "solid_violation" in lane_status:
            Kp = 0.008  # ROS2용 조향 게인 (더 강한 보정)
            Ki = 0.0001
            Kd = 0.003
            self.get_logger().warn("실선 위반 감지! 강한 보정 적용")
        else:
            # 일반적인 PID 계수 (ROS2 Twist 메시지용)
            Kp = 0.005
            Ki = 0.00005
            Kd = 0.002

        # 적분항 계산 (windup 방지)
        self.integral += error
        self.integral = max(-200, min(200, self.integral))

        # 미분항 계산
        derivative = error - self.prev_error
        self.prev_error = error

        # PID 출력 계산
        steer = Kp * error + Ki * self.integral + Kd * derivative

        # 실선 위반 누적시 추가 제한
        if self.solid_line_violation_count > 3:
            if "left_solid_violation" in lane_status:
                steer = max(steer, 0.1)  # 최소 오른쪽 조향
            elif "right_solid_violation" in lane_status:
                steer = min(steer, -0.1)  # 최소 왼쪽 조향

        # 조향각 제한 (ROS2 angular.z 범위)
        steer = max(-1.0, min(1.0, steer))

        return steer

    def main_loop(self):
        """메인 제어 루프"""
        if self.image is None:
            return

        # 영상 출력
        cv2.imshow("Original", self.image)
        cv2.waitKey(1)

        # 신호등 인식 단계
        if not self.started:
            status = self.detect_traffic_light(self.image)
            self.traffic_light_attempts += 1

            self.get_logger().info(f"[신호등 상태: {status}] 시도: {self.traffic_light_attempts}")

            if status == "green":
                self.started = True
                self.get_logger().info("🟢 초록불 감지! 출발합니다!")
            elif self.traffic_light_attempts > self.max_attempts:
                self.started = True
                self.get_logger().warn("⚠️ 신호등 인식 시간 초과, 강제 출발")
            else:
                self.drive(angle=0.0, speed=0.0)
                return

        # 차선 인식 및 주행 (실선 준수 포함)
        try:
            error = self.detect_lane_error(self.image)

            # PID 제어로 조향 계산
            steer = self.compute_steering_pid(error)

            # 급격한 커브나 실선 위반 시 속도 조절
            if abs(steer) > 0.7 or self.solid_line_violation_count > 0:
                speed = self.Fix_Speed - 0.1  # 감속
            elif abs(steer) > 0.5:
                speed = self.Fix_Speed - 0.05
            elif abs(steer) > 0.3:
                speed = self.Fix_Speed - 0.02
            else:
                speed = self.Fix_Speed

            # 실선 위반 누적 시 추가 감속
            if self.solid_line_violation_count > 5:
                speed = max(0.1, speed - 0.1)
                self.get_logger().warn(f"실선 위반 누적 {self.solid_line_violation_count}회 - 안전 감속")

            self.get_logger().info(f"[주행] 오차: {error:.1f}, 조향: {steer:.3f}, 속도: {speed:.2f}, 위반: {self.solid_line_violation_count}")

            self.drive(angle=steer, speed=speed)

        except Exception as e:
            self.get_logger().error(f"주행 중 오류: {e}")
            self.drive(angle=0.0, speed=0.1)  # 안전 모드

def main(args=None):
    rclpy.init(args=args)
    node = Autoracer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("사용자에 의한 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
