import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import cv2
import numpy as np
import time

# QOS 프로파일 설정
qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

# --- 상수 정의 ---
# 이미지 처리 관련
WIDTH, HEIGHT = 640, 480
ROI_START_ROW, ROI_END_ROW = HEIGHT // 2, HEIGHT - 50 # 차선 및 라바콘 인식을 위한 관심 영역(ROI)

# 색상 범위 (HSV)
LOWER_GREEN = np.array([40, 100, 100])
UPPER_GREEN = np.array([80, 255, 255])
LOWER_ORANGE = np.array([5, 130, 130])
UPPER_ORANGE = np.array([25, 255, 255])
LOWER_WHITE = np.array([0, 0, 200])
UPPER_WHITE = np.array([180, 30, 255])
LOWER_YELLOW = np.array([20, 100, 100])
UPPER_YELLOW = np.array([30, 255, 255])

# 주행 상태
STATE_WAITING_FOR_GREEN_LIGHT = 0
STATE_LANE_AND_CONE_FOLLOWING = 1
STATE_OBSTACLE_AVOIDANCE_START = 2
STATE_OBSTACLE_AVOIDANCE_RETURN = 3

# 차량 제어 관련
BASE_SPEED = 0.5
STEERING_GAIN = 1.2 # 조향 민감도
OBSTACLE_THRESHOLD_DISTANCE = 1.0 # 장애물 인식 거리 (m)

class Autoracer(Node):
    def __init__(self):
        super().__init__('Autoracer')
        
        # --- 구독 (Subscription) ---
        self.img_sub = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.image_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, qos_profile)

        # --- 발행 (Publication) ---
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)

        # --- 클래스 변수 초기화 ---
        self.current_state = STATE_WAITING_FOR_GREEN_LIGHT
        self.obstacle_detected = False
        self.obstacle_avoidance_timer = None
        self.get_logger().info('Autoracer Node has been initialized.')
        self.get_logger().info(f'Initial State: WAITING_FOR_GREEN_LIGHT')

    def image_callback(self, msg):
        """카메라 이미지를 받아 처리하고, 상태에 따라 주행 로직을 수행하는 메인 콜백 함수"""
        
        # 1. 이미지 디코딩 및 전처리
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image_hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)

        steering_angle = 0.0
        speed = 0.0

        # --- 상태별 로직 분기 ---
        if self.current_state == STATE_WAITING_FOR_GREEN_LIGHT:
            # [미션 1: 초록색 신호등 인식 후 출발]
            if self.is_green_light(image_hsv):
                self.get_logger().info('Green light detected! Starting...')
                self.current_state = STATE_LANE_AND_CONE_FOLLOWING
        
        elif self.current_state == STATE_LANE_AND_CONE_FOLLOWING:
            # [미션 2 & 3: 라바콘 및 차선 주행]
            steering_angle = self.get_lane_and_cone_steering(image_bgr, image_hsv)
            speed = BASE_SPEED
        
        elif self.current_state == STATE_OBSTACLE_AVOIDANCE_START:
            # [미션 4: 장애물 회피 - 2차선으로]
            self.get_logger().info('State: OBSTACLE_AVOIDANCE_START - Changing to 2nd lane.')
            steering_angle = -0.6 # 왼쪽으로 크게 꺾음
            speed = BASE_SPEED * 0.8
            # 일정 시간 후 다음 상태로 전환
            if self.obstacle_avoidance_timer is None:
                self.obstacle_avoidance_timer = time.time()
            elif time.time() - self.obstacle_avoidance_timer > 2.5: # 2.5초간 차선 변경
                self.current_state = STATE_OBSTACLE_AVOIDANCE_RETURN
                self.obstacle_avoidance_timer = None
        
        elif self.current_state == STATE_OBSTACLE_AVOIDANCE_RETURN:
            # [미션 4: 장애물 회피 - 1차선으로 복귀]
            self.get_logger().info('State: OBSTACLE_AVOIDANCE_RETURN - Returning to 1st lane.')
            steering_angle = 0.6 # 오른쪽으로 크게 꺾음
            speed = BASE_SPEED * 0.8
            # 일정 시간 후 원래 주행 상태로 복귀
            if self.obstacle_avoidance_timer is None:
                self.obstacle_avoidance_timer = time.time()
            elif time.time() - self.obstacle_avoidance_timer > 2.5: # 2.5초간 복귀
                self.current_state = STATE_LANE_AND_CONE_FOLLOWING
                self.obstacle_avoidance_timer = None
                self.obstacle_detected = False # 장애물 플래그 초기화
                self.get_logger().info('Obstacle avoidance complete. Returning to lane following.')

        # 최종 제어 신호 발행
        self.publish_drive_command(speed, steering_angle)

    def lidar_callback(self, msg):
        """LiDAR 데이터를 받아 전방 장애물을 감지하는 콜백 함수"""
        if self.current_state != STATE_LANE_AND_CONE_FOLLOWING:
            return # 차선 주행 상태가 아닐 때는 장애물 감지 안함

        # 전방 30도 범위의 LiDAR 데이터만 사용
        num_ranges = len(msg.ranges)
        center_index = num_ranges // 2
        scan_width = num_ranges // 12 # 360 / 12 = 30도
        
        front_ranges = msg.ranges[center_index - scan_width : center_index + scan_width]
        
        # 유효한 거리 값만 필터링 (0 또는 inf 제외)
        valid_ranges = [r for r in front_ranges if np.isfinite(r) and r > 0.1]
        
        if not valid_ranges:
            return

        min_distance = min(valid_ranges)
        
        # 장애물이 임계 거리보다 가깝고, 이전에 감지되지 않았다면 상태 변경
        if min_distance < OBSTACLE_THRESHOLD_DISTANCE and not self.obstacle_detected:
            self.obstacle_detected = True
            self.current_state = STATE_OBSTACLE_AVOIDANCE_START
            self.get_logger().info(f'Obstacle detected at {min_distance:.2f}m. Starting avoidance maneuver.')

    def is_green_light(self, hsv_image):
        """이미지 상단 영역에서 녹색 신호를 감지"""
        h, w, _ = hsv_image.shape
        # 신호등이 있을 법한 상단 1/4 영역만 사용
        roi = hsv_image[0:h//4, w//3:2*w//3]
        
        green_mask = cv2.inRange(roi, LOWER_GREEN, UPPER_GREEN)
        green_pixels = cv2.countNonZero(green_mask)
        
        # 특정 픽셀 수 이상이면 녹색 신호로 판단
        return green_pixels > 500

    def get_lane_and_cone_steering(self, bgr_image, hsv_image):
        """차선과 라바콘을 동시에 인식하여 조향각을 계산"""
        
        # 1. Bird's-Eye View 변환
        bev_image, M_inv = self.perspective_transform(bgr_image)
        bev_hsv = cv2.cvtColor(bev_image, cv2.COLOR_BGR2HSV)
        
        # 2. 색상 마스크 생성
        orange_mask = cv2.inRange(bev_hsv, LOWER_ORANGE, UPPER_ORANGE)
        white_mask = cv2.inRange(bev_hsv, LOWER_WHITE, UPPER_WHITE)
        yellow_mask = cv2.inRange(bev_hsv, LOWER_YELLOW, UPPER_YELLOW)
        lane_mask = cv2.bitwise_or(white_mask, yellow_mask)

        # 3. 라바콘 중심점 찾기
        cone_center_x = self.find_cone_center(orange_mask)
        
        # 4. 차선 중심점 찾기
        lane_center_x = self.find_lane_center(lane_mask)

        # 5. 최종 주행 경로 중심 계산
        # 라바콘과 차선이 모두 인식되면, 둘의 평균점을 중심으로 주행
        if cone_center_x is not None and lane_center_x is not None:
            center_x = (cone_center_x + lane_center_x) / 2
        elif cone_center_x is not None:
            center_x = cone_center_x
        elif lane_center_x is not None:
            center_x = lane_center_x
        else:
            # 아무것도 인식되지 않으면 직진
            return 0.0
            
        # 6. 조향각 계산
        # 화면 중심과의 오차(error)를 계산하여 조향각으로 변환
        error = center_x - (WIDTH / 2)
        steering_angle = np.clip(error * STEERING_GAIN / (WIDTH / 2), -1.0, 1.0)
        
        return steering_angle

    def perspective_transform(self, image):
        """이미지를 Bird's-Eye View로 변환"""
        src = np.float32([
            (WIDTH * 0.15, HEIGHT * 0.8),
            (WIDTH * 0.85, HEIGHT * 0.8),
            (0, HEIGHT * 0.5),
            (WIDTH, HEIGHT * 0.5)
        ])
        dst = np.float32([
            (WIDTH * 0.1, HEIGHT),
            (WIDTH * 0.9, HEIGHT),
            (WIDTH * 0.1, 0),
            (WIDTH * 0.9, 0)
        ])
        M = cv2.getPerspectiveTransform(src, dst)
        M_inv = cv2.getPerspectiveTransform(dst, src)
        warped_img = cv2.warpPerspective(image, M, (WIDTH, HEIGHT))
        return warped_img, M_inv

    def find_cone_center(self, mask):
        """주황색 마스크에서 라바콘들의 중심점을 찾음"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        left_cones, right_cones = [], []
        center_line = WIDTH / 2

        for cnt in contours:
            if cv2.contourArea(cnt) < 100: continue
            M = cv2.moments(cnt)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                if cx < center_line:
                    left_cones.append(cx)
                else:
                    right_cones.append(cx)
        
        if not left_cones or not right_cones:
            return None

        # 가장 안쪽의 좌/우 라바콘의 중심을 주행 경로로 설정
        avg_left = max(left_cones)
        avg_right = min(right_cones)
        return (avg_left + avg_right) / 2

    def find_lane_center(self, mask):
        """흰색/노란색 마스크에서 차선의 중심점을 찾음"""
        # 마스크의 하단 절반 영역의 x축 픽셀 분포를 분석
        hist = np.sum(mask[mask.shape[0]//2:, :], axis=0)
        midpoint = np.int32(hist.shape[0] / 2)
        
        left_base = np.argmax(hist[:midpoint])
        right_base = np.argmax(hist[midpoint:]) + midpoint

        # 차선이 한쪽만 인식될 경우를 대비
        if hist[left_base] < 500 and hist[right_base] < 500: # 인식된 픽셀이 너무 적으면 무시
             return None
        if hist[left_base] > 500 and hist[right_base] < 500: # 왼쪽만 인식
             return left_base + 320 # 차선 폭을 320px로 가정하고 오른쪽으로 이동
        if hist[left_base] < 500 and hist[right_base] > 500: # 오른쪽만 인식
             return right_base - 320 # 차선 폭을 320px로 가정하고 왼쪽으로 이동

        return (left_base + right_base) / 2
        
    def publish_drive_command(self, speed, steering_angle):
        """계산된 속도와 조향각으로 제어 메시지를 발행"""
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = speed
        msg.drive.steering_angle = steering_angle
        self.drive_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Autoracer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
