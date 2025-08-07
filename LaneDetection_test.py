import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np

qos_profile = QoSProfile(depth=10)
qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

class Autoracer(Node):
    def __init__(self):
        super().__init__('Autoracer')
        self.img_sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10)
        self.img_sub  # prevent unused variable warning
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile
        )
        self.lidar_sub  # prevent unused variable warning
        
        # Lane detection parameters
        self.img_width = 1280
        self.img_height = 720
        # 라이다로 가려지는 부분 (하단 5/17)
        self.lidar_blocked_height = int(self.img_height * 5 / 17)
        self.roi_height = self.img_height - self.lidar_blocked_height
        
    def detect_white_lane(self, image):
        """흰색 차선 검출"""
        # HSV 색공간으로 변환
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 흰색 범위 (HSV)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])
        
        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        return white_mask
    
    def detect_yellow_lane(self, image):
        """노란색 차선 검출"""
        # HSV 색공간으로 변환
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 노란색 범위 (HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        return yellow_mask
    
    def apply_roi(self, image):
        """관심 영역(ROI) 적용 - 라이다 가려진 부분 제외"""
        height, width = image.shape[:2]
        
        # 사다리꼴 ROI 정의 (하단은 라이다 가림 부분 제외)
        roi_bottom = height - self.lidar_blocked_height - 10  # 여유분 10px
        roi_top = int(roi_bottom * 0.6)  # 상단은 60% 지점
        
        # 사다리꼴 꼭짓점 정의
        vertices = np.array([[
            [int(width * 0.1), roi_bottom],           # 좌하단
            [int(width * 0.4), roi_top],              # 좌상단
            [int(width * 0.6), roi_top],              # 우상단
            [int(width * 0.9), roi_bottom]            # 우하단
        ]], dtype=np.int32)
        
        # 마스크 생성
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, vertices, 255)
        
        # ROI 적용
        masked_image = cv2.bitwise_and(image, mask)
        return masked_image
    
    def detect_lanes_hough(self, image):
        """Hough Transform을 사용한 차선 검출"""
        # 그레이스케일 변환
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
            
        # 가우시안 블러 적용
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # ROI 적용
        roi_blur = self.apply_roi(blur)
        
        # Canny Edge Detection
        edges = cv2.Canny(roi_blur, 50, 150)
        
        # Hough Line Transform
        lines = cv2.HoughLinesP(
            edges,
            rho=1,              # 거리 해상도 (픽셀)
            theta=np.pi/180,    # 각도 해상도 (라디안)
            threshold=50,       # 최소 투표 수
            minLineLength=50,   # 최소 선분 길이
            maxLineGap=150      # 최대 선분 간격
        )
        
        return lines, edges
    
    def separate_left_right_lanes(self, lines, image):
        """검출된 선분을 좌/우 차선으로 분류"""
        if lines is None:
            return [], []
            
        img_center = self.img_width // 2
        left_lines = []
        right_lines = []
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            # 기울기 계산
            if x2 - x1 == 0:
                continue
            slope = (y2 - y1) / (x2 - x1)
            
            # 기울기가 너무 작으면 제외 (수평선 제거)
            if abs(slope) < 0.3:
                continue
                
            # 선분의 중점 계산
            center_x = (x1 + x2) // 2
            
            # 좌/우 분류
            if center_x < img_center and slope < 0:  # 좌측 차선 (음의 기울기)
                left_lines.append(line[0])
            elif center_x > img_center and slope > 0:  # 우측 차선 (양의 기울기)
                right_lines.append(line[0])
                
        return left_lines, right_lines
    
    def draw_lane_lines(self, image, left_lines, right_lines):
        """차선 그리기"""
        line_img = np.zeros_like(image)
        
        # 좌측 차선 (흰색)
        for line in left_lines:
            x1, y1, x2, y2 = line
            cv2.line(line_img, (x1, y1), (x2, y2), (255, 255, 255), 3)
            
        # 우측 차선 (흰색)
        for line in right_lines:
            x1, y1, x2, y2 = line
            cv2.line(line_img, (x1, y1), (x2, y2), (255, 255, 255), 3)
            
        # 원본 이미지와 합성
        result = cv2.addWeighted(image, 0.8, line_img, 1, 0)
        return result
    
    def process_lane_detection(self, image):
        """전체 차선 검출 프로세스"""
        # 흰색/노란색 차선 검출
        white_mask = self.detect_white_lane(image)
        yellow_mask = self.detect_yellow_lane(image)
        
        # 두 마스크 합성
        combined_mask = cv2.bitwise_or(white_mask, yellow_mask)
        
        # 마스크를 원본 이미지에 적용
        masked_image = cv2.bitwise_and(image, image, mask=combined_mask)
        
        # Hough Transform으로 직선 검출
        lines, edges = self.detect_lanes_hough(combined_mask)
        
        # 좌/우 차선 분류
        left_lines, right_lines = self.separate_left_right_lanes(lines, image)
        
        # 결과 이미지 생성
        result_image = self.draw_lane_lines(image, left_lines, right_lines)
        
        # 디버그 정보 출력
        self.get_logger().info(f"Detected - Left lanes: {len(left_lines)}, Right lanes: {len(right_lines)}")
        
        return result_image, edges, combined_mask

    def image_callback(self, msg):
        try:
            # 압축된 이미지 디코딩
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if image is None:
                self.get_logger().error("Failed to decode image")
                return
                
            # 차선 검출 처리
            result_image, edges, mask = self.process_lane_detection(image)
            
            # ROI 시각화를 위한 사다리꼴 그리기
            roi_vis = result_image.copy()
            height, width = roi_vis.shape[:2]
            roi_bottom = height - self.lidar_blocked_height - 10
            roi_top = int(roi_bottom * 0.6)
            
            vertices = np.array([[
                [int(width * 0.1), roi_bottom],
                [int(width * 0.4), roi_top],
                [int(width * 0.6), roi_top],
                [int(width * 0.9), roi_bottom]
            ]], dtype=np.int32)
            
            cv2.polylines(roi_vis, vertices, True, (0, 255, 0), 2)
            
            # 라이다 가림 영역 표시
            cv2.rectangle(roi_vis, (0, height - self.lidar_blocked_height), 
                         (width, height), (0, 0, 255), 2)
            cv2.putText(roi_vis, "LiDAR Blocked Area", (10, height - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # 이미지 표시
            cv2.imshow("Lane Detection", roi_vis)
            cv2.imshow("Edges", edges)
            cv2.imshow("Color Mask", mask)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error in image processing: {str(e)}")

    def lidar_callback(self, msg):
        # LaserScan 메시지 수신 시 실행되는 콜백 함수
        total_points = len(msg.ranges)
    
        # 가운데 10개 인덱스 계산
        center = total_points // 2
        half_width = 10 // 2
    
        start_idx = max(0, center - half_width)
        end_idx = min(total_points, center + half_width)
    
        center_ranges = msg.ranges[start_idx:end_idx]
        # 소수점 3자리 포맷팅
        formatted_ranges = [f"{r:.3f}" for r in center_ranges]
        # 라이다 정보는 필요시에만 출력 (너무 많은 로그 방지)
        # self.get_logger().info(f"Center 10 ranges (3 decimals): {formatted_ranges}")

def main(args=None):
    rclpy.init(args=args)
    node = Autoracer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
