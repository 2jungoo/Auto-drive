#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import threading
import time
import math

class GreenLightDetector(Node):
    def __init__(self):
        super().__init__('greenlight_detector')
        
        # 이미지 데이터
        self.current_image = None
        self.processed_frame = None
        self.image_lock = threading.Lock()
        
        # 녹색 신호등 검출 상태
        self.green_detected = False
        self.green_confidence = 0.0
        self.green_detection_count = 0
        self.consecutive_green_frames = 0
        self.start_command_given = False
        
        # 가상 속도 (실제 발행하지 않음)
        self.virtual_speed = 0.0
        self.target_virtual_speed = 0.0
        self.speed_ramp_rate = 0.02  # 속도 증가율
        
        # 성능 데이터
        self.frame_count = 0
        self.start_time = time.time()
        self.camera_fps = 0
        self.last_camera_time = 0
        
        # ROS2 구독자
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10
        )
        
        # 제어 루프 타이머 (디버깅용)
        self.control_timer = self.create_timer(0.05, self.debug_control_loop)
        
        self.get_logger().info('🚦 Green Light Detector Test Started!')
        self.get_logger().info('📹 Looking for green traffic light to start...')

    def image_callback(self, msg):
        """이미지 콜백"""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if image is not None:
                with self.image_lock:
                    self.current_image = image.copy()
                
                self.detect_green_light(image)
                
                # FPS 계산
                self.frame_count += 1
                current_time = time.time()
                if self.last_camera_time > 0:
                    fps = 1.0 / (current_time - self.last_camera_time)
                    self.camera_fps = round(fps, 1)
                self.last_camera_time = current_time
                
        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

    def detect_green_light(self, image):
        """녹색 신호등 검출"""
        processed = image.copy()
        height, width = image.shape[:2]
        
        # 상태 헤더 그리기
        self.draw_status_header(processed)
        
        # HSV 변환
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 녹색 범위 설정 (여러 범위로 더 정확한 검출)
        # 밝은 녹색 (신호등)
        lower_green1 = np.array([40, 120, 120])
        upper_green1 = np.array([80, 255, 255])
        
        # 어두운 녹색 (그림자가 있는 신호등)
        lower_green2 = np.array([35, 80, 80])
        upper_green2 = np.array([85, 200, 200])
        
        # LED 특성을 고려한 밝은 녹색
        lower_green3 = np.array([50, 150, 180])
        upper_green3 = np.array([70, 255, 255])
        
        # 마스크 생성 및 결합
        green_mask1 = cv2.inRange(hsv, lower_green1, upper_green1)
        green_mask2 = cv2.inRange(hsv, lower_green2, upper_green2)
        green_mask3 = cv2.inRange(hsv, lower_green3, upper_green3)
        
        green_mask = cv2.bitwise_or(green_mask1, green_mask2)
        green_mask = cv2.bitwise_or(green_mask, green_mask3)
        
        # 노이즈 제거
        kernel_small = np.ones((3, 3), np.uint8)
        kernel_medium = np.ones((5, 5), np.uint8)
        
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel_small, iterations=2)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel_medium, iterations=1)
        
        # 관심 영역 설정 (화면 상단 50% - 신호등 위치)
        roi_mask = np.zeros_like(green_mask)
        roi_mask[0:int(height*0.5), :] = 255
        green_mask = cv2.bitwise_and(green_mask, roi_mask)
        
        # 컨투어 검출
        contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 신호등 후보 필터링
        green_lights = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 50:  # 최소 면적
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w) / h
                
                # 신호등 형태 검증
                # 1. 원형 또는 정사각형에 가까운 형태
                # 2. 화면 상단에 위치
                # 3. 적절한 크기
                center_x = x + w // 2
                center_y = y + h // 2
                
                # 원형도 계산
                perimeter = cv2.arcLength(contour, True)
                if perimeter > 0:
                    circularity = 4 * math.pi * area / (perimeter * perimeter)
                    
                    # 신호등 조건 검사
                    if (0.5 < aspect_ratio < 2.0 and      # 원형/정사각형
                        w > 8 and h > 8 and               # 최소 크기
                        center_y < height * 0.5 and       # 화면 상단
                        circularity > 0.3 and             # 원형에 가까움
                        area > 80):                       # 충분한 면적
                        
                        # 밝기 검증 (신호등은 밝아야 함)
                        roi_brightness = cv2.mean(image[y:y+h, x:x+w])[0]
                        
                        if roi_brightness > 100:  # 충분히 밝음
                            confidence = area * circularity * (roi_brightness / 255.0)
                            
                            green_lights.append({
                                'x': x, 'y': y, 'w': w, 'h': h,
                                'center_x': center_x,
                                'center_y': center_y,
                                'area': area,
                                'confidence': confidence,
                                'brightness': roi_brightness,
                                'circularity': circularity
                            })
                            
                            # 시각화
                            cv2.rectangle(processed, (x, y), (x + w, y + h), (0, 255, 0), 3)
                            cv2.putText(processed, f'GREEN({area:.0f})', (x, y - 10), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            cv2.circle(processed, (center_x, center_y), 5, (0, 255, 0), -1)
                            
                            # 신뢰도 표시
                            cv2.putText(processed, f'Conf:{confidence:.1f}', (x, y + h + 15), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        
        # 가장 신뢰도 높은 녹색 신호등 선택
        if len(green_lights) > 0:
            best_green = max(green_lights, key=lambda x: x['confidence'])
            self.green_confidence = min(100, best_green['confidence'] * 0.5)
            
            # 연속 검출 프레임 증가
            self.consecutive_green_frames += 1
            self.green_detection_count += 1
            
            # 10프레임 이상 연속 검출 시 신호등 확정
            if self.consecutive_green_frames >= 10:
                self.green_detected = True
                
                # 시작 명령 (한 번만)
                if not self.start_command_given:
                    self.start_command_given = True
                    self.target_virtual_speed = 0.5  # 목표 속도 설정
                    self.get_logger().info('🚦✅ GREEN LIGHT DETECTED! Starting virtual acceleration...')
            
            # 가장 확실한 신호등에 특별 표시
            x, y, w, h = best_green['x'], best_green['y'], best_green['w'], best_green['h']
            cv2.rectangle(processed, (x-5, y-5), (x+w+5, y+h+5), (255, 255, 0), 2)
            cv2.putText(processed, '🚦 BEST', (x, y - 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        else:
            # 녹색 신호등을 찾지 못한 경우
            self.consecutive_green_frames = max(0, self.consecutive_green_frames - 1)
            self.green_confidence = max(0, self.green_confidence - 5)
            
            if self.consecutive_green_frames == 0:
                self.green_detected = False
        
        # 마스크 시각화 (좌하단)
        mask_resized = cv2.resize(green_mask, (160, 120))
        mask_colored = cv2.applyColorMap(mask_resized, cv2.COLORMAP_SPRING)
        processed[height-130:height-10, 10:170] = mask_colored
        cv2.putText(processed, "GREEN MASK", (15, height-135), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # 검출 통계 표시
        cv2.putText(processed, f"🚦 GREEN LIGHTS: {len(green_lights)} detected | Consecutive: {self.consecutive_green_frames}", 
                   (10, height - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        with self.image_lock:
            self.processed_frame = processed.copy()

    def draw_status_header(self, image):
        """상태 정보 헤더 그리기"""
        height, width = image.shape[:2]
        
        # 반투명 배경
        overlay = image.copy()
        cv2.rectangle(overlay, (0, 0), (width, 140), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)
        
        # 테스트 경과 시간
        elapsed = time.time() - self.start_time
        time_str = f"{int(elapsed//60):02d}:{int(elapsed%60):02d}"
        
        # 상태에 따른 색상
        if self.green_detected:
            status_color = (0, 255, 0)  # 초록
            status_text = "✅ GREEN DETECTED - READY TO GO!"
        elif self.green_confidence > 30:
            status_color = (0, 255, 255)  # 노랑
            status_text = "🔍 GREEN SIGNAL DETECTED..."
        else:
            status_color = (0, 0, 255)  # 빨강
            status_text = "🔴 WAITING FOR GREEN LIGHT"
        
        # 텍스트 정보
        cv2.putText(image, f'🚦 Green Light Detection Test | Status: {status_text}', 
                   (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        cv2.putText(image, f'⏱️ Time: {time_str} | Frame: {self.frame_count} | FPS: {self.camera_fps}', 
                   (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(image, f'🎯 Confidence: {self.green_confidence:.1f}% | Consecutive Frames: {self.consecutive_green_frames}', 
                   (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(image, f'🚗 Virtual Speed: {self.virtual_speed:.2f} m/s | Target: {self.target_virtual_speed:.2f} m/s', 
                   (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        
        # 신호등 상태 LED
        led_size = 20
        led_x = width - 80
        
        # 빨간불 (항상 기본)
        cv2.circle(image, (led_x, 30), led_size, (0, 0, 100), -1)
        cv2.circle(image, (led_x, 30), led_size, (255, 255, 255), 2)
        
        # 노란불 (검출 중)
        yellow_color = (0, 255, 255) if self.green_confidence > 30 else (0, 100, 100)
        cv2.circle(image, (led_x, 65), led_size, yellow_color, -1)
        cv2.circle(image, (led_x, 65), led_size, (255, 255, 255), 2)
        
        # 초록불 (확정)
        green_color = (0, 255, 0) if self.green_detected else (0, 100, 0)
        cv2.circle(image, (led_x, 100), led_size, green_color, -1)
        cv2.circle(image, (led_x, 100), led_size, (255, 255, 255), 2)

    def debug_control_loop(self):
        """디버깅용 제어 루프 (실제 명령은 발행하지 않음)"""
        # 가상 속도 제어 (부드러운 가속)
        if self.green_detected and self.start_command_given:
            # 목표 속도까지 부드럽게 가속
            if self.virtual_speed < self.target_virtual_speed:
                self.virtual_speed += self.speed_ramp_rate
                if self.virtual_speed > self.target_virtual_speed:
                    self.virtual_speed = self.target_virtual_speed
        else:
            # 감속 또는 정지
            if self.virtual_speed > 0:
                self.virtual_speed -= self.speed_ramp_rate * 2  # 더 빠른 감속
                if self.virtual_speed < 0:
                    self.virtual_speed = 0.0
        
        # 로그 출력 (5초마다)
        if hasattr(self, 'last_log_time'):
            if time.time() - self.last_log_time > 5.0:
                self.print_debug_status()
                self.last_log_time = time.time()
        else:
            self.last_log_time = time.time()

    def print_debug_status(self):
        """디버깅 상태 출력"""
        status_msg = f"""
╔══════════════════════════════════════╗
║        🚦 GREEN LIGHT TEST STATUS     ║
╠══════════════════════════════════════╣
║ Green Detected: {'✅ YES' if self.green_detected else '❌ NO':<20} ║
║ Confidence: {self.green_confidence:>22.1f}% ║
║ Consecutive Frames: {self.consecutive_green_frames:>14} ║
║ Virtual Speed: {self.virtual_speed:>17.2f} m/s ║
║ Start Command: {'✅ GIVEN' if self.start_command_given else '❌ WAITING':<20} ║
║ Total Detections: {self.green_detection_count:>16} ║
║ Camera FPS: {self.camera_fps:>22.1f} ║
╚══════════════════════════════════════╝
        """
        self.get_logger().info(status_msg)
        
        # 시작 명령이 주어진 경우 추가 정보
        if self.start_command_given:
            acceleration_status = f"🚀 ACCELERATING: {self.virtual_speed:.2f}/{self.target_virtual_speed:.2f} m/s"
            if self.virtual_speed >= self.target_virtual_speed:
                acceleration_status = f"🏁 MAX SPEED REACHED: {self.virtual_speed:.2f} m/s"
            self.get_logger().info(acceleration_status)

    def run_test(self):
        """테스트 실행 (OpenCV 윈도우로 실시간 표시)"""
        self.get_logger().info("🖼️  Starting OpenCV display window...")
        
        while rclpy.ok():
            try:
                # ROS2 스핀 (non-blocking)
                rclpy.spin_once(self, timeout_sec=0.01)
                
                # 이미지 표시
                with self.image_lock:
                    if self.processed_frame is not None:
                        # 윈도우 크기 조정 (보기 편하게)
                        display_frame = cv2.resize(self.processed_frame, (1024, 768))
                        cv2.imshow('🚦 Green Light Detection Test', display_frame)
                        
                        key = cv2.waitKey(1) & 0xFF
                        if key == ord('q') or key == 27:  # 'q' 또는 ESC
                            break
                        elif key == ord('r'):  # 'r' 키로 리셋
                            self.reset_detection()
                            self.get_logger().info("🔄 Detection reset!")
                            
            except KeyboardInterrupt:
                break
                
        cv2.destroyAllWindows()

    def reset_detection(self):
        """검출 상태 리셋"""
        self.green_detected = False
        self.green_confidence = 0.0
        self.consecutive_green_frames = 0
        self.start_command_given = False
        self.virtual_speed = 0.0
        self.target_virtual_speed = 0.0
        self.green_detection_count = 0

def main(args=None):
    rclpy.init(args=args)
    
    try:
        detector = GreenLightDetector()
        
        print("""
╔═══════════════════════════════════════════════╗
║          🚦 GREEN LIGHT DETECTION TEST         ║
╠═══════════════════════════════════════════════╣
║  • Looking for green traffic lights           ║
║  • Press 'q' or ESC to quit                   ║
║  • Press 'r' to reset detection               ║
║  • Virtual speed will increase upon detection ║
║  • No actual robot commands are sent          ║
╚═══════════════════════════════════════════════╝
        """)
        
        # 테스트 실행
        detector.run_test()
        
    except KeyboardInterrupt:
        print("\n🏁 Green Light Detection Test Ended!")
    finally:
        if 'detector' in locals():
            detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
