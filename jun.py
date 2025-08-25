#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
자동차 주행 디버그 체커
- 주행 명령 발행 상태 확인
- 센서 데이터 수신 상태 확인
- 간단한 전진 테스트
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import time

class DrivingDebugChecker(Node):
    def __init__(self):
        super().__init__('driving_debug_checker')
        
        # QoS 설정
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.steer_pub = self.create_publisher(Float32, '/cmd/steering', 10)
        self.throt_pub = self.create_publisher(Float32, '/cmd/throttle', 10)
        
        # Subscribers
        self.img_sub = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.image_callback, qos)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, qos)
        
        # 상태 변수
        self.image_received = False
        self.lidar_received = False
        self.last_image_time = 0
        self.last_lidar_time = 0
        self.test_phase = 0
        self.start_time = time.time()
        
        # 타이머 (10Hz)
        self.timer = self.create_timer(0.1, self.debug_callback)
        
        self.get_logger().info("=== 자동차 주행 디버그 체커 시작 ===")
        self.get_logger().info("단계별 테스트를 진행합니다...")

    def image_callback(self, msg):
        self.image_received = True
        self.last_image_time = time.time()

    def lidar_callback(self, msg):
        self.lidar_received = True
        self.last_lidar_time = time.time()
        
        # 전방 거리 체크
        if len(msg.ranges) > 0:
            center = len(msg.ranges) // 2
            front_ranges = msg.ranges[max(0, center-10):min(len(msg.ranges), center+10)]
            valid_ranges = [r for r in front_ranges if 0.1 < r < 10.0]
            if valid_ranges:
                min_dist = min(valid_ranges)
                if min_dist < 0.5:  # 50cm 이내 장애물
                    self.get_logger().warn(f"⚠️  전방 장애물 감지: {min_dist:.2f}m")

    def send_drive_command(self, linear_speed, angular_speed, throttle=0.0, steering=0.0):
        """주행 명령 발송"""
        # Twist 메시지
        cmd = Twist()
        cmd.linear.x = float(linear_speed)
        cmd.angular.z = float(angular_speed)
        self.cmd_pub.publish(cmd)
        
        # 개별 제어 메시지
        if throttle != 0.0:
            self.throt_pub.publish(Float32(data=float(throttle)))
        if steering != 0.0:
            self.steer_pub.publish(Float32(data=float(steering)))
        
        self.get_logger().info(f"📤 명령 발송 - Speed: {linear_speed:.2f}, Turn: {angular_speed:.2f}, Throttle: {throttle:.2f}")

    def debug_callback(self):
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        # === 단계별 테스트 ===
        
        if self.test_phase == 0:  # 센서 확인 단계 (0-5초)
            if elapsed < 5.0:
                self.get_logger().info(f"🔍 센서 상태 확인 중... ({elapsed:.1f}/5.0초)")
                self.get_logger().info(f"   📷 카메라: {'✅ 수신됨' if self.image_received else '❌ 미수신'}")
                self.get_logger().info(f"   📡 라이다: {'✅ 수신됨' if self.lidar_received else '❌ 미수신'}")
                
                if self.image_received and self.lidar_received:
                    self.get_logger().info("✅ 모든 센서 정상 수신!")
                    self.test_phase = 1
                    self.start_time = current_time
            else:
                self.get_logger().warn("⚠️  센서 수신 시간 초과, 테스트 계속 진행...")
                self.test_phase = 1
                self.start_time = current_time
                
        elif self.test_phase == 1:  # 정지 명령 단계 (0-2초)
            if elapsed < 2.0:
                self.send_drive_command(0.0, 0.0, 0.0, 0.0)
                if elapsed < 1.0:
                    self.get_logger().info("🛑 정지 명령 발송 중...")
            else:
                self.test_phase = 2
                self.start_time = current_time
                
        elif self.test_phase == 2:  # 전진 테스트 단계 (0-3초)
            if elapsed < 3.0:
                self.send_drive_command(0.3, 0.0, 0.4, 0.0)  # 0.3m/s 전진
                self.get_logger().info(f"🚗 전진 테스트 중... ({elapsed:.1f}/3.0초)")
            else:
                self.test_phase = 3
                self.start_time = current_time
                
        elif self.test_phase == 3:  # 좌회전 테스트 (0-2초)
            if elapsed < 2.0:
                self.send_drive_command(0.2, 0.5, 0.3, 0.5)  # 좌회전하며 전진
                self.get_logger().info(f"↩️  좌회전 테스트 중... ({elapsed:.1f}/2.0초)")
            else:
                self.test_phase = 4
                self.start_time = current_time
                
        elif self.test_phase == 4:  # 우회전 테스트 (0-2초)
            if elapsed < 2.0:
                self.send_drive_command(0.2, -0.5, 0.3, -0.5)  # 우회전하며 전진
                self.get_logger().info(f"↪️  우회전 테스트 중... ({elapsed:.1f}/2.0초)")
            else:
                self.test_phase = 5
                self.start_time = current_time
                
        elif self.test_phase == 5:  # 정지 및 완료 (0-2초)
            if elapsed < 2.0:
                self.send_drive_command(0.0, 0.0, 0.0, 0.0)
                self.get_logger().info("🛑 테스트 완료, 정지 중...")
            else:
                self.test_phase = 6
                
        elif self.test_phase == 6:  # 결과 리포트
            self.get_logger().info("=" * 50)
            self.get_logger().info("🎯 주행 테스트 완료!")
            self.get_logger().info("=" * 50)
            self.get_logger().info("📊 테스트 결과:")
            self.get_logger().info(f"   📷 카메라 수신: {'✅' if self.image_received else '❌'}")
            self.get_logger().info(f"   📡 라이다 수신: {'✅' if self.lidar_received else '❌'}")
            self.get_logger().info("   🚗 주행 명령: ✅ 발송됨")
            self.get_logger().info("")
            self.get_logger().info("🔧 문제 해결 가이드:")
            if not self.image_received:
                self.get_logger().info("   📷 카메라: 카메라 노드 실행 확인 필요")
            if not self.lidar_received:
                self.get_logger().info("   📡 라이다: 라이다 노드 실행 확인 필요")
            self.get_logger().info("   🚗 자동차가 움직이지 않는다면:")
            self.get_logger().info("      - PCA9685 연결 확인")
            self.get_logger().info("      - 서보 모터 전원 확인")
            self.get_logger().info("      - cmd_vel_servo.py 노드 실행 확인")
            self.get_logger().info("=" * 50)
            
            # 연속 테스트 모드
            self.test_phase = 7
            self.start_time = current_time
            
        else:  # 연속 모니터링 모드
            if int(elapsed) % 5 == 0 and elapsed > 0:  # 5초마다
                self.get_logger().info("🔄 연속 모니터링 중...")
                self.send_drive_command(0.0, 0.0)  # 정지 유지


def main(args=None):
    rclpy.init(args=args)
    
    print("\n" + "="*60)
    print("🚗 자동차 주행 디버그 체커")
    print("="*60)
    print("이 프로그램은 다음을 확인합니다:")
    print("1. 센서 데이터 수신 상태")
    print("2. 주행 명령 발송 상태")
    print("3. 기본 주행 동작 테스트")
    print("="*60)
    print()
    
    node = DrivingDebugChecker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 디버그 체커 종료")
    finally:
        # 마지막에 정지 명령
        cmd = Twist()
        node.cmd_pub.publish(cmd)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
