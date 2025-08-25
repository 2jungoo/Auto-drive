import cv2
import numpy as np
import RPi.GPIO as GPIO  # Jetson Nano는 Jetson.GPIO 사용 가능
import time

# ------------------ RC카 모터 핀 설정 ------------------
MOTOR_LEFT_FORWARD = 18
MOTOR_LEFT_BACKWARD = 23
MOTOR_RIGHT_FORWARD = 24
MOTOR_RIGHT_BACKWARD = 25

GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_LEFT_FORWARD, GPIO.OUT)
GPIO.setup(MOTOR_LEFT_BACKWARD, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_FORWARD, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_BACKWARD, GPIO.OUT)

pwm_left_f = GPIO.PWM(MOTOR_LEFT_FORWARD, 100)
pwm_left_b = GPIO.PWM(MOTOR_LEFT_BACKWARD, 100)
pwm_right_f = GPIO.PWM(MOTOR_RIGHT_FORWARD, 100)
pwm_right_b = GPIO.PWM(MOTOR_RIGHT_BACKWARD, 100)

pwm_left_f.start(0)
pwm_left_b.start(0)
pwm_right_f.start(0)
pwm_right_b.start(0)

# ------------------ RC카 제어 함수 ------------------
def stop_car():
    pwm_left_f.ChangeDutyCycle(0)
    pwm_left_b.ChangeDutyCycle(0)
    pwm_right_f.ChangeDutyCycle(0)
    pwm_right_b.ChangeDutyCycle(0)
    print("차 정지")

def drive_forward(speed=50):
    pwm_left_f.ChangeDutyCycle(speed)
    pwm_left_b.ChangeDutyCycle(0)
    pwm_right_f.ChangeDutyCycle(speed)
    pwm_right_b.ChangeDutyCycle(0)
    print(f"차 주행 - 속도 {speed}%")

# ------------------ 카메라 열기 ------------------
cap = cv2.VideoCapture(0)

# ------------------ HSV 색 범위 ------------------
# 빨강
lower_red1 = np.array([0,100,100])
upper_red1 = np.array([10,255,255])
lower_red2 = np.array([160,100,100])
upper_red2 = np.array([179,255,255])

# 초록
lower_green = np.array([40,50,50])
upper_green = np.array([90,255,255])

# 주황
lower_orange = np.array([10,100,100])
upper_orange = np.array([25,255,255])

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        red_mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        orange_mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # ------------------ 신호등 감지 ------------------
        if cv2.countNonZero(red_mask) > 500:
            stop_car()
            cv2.putText(frame, "RED - STOP", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
        elif cv2.countNonZero(orange_mask) > 500:
            drive_forward(speed=20)  # 주황색 감속 주행
            cv2.putText(frame, "ORANGE - SLOW", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,165,255), 2)
        elif cv2.countNonZero(green_mask) > 500:
            drive_forward(speed=60)  # 초록색 정상 주행
            cv2.putText(frame, "GREEN - GO", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        else:
            stop_car()
            cv2.putText(frame, "NO SIGNAL - STOP", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)

        # ------------------ 영상 출력 ------------------
        cv2.imshow("Traffic Light Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # ------------------ 종료 시 GPIO 정리 ------------------
    stop_car()
    pwm_left_f.stop()
    pwm_left_b.stop()
    pwm_right_f.stop()
    pwm_right_b.stop()
    GPIO.cleanup()
    cap.release()
    cv2.destroyAllWindows()
