 -*- coding: utf-8 -*-

import cv2

# 웹캠 열기 (0은 기본 카메라)
#url = "rtspsrc latency=10 location=rtsp://192.168.200.150:5005/routecam ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink"
#url = 'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=3280, height=2464, format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink'

#cap = cv2.VideoCapture(url, cv2.CAP_GSTREAMER)
cap = cv2.VideoCapture('nvarguscamerasrc ! video/x-raw(memory:NVMM), width=3280, height=2464, format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink', cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("❌ 웹캠을 열 수 없습니다.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ 프레임을 읽을 수 없습니다.")
        break

    # 화면에 표시
    cv2.imshow('YOLOv8 Webcam Detection', frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 정리
cap.release()
cv2.destroyAllWindows()
