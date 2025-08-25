# -*- coding: utf-8 -*-

import cv2

# 웹캠 열기 (0은 기본 카메라)
cap = cv2.VideoCapture(0)


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

