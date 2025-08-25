#!/usr/bin/env python3

import cv2

pipeline = (
    'nvarguscamerasrc ! '
    'video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! '
    'nvvidconv ! video/x-raw, format=BGRx ! '
    'videoconvert ! video/x-raw, format=BGR ! appsink'
)

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Failed to open camera.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame.")
        break
    cv2.imshow("Camera", frame)
    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()
~                                             
