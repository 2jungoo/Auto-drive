#!/usr/bin/env python3
# -*- coding: utf-7 -*-

import threading, time
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, LaserScan, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from cv_bridge import CvBridge

import cv2
import numpy as np
import math


# =========================
# ⚙️ 런타임/성능 튜닝 파라미터
# =========================
FIXED_THROTTLE     = 0.40      # [0.0~1.0] 참고값: 0.40~0.55
FIXED_SPEED_MPS    = 0.35      # [m/s]
EMER_STOP_DIST     = 0.45      # [m] 전방 아주 가까우면 즉시 정지
USE_BOTTOM_CROP    = False      # 본네트 영역 제외(카메라 각도 좋으면 False 권장)

# 🔒 Lock-Follow & 디버그 스로틀링
LOCK_TTL_FRAMES      = 12    # 잠금 유지 프레임 수(≈0.4s @30Hz). 커브 많으면 8~10
REDETECT_MAX_PERIOD = 45    # 이 주기마다 최소 1번은 풀 재검출
DEBUG_EVERY         = 3     # 디버그/스트림 전송 간격(프레임). 3~5 권장
JPEG_QUALITY        = 60    # MJPEG 품질(낮출수록 CPU↓)

# (선택) 내부 처리 해상도 축소: 1.0=원본, 0.75 권장(지연↓)
PROC_SCALE          = 1.0

# =========================
# 색상 마스크 튜닝(현장 조도에 맞게 소폭 조정)
# =========================
# 흰 실선: HLS(밝고 저채도) + LAB(고휘도 중성색) 결합
WHITE_L_MIN = 180
WHITE_S_MAX = 110
LAB_L_MIN   = 200
LAB_A_DEV   = 12
LAB_B_DEV   = 12

# 노란 점선(HLS)
YELLOW_H_MIN, YELLOW_H_MAX = 15, 40
YELLOW_S_MIN = 80
YELLOW_L_MIN = 80

# 노란선 신뢰 픽셀 임계 (BEV에서)
YELLOW_PIX_MIN_WARP = 400
