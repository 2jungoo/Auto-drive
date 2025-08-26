#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Autoracer 2025 – Signal→Cone→Lane + Safe Lane-change (ROS2)
- Camera: /image_raw/compressed  (gscam → image_raw → republish → image_raw/compressed)
- Lidar  : /scan
- Cmd    : /cmd_vel
"""

# ================================
# 1) Imports & ROS QoS
# ================================
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist
import cv2, numpy as np, time, math, socket, threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from enum import Enum

qos_profile = QoSProfile(depth=10)
qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT


# ================================
# 2) Drive Modes
# ================================
class DriveMode(Enum):
    WAITING_FOR_GREEN   = "WAITING_FOR_GREEN"     # 신호 대기
    RUBBERCON_AVOIDANCE = "RUBBERCON_AVOID"       # 라바콘 구간
    LANE_FOLLOWING      = "LANE_FOLLOW"           # 차선 주행
    EMERGENCY_STOP      = "EMERGENCY_STOP"        # 응급정지


# ================================
# 3) Web Dashboard (mjpeg + stats)
# ================================
class WebViewer(BaseHTTPRequestHandler):
    def __init__(self, autoracer_node, *args, **kwargs):
        self.autoracer = autoracer_node
        super().__init__(*args, **kwargs)

    def do_GET(self):
        if self.path == '/':
            self.send_response(200); self.send_header('Content-Type','text/html'); self.end_headers()
            html = f"""
            <html><head><title>Autoracer 2025</title>
            <style>
              body{{background:#111;color:#eee;font-family:Segoe UI,Arial;margin:0;padding:20px}}
              .wrap{{max-width:1400px;margin:0 auto;display:flex;gap:20px}}
              .panel{{background:#1d1f20;border-radius:12px;padding:20px;flex:1}}
              img{{width:100%;border:1px solid #333;border-radius:8px}}
              h1{{color:#00ffa0;text-align:center}}
              .grid{{display:grid;grid-template-columns:1fr 1fr;gap:8px;font-size:14px}}
              .k{{opacity:.7}} .v{{text-align:right;font-weight:700;color:#0ff}}
            </style></head><body>
            <h1>🏁 Autoracer 2025 – Signal→Cone→Lane</h1>
            <div class="wrap">
              <div class="panel" style="flex:2">
                <h3>📹 Live</h3>
                <img src="/stream.mjpg">
              </div>
              <div class="panel" style="flex:1">
                <h3>🎛 Status</h3>
                <div class="grid" id="g"></div>
              </div>
            </div>
            <script>
              const g=document.getElementById('g');
              function row(k,v){return `<div class='k'>${k}</div><div class='v'>${v}</div>`}
              async function tick(){{
                try{{
                  const r=await fetch('/stats'); const s=await r.json();
                  g.innerHTML = [
                    row('Mode', s.current_mode),
                    row('Traffic', s.traffic_light),
                    row('Cone', s.rubbercon_status),
                    row('Lane', s.lane_status),
                    row('Conf', s.detection_confidence+'%'),
                    row('FPS', s.camera_fps),
                    row('Lidar(min front)', s.lidar_distance+' m'),
                    row('Speed', s.speed+' m/s'),
                    row('Steer', s.steering_angle+'°')
                  ].join('');
                }}catch(e){{}}
              }}
              setInterval(tick,500); tick();
            </script>
            </body></html>
            """
            self.wfile.write(html.encode('utf-8'))

        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    frame = self.autoracer.get_processed_frame()
                    if frame is not None:
                        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                        self.wfile.write(b'--frame\r\n')
                        self.send_header('Content-Type','image/jpeg')
                        self.send_header('Content-Length', str(len(buf)))
                        self.end_headers()
                        self.wfile.write(buf.tobytes())
                        self.wfile.write(b'\r\n')
                    time.sleep(0.033)
            except Exception as e:
                self.autoracer.get_logger().error(f'stream error: {e}')

        elif self.path == '/stats':
            self.send_response(200); self.send_header('Content-Type','application/json'); self.end_headers()
            import json; self.wfile.write(json.dumps(self.autoracer.get_stats()).encode())
        else:
            self.send_response(404); self.end_headers()


# ================================
# 4) Core Node
# ================================
class Autoracer(Node):
    def __init__(self):
        super().__init__('Autoracer')

        # --- State: images / lidar
        self.current_image   = None
        self.processed_frame = None
        self.image_lock      = threading.Lock()
        self.lidar_data      = None

        # --- Mission state
        self.current_mode          = DriveMode.WAITING_FOR_GREEN
        self.mission_start_time    = time.time()
        self.rubbercon_passed      = False
        self.lane_detected         = False

        # --- Control variables
        self.current_speed    = 0.0
        self.current_steering = 0.0
        self.target_speed     = 0.0
        self.target_steering  = 0.0
        self.prev_error       = 0.0
        self.integral_error   = 0.0

        # --- FPS / stats
        self.frame_count     = 0
        self.camera_fps      = 0.0
        self.last_camera_t   = 0.0
        self.detection_confidence = 0.0

        # --- BEV and lane model
        self.bev_matrix = None
        self.inv_bev_matrix = None
        self.setup_bev_transform()
        self.prev_left_base, self.prev_right_base = 160, 480
        self.last_left_line  = None
        self.last_right_line = None
        self.last_lane_center= None
        self.lane_confidence = 0.0

        # --- TRAFFIC LIGHT (green start)
        self.tl_green_frames  = 0
        self.tl_color         = "NONE"
        self.MIN_GREEN_FRAMES = 5

        # --- Lane-change (obstacle-based, keep inside outer white lanes)
        self.lane_change_active   = False
        self.lane_change_dir      = None   # 'L' or 'R'
        self.lane_change_started  = 0.0
        self.LC_TRIGGER_DIST      = 0.60   # m, 앞 장애물 트리거
        self.LC_CLEAR_DIST        = 0.90   # m, 복귀 조건
        self.LC_DURATION_MAX      = 3.0    # s, 안정 타임아웃
        self.LC_OFFSET_PX         = 90     # px, 차선 내 목표 오프셋
        self.LANE_MARGIN_PX       = 25     # 흰 외곽선 안쪽 안전여유

        # --- ROS I/O
        self.image_sub = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.image_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, qos_profile)
        self.cmd_pub   = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Web server
        self.web_port = 8080
        self.start_web_server()

        # --- Control loop (20Hz)
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info('🚦 Waiting GREEN → 🚧 Cone → 🛣 Lane')
        self.get_logger().info(f'🌐 Dashboard: http://{self.get_ip_address()}:{self.web_port}/')

    # -------------------------
    # Utils
    # -------------------------
    def get_ip_address(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(('8.8.8.8', 80)); ip=s.getsockname()[0]; s.close(); return ip
        except: return 'localhost'

    def start_web_server(self):
        def handler(*args, **kw): return WebViewer(self, *args, **kw)
        def run():
            for port in range(8080,8090):
                try:
                    srv = HTTPServer(('0.0.0.0',port), handler)
                    self.web_port = port
                    self.get_logger().info(f'🌐 Web @ {self.get_ip_address()}:{port}')
                    srv.serve_forever()
                    break
                except OSError as e:
                    if getattr(e,'errno',None)==98: continue
                    self.get_logger().error(f'web err: {e}'); break
        threading.Thread(target=run, daemon=True).start()

    def setup_bev_transform(self):
        src = np.float32([[80,480],[560,480],[240,280],[400,280]])
        dst = np.float32([[150,480],[490,480],[150,  0],[490,  0]])
        self.bev_matrix    = cv2.getPerspectiveTransform(src,dst)
        self.inv_bev_matrix= cv2.getPerspectiveTransform(dst,src)

    # -------------------------
    # ROS Callbacks
    # -------------------------
    def image_callback(self, msg: CompressedImage):
        try:
            img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if img is None: return
            with self.image_lock: self.current_image = img.copy()
            self.process_image(img)

            # FPS
            self.frame_count += 1
            now = time.time()
            if self.last_camera_t>0:
                self.camera_fps = round(1.0/(now-self.last_camera_t),1)
            self.last_camera_t = now
        except Exception as e:
            self.get_logger().error(f'Image error: {e}')

    def lidar_callback(self, msg: LaserScan):
        # 저장만 하고, 판단은 control_loop에서 일괄 처리
        self.lidar_data = msg

    # -------------------------
    # High-level pipeline
    # -------------------------
    def process_image(self, image):
        vis = image.copy()

        # 1) Status header
        self.draw_status_header(vis)

        # 2) Mode-specific processing
        if self.current_mode == DriveMode.WAITING_FOR_GREEN:
            self.detect_green_light(vis)

        elif self.current_mode == DriveMode.RUBBERCON_AVOIDANCE:
            self.detect_and_avoid_rubbercon(vis)

        elif self.current_mode == DriveMode.LANE_FOLLOWING:
            self.detect_lane_and_follow(vis)

        # 3) Lidar overlay (mini HUD)
        self.draw_lidar_overlay(vis)

        with self.image_lock:
            self.processed_frame = vis

    # -------------------------
    # Drawing / HUD
    # -------------------------
    def draw_status_header(self, img):
        h,w = img.shape[:2]
        overlay = img.copy()
        cv2.rectangle(overlay,(0,0),(w,110),(0,0,0),-1)
        cv2.addWeighted(overlay,0.65,img,0.35,0,img)
        elapsed = int(time.time()-self.mission_start_time)
        tstr = f"{elapsed//60:02d}:{elapsed%60:02d}"
        cv2.putText(img, f"Mode: {self.current_mode.value}  |  Time: {tstr}  |  FPS: {self.camera_fps}",
                    (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,160),2)
        cv2.putText(img, f"Speed: {self.current_speed:.2f} m/s  |  Steer: {math.degrees(self.current_steering):.1f} deg  |  Conf: {self.detection_confidence:.1f}%",
                    (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255),1)
        # traffic light LEDs (top-right)
        lx = w-55; s=18; gap=28
        red   = (0,0,255)   if self.tl_color=="RED"   else (40,0,0)
        orange= (0,165,255) if self.tl_color=="ORANGE" else (40,40,0)
        green = (0,255,0)   if self.tl_color=="GREEN" else (0,40,0)
        cv2.circle(img,(lx,22), s, red,  -1); cv2.circle(img,(lx,22), s,(255,255,255),2)
        cv2.circle(img,(lx,22+gap), s, orange,-1); cv2.circle(img,(lx,22+gap), s,(255,255,255),2)
        cv2.circle(img,(lx,22+2*gap), s, green,-1); cv2.circle(img,(lx,22+2*gap), s,(255,255,255),2)

    # -------------------------
    # (A) Traffic light (GREEN start)
    # -------------------------
    def detect_green_light(self, vis):
        img = vis
        h,w = img.shape[:2]
        roi = img[:h//3, :]                              # 상단 1/3
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        ranges = {
            "RED":[(np.array([0,120,120]),np.array([10,255,255])),
                   (np.array([170,120,120]),np.array([180,255,255]))],
            "ORANGE":[(np.array([10,150,150]),np.array([25,255,255]))],
            "GREEN":[(np.array([40,120,120]),np.array([80,255,255])),
                     (np.array([50,150,180]),np.array([70,255,255]))]
        }

        best = {"color":"NONE","conf":0.0}
        for color, lst in ranges.items():
            m=None
            for lo,hi in lst:
                if m is None: m=cv2.inRange(hsv,lo,hi)
                else: m=cv2.bitwise_or(m, cv2.inRange(hsv,lo,hi))
            k = np.ones((5,5),np.uint8)
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k, iterations=2)
            m = cv2.morphologyEx(m, cv2.MORPH_CLOSE,k, iterations=1)

            cnts,_=cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                area = cv2.contourArea(c)
                if area<200: continue
                x,y,ww,hh = cv2.boundingRect(c)
                per = cv2.arcLength(c,True)
                circ= 4*math.pi*area/(per*per) if per>0 else 0
                if 0.3<circ and 0.5<ww/max(1,hh)<2.0:
                    bright = cv2.mean(roi[y:y+hh, x:x+ww])[0]
                    conf = area * circ * (bright/255.0) * 0.1
                    if conf>best["conf"]:
                        best={"color":color,"conf":conf,"box":(x,y,ww,hh)}

        self.tl_color = best["color"]
        if best["color"]!="NONE":
            x,y,ww,hh = best["box"]
            col = (0,255,0) if best["color"]=="GREEN" else ((0,0,255) if best["color"]=="RED" else (0,165,255))
            cv2.rectangle(roi,(x,y),(x+ww,y+hh), col, 3)
            cv2.putText(roi, f"{best['color']} {best['conf']:.1f}", (x,y-8), cv2.FONT_HERSHEY_SIMPLEX,0.6,col,2)

        # GREEN start logic
        if self.tl_color=="GREEN":
            self.tl_green_frames += 1
        else:
            self.tl_green_frames = max(0, self.tl_green_frames-1)

        if self.tl_green_frames >= self.MIN_GREEN_FRAMES:
            self.get_logger().info("🟢 GREEN confirmed → start & go to CONE mission")
            self.current_mode = DriveMode.RUBBERCON_AVOIDANCE
            self.mission_start_time = time.time()
            self.target_speed = 0.35

    # -------------------------
    # (B) Rubbercone avoidance (vision)
    # -------------------------
    def detect_and_avoid_rubbercon(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lo1,hi1 = np.array([5,100,100]),  np.array([25,255,255])
        lo2,hi2 = np.array([165,100,100]),np.array([180,255,255])
        lo3,hi3 = np.array([25,150,150]), np.array([35,255,255])

        m1=cv2.inRange(hsv,lo1,hi1); m2=cv2.inRange(hsv,lo2,hi2); m3=cv2.inRange(hsv,lo3,hi3)
        m = cv2.bitwise_or(cv2.bitwise_or(m1,m2),m3)

        h,w = img.shape[:2]
        roi = np.zeros_like(m); roi[int(h*0.3):,:]=255; m=cv2.bitwise_and(m,roi)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3,3),np.uint8), iterations=2)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE,np.ones((7,7),np.uint8), iterations=1)

        cnts,_=cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cones=[]
        for c in cnts:
            area=cv2.contourArea(c)
            if area<200: continue
            x,y,ww,hh = cv2.boundingRect(c)
            if hh<15 or ww<10: continue
            ar = ww/max(1,hh)
            cy = y+hh//2
            sol = area/(ww*hh)
            per = cv2.arcLength(c,True); comp = 4*math.pi*area/(per*per) if per>0 else 0
            if 0.3<ar<1.5 and cy>h*0.4 and sol>0.4 and comp>0.3:
                cones.append({
                    "x":x,"y":y,"w":ww,"h":hh,
                    "cx":x+ww//2,"cy":cy,"area":area,
                    "conf": area*sol*comp
                })
                cv2.rectangle(img,(x,y),(x+ww,y+hh),(0,165,255),2)
                cv2.circle(img,(x+ww//2,cy),5,(0,165,255),-1)

        cones.sort(key=lambda c:c["conf"], reverse=True)
        cones = cones[:4]
        self.detection_confidence = min(100, sum(c["conf"] for c in cones)/10.0) if cones else 0.0

        # 타겟 포인트 계산 (양 콘 사이 중점 우선)
        img_cx = w//2
        left  = [c for c in cones if c["cx"] < img_cx-40]
        right = [c for c in cones if c["cx"] > img_cx+40]

        if left and right:
            L=max(left,key=lambda c:c["conf"]); R=max(right,key=lambda c:c["conf"])
            tx = (L["cx"]+R["cx"])//2; ty = min(L["cy"],R["cy"])
            err = tx - img_cx
            cv2.circle(img,(tx,ty),10,(255,0,255),-1)
            cv2.line(img,(img_cx,h),(tx,ty),(255,0,255),2)
            self.pid_steer_speed(err, mode="RUBBERCON")
        elif left:
            tx = max(left,key=lambda c:c["conf"])["cx"] + 120
            self.pid_steer_speed(tx-img_cx, mode="RUBBERCON")
        elif right:
            tx = min(right,key=lambda c:c["conf"])["cx"] - 120
            self.pid_steer_speed(tx-img_cx, mode="RUBBERCON")
        else:
            # 콘이 안 보이면 카운트 올리다 통과 처리
            if not hasattr(self,'_cone_miss'): self._cone_miss=0
            self._cone_miss += 1
            if self._cone_miss>10:
                self.get_logger().info("✅ Cone section done → LANE_FOLLOW")
                self.current_mode = DriveMode.LANE_FOLLOWING
                self.mission_start_time = time.time()
                self._cone_miss = 0

        # 작은 마스크 미니뷰
        mini = cv2.applyColorMap(cv2.resize(m,(160,120)), cv2.COLORMAP_JET)
        img[10:130,10:170]=mini
        cv2.putText(img,"CONE",(12,145),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,165,255),1)

    # -------------------------
    # (C) Lane following (yellow preferred)
    # -------------------------
    def detect_lane_and_follow(self, img):
        # BEV
        if self.bev_matrix is not None:
            bev = cv2.warpPerspective(img, self.bev_matrix, (640,480))
        else:
            bev = img.copy()

        hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
        # 흰선
        white1 = cv2.inRange(hsv, np.array([0,0,200]),   np.array([180,25,255]))
        white2 = cv2.inRange(hsv, np.array([0,0,160]),   np.array([180,40,200]))
        # 노란 중앙선(가중)
        yellow = cv2.inRange(hsv, np.array([20,100,100]),np.array([30,255,255]))
        gray   = cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)
        _, bright = cv2.threshold(gray, 180,255, cv2.THRESH_BINARY)
        adaptive = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,15,-10)

        lane_mask = white1 | white2 | bright | adaptive
        lane_mask = cv2.bitwise_or(lane_mask, yellow)  # 중앙선도 포함
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_OPEN,  np.ones((3,3),np.uint8),1)
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8),2)

        # 슬라이딩 윈도우 (좌/우 흰선 기반)
        left,right,center = self.sliding_window_lane_detection_2023(lane_mask)

        # 중앙선(노란색) 픽셀 충분하면 우선 사용 (중앙선 추종)
        ypix = cv2.countNonZero(yellow)
        if ypix>600:
            ys, xs = np.nonzero(yellow)
            if xs.size>300:
                yfit = np.polyfit(ys, xs, 2)
                y_eval = lane_mask.shape[0]*3//4
                yx = yfit[0]*y_eval**2 + yfit[1]*y_eval + yfit[2]
                center = float(np.clip(yx, 0, lane_mask.shape[1]-1))
                # 노란선 우선시: 신뢰도 상향
                self.lane_confidence = max(self.lane_confidence, 80.0)

        self.last_left_line, self.last_right_line, self.last_lane_center = left, right, center
        self.draw_lane_overlay_2023(img, bev, lane_mask, left, right, center)

        if center is not None:
            img_cx = img.shape[1]//2
            err = center - img_cx

            # 장애물 기반 차선 변경(안전): 흰 외곽선 넘지 않게!!!
            # 라이다로 전방 최소거리 확인
            min_front = self.get_front_min_dist()
            if min_front is not None:
                now = time.time()
                if (not self.lane_change_active) and min_front < self.LC_TRIGGER_DIST and left is not None and right is not None:
                    # 좌/우 측면 여유 비교해서 방향 결정
                    side = self.choose_safer_side()
                    if side:
                        self.lane_change_active = True
                        self.lane_change_dir = side        # 'L' or 'R'
                        self.lane_change_started = now
                        self.get_logger().warn(f"↪ Lane-change start ({side}) – obstacle {min_front:.2f} m")

                if self.lane_change_active:
                    # 현재 차선 경계에서 margin 안넘도록 목표중심 보정
                    y_eval = lane_mask.shape[0]*3//4
                    lx = left[0]*y_eval**2 + left[1]*y_eval + left[2]  if left  is not None else 80
                    rx = right[0]*y_eval**2+ right[1]*y_eval+ right[2] if right is not None else 560

                    desired = center + (self.LC_OFFSET_PX if self.lane_change_dir=='R' else -self.LC_OFFSET_PX)
                    safe_desired = float(np.clip(desired, lx+self.LANE_MARGIN_PX, rx-self.LANE_MARGIN_PX))
                    err = safe_desired - img_cx

                    # 종료조건: 전방 시야가 확보되거나 타임아웃
                    clear = (min_front >= self.LC_CLEAR_DIST)
                    timeout = (now - self.lane_change_started > self.LC_DURATION_MAX)
                    if clear or timeout:
                        self.get_logger().info("↩ Lane-change end")
                        self.lane_change_active=False; self.lane_change_dir=None

            # PID 제어
            self.pid_steer_speed(err, mode="LANE")
            self.lane_detected = True
        else:
            # 차선 미검출 - 이전값 유지 + 탐색
            self.lane_detected = False
            self.pid_steer_speed(self.prev_error*0.5, mode="LANE")

    # -------------------------
    # Lane helpers
    # -------------------------
    def sliding_window_lane_detection_2023(self, binary_image):
        h,w = binary_image.shape
        hist = np.sum(binary_image[h*2//3:,:], axis=0)
        hist = np.convolve(hist, np.ones(10)/10, mode='same')
        mid = w//2

        def base_from(hist_slice, prev, is_left):
            if 0<prev<len(hist_slice):
                sr=50; s=max(0,prev-sr); e=min(len(hist_slice),prev+sr)
                loc = np.argmax(hist_slice[s:e]); val=hist_slice[s+loc]
                if val>100: return s+loc
            m = np.argmax(hist_slice)
            return m if hist_slice[m]>50 else (len(hist_slice)//2)

        left_base  = base_from(hist[:mid], self.prev_left_base, True)
        right_base = base_from(hist[mid:], self.prev_right_base-mid, False) + mid

        nwindows=12; wh=h//nwindows; margin=60; minpix=30
        leftx, rightx = left_base, right_base
        left_inds=[]; right_inds=[]; lc=0; rc=0

        nonzero = binary_image.nonzero()
        nz_y = np.array(nonzero[0]); nz_x = np.array(nonzero[1])

        for widx in range(nwindows):
            wy_low  = h-(widx+1)*wh; wy_high=h-widx*wh
            wl_low  = max(0, leftx - margin);  wl_high  = min(w, leftx + margin)
            wr_low  = max(0, rightx- margin);  wr_high  = min(w, rightx+ margin)
            good_left  = ((nz_y>=wy_low)&(nz_y<wy_high)&(nz_x>=wl_low)&(nz_x<wl_high)).nonzero()[0]
            good_right = ((nz_y>=wy_low)&(nz_y<wy_high)&(nz_x>=wr_low)&(nz_x<wr_high)).nonzero()[0]
            if len(good_left)>0:
                left_inds.append(good_left); lc+=len(good_left)
                if len(good_left)>minpix: leftx=int(np.mean(nz_x[good_left]))
            if len(good_right)>0:
                right_inds.append(good_right); rc+=len(good_right)
                if len(good_right)>minpix: rightx=int(np.mean(nz_x[good_right]))

        self.prev_left_base, self.prev_right_base = leftx, rightx

        left_line=right_line=None; center=None
        if lc>200 and len(left_inds)>5:
            ly = np.concatenate([nz_y[i] for i in left_inds]); lx = np.concatenate([nz_x[i] for i in left_inds])
            if len(lx)>100:
                try: left_line = np.polyfit(ly,lx,2)
                except: pass
        if rc>200 and len(right_inds)>5:
            ry = np.concatenate([nz_y[i] for i in right_inds]); rx = np.concatenate([nz_x[i] for i in right_inds])
            if len(rx)>100:
                try: right_line = np.polyfit(ry,rx,2)
                except: pass

        if left_line is not None and right_line is not None:
            y = h*3//4
            lx = left_line[0]*y**2 + left_line[1]*y + left_line[2]
            rx = right_line[0]*y**2+ right_line[1]*y + right_line[2]
            lw = abs(rx-lx)
            if 100<lw<400:
                center = (lx+rx)/2; self.lane_confidence = min(100,(lc+rc)/20)
            else:
                self.lane_confidence = 0
        elif left_line is not None:
            y=h*3//4; lx=left_line[0]*y**2+left_line[1]*y+left_line[2]
            center = lx + 160; self.lane_confidence=min(100,lc/15)
        elif right_line is not None:
            y=h*3//4; rx=right_line[0]*y**2+right_line[1]*y+right_line[2]
            center = rx - 160; self.lane_confidence=min(100,rc/15)
        else:
            self.lane_confidence = 0

        return left_line, right_line, center

    def draw_lane_overlay_2023(self, orig, bev, mask, left, right, center):
        H,W = bev.shape[:2]
        col = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        ys = np.linspace(0,H-1,H).astype(int)

        def poly_pts(fit):
            xs = (fit[0]*ys**2 + fit[1]*ys + fit[2]).astype(int)
            xs = np.clip(xs, 0, W-1)
            return np.array(list(zip(xs,ys)), np.int32)

        if left is not None:
            cv2.polylines(col, [poly_pts(left)], False,(255,100,100),8)
        if right is not None:
            cv2.polylines(col, [poly_pts(right)], False,(100,100,255),8)

        if left is not None and right is not None:
            l = (left[0]*ys**2 + left[1]*ys + left[2]).astype(int)
            r = (right[0]*ys**2+ right[1]*ys+ right[2]).astype(int)
            l = np.clip(l,0,W-1); r=np.clip(r,0,W-1)
            pts_left  = np.transpose(np.vstack([l,ys]))
            pts_right = np.flipud(np.transpose(np.vstack([r,ys])))
            pts = np.hstack((pts_left, pts_right))
            cv2.fillPoly(col, [pts.astype(np.int32)], (0,100,0))

        if center is not None:
            cx = int(np.clip(center, 0, W-1))
            cv2.line(col,(cx,H//2),(cx,H),(0,255,255),6)

        if self.inv_bev_matrix is not None:
            overlay = cv2.warpPerspective(col, self.inv_bev_matrix, (orig.shape[1], orig.shape[0]))
            m = (overlay.sum(axis=2)>0).astype(np.uint8)
            for c in range(3):
                orig[:,:,c] = np.where(m, cv2.addWeighted(orig[:,:,c],0.6, overlay[:,:,c],0.4,0), orig[:,:,c])

        mini = cv2.applyColorMap(cv2.resize(mask,(200,150)), cv2.COLORMAP_RAINBOW)
        x0 = orig.shape[1]-210; y0=130
        orig[y0:y0+150, x0:x0+200] = mini
        cv2.putText(orig,"BEV",(x0,y0-8),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)

    # -------------------------
    # Control
    # -------------------------
    def pid_steer_speed(self, error_px, mode="LANE"):
        # PID gains
        if mode=="RUBBERCON":  kp,ki,kd = 0.0035, 0.00008, 0.002;  max_steer=0.5;  base_speed=0.35
        elif mode=="LANE":     kp,ki,kd = 0.0025, 0.00005, 0.0018; max_steer=0.35; base_speed=0.4
        else:                  kp,ki,kd = 0.002,  0.0001,  0.001;   max_steer=0.3;  base_speed=0.2

        dt=0.05
        self.integral_error += error_px*dt
        self.integral_error = float(np.clip(self.integral_error, -800, 800))
        d = (error_px - self.prev_error)/dt
        out = -(kp*error_px + ki*self.integral_error + kd*d)

        # 속도-조향 연동 제한
        speed_factor = max(0.5, 1.0 - abs(self.current_speed)*0.3)
        max_steer *= speed_factor
        self.target_steering = float(np.clip(out, -max_steer, max_steer))
        self.prev_error = error_px

        # 속도: 조향 크기에 따라 가감
        s_mag = abs(self.target_steering)
        if mode=="LANE":
            base_speed = 0.7 if s_mag<0.10 else (0.25 if s_mag<0.25 else 0.10)
        self.target_speed = base_speed * (0.9 if s_mag>0.3 else 1.0)

    def control_loop(self):
        cmd = Twist()

        # Low-pass for smoothness
        a_v, a_w = 0.4, 0.6
        self.current_speed    = a_v*self.target_speed   + (1-a_v)*self.current_speed
        self.current_steering = a_w*self.target_steering+ (1-a_w)*self.current_steering

        # EMERGENCY STOP (very near)
        min_front = self.get_front_min_dist()
        if min_front is not None and min_front < 0.15:
            self.current_mode = DriveMode.EMERGENCY_STOP
        if self.current_mode == DriveMode.EMERGENCY_STOP:
            self.target_speed = self.current_speed = 0.0
            self.target_steering = self.current_steering = 0.0
            # 간단 복귀: 시야 확보되면 이전 단계로
            if min_front is None or min_front > 0.25:
                self.current_mode = DriveMode.RUBBERCON_AVOIDANCE if not self.rubbercon_passed else DriveMode.LANE_FOLLOWING

        cmd.linear.x  = float(np.clip(self.current_speed, 0.0, 0.8))
        cmd.angular.z = float(np.clip(self.current_steering, -0.6, 0.6))
        self.cmd_pub.publish(cmd)

    # -------------------------
    # Lidar helpers
    # -------------------------
    def get_front_min_dist(self):
        if self.lidar_data is None or len(self.lidar_data.ranges)==0: return None
        arr = np.array(self.lidar_data.ranges, dtype=float)
        n = len(arr); c=n//2; span = min(25, n//12)     # ±~15도
        seg = arr[c-span:c+span]
        seg = seg[(seg>0.05) & (seg<10.0)]
        return float(seg.min()) if seg.size else None

    def choose_safer_side(self):
        """좌/우 전방측 평균거리 비교로 안전한 방향 선택 ('L' or 'R')"""
        if self.lidar_data is None: return 'R'
        arr = np.array(self.lidar_data.ranges, dtype=float)
        n=len(arr); c=n//2; span=min(35, n//10)
        left  = arr[c+5:c+5+span]   # 약간 좌측
        right = arr[c-(5+span):c-5] # 약간 우측
        l = np.mean(left[(left>0.05)&(left<10.0)])  if left.size  else 0.0
        r = np.mean(right[(right>0.05)&(right<10.0)]) if right.size else 0.0
        return 'L' if l>r else 'R'

    # -------------------------
    # Web helpers
    # -------------------------
    def get_processed_frame(self):
        with self.image_lock:
            return None if self.processed_frame is None else self.processed_frame.copy()

    def get_stats(self):
        md = self.current_mode.value
        lf = f"{self.get_front_min_dist():.2f}" if self.get_front_min_dist() is not None else "N/A"
        rubbercon_status = "✅ PASSED" if self.rubbercon_passed else ("↔ AVOIDING" if md==DriveMode.RUBBERCON_AVOIDANCE.value else "—")
        lane_status = "✅ FOLLOWING" if self.lane_detected else "🔎 SEARCHING"
        return {
            "current_mode": md,
            "traffic_light": self.tl_color,
            "rubbercon_status": rubbercon_status,
            "lane_status": lane_status,
            "detection_confidence": f"{self.detection_confidence:.1f}",
            "camera_fps": self.camera_fps,
            "lidar_distance": lf,
            "speed": f"{self.current_speed:.2f}",
            "steering_angle": f"{math.degrees(self.current_steering):.1f}",
        }


# ================================
# 5) main
# ================================
def main(args=None):
    rclpy.init(args=args)
    try:
        node = Autoracer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nBye.")
    finally:
        if 'node' in locals(): node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
