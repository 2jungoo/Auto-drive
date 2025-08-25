import time
import cv2

# Create capture
#url = 'rtspsrc latency=0 location=rtsp://192.168.200.150:5005/routecam ! rtpjitterbuffer ! rtph264depay ! avdec_h264 direct-rendering=0 ! queue leaky=2 ! videoconvert ! fpsdisplaysink video-sink=xvimagesink text-overlay=false sync=false -v'
#url = 'rtsp://192.168.200.150:5005/routecam'
#url = 'http://192.168.200.150:5005/routecam'
url = "rtspsrc latency=10 location=rtsp://192.168.200.150:5005/routecam ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink"

cap = cv2.VideoCapture(url, cv2.CAP_GSTREAMER)

# Check if cap is open
if cap.isOpened() is not True:
    print ("Cannot open camera. Exiting.")
    quit()

# Loop it
while True:
    # Get the frame
    ret, frame = cap.read()
    # Check
    if ret is True:
        # Flip frame
#        frame = cv2.flip(frame, 1)

        cv2.imshow('RouteCAM', frame)

    else:
        print ("Camera error.")
        time.sleep(10)

cap.release()

