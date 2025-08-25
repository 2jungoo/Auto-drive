pkill -f gscam || true
pkill -f gst-launch || true

ros2 run gscam gscam_node --ros-args \
  -p gscam_config:="nvarguscamerasrc sensor-id=0 bufapi-version=true ! \
video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! \
nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! \
video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false" \
  -p frame_id:=camera_link -p sync_sink:=false
