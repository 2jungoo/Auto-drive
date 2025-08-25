pkill -f image_transport || true
sudo apt-get update && sudo apt-get install -y ros-foxy-image-transport-plugins
ros2 run image_transport republish raw in:=/image_raw compressed out:=/image_raw
