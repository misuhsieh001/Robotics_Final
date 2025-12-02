#!/bin/bash
# Start USB camera with optimized brightness settings

echo "Starting USB camera with brightness optimizations..."

ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:="/dev/video0" \
  -p framerate:=30.0 \
  -p pixel_format:=yuyv \
  -p image_width:=640 \
  -p image_height:=480 \
  -p brightness:=180 \
  -p gain:=100 \
  -p auto_exposure:=1 \
  -p exposure:=500
