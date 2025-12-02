# USB Camera Upgrade Guide
## From 0.3 FPS to 30 FPS

---

## 📊 Executive Summary

**Problem:** Techman built-in camera limited to 0.3 FPS (one frame every 3+ seconds)  
**Solution:** Migrated to USB webcam achieving 30 FPS  
**Result:** **100x performance improvement**, enabling real-time face tracking

---

## 🎯 Why the Upgrade Was Necessary

### Original System (Techman Camera)
- **Frame Rate:** ~0.3 FPS (0.3 Hz)
- **Latency:** 3000ms+ per frame
- **Method:** `Vision_DoJob(job1)` service call
- **Resolution:** 2688×2048 (overkill for tracking)
- **Result:** Unusable for real-time tracking ❌

### Problems Encountered
1. **Extreme Lag** - Robot reacting to 3-second-old face positions
2. **Missed Movements** - Human moves, robot still processing old frame
3. **Poor UX** - Live view window updating once every 3 seconds
4. **Tracking Impossible** - Cannot follow moving subject at 0.3 FPS

### Why Techman Camera Was Slow
The Techman camera doesn't publish continuously like a typical ROS camera node. Instead:
- Requires `Vision_DoJob(job1)` service call per frame
- Service call takes ~3 seconds to complete
- Designed for single-snapshot tasks (like cube detection)
- Not designed for continuous video streaming

---

## ✅ USB Camera Solution

### Hardware Setup
```bash
# 1. Connect USB webcam to robot PC
# Verify device detected
ls -l /dev/video*
# Should show: /dev/video0

# 2. Check camera capabilities
v4l2-ctl --device=/dev/video0 --list-formats-ext
```

### Software Installation
```bash
# Install ROS2 USB camera package
sudo apt update
sudo apt install ros-jazzy-usb-cam

# Install camera control utilities
sudo apt install v4l-utils
```

### Camera Configuration
```bash
# Optimize image quality for face detection
v4l2-ctl --device=/dev/video0 --set-ctrl=brightness=128
v4l2-ctl --device=/dev/video0 --set-ctrl=gain=50
v4l2-ctl --device=/dev/video0 --set-ctrl=auto_exposure=3

# Verify settings
v4l2-ctl --device=/dev/video0 --all
```

---

## 🚀 Running the USB Camera

### Option 1: Manual Launch (Two Terminals)

**Terminal 1: Start USB Camera Node**
```bash
cd /home/robot/workspace2/team11_ws_final_project/Robotics_Final_Project
source install/setup.bash

ros2 run usb_cam usb_cam_node_exe --ros-args \
  --param video_device:=/dev/video0 \
  --param image_width:=640 \
  --param image_height:=480 \
  --param framerate:=30.0 \
  --param camera_name:=usb_camera \
  --param pixel_format:=yuyv
```

**Terminal 2: Start Vlogger Control**
```bash
cd /home/robot/workspace2/team11_ws_final_project/Robotics_Final_Project
source install/setup.bash

ros2 run vlogger_system vlogger_control
```

### Option 2: Launch Script (Recommended)
```bash
./run_vlogger.sh
```

---

## 🔧 Code Changes Required

### 1. Updated Image Topic Subscription
**File:** `src/vlogger_system/vlogger_system/vlogger_control.py`

```python
# OLD (Techman camera)
self.image_sub = self.create_subscription(
    Image,
    '/techman_image',  # Techman camera topic
    self.image_callback,
    10
)

# NEW (USB camera)
self.image_sub = self.create_subscription(
    Image,
    '/image_raw',  # USB camera topic (standard)
    self.image_callback,
    10
)
```

### 2. Updated Image Center Constants
```python
# OLD (Techman camera: 2688×2048)
IMAGE_CENTER_X = 1332.6
IMAGE_CENTER_Y = 1013.7

# NEW (USB camera: 640×480)
IMAGE_CENTER_X = 320.0
IMAGE_CENTER_Y = 240.0
```

### 3. Removed Vision_DoJob Trigger
```python
# OLD: Camera trigger timer (no longer needed)
# self.camera_trigger_timer = self.create_timer(0.1, self.trigger_camera_capture)

# USB camera publishes continuously, no triggering needed
```

---

## 📈 Performance Comparison

| Metric | Techman Camera | USB Camera | Improvement |
|--------|----------------|------------|-------------|
| **Frame Rate** | 0.3 FPS | 30 FPS | **100x** |
| **Latency** | 3000ms | 33ms | **91x faster** |
| **Resolution** | 2688×2048 | 640×480 | More efficient |
| **Publishing** | On-demand | Continuous | Reliable |
| **Setup** | Vision job config | Plug & play | Easier |
| **Cost** | Included | $20-50 | Affordable |

---

## 🎛️ Camera Parameters Explained

### Resolution: 640×480
- **Why:** Optimal balance of quality and performance
- **MediaPipe:** Works excellently at this resolution
- **Processing:** Low CPU/GPU load
- **Bandwidth:** Minimal USB bandwidth usage

### Frame Rate: 30 FPS
- **Why:** Standard video frame rate
- **Real-time:** Sufficient for smooth tracking
- **Overhead:** System can keep up easily
- **Alternative:** Can reduce to 15 FPS if needed

### Pixel Format: YUYV
- **Why:** Native format for most USB cameras
- **Conversion:** ROS automatically converts to RGB
- **Efficiency:** No unnecessary transcoding

### Camera Controls
```bash
# Brightness (0-255, default 128)
v4l2-ctl --device=/dev/video0 --set-ctrl=brightness=128

# Gain (0-100, default 50)
v4l2-ctl --device=/dev/video0 --set-ctrl=gain=50

# Auto Exposure (1=manual, 3=auto)
v4l2-ctl --device=/dev/video0 --set-ctrl=auto_exposure=3

# Manual Exposure (if auto_exposure=1)
v4l2-ctl --device=/dev/video0 --set-ctrl=exposure_absolute=200

# Saturation (0-100, default 64)
v4l2-ctl --device=/dev/video0 --set-ctrl=saturation=64

# Contrast (0-100, default 32)
v4l2-ctl --device=/dev/video0 --set-ctrl=contrast=32
```

---

## 🐛 Troubleshooting

### Camera Not Detected
```bash
# Check USB connection
lsusb
# Should show your webcam

# Check video devices
ls -l /dev/video*
# Should show /dev/video0 (or video1, video2, etc.)

# Try different USB port if not detected
```

### Permission Denied
```bash
# Add user to video group
sudo usermod -a -G video $USER

# Re-login or reboot for changes to take effect

# Verify permissions
groups
# Should include "video"
```

### Wrong Device Number
```bash
# If camera is /dev/video1 instead of /dev/video0
ros2 run usb_cam usb_cam_node_exe --ros-args \
  --param video_device:=/dev/video1  # Changed from video0
```

### No Images Published
```bash
# Check if node is running
ros2 node list | grep usb_cam

# Check topic
ros2 topic list | grep image

# Monitor topic frequency
ros2 topic hz /image_raw
# Should show ~30 Hz

# Echo one message
ros2 topic echo /image_raw --once
```

### Low Frame Rate
```bash
# Check CPU usage
htop
# Look for high CPU processes

# Reduce frame rate if needed
ros2 run usb_cam usb_cam_node_exe --ros-args \
  --param framerate:=15.0  # Half speed

# Check USB bandwidth (if multiple cameras)
lsusb -t
```

### Poor Image Quality
```bash
# Too dark
v4l2-ctl --device=/dev/video0 --set-ctrl=brightness=180
v4l2-ctl --device=/dev/video0 --set-ctrl=gain=80

# Too bright
v4l2-ctl --device=/dev/video0 --set-ctrl=brightness=80
v4l2-ctl --device=/dev/video0 --set-ctrl=gain=20

# Washed out colors
v4l2-ctl --device=/dev/video0 --set-ctrl=saturation=80

# Low contrast
v4l2-ctl --device=/dev/video0 --set-ctrl=contrast=50
```

---

## 🔍 Verification Steps

### 1. Camera Hardware Test
```bash
# View camera stream directly (without ROS)
ffplay /dev/video0
# Should show live video
# Press 'q' to quit
```

### 2. ROS Topic Test
```bash
# Start camera node
ros2 run usb_cam usb_cam_node_exe --ros-args --param video_device:=/dev/video0

# In another terminal, check frequency
ros2 topic hz /image_raw
# Should show: average rate: 30.000

# Check image size
ros2 topic echo /image_raw --once | grep -E "(height|width)"
# Should show: height: 480, width: 640
```

### 3. Integration Test
```bash
# Run full vlogger system
./run_vlogger.sh

# Expected output:
# - Window opens showing live view
# - FPS counter shows ~30 FPS
# - Face detection appears within 1 second
# - Green rectangle tracks face smoothly
```

---

## 💡 Optimization Tips

### For Best Performance
1. **Use native resolution** - 640×480 is native for most webcams
2. **Match frame rate** - 30 FPS is standard, don't go higher unless needed
3. **Enable auto-exposure** - Adapts to lighting changes automatically
4. **Keep USB cable short** - Reduces latency and interference
5. **Use USB 2.0 port** - Sufficient for this resolution/framerate

### For Lower CPU Usage
```bash
# Reduce frame rate
--param framerate:=15.0  # Half the FPS

# Reduce resolution
--param image_width:=320
--param image_height:=240
```

### For Better Quality
```bash
# Use higher resolution (requires more processing)
--param image_width:=1280
--param image_height:=720
--param framerate:=30.0

# Note: MediaPipe will auto-scale internally
```

---

## 📚 Additional Resources

### USB Camera Recommendations
- **Logitech C270** - Budget option, 720p, good compatibility
- **Logitech C920** - Premium option, 1080p, excellent quality
- **Microsoft LifeCam** - Good alternative, reliable drivers
- **Generic USB webcams** - Most work fine at 640×480

### ROS2 usb_cam Documentation
- [GitHub: ros-drivers/usb_cam](https://github.com/ros-drivers/usb_cam)
- [ROS2 Package Index](https://index.ros.org/p/usb_cam/)

### v4l2 Documentation
- `man v4l2-ctl` - Complete manual
- [Video4Linux Wiki](https://www.linuxtv.org/wiki/index.php/Main_Page)

---

## ✅ Success Criteria

After successful USB camera upgrade, you should have:

- ✅ Camera publishing at 30 FPS to `/image_raw`
- ✅ Live view window updating smoothly (30 Hz)
- ✅ Face detection working in real-time
- ✅ Robot tracking face movements immediately
- ✅ No lag between human movement and robot response
- ✅ Stable operation for extended periods
- ✅ FPS counter showing ~30 in live view window

---

*This upgrade transformed the vlogger from a proof-of-concept to a production-ready system!*

**Last Updated:** December 3, 2025  
**Status:** ✅ Successfully deployed, 30 FPS confirmed
