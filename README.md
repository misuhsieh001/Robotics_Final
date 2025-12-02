# TM5-900 Indoor Vlogger System

An autonomous vlogging system using the TM5-900 robot arm that tracks human faces and responds to hand gestures for automated camera operation. Features real-time face tracking at 30 FPS using USB webcam with MediaPipe-powered detection.

## 🎯 Features

- **High-Speed Face Tracking** - 30 FPS real-time tracking using USB webcam (upgraded from 0.3 FPS Techman camera)
- **Face Detection & Tracking** - Automatically centers human face in camera frame using MediaPipe Face Mesh
- **Intelligent Distance Control** - Maintains optimal framing based on face size (target: 100px, tolerance: ±15px)
- **Gesture Recognition** - Hand gestures provide manual distance control:
  - 👆 **1 finger** → Move closer (100mm)
  - 🖐️ **5 fingers** → Back up (100mm)
- **Live View Window** - Real-time display with detection overlays, tracking status, and face size indicators
- **Safety Features** - Robot stops immediately when no face is detected
- **Optimized Performance** - Conservative movement parameters to prevent speed errors and arm limit collisions

## 🚀 Quick Start

### Prerequisites
- ROS2 Jazzy (or Humble or later)
- TM5-900 robot arm with tm_driver running
- Python 3.12+
- USB Webcam (640×480 or higher resolution)
- NVIDIA GPU (optional, for acceleration)

### Installation

1. **Clone the repository:**
   ```bash
   cd ~/workspace2/team11_ws_final_project
   git clone https://github.com/misuhsieh001/Robotics_Final.git Robotics_Final_Project
   cd Robotics_Final_Project
   ```

2. **Install USB camera package:**
   ```bash
   sudo apt update
   sudo apt install ros-jazzy-usb-cam
   ```

3. **Configure camera settings (optional but recommended):**
   ```bash
   # Install v4l-utils for camera control
   sudo apt install v4l-utils
   
   # Adjust brightness, gain, and exposure for optimal image quality
   v4l2-ctl --device=/dev/video0 --set-ctrl=brightness=128
   v4l2-ctl --device=/dev/video0 --set-ctrl=gain=50
   v4l2-ctl --device=/dev/video0 --set-ctrl=auto_exposure=3
   ```

4. **Build the workspace:**
   ```bash
   colcon build
   source install/setup.bash
   ```

5. **Verify MediaPipe installation:**
   ```bash
   python3 test_mediapipe.py
   ```

### Running the Vlogger

**Step 1: Start the USB camera node**
```bash
# In Terminal 1
source install/setup.bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  --param video_device:=/dev/video0 \
  --param image_width:=640 \
  --param image_height:=480 \
  --param framerate:=30.0 \
  --param camera_name:=usb_camera \
  --param pixel_format:=yuyv
```

**Step 2: Launch the vlogger control**
```bash
# In Terminal 2
source install/setup.bash
ros2 run vlogger_system vlogger_control
```

**Alternative: Use the convenience script (starts camera automatically)**
```bash
./run_vlogger.sh
```

## 📖 Usage

### Window Controls
- **'q'** - Quit the application
- **'s'** - Save current frame as JPG
- **'r'** - Reset robot to initial position

### Gesture Commands
- **1 finger** - Move robot closer (100mm)
- **5 fingers** - Move robot away (100mm)
- Auto-distance re-enables after 5 seconds

### Expected Behavior
1. USB camera starts publishing at 30 FPS to `/image_raw` topic
2. Vlogger window opens showing "Initializing camera..."
3. Live feed begins with face detection overlays
4. When face detected:
   - Green rectangle around face
   - Face size displayed (target: 100px ± 15px)
   - Robot smoothly centers face in frame
   - Automatic distance adjustment maintains optimal framing
5. Hand gestures provide manual distance override
6. **Safety:** Robot stops immediately if no face detected

### Performance Indicators
- **FPS Display:** Shows actual frame rate (~30 FPS with USB camera)
- **Face Size:** Current face size vs target (85-115px range is acceptable)
- **Status Messages:** Movement commands, safety checks, adjustment triggers
- **Debug Logs:** Face size, position offsets, movement calculations

## ⚙️ Configuration

Key parameters in `vlogger_control.py`:

```python
# Image Resolution (USB Camera)
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
IMAGE_CENTER_X = 320.0  # Center X coordinate
IMAGE_CENTER_Y = 240.0  # Center Y coordinate

# Face Tracking
self.target_face_size = 100.0     # pixels - optimal face size
self.face_size_tolerance = 15.0   # pixels - acceptable range (85-115px)
self.centering_threshold = 40     # pixels - movement deadzone
self.min_movement = 10.0          # mm - minimum movement threshold

# Distance Adjustment
self.auto_distance_adjust = True  # Enable automatic depth control
distance_multiplier = 2.0         # Adjustment strength
max_adjustment = 100.0            # mm - maximum depth movement

# Movement Parameters
scale = 0.3                       # Movement scaling factor (conservative)
max_step = 150                    # mm - maximum single movement

# Control Timing
control_timer = 0.2               # seconds - 5 Hz control loop
movement_rate_limit = 1.0         # seconds - max 1 movement per second
settling_time = 1.0               # seconds - pause after movement

# Robot Speed (PTP Command)
speed = 35                        # % - conservative to prevent errors
acceleration = 100                # mm/s² - smooth motion

# Position Limits (safety boundaries)
X: 100-600mm (depth - towards/away from camera)
Y: 0-600mm (left/right movement)
Z: 200-700mm (up/down movement)
```

### Tuning Guide

**If robot moves too slowly:**
- Increase `speed` (currently 35%, max safe ~50%)
- Decrease `movement_rate_limit` (currently 1.0s)

**If robot is too sensitive:**
- Increase `centering_threshold` (currently 40px)
- Decrease `scale` (currently 0.3)

**If distance control is too aggressive:**
- Decrease `distance_multiplier` (currently 2.0)
- Increase `face_size_tolerance` (currently 15px)

**If robot hits arm limits:**
- Check `scale` is not too high (keep ≤ 0.5)
- Verify `max_step` is reasonable (currently 150mm)
- Review position limits in code

## 🔧 System Architecture

```
┌─────────────────────┐
│   USB Webcam        │
│   (/dev/video0)     │
│   640×480 @ 30 FPS  │
└──────────┬──────────┘
           │
           ↓
┌─────────────────────────────┐
│  usb_cam_node               │
│  Publishes to /image_raw    │
└──────────┬──────────────────┘
           │ 30 FPS
           ↓
┌─────────────────────────────────────┐
│    Image Processing (ROS2 Node)     │
│  - Face Detection (MediaPipe Mesh)  │
│  - Hand Gestures (MediaPipe Hands)  │
│  - Position Calculation             │
│  - Face Size Analysis               │
│  - Live View Display (30 Hz)        │
└──────────┬──────────────────────────┘
           │
           ↓
┌─────────────────────────────────────┐
│    Control Loop (5 Hz)              │
│  - Safety Checks (no face = stop)   │
│  - Movement Calculation              │
│  - Rate Limiting (max 1 Hz)         │
│  - Distance Adjustment Logic        │
│  - Robot Commands (PTP)             │
└──────────┬──────────────────────────┘
           │
           ↓
┌─────────────────────────────────────┐
│   TM5-900 Robot Arm                 │
│   (tm_msgs SendScript Service)      │
│   35% speed, 100mm/s² acceleration  │
└─────────────────────────────────────┘
```

### Data Flow Rates
- **Camera Capture:** 30 Hz (30 FPS)
- **Image Processing:** 30 Hz (MediaPipe on CPU)
- **Window Updates:** 30 Hz (responsive display)
- **Control Loop:** 5 Hz (position calculations)
- **Robot Commands:** ≤1 Hz (movement rate limit)
- **Settling Time:** 1.0s after each movement

## 🛡️ Safety Features

1. **Primary Safety Check** - Robot stops immediately if no face is detected (prevents runaway)
2. **Position Validation** - Checks for valid coordinate ranges before any movement
3. **Workspace Limits** - Hard-coded boundaries prevent dangerous positions and arm limit collisions
4. **Rate Limiting** - Maximum 1 movement per second prevents system overload
5. **Movement Scaling** - Conservative 0.3x scaling factor prevents excessive motion
6. **Speed Limiting** - 35% speed prevents "Robot speed: 100%" TMFlow errors
7. **Settling Time** - 1.0s pause after each movement for smooth operation
8. **Position History Clearing** - Prevents stale data from causing erratic movements

See [SAFETY_FEATURES.md](SAFETY_FEATURES.md) for detailed documentation.

## 📊 Performance

### Hardware
- **GPU:** NVIDIA GeForce RTX 2060 (used for OpenCV operations)
- **Camera:** USB Webcam (Logitech or compatible)
- **Resolution:** 640×480 native
- **Frame Rate:** 30 FPS (100x improvement over Techman camera)
- **Processing Time:** <33ms per frame (real-time)

### Optimization History
- **Initial:** Techman built-in camera via Vision_DoJob (~0.3 FPS) ❌
- **Upgraded:** USB webcam with ros-jazzy-usb-cam (30 FPS) ✅
- **Camera tuning:** Brightness, gain, exposure optimized via v4l2-ctl ✅
- **Rate limiting:** Increased from 0.2s to 1.0s between movements ✅
- **Speed reduction:** PTP speed from 80% → 35% to fix TMFlow errors ✅
- **Scale reduction:** Movement scale from 1.0 → 0.3 to prevent arm limits ✅
- **Interrupt removal:** Simplified from StopAndClearBuffer() to basic PTP ✅

### Current Performance Metrics
- **Face Detection:** MediaPipe Face Mesh on CPU
- **Hand Detection:** MediaPipe Hands on CPU
- **Control Loop:** 5 Hz (every 200ms)
- **Movement Rate:** Max 1 Hz (every 1000ms)
- **Display Update:** 30 Hz (smooth live view)
- **Latency:** <100ms from detection to command

### Known Limitations
- Distance adjustment (face size-based depth control) implemented but requires testing
- MediaPipe runs on CPU (GPU acceleration available but not yet enabled)
- Initial position command uses old speed parameters (100%, needs update to 35%)

## 📁 Project Structure

```
Robotics_Final_Project/
├── src/
│   ├── vlogger_system/          # Main vlogger package
│   │   ├── vlogger_control.py   # Core control logic
│   │   └── ...
│   └── send_script/             # Robot command utilities
├── venv/                        # Python virtual environment
├── test_*.py                    # Test scripts
├── run_vlogger.sh              # Launch script
├── README.md                    # This file
├── FIXES_SUMMARY.md            # Development history
├── SAFETY_FEATURES.md          # Safety documentation
├── MEDIAPIPE_FIX.md            # MediaPipe setup guide
├── WINDOW_FIX.md               # Technical details
└── CAMERA_DIAGNOSTIC.md        # Troubleshooting guide
```

## 🐛 Troubleshooting

### USB Camera Not Working
```bash
# Check if camera is detected
ls -l /dev/video*

# Test camera with v4l2
v4l2-ctl --device=/dev/video0 --list-formats-ext

# Check camera topic
ros2 topic list | grep image
ros2 topic hz /image_raw
```

### Low or No Frame Rate
```bash
# Verify USB camera node is running
ros2 node list | grep usb_cam

# Check image topic frequency
ros2 topic hz /image_raw
# Should show ~30 Hz

# Restart camera node if needed
pkill -f usb_cam_node_exe
ros2 run usb_cam usb_cam_node_exe --ros-args --param video_device:=/dev/video0
```

### MediaPipe Not Available
```bash
# Verify MediaPipe is installed in venv
source venv/bin/activate
pip list | grep mediapipe
deactivate

# Test import
python3 test_mediapipe.py
# Should show: ✅ MediaPipe imported successfully!

# Reinstall if needed
source venv/bin/activate
pip install mediapipe==0.10.21
deactivate
```

### Window Crashes or Not Responding
- Ensure DISPLAY environment variable is set: `echo $DISPLAY`
- Check for X11 errors in terminal output
- Kill any existing windows: `pkill -f "Vlogger View"`
- See [WINDOW_FIX.md](WINDOW_FIX.md) for technical details

### Robot Errors

**"Robot speed: 100%" Error:**
- This was fixed by reducing PTP speed parameter from 80% to 35%
- If error persists, check TMFlow safety settings

**Robot Hitting Arm Limits:**
- Verify `scale` parameter is ≤ 0.3 in code
- Check `max_step` is not too large (should be ≤ 150mm)
- Ensure initial position is centered in workspace

**Robot Stops After 20 Seconds:**
- This was fixed by increasing movement rate limit to 1.0s
- System now allows maximum 1 movement per second

**Robot Not Moving:**
- Check if face is detected (look for green rectangle in window)
- Verify face size is outside tolerance range (should be <85px or >115px for distance adjustment)
- Check logs for "SAFETY: No human detected" messages
- Ensure `tm_driver` is running: `ros2 node list | grep tm_driver`

### Image Too Dark/Bright
```bash
# Adjust camera settings
v4l2-ctl --device=/dev/video0 --set-ctrl=brightness=128  # 0-255
v4l2-ctl --device=/dev/video0 --set-ctrl=gain=50         # 0-100
v4l2-ctl --device=/dev/video0 --set-ctrl=auto_exposure=3 # Auto mode

# List all available controls
v4l2-ctl --device=/dev/video0 --list-ctrls
```

### Debug Mode
Enable detailed logging by checking the terminal output:
- Face size values and target
- Movement calculations
- Distance adjustment triggers
- Safety check results
- PTP command parameters

All debug information is logged to the console automatically.

## 🔄 Recent Updates

**December 2025 - Major Performance Upgrade:**
- ✅ **USB Webcam Integration** - Upgraded from 0.3 FPS Techman camera to 30 FPS USB camera (100x improvement)
- ✅ **Camera Quality Tuning** - Optimized brightness, gain, and exposure using v4l2-ctl
- ✅ **Speed Error Fix** - Reduced PTP speed from 80% to 35% to eliminate TMFlow errors
- ✅ **Safety Improvements** - Conservative movement scaling (0.3x) prevents arm limit collisions
- ✅ **Rate Limiting** - Increased to 1.0s between movements for stable operation
- ✅ **Simplified Control Logic** - Removed StopAndClearBuffer() interrupt that caused issues
- ✅ **Distance Adjustment** - Separated X-axis (depth) from Y-axis (horizontal) movement
- ✅ **Enhanced Debugging** - Added comprehensive logging for face size and movement calculations
- ✅ **Image Center Update** - Corrected constants for 640×480 resolution (was using old values)

**Previous Optimizations:**
- ✅ Face size-based distance adjustment
- ✅ MediaPipe GPU acceleration support
- ✅ Live view window stability fixes (30 Hz update timer)
- ✅ Gesture recognition with manual distance override
- ✅ Position history smoothing
- ✅ PyTorch with CUDA support installed

## 📚 Additional Documentation

- **[USB_CAMERA_UPGRADE.md](USB_CAMERA_UPGRADE.md)** - Complete USB camera setup guide (0.3 FPS → 30 FPS upgrade)
- **[FIXES_SUMMARY.md](FIXES_SUMMARY.md)** - Complete development history and all fixes applied
- **[SAFETY_FEATURES.md](SAFETY_FEATURES.md)** - Detailed safety mechanism documentation
- **[CAMERA_DIAGNOSTIC.md](CAMERA_DIAGNOSTIC.md)** - Legacy Techman camera troubleshooting (deprecated)
- **[MEDIAPIPE_FIX.md](MEDIAPIPE_FIX.md)** - MediaPipe installation and venv integration
- **[WINDOW_FIX.md](WINDOW_FIX.md)** - Technical details on window management and 30 Hz timer

## 🎓 Learning Resources

### Key ROS2 Concepts Used
- **Service Clients** - `tm_msgs.srv.SendScript` for robot commands
- **Image Transport** - Subscribing to camera topics
- **Timers** - Multiple timers for control loop, detection, and display
- **Parameter Handling** - Camera resolution and control parameters

### MediaPipe Integration
- **Face Mesh** - 468 facial landmarks for accurate tracking
- **Hands** - Hand landmark detection for gesture recognition
- **CPU Processing** - Optimized for real-time performance

### Robot Programming
- **PTP Commands** - Point-to-point motion with speed/acceleration control
- **Coordinate Mapping** - Image space (pixels) → Robot space (millimeters)
- **Safety Boundaries** - Workspace limits to prevent collisions

## 👥 Authors

Team 11 - Robotics Final Project

## 📄 License

This project is part of an academic robotics course.
