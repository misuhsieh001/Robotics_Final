# TM5-900 Indoor Vlogger System

An autonomous vlogging system using the TM5-900 robot arm that tracks human faces and responds to hand gestures for automated camera operation.

## 🎯 Features

- **Face Detection & Tracking** - Automatically centers human face in camera frame using MediaPipe Face Mesh
- **Gesture Recognition** - Hand gestures control camera distance:
  - 👆 **1 finger** → Move closer
  - 🖐️ **5 fingers** → Back up
- **Auto-Distance Adjustment** - Maintains optimal framing based on face size
- **Live View Window** - Real-time display with detection overlays and tracking information
- **Safety Features** - Robot stops immediately when no face is detected

## 🚀 Quick Start

### Prerequisites
- ROS2 (Humble or later)
- TM5-900 robot arm with tm_driver running
- Python 3.12+
- NVIDIA GPU (optional, for acceleration)

### Installation

1. **Clone the repository:**
   ```bash
   cd ~/workspace2/team11_ws_final_project
   git clone https://github.com/misuhsieh001/Robotics_Final.git Robotics_Final_Project
   cd Robotics_Final_Project
   ```

2. **Build the workspace:**
   ```bash
   colcon build
   source install/setup.bash
   ```

3. **Verify MediaPipe installation:**
   ```bash
   python3 test_mediapipe.py
   ```

### Running the Vlogger

**Option 1: Using the launch script (recommended)**
```bash
./run_vlogger.sh
```

**Option 2: Manual launch**
```bash
source install/setup.bash
ros2 run vlogger_system vlogger_control
```

## 📖 Usage

### Window Controls
- **'q'** - Quit the application
- **'s'** - Save current frame
- **'r'** - Reset robot to initial position

### Gesture Commands
- **1 finger** - Move robot closer (100mm)
- **5 fingers** - Move robot away (100mm)
- Auto-distance re-enables after 5 seconds

### Expected Behavior
1. Window opens showing "Initializing camera..."
2. Camera starts capturing at ~0.3 FPS (Techman Vision_DoJob limitation)
3. Face detection overlay appears when human is in frame
4. Robot automatically centers the face and maintains optimal distance
5. Hand gestures provide manual distance control

## ⚙️ Configuration

Key parameters in `vlogger_control.py`:

```python
# Tracking
self.centering_threshold = 25  # pixels - movement deadzone
self.target_face_size = 100.0  # pixels - optimal face size
self.min_movement = 2.0        # mm - minimum movement threshold

# Control timing
control_timer = 0.1            # 10 Hz control loop
settling_time = 0.5            # seconds after movement

# Position limits
X: 100-600mm (depth)
Y: 0-600mm (side)
Z: 200-700mm (height)
```

## 🔧 System Architecture

```
┌─────────────────┐
│  Techman Camera │
│  (Vision_DoJob) │
└────────┬────────┘
         │ ~0.3 FPS
         ↓
┌─────────────────────────────┐
│    Image Processing         │
│  - Face Detection (MediaPipe)│
│  - Hand Gestures (MediaPipe) │
│  - Position Calculation      │
└────────┬────────────────────┘
         │
         ↓
┌─────────────────────────────┐
│    Control Loop (10 Hz)     │
│  - Safety Checks             │
│  - Movement Calculation      │
│  - Robot Commands            │
└────────┬────────────────────┘
         │
         ↓
┌─────────────────────────────┐
│   TM5-900 Robot Arm         │
│   (SendScript Service)      │
└─────────────────────────────┘
```

## 🛡️ Safety Features

1. **Primary Safety Check** - Robot stops immediately if no face is detected
2. **Position Validation** - Checks for valid coordinate ranges before movement
3. **Workspace Limits** - Hard-coded boundaries prevent dangerous positions
4. **Settling Time** - 0.5s pause after movement to avoid jerky motion
5. **Rate Limiting** - Movement commands limited to prevent overload

See [SAFETY_FEATURES.md](SAFETY_FEATURES.md) for detailed documentation.

## 📊 Performance

### Hardware
- **GPU:** NVIDIA GeForce RTX 2060
- **Camera:** Techman built-in (Vision_DoJob)
- **Frame Rate:** ~0.3 FPS (camera limitation)
- **Processing Time:** <100ms per frame

### Optimization
- MediaPipe runs on GPU (RTX 2060)
- Face detection with landmark refinement
- Reduced image resolution (480px width)
- Position smoothing (2-frame average)

**Note:** Low FPS is due to Techman camera requiring 3-second Vision_DoJob service call per frame. For higher FPS (30 FPS), use an external USB camera.

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

### MediaPipe Not Available
```bash
# Verify MediaPipe is installed in venv
source venv/bin/activate
pip list | grep mediapipe
deactivate

# Test import
python3 test_mediapipe.py
```

### Window Crashes
- Ensure DISPLAY environment variable is set: `echo $DISPLAY`
- Check for X11 errors in terminal output
- See [WINDOW_FIX.md](WINDOW_FIX.md) for details

### Camera Not Responding
- Verify Vision_DoJob "job1" exists in TMFlow
- Check tm_driver is running: `ros2 node list | grep tm_driver`
- See [CAMERA_DIAGNOSTIC.md](CAMERA_DIAGNOSTIC.md) for diagnostics

### Low Frame Rate
- **Expected behavior** - Techman camera is limited to ~0.3 FPS
- For higher FPS, connect external USB camera and modify camera topic
- See documentation for USB camera setup

## 🔄 Recent Updates

**Latest Optimizations:**
- ✅ Direct centering with no movement limits
- ✅ Face size-based distance adjustment
- ✅ 10Hz control loop for faster response
- ✅ 0.5s settling time after movement
- ✅ Detection resumes immediately after settling
- ✅ PyTorch with CUDA support installed

## 📚 Additional Documentation

- [FIXES_SUMMARY.md](FIXES_SUMMARY.md) - Complete fix history
- [SAFETY_FEATURES.md](SAFETY_FEATURES.md) - Safety mechanisms
- [MEDIAPIPE_FIX.md](MEDIAPIPE_FIX.md) - MediaPipe setup
- [WINDOW_FIX.md](WINDOW_FIX.md) - Window management
- [CAMERA_DIAGNOSTIC.md](CAMERA_DIAGNOSTIC.md) - Camera troubleshooting

## 👥 Authors

Team 11 - Robotics Final Project

## 📄 License

This project is part of an academic robotics course.
