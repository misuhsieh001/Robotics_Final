# Cube Stacker - Depth Camera Integration

This package provides two implementations for detecting cubes using a depth camera and stacking them with a robot arm.

## Files Created

1. **cube_stacker.py** - Continuous processing version that subscribes to camera topics
2. **cube_stacker_simple.py** - Single-shot version similar to send_script.py approach

## Features

Both implementations include:
- RGB and Depth camera image processing
- 3D cube coordinate calculation using camera intrinsics (K-matrix)
- Transformation from camera frame to robot base frame
- Automated pick and place operations
- Color-based cube detection (red, green, blue, yellow, white)

## Setup

### 1. Camera Topics

Before running, verify your depth camera topics:

```bash
ros2 topic list | grep camera
```

Update the topic names in the scripts if needed:
- RGB topic: `camera/color/image_raw` (default)
- Depth topic: `camera/depth/image_raw` (default)

### 2. Camera Calibration

The scripts use a K-matrix (camera intrinsic matrix) for 3D coordinate calculation:

```python
K = np.array([
    [613.57, 0.0,    286.54],
    [0.0,    613.54, 251.36],
    [0.0,    0.0,    1.0   ]
])
```

**Update these values** based on your camera's calibration parameters.

### 3. Coordinate Transformation

The transformation from camera frame to robot base frame uses these parameters:

```python
X_robot = X_c * 0.123726312 + Y_c * (-0.115410351) + 268.474584
Y_robot = X_c * (-0.106878157) + Y_c * (-0.114065432) + 558.216611
```

**These need to be calibrated** for your specific setup through hand-eye calibration.

## Usage

### Option 1: Using cube_stacker_simple.py (Recommended)

This version is easier to use and similar to the existing send_script.py:

```bash
# Build the workspace (if needed)
cd /home/robot/colcon_ws
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Run the script
python3 src/tm2_ros2/image_sub/src/cube_stacker_simple.py
```

**Workflow:**
1. Robot moves to camera viewing position
2. Captures one RGB and one depth image
3. Detects cubes and calculates 3D coordinates
4. Prints coordinates in format: `#ID 0: Coord (-106.99, -32.17, 567.07)`
5. Picks and stacks each cube one by one

### Option 2: Using cube_stacker.py (Continuous)

This version subscribes to camera topics and processes continuously:

```bash
python3 src/tm2_ros2/image_sub/src/cube_stacker.py
```

## Configuration Parameters

### Camera Position
Adjust the robot position for optimal camera view:
```python
targetP1 = "230.00, 230, 730, -180.00, 0.0, 135.00"
```

### Stacking Location
Change where cubes are stacked:
```python
stack_x, stack_y = 480.0, 180.0
stack_height = 125  # Starting height
```

### Height Increment
Distance between stacked cubes:
```python
stack_height += 28  # Adjust based on cube size
```

### Color Detection
Modify HSV color ranges in `color_ranges` dictionary if detection is not working well.

## Calibration Steps

### 1. Camera Intrinsics (K-matrix)

Get your camera's intrinsic parameters from:
- Camera manufacturer specifications
- ROS2 camera_calibration package
- camera_info topic

```bash
ros2 topic echo /camera/color/camera_info
```

### 2. Hand-Eye Calibration

To get accurate transformation parameters:

1. Place a known object at a measured position
2. Capture RGB and depth images
3. Calculate object position in camera frame
4. Measure actual robot frame coordinates
5. Calculate transformation matrix parameters

Example calibration script structure:
```python
# Known positions
camera_points = [(X_c1, Y_c1), (X_c2, Y_c2), ...]
robot_points = [(X_r1, Y_r1), (X_r2, Y_r2), ...]

# Solve for transformation parameters
# X_r = a*X_c + b*Y_c + c
# Y_r = d*X_c + e*Y_c + f
```

## Troubleshooting

### No cubes detected
- Check RGB image: `/home/robot/colcon_ws/detected_cubes_simple.png`
- Adjust color ranges in `color_ranges`
- Ensure good lighting conditions

### Invalid depth values
- Verify depth camera is publishing data
- Check depth topic: `ros2 topic echo /camera/depth/image_raw`
- Ensure objects are within camera's depth range

### Incorrect 3D coordinates
- Verify K-matrix parameters
- Check depth image units (mm vs meters)
- Calibrate hand-eye transformation

### Robot movement issues
- Verify send_script service: `ros2 service list | grep send_script`
- Check set_io service for gripper: `ros2 service list | grep set_io`
- Ensure robot is in proper mode and connected

## Integration with Existing Code

To integrate into your existing image_sub.py or send_script.py:

1. Copy the relevant functions:
   - `get_valid_depth()`
   - `detect_cubes_3d()`
   - `image_capture_callback()` (if needed)

2. Add the K-matrix and transformation parameters

3. Call the detection function after image capture

4. Use the returned coordinates for your robot control logic

## Output Format

The scripts output cube coordinates in the requested format:

```
#ID 0: Coord (-106.99, -32.17, 567.07)
#ID 1: Coord (72.08, -39.30, 567.00)
#ID 2: Coord (93.18, -147.20, 566.09)
#ID 3: Coord (-114.01, -144.65, 566.83)
#ID 4: Coord (-37.71, -179.08, 566.00)
```

These coordinates are in the robot base frame and can be used directly for motion commands.

## Notes

- The scripts assume a Techman robot with specific services (SendScript, SetIO)
- Adjust sleep times based on your robot's speed
- The stacking logic assumes cubes can be safely stacked vertically
- Always test in simulation or with safety measures first

## Based On

This implementation integrates:
- hw3_c.py template (depth camera processing and 3D coordinate calculation)
- image_sub.py (ROS2 image subscription and robot control)
- send_script.py (robot command sending structure)
