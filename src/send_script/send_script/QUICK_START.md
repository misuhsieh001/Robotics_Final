# Quick Start Guide - Depth Camera Cube Detection

## Step 4: Transform Centroids to Robot Arm Pose ✅

The depth camera integration is now complete in `image_sub.py`!

## What You Have

Two approaches for cube detection with depth camera:

### Option 1: image_sub.py (Integrated - RECOMMENDED)
- ✅ Fully integrated into your existing workflow
- ✅ Subscribes to RGB + Depth camera continuously
- ✅ Outputs 5 cube coordinates automatically
- ✅ Ready for robot control integration

### Option 2: Standalone Scripts
- `cube_stacker.py` - Full automated pick-and-place
- `cube_stacker_simple.py` - Single-shot detection and stacking

## Quick Test

### 1. Start Your Depth Camera

Make sure your depth camera is running and publishing to:
- RGB: `techman_image` or `camera/color/image_raw`
- Depth: `camera/depth/image_raw`

```bash
# Check topics
ros2 topic list | grep -E "image|depth"
```

### 2. Run Image Subscriber with Depth

```bash
cd ~/workspace2/team11_ws
colcon build --symlink-install
source install/setup.bash
ros2 run send_script image_sub
```

### 3. Expected Output

```
[INFO] [image_sub]: Received image
[INFO] [image_sub]: Depth image received
[INFO] [image_sub]: Found 5 cubes
[INFO] [image_sub]: === Cube Coordinates (using Depth Camera) ===
[INFO] [image_sub]: #ID 0: Coord (-106.99, -32.17, 567.07)
[INFO] [image_sub]: #ID 1: Coord (72.08, -39.30, 567.00)
[INFO] [image_sub]: #ID 2: Coord (93.18, -147.20, 566.09)
[INFO] [image_sub]: #ID 3: Coord (-114.01, -144.65, 566.83)
[INFO] [image_sub]: #ID 4: Coord (-37.71, -179.08, 566.00)
[INFO] [image_sub]: ===========================================
```

## Configuration

### If Depth Topic is Different

Edit line 44 in `image_sub.py`:
```python
'camera/depth/image_raw'  # Change to your depth topic name
```

Common depth topic names:
- `/camera/depth/image_raw`
- `/camera/aligned_depth_to_color/image_raw` (RealSense)
- `/depth_camera/depth/image_raw`

### If Depth Units are Wrong

If coordinates seem way off, depth might be in meters instead of mm.

Edit line 142 in `image_sub.py`:
```python
Z_cam = self.get_valid_depth(u, v) * 1000  # Add * 1000 if depth is in meters
```

## Key Features Added

| Feature | Description | Line in image_sub.py |
|---------|-------------|---------------------|
| Depth Subscription | Subscribes to depth camera | 43-45 |
| Depth Callback | Receives depth images | 79-88 |
| Valid Depth Extraction | Gets depth with noise filtering | 90-120 |
| 3D Coordinate Calculation | Transforms pixel→camera→robot | 122-163 |
| 5 Cube Output | Logs coordinates in requested format | 198-215 |

## What Happens Behind the Scenes

```
1. RGB Image → Color Detection → Find 5 Cubes
                                      ↓
2. For each cube centroid (u, v) → Get Depth Z
                                      ↓
3. Calculate 3D in camera frame:
   X_cam = (u - cx) * Z / fx
   Y_cam = (v - cy) * Z / fy
                                      ↓
4. Transform to robot base frame:
   X_robot = transformation(X_cam, Y_cam)
   Y_robot = transformation(X_cam, Y_cam)
                                      ↓
5. Output: #ID X: Coord (X, Y, Z)
```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| No depth data received | Check depth topic: `ros2 topic echo /camera/depth/image_raw` |
| Depth values are 0.0 | Objects outside camera range or bad lighting |
| Only 3-4 cubes detected | Adjust color ranges or lower area threshold |
| Coordinates way off | Check depth units (mm vs m), calibrate transformation |
| "service not available" | Make sure robot services are running |

## Documentation Files

1. **DEPTH_INTEGRATION_README.md** - Detailed technical documentation
2. **CUBE_STACKER_README.md** - Standalone script documentation
3. **QUICK_START.md** - This file
4. **image_sub.py** - Main implementation (lines 1-334)

## Next Steps

Now that you have the 3D coordinates, you can:

1. ✅ Verify coordinates are accurate
2. ✅ Integrate with motion planning in `send_script.py`
3. ✅ Implement pick-and-place logic
4. ✅ Stack the cubes!

Example integration in send_script.py:
```python
# Get cube coordinates from image_sub
detected_cubes = [
    {'id': 0, 'coord': (-106.99, -32.17, 567.07)},
    {'id': 1, 'coord': (72.08, -39.30, 567.00)},
    # ... etc
]

# Pick and stack each cube
for cube in detected_cubes:
    X, Y, Z = cube['coord']
    # Move to cube
    send_script(f'PTP("CPP",{X},{Y},{Z+50},-180,0,0,100,200,0,false)')
    # Pick it up
    set_io(1.0)
    # Move to stack location
    # ... etc
```

## Summary

✅ **Step 4 Complete!**

The `image_sub.py` file now:
- Subscribes to RGB and Depth cameras
- Detects 5 cubes using color segmentation
- Calculates accurate 3D coordinates using depth data
- Transforms coordinates from camera frame to robot base frame
- Outputs in the requested format: `#ID X: Coord (X, Y, Z)`

Ready to integrate with robot motion control for autonomous cube stacking!
