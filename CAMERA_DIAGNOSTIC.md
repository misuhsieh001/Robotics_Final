# Camera Diagnostic Guide (LEGACY - Techman Camera)

> **⚠️ NOTE:** This document describes the **old Techman built-in camera** system that was replaced by USB webcam.  
> **For current USB camera setup, see [USB_CAMERA_UPGRADE.md](USB_CAMERA_UPGRADE.md)**

---

## Historical Context

This diagnostic guide was created when the vlogger used the TM5-900's built-in Techman camera via Vision_DoJob service calls. The system achieved only ~0.3 FPS due to the 3-second service call latency.

**Status:** 🗄️ Deprecated - Replaced by USB webcam (30 FPS)  
**Kept for:** Reference and understanding original system architecture

---

## Original Issue Summary
The vlogger window showed "Initializing camera..." but never displayed live images, even though images appeared in TMFlow.

## Root Cause (Historical)
The TM5-900 camera **does not publish images continuously**. It only publishes when triggered by the `Vision_DoJob(job1)` command (as seen in `cube_stacker_logic.py` line 431).

## Solution Implemented (Then Deprecated)
Added automatic camera triggering at 10 Hz in `vlogger_control.py`:
- Line 163: Camera trigger timer created
- Line 372-395: `trigger_camera_capture()` function sends `Vision_DoJob(job1)` every 100ms

**Problem:** Even with triggering, frame rate was limited to ~0.3 FPS (one frame every 3+ seconds)

**Final Solution:** Migrated to USB webcam achieving 30 FPS ✅

---

## Legacy Diagnostic Output

When running the old Techman camera system, you would see:

### 1. Initialization
```
==============================================
  TM5-900 VLOGGER CONTROLLER INITIALIZED
==============================================
📷 Camera trigger: Sending Vision_DoJob(job1) at 10 Hz
⏳ Waiting for camera images...
==============================================
```

### 2. Camera Triggers (first 3)
```
📷 Trigger #1: Sending Vision_DoJob(job1)...
📷 Trigger #2: Sending Vision_DoJob(job1)...
📷 Trigger #3: Sending Vision_DoJob(job1)...
```

### 3. First Image Received (SUCCESS!)
```
======================================================================
✅ SUCCESS! Camera is publishing images!
   Image size: 2688x2048
   Encoding: 8UC3
   Vision job "job1" is working correctly!
======================================================================
```

## Troubleshooting

### If you see triggers but NO images:

**Problem:** Vision job "job1" is not configured or not working

**Solutions:**
1. Check TMFlow to verify "job1" exists and is properly configured
2. Try running cube_stacker first to verify camera works:
   ```bash
   ros2 run send_script cube_stacker_logic
   ```
3. If cube_stacker works but vlogger doesn't, the issue is with the vision job

### If you see "service not ready" warning:

**Problem:** `send_script` service is not available

**Solutions:**
1. Make sure tm_driver is running:
   ```bash
   ros2 node list | grep tm_driver
   ```
2. Restart the driver if needed

### If NO triggers appear:

**Problem:** Camera trigger timer not starting

**Solutions:**
1. Check if vlogger node initialized properly
2. Look for errors in terminal output
3. Verify `arm_client` service client was created

## How to Run

```bash
cd /home/robot/workspace2/team11_ws_final_project/Robotics_Final_Project
source install/setup.bash
ros2 run vlogger_system vlogger_control
```

## Expected Behavior

1. **Within 1 second:** See camera trigger messages
2. **Within 2-3 seconds:** See first image received message  
3. **Live view window:** Should update with camera feed showing face tracking

## Key Difference from cube_stacker

| Feature | cube_stacker | vlogger |
|---------|--------------|---------|
| Trigger frequency | Once (line 431) | Continuous (10 Hz) |
| Purpose | Single snapshot for detection | Live video stream |
| Command | `Vision_DoJob(job1)` | Same, but repeated |

## Vision Job Requirements

The Vision job "job1" must be:
- ✅ Created in TMFlow
- ✅ Configured to publish to `/techman_image` topic
- ✅ Set to trigger on command (not automatic)
- ✅ Image resolution: 2688x2048 (or compatible)

---

**Last Updated:** December 1, 2025  
**Status:** Diagnostics enhanced - awaiting user testing
