# Safety Features Documentation
## TM5-900 Vlogger System

---

## Overview

This document describes the safety mechanisms implemented to ensure the robot arm **does not move** when no human face is detected in the camera frame.

---

## Critical Safety Checks

### **1. Primary Safety Check: No Human Detection**

**Location:** [vlogger_control.py:469-471](src/vlogger_system/vlogger_system/vlogger_control.py#L469-L471)

```python
def control_loop(self):
    """
    Main control loop - runs at 5 Hz.
    SAFETY: Robot will NOT move if no human face is detected in frame.
    """
    # CRITICAL SAFETY CHECK: Do not move if no human is detected
    if self.current_human_pos is None:
        # Robot stays stationary when no face is in frame
        return
```

**How it works:**
- Control loop runs every 200ms (5 Hz)
- **First action:** Check if `self.current_human_pos` is `None`
- If `None` → **Immediately return** without any movement calculation
- Robot remains stationary at last known position

---

### **2. Position Data Clearing**

**Location:** [vlogger_control.py:219-221](src/vlogger_system/vlogger_system/vlogger_control.py#L219-L221)

```python
else:
    # No human detected - clear position data to prevent movement
    self.current_human_pos = None
    self.position_history.clear()  # Clear history to prevent stale data
    cv2.putText(display_image, "NO HUMAN DETECTED",
              (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
```

**How it works:**
- When face detection fails (no human in frame)
- `current_human_pos` set to `None`
- `position_history` deque **cleared completely**
- **Prevents** robot from using old/stale position data
- Live view displays **"NO HUMAN DETECTED"** warning in red

---

### **3. Position Data Validation**

**Location:** [vlogger_control.py:475-478](src/vlogger_system/vlogger_system/vlogger_control.py#L475-L478)

```python
# Additional safety check: validate position data
if face_size <= 0 or not (0 <= human_x <= 3000) or not (0 <= human_y <= 3000):
    self.get_logger().warning('Invalid human position data detected, skipping movement')
    return
```

**How it works:**
- Even if `current_human_pos` is not `None`, validate the data
- Check face size is positive (valid detection)
- Check X and Y coordinates are within reasonable bounds (0-3000 pixels)
- If any check fails → Log warning and **skip movement**
- **Prevents** erroneous movement from corrupted data

---

## Safety Flow Diagram

```
Camera Frame Arrives (30 Hz)
    ↓
Face Detection (MediaPipe)
    ↓
    ├─→ Face Detected?
    │   ├─→ YES:
    │   │   ├── Update position_history
    │   │   ├── Calculate smoothed position
    │   │   └── Set current_human_pos = (x, y, size)
    │   │
    │   └─→ NO:
    │       ├── Set current_human_pos = None ✓
    │       ├── Clear position_history ✓
    │       └── Display "NO HUMAN DETECTED" warning ✓
    ↓
Control Loop (5 Hz)
    ↓
    ├─→ Check 1: current_human_pos == None?
    │   └─→ YES: RETURN (No movement) ✓✓✓
    │
    ├─→ Check 2: face_size <= 0?
    │   └─→ YES: RETURN (No movement) ✓
    │
    ├─→ Check 3: Position out of bounds?
    │   └─→ YES: RETURN (No movement) ✓
    │
    └─→ All checks pass:
        ↓
        Calculate movement
        ↓
        Execute if needed
```

---

## Testing Scenarios

### **Scenario 1: Human Leaves Frame**

```
T=0.0s  Human in frame, tracking active
        └── current_human_pos = (1200, 950, 400)
        └── Robot moving to center human

T=1.0s  Human walks out of camera view
        └── Face detection: False
        └── current_human_pos = None ✓
        └── position_history.clear() ✓

T=1.2s  Control loop executes
        └── Check: current_human_pos == None? → YES
        └── RETURN immediately ✓
        └── Robot DOES NOT MOVE ✓✓✓

T=1.4s  Control loop executes again
        └── Check: current_human_pos == None? → YES
        └── RETURN immediately ✓
        └── Robot remains stationary ✓

T=2.0s  Human re-enters frame
        └── Face detected
        └── current_human_pos = (1250, 980, 410)
        └── Robot resumes tracking ✓
```

### **Scenario 2: Occlusion (Hand Covers Face)**

```
T=0.0s  Human in frame, face visible
        └── Tracking active

T=0.5s  Human covers face with hand
        └── Face detection: False (hand blocks face)
        └── current_human_pos = None ✓
        └── Robot stops moving ✓

T=1.0s  Human removes hand
        └── Face detection: True
        └── Tracking resumes ✓
```

### **Scenario 3: Multiple People (Only First Detected)**

```
T=0.0s  Person A in frame (detected)
        └── Tracking Person A

T=1.0s  Person A leaves, Person B enters
        └── If Person A leaves first:
            ├── current_human_pos = None ✓
            ├── Robot stops ✓
            └── When Person B detected, tracking starts ✓
```

### **Scenario 4: Poor Lighting / Face Not Detected**

```
T=0.0s  Room lights turn off
        └── Face detection quality drops

T=0.2s  Face detection: False
        └── current_human_pos = None ✓
        └── Robot stops immediately ✓

T=1.0s  Lights turn back on
        └── Face detected again
        └── Tracking resumes ✓
```

---

## Live View Indicators

When no human is detected, the live view shows:

```
┌─────────────────────────────────────────┐
│          Camera Feed                     │
│                                          │
│                                          │
│  NO HUMAN DETECTED (large red text) ✓  │
│                                          │
│                                          │
├─────────────────────────────────────────┤
│ Status Panel                             │
│ FPS: 29.8    Robot: (230, 230, 800)    │
│ Mode: WAITING ✓  Target Z: 800mm       │
└─────────────────────────────────────────┘
```

**Visual Indicators:**
- ❌ **"NO HUMAN DETECTED"** in red (large font)
- 🟠 **Mode: WAITING** (orange color instead of green)
- ⏸️ No face mesh overlay
- ⏸️ No bounding box
- ⏸️ No offset calculations shown

---

## Logs & Monitoring

### **Normal Operation (Human Detected)**
```
[INFO] [vlogger_controller]: Moving: (230.0, 230.0, 800.0) → (235.0, 228.0, 785.0)
[INFO] [vlogger_controller]: Moving: (235.0, 228.0, 785.0) → (240.0, 230.0, 780.0)
```

### **No Human Detected (Safe State)**
```
(No movement logs - control loop returns early)
```

### **Invalid Position Data**
```
[WARN] [vlogger_controller]: Invalid human position data detected, skipping movement
```

---

## Code Locations Summary

| Safety Feature | File | Line | Status |
|----------------|------|------|--------|
| Primary null check | vlogger_control.py | 469-471 | ✅ Active |
| Position clearing | vlogger_control.py | 219-221 | ✅ Active |
| Data validation | vlogger_control.py | 475-478 | ✅ Active |
| History clearing | vlogger_control.py | 221 | ✅ Active |
| Visual warning | vlogger_control.py | 222-223 | ✅ Active |
| Mode display | vlogger_control.py | 237-240 | ✅ Active |

---

## Additional Safety Features

### **1. Workspace Limits**
**Location:** [vlogger_control.py:527-528](src/vlogger_system/vlogger_system/vlogger_control.py#L527-L528)

```python
# Safety limits for workspace
new_x = max(0.0, min(500.0, new_x))
new_y = max(0.0, min(500.0, new_y))
```

Even when human is detected, robot cannot move outside defined workspace.

### **2. Minimum Movement Threshold**
**Location:** [vlogger_control.py:494-499](src/vlogger_system/vlogger_system/vlogger_control.py#L494-L499)

```python
if move_distance > self.min_movement:
    # Rate limit movements (max 2 Hz)
    current_time = time.time()
    if current_time - self.last_move_time > 0.5:
        self.move_robot(new_x, new_y, new_z)
```

Prevents unnecessary micro-movements, reduces wear on robot.

### **3. Rate Limiting**
Maximum 2 movements per second (500ms minimum interval).

### **4. Service Ready Check**
**Location:** [vlogger_control.py:571-573](src/vlogger_system/vlogger_system/vlogger_control.py#L571-L573)

```python
if not self.arm_client.service_is_ready():
    self.get_logger().warning('send_script service not ready')
    return False
```

Don't send commands if robot service is not ready.

---

## Emergency Stop

### **Method 1: Press 'q' Key**
- Focus on live view window
- Press `q` key
- System shuts down gracefully
- Robot stops at current position

### **Method 2: Ctrl+C in Terminal**
```bash
^C
[INFO] [vlogger_controller]: Shutting down...
```

### **Method 3: Close Live View Window**
- Click X button on window
- System detects and shuts down

---

## Best Practices

### **For Operators:**

1. ✅ **Always monitor live view** - Shows real-time system state
2. ✅ **Stay in camera view** - Robot only moves when face is detected
3. ✅ **Face the camera** - Frontal face detection works best
4. ✅ **Good lighting** - Improves detection reliability
5. ✅ **Watch for "WAITING" mode** - Indicates no movement

### **For Developers:**

1. ✅ **Never remove safety checks** - Critical for safe operation
2. ✅ **Test no-face scenarios** - Ensure robot stops properly
3. ✅ **Log all movements** - Helps debugging and monitoring
4. ✅ **Validate all inputs** - Check bounds, null values, etc.
5. ✅ **Keep position history small** - Prevents stale data accumulation

---

## Verification Checklist

Before deploying the system, verify:

- [ ] Robot stops when human leaves frame
- [ ] Position data cleared when face lost
- [ ] Live view shows "NO HUMAN DETECTED" warning
- [ ] Mode changes to "WAITING" when no face
- [ ] Robot resumes tracking when human returns
- [ ] No movement with invalid position data
- [ ] Workspace limits enforced
- [ ] Rate limiting working (max 2 Hz)
- [ ] Emergency stop methods functional
- [ ] All safety logs appearing correctly

---

## Summary

### **Three-Layer Safety System:**

1. **Detection Layer**
   - No face → `current_human_pos = None`
   - Clear position history immediately

2. **Control Layer**
   - Check `current_human_pos == None` → Return early
   - Validate position data → Skip if invalid

3. **Execution Layer**
   - Workspace limits enforced
   - Rate limiting active
   - Service ready checks

### **Result:**
**Robot WILL NOT MOVE when no human face is in camera frame.**

---

## Contact

For questions about safety features:
- See implementation in [vlogger_control.py](src/vlogger_system/vlogger_system/vlogger_control.py)
- Check [LIVEVIEW_GUIDE.md](LIVEVIEW_GUIDE.md) for user documentation
- Review [PRESENTATION_SLIDES.md](PRESENTATION_SLIDES.md) for system overview
