# NumPy Compatibility Fix

## 🐛 Problem

When running `ros2 run vlogger_system vlogger_control`, the system crashes with:

```
A module that was compiled using NumPy 1.x cannot be run in
NumPy 2.2.6 as it may crash. To support both 1.x and 2.x
versions of NumPy, modules must be compiled with NumPy 2.0.

[ros2run]: Segmentation fault
```

## 🔍 Root Cause

- **System NumPy version:** 2.2.6 (too new)
- **cv_bridge compiled with:** NumPy 1.x
- **Conflict:** Binary incompatibility between NumPy versions

The cv_bridge package (used to convert ROS image messages to OpenCV format) was compiled against NumPy 1.x and cannot work with NumPy 2.x.

## ✅ Solution

Downgrade system NumPy to version 1.x:

```bash
pip3 install "numpy<2" --force-reinstall --break-system-packages
```

### Verification

```bash
# Check NumPy version
python3 -c "import numpy; print(f'NumPy version: {numpy.__version__}')"

# Should output:
# NumPy version: 1.26.4
```

## 📋 Technical Details

### Why NumPy 2.x Doesn't Work

NumPy 2.0 introduced breaking changes to the C API. Packages compiled against NumPy 1.x (like cv_bridge, OpenCV bindings) have binary dependencies on NumPy 1.x structures and cannot load with NumPy 2.x.

### The `--break-system-packages` Flag

This flag is required on systems where pip is managed externally (like Ubuntu 24.04+). It allows modifying system Python packages despite the restriction.

**Alternative (if flag causes issues):**
```bash
# Use pipx or install in a virtual environment
python3 -m venv ~/numpy_env
source ~/numpy_env/bin/activate
pip install "numpy<2"
```

However, since ROS2 uses system Python, the system NumPy must be 1.x.

### Why Not Upgrade cv_bridge?

The ROS2 Jazzy cv_bridge package is pre-built and installed via apt. Rebuilding it from source to support NumPy 2.x would require:
1. Downloading cv_bridge source
2. Recompiling with NumPy 2.x headers
3. Managing the custom build

Downgrading NumPy is the simpler, recommended solution.

## 🔄 When This Issue Occurs

You'll see this error when:
1. **NumPy gets auto-upgraded** by another package (e.g., `pip install some-ml-package`)
2. **Fresh system install** where NumPy 2.x is the default
3. **After system updates** that upgrade Python packages

## 🛡️ Preventing Future Issues

### Pin NumPy Version (Recommended)

Create a pip constraints file:

```bash
# Create constraints file
echo "numpy<2.0" > ~/numpy-constraints.txt

# Use when installing packages
pip3 install package_name --constraint ~/numpy-constraints.txt
```

### Check Before Running

Add to your launch script:

```bash
#!/bin/bash
# Check NumPy version before running vlogger
NUMPY_VERSION=$(python3 -c "import numpy; print(numpy.__version__.split('.')[0])")

if [ "$NUMPY_VERSION" == "2" ]; then
    echo "❌ ERROR: NumPy 2.x detected. Please downgrade:"
    echo "   pip3 install 'numpy<2' --force-reinstall --break-system-packages"
    exit 1
fi

echo "✅ NumPy version OK: $(python3 -c 'import numpy; print(numpy.__version__)')"

# Continue with vlogger launch...
```

## 📊 Compatibility Matrix

| Package | NumPy 1.x | NumPy 2.x |
|---------|-----------|-----------|
| **cv_bridge (ROS2 Jazzy)** | ✅ Works | ❌ Crashes |
| **OpenCV** | ✅ Works | ⚠️ Depends on build |
| **MediaPipe** | ✅ Works | ✅ Works |
| **ROS2 core** | ✅ Works | ❌ Some issues |

## 🧪 Testing After Fix

```bash
# Test 1: Import NumPy
python3 -c "import numpy; print('NumPy OK:', numpy.__version__)"

# Test 2: Import cv_bridge
python3 -c "from cv_bridge import CvBridge; print('cv_bridge OK')"

# Test 3: Import OpenCV with NumPy
python3 -c "import cv2; import numpy; print('OpenCV + NumPy OK')"

# Test 4: Run vlogger
cd /home/robot/workspace2/team11_ws_final_project/Robotics_Final_Project
source install/setup.bash
ros2 run vlogger_system vlogger_control
```

Expected output:
```
NumPy OK: 1.26.4
cv_bridge OK
OpenCV + NumPy OK
[INFO] [vlogger_controller]: 📸 FIRST IMAGE RECEIVED: 640x480
```

## 📚 Related Issues

- [NumPy 2.0 Migration Guide](https://numpy.org/devdocs/numpy_2_0_migration_guide.html)
- [ROS Answers: cv_bridge NumPy 2.0](https://answers.ros.org/question/416796/cv_bridge-numpy-20-compatibility/)
- [OpenCV GitHub Issue #24762](https://github.com/opencv/opencv/issues/24762)

## ✅ Success Criteria

After applying the fix:
- ✅ `python3 -c "import numpy; print(numpy.__version__)"` shows 1.x
- ✅ No "compiled using NumPy 1.x" errors
- ✅ No segmentation faults
- ✅ Vlogger control runs without crashes
- ✅ Image conversion works (cv_bridge)

---

**Last Updated:** December 4, 2025  
**Status:** ✅ Fixed - NumPy 1.26.4 installed  
**Severity:** Critical (system crash)  
**Fix Time:** < 1 minute
