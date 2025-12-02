#!/usr/bin/env python3
"""
Quick diagnostic script to check if OpenCV can display windows
"""
import cv2
import numpy as np
import os

print("=" * 60)
print("DISPLAY DIAGNOSTIC")
print("=" * 60)

# Check DISPLAY environment variable
display = os.environ.get('DISPLAY', None)
print(f"DISPLAY environment variable: {display}")

if display is None:
    print("\n❌ ERROR: DISPLAY is not set!")
    print("\nTo fix this:")
    print("  1. If using SSH, connect with X11 forwarding:")
    print("     ssh -X user@hostname")
    print("  2. Or set DISPLAY manually:")
    print("     export DISPLAY=:0")
    print("  3. Make sure X11 server is running on your local machine")
else:
    print("✓ DISPLAY is set")

# Try to create a test window
print("\nTrying to create OpenCV window...")
try:
    # Create a simple test image
    test_img = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(test_img, "OpenCV Window Test", (50, 240),
                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
    cv2.putText(test_img, "If you see this, display works!", (50, 300),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.putText(test_img, "Press any key to close", (50, 360),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    
    cv2.namedWindow('Display Test', cv2.WINDOW_NORMAL)
    cv2.imshow('Display Test', test_img)
    
    print("✓ Window created successfully!")
    print("\nWindow should be visible on your screen.")
    print("Press any key in the window to close it...")
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    print("\n✅ SUCCESS: Display is working correctly!")
    print("Your vlogger live view should work now.")
    
except Exception as e:
    print(f"\n❌ ERROR: Failed to create window: {str(e)}")
    print("\nPossible solutions:")
    print("  1. Check if X11 server is running")
    print("  2. Install required packages:")
    print("     sudo apt-get install python3-opencv libopencv-dev")
    print("  3. If using Docker, run with: docker run -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix")
    print("  4. Try: xhost +local:docker (if using Docker)")

print("=" * 60)
