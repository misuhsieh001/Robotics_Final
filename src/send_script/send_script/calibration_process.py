#!/usr/bin/env python
"""
Camera Calibration Processing for TM5-900
Based on hw3_a.py - processes captured images and computes camera matrix
"""

import numpy as np
import cv2
import glob
import os
import sys

# ==============================================================================
# Calibration Settings for TM5-900
# ==============================================================================

# The number of inner corners per a chessboard row and column (18x13 for TM5-900)
CHECKERBOARD = (17, 12)

# The real-world size of a square on the chessboard (in mm)
SQUARE_SIZE = 20.0

# Set the termination criteria for cornerSubPix
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# For storing object points and image points from all the images.
objpoints = []  # 3D point in real world space
imgpoints = []  # 2D points in image plane.

# Create a single object point grid for the chessboard pattern
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE

print("="*70)
print("  TM5-900 Camera Calibration Processing")
print("="*70)
print(f"Chessboard pattern: {CHECKERBOARD[0]} x {CHECKERBOARD[1]} inner corners")
print(f"Square size: {SQUARE_SIZE} mm")
print("="*70 + "\n")

# Load images
image_path = '/home/robot/workspace2/team11_ws/calibration_images_tm5900/*.png'
images = glob.glob(image_path)

if len(images) == 0:
    print("--- Fatal Error ---")
    print(f"No images found at: {image_path}")
    print("Run calibration_capture.py first to capture images!")
    sys.exit()

print(f"Found {len(images)} images. Starting corner detection...\n")
h, w = 0, 0  # Image height and width

# Set OpenCV window for visualization
WINDOW_NAME = 'TM5-900 Calibration Check'
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WINDOW_NAME, 900, 700)

detected_count = 0

for i, fname in enumerate(images):
    img = cv2.imread(fname)
    if img is None:
        print(f"Error: Cannot load image: {fname}, continuing...")
        continue

    if h == 0 and w == 0:
        h, w = img.shape[:2]

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, flags)

    if ret == True:
        print(f"✅ Image {i+1}/{len(images)} ({os.path.basename(fname)}): Successful")
        detected_count += 1

        # Refine corner locations to sub-pixel accuracy
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        # Append object points and image points
        objpoints.append(objp)
        imgpoints.append(corners2)

        # Draw and display the corners
        img_drawn = cv2.drawChessboardCorners(img.copy(), CHECKERBOARD, corners2, ret)

        # Scale down for display
        scale_percent = 50
        width_resized = int(img_drawn.shape[1] * scale_percent / 100)
        height_resized = int(img_drawn.shape[0] * scale_percent / 100)
        resized_img = cv2.resize(img_drawn, (width_resized, height_resized), interpolation=cv2.INTER_AREA)

        # Show the image with detected corners
        cv2.imshow(WINDOW_NAME, resized_img)
        cv2.setWindowTitle(WINDOW_NAME, f'SUCCESS ({detected_count}/{len(images)}) - {os.path.basename(fname)}')
        cv2.waitKey(500)

    else:
        # Detection failed
        print(f"❌ Image {i+1}/{len(images)} ({os.path.basename(fname)}): Detection failed")
        cv2.setWindowTitle(WINDOW_NAME, f'FAILURE - {os.path.basename(fname)}')
        scale_percent = 50
        width_resized = int(img.shape[1] * scale_percent / 100)
        height_resized = int(img.shape[0] * scale_percent / 100)
        resized_img_fail = cv2.resize(img, (width_resized, height_resized), interpolation=cv2.INTER_AREA)
        cv2.imshow(WINDOW_NAME, resized_img_fail)
        cv2.waitKey(500)

cv2.destroyAllWindows()

if detected_count < 5:  # Need at least 5 images for reliable calibration
    print(f"\n--- Warning: Only {detected_count} images detected (need at least 5) ---")
elif detected_count == 0:
    print("\n--- Fatal error: No chessboard patterns detected ---")
    sys.exit()

print(f"\n--- Successfully detected {detected_count} images. Starting calibration ---\n")

# Execute camera calibration
ret_calib, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (w, h), None, None)

print("="*70)
print("  TM5-900 CALIBRATION RESULTS")
print("="*70)
print("\n*** COPY THIS CAMERA MATRIX INTO YOUR CODE ***\n")
print("Intrinsic Matrix K:")
print(mtx)
print("\nFor Python code, use:")
print("CAMERA_MATRIX = np.array([")
print(f"    [{mtx[0,0]:.2f}, {mtx[0,1]:.2f}, {mtx[0,2]:.2f}],")
print(f"    [{mtx[1,0]:.2f}, {mtx[1,1]:.2f}, {mtx[1,2]:.2f}],")
print(f"    [{mtx[2,0]:.2f}, {mtx[2,1]:.2f}, {mtx[2,2]:.2f}]")
print("], dtype=np.float32)")
print("\nDistortion Coefficients D:")
print(dist)

# Calculate re-projection error
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    mean_error += error

total_mean_error = mean_error / len(objpoints)
print(f"\nMean Re-projection Error: {total_mean_error:.4f} pixels")

# Optimize the camera matrix
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

print("\nOptimized Intrinsic Matrix:")
print(newcameramtx)

# Save calibration results
output_file = '/home/robot/workspace2/team11_ws/calibration_results_tm5900.npz'
np.savez(output_file,
         camera_matrix=mtx,
         dist_coeffs=dist,
         optimized_matrix=newcameramtx,
         roi=roi,
         image_size=(w, h))

print(f"\n✓ Calibration data saved to: {output_file}")
print("="*70)

# Display undistortion demonstration
if len(images) > 0:
    original_img = cv2.imread(images[0])
    if original_img is not None:
        # Undistort the image
        undistorted_img = cv2.undistort(original_img, mtx, dist, None, newcameramtx)

        # Display comparison
        cv2.namedWindow('TM5-900 Undistortion Result', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('TM5-900 Undistortion Result', 1200, 600)

        display_img = np.hstack((original_img, undistorted_img))
        cv2.imshow('TM5-900 Undistortion Result', display_img)
        print("\nPress any key to exit...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

print("\n=== TM5-900 Calibration Complete ===\n")


def main():
    """Entry point for ROS2 execution"""
    pass  # All code runs at module level


if __name__ == '__main__':
    main()
