#!/usr/bin/env python

import rclpy
import time
import sys
import cv2
import numpy as np
import glob
import os
import math

# Dynamically determine Python version for tm_msgs path
python_version = f"python{sys.version_info.major}.{sys.version_info.minor}"
tm_msgs_path = f'/home/robot/colcon_ws/install/tm_msgs/lib/{python_version}/site-packages'
sys.path.append(tm_msgs_path)
from tm_msgs.msg import *
from tm_msgs.srv import *

# Explicitly import required services
try:
    from tm_msgs.srv import SendScript, SetIO
except ImportError:
    print("WARNING: tm_msgs.srv not found. Make sure tm_msgs package is installed.")
    print(f"Looked in: {tm_msgs_path}")
    raise

# ==============================================================================
# Camera Calibration Settings for TM5-900
# ==============================================================================

# The number of inner corners per a chessboard row and column (18x13 for TM5-900)
CHECKERBOARD = (18, 13)

# The real-world size of a square on the chessboard (in mm)
SQUARE_SIZE = 20.0

# Set the termination criteria for cornerSubPix
CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# ==============================================================================
# Camera Intrinsic Matrix (ACTUAL TM5-900 CALIBRATION - 17x12 chessboard)
# ==============================================================================
# Calibrated on 2025-11-20 with mean re-projection error: 0.0778 pixels
CAMERA_MATRIX = np.array([
    [2711.03, 0.0,     1332.58],
    [0.0,     2709.31, 1013.72],
    [0.0,     0.0,     1.0    ]
], dtype=np.float32)

# Distortion coefficients from calibration
DIST_COEFFS = np.array([0.09845879, -0.89589613, -0.00460501, 0.00533473, 4.41194115], dtype=np.float32)

# arm client
def send_script(script):
    arm_node = rclpy.create_node('arm')
    arm_cli = arm_node.create_client(SendScript, 'send_script')

    while not arm_cli.wait_for_service(timeout_sec=1.0):
        arm_node.get_logger().info('service not availabe, waiting again...')

    move_cmd = SendScript.Request()
    move_cmd.script = script
    future = arm_cli.call_async(move_cmd)
    rclpy.spin_until_future_complete(arm_node, future)
    arm_node.destroy_node()

# gripper client
def set_io(state):
    gripper_node = rclpy.create_node('gripper')
    gripper_cli = gripper_node.create_client(SetIO, 'set_io')

    while not gripper_cli.wait_for_service(timeout_sec=1.0):
        gripper_node.get_logger().info('service not availabe, waiting again...')

    io_cmd = SetIO.Request()
    io_cmd.module = 1
    io_cmd.type = 1
    io_cmd.pin = 0
    io_cmd.state = state
    future = gripper_cli.call_async(io_cmd)
    rclpy.spin_until_future_complete(gripper_node, future)
    gripper_node.destroy_node()

def image_to_robot_coords(u, v):
    """
    Transform image pixel coordinates to robot coordinates using fixed camera height.
    NO DEPTH CAMERA - uses fixed Z-height for table.

    Args:
        u, v: Image pixel coordinates

    Returns:
        (X_robot, Y_robot, Z_robot): 3D coordinates in robot frame (mm)
    """
    # Extract camera intrinsics
    fx = CAMERA_MATRIX[0, 0]
    fy = CAMERA_MATRIX[1, 1]
    cx = CAMERA_MATRIX[0, 2]
    cy = CAMERA_MATRIX[1, 2]

    # Fixed camera parameters (NO DEPTH CAMERA)
    # Initial camera position: (230.00, 230, 730) - camera is 730mm above robot base
    # Robot base is ON the table, so table surface is at Z = 0
    camera_height = 730.0  # mm
    table_z = 0.0  # mm, table surface is at robot base level (Z=0)
    cube_height = 50.0  # mm, estimated cube height (adjust as needed)
    camera_to_table_distance = camera_height - table_z  # mm (= 730mm)

    # Use fixed camera-to-table distance
    Z_cam = camera_to_table_distance  # mm

    # Convert pixel to camera coordinates (inverse projection)
    # This gives us the X,Y offset from the camera center in mm
    X_offset = (u - cx) * Z_cam / fx
    Y_offset = (v - cy) * Z_cam / fy

    # Transform to robot coordinates
    # Assuming camera is at initial position (230, 230, 730)
    # and looking straight down (Z-axis pointing down)
    X_robot = 230.0 + X_offset  # mm
    Y_robot = 230.0 + Y_offset  # mm
    Z_robot = table_z + cube_height / 2.0  # mm, center of cube

    return X_robot, Y_robot, Z_robot

def find_all_contours(img):
    """
    Find contours and calculate centroids of all objects in the image.

    Args:
        img: Input BGR image

    Returns:
        tuple: (detected_objects, processed_image, cleaned_mask)
            - detected_objects: List of dicts with contour, centroid, area, angle info
            - processed_image: Image copy with visualizations
            - cleaned_mask: Binary mask after morphological operations
    """
    # Make a copy for drawing
    output_image = img.copy()

    # Convert to HSV color space
    hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # HSV color ranges optimized for detecting all 5 cube colors
    # Ranges tuned to detect saturated colors + white cubes
    color_ranges = {
        'red1': (np.array([0, 100, 50]), np.array([10, 255, 255])),
        'red2': (np.array([170, 100, 50]), np.array([180, 255, 255])),
        'green': (np.array([35, 40, 40]), np.array([85, 255, 255])),  # Broadened green range
        'blue': (np.array([90, 50, 50]), np.array([130, 255, 255])),
        'yellow': (np.array([15, 100, 100]), np.array([35, 255, 255])),
        'white': (np.array([0, 0, 180]), np.array([180, 40, 255])),  # High brightness, low saturation for white cubes
    }

    # Create combined mask for all colors
    combined_mask = np.zeros(img.shape[:2], dtype="uint8")

    for color_name, (lower, upper) in color_ranges.items():
        mask = cv2.inRange(hsv_image, lower, upper)

        # Handle red color (wraps around in HSV color space)
        if color_name == 'red1':
            mask_red2 = cv2.inRange(hsv_image, color_ranges['red2'][0],
                                   color_ranges['red2'][1])
            mask = cv2.bitwise_or(mask, mask_red2)
        elif color_name == 'red2':
            continue

        combined_mask = cv2.bitwise_or(combined_mask, mask)

    # Apply stronger morphological operations to clean noise and merge cube regions
    # Larger kernels to handle noisy color detection
    kernel_open = np.ones((5, 5), np.uint8)
    mask_opened = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel_open, iterations=2)
    kernel_close = np.ones((15, 15), np.uint8)
    cleaned_mask = cv2.morphologyEx(mask_opened, cv2.MORPH_CLOSE, kernel_close, iterations=2)

    # Find all contours - RETR_EXTERNAL gets only outer contours
    contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL,
                                  cv2.CHAIN_APPROX_SIMPLE)

    # Filter contours and calculate centroids for each detected object
    detected_objects = []
    print(f"\n--- Found {len(contours)} total contours ---")

    for i, cnt in enumerate(contours):
        area = cv2.contourArea(cnt)
        # Area threshold adjusted for cleaned morphological mask
        if area > 1500:  # Minimum area for cubes after morphological operations
            # Additional check: approximate contour shape
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)

            # Cubes typically have 4-8 vertices, but be flexible
            if len(approx) >= 3:  # Relaxed from 4 to 3
                # Calculate moments and centroid
                M = cv2.moments(cnt)
                if M["m00"] == 0:
                    continue

                # Calculate centroid (from hw3_b.py)
                cX = M["m10"] / M["m00"]
                cY = M["m01"] / M["m00"]

                # Fit ellipse to get orientation angle (if enough points)
                angle_deg = 0.0
                angle_rad = 0.0
                if len(cnt) >= 5:  # Need at least 5 points for fitEllipse
                    try:
                        ellipse = cv2.fitEllipse(cnt)
                        angle_deg = ellipse[2]  # Angle in degrees
                        angle_rad = angle_deg * math.pi / 180.0
                        # Normalize angle to 0-90 range
                        if angle_deg > 90:
                            angle_deg = angle_deg - 90
                    except:
                        pass  # Skip angle calculation if fitEllipse fails

                # Store detected object information
                detected_objects.append({
                    'id': i,
                    'contour': cnt,
                    'centroid': (cX, cY),
                    'area': area,
                    'angle_deg': angle_deg,
                    'angle_rad': angle_rad,
                    'vertices': len(approx)
                })

                # Draw contour, centroid, and info
                cv2.drawContours(output_image, [cnt], -1, (0, 255, 0), 2)
                cv2.circle(output_image, (int(cX), int(cY)), 5, (0, 0, 255), -1)
                cv2.putText(output_image, f"#{i} ({int(cX)},{int(cY)})",
                          (int(cX) - 40, int(cY) - 10),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

                print(f"Cube #{i}: Centroid=({cX:.2f}, {cY:.2f}), Area={int(area)}, "
                      f"Angle={angle_deg:.2f}°, Vertices={len(approx)}")

    print(f"--- Detected {len(detected_objects)} cubes with centroids ---\n")

    return detected_objects, output_image, cleaned_mask

def camera_calibration(image_path='calibration_images/*.PNG', display_results=True):
    """
    Perform camera calibration using chessboard images for TM5-900 robot.

    Args:
        image_path: Path pattern to calibration images (default: 'calibration_images/*.PNG')
        display_results: Whether to display calibration results (default: True)

    Returns:
        tuple: (mtx, dist, newcameramtx, roi) - Camera matrix, distortion coefficients,
               optimized camera matrix, and region of interest
    """
    # For storing object points and image points from all the images
    objpoints = []  # 3D point in real world space
    imgpoints = []  # 2D points in image plane

    # Create a single object point grid for the chessboard pattern
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE

    # Load calibration images
    images = glob.glob(image_path)

    if len(images) == 0:
        print("--- Fatal Error ---")
        print(f"No images found at {image_path}. Check the path and file type.")
        return None, None, None, None

    print(f"Found {len(images)} images. Starting corner detection...")
    h, w = 0, 0  # Image height and width

    if display_results:
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
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), CRITERIA)

            # Append object points and image points
            objpoints.append(objp)
            imgpoints.append(corners2)

            if display_results:
                # Draw and display the corners
                img_drawn = cv2.drawChessboardCorners(img.copy(), CHECKERBOARD, corners2, ret)

                # Scale down for display
                scale_percent = 50
                width_resized = int(img_drawn.shape[1] * scale_percent / 100)
                height_resized = int(img_drawn.shape[0] * scale_percent / 100)
                resized_img = cv2.resize(img_drawn, (width_resized, height_resized),
                                        interpolation=cv2.INTER_AREA)

                cv2.imshow(WINDOW_NAME, resized_img)
                cv2.setWindowTitle(WINDOW_NAME,
                                  f'SUCCESS ({detected_count}/{len(images)}) - {os.path.basename(fname)}')
                cv2.waitKey(500)
        else:
            print(f"❌ Image {i+1}/{len(images)} ({os.path.basename(fname)}): Detection failed")
            if display_results:
                cv2.setWindowTitle(WINDOW_NAME, f'FAILURE - {os.path.basename(fname)}')
                scale_percent = 50
                width_resized = int(img.shape[1] * scale_percent / 100)
                height_resized = int(img.shape[0] * scale_percent / 100)
                resized_img_fail = cv2.resize(img, (width_resized, height_resized),
                                             interpolation=cv2.INTER_AREA)
                cv2.imshow(WINDOW_NAME, resized_img_fail)
                cv2.waitKey(500)

    if display_results:
        cv2.destroyAllWindows()

    if detected_count < 5:
        print(f"\n--- Warning: Only {detected_count} images detected (need at least 5) ---")
        if detected_count == 0:
            print("\n--- Fatal error: No chessboard patterns detected ---")
            return None, None, None, None

    print(f"\n--- Successfully detected {detected_count} images. Starting calibration ---")

    # Execute camera calibration
    ret_calib, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints,
                                                              (w, h), None, None)

    print("\n--- Calibration Result for TM5-900 ---")
    print("Intrinsic Matrix K:\n", mtx)
    print("\nDistortion Coefficient D:\n", dist)

    # Calculate re-projection error
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error

    total_mean_error = mean_error / len(objpoints)
    print("\nMean Re-projection Error:", total_mean_error)

    # Optimize the camera matrix based on free scaling parameter
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    print("\nOptimized Intrinsic Matrix:\n", newcameramtx)

    # Display undistortion result
    if display_results and len(images) > 0:
        original_img = cv2.imread(images[0])
        if original_img is not None:
            # Undistort the image
            undistorted_img = cv2.undistort(original_img, mtx, dist, None, newcameramtx)

            # Crop the image based on the ROI
            x, y, w_crop, h_crop = roi
            undistorted_img_cropped = undistorted_img[y:y+h_crop, x:x+w_crop]

            # Display the original and undistorted images side by side
            cv2.namedWindow('TM5-900 Undistortion Result', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('TM5-900 Undistortion Result', 1200, 600)

            display_img = np.hstack((original_img, undistorted_img))
            cv2.imshow('TM5-900 Undistortion Result', display_img)
            print("\nPress any key to continue...")
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    print("\n--- Camera Calibration Complete ---")
    return mtx, dist, newcameramtx, roi

def main(args=None):

    rclpy.init(args=args)

    # ==============================================================================
    # Camera Calibration for TM5-900 (18x13 chessboard)
    # ==============================================================================
    # Uncomment the following lines to run camera calibration
    # Make sure you have calibration images in 'calibration_images/' folder

    # Run calibration
    # mtx, dist, newcameramtx, roi = camera_calibration(
    #     image_path='calibration_images/*.PNG',
    #     display_results=True
    # )
    #
    # if mtx is not None:
    #     print("\nCalibration successful! You can now use the camera parameters.")
    #     print("Camera Matrix:\n", mtx)
    #     print("\nDistortion Coefficients:\n", dist)
    # else:
    #     print("\nCalibration failed. Please check your images.")

    # ==============================================================================
    # Robot Movement Commands
    # ==============================================================================

    #--- move command by joint angle ---#
    # script = 'PTP(\"JPP\",45,0,90,0,90,0,35,200,0,false)'

    #--- move command by end effector's pose (x,y,z,a,b,c) ---#
    # targetP1 = "398.97, -122.27, 748.26, -179.62, 0.25, 90.12"

    # Initial camera position for taking image (Please do not change the values)
    # For right arm: targetP1 = "230.00, 230, 730, -180.00, 0.0, 135.00"
    # For left  arm: targetP1 = "350.00, 350, 730, -180.00, 0.0, 135.00"
    targetP1 = "230.00, 230, 730, -180.00, 0.0, 135.00"
    targetP2 = "250.00, 250, 500, -180.00, 0.0, 135.00"
    script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
    script2 = "PTP(\"CPP\","+targetP2+",100,200,0,false)"
    send_script(script1)
    send_script(script2)

# What does Vision_DoJob do? Try to use it...
# -------------------------------------------------
    send_script("Vision_DoJob(job1)")
    cv2.waitKey(1)
    send_script("Vision_DoJob(job1)")
    cv2.waitKey(1)
#--------------------------------------------------

    set_io(1.0) # 1.0: close gripper, 0.0: open gripper
    set_io(0.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()




