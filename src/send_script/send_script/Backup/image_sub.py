#!/usr/bin/env python
"""
Image Subscriber for TM5-900 Robot - NO DEPTH CAMERA

This node subscribes to RGB camera images to detect cubes and calculate
their 2D coordinates, then transforms them to robot coordinates using fixed camera height.

Features:
- RGB image subscription for color-based cube detection
- Fixed camera height assumption for coordinate transformation
- Camera calibration-based coordinate transformation
- Outputs cube coordinates in format: #ID X: Coord (X, Y, Z)
- Z coordinate is fixed at table height since NO depth camera available

Note: Camera has NO depth detection capability - uses fixed Z-height.
"""
import rclpy

from rclpy.node import Node

import sys
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

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math


class ImageSub(Node):
    def __init__(self, nodeName):
        super().__init__(nodeName)

        # Subscribe to RGB camera image
        self.subscription = self.create_subscription(Image,
        'techman_image', self.image_callback, 10)
        self.subscription

        # Subscribe to robot tool pose
        self.pose_subscription = self.create_subscription(PoseStamped,
        '/tool_pose', self.pose_callback, 10)

        self.bridge = CvBridge()
        self.image_count = 0

        # Camera intrinsic matrix K (ACTUAL TM5-900 CALIBRATION - 17x12 chessboard)
        # Calibrated on 2025-11-20 with mean re-projection error: 0.0778 pixels
        self.camera_matrix = np.array([
            [2711.03, 0.0,     1332.58],
            [0.0,     2709.31, 1013.72],
            [0.0,     0.0,     1.0    ]
        ], dtype=np.float32)

        # Distortion coefficients from calibration
        self.dist_coeffs = np.array([0.09845879, -0.89589613, -0.00460501, 0.00533473, 4.41194115], dtype=np.float32)

        # Store current robot tool pose
        self.current_tool_pose = None

        # Fixed camera parameters (NO DEPTH CAMERA)
        # Initial camera position: (230.00, 230, 730) - camera is 730mm above robot base
        # Robot base is ON the table, so table surface is at Z = 0
        self.camera_height = 730.0  # mm, height of camera above robot base (from initial position)
        self.table_z = 0.0  # mm, table surface is at robot base level (Z=0)
        self.cube_height = 50.0  # mm, estimated cube height (adjust as needed)

        # Camera to table distance (for calculating X, Y from pixel coordinates)
        self.camera_to_table_distance = self.camera_height - self.table_z  # mm (= 730mm)

    def pose_callback(self, msg):
        """Store the current robot tool pose."""
        self.current_tool_pose = msg.pose

    def image_to_robot_coords(self, u, v):
        """
        Transform image pixel coordinates to robot coordinates using fixed camera height.
        NO DEPTH CAMERA - uses fixed Z-height for table.

        Args:
            u, v: Image pixel coordinates

        Returns:
            (X_robot, Y_robot, Z_robot): 3D coordinates in robot frame (mm)
        """
        # Extract camera intrinsics
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        # Use fixed camera-to-table distance (no depth camera available)
        Z_cam = self.camera_to_table_distance  # mm

        # Convert pixel to camera coordinates (inverse projection)
        # This gives us the X,Y offset from the camera center in mm
        X_offset = (u - cx) * Z_cam / fx
        Y_offset = (v - cy) * Z_cam / fy

        # Transform to robot coordinates
        # Assuming camera is at initial position (230, 230, 730)
        # and looking straight down (Z-axis pointing down)
        X_robot = 230.0 + X_offset  # mm
        Y_robot = 230.0 + Y_offset  # mm
        Z_robot = self.table_z + self.cube_height / 2.0  # mm, center of cube

        return X_robot, Y_robot, Z_robot

    def image_callback(self, data):
        self.get_logger().info('Received image')

        # TODO (write your code here)
        try:
            # Convert ROS Image message to OpenCV format
            # Use 'passthrough' to keep the original encoding, then convert if needed
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            except Exception:
                # If bgr8 conversion fails, try passthrough and convert manually
                cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
                # If image is not already BGR, convert it
                if len(cv_image.shape) == 2:
                    # Grayscale image, convert to BGR
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
                elif cv_image.shape[2] == 4:
                    # RGBA image, convert to BGR
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2BGR)

            # Find contours and calculate centroids of all objects
            detected_objects, processed_image, mask = self.find_all_contours(cv_image)

            # Draw coordinate axes on the image (origin at top-left)
            # X-axis (horizontal, pointing right) - RED
            cv2.arrowedLine(processed_image, (50, 50), (250, 50), (0, 0, 255), 3, tipLength=0.1)
            cv2.putText(processed_image, 'X', (260, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)

            # Y-axis (vertical, pointing down) - GREEN
            cv2.arrowedLine(processed_image, (50, 50), (50, 250), (0, 255, 0), 3, tipLength=0.1)
            cv2.putText(processed_image, 'Y', (60, 270), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)

            # Origin label
            cv2.circle(processed_image, (50, 50), 5, (255, 255, 255), -1)
            cv2.putText(processed_image, 'O', (20, 45), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)

            # Log detection results with centroids
            self.get_logger().info(f'Found {len(detected_objects)} cubes')

            # Draw all contours and calculate robot coordinates
            # NO DEPTH CAMERA - using fixed camera height and table Z

            # Limit to 5 cubes and log in requested format
            cubes_to_process = detected_objects[:5]

            self.get_logger().info('=== Cube Coordinates (using Fixed Camera Height - NO DEPTH) ===')
            for obj in cubes_to_process:
                # Draw contour
                cv2.drawContours(processed_image, [obj['contour']], -1, (0, 255, 0), 3)

                # Transform image coordinates to robot coordinates
                u, v = obj['centroid']
                X_robot, Y_robot, Z_robot = self.image_to_robot_coords(u, v)

                # Store robot coordinates in object
                obj['robot_coords'] = (X_robot, Y_robot, Z_robot)

                # Draw larger ID number and coordinates on image
                cube_id = obj['id']

                # Draw centroid circle
                cv2.circle(processed_image, (int(u), int(v)), 8, (0, 0, 255), -1)

                # Prepare text with ID and coordinates
                id_text = f"ID {cube_id}"
                coord_text = f"({X_robot:.1f}, {Y_robot:.1f}, {Z_robot:.1f})"

                # Position text above and to the right of centroid
                text_x = int(u) + 15
                text_y = int(v) - 20

                # Draw text with much larger font and white background for better visibility
                font = cv2.FONT_HERSHEY_SIMPLEX
                id_font_scale = 1.5  # Larger ID font
                coord_font_scale = 1.2  # Larger coordinate font
                thickness = 3  # Thicker text

                # Get text size for background rectangle
                (id_w, id_h), _ = cv2.getTextSize(id_text, font, id_font_scale, thickness)
                (coord_w, coord_h), _ = cv2.getTextSize(coord_text, font, coord_font_scale, thickness)

                # Draw background rectangles for better readability
                cv2.rectangle(processed_image,
                             (text_x - 8, text_y - id_h - 8),
                             (text_x + id_w + 8, text_y + 8),
                             (0, 0, 0), -1)
                cv2.rectangle(processed_image,
                             (text_x - 8, text_y + 15),
                             (text_x + coord_w + 8, text_y + 15 + coord_h + 8),
                             (0, 0, 0), -1)

                # Draw ID text (much larger)
                cv2.putText(processed_image, id_text,
                           (text_x, text_y),
                           font, id_font_scale, (0, 255, 255), thickness)

                # Draw coordinate text (larger)
                cv2.putText(processed_image, coord_text,
                           (text_x, text_y + 45),
                           font, coord_font_scale, (255, 255, 255), thickness)

                # Log in requested format: #ID X: Coord (X, Y, Z)
                self.get_logger().info(
                    f"#ID {cube_id}: Coord ({X_robot:.2f}, {Y_robot:.2f}, {Z_robot:.2f})"
                )

            self.get_logger().info('===========================================')

            # Resize image for display (make window smaller)
            display_height = 600
            h, w = processed_image.shape[:2]
            scale = display_height / h
            display_width = int(w * scale)
            display_image = cv2.resize(processed_image, (display_width, display_height))
            display_mask = cv2.resize(mask, (display_width, display_height))

            # Display the image with contours and mask for debugging
            cv2.namedWindow('TM5-900 - Cube Contours', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('TM5-900 - Cube Contours', display_width, display_height)
            cv2.imshow('TM5-900 - Cube Contours', display_image)

            # Show mask to debug color detection
            cv2.namedWindow('TM5-900 - Color Mask (Debug)', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('TM5-900 - Color Mask (Debug)', display_width, display_height)
            cv2.imshow('TM5-900 - Color Mask (Debug)', display_mask)
            cv2.waitKey(1)

            self.image_count += 1

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def find_all_contours(self, img):
        """
        Find contours and calculate centroids of all objects in the image.

        Args:
            img: Input BGR image from camera

        Returns:
            tuple: (detected_objects, processed_image, cleaned_mask)
                - detected_objects: List of dicts with contour, centroid, area, angle info
                - processed_image: Image copy for visualization
                - cleaned_mask: Binary mask showing detected colors
        """
        # Make a copy for drawing
        output_image = img.copy()

        # Convert to HSV color space
        hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # HSV color ranges optimized for detecting all 5 cube colors
        # Ranges tuned to detect saturated colors + white cubes
        color_ranges = {
            'red1': (np.array([0, 50, 50]), np.array([10, 255, 255])),
            'red2': (np.array([170, 50, 50]), np.array([180, 255, 255])),
            'green': (np.array([35, 40, 40]), np.array([85, 255, 255])),  # Broadened green range
            'blue': (np.array([90, 50, 50]), np.array([130, 255, 255])), 
            'yellow': (np.array([15, 100, 100]), np.array([35, 255, 255])),
            'white': (np.array([0, 0, 100]), np.array([180, 80, 255])),  # High brightness, low saturation for white cubes
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

                    # Draw centroid and info on the output image
                    cv2.circle(output_image, (int(cX), int(cY)), 5, (0, 0, 255), -1)
                    cv2.putText(output_image, f"#{i} ({int(cX)},{int(cY)})",
                              (int(cX) - 40, int(cY) - 10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        return detected_objects, output_image, cleaned_mask

def send_script(script):
    arm_node = rclpy.create_node('arm')
    arm_cli = arm_node.create_client(SendScript, 'send_script')

    while not arm_cli.wait_for_service(timeout_sec=1.0):
        arm_node.get_logger().info('service not availabe, waiting again...')

    move_cmd = SendScript.Request()
    move_cmd.script = script
    arm_cli.call_async(move_cmd)
    arm_node.destroy_node()

def set_io(state):
    gripper_node = rclpy.create_node('gripper')
    gripper_cli = gripper_node.create_client(SetIO, 'set_io')

    while not gripper_cli.wait_for_service(timeout_sec=1.0):
        gripper_node.get_logger().info('service not available, waiting again...')

    io_cmd = SetIO.Request()
    io_cmd.module = 1
    io_cmd.type = 1
    io_cmd.pin = 0
    io_cmd.state = state
    gripper_cli.call_async(io_cmd)
    gripper_node.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageSub('image_sub')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()