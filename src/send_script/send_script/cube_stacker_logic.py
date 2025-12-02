#!/usr/bin/env python
"""
Cube Stacking Logic Implementation for TM5-900

This implements the complete pick-and-stack logic:
1. Take one photo, detect all 5 cubes
2. Choose base cube (closest to image center)
3. For each other cube:
   - Move above cube
   - Rotate gripper perpendicular to cube's line
   - Pick up cube
   - Move to base
   - Rotate gripper perpendicular to base's line
   - Place on stack
   - Repeat
"""

import rclpy
from rclpy.node import Node
import time
import sys
import math
import numpy as np

# Dynamically determine Python version for tm_msgs path
python_version = f"python{sys.version_info.major}.{sys.version_info.minor}"
tm_msgs_path = f'/home/robot/colcon_ws/install/tm_msgs/lib/{python_version}/site-packages'
sys.path.append(tm_msgs_path)
from tm_msgs.msg import *
from tm_msgs.srv import *

# Explicitly import required services for clarity
try:
    from tm_msgs.srv import SendScript, SetIO
except ImportError:
    print("WARNING: tm_msgs.srv not found. Make sure tm_msgs package is installed.")
    print(f"Looked in: {tm_msgs_path}")
    raise

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# ==============================================================================
# Camera Calibration Settings
# ==============================================================================
CAMERA_MATRIX = np.array([
    [2711.03, 0.0,     1332.58],
    [0.0,     2709.31, 1013.72],
    [0.0,     0.0,     1.0    ]
], dtype=np.float32)

DIST_COEFFS = np.array([0.09845879, -0.89589613, -0.00460501, 0.00533473, 4.41194115], dtype=np.float32)

# ==============================================================================
# Hand-Eye Calibration Parameters (Pixel to Robot Transformation)
# ==============================================================================
# Transformation: X_robot = a1*u + b1*v + c1
#                 Y_robot = a2*u + b2*v + c2
# Calibration points used:
#   Image(1134,711) -> Arm(443.56,488.04)
#   Image(1911,706) -> Arm(553.29,375.31)
#   Image(1159,1414) -> Arm(347.21,378.89)
#   Image(1886,1403) -> Arm(457.76,275.65)
HAND_EYE_A1 = 0.1448444262
HAND_EYE_B1 = -0.1370812701
HAND_EYE_C1 = 375.0298854340

HAND_EYE_A2 = -0.1452133199
HAND_EYE_B2 = -0.1491544471
HAND_EYE_C2 = 758.4397617280


class CubeStacker(Node):
    def __init__(self):
        super().__init__('cube_stacker')

        # Image subscriber
        self.subscription = self.create_subscription(
            Image,
            'techman_image',
            self.image_callback,
            10)

        self.bridge = CvBridge()

        # Storage for detected cubes
        self.cubes_detected = []
        self.image_received = False

        # Fixed parameters
        self.camera_height = 500.0  # mm - actual camera position from line 436
        self.table_z = 0.0  # mm
        self.cube_height = 25.0  # mm
        self.camera_to_table_distance = 500.0  # mm - matches actual camera height
        self.safety_offset = 2.0  # mm - gap between base and first cube
        self.drop_compensation = 2.0  # mm - cube drops when gripper opens
        self.gripper_offset = 100.0  # mm - distance from arm flange to gripper tip

        # Image center (for choosing base cube)
        self.image_center_x = CAMERA_MATRIX[0, 2]  # cx = 1332.58
        self.image_center_y = CAMERA_MATRIX[1, 2]  # cy = 1013.72

        self.get_logger().info('Cube Stacker Node initialized')
        self.get_logger().info(f'Image center: ({self.image_center_x}, {self.image_center_y})')

    def image_callback(self, data):
        """Process incoming image and detect cubes"""
        if self.image_received:
            return  # Only process first image

        self.get_logger().info('Received image, detecting cubes...')

        try:
            # Convert ROS Image to OpenCV
            # Handle 8UC3 encoding (raw 3-channel image)
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            except Exception as e:
                # If bgr8 fails, try passthrough for 8UC3 format
                self.get_logger().info(f'BGR8 conversion failed, using passthrough for {data.encoding}')
                cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

            # Save the original captured image
            import os
            from datetime import datetime
            save_dir = '/home/robot/workspace2/team11_ws'
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            original_image_path = os.path.join(save_dir, f'captured_image_{timestamp}.png')
            cv2.imwrite(original_image_path, cv_image)
            self.get_logger().info(f'✓ Saved original image to: {original_image_path}')

            # Detect all cubes
            detected_objects, processed_image, mask = self.find_all_contours(cv_image)

            if len(detected_objects) < 2:
                self.get_logger().error(f'Only found {len(detected_objects)} cubes, need at least 2!')
                return

            # Limit to 5 cubes
            detected_objects = detected_objects[:5]

            # Calculate robot coordinates and draw annotations on image
            for obj in detected_objects:
                u, v = obj['centroid']
                X, Y, Z = self.image_to_robot_coords(u, v)
                obj['robot_coords'] = (X, Y, Z)

                self.get_logger().info(
                    f"Cube #{obj['id']}: Pixel ({u:.1f}, {v:.1f}) → "
                    f"Robot ({X:.1f}, {Y:.1f}, {Z:.1f}), Angle: {obj['angle_deg']:.1f}°"
                )

                # Draw enhanced annotations on the processed image
                self._draw_cube_annotations(processed_image, obj)

            self.cubes_detected = detected_objects
            self.image_received = True

            # Save the annotated image
            annotated_image_path = os.path.join(save_dir, f'annotated_image_{timestamp}.png')
            cv2.imwrite(annotated_image_path, processed_image)
            self.get_logger().info(f'✓ Saved annotated image to: {annotated_image_path}')

            # Display image
            cv2.namedWindow('Detected Cubes', cv2.WINDOW_NORMAL)
            cv2.imshow('Detected Cubes', processed_image)
            cv2.waitKey(100)  # Wait 100ms for window to appear

            self.get_logger().info(f'Successfully detected {len(self.cubes_detected)} cubes!')

            # Also display the mask for debugging
            cv2.namedWindow('Color Mask', cv2.WINDOW_NORMAL)
            cv2.imshow('Color Mask', mask)
            cv2.waitKey(100)

        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')

    def image_to_robot_coords(self, u, v):
        """Convert pixel coordinates to robot coordinates using hand-eye calibration"""
        # Use hand-eye calibration parameters
        # X_robot = a1*u + b1*v + c1
        # Y_robot = a2*u + b2*v + c2
        X_robot = HAND_EYE_A1 * u + HAND_EYE_B1 * v + HAND_EYE_C1
        Y_robot = HAND_EYE_A2 * u + HAND_EYE_B2 * v + HAND_EYE_C2

        # Z coordinate with gripper offset compensation
        # We want gripper tip at 300mm, so arm flange must be at 300mm + gripper_offset
        Z_desired_gripper = 300.0  # Where we want the gripper tip
        Z_robot = Z_desired_gripper + self.gripper_offset  # Command arm to compensate for gripper length

        return X_robot, Y_robot, Z_robot

    def _draw_cube_annotations(self, image, obj):
        """Draw enhanced annotations on cube including perpendicular line and coordinates"""
        cX, cY = obj['centroid']
        angle_deg = obj['angle_deg']
        angle_rad = obj['angle_rad']
        cube_id = obj['id']
        X, Y, Z = obj['robot_coords']

        # Draw centroid as a larger circle
        cv2.circle(image, (int(cX), int(cY)), 12, (0, 0, 255), -1)
        cv2.circle(image, (int(cX), int(cY)), 15, (255, 255, 255), 2)

        # Draw CUBE'S SIDE direction (blue line)
        # angle_rad is perpendicular to side, so cube's side is at angle_rad - pi/2
        line_length = 80
        cube_side_rad = angle_rad - math.pi / 2.0  # Cube's side direction
        side_line_end_x = int(cX + line_length * math.cos(cube_side_rad))
        side_line_end_y = int(cY + line_length * math.sin(cube_side_rad))
        cv2.line(image, (int(cX), int(cY)), (side_line_end_x, side_line_end_y), (255, 0, 0), 4)

        # Draw GRIPPER direction (green arrow) - perpendicular to cube's side
        # angle_rad is already the perpendicular to the cube's side
        # This is the direction the gripper will approach from
        gripper_angle_rad = angle_rad  # Perpendicular to cube side
        gripper_line_end_x = int(cX + line_length * math.cos(gripper_angle_rad))
        gripper_line_end_y = int(cY + line_length * math.sin(gripper_angle_rad))

        # Add arrowhead to show gripper approach direction
        cv2.arrowedLine(image, (int(cX), int(cY)), (gripper_line_end_x, gripper_line_end_y),
                       (0, 255, 0), 4, tipLength=0.2)

        # Prepare text annotations
        font = cv2.FONT_HERSHEY_SIMPLEX

        # Text 1: Cube ID and angle
        text_id = f"Cube #{cube_id}"
        text_angle = f"Angle: {angle_deg:.1f}°"

        # Text 2: Pixel coordinates
        text_pixel = f"Pixel: ({int(cX)}, {int(cY)})"

        # Text 3: Robot coordinates
        text_robot = f"Robot: ({X:.1f}, {Y:.1f})"

        # Font sizes
        font_scale_large = 1.2  # For cube ID
        font_scale_medium = 0.9  # For other info
        thickness_bold = 3
        thickness_normal = 2

        # Position text to the right of the cube
        text_x = int(cX) + 30
        text_y_start = int(cY) - 60
        line_spacing = 35

        # Draw text with black background for better visibility
        def draw_text_with_background(img, text, pos, font_scale, thickness, color=(255, 255, 255)):
            (text_w, text_h), baseline = cv2.getTextSize(text, font, font_scale, thickness)
            # Draw black background rectangle
            cv2.rectangle(img,
                         (pos[0] - 5, pos[1] - text_h - 5),
                         (pos[0] + text_w + 5, pos[1] + baseline + 5),
                         (0, 0, 0), -1)
            # Draw white text
            cv2.putText(img, text, pos, font, font_scale, color, thickness)

        # Draw Cube ID (larger, yellow)
        draw_text_with_background(image, text_id, (text_x, text_y_start),
                                 font_scale_large, thickness_bold, (0, 255, 255))

        # Draw Angle (cyan)
        draw_text_with_background(image, text_angle, (text_x, text_y_start + line_spacing),
                                 font_scale_medium, thickness_normal, (255, 255, 0))

        # Draw Pixel coordinates (white)
        draw_text_with_background(image, text_pixel, (text_x, text_y_start + line_spacing * 2),
                                 font_scale_medium, thickness_normal, (255, 255, 255))

        # Draw Robot coordinates (green)
        draw_text_with_background(image, text_robot, (text_x, text_y_start + line_spacing * 3),
                                 font_scale_medium, thickness_normal, (0, 255, 0))

        # Draw legend at bottom of cube
        legend_y = int(cY) + 80
        draw_text_with_background(image, "Blue: Cube side", (text_x, legend_y),
                                 0.6, 1, (255, 150, 0))
        draw_text_with_background(image, "Green: Gripper (perp to side)", (text_x, legend_y + 25),
                                 0.6, 1, (0, 255, 150))

    def find_all_contours(self, img):
        """Find contours and calculate centroids of all objects"""
        output_image = img.copy()
        hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Color ranges
        color_ranges = {
            #'red1': (np.array([0, 50, 50]), np.array([10, 255, 255])),
            #'red2': (np.array([170, 50, 50]), np.array([180, 255, 255])),
            'green': (np.array([35, 60, 100]), np.array([85, 255, 255])),
            'blue': (np.array([90, 50, 50]), np.array([130, 255, 255])),
            'yellow': (np.array([15, 100, 100]), np.array([35, 255, 255])),
            'white': (np.array([0, 0, 100]), np.array([180, 80, 255])),
        }

        # Create combined mask
        combined_mask = np.zeros(img.shape[:2], dtype="uint8")
        for color_name, (lower, upper) in color_ranges.items():
            mask = cv2.inRange(hsv_image, lower, upper)
            if color_name == 'red1':
                mask_red2 = cv2.inRange(hsv_image, color_ranges['red2'][0], color_ranges['red2'][1])
                mask = cv2.bitwise_or(mask, mask_red2)
            elif color_name == 'red2':
                continue
            combined_mask = cv2.bitwise_or(combined_mask, mask)

        # Morphological operations
        kernel_open = np.ones((5, 5), np.uint8)
        mask_opened = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel_open, iterations=2)
        kernel_close = np.ones((15, 15), np.uint8)
        cleaned_mask = cv2.morphologyEx(mask_opened, cv2.MORPH_CLOSE, kernel_close, iterations=2)

        # Find contours
        contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_objects = []
        for i, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            if area > 1500:
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)

                if len(approx) >= 3:
                    M = cv2.moments(cnt)
                    if M["m00"] == 0:
                        continue

                    cX = M["m10"] / M["m00"]
                    cY = M["m01"] / M["m00"]

                    # Calculate angle using minimum area rectangle (better for rectangular cubes)
                    # minAreaRect returns: ((center_x, center_y), (width, height), angle)
                    # angle is in range [-90, 0) degrees
                    angle_deg = 0.0
                    angle_rad = 0.0
                    try:
                        rect = cv2.minAreaRect(cnt)
                        # Get the angle of the rectangle's side
                        rect_angle = rect[2]
                        # The perpendicular to the cube's side
                        # This is the direction the gripper should approach from
                        angle_deg = rect_angle + 90.0
                        # Normalize to [0, 180) range
                        if angle_deg >= 180:
                            angle_deg -= 180
                        elif angle_deg < 0:
                            angle_deg += 180
                        angle_rad = angle_deg * math.pi / 180.0
                    except:
                        pass

                    detected_objects.append({
                        'id': i,
                        'contour': cnt,
                        'centroid': (cX, cY),
                        'area': area,
                        'angle_deg': angle_deg,
                        'angle_rad': angle_rad,
                        'vertices': len(approx)
                    })

                    # Draw only contour outline (annotations will be drawn later)
                    cv2.drawContours(output_image, [cnt], -1, (0, 255, 0), 3)

        return detected_objects, output_image, cleaned_mask

    def choose_base_cube(self, cubes):
        """
        Choose the cube closest to the image center as the base.
        This cube will NOT be moved.
        """
        min_distance = float('inf')
        base_cube = None
        base_index = -1

        for i, cube in enumerate(cubes):
            u, v = cube['centroid']
            # Calculate distance from image center
            distance = math.sqrt((u - self.image_center_x)**2 + (v - self.image_center_y)**2)

            if distance < min_distance:
                min_distance = distance
                base_cube = cube
                base_index = i

        self.get_logger().info(f'=== BASE CUBE SELECTED ===')
        self.get_logger().info(f'Base: Cube #{base_cube["id"]} at pixel ({base_cube["centroid"][0]:.1f}, {base_cube["centroid"][1]:.1f})')
        self.get_logger().info(f'Distance from image center: {min_distance:.1f} pixels')
        self.get_logger().info(f'Base angle: {base_cube["angle_deg"]:.1f}°')
        self.get_logger().info(f'Base robot coords: {base_cube["robot_coords"]}')

        return base_cube, base_index


def send_script(script):
    """Send movement command to robot"""
    arm_node = rclpy.create_node('arm')
    arm_cli = arm_node.create_client(SendScript, 'send_script')
    print('==========================')
    print(script)
    print('==========================')
    while not arm_cli.wait_for_service(timeout_sec=1.0):
        arm_node.get_logger().info('service not available, waiting again...')

    move_cmd = SendScript.Request()
    move_cmd.script = script
    future = arm_cli.call_async(move_cmd)
    rclpy.spin_until_future_complete(arm_node, future)
    arm_node.destroy_node()


def set_io(state):
    """Control gripper (1.0 = close, 0.0 = open)"""
    gripper_node = rclpy.create_node('gripper')
    gripper_cli = gripper_node.create_client(SetIO, 'set_io')

    while not gripper_cli.wait_for_service(timeout_sec=1.0):
        gripper_node.get_logger().info('service not available, waiting again...')

    io_cmd = SetIO.Request()
    io_cmd.module = 1
    io_cmd.type = 1
    io_cmd.pin = 0
    io_cmd.state = state
    future = gripper_cli.call_async(io_cmd)
    rclpy.spin_until_future_complete(gripper_node, future)
    gripper_node.destroy_node()


def wait_for_enter(logger, message="Press ENTER to continue..."):
    """Wait for user to press Enter before continuing"""
    import sys
    import select

    logger.info(f'→ {message}')
    while True:
        cv2.waitKey(100)  # Keep processing OpenCV events
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline()
            break


def pick_and_place(cube, base_cube, stack_height, logger):

    """
    Pick up a cube and place it on the base cube.

    Args:
        cube: The cube to pick up
        base_cube: The base cube to stack on
        stack_height: Current height of the stack (mm)
        logger: ROS logger for output
    """
    X, Y, Z = cube['robot_coords']
    cube_angle = cube['angle_deg']

    base_X, base_Y, base_Z = base_cube['robot_coords']
    base_angle = base_cube['angle_deg']

    # Calculate gripper angle - gripper aligns with perpendicular to cube's side
    # angle_deg already represents the perpendicular to the cube's side
    # Transform from image coordinates to robot coordinates
    gripper_angle_pickup = 135 - cube_angle

    # Normalize to -180 to 180 range for robot
    if gripper_angle_pickup > 90:
        gripper_angle_pickup = gripper_angle_pickup - 180
    elif gripper_angle_pickup < -90:
        gripper_angle_pickup = gripper_angle_pickup + 180

    # Calculate gripper angle for base cube (same transformation)
    gripper_angle_place = 135 - base_angle
    if gripper_angle_place > 90:
        gripper_angle_place = gripper_angle_place - 180
    elif gripper_angle_place < -90:
        gripper_angle_place = gripper_angle_place + 180

    logger.info(f'')
    logger.info(f'=== PICKING CUBE #{cube["id"]} ===')
    logger.info(f'Cube position: ({X:.1f}, {Y:.1f}, {Z:.1f})')
    logger.info(f'Cube angle: {cube_angle:.1f}° (perpendicular to cube side)')
    logger.info(f'Gripper angle for pickup: {gripper_angle_pickup:.1f}° (perpendicular to cube side)')

    # Safety height above cubes (gripper tip position)
    # Need to add gripper offset for arm command

    safe_height_gripper = 300.0  # mm - where we want gripper tip
    safe_height = safe_height_gripper  # mm - arm flange command position

    # Step 1: Move above the cube
    logger.info(f'Step 1: Moving above cube #{cube["id"]}...')
    wait_for_enter(logger, "Press ENTER to move above cube...")
    send_script(f'PTP("CPP",{X:.2f},{Y:.2f},{safe_height},-180.00,0.0,{gripper_angle_pickup:.2f},100,200,0,false)')
    time.sleep(0.5)

    # Step 2a: Move down to approach cube (rotate gripper perpendicular to cube's line)
    logger.info(f'Step 2: Moving down to cube...')
    logger.info(f'  → Target position: X={X:.2f}, Y={Y:.2f}, Z={(Z-270):.2f} mm (arm flange)')
    logger.info(f'  → Gripper tip will be at: Z={(Z-270):.2f} mm')
    wait_for_enter(logger, "Press ENTER to move down to cube...")
    send_script(f'PTP("CPP",{X:.2f},{Y:.2f},{(Z-270):.2f},-180.00,0.0,{gripper_angle_pickup:.2f},100,200,0,false)')
    time.sleep(0.5)

    # Step 2: Move down to cube (rotate gripper perpendicular to cube's line)
    logger.info(f'Step 2: Moving down to cube...')
    logger.info(f'  → Target position: X={X:.2f}, Y={Y:.2f}, Z={(Z-280):.2f} mm (arm flange)')
    logger.info(f'  → Gripper tip will be at: Z={(Z-280):.2f} mm')
    wait_for_enter(logger, "Press ENTER to move down to cube...")
    send_script(f'PTP("CPP",{X:.2f},{Y:.2f},{(Z-280):.2f},-180.00,0.0,{gripper_angle_pickup:.2f},100,200,0,false)')
    time.sleep(0.5)

    # Step 3: Close gripper (grab cube)
    logger.info(f'Step 3: Closing gripper...')
    wait_for_enter(logger, "Press ENTER to close gripper...")
    set_io(1.0)
    time.sleep(1.0)  # Wait for gripper to close fully

    # Step 4: Lift cube up
    logger.info(f'Step 4: Lifting cube...')
    wait_for_enter(logger, "Press ENTER to lift cube...")
    send_script(f'PTP("CPP",{X:.2f},{Y:.2f},{safe_height},-180.00,0.0,{gripper_angle_pickup:.2f},100,200,0,false)')
    time.sleep(0.5)

    # Step 5: Move above base cube
    logger.info(f'Step 5: Moving above base cube...')
    wait_for_enter(logger, "Press ENTER to move above base cube...")
    send_script(f'PTP("CPP",{base_X:.2f},{base_Y:.2f},{safe_height},-180.00,0.0,{gripper_angle_pickup:.2f},100,200,0,false)')
    time.sleep(0.5)

    # Step 6: Rotate gripper perpendicular to base cube's line
    logger.info(f'Step 6: Rotating gripper to {gripper_angle_place:.1f}° (perpendicular to base)...')
    wait_for_enter(logger, "Press ENTER to rotate gripper...")
    send_script(f'PTP("CPP",{base_X:.2f},{base_Y:.2f},{safe_height},-180.00,0.0,{gripper_angle_place:.2f},100,200,0,false)')
    time.sleep(0.5)

    # Step 7: Lower onto stack
    logger.info(f'Step 7: Placing on stack at height {stack_height:.1f}mm...')
    wait_for_enter(logger, "Press ENTER to lower onto stack...")
    send_script(f'PTP("CPP",{base_X:.2f},{base_Y:.2f},{stack_height:.2f},-180.00,0.0,{gripper_angle_place:.2f},50,200,0,false)')
    time.sleep(0.5)

    # Step 8: Open gripper (release cube)
    logger.info(f'Step 8: Opening gripper...')
    wait_for_enter(logger, "Press ENTER to open gripper...")
    set_io(0.0)
    time.sleep(1.0)

    # Step 9: Move straight up (Z-axis only) to avoid collision with stack
    # IMPORTANT: X,Y coordinates stay the same (base_X, base_Y) to guarantee vertical movement
    # Only Z changes: stack_height → safe_height
    logger.info(f'Step 9: Moving straight up to avoid collision...')
    wait_for_enter(logger, "Press ENTER to move up...")
    send_script(f'PTP("CPP",{base_X:.2f},{base_Y:.2f},{safe_height},-180.00,0.0,{gripper_angle_place:.2f},100,200,0,false)')
    time.sleep(0.5)

    logger.info(f'✓ Cube #{cube["id"]} successfully placed on stack!')


def main(args=None):
    rclpy.init(args=args)

    # Create cube stacker node
    stacker = CubeStacker()

    logger = stacker.get_logger()
    logger.info('==============================================')
    logger.info('  TM5-900 CUBE STACKING - STARTING')
    logger.info('==============================================')

    # Step 1: Move to camera position
    logger.info('Step 1: Moving to camera position...')
    targetP1 = "360.00, 360, 600, -180.00, 0.0, 135.00"
    script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
    send_script(script1)
    time.sleep(1.0)

    # Step 2: Capture image and detect cubes
    logger.info('Step 2: Capturing image...')
    send_script("Vision_DoJob(job1)")

    # Wait for image processing
    logger.info('Waiting for image processing...')
    timeout = 20.0  # 10 second timeout
    start_time = time.time()
    while not stacker.image_received:
        rclpy.spin_once(stacker, timeout_sec=0.1)
        if time.time() - start_time > timeout:
            logger.error('Timeout waiting for image!')
            rclpy.shutdown()
            return

    logger.info(f'Image processed! Detected {len(stacker.cubes_detected)} cubes.')

    if len(stacker.cubes_detected) < 2:
        logger.error('Need at least 2 cubes to stack!')
        cv2.destroyAllWindows()
        rclpy.shutdown()
        return

    # Keep windows open for viewing and wait for user confirmation
    logger.info('')
    logger.info('═══════════════════════════════════════════════════')
    logger.info('  Image windows displayed. Review detected cubes.')
    wait_for_enter(stacker.get_logger(), "Press ENTER to start stacking process...")
    logger.info('═══════════════════════════════════════════════════')
    logger.info('')

    # Step 3: Choose base cube (closest to image center)
    logger.info('Step 3: Choosing base cube...')
    base_cube, base_index = stacker.choose_base_cube(stacker.cubes_detected)

    # Get cubes to stack (all except base)
    cubes_to_stack = [cube for i, cube in enumerate(stacker.cubes_detected) if i != base_index]

    logger.info(f'')
    logger.info(f'=== STACKING PLAN ===')
    logger.info(f'Base cube: #{base_cube["id"]} (will NOT move)')
    logger.info(f'Cubes to stack: {len(cubes_to_stack)}')
    for cube in cubes_to_stack:
        logger.info(f'  - Cube #{cube["id"]} at {cube["robot_coords"]}')
    logger.info(f'')

    # Step 4: Stack cubes one by one
    # Initial stack height accounts for:
    # - Base cube top: 25mm
    # - Half cube height: 12.5mm (to center)
    # - Drop compensation: 2mm (cube drops when gripper opens)
    # - Gripper offset: 100mm (distance from arm flange to gripper tip)
    # Place at (25 + 12.5 + 2 + 100) = 139.5mm arm position → gripper at 39.5mm → drops to 37.5mm
    stack_height_gripper = stacker.table_z + stacker.cube_height + (stacker.cube_height / 2.0) + stacker.drop_compensation
    stack_height = stack_height_gripper + stacker.gripper_offset  # Add gripper offset for arm command
    stack_height += 15.0
    for i, cube in enumerate(cubes_to_stack):
        logger.info(f'')
        logger.info(f'╔════════════════════════════════════════════╗')
        logger.info(f'║  STACKING CUBE {i+1}/{len(cubes_to_stack)}                     ║')
        logger.info(f'╚════════════════════════════════════════════╝')

        # Pick and place this cube
        pick_and_place(cube, base_cube, stack_height, logger)

        # Increase stack height for next cube
        # Only add cube_height (cubes touch after drop)
        # Drop compensation already built into each placement
        stack_height += stacker.cube_height
        logger.info(f'Stack height now: {stack_height:.1f}mm (cubes touch after 2mm drop)')

        # Small delay between cubes
        time.sleep(1.0)

    # All done!
    logger.info(f'')
    logger.info(f'==============================================')
    logger.info(f'  ✓✓✓ ALL CUBES STACKED SUCCESSFULLY! ✓✓✓')
    logger.info(f'==============================================')
    logger.info(f'Total cubes stacked: {len(cubes_to_stack)}')
    logger.info(f'Final stack height: {stack_height:.1f}mm')
    logger.info(f'')

    # Move to home position
    logger.info('Moving to home position...')
    send_script("PTP(\"CPP\",230.00,230,730,-180.00,0.0,135.00,100,200,0,false)")

    # Cleanup
    stacker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
