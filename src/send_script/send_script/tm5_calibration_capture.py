#!/usr/bin/env python
"""
TM5-900 Camera Calibration Image Capture
Integrates robot control + image capture for calibration
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time
import sys

# Dynamically determine Python version for tm_msgs path
python_version = f"python{sys.version_info.major}.{sys.version_info.minor}"
tm_msgs_path = f'/home/robot/colcon_ws/install/tm_msgs/lib/{python_version}/site-packages'
sys.path.append(tm_msgs_path)
from tm_msgs.msg import *
from tm_msgs.srv import *

# Explicitly import required services
try:
    from tm_msgs.srv import SendScript
except ImportError:
    print("WARNING: tm_msgs.srv not found. Make sure tm_msgs package is installed.")
    print(f"Looked in: {tm_msgs_path}")
    raise


def send_script(script):
    """Send script command to TM5-900 arm"""
    arm_node = rclpy.create_node('arm')
    arm_cli = arm_node.create_client(SendScript, 'send_script')

    while not arm_cli.wait_for_service(timeout_sec=1.0):
        arm_node.get_logger().info('Waiting for send_script service...')

    move_cmd = SendScript.Request()
    move_cmd.script = script
    arm_cli.call_async(move_cmd)
    time.sleep(0.5)  # Give time for command to process
    arm_node.destroy_node()


class TM5CalibrationCapture(Node):
    def __init__(self):
        super().__init__('tm5_calibration_capture')

        # Subscribe to camera
        self.subscription = self.create_subscription(
            Image,
            'techman_image',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()

        # Calibration parameters
        self.num_images = 20
        self.current_image = 0
        self.ready_to_capture = False
        self.image_captured = False
        self.latest_image = None

        # Create output directory
        self.output_dir = '/home/robot/workspace2/team11_ws/calibration_images_tm5900'
        os.makedirs(self.output_dir, exist_ok=True)

        self.get_logger().info('=== TM5-900 Automated Calibration Capture ===')
        self.get_logger().info(f'Will capture {self.num_images} images from different angles')
        self.get_logger().info(f'Saving to: {self.output_dir}')
        self.get_logger().info('Robot will move automatically to each position')

    def image_callback(self, msg):
        """Store latest image"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {str(e)}')

    def capture_image(self):
        """Trigger camera capture and save image"""
        if self.latest_image is None:
            self.get_logger().warn('No image available yet...')
            return False

        try:
            # Save image
            filename = f'calib_{self.current_image:03d}.png'
            filepath = os.path.join(self.output_dir, filename)
            cv2.imwrite(filepath, self.latest_image)

            self.get_logger().info(f'✓ Saved {filename}')
            return True

        except Exception as e:
            self.get_logger().error(f'Error saving image: {str(e)}')
            return False

    def generate_calibration_positions(self):
        """Generate 20 different camera positions to view the fixed calibration board"""
        # Board is placed at approximately (230, 230) on the table
        # Generate positions in a grid pattern with varying heights and angles

        positions = []

        # Center position
        center_x, center_y = 230.0, 230.0

        # Generate 20 positions with varying X, Y, Z, and rotation around 730mm working height
        # Pattern: 4 heights x 5 positions each
        x_offsets = [-60, -20, 20, 60]  # 4 X positions
        y_offsets = [-50, -25, 0, 25, 50]  # 5 Y positions
        z_heights = [680, 705, 730, 755]  # 4 different heights around 730mm

        position_id = 0
        for z_idx, z in enumerate(z_heights):
            # For each height, select 5 different X,Y combinations
            for i in range(5):
                if position_id >= self.num_images:
                    break

                # Alternate between X and Y variations
                if i < len(x_offsets):
                    x_off = x_offsets[i]
                    y_off = y_offsets[i]
                else:
                    x_off = x_offsets[i % len(x_offsets)]
                    y_off = y_offsets[i % len(y_offsets)]

                x = center_x + x_off
                y = center_y + y_off

                # Vary the rotation for different angles
                rz = 135.0 + (position_id - 10) * 5  # Vary from 85° to 185°

                positions.append({
                    'id': position_id,
                    'x': x,
                    'y': y,
                    'z': z,
                    'rx': -180.0,
                    'ry': 0.0,
                    'rz': rz
                })

                position_id += 1

        return positions[:self.num_images]

    def run_calibration_sequence(self):
        """Main calibration capture sequence with automated arm movement"""
        self.get_logger().info('\n=== Starting Automated Calibration Sequence ===')
        self.get_logger().info('The robot will move to 20 different positions')
        self.get_logger().info('Keep the calibration board in a FIXED position!\n')

        time.sleep(3)  # Give user time to read

        # Generate calibration positions
        positions = self.generate_calibration_positions()

        self.get_logger().info(f'Generated {len(positions)} camera positions')
        self.get_logger().info('Starting automated capture...\n')

        for i, pos in enumerate(positions):
            self.current_image = i

            # Move robot to position
            self.get_logger().info(
                f'[{i+1}/{self.num_images}] Moving to position '
                f'({pos["x"]:.1f}, {pos["y"]:.1f}, {pos["z"]:.1f})...'
            )

            target = (f'{pos["x"]:.2f}, {pos["y"]:.2f}, {pos["z"]:.2f}, '
                     f'{pos["rx"]:.2f}, {pos["ry"]:.2f}, {pos["rz"]:.2f}')
            script = f'PTP("CPP",{target},100,200,0,false)'
            send_script(script)
            time.sleep(2.5)  # Wait for robot to reach position

            # Trigger camera via Vision_DoJob
            self.get_logger().info(f'  Triggering camera...')
            send_script("Vision_DoJob(job1)")
            time.sleep(1.0)  # Wait for camera capture

            # Capture and save
            if self.capture_image():
                self.get_logger().info(f'  ✓ Image {i+1} captured\n')
            else:
                self.get_logger().warn(f'  ✗ Failed to capture image {i+1}, continuing...\n')

        self.get_logger().info('\n=== CAPTURE COMPLETE ===')
        self.get_logger().info(f'Saved {self.num_images} images to {self.output_dir}')
        self.get_logger().info('Now run: ros2 run send_script calibration_process')


def main(args=None):
    rclpy.init(args=args)

    print("\n" + "="*70)
    print("  TM5-900 Camera Calibration - FULLY AUTOMATED Capture")
    print("="*70)
    print("\nThis script will:")
    print("1. Automatically move robot to 20 different positions")
    print("2. Capture images from different angles and distances")
    print("3. Save all images for calibration processing")
    print("\n⚠️  IMPORTANT PREPARATION:")
    print("- Place calibration board at position (230, 230) on table")
    print("- Keep the board FIXED in that position")
    print("- DO NOT move the board during capture")
    print("- Robot will move around the board automatically")
    print("\nBoard should be:")
    print("- 17x12 inner corners chessboard")
    print("- Flat on table or tilted at stable angle")
    print("- Well-lit and clearly visible")
    print("\nStarting in 5 seconds...")
    print("="*70 + "\n")

    time.sleep(5)

    node = TM5CalibrationCapture()

    # Run calibration in a separate thread
    import threading
    calib_thread = threading.Thread(target=node.run_calibration_sequence)
    calib_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Capture interrupted by user')
    finally:
        calib_thread.join(timeout=1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
