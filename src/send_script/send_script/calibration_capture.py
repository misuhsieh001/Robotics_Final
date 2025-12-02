#!/usr/bin/env python
"""
Camera Calibration Image Capture for TM5-900
Captures 20 images with pauses to allow manual chessboard repositioning
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time
import sys

sys.path.append('/home/robot/colcon_ws/install/tm_msgs/lib/python3.6/site-packages')
from tm_msgs.msg import *
from tm_msgs.srv import *


class CalibrationCapture(Node):
    def __init__(self):
        super().__init__('calibration_capture')

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
        self.capture_interval = 5  # seconds between captures
        self.last_capture_time = time.time()
        self.capturing = False

        # Create output directory
        self.output_dir = '/home/robot/workspace2/team11_ws/calibration_images_tm5900'
        os.makedirs(self.output_dir, exist_ok=True)

        self.get_logger().info('=== TM5-900 Calibration Image Capture ===')
        self.get_logger().info(f'Will capture {self.num_images} images')
        self.get_logger().info(f'Saving to: {self.output_dir}')
        self.get_logger().info(f'Interval: {self.capture_interval} seconds between captures')
        self.get_logger().info('Starting in 3 seconds...')

        # Start capture after delay
        time.sleep(3)
        self.capturing = True
        self.get_logger().info('STARTING CAPTURE!')

    def image_callback(self, msg):
        if not self.capturing or self.current_image >= self.num_images:
            return

        current_time = time.time()
        time_since_last = current_time - self.last_capture_time

        # Check if it's time to capture
        if time_since_last >= self.capture_interval:
            try:
                # Convert ROS image to OpenCV
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

                # Save image
                filename = f'calib_{self.current_image:03d}.png'
                filepath = os.path.join(self.output_dir, filename)
                cv2.imwrite(filepath, cv_image)

                self.current_image += 1
                self.last_capture_time = current_time

                remaining = self.num_images - self.current_image
                self.get_logger().info(
                    f'✓ Captured image {self.current_image}/{self.num_images} - {filename}'
                )

                if remaining > 0:
                    self.get_logger().info(
                        f'>> Move/rotate the chessboard now! '
                        f'Next capture in {self.capture_interval} seconds... ({remaining} remaining)'
                    )
                else:
                    self.get_logger().info('=== CAPTURE COMPLETE! ===')
                    self.get_logger().info(f'Saved {self.num_images} images to {self.output_dir}')
                    self.get_logger().info('Now run calibration to compute camera matrix.')
                    self.capturing = False

            except Exception as e:
                self.get_logger().error(f'Error capturing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)

    print("\n" + "="*60)
    print("  TM5-900 Camera Calibration - Image Capture")
    print("="*60)
    print("\nInstructions:")
    print("1. Place 18x13 chessboard in camera view")
    print("2. Script will capture 20 images automatically")
    print("3. You have 5 seconds between each capture to:")
    print("   - Move the chessboard to a new position")
    print("   - Rotate/tilt it to a different angle")
    print("   - Vary the distance from camera")
    print("\nTips for good calibration:")
    print("- Cover different areas of the image")
    print("- Include tilted/angled views")
    print("- Vary distance (near and far)")
    print("- Keep chessboard flat and fully visible")
    print("\nStarting in 3 seconds...")
    print("="*60 + "\n")

    node = CalibrationCapture()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Capture interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
