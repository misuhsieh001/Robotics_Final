#!/usr/bin/env python
"""
Simple test script to verify camera feed and face detection
Run this before starting the full vlogger system
"""

import rclpy
from rclpy.node import Node
import sys
import cv2
import numpy as np

python_version = f"python{sys.version_info.major}.{sys.version_info.minor}"
tm_msgs_path = f'/home/robot/colcon_ws/install/tm_msgs/lib/{python_version}/site-packages'
sys.path.append(tm_msgs_path)

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Try to import MediaPipe
try:
    import mediapipe as mp
    MEDIAPIPE_AVAILABLE = True
    print("✓ MediaPipe is available")
except ImportError:
    MEDIAPIPE_AVAILABLE = False
    print("✗ MediaPipe not available (gesture recognition disabled)")

class CameraTest(Node):
    def __init__(self):
        super().__init__('camera_test')

        self.subscription = self.create_subscription(
            Image,
            'techman_image',
            self.image_callback,
            10)

        self.bridge = CvBridge()

        # Face detection
        cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        self.face_cascade = cv2.CascadeClassifier(cascade_path)

        # Hand detection
        if MEDIAPIPE_AVAILABLE:
            self.mp_hands = mp.solutions.hands
            self.hands = self.mp_hands.Hands(
                static_image_mode=False,
                max_num_hands=2,
                min_detection_confidence=0.5
            )
            self.mp_drawing = mp.solutions.drawing_utils

        self.frame_count = 0
        self.get_logger().info('Camera test node initialized')
        self.get_logger().info('Press "q" to quit')

    def image_callback(self, data):
        try:
            # Convert image
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            self.frame_count += 1

            # Create display
            display = cv_image.copy()

            # Draw frame info
            cv2.putText(display, f"Frame: {self.frame_count}",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Detect faces
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray, 1.1, 5, minSize=(100, 100))

            for (x, y, w, h) in faces:
                cv2.rectangle(display, (x, y), (x+w, y+h), (255, 0, 0), 3)
                cv2.putText(display, "FACE", (x, y-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

            cv2.putText(display, f"Faces: {len(faces)}",
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Detect hands
            if MEDIAPIPE_AVAILABLE:
                rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                results = self.hands.process(rgb_image)

                hand_count = 0
                if results.multi_hand_landmarks:
                    hand_count = len(results.multi_hand_landmarks)
                    for hand_landmarks in results.multi_hand_landmarks:
                        self.mp_drawing.draw_landmarks(
                            display,
                            hand_landmarks,
                            self.mp_hands.HAND_CONNECTIONS
                        )

                cv2.putText(display, f"Hands: {hand_count}",
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Draw image center crosshair
            h, w = display.shape[:2]
            center_x, center_y = w // 2, h // 2
            cv2.circle(display, (center_x, center_y), 10, (0, 255, 255), 2)
            cv2.line(display, (center_x - 30, center_y), (center_x + 30, center_y),
                    (0, 255, 255), 2)
            cv2.line(display, (center_x, center_y - 30), (center_x, center_y + 30),
                    (0, 255, 255), 2)

            # Show image
            cv2.imshow('Camera Test', display)
            key = cv2.waitKey(1)

            if key == ord('q'):
                self.get_logger().info('Quit requested')
                raise KeyboardInterrupt

        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

def main(args=None):
    print("=" * 60)
    print("  TM5-900 Camera Test")
    print("=" * 60)
    print()
    print("This test will:")
    print("  1. Connect to the robot's camera feed")
    print("  2. Display the video stream")
    print("  3. Detect faces in real-time")
    print("  4. Detect hands (if MediaPipe is available)")
    print()
    print("Instructions:")
    print("  - Position yourself in front of the camera")
    print("  - Move around to test face tracking")
    print("  - Show your hands to test hand detection")
    print("  - Press 'q' to quit")
    print()
    print("=" * 60)
    print()

    rclpy.init(args=args)

    node = CameraTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nTest completed!")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

    print()
    print("=" * 60)
    print("Camera test finished!")
    print("If everything worked, you can now run: python3 vlogger_control.py")
    print("=" * 60)

if __name__ == '__main__':
    main()
