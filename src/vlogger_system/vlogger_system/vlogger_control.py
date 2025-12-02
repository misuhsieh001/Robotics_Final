#!/usr/bin/env python
"""
TM5-900 Indoor Vlogger System

This system uses the TM5-900 robot arm as an automated vlogger that:
1. Holds a GoPro camera (with stick)
2. Detects and tracks human face/skeleton
3. Keeps the human centered in the camera frame
4. Maintains a fixed distance from the human
5. Responds to finger gestures:
   - 1 finger: Move closer
   - 5 fingers: Back up
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import sys
import math
import numpy as np
import cv2
from collections import deque
import os
import threading

# Add venv site-packages to Python path for MediaPipe
venv_site_packages = '/home/robot/workspace2/team11_ws_final_project/Robotics_Final_Project/venv/lib/python3.12/site-packages'
if os.path.exists(venv_site_packages) and venv_site_packages not in sys.path:
    sys.path.insert(0, venv_site_packages)

# Dynamically determine Python version for tm_msgs path
python_version = f"python{sys.version_info.major}.{sys.version_info.minor}"
tm_msgs_path = f'/home/robot/colcon_ws/install/tm_msgs/lib/{python_version}/site-packages'
sys.path.append(tm_msgs_path)

try:
    from tm_msgs.srv import SendScript, SetIO
    from tm_msgs.msg import FeedbackState
except ImportError:
    print("WARNING: tm_msgs not found. Make sure tm_msgs package is installed.")
    print(f"Looked in: {tm_msgs_path}")
    raise

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Try to import MediaPipe for hand detection
try:
    import mediapipe as mp
    MEDIAPIPE_AVAILABLE = True
except ImportError:
    print("WARNING: MediaPipe not available. Gesture recognition will be disabled.")
    print("Install with: pip install mediapipe")
    MEDIAPIPE_AVAILABLE = False


# ==============================================================================
# Camera Settings
# ==============================================================================
# Camera resolution: 2688 × 2048 pixels
# Image center is used as the target position for centering the human
IMAGE_CENTER_X = 1332.58  # cx - horizontal center of image
IMAGE_CENTER_Y = 1013.72  # cy - vertical center of image

# Note: Full camera calibration (focal length, distortion) not needed because:
# - Face size ratio is used for distance estimation (monocular vision)
# - No 3D reconstruction or undistortion required
# - Only image center is needed for centering calculations


class VloggerController(Node):
    def detect_camera_topic(self):
        """Detect available camera topic"""
        import subprocess
        try:
            # Get list of topics
            result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True, timeout=2)
            topics = result.stdout.split('\n')

            # Priority order of camera topics to try (with leading slash for proper matching)
            camera_candidates = [
                '/camera/color/image_raw',  # RealSense/USB camera
                '/camera/image_raw',         # Generic camera
                '/usb_cam/image_raw',        # USB camera
                'techman_image',             # Techman vision (requires Vision_DoJob)
            ]

            for topic in camera_candidates:
                if topic in topics:
                    return topic

            # Default to techman_image if nothing else found
            return 'techman_image'

        except Exception as e:
            # If detection fails, use techman_image as default
            return 'techman_image'
    def __init__(self):
        super().__init__('vlogger_controller')

        # Image subscriber - Try multiple camera sources
        # Priority: 1) USB camera, 2) ROS camera, 3) Techman vision
        self.camera_topic = self.detect_camera_topic()
        self.get_logger().info(f'Using camera topic: {self.camera_topic}')

        # Use a custom QoS profile for best performance and lowest latency
        # Reliability: BEST_EFFORT (drop packets if network is slow)
        # History: KEEP_LAST (only keep the latest messages)
        # Depth: 1 (only keep the very latest message, drop everything else)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            qos_profile)

        self.bridge = CvBridge()

        # Performance optimization: Frame skipping
        # Only process every Nth frame to reduce CPU load
        # For responsive behavior, process every frame by default. Increase this if CPU bound.
        self.process_every_n_frames = 1  # Process every frame (was 3)
        self.processed_frame_count = 0

        # Image center (target position for human)
        # Will be updated based on actual image size
        self.image_center_x = 320.0
        self.image_center_y = 240.0

        # Tracking parameters
        self.current_human_pos = None  # (x, y) in image coordinates
        self.target_distance = 800.0  # mm - desired distance from human
        self.min_distance = 300.0  # mm
        self.max_distance = 700.0  # mm

        # Distance estimation using face size (monocular vision)
        # Average human face width: ~150mm
        # Using similar triangles: face_size_pixels = (face_width_mm * focal_length) / distance_mm
        self.average_face_width_mm = 150.0  # mm - average adult face width
        
        # Adjusted for 640x480 resolution (approx 1/4 of original 2592 width)
        self.target_face_size = 100.0  # pixels - target face size for good framing (was 400)
        self.face_size_tolerance = 25.0  # pixels - acceptable variation (was 100)
        self.auto_distance_adjust = True  # Enable automatic distance adjustment based on face size

        # Movement parameters
        self.centering_threshold = 25  # pixels - if human is within this, don't move (was 100)
        self.movement_scale = 0.3  # Scale factor for movement (prevent overshooting)
        # Allow finer distance adjustments to react when face size changes
        self.min_movement = 2.0  # mm - minimum movement to execute (lowered from 10.0)

        # Current robot position (initialize at starting position)
        self.current_x = 300.0
        self.current_y = 300.0
        self.current_z = 400.0  # Starting height for vlogging
        self.current_rx = 90.0
        self.current_ry = 0.0
        self.current_rz = 50.0

        # Face detection (OpenCV Haar Cascade)
        # cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        # self.face_cascade = cv2.CascadeClassifier(cascade_path)

        # Hand detection and gesture recognition (MediaPipe)
        self.gesture_mode = MEDIAPIPE_AVAILABLE
        if self.gesture_mode:
            self.mp_hands = mp.solutions.hands
            self.hands = self.mp_hands.Hands(
                static_image_mode=False,
                max_num_hands=1,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5
            )
            # --- 新增 Face Mesh 初始化 ---
            self.mp_face_mesh = mp.solutions.face_mesh
            self.face_mesh = self.mp_face_mesh.FaceMesh(
                max_num_faces=1, # 只追蹤一張臉
                refine_landmarks=True, # 啟用地標精煉 (提高精度)
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5
            )
            self.mp_drawing = mp.solutions.drawing_utils

        # Performance tuning flags
        # Disable expensive per-frame face-mesh landmark drawing to improve FPS
        self.draw_face_mesh = False

        # Gesture commands (with debouncing)
        self.last_gesture = None
        self.gesture_cooldown = 2.0  # seconds
        self.last_gesture_time = 0.0

        # Smoothing for human position
        self.position_history = deque(maxlen=5)  # Average last 5 positions

        # FPS calculation for live view
        self.frame_count = 0
        self.fps_start_time = time.time()
        self.current_fps = 0.0

        # Latest display image (for window updates)
        self.latest_display_image = None
        self.image_lock = threading.Lock()  # Thread-safe lock for image synchronization
        self.latest_frame_timestamp = 0.0  # Timestamp of latest frame
        self.last_displayed_timestamp = -1.0  # Timestamp of last displayed frame (start with -1 to force initial update)

        # Control loop timer (1 Hz - User requested movement every 1 second)
        self.control_timer = self.create_timer(1.0, self.control_loop)
        self.last_move_time = time.time()
        
        # Window update timer (30 Hz - keep window responsive)
        self.window_timer = self.create_timer(0.033, self.update_window)

        # Camera trigger timer - DISABLED, will use callback-based triggering instead
        # Vision_DoJob takes ~3 seconds to process, so we trigger after each image arrives
        # self.camera_trigger_timer = self.create_timer(0.2, self.trigger_camera_capture)
        self.waiting_for_image = False
        self.last_trigger_time = 0.0

        # Create persistent service client for robot commands (FIX: avoid creating new node each time)
        self.arm_client = self.create_client(SendScript, 'send_script')
        self.get_logger().info('Waiting for send_script service...')
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('send_script service not available, waiting...')

        # Create live view window with error handling
        try:
            import os
            display = os.environ.get('DISPLAY', 'not set')
            self.get_logger().info(f'DISPLAY environment variable: {display}')
            
            # Destroy any existing windows first
            cv2.destroyAllWindows()
            cv2.waitKey(1)
            
            # Create window with specific flags to prevent crashes
            cv2.namedWindow('Vlogger View', cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
            cv2.resizeWindow('Vlogger View', 1280, 960)
            cv2.moveWindow('Vlogger View', 100, 100)
            
            # Create initial test image with instructions
            test_img = np.zeros((960, 1280, 3), dtype=np.uint8)
            cv2.putText(test_img, "TM5-900 VLOGGER SYSTEM", 
                       (400, 400), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
            cv2.putText(test_img, "Initializing camera...", 
                       (480, 480), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            cv2.putText(test_img, "Window will update when camera feed is received", 
                       (350, 540), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1)
            
            cv2.imshow('Vlogger View', test_img)
            cv2.waitKey(100)  # Longer wait to ensure window is created
            
            self.latest_display_image = test_img  # Initialize with test image
            self.window_created = True
            self.get_logger().info('Live view window created successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to create window: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.window_created = False

        self.get_logger().info('==============================================')
        self.get_logger().info('  TM5-900 VLOGGER CONTROLLER INITIALIZED')
        self.get_logger().info('==============================================')
        self.get_logger().info(f'Image center: ({self.image_center_x:.1f}, {self.image_center_y:.1f})')
        self.get_logger().info(f'Target distance: {self.target_distance:.1f}mm')
        self.get_logger().info(f'Gesture recognition: {"ENABLED" if self.gesture_mode else "DISABLED"}')
        self.get_logger().info(f'Live view window: {"ENABLED" if self.window_created else "DISABLED (check DISPLAY)"}')
        self.get_logger().info('==============================================')
        self.get_logger().info('📷 Camera: Using Vision_DoJob(job1) with callback-based triggering')
        self.get_logger().info('⏳ Triggering initial capture...')
        self.get_logger().info('==============================================')
        
        # Diagnostic flags
        self.first_image_received = False
        self.image_count = 0
        self.camera_trigger_count = 0
        self._capture_in_progress = False  # Prevent re-entrant camera captures

    def image_callback(self, data):
        """Process incoming image to detect human and gestures"""
        # Count first image received (only log first arrival)
        self.image_count += 1
        if not self.first_image_received:
            self.get_logger().info(f'📸 FIRST IMAGE RECEIVED: {data.width}x{data.height}, encoding={data.encoding}')
            self.first_image_received = True

        # Mark that we received an image
        self.waiting_for_image = False

        try:
            # Convert ROS Image to OpenCV
            # self.get_logger().info('Converting image to OpenCV format...')
            if data.encoding == '8UC3':
                # 8UC3 is generic, assume it's BGR/RGB compatible and pass through
                cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            else:
                cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            
            # Resize image for performance (target width ~640px)
            # Original is 2592x1944, so scale factor approx 0.25
            # Use a smaller target width to improve FPS on low-power systems
            target_width = getattr(self, 'target_width', 480)
            scale = target_width / cv_image.shape[1]
            width = int(cv_image.shape[1] * scale)
            height = int(cv_image.shape[0] * scale)
            dim = (width, height)
            
            # Perform resize
            cv_image = cv2.resize(cv_image, dim, interpolation=cv2.INTER_AREA)
            
            # Update image center based on actual resized dimensions
            self.image_center_x = width / 2.0
            self.image_center_y = height / 2.0
            
            # self.get_logger().info(f'Converted & Resized! Shape: {cv_image.shape}')

            # Create display image
            display_image = cv_image.copy()

            # Frame skipping for detection (configurable)
            self.processed_frame_count += 1
            should_process_detection = (self.processed_frame_count % self.process_every_n_frames == 0)

            if should_process_detection:
                # Detect face
                human_detected, human_x, human_y, face_size = self.detect_human(cv_image, display_image)

                if human_detected:
                    # Update position with smoothing
                    self.position_history.append((human_x, human_y, face_size))

                    # Calculate smoothed position
                    if len(self.position_history) >= 3:
                        avg_x = np.mean([p[0] for p in self.position_history])
                        avg_y = np.mean([p[1] for p in self.position_history])
                        avg_size = np.mean([p[2] for p in self.position_history])
                        self.current_human_pos = (avg_x, avg_y, avg_size)
                    else:
                        self.current_human_pos = (human_x, human_y, face_size)
                else:
                    # No human detected - clear position data to prevent movement
                    self.current_human_pos = None
                    self.position_history.clear()  # Clear history to prevent stale data

                # Detect gestures (if enabled)
                if self.gesture_mode:
                    gesture = self.detect_gesture(cv_image, display_image)
            
            # Always draw overlay based on current state (even if detection was skipped this frame)
            if self.current_human_pos:
                human_x, human_y, face_size = self.current_human_pos
                
                # Draw target center
                cv2.circle(display_image,
                          (int(self.image_center_x), int(self.image_center_y)),
                          10, (0, 255, 0), 2)
                cv2.line(display_image,
                        (int(self.image_center_x) - 20, int(self.image_center_y)),
                        (int(self.image_center_x) + 20, int(self.image_center_y)),
                        (0, 255, 0), 2)
                cv2.line(display_image,
                        (int(self.image_center_x), int(self.image_center_y) - 20),
                        (int(self.image_center_x), int(self.image_center_y) + 20),
                        (0, 255, 0), 2)

                # Calculate offset from center
                offset_x = human_x - self.image_center_x
                offset_y = human_y - self.image_center_y

                # Display tracking info
                cv2.putText(display_image, f"Offset: ({offset_x:.0f}, {offset_y:.0f})",
                          (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display_image, f"Face size: {face_size:.0f}px (target: {self.target_face_size:.0f})",
                          (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # Show auto-distance status
                auto_status = "AUTO" if self.auto_distance_adjust else "MANUAL"
                status_color = (0, 255, 255) if self.auto_distance_adjust else (255, 255, 0)
                cv2.putText(display_image, f"Distance: {auto_status}",
                          (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            else:
                cv2.putText(display_image, "NO HUMAN DETECTED",
                          (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

            if self.gesture_mode and self.last_gesture and (time.time() - self.last_gesture_time < self.gesture_cooldown):
                 cv2.putText(display_image, f"Gesture: {self.last_gesture}",
                              (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

            # Calculate and display FPS
            self.frame_count += 1
            if self.frame_count >= 30:  # Update FPS every 30 frames
                current_time = time.time()
                elapsed = current_time - self.fps_start_time
                self.current_fps = self.frame_count / elapsed if elapsed > 0 else 0
                self.frame_count = 0
                self.fps_start_time = current_time

            # Draw status panel (bottom of image)
            h, w = display_image.shape[:2]
            panel_height = 120
            cv2.rectangle(display_image, (0, h - panel_height), (w, h), (0, 0, 0), -1)
            cv2.rectangle(display_image, (0, h - panel_height), (w, h), (255, 255, 255), 2)

            # Display system information
            y_pos = h - 90
            cv2.putText(display_image, f"FPS: {self.current_fps:.1f}",
                       (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(display_image, f"Robot Pos: ({self.current_x:.0f}, {self.current_y:.0f}, {self.current_z:.0f})",
                       (200, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            y_pos += 30
            mode_text = "TRACKING" if self.current_human_pos else "WAITING"
            mode_color = (0, 255, 0) if self.current_human_pos else (0, 165, 255)
            cv2.putText(display_image, f"Mode: {mode_text}",
                       (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, mode_color, 2)

            distance_status = f"Target X: {self.target_distance:.0f}mm"
            cv2.putText(display_image, distance_status,
                       (200, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            y_pos += 30
            cv2.putText(display_image, "Press 'q' to quit | 'r' to reset | 's' to save frame",
                       (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            # Store the display image for window update timer with thread-safe lock
            # Use debug-level logs here to avoid spamming the logger at INFO for every frame
            self.get_logger().debug('Storing display image for window update')
            with self.image_lock:
                self.latest_display_image = display_image.copy()
                self.latest_frame_timestamp = time.time()
            self.get_logger().debug(f'Image stored. Timestamp: {self.latest_frame_timestamp}')

        except Exception as e:
            self.get_logger().error(f'❌ ERROR in image callback: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def update_window(self):
        """
        Dedicated timer callback to update the OpenCV window.
        Runs at 30 Hz to keep window responsive and handle keyboard input.
        """
        if not self.window_created:
            return
        # Ensure display_image is always defined in this scope so exceptions
        # won't produce UnboundLocalError when accessing the variable in logs.
        display_image = None
        try:
            # Get latest image with thread-safe lock
            with self.image_lock:
                if self.latest_display_image is None:
                    # Still waiting for first image
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        self.get_logger().info('User pressed "q" - shutting down...')
                        rclpy.shutdown()
                    return

                # Check if we have a new frame to display
                if self.latest_frame_timestamp <= self.last_displayed_timestamp:
                    # No new frame, just handle keyboard input
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        self.get_logger().info('User pressed "q" - shutting down...')
                        rclpy.shutdown()
                    return

                # Make a copy for display
                display_image = self.latest_display_image.copy()
                frame_age = time.time() - self.latest_frame_timestamp
                self.last_displayed_timestamp = self.latest_frame_timestamp

            # Add frame freshness indicator (top-right corner)
            if display_image is None:
                # Nothing to do (shouldn't happen because we returned earlier), but guard anyway
                return

            h, w = display_image.shape[:2]
            freshness_text = f"Frame age: {frame_age:.2f}s"
            freshness_color = (0, 255, 0) if frame_age < 1.0 else (0, 165, 255) if frame_age < 3.0 else (0, 0, 255)
            cv2.putText(display_image, freshness_text,
                       (w - 250, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, freshness_color, 2)

            # Display the image
            cv2.imshow('Vlogger View', display_image)

            # Handle keyboard input (must be called regularly to keep window alive)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                self.get_logger().info('User pressed "q" - shutting down...')
                rclpy.shutdown()
            elif key == ord('s'):
                filename = f"vlogger_frame_{int(time.time())}.jpg"
                cv2.imwrite(filename, display_image)
                self.get_logger().info(f'Frame saved: {filename}')
            elif key == ord('r'):
                self.get_logger().info('Resetting to initial position...')
                self.position_history.clear()
                self.current_human_pos = None

        except Exception as e:
            # Log the error and disable window updates to prevent repeated exceptions.
            import traceback
            self.get_logger().error(f'Error updating window: {str(e)}')
            self.get_logger().error(traceback.format_exc())
            self.window_created = False  # Disable window updates on error

    def trigger_camera_capture(self):
        """
        Trigger the TM5-900 camera to capture a frame.
        The camera requires Vision_DoJob command to capture each image.
        Called from control_loop when ready for next frame (~3 second intervals).
        Returns: True if trigger was sent, False if skipped
        """
        try:
            # Prevent re-entrant calls (can happen if timer fires while we're waiting)
            if hasattr(self, '_capture_in_progress') and self._capture_in_progress:
                self.get_logger().debug('Capture already in progress, skipping...')
                return False
            
            self._capture_in_progress = True
            
            if not self.arm_client.service_is_ready():
                if not hasattr(self, '_service_warning_shown'):
                    self.get_logger().warning('⚠️  send_script service not ready')
                    self._service_warning_shown = True
                self._capture_in_progress = False
                return False

            # Count triggers for diagnostics (only log sparingly)
            self.camera_trigger_count += 1
            if self.camera_trigger_count <= 2 or self.camera_trigger_count % 20 == 0:
                self.get_logger().info(f'📷 Capturing frame #{self.camera_trigger_count}...')

            # Send Vision_DoJob command to trigger camera capture
            move_cmd = SendScript.Request()
            move_cmd.script = "Vision_DoJob(job1)"

            # Call service asynchronously (non-blocking)
            # Don't wait here - let the callback handle completion
            future = self.arm_client.call_async(move_cmd)
            future.add_done_callback(self._vision_job_done_callback)
            return True

        except Exception as e:
            self._capture_in_progress = False
            # Log error only once to avoid spam
            if not hasattr(self, '_camera_error_logged'):
                self.get_logger().error(f'Error triggering camera: {str(e)}')
                self._camera_error_logged = True
            return False
    
    def _vision_job_done_callback(self, future):
        """Callback when Vision_DoJob completes"""
        self._capture_in_progress = False
        try:
            result = future.result()
            self.get_logger().debug(f'Vision_DoJob completed: {result}')
        except Exception as e:
            self.get_logger().error(f'Vision_DoJob failed: {e}')

    # def detect_human(self, image, display_image):
    #     """
    #     Detect human face in the image.
    #     Returns: (detected, center_x, center_y, face_size)
    #     """
    #     # Convert to grayscale for face detection
    #     gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    #     # Detect faces
    #     faces = self.face_cascade.detectMultiScale(
    #         gray,
    #         scaleFactor=1.1,
    #         minNeighbors=5,
    #         minSize=(100, 100)
    #     )

    #     if len(faces) > 0:
    #         # Use the largest face (closest person)
    #         largest_face = max(faces, key=lambda f: f[2] * f[3])
    #         x, y, w, h = largest_face

    #         # Calculate center
    #         center_x = x + w / 2
    #         center_y = y + h / 2

    #         # Face size (width as proxy for distance)
    #         face_size = w

    #         # Draw rectangle and center point
    #         cv2.rectangle(display_image, (x, y), (x + w, y + h), (255, 0, 0), 3)
    #         cv2.circle(display_image, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)

    #         # Draw line from face to center
    #         cv2.line(display_image,
    #                 (int(center_x), int(center_y)),
    #                 (int(self.image_center_x), int(self.image_center_y)),
    #                 (255, 255, 0), 2)

    #         return True, center_x, center_y, face_size

    #     return False, 0, 0, 0
    def detect_human(self, image, display_image):
        """
        Detect human face using MediaPipe Face Mesh.
        Returns: (detected, center_x, center_y, face_size_width)
        """
        if not self.gesture_mode:
             # If MediaPipe is not available, skip detection, and uncomment the line below to use Haar Cascade instead
             # return self._detect_human_haar(image, display_image)
             return False, 0, 0, 0
             
        # Convert to RGB format (required by MediaPipe)
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Process image
        results = self.face_mesh.process(rgb_image)

        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                
                # 1. Extract pixel coordinates of all landmarks
                h, w, c = image.shape
                x_coords = []
                y_coords = []

                for landmark in face_landmarks.landmark:
                    x_coords.append(int(landmark.x * w))
                    y_coords.append(int(landmark.y * h))

                # 2. Calculate bounding box
                x_min, x_max = min(x_coords), max(x_coords)
                y_min, y_max = min(y_coords), max(y_coords)
                
                # 3. Calculate center and size
                face_size_width = x_max - x_min
                center_x = x_min + face_size_width / 2
                center_y = y_min + (y_max - y_min) / 2 # Use height to calculate Y center

                # 4. Drawing the bounding box and center
                cv2.rectangle(display_image, (x_min, y_min), (x_max, y_max), (255, 0, 0), 3)
                cv2.circle(display_image, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)

                # Draw Face Mesh landmarks (optional, expensive). Only draw when enabled.
                if getattr(self, 'draw_face_mesh', False):
                    self.mp_drawing.draw_landmarks(
                        display_image,
                        face_landmarks,
                        self.mp_face_mesh.FACEMESH_CONTOURS,
                        self.mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=1, circle_radius=1),
                        self.mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=1, circle_radius=1))
                
                return True, center_x, center_y, face_size_width

        return False, 0, 0, 0

    def detect_gesture(self, image, display_image):
        """
        Detect hand gestures using MediaPipe.
        Returns: "CLOSER" (1 finger), "BACKUP" (5 fingers), or None
        """
        if not self.gesture_mode:
            return None

        # Check cooldown - don't detect new gestures during cooldown
        current_time = time.time()
        if current_time - self.last_gesture_time < self.gesture_cooldown:
            return None  # FIX: Return None instead of old gesture during cooldown

        # Convert to RGB for MediaPipe
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_image)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw hand landmarks
                self.mp_drawing.draw_landmarks(
                    display_image,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS
                )

                # Count extended fingers
                fingers_up = self.count_fingers(hand_landmarks)

                gesture = None
                if fingers_up == 1:
                    gesture = "CLOSER"
                    self.last_gesture = gesture
                    self.last_gesture_time = current_time
                    self.get_logger().info('👆 Gesture detected: MOVE CLOSER')
                elif fingers_up == 5:
                    gesture = "BACKUP"
                    self.last_gesture = gesture
                    self.last_gesture_time = current_time
                    self.get_logger().info('🖐️ Gesture detected: BACK UP')

                return gesture

        return None

    def count_fingers(self, hand_landmarks):
        """
        Count number of extended fingers.
        Returns: 0-5
        """
        # Finger tip and PIP joint indices
        finger_tips = [
            self.mp_hands.HandLandmark.THUMB_TIP,
            self.mp_hands.HandLandmark.INDEX_FINGER_TIP,
            self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
            self.mp_hands.HandLandmark.RING_FINGER_TIP,
            self.mp_hands.HandLandmark.PINKY_TIP
        ]

        finger_pips = [
            self.mp_hands.HandLandmark.THUMB_IP,
            self.mp_hands.HandLandmark.INDEX_FINGER_PIP,
            self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP,
            self.mp_hands.HandLandmark.RING_FINGER_PIP,
            self.mp_hands.HandLandmark.PINKY_PIP
        ]

        fingers_up = 0

        for tip, pip in zip(finger_tips, finger_pips):
            tip_y = hand_landmarks.landmark[tip].y
            pip_y = hand_landmarks.landmark[pip].y

            # Finger is up if tip is above PIP joint
            if tip_y < pip_y:
                fingers_up += 1

        return fingers_up

    def control_loop(self):
        """
        Main control loop - runs at 5 Hz.
        Calculates and executes robot movements to track human.
        Also triggers camera captures when ready.

        SAFETY: Robot will NOT move if no human face is detected in frame.
        """
        # Trigger camera capture if using techman_image (requires Vision_DoJob)
        # For other cameras (USB/ROS), images arrive automatically
        if self.camera_topic == 'techman_image':
            # Diagnostic: Check if anyone is publishing
            pubs = self.count_publishers(self.camera_topic)
            if pubs == 0 and self.frame_count % 50 == 0: # Log every ~10s
                self.get_logger().warning(f'⚠️ NO PUBLISHERS detected on {self.camera_topic}!')
            
            current_time = time.time()
            
            # Timeout logic: if waiting too long (> 8s), reset and retry
            # Increased timeout to 8s to allow for slow network/robot processing
            if self.waiting_for_image and (current_time - self.last_trigger_time) > 8.0:
                self.get_logger().warning(f'Timeout waiting for image (last trigger: {current_time - self.last_trigger_time:.1f}s ago). Retrying...')
                self.waiting_for_image = False
                self._capture_in_progress = False  # Force reset capture flag
            
            if not self.waiting_for_image and (current_time - self.last_trigger_time) > 0.1:
                if self.trigger_camera_capture():
                    self.waiting_for_image = True
                    self.last_trigger_time = current_time

        # CRITICAL SAFETY CHECK: Do not move if no human is detected
        if self.current_human_pos is None:
            # Robot stays stationary when no face is in frame
            return

        human_x, human_y, face_size = self.current_human_pos

        # Additional safety check: validate position data
        if face_size <= 0 or not (0 <= human_x <= 3000) or not (0 <= human_y <= 3000):
            self.get_logger().warning('Invalid human position data detected, skipping movement')
            return

        # Calculate offset from center
        offset_x = human_x - self.image_center_x
        offset_y = human_y - self.image_center_y
        distance_from_center = math.sqrt(offset_x**2 + offset_y**2)

        # Check if centering is needed
        needs_centering = distance_from_center > self.centering_threshold

        # Check for gesture commands
        gesture_command = self.process_gesture_command()

        # Only move if needed (either centering or gesture command)
        if needs_centering or gesture_command:
            # Calculate new robot position
            new_x, new_y, new_z = self.calculate_new_position(
                offset_x, offset_y, face_size, gesture_command
            )

            # Check if movement is significant enough
            move_distance = math.sqrt(
                (new_x - self.current_x)**2 +
                (new_y - self.current_y)**2 +
                (new_z - self.current_z)**2
            )

            if move_distance > self.min_movement:
                # Rate limit movements (max 2 Hz)
                current_time = time.time()
                # Allow faster reaction: reduce move rate limit to 0.2s (5 Hz)
                if current_time - self.last_move_time > 0.2:
                    self.move_robot(new_x, new_y, new_z)
                    self.last_move_time = current_time

    def process_gesture_command(self):
        """
        Process gesture commands and return distance adjustment.
        Returns: "closer", "backup", or None

        FIX: Gesture commands are active for cooldown period after detection.
        This allows the robot to execute the gesture command even though
        detect_gesture() won't detect new gestures during cooldown.
        """
        if not self.gesture_mode or not self.last_gesture:
            return None

        current_time = time.time()
        # Keep gesture active during cooldown period
        if current_time - self.last_gesture_time < self.gesture_cooldown:
            return self.last_gesture.lower()

        return None

    def calculate_new_position(self, offset_x, offset_y, face_size, gesture_command):
        """
        Calculate new robot position based on human position and gestures.
        
        MAPPING (Standard Robot Frame):
        - Image X (Horizontal) -> Robot Y (Left/Right)
        - Image Y (Vertical)   -> Robot Z (Up/Down)
        - Face Size (Distance) -> Robot X (Forward/Back)
        """
        # Start with current position
        new_x = self.current_x
        new_y = self.current_y
        new_z = self.current_z

        # Convert pixel offset to robot movement
        scale = 0.3  # Scaling factor

        # 1. Horizontal Tracking (Image X -> Robot X, Y)
        # Human Right (Offset X > 0) -> Arm moves (-1, -1) direction
        # Human Left (Offset X < 0) -> Arm moves (1, 1) direction
        move_x = -1.0 * offset_x * scale
        move_y = -1.0 * offset_x * scale
        
        # 2. Vertical Tracking (Image Y -> Robot Z)
        # If Face is Down (Offset Y > 0), Robot must move Down (-Z)
        move_z = -1.0 * offset_y * scale
        
        # Clamp movement to max step size
        max_step = 50.0

        move_x = max(-max_step, min(max_step, move_x))
        move_y = max(-max_step, min(max_step, move_y))
        move_z = max(-max_step, min(max_step, move_z))
        
        new_x += move_x
        new_y += move_y
        new_z += move_z

        # 3. Distance Tracking (Face Size -> Robot X)
        # Handle gesture commands for distance (X-axis)
        if gesture_command == "closer":
            new_x += 100.0  # Move Forward (+X)
            new_y -= 100.0  # Move Right (-Y)
            self.get_logger().info(f'Gesture: Moving closer {self.current_x:.1f} → {new_x:.1f}mm')
            self.auto_distance_adjust = False
        elif gesture_command == "backup":
            new_x -= 100.0  # Move Back (-X)
            new_y += 100.0  # Move Left (+Y)
            self.get_logger().info(f'Gesture: Backing up {self.current_x:.1f} → {new_x:.1f}mm')
            self.auto_distance_adjust = False
        else:
            # Re-enable auto-adjustment after 5 seconds
            if not self.last_gesture or (time.time() - self.last_gesture_time > 5.0):
                self.auto_distance_adjust = True

            # AUTOMATIC DISTANCE ADJUSTMENT
            if self.auto_distance_adjust and face_size > 0:
                face_size_diff = face_size - self.target_face_size

                if abs(face_size_diff) > self.face_size_tolerance:
                    # If face too big (diff > 0), move Back (-1, 1) direction
                    # If face too small (diff < 0), move Forward (1, -1) direction
                    # Increase gain so face-size changes produce noticeable robot motion.
                    # face_size_diff is in pixels; multiplier maps it to mm adjustment.
                    adjustment = face_size_diff * 1.0

                    max_adjustment = 80.0
                    adjustment = max(-max_adjustment, min(max_adjustment, adjustment))

                    new_x -= adjustment  # Subtract adjustment to move in correct direction
                    new_y += adjustment  # Add adjustment to move in (1, -1) direction

                    if abs(adjustment) > 5.0:
                        self.get_logger().info(
                            f'Auto-distance: face {face_size:.0f}px, adjusting X by {-adjustment:+.1f}mm, Y by {adjustment:+.1f}mm'
                        )

        # Enforce workspace limits
        # X (Depth): 100mm to 600mm
        new_x = max(100.0, min(600.0, new_x))
        
        # Y (Side): 0mm to 600mm
        new_y = max(0.0, min(600.0, new_y))
        
        # Z (Height): 200mm to 700mm
        new_z = max(200.0, min(700.0, new_z))

        return new_x, new_y, new_z

    def move_robot(self, x, y, z):
        """
        Send movement command to robot with improved error handling.
        """
        try:
            # Keep orientation pointing forward (camera toward human)
            rx = self.current_rx
            ry = self.current_ry
            rz = self.current_rz

            # Create movement script
            script = f'PTP("CPP",{x:.2f},{y:.2f},{z:.2f},{rx:.2f},{ry:.2f},{rz:.2f},80,150,0,false)'

            self.get_logger().info(f'Moving: ({self.current_x:.1f}, {self.current_y:.1f}, {self.current_z:.1f}) → ({x:.1f}, {y:.1f}, {z:.1f})')

            # Send command using persistent client (FIX: no longer creates new node)
            success = self.send_script(script)

            if success:
                # Update current position only if command succeeded
                self.current_x = x
                self.current_y = y
                self.current_z = z
            else:
                self.get_logger().warning('Movement command failed, position not updated')

        except Exception as e:
            self.get_logger().error(f'Error moving robot: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def send_script(self, script):
        """
        Send movement command to robot using persistent service client.
        Returns: True if successful (request sent), False otherwise

        FIX: Non-blocking call to avoid freezing the control loop.
        """
        try:
            if not self.arm_client.service_is_ready():
                self.get_logger().warning('send_script service not ready')
                return False

            move_cmd = SendScript.Request()
            move_cmd.script = script

            # Call service asynchronously (fire and forget)
            future = self.arm_client.call_async(move_cmd)
            future.add_done_callback(self._send_script_done)
            
            return True

        except Exception as e:
            self.get_logger().error(f'Error in send_script: {str(e)}')
            return False

    def _send_script_done(self, future):
        """Callback for send_script service result"""
        try:
            response = future.result()
            if not (response and hasattr(response, 'ok') and response.ok):
                self.get_logger().warning(f'send_script returned not ok: {response}')
        except Exception as e:
            self.get_logger().error(f'Error getting service response: {str(e)}')


def send_script(script):
    """
    Send movement command to robot (standalone function for initial setup).
    This is only used in main() for initial positioning before the controller is created.

    NOTE: This still creates a temporary node, but it's only called once during initialization,
    not repeatedly during runtime like before (which was the bug).
    """
    arm_node = rclpy.create_node('arm_initial_position')
    arm_cli = arm_node.create_client(SendScript, 'send_script')

    timeout_count = 0
    while not arm_cli.wait_for_service(timeout_sec=1.0):
        arm_node.get_logger().info('send_script service not available, waiting...')
        timeout_count += 1
        if timeout_count > 10:
            arm_node.get_logger().error('send_script service timeout after 10 seconds')
            arm_node.destroy_node()
            return False

    move_cmd = SendScript.Request()
    move_cmd.script = script
    future = arm_cli.call_async(move_cmd)
    rclpy.spin_until_future_complete(arm_node, future)

    try:
        future.result()  # Wait for completion
        arm_node.destroy_node()
        return True
    except Exception as e:
        arm_node.get_logger().error(f'Error in send_script: {str(e)}')
        arm_node.destroy_node()
        return False


def main(args=None):
    rclpy.init(args=args)

    controller = VloggerController()

    logger = controller.get_logger()
    logger.info('')
    logger.info('==============================================')
    logger.info('  STARTING VLOGGER MODE')
    logger.info('==============================================')
    logger.info('Instructions:')
    logger.info('  - Stay in view of the camera')
    logger.info('  - Robot will keep you centered')
    logger.info('  - Show 1 finger: Move closer')
    logger.info('  - Show 5 fingers: Back up')
    logger.info('==============================================')
    logger.info('')

    # Move to initial vlogger position
    logger.info('Moving to initial position...')
    initial_position = "300.00, 300.00, 400.00, 90.00, 0.0, 50.0"
    script = "PTP(\"CPP\"," + initial_position + ",100,200,0,false)"
    send_script(script)
    time.sleep(2.0)

    logger.info('Ready! Starting tracking...')

    # Trigger first camera capture (only if using Techman vision)
    if controller.camera_topic == 'techman_image':
        logger.info('Using Vision_DoJob(job1) for camera capture. Waiting for control loop to trigger first frame...')
        # Do NOT trigger here manually - let the control loop handle it to ensure proper timing and state management
        # controller.trigger_camera_capture()
        # controller.waiting_for_image = True
    else:
        logger.info(f'Using live camera feed from {controller.camera_topic}')

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        logger.info('Shutting down...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
