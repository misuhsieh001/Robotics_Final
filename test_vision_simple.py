#!/usr/bin/env python3
"""
Simple test to check if Vision_DoJob(job1) works and publishes images.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tm_msgs.srv import SendScript
import sys

# Add path for tm_msgs
python_version = f"python{sys.version_info.major}.{sys.version_info.minor}"
tm_msgs_path = f'/home/robot/colcon_ws/install/tm_msgs/lib/{python_version}/site-packages'
sys.path.append(tm_msgs_path)

class VisionTester(Node):
    def __init__(self):
        super().__init__('vision_tester')

        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            'techman_image',
            self.image_callback,
            10)

        # Create service client
        self.arm_client = self.create_client(SendScript, 'send_script')

        self.image_received = False
        self.get_logger().info('Vision Tester initialized. Waiting for send_script service...')

        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for send_script service...')

    def image_callback(self, data):
        self.get_logger().info(f'✅✅✅ IMAGE RECEIVED! Size: {data.width}x{data.height} ✅✅✅')
        self.image_received = True

    def trigger_vision(self):
        self.get_logger().info('Sending Vision_DoJob(job1)...')

        cmd = SendScript.Request()
        cmd.script = "Vision_DoJob(job1)"

        future = self.arm_client.call_async(cmd)

        # Wait for completion
        import time
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > 15.0:
                self.get_logger().error('❌ Vision_DoJob TIMED OUT after 15 seconds')
                return False
            rclpy.spin_once(self, timeout_sec=0.1)

        try:
            result = future.result()
            self.get_logger().info(f'Vision_DoJob completed. Result: {result}')
            return True
        except Exception as e:
            self.get_logger().error(f'Vision_DoJob error: {e}')
            return False

def main():
    rclpy.init()

    tester = VisionTester()

    print('\n' + '='*70)
    print('VISION SYSTEM TEST')
    print('='*70)
    print('This will test if Vision_DoJob(job1) works and publishes images.')
    print('='*70 + '\n')

    # Test 1: Trigger vision job
    print('TEST 1: Triggering Vision_DoJob(job1)...')
    success = tester.trigger_vision()

    if not success:
        print('\n❌ FAILED: Vision_DoJob did not complete successfully')
        print('   Possible causes:')
        print('   1. Vision job "job1" is not configured in TMflow')
        print('   2. Camera is not connected or not working')
        print('   3. Robot is not in the correct mode')
        rclpy.shutdown()
        return

    print('✅ Vision_DoJob completed')

    # Test 2: Wait for image
    print('\nTEST 2: Waiting for image on /techman_image topic...')
    print('Waiting up to 10 seconds...')

    import time
    start_time = time.time()
    while not tester.image_received and (time.time() - start_time) < 10.0:
        rclpy.spin_once(tester, timeout_sec=0.1)

    if tester.image_received:
        print('\n✅✅✅ SUCCESS! Camera is working and publishing images! ✅✅✅')
    else:
        print('\n❌ FAILED: No image received after Vision_DoJob')
        print('   Possible causes:')
        print('   1. Vision job "job1" is not configured to publish to ROS')
        print('   2. Vision job output is not connected to /techman_image topic')
        print('   3. Camera captured but image was not published')

    print('\n' + '='*70)
    print('TEST COMPLETE')
    print('='*70)

    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
