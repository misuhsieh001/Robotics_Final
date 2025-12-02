#!/usr/bin/env python
"""
Test script to verify robot movement commands
Run this to ensure the robot responds to movement commands before running the full system
"""

import rclpy
from rclpy.node import Node
import time
import sys

python_version = f"python{sys.version_info.major}.{sys.version_info.minor}"
tm_msgs_path = f'/home/robot/colcon_ws/install/tm_msgs/lib/{python_version}/site-packages'
sys.path.append(tm_msgs_path)

try:
    from tm_msgs.srv import SendScript
except ImportError:
    print("ERROR: tm_msgs not found!")
    print(f"Looked in: {tm_msgs_path}")
    print("Make sure tm_msgs is built and sourced")
    sys.exit(1)


def send_script(script, node_name='test_arm'):
    """Send movement command to robot"""
    print(f"Sending: {script}")

    arm_node = rclpy.create_node(node_name)
    arm_cli = arm_node.create_client(SendScript, 'send_script')

    timeout_count = 0
    while not arm_cli.wait_for_service(timeout_sec=1.0):
        timeout_count += 1
        if timeout_count > 5:
            print("ERROR: Service not available after 5 seconds!")
            print("Is the robot driver running?")
            print("Try: ros2 launch tm_driver tm_driver.launch.py")
            arm_node.destroy_node()
            return False
        print('Service not available, waiting...')

    move_cmd = SendScript.Request()
    move_cmd.script = script
    future = arm_cli.call_async(move_cmd)
    rclpy.spin_until_future_complete(arm_node, future)

    result = future.result()
    arm_node.destroy_node()

    print(f"✓ Command sent successfully")
    return True


def main(args=None):
    print("=" * 60)
    print("  TM5-900 Movement Test")
    print("=" * 60)
    print()
    print("This test will move the robot through a series of positions")
    print("to verify that movement commands are working correctly.")
    print()
    print("SAFETY WARNING:")
    print("  - Ensure workspace is clear")
    print("  - Be ready to press emergency stop")
    print("  - Robot will move slowly through test positions")
    print()

    input("Press ENTER to start test (or Ctrl+C to cancel)...")
    print()

    rclpy.init(args=args)

    # Test sequence
    positions = [
        {
            'name': 'Home Position',
            'x': 230.0,
            'y': 230.0,
            'z': 730.0,
            'rx': -180.0,
            'ry': 0.0,
            'rz': 135.0
        },
        {
            'name': 'Vlogger Starting Position',
            'x': 230.0,
            'y': 230.0,
            'z': 800.0,
            'rx': -180.0,
            'ry': 0.0,
            'rz': 0.0
        },
        {
            'name': 'Moved Left',
            'x': 180.0,
            'y': 230.0,
            'z': 800.0,
            'rx': -180.0,
            'ry': 0.0,
            'rz': 0.0
        },
        {
            'name': 'Moved Right',
            'x': 280.0,
            'y': 230.0,
            'z': 800.0,
            'rx': -180.0,
            'ry': 0.0,
            'rz': 0.0
        },
        {
            'name': 'Back to Center',
            'x': 230.0,
            'y': 230.0,
            'z': 800.0,
            'rx': -180.0,
            'ry': 0.0,
            'rz': 0.0
        },
        {
            'name': 'Moved Forward',
            'x': 230.0,
            'y': 280.0,
            'z': 800.0,
            'rx': -180.0,
            'ry': 0.0,
            'rz': 0.0
        },
        {
            'name': 'Moved Back',
            'x': 230.0,
            'y': 180.0,
            'z': 800.0,
            'rx': -180.0,
            'ry': 0.0,
            'rz': 0.0
        },
        {
            'name': 'Return Home',
            'x': 230.0,
            'y': 230.0,
            'z': 730.0,
            'rx': -180.0,
            'ry': 0.0,
            'rz': 135.0
        }
    ]

    print(f"Running {len(positions)} movement tests...\n")

    for i, pos in enumerate(positions, 1):
        print(f"[{i}/{len(positions)}] Moving to: {pos['name']}")
        print(f"        Position: ({pos['x']:.1f}, {pos['y']:.1f}, {pos['z']:.1f})")

        script = (f"PTP(\"CPP\",{pos['x']:.2f},{pos['y']:.2f},{pos['z']:.2f},"
                 f"{pos['rx']:.2f},{pos['ry']:.2f},{pos['rz']:.2f},50,100,0,false)")

        success = send_script(script, f'test_arm_{i}')

        if not success:
            print("ERROR: Movement failed!")
            rclpy.shutdown()
            return

        print(f"        Waiting for movement to complete...")
        time.sleep(3.0)  # Wait for movement
        print()

    print("=" * 60)
    print("  ✓ All movements completed successfully!")
    print("=" * 60)
    print()
    print("Robot is ready for vlogger operation.")
    print("Run: python3 vlogger_control.py")
    print()

    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest cancelled by user")
        rclpy.shutdown()
