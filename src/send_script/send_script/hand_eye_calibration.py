#!/usr/bin/env python3
"""
Hand-Eye Calibration Tool for TM5-900

This tool helps you calibrate the transformation from pixel coordinates to robot coordinates.

Usage:
1. Place a marker/cube at multiple known robot positions
2. For each position, record the robot X,Y and pixel u,v coordinates
3. Run this script to calculate the transformation matrix
"""

import numpy as np
import json

print("="*70)
print("  TM5-900 HAND-EYE CALIBRATION TOOL")
print("="*70)
print("\nThis tool will help you calibrate pixel-to-robot transformation.")
print("You need at least 4 calibration points (more is better).\n")
print("For each point:")
print("  1. Move robot to a known (X, Y) position")
print("  2. Observe the marker/cube pixel position (u, v) in the image")
print("  3. Enter the coordinates when prompted")
print("\n" + "="*70 + "\n")

# Collect calibration data
calibration_points = []

print("How many calibration points do you want to enter? (minimum 4): ", end='')
num_points = int(input())

if num_points < 4:
    print("ERROR: Need at least 4 points for accurate calibration!")
    exit(1)

for i in range(num_points):
    print(f"\n--- Calibration Point {i+1}/{num_points} ---")

    print("  Robot X coordinate (mm): ", end='')
    robot_x = float(input())

    print("  Robot Y coordinate (mm): ", end='')
    robot_y = float(input())

    print("  Image pixel U coordinate (horizontal): ", end='')
    pixel_u = float(input())

    print("  Image pixel V coordinate (vertical): ", end='')
    pixel_v = float(input())

    calibration_points.append({
        'robot': (robot_x, robot_y),
        'pixel': (pixel_u, pixel_v)
    })

    print(f"  ✓ Recorded: Robot ({robot_x}, {robot_y}) ← Pixel ({pixel_u}, {pixel_v})")

print("\n" + "="*70)
print("  CALCULATING TRANSFORMATION...")
print("="*70 + "\n")

# Extract data for solving
pixel_coords = np.array([p['pixel'] for p in calibration_points])  # [u, v]
robot_coords = np.array([p['robot'] for p in calibration_points])  # [X, Y]

# We want to solve: [X, Y] = A * [u, v] + b
# Or in matrix form: [X, Y] = [a11, a12] * [u] + [b1]
#                              [a21, a22]   [v]   [b2]
#
# This can be rewritten as:
# X = a11*u + a12*v + b1
# Y = a21*u + a22*v + b2

# Build the system: [u, v, 1] * [a11, a12] = X
#                                [a21, a22]
#                                [b1,  b2 ]

# Add column of ones for the constant term
ones = np.ones((len(calibration_points), 1))
A_matrix = np.hstack([pixel_coords, ones])  # [u, v, 1]

# Solve for X coordinates
transform_x = np.linalg.lstsq(A_matrix, robot_coords[:, 0], rcond=None)[0]
# transform_x = [a11, a12, b1]

# Solve for Y coordinates
transform_y = np.linalg.lstsq(A_matrix, robot_coords[:, 1], rcond=None)[0]
# transform_y = [a21, a22, b2]

print("TRANSFORMATION MATRIX:")
print(f"  X_robot = {transform_x[0]:.6f} * u + {transform_x[1]:.6f} * v + {transform_x[2]:.2f}")
print(f"  Y_robot = {transform_y[0]:.6f} * u + {transform_y[1]:.6f} * v + {transform_y[2]:.2f}")
print()

# Verify the transformation
print("VERIFICATION (Reprojection Check):")
print("-" * 70)
print(f"{'Point':<8} {'Robot X,Y':<20} {'Pixel U,V':<20} {'Predicted X,Y':<20} {'Error (mm)':<15}")
print("-" * 70)

total_error = 0
for i, point in enumerate(calibration_points):
    u, v = point['pixel']
    true_x, true_y = point['robot']

    pred_x = transform_x[0] * u + transform_x[1] * v + transform_x[2]
    pred_y = transform_y[0] * u + transform_y[1] * v + transform_y[2]

    error = np.sqrt((pred_x - true_x)**2 + (pred_y - true_y)**2)
    total_error += error

    print(f"{i+1:<8} ({true_x:.1f}, {true_y:.1f})  ({u:.1f}, {v:.1f})  ({pred_x:.1f}, {pred_y:.1f})  {error:.2f}")

mean_error = total_error / len(calibration_points)
print("-" * 70)
print(f"Mean Reprojection Error: {mean_error:.2f} mm")

if mean_error > 10:
    print("⚠️  WARNING: High error! Check your calibration points.")
else:
    print("✓ Good calibration!")

print("\n" + "="*70)
print("  COPY THIS CODE INTO YOUR cube_stacker_logic.py")
print("="*70)
print()
print("Replace the image_to_robot_coords function with:")
print()
print("def image_to_robot_coords(self, u, v):")
print('    """Convert pixel coordinates to robot coordinates (hand-eye calibrated)"""')
print(f"    X_robot = {transform_x[0]:.6f} * u + {transform_x[1]:.6f} * v + {transform_x[2]:.2f}")
print(f"    Y_robot = {transform_y[0]:.6f} * u + {transform_y[1]:.6f} * v + {transform_y[2]:.2f}")
print("    Z_robot = 300.0  # Gripping position at 300mm height")
print("    return X_robot, Y_robot, Z_robot")
print()

# Save to file
output_file = '/home/robot/workspace2/team11_ws/hand_eye_calibration.json'
calibration_data = {
    'calibration_points': calibration_points,
    'transform_x': transform_x.tolist(),
    'transform_y': transform_y.tolist(),
    'mean_error': float(mean_error)
}

with open(output_file, 'w') as f:
    json.dump(calibration_data, f, indent=2)

print(f"✓ Calibration data saved to: {output_file}")
print("\n" + "="*70)
print("  CALIBRATION COMPLETE!")
print("="*70)
