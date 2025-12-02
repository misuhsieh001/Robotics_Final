#!/usr/bin/env python3
"""
Test and visualize the pixel-to-robot coordinate transformation
"""

# Hand-eye calibration parameters
a1 = 0.17739
b1 = -0.17893
c1 = 370.15
a2 = -0.17715
b2 = -0.18011
c2 = 824.72251
z_fixed = 300

def transform(u, v):
    """Transform pixel coordinates to robot coordinates"""
    x_robot = a1 * u + b1 * v + c1
    y_robot = a2 * u + b2 * v + c2
    z_robot = z_fixed
    return x_robot, y_robot, z_robot

print("="*70)
print("  TRANSFORMATION TEST TOOL")
print("="*70)
print()
print("This tool helps test the pixel-to-robot transformation.")
print()
print("Current transformation parameters:")
print(f"  a1 = {a1}")
print(f"  b1 = {b1}")
print(f"  c1 = {c1}")
print(f"  a2 = {a2}")
print(f"  b2 = {b2}")
print(f"  c2 = {c2}")
print()
print("Formulas:")
print(f"  X_robot = {a1} * u + {b1} * v + {c1}")
print(f"  Y_robot = {a2} * u + {b2} * v + {c2}")
print(f"  Z_robot = {z_fixed}")
print()
print("="*70)
print()

# Test some reference points
test_points = [
    ("Image center", 1332.58, 1013.72),
    ("Top-left", 800, 600),
    ("Top-right", 1800, 600),
    ("Bottom-left", 800, 1400),
    ("Bottom-right", 1800, 1400),
]

print("Reference point transformations:")
print("-"*70)
print(f"{'Description':<20} {'Pixel (u,v)':<20} {'Robot (X,Y,Z)'}")
print("-"*70)

for desc, u, v in test_points:
    x, y, z = transform(u, v)
    print(f"{desc:<20} ({u:.1f}, {v:.1f}){'':<8} ({x:.1f}, {y:.1f}, {z:.1f})")

print("-"*70)
print()

# Interactive testing
print("="*70)
print("  INTERACTIVE TESTING")
print("="*70)
print()
print("Enter pixel coordinates from your detected cubes to see robot coordinates.")
print("(Press Ctrl+C to exit)")
print()

try:
    while True:
        try:
            u = float(input("Enter pixel U (horizontal): "))
            v = float(input("Enter pixel V (vertical): "))

            x, y, z = transform(u, v)

            print(f"  → Robot coordinates: ({x:.2f}, {y:.2f}, {z:.2f})")
            print()

        except ValueError:
            print("Invalid input! Please enter numbers.")
            print()
except KeyboardInterrupt:
    print("\n\nExiting...")
