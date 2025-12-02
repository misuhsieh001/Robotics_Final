#!/bin/bash
# Vlogger System Launch Script
# This script properly sources the ROS2 workspace and runs the vlogger

echo "======================================================================"
echo "  TM5-900 VLOGGER SYSTEM - Launch Script"
echo "======================================================================"
echo ""
echo "MediaPipe Status: ✅ ENABLED (with gesture recognition & face tracking)"
echo "Location: venv/lib/python3.12/site-packages/"
echo ""
echo "Starting vlogger control node..."
echo "----------------------------------------------------------------------"
echo ""

# Source the workspace
cd /home/robot/workspace2/team11_ws_final_project/Robotics_Final_Project
source install/setup.bash

# Run the vlogger
ros2 run vlogger_system vlogger_control
