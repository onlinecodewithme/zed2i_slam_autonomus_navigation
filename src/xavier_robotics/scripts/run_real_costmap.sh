#!/bin/bash

# Script to run the real costmap inspection with proper environment setup

# Ensure the script is executable
if [ ! -x "$0" ]; then
    echo "Making script executable..."
    chmod +x "$0"
fi

# Set up ROS2 environment
source /opt/ros/humble/setup.bash
source ~/xavier_robotics/install/setup.bash

# Kill any existing ROS2 processes that might interfere
pkill -f "ros2|zed_|slam_toolbox|nav2|rviz2" || true

# Wait a moment to ensure everything is terminated
sleep 2

# Make sure the maps directory exists
mkdir -p ~/maps

# Launch the real costmap inspection (using direct path to ensure it works)
ros2 launch $(dirname "$0")/../launch/real_costmap_inspection.launch.py

# Save map on exit
echo "Saving any generated maps to ~/maps/"
cp -f /tmp/costmap_*.pgm ~/maps/ 2>/dev/null || true

echo "Done!"
