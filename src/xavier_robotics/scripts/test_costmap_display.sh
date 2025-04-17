#!/bin/bash

# Exit on any error
set -e

# Kill any existing ROS2 processes to ensure a clean start
echo "Cleaning up any existing ROS2 processes..."
pkill -f "ros2|zed_|slam_toolbox|nav2|rviz2" 2>/dev/null || true
sleep 2

# Create maps directory if it doesn't exist
MAPS_DIR=~/maps
mkdir -p $MAPS_DIR
echo "Maps will be saved to: $MAPS_DIR"

# Setup environment
echo "Setting up ROS2 environment..."
. /opt/ros/humble/setup.bash
. $(pwd)/install/setup.bash

# Set display variable for RViz
export DISPLAY=:1
export $(dbus-launch)
echo "Display set to: $DISPLAY"

echo "Launching costmap test with RViz..."
ros2 launch autonomous_nav map_publisher_test.launch.py

echo ""
echo "=== Test Complete ==="
echo "Costmaps have been saved to: $MAPS_DIR"
echo "You can view them with: ./src/xavier_robotics/scripts/check_costmaps.sh"
