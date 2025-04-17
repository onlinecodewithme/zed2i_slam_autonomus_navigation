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

# Launch the inspection system with the right display setup
echo "Launching autonomous airplane inspection system..."
export DISPLAY=:1
export $(dbus-launch)
echo "Display set to: $DISPLAY"

# Run the main launch file
ros2 launch xavier_robotics autonomous_airplane_inspection.launch.py

echo ""
echo "If RViz didn't appear or you see 'Map not received', you can run:"
echo "DISPLAY=:1 ros2 run autonomous_nav map_publisher.py"
echo ""

# Save any temporary costmaps that might be in /tmp
echo "Saving any costmaps from /tmp to $MAPS_DIR..."
cp -f /tmp/costmap_*.pgm $MAPS_DIR/ 2>/dev/null || true

echo "=== Inspection Complete ==="
echo "Costmaps have been saved to: $MAPS_DIR"
echo "You can view them with: ristretto $MAPS_DIR/autonomous_inspection_costmap_*.pgm"
