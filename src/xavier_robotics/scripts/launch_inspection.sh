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

# Run the main launch file with RViz using the standard navigation config
echo "Launching autonomous airplane inspection system with RViz..."
ros2 launch xavier_robotics autonomous_airplane_inspection.launch.py

# If RViz doesn't appear or shows incorrect visualization, you can run this instead:
# echo "If you see purple test pattern in the costmap, run this command in a new terminal:"
# echo "ros2 run rviz2 rviz2 -d $(pwd)/src/autonomous_nav/rviz/real_costmap.rviz"

echo ""
echo "If RViz didn't appear, you may need to check display settings."
echo "NOTE: We no longer use map_publisher.py to prevent test data - only real ZED camera data is used."
echo ""

# Save any temporary costmaps that might be in /tmp
echo "Saving any costmaps from /tmp to $MAPS_DIR..."
cp -f /tmp/costmap_*.pgm $MAPS_DIR/ 2>/dev/null || true

echo "=== Inspection Complete ==="
echo "Costmaps have been saved to: $MAPS_DIR"
echo "You can view them with: ristretto $MAPS_DIR/autonomous_inspection_costmap_*.pgm"
