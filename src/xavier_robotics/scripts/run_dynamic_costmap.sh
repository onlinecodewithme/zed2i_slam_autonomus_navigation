#!/bin/bash

# Script to run the dynamic costmap generator with proper environment setup for the ZED 2i camera

# Ensure the script is executable
if [ ! -x "$0" ]; then
    echo "Making script executable..."
    chmod +x "$0"
fi

# Set up ROS2 environment
source /opt/ros/humble/setup.bash
source ~/xavier_robotics/install/setup.bash

# Set DISPLAY environment variable for UI applications
export DISPLAY=:1

# Kill any existing costmap processes that might interfere
pkill -f "local_costmap_generator" || true
pkill -f "rviz2" || true

# Wait a moment to ensure everything is terminated
sleep 2

# Make sure the maps directory exists for saving costmaps
mkdir -p ~/maps

# Set executable permissions for the Python script
chmod +x ~/xavier_robotics/src/autonomous_nav/scripts/local_costmap_generator.py

# Ensure ZED driver node is running
if ! ros2 node list | grep -q "zed_node"; then
    echo "ZED node not found. Starting ZED driver..."
    # Start ZED node in the background
    ros2 launch zed_wrapper zed2i.launch.py &
    # Wait for ZED node to initialize
    sleep 5
fi

# Launch the dynamic costmap generator directly from source
echo "Launching dynamic costmap generator with RViz visualization..."
# Run directly from source instead of using installed files
ros2 launch $(dirname "$0")/../../autonomous_nav/launch/dynamic_costmap.launch.py

# Save map on exit if needed
echo "Saving any generated costmaps to ~/maps/"
cp -f /tmp/costmap_*.pgm ~/maps/ 2>/dev/null || true

echo "Done!"
