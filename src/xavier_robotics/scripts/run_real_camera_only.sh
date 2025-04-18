#!/bin/bash

echo "******************************************"
echo "* REAL CAMERA ONLY - STARTING WITH DEBUG *"
echo "******************************************"

# Kill ALL ROS and visualization processes
echo "Stopping all ROS and visualization processes..."
pkill -9 -f "ros2|zed|slam|nav2|rviz|python3|local_costmap" 2>/dev/null || true
sleep 3  # Give processes time to fully terminate

# Create maps directory if it doesn't exist
MAPS_DIR=~/maps
mkdir -p $MAPS_DIR
echo "Maps will be saved to: $MAPS_DIR"

# Remove SOME ROS caches and temporary files (keeping logs structure)
echo "Cleaning ROS temporary files..."
rm -rf /tmp/costmap_* /tmp/launch_params_* 2>/dev/null || true
rm -rf /tmp/robot_description* 2>/dev/null || true

# Ensure ROS log directory exists with proper permissions
echo "Setting up ROS log directory structure..."
mkdir -p ~/.ros/log
chmod -R 755 ~/.ros

# Use a simpler approach to ROS2 logging
# The issue is with deep nested paths in ROS2's launch system
export ROS_LOG_DIR=/tmp/ros2_logs
mkdir -p ${ROS_LOG_DIR}
chmod -R 777 ${ROS_LOG_DIR}
rm -rf ${ROS_LOG_DIR}/*

# Create core log files and set permissive permissions
touch ${ROS_LOG_DIR}/launch.log
touch ${ROS_LOG_DIR}/rosout.log
chmod 666 ${ROS_LOG_DIR}/*.log

# Also disable debug logging to minimize issues
export RCUTILS_LOGGING_MIN_SEVERITY=INFO
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] {message}"

echo "Using simplified log directory: ${ROS_LOG_DIR}"

# Setup environment with thorough display configuration
echo "Setting up environment variables..."
export DISPLAY=:1
export $(dbus-launch)
echo "Display set to: $DISPLAY"
xhost +local:* || true

# Setup ROS environment
echo "Setting up ROS2 environment..."
source /opt/ros/humble/setup.bash
source $(pwd)/install/setup.bash

echo ""
echo "==============================================="
echo "LAUNCHING REAL CAMERA ONLY MODE - ZERO TEST DATA"
echo "This will use ONLY real data from the ZED camera"
echo "with NO test patterns or simulated obstacles"
echo "==============================================="
echo ""

# Debug: verify Python and ROS are working
echo "Verifying Python and ROS2 installation..."
python3 --version
ros2 --help >/dev/null && echo "ROS2 CLI is working" || echo "ERROR: ROS2 CLI not working!"

# Run the real-camera-only launch file with verbose output
echo "Starting real camera launch..."
ros2 launch xavier_robotics real_camera_only.launch.py --debug

echo ""
echo "=== Session Complete ==="
echo "Costmaps have been saved to: $MAPS_DIR"
echo "You can view them with: ristretto $MAPS_DIR/real_camera_costmap_*.pgm"
