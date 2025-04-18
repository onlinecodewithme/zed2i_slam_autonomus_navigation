#!/bin/bash

# RUN STATIC MAP NAVIGATION
# This script launches the static map navigation system
# using a saved costmap as the static map for navigation

set -e  # Exit on error

# Print header
echo "**********************************"
echo "* STATIC MAP NAVIGATION LAUNCHER *"
echo "**********************************"

# Kill any existing ROS processes and clean up
echo "Stopping all ROS and visualization processes..."
pkill -f "ros2" || true
pkill -f "rviz" || true
sleep 1

# Set up maps directory
MAPS_DIR="$HOME/maps"
echo "Maps will be loaded from: $MAPS_DIR"

# Check if maps directory exists
if [ ! -d "$MAPS_DIR" ]; then
    echo "ERROR: Maps directory $MAPS_DIR does not exist."
    echo "Run the real_camera_only.launch.py first to generate maps."
    exit 1
fi

# Check if there are any map files
if [ ! "$(ls -A $MAPS_DIR/*.yaml 2>/dev/null)" ]; then
    echo "ERROR: No map files found in $MAPS_DIR."
    echo "Run the real_camera_only.launch.py first to generate maps."
    exit 1
fi

# Find the latest map file as default
LATEST_MAP=$(ls -t $MAPS_DIR/*.yaml | head -1)
LATEST_MAP_NAME=$(basename "$LATEST_MAP")

# List available maps
echo "Available maps:"
ls -lt $MAPS_DIR/*.yaml | awk '{print NR":", $9}'
echo ""

# Ask user to select a map
echo "Enter the number of the map to use (or press Enter for latest map: $LATEST_MAP_NAME):"
read MAP_NUMBER

# Process map selection
if [ -z "$MAP_NUMBER" ]; then
    # Use latest map
    SELECTED_MAP="$LATEST_MAP"
    echo "Using latest map: $LATEST_MAP_NAME"
else
    # Use selected map
    SELECTED_MAP=$(ls -t $MAPS_DIR/*.yaml | sed -n "${MAP_NUMBER}p")
    if [ -z "$SELECTED_MAP" ]; then
        echo "Invalid selection. Using latest map: $LATEST_MAP_NAME"
        SELECTED_MAP="$LATEST_MAP"
    else
        echo "Selected map: $(basename "$SELECTED_MAP")"
    fi
fi

# Cleaning temporary ROS2 files
echo "Cleaning ROS temporary files..."
rm -rf /tmp/ros2_* 2>/dev/null || true

# Setting up log directory
echo "Setting up ROS log directory structure..."
export ROS_LOG_DIR="/tmp/ros2_logs"
mkdir -p $ROS_LOG_DIR

# Set up environment variables
echo "Setting up environment variables..."
export DISPLAY=:1
export PYTHONUNBUFFERED=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1

# Source ROS2 and the local workspace
echo "Setting up ROS2 environment..."
source /opt/ros/humble/setup.bash
source ~/xavier_robotics/install/setup.bash

# Extra info
echo ""
echo "==============================================="
echo "LAUNCHING STATIC MAP NAVIGATION MODE"
echo "Using map: $(basename "$SELECTED_MAP")"
echo "This will use the saved map for navigation"
echo "==============================================="
echo ""

# Verify Python and ROS2 installation
echo "Verifying Python and ROS2 installation..."
python3 --version
ros2 --help > /dev/null && echo "ROS2 CLI is working"

# Launch the navigation system
echo "Starting static map navigation..."
ros2 launch xavier_robotics static_map_navigation.launch.py map_yaml_path:="$SELECTED_MAP"
