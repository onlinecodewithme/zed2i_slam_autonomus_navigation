#!/bin/bash

# Script to save the current dynamic costmap to disk

# Ensure the script is executable
if [ ! -x "$0" ]; then
    echo "Making script executable..."
    chmod +x "$0"
fi

# Set up ROS2 environment
source /opt/ros/humble/setup.bash
source ~/xavier_robotics/install/setup.bash

# Set default map name
MAP_NAME="dynamic_costmap_$(date +%Y%m%d_%H%M%S)"

# Allow custom map name from command line parameter
if [ "$1" != "" ]; then
    MAP_NAME="$1"
fi

# Make sure the maps directory exists
MAPS_DIR="$HOME/maps"
mkdir -p "$MAPS_DIR"

# Set executable permission for the costmap saver script
chmod +x ~/xavier_robotics/src/autonomous_nav/scripts/costmap_saver.py

# Run the costmap saver
echo "Saving current costmap as '$MAP_NAME'..."
ros2 run autonomous_nav costmap_saver.py --ros-args -p map_name:=$MAP_NAME -p output_dir:=$MAPS_DIR

echo "Costmap saved to: $MAPS_DIR/$MAP_NAME*.pgm and $MAPS_DIR/$MAP_NAME*.yaml"
