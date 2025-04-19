#!/bin/bash

# Run costmap generator and debug tool for autonomous navigation
# This script makes it easier to run the necessary components for local costmap generation
# with the ZED2i camera and visualize the output

# Make script stop on first error
set -e

# Display usage information
show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Run the local costmap generator with ZED2i camera."
    echo ""
    echo "Options:"
    echo "  -h, --help            Display this help message"
    echo "  -s, --save-images     Save costmap debug images to disk"
    echo "  -o, --output-dir DIR  Directory to save debug images (default: /tmp/costmap_debug)"
    echo "  -r, --rviz            Launch RViz with costmap configuration"
    echo ""
    echo "Examples:"
    echo "  $0                    Run costmap generator and debug tool"
    echo "  $0 -s -r              Run costmap generator and debug tool, save images and launch RViz"
}

# Parse command line options
SAVE_IMAGES=false
OUTPUT_DIR="/tmp/costmap_debug"
LAUNCH_RVIZ=false

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        -s|--save-images)
            SAVE_IMAGES=true
            shift
            ;;
        -o|--output-dir)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        -r|--rviz)
            LAUNCH_RVIZ=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# Source ROS 2 and the workspace
echo "Sourcing ROS 2 and workspace..."
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "ERROR: ROS 2 Humble installation not found at /opt/ros/humble"
    exit 1
fi

if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
elif [ -f ../../../install/setup.bash ]; then
    source ../../../install/setup.bash
else
    echo "WARNING: Could not find workspace setup.bash file"
fi

# Get the current directory
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
PROJECT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

echo "Starting ZED camera and costmap generator..."

# Launch RViz if requested
if [ "$LAUNCH_RVIZ" = true ]; then
    echo "Launching RViz with costmap configuration..."
    ros2 run rviz2 rviz2 -d "$PROJECT_DIR/src/autonomous_nav/rviz/costmap_only.rviz" &
    RVIZ_PID=$!
    sleep 2  # Give RViz time to start
fi

# Launch costmap debug node
echo "Launching costmap debug node..."
if [ "$SAVE_IMAGES" = true ]; then
    echo "Saving debug images to: $OUTPUT_DIR"
    ros2 run xavier_robotics debug_costmap.py --ros-args -p save_images:=true -p output_dir:=$OUTPUT_DIR &
else
    ros2 run xavier_robotics debug_costmap.py &
fi
DEBUG_PID=$!

# Launch local costmap generator
echo "Launching local costmap generator node..."
ros2 run autonomous_nav local_costmap_generator.py &
COSTMAP_PID=$!

# Wait for user to press Ctrl+C
echo ""
echo "Local costmap generator is running!"
echo "Press Ctrl+C to stop all processes"
echo ""

# Handle termination and cleanup
cleanup() {
    echo ""
    echo "Stopping all processes..."
    kill $COSTMAP_PID 2>/dev/null || true
    kill $DEBUG_PID 2>/dev/null || true
    if [ "$LAUNCH_RVIZ" = true ]; then
        kill $RVIZ_PID 2>/dev/null || true
    fi
    exit 0
}

# Set the trap for SIGINT (Ctrl+C)
trap cleanup SIGINT

# Wait for background processes
wait $COSTMAP_PID
