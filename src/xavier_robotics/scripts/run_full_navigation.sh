#!/bin/bash

# Run the full navigation stack with ZED2i camera for autonomous airplane inspection
# This script ties together the local costmap generator, global costmap relay,
# and optional components for autonomous navigation

# Make script stop on first error
set -e

# Display usage information
show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Run the autonomous navigation stack with ZED2i camera."
    echo ""
    echo "Options:"
    echo "  -h, --help            Display this help message"
    echo "  -r, --rviz            Launch RViz with navigation configuration"
    echo "  -d, --debug           Launch debug nodes to monitor the system"
    echo "  -i, --inspector       Launch the autonomous airplane inspector"
    echo ""
    echo "Examples:"
    echo "  $0                    Run costmap generator and global relay"
    echo "  $0 -r -d              Run with RViz and debug nodes"
    echo "  $0 -r -i              Run with RViz and airplane inspector"
}

# Parse command line options
LAUNCH_RVIZ=false
LAUNCH_DEBUG=false
LAUNCH_INSPECTOR=false

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        -r|--rviz)
            LAUNCH_RVIZ=true
            shift
            ;;
        -d|--debug)
            LAUNCH_DEBUG=true
            shift
            ;;
        -i|--inspector)
            LAUNCH_INSPECTOR=true
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

echo "Starting ZED camera and navigation stack..."

# Create a directory for process IDs
mkdir -p /tmp/navigation_pids

# Start static transform publishers for map->base_link
echo "Publishing static transforms..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link &
echo $! > /tmp/navigation_pids/tf_pub_map.pid

# For ZED camera
ros2 run tf2_ros static_transform_publisher 0 0 0.3 0 0 0 base_link zed2i_camera_center &
echo $! > /tmp/navigation_pids/tf_pub_zed.pid

# Launch RViz if requested
if [ "$LAUNCH_RVIZ" = true ]; then
    echo "Launching RViz with navigation configuration..."
    ros2 run rviz2 rviz2 -d "$PROJECT_DIR/src/autonomous_nav/rviz/navigation.rviz" &
    echo $! > /tmp/navigation_pids/rviz.pid
    sleep 2  # Give RViz time to start
fi

# Launch debug nodes if requested
if [ "$LAUNCH_DEBUG" = true ]; then
    echo "Launching debug nodes..."
    ros2 run xavier_robotics debug_costmap.py &
    echo $! > /tmp/navigation_pids/debug.pid
fi

# Launch local costmap generator
echo "Launching local costmap generator..."
ros2 run autonomous_nav local_costmap_generator.py &
echo $! > /tmp/navigation_pids/local_costmap.pid

# Launch global costmap relay
echo "Launching global costmap relay..."
ros2 run autonomous_nav global_costmap_relay.py &
echo $! > /tmp/navigation_pids/global_relay.pid

# Launch autonomous airplane inspector if requested
if [ "$LAUNCH_INSPECTOR" = true ]; then
    echo "Launching autonomous airplane inspector..."
    ros2 run autonomous_nav autonomous_airplane_inspector.py &
    echo $! > /tmp/navigation_pids/inspector.pid
fi

# Wait for user to press Ctrl+C
echo ""
echo "Navigation stack is running!"
echo "Press Ctrl+C to stop all processes"
echo ""

# Handle termination and cleanup
cleanup() {
    echo ""
    echo "Stopping all processes..."
    
    # Kill all processes using saved PIDs
    for pid_file in /tmp/navigation_pids/*.pid; do
        if [ -f "$pid_file" ]; then
            pid=$(cat "$pid_file")
            kill $pid 2>/dev/null || true
            rm "$pid_file"
        fi
    done
    
    # Clean up PID directory
    rmdir /tmp/navigation_pids 2>/dev/null || true
    
    exit 0
}

# Set the trap for SIGINT (Ctrl+C)
trap cleanup SIGINT

# Wait for user to press Ctrl+C
wait
