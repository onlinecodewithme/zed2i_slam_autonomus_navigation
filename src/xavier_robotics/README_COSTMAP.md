# Autonomous Airplane Inspection with 2D Costmap Generation

This document explains how to run the autonomous airplane inspection system with properly functioning costmap generation.

## Overview of Fixes

The following issues have been fixed:

1. **TF Tree Issues**:
   - The static map→odom transform is now published early in the launch sequence
   - Static_odom_publisher is launched before other components to ensure TF is available
   - Fixed transform chain to avoid "map not received" errors

2. **Behavior Tree Navigator Error**:
   - Fixed by handling the behavior tree XML path correctly
   - Added fallback for missing navigation files
   - Provided path verification to avoid startup errors

3. **RViz Display Issues**:
   - Added DISPLAY environment variable properly
   - Included dbus-launch to ensure proper X11 authentication
   - Created launcher script with all necessary environment setup

4. **Costmap Generation**:
   - Improved costmap generator configuration
   - Added automatic saving of costmaps to ~/maps directory
   - Created utility script to check and display generated costmaps

## How to Run

### Option 1: Using the Launch Script (Recommended)

The launcher script handles all environment setup and cleanup automatically:

```bash
# Navigate to project root
cd /home/x4/xavier_robotics

# Run the launcher
./src/xavier_robotics/scripts/launch_inspection.sh
```

This script will:
1. Kill any existing ROS2 processes
2. Create the maps directory if needed
3. Set up the ROS2 environment
4. Configure display settings for RViz
5. Launch the inspection system
6. Save costmaps when done

### Option 2: Manual Launch

If you prefer to launch manually:

```bash
# Kill any existing processes
pkill -f "ros2|zed_|slam_toolbox|nav2|rviz2" || true

# Navigate to project root
cd /home/x4/xavier_robotics

# Source ROS2 and project setup
. /opt/ros/humble/setup.bash
. install/setup.bash

# Set display variable
export DISPLAY=:1
export $(dbus-launch)

# Launch the system
ros2 launch xavier_robotics autonomous_airplane_inspection.launch.py
```

## Checking and Viewing Costmaps

A utility script is provided to check for generated costmaps and display them:

```bash
./src/xavier_robotics/scripts/check_costmaps.sh
```

This will:
1. Check for costmaps in ~/maps directory
2. Check for temporary costmaps in /tmp
3. Copy any temporary costmaps to ~/maps
4. Display the latest costmap if an image viewer is available

## Costmap Generation Process

The system generates costmaps in the following way:

1. The ZED camera provides point cloud data from its depth sensor
2. The point_cloud_processor processes this data to identify obstacles
3. The local_costmap_generator creates a 2D occupancy grid from processed data
4. The costmap_saver periodically saves the costmap to ~/maps directory

Even if some navigation components fail (like the behavior tree navigator), the costmap generation process will still work independently.

## Known Limitations

- The airplane_detector node requires PyTorch but has been disabled due to missing dependencies
- RViz may have trouble displaying on some systems, but the costmap generation still works
- In simulation environments, the ZED camera may not connect properly, but the system is designed to handle this gracefully

## Troubleshooting

If costmaps are not being generated:

1. Check if the local_costmap_generator node is running:
   ```bash
   ros2 node list | grep local_costmap_generator
   ```

2. Check if the ZED camera is publishing point cloud data:
   ```bash
   ros2 topic echo /zed2i/zed_node/point_cloud/cloud_registered --once
   ```

3. Check the ~/maps directory for existing costmaps:
   ```bash
   ls -la ~/maps/autonomous_inspection_costmap_*.pgm
   ```

4. Try running the system with environment variables explicitly set:
   ```bash
   DISPLAY=:1 ./src/xavier_robotics/scripts/launch_inspection.sh
