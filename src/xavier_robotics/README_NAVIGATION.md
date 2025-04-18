# Navigation System for Xavier Robotics

This document explains how to use the navigation system for autonomous robot operation.

## Overview

The navigation system uses the ZED camera to:
1. Generate real-time costmaps from depth data
2. Save these costmaps as static maps for navigation
3. Load saved maps for autonomous navigation planning

## Two Main Operating Modes

### 1. Real Camera Mode (Map Creation)

This mode uses the ZED camera to scan the environment and generate costmaps, which are automatically saved to disk.

**Run with:**
```bash
./src/xavier_robotics/scripts/run_real_camera_only.sh
```

**What it does:**
- Connects to the ZED 2i camera
- Processes depth and point cloud data
- Creates a dynamic costmap based on 3D data
- Displays the costmap in RViz
- Saves the costmap periodically to `~/maps/`

### 2. Static Map Navigation Mode

This mode loads a previously saved map for navigation planning.

**Run with:**
```bash
./src/xavier_robotics/scripts/run_static_map_navigation.sh
```

**What it does:**
- Lists available maps in the `~/maps/` directory
- Lets you select which map to use
- Loads the map as a static map for navigation
- Continues to update the local costmap with real-time data
- Displays both the static map and costmaps in RViz

## Workflow

1. **Map Creation**:
   - Run `./src/xavier_robotics/scripts/run_real_camera_only.sh`
   - Move the robot around to scan the environment
   - Maps are saved automatically to `~/maps/`
   - Press Ctrl+C when done mapping

2. **Navigation Using a Map**:
   - Run `./src/xavier_robotics/scripts/run_static_map_navigation.sh`
   - Select a map from the list (or press Enter for the latest)
   - The system will load the map and show it in RViz
   - Costmaps will be updated based on new sensor data

## Maps

Maps are saved in the `~/maps/` directory with:
- `.pgm` files containing the image data
- `.yaml` files containing metadata (resolution, origin, etc.)

The map name format is: `real_camera_costmap_YYYYMMDD_HHMMSS_XXX.yaml`

## Troubleshooting

If the ZED camera is not connecting:
- Check that it's properly plugged in
- Verify that `zed_camera_check.py` is showing correct detection
- Try running `ls /dev/video*` to see if camera devices are present

If costmaps are not visible in RViz:
- Check that RViz is using the right configuration 
- Ensure the correct frame IDs are being used
- Verify topics are being published with `ros2 topic list` and `ros2 topic echo`

## Technical Details

### Topics

- `/map` - The static map (when in navigation mode)
- `/local_costmap` - The local costmap based on current sensor data
- `/global_costmap/costmap` - The global costmap (relayed from local costmap)
- `/local_costmap_updates` - Updates to the local costmap
- `/global_costmap/costmap_updates` - Updates to the global costmap
- `/zed2i/zed_node/point_cloud/cloud_registered` - Point cloud from ZED camera
- `/zed2i/zed_node/depth/depth_registered` - Depth image from ZED camera

### Frame IDs

- `map` - The fixed global reference frame
- `odom` - The odometry frame
- `base_link` - The robot base frame
- `camera_link` - The camera frame
