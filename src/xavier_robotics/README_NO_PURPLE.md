# Real Camera Only Mode - No Test Data

This document explains how to use the real camera mode without any test data patterns.

## Problem Solved

The purple pattern in RViz was caused by multiple issues:

1. Test data being displayed instead of real camera data
2. RViz color scheme settings showing unknown cells as purple
3. Duplicate code in the costmap generator
4. ROS caches persisting old visualizations

## Solution

A completely new launch system has been created that:

1. Uses only real ZED camera data
2. Properly clears all ROS caches
3. Uses the right color scheme in RViz
4. Is completely isolated from any test code

## How to Use It

Run this command:

```bash
./src/xavier_robotics/scripts/run_real_camera_only.sh
```

This script will:
- Kill all existing ROS processes
- Remove all ROS caches
- Launch the ZED camera drivers
- Start the costmap generator with real data only
- Open RViz with the right configuration
- Save maps to ~/maps for multi-goal navigation

## Features

- **Dynamic costmap:** The map grows as you move the camera around
- **Real-time obstacle detection:** Only actual obstacles from camera data
- **Proper visualization:** No purple test patterns
- **Map saving:** Automatically saves costmaps for navigation

## Technical Details

The key changes made include:

1. `real_camera_only.launch.py`: A dedicated launch file that bypasses all test code
2. `run_real_camera_only.sh`: A script that thoroughly cleans and launches
3. RViz color schemes changed to "costmap" instead of "map"
4. Test data generators disabled completely
5. Duplicate update method removed from local_costmap_generator.py
