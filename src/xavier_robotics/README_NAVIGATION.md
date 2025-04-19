# Autonomous Navigation with ZED2i Camera Integration

This document explains how to set up and run autonomous navigation for airplane inspection using the ZED2i depth camera as the primary perception system. This system uses the camera's point cloud data to generate costmaps for navigation.

## System Components

The autonomous navigation system consists of these key components:

1. **Local Costmap Generator**: Processes ZED2i camera data (pointcloud) to create a costmap
2. **Global Costmap Relay**: Bridges the local costmap to the expected global costmap for the inspector
3. **Autonomous Airplane Inspector**: Plans and executes inspection paths around aircraft
4. **Debugging Utilities**: Tools to monitor costmap messages and system status

## Fixed Issues

We addressed several key issues in the costmap generation system:

1. **OccupancyGrid Message Format**: Fixed data value range to comply with ROS2 OccupancyGrid requirements [-128, 127] 
2. **Costmap Visibility**: Now initializing costmap with free space (0) instead of unknown (-1) for better RViz visualization
3. **Topic Synchronization**: Added relay to bridge the generated costmap to the expected topics for navigation

## Running the System

### 1. Basic Costmap Generation

To test just the costmap generation:

```bash
./src/xavier_robotics/scripts/run_costmap_with_debug.sh --rviz
```

This will start:
- Local costmap generator with ZED2i camera
- Debug node to monitor the costmap
- RViz for visualizing the costmap

### 2. Full Navigation System

To run the full autonomous navigation system:

```bash
./src/xavier_robotics/scripts/run_full_navigation.sh --rviz --inspector
```

Options:
- `--rviz`: Launch RViz for visualization
- `--debug`: Launch debug nodes for monitoring
- `--inspector`: Launch the autonomous airplane inspector

### 3. Custom Configuration

You can also run individual components for custom testing:

```bash
# Run just the costmap generator
ros2 run autonomous_nav local_costmap_generator.py

# Run the costmap relay
ros2 run autonomous_nav global_costmap_relay.py

# Run just the autonomous inspector
ros2 run autonomous_nav autonomous_airplane_inspector.py
```

## Troubleshooting

### RViz Costmap Display Issues

If costmaps are not visible in RViz:

1. Verify that transform frames exist:
   ```bash
   ros2 run tf2_ros tf2_echo map base_link
   ```

2. Check costmap topics are publishing:
   ```bash
   ros2 topic info /local_costmap
   ros2 topic info /global_costmap/costmap
   ```

3. Check for error messages in the debug node:
   ```bash
   ros2 run xavier_robotics debug_costmap.py
   ```

### ZED Camera Issues

If ZED camera data is not coming through:

1. Verify camera is connected:
   ```bash
   ros2 run autonomous_nav zed_camera_check.py
   ```
   
2. Check if pointcloud topic is publishing:
   ```bash
   ros2 topic echo /zed2i/zed_node/point_cloud/cloud_registered --once
   ```

## Architecture Overview

The navigation system follows this data flow:

```
ZED2i Camera 
    ↓
Point Cloud Data
    ↓
Local Costmap Generator
    ↓
Global Costmap Relay
    ↓
Autonomous Airplane Inspector
    ↓
Navigation Commands
```

## Next Steps

For further enhancement of the system:

1. Improve obstacle detection sensitivity
2. Add dynamic map updating based on camera movement
3. Integrate with path planning algorithms
4. Add additional inspection patterns for aircraft inspection
