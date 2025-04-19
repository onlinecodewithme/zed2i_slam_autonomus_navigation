# Costmap Generation with ZED2i Camera

This module implements local costmap generation for autonomous navigation using a ZED2i camera. It can generate costmaps for obstacles and free space based on the point cloud data from the ZED2i camera.

## Features

- Dynamic costmap generation from ZED2i camera data (pointcloud or depth image)
- Automatic map growth as the camera moves to new areas
- Obstacle inflation for better path planning
- Compatible with Nav2 and RViz visualization

## Implementation Details

The implementation has been organized as a set of reusable modules:

- **grid_utils.py**: Utilities for grid coordinate transformations and operations
- **sensor_processing.py**: Processes sensor data from ZED2i camera (pointcloud & depth)
- **costmap_generator.py**: Core costmap generation and management logic
- **local_costmap_generator.py**: ROS2 node that uses the above components

Additionally, debug utilities are provided:
- **debug_costmap.py**: Debug node that monitors costmap messages and provides statistics
- **run_costmap_with_debug.sh**: Script to run everything together

## Key Changes

1. **Improved Visualization**: Costmaps now initialize with free space (0) instead of unknown (-1), making them immediately visible in RViz
2. **Modular Design**: Refactored into reusable components for better maintainability
3. **Robust Processing**: Better error handling and logging
4. **Debugging Tools**: Added tools to monitor and debug costmap generation

## Usage

### Building the Code

```bash
cd ~/ros2_ws
colcon build --packages-select autonomous_nav xavier_robotics
source install/setup.bash
```

### Running with the ZED2i Camera

Use the provided script to run the costmap generator with debugging tools:

```bash
# Basic usage
./src/xavier_robotics/scripts/run_costmap_with_debug.sh

# Run with RViz visualization
./src/xavier_robotics/scripts/run_costmap_with_debug.sh --rviz

# Save debug images
./src/xavier_robotics/scripts/run_costmap_with_debug.sh --save-images --output-dir /tmp/costmap_debug
```

### Manual Launch

If you prefer to run components individually:

1. Start the ZED2i ROS2 driver (if not already running):
```bash
ros2 launch zed_wrapper zed2i.launch.py
```

2. Run the local costmap generator:
```bash
ros2 run autonomous_nav local_costmap_generator.py
```

3. Run the debug utility:
```bash
ros2 run xavier_robotics debug_costmap.py
```

4. Launch RViz with the costmap configuration:
```bash
ros2 run rviz2 rviz2 -d src/autonomous_nav/rviz/costmap_only.rviz
```

## Configuration

Parameters for the costmap generator can be adjusted in three ways:

1. Directly in the node code in `local_costmap_generator.py`
2. Using ROS2 parameter overrides on the command line:
```bash
ros2 run autonomous_nav local_costmap_generator.py --ros-args -p costmap_resolution:=0.1
```
3. With a ROS2 params file in a launch configuration

## Troubleshooting

If the costmap is not visible in RViz:

1. Check that the ZED2i camera is properly connected and the driver is publishing point cloud data
2. Verify the transforms are being published correctly:
```bash
ros2 run tf2_ros tf2_echo map camera_link
```
3. Ensure the costmap topic is being published:
```bash
ros2 topic info /local_costmap
```
4. Run the debug node to monitor costmap statistics:
```bash
ros2 run xavier_robotics debug_costmap.py
```

## Technical Notes

- The costmap is published as an OccupancyGrid message on the `/local_costmap` topic
- Values in the costmap: 0 = free space, 100 = obstacle, 50-99 = inflation around obstacles
- The costmap resolution is configurable (default 0.05m per cell)

For further development or integration with the navigation system, refer to the source code comments.
