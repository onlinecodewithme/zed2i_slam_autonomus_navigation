# Dynamic Costmap Generator for ZED 2i Camera

This module provides a dynamic costmap generator for autonomous navigation using the ZED 2i depth camera. Unlike fixed-size costmaps, this implementation creates a map that grows and adapts based on camera movement.

## Features

- **Dynamic map growth**: The map automatically expands as the camera approaches its edges
- **Real-time updates**: Continuous mapping based on camera movement
- **Persistent obstacle mapping**: Builds up a complete environment map over time
- **Auto-inflation**: Automatically inflates obstacles for safer path planning
- **Save capability**: Compatible with existing costmap_saver.py

## Issues Fixed

1. **Fixed-size map limitation**: The costmap now dynamically grows with camera movement instead of having a preset size
2. **Blinking in RViz**: Fixed by ensuring consistent timestamps between header.stamp and map_load_time

## Usage

### Running the Dynamic Costmap Generator

Use the provided script to launch the dynamic costmap generator:

```bash
./src/xavier_robotics/scripts/run_dynamic_costmap.sh
```

This script will:
1. Set up the ROS 2 environment
2. Kill any existing costmap processes
3. Launch the dynamic costmap generator with RViz visualization
4. Save any generated maps on exit

### Configuration Parameters

The dynamic costmap generator can be configured with the following parameters:

- `costmap_resolution`: Resolution of the costmap in meters/cell (default: 0.05)
- `initial_costmap_width`: Initial width of the costmap in meters (default: 20.0)
- `initial_costmap_height`: Initial height of the costmap in meters (default: 20.0)
- `initial_costmap_origin_x`: Initial X coordinate of the costmap origin (default: -10.0)
- `initial_costmap_origin_y`: Initial Y coordinate of the costmap origin (default: -10.0)
- `min_height`: Minimum height for obstacle detection in meters (default: 0.05)
- `max_height`: Maximum height for obstacle detection in meters (default: 2.0)
- `obstacle_threshold`: Threshold for obstacle detection in meters (default: 0.5)
- `inflation_radius`: Radius for obstacle inflation in meters (default: 0.5)
- `update_rate`: Rate for costmap updates in Hz (default: 10.0)
- `use_pointcloud`: Whether to use point cloud or depth image for costmap generation (default: true)
- `map_growth_factor`: How much to grow the map by when expanding (default: 1.5)
- `edge_tolerance`: How close the camera needs to be to an edge to trigger growth in meters (default: 4.0)

You can modify these parameters in the launch file `src/autonomous_nav/launch/dynamic_costmap.launch.py`.

### Saving Costmaps

The costmap is published as a standard ROS 2 OccupancyGrid message on the `/local_costmap` topic, which can be saved using the existing `costmap_saver.py` script:

```bash
ros2 run autonomous_nav costmap_saver.py --map-name my_costmap
```

This will save the current costmap as `my_costmap.pgm` and `my_costmap.yaml` in the `/tmp` directory.

### Integrating with Navigation

To use the dynamic costmap for navigation, you can subscribe to the `/local_costmap` topic in your navigation stack. This costmap is compatible with standard ROS 2 navigation packages.

## Troubleshooting

If you encounter any issues:

1. Ensure the ZED 2i camera is properly connected and initialized
2. Check that the camera transformation frames are correctly published
3. Verify the ROS 2 environment is properly sourced

For blinking issues in RViz, try increasing the persistence timeout in RViz's display settings.
