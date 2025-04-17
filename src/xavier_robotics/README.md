# Xavier Robotics - Autonomous Airplane Inspection System

This project implements an autonomous robot system capable of performing airplane inspections by navigating around an aircraft while avoiding obstacles. The system uses a ZED 2i camera for visual perception, SLAM for mapping, and Nav2 for navigation.

## System Requirements

- ROS 2 (Humble)
- ZED SDK (installed)
- ZED 2i stereo camera
- Tracked robot platform (1070mm × 820mm × 680mm)

## Features

- **Autonomous Navigation**: Navigates around airplanes while avoiding obstacles
- **SLAM Mapping**: Builds a map of the environment in real-time
- **Object Detection**: Detects airplanes using computer vision
- **Dynamic Obstacle Avoidance**: Avoids obstacles using point cloud data from the ZED camera
- **Inspection Path Planning**: Generates optimal inspection paths around detected airplanes

## System Architecture

The system consists of several ROS 2 packages:

1. **robot_description**: Contains the URDF description of the robot
2. **robot_interfaces**: Defines custom messages and services for the system
3. **autonomous_nav**: Handles navigation, path planning, and obstacle avoidance
4. **airplane_detection**: Detects airplanes using computer vision
5. **inspection_planner**: Plans inspection paths around airplanes
6. **xavier_robotics**: Top-level package that coordinates all components

## Setup Instructions

1. Connect the ZED 2i camera to your Jetson Orin NX
2. Ensure the ZED SDK is properly installed
3. Build the project:
   ```bash
   cd ~/xavier_robotics
   colcon build
   source install/setup.bash
   ```

## Usage Instructions

### 1. Start SLAM and build a map

For initial mapping of the environment:

```bash
ros2 launch autonomous_nav zed2i_slam.launch.py
```

Drive the robot around manually to create a map of the environment. You can save the map using:

```bash
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "map_url: 'map'"
```

### 2. Start the full autonomous inspection system

```bash
ros2 launch xavier_robotics autonomous_airplane_inspection.launch.py
```

The system will:
1. Initialize the ZED camera
2. Start SLAM or load a map
3. Initialize the navigation stack
4. Start the airplane detection algorithm
5. When an airplane is detected, plan and execute an inspection path

## Launch Files

- **zed2i_slam.launch.py**: Launches ZED camera and SLAM for mapping
- **autonomous_airplane_inspection.launch.py**: Launches the full inspection system

## Key Components

### Airplane Detection

The airplane detection node uses computer vision to identify airplanes in the environment. It publishes detected airplanes as `AirplaneDetection` messages.

### Autonomous Navigation

The navigation system uses Nav2 and the ZED camera's point cloud data to navigate safely around obstacles. It can plan paths to reach inspection waypoints.

### Inspection Planning

When an airplane is detected, the inspection planner generates a path that ensures complete coverage of the aircraft for inspection purposes.

### Point Cloud Processing

The system processes point cloud data from the ZED camera to identify obstacles and create a traversability map for navigation.

## Configuration

You can modify the system behavior by editing the configuration files:

- **slam_params.yaml**: Configure SLAM parameters
- **nav2_params.yaml**: Configure navigation parameters
- **detection_params.yaml**: Configure airplane detection parameters
- **inspection_params.yaml**: Configure inspection planning parameters

## Troubleshooting

If the ZED camera is not detected:
- Ensure it's properly connected to the Jetson
- Check that the ZED SDK is installed and working

If navigation is not working:
- Ensure a valid map is being used
- Check that the static_odom_publisher is running to provide initial transforms

## Development

To extend or modify the system:
1. Add new detection models to the airplane_detection package
2. Extend the inspection_planner for different inspection patterns
3. Modify the navigation parameters for different environments

## License

This project is licensed under the Apache 2.0 License.
