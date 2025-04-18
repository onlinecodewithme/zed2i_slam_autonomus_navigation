#!/usr/bin/env python3

# DISABLED: This script has been disabled to prevent test data from being displayed
# in RViz. Use the real_camera_only.launch.py system instead.

import rclpy
from rclpy.node import Node

def main(args=None):
    print("ERROR: This test data generator has been DISABLED to avoid purple test patterns.")
    print("       Use xavier_robotics/scripts/run_real_camera_only.sh instead.")
    return 1  # Return error code

if __name__ == '__main__':
    main()
