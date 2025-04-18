#!/usr/bin/env python3

# DISABLED: This launch file has been disabled to prevent test data from being displayed
# in RViz. Use the real_camera_only.launch.py system instead.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo, EmitEvent
from launch.events import Shutdown

def generate_launch_description():
    error_msg = LogInfo(
        msg="\n\n" + 
            "*************************** ERROR ***************************\n" +
            "* This test data generator has been DISABLED to avoid       *\n" +
            "* purple test patterns in RViz.                             *\n" +
            "*                                                           *\n" +
            "* Use this command instead:                                 *\n" +
            "* ./src/xavier_robotics/scripts/run_real_camera_only.sh     *\n" +
            "*************************************************************\n\n"
    )
    
    # Force shutdown after displaying error
    shutdown = EmitEvent(event=Shutdown(reason="Test data generation disabled"))
    
    return LaunchDescription([
        error_msg,
        shutdown
    ])
