#!/usr/bin/env python3

import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get directories
    autonomous_nav_dir = get_package_share_directory('autonomous_nav')
    xavier_robotics_dir = get_package_share_directory('xavier_robotics')
    
    # Configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Map to Odom transform publisher
    static_map_to_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    
    # Local costmap generator
    local_costmap_generator_node = Node(
        executable=os.path.join(os.getcwd(), 'src/autonomous_nav/scripts/local_costmap_generator.py'),
        name='local_costmap_generator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'depth_topic': '/zed2i/zed_node/depth/depth_registered',
            'camera_info_topic': '/zed2i/zed_node/rgb/camera_info',
            'pointcloud_topic': '/zed2i/zed_node/point_cloud/cloud_registered',
            'costmap_topic': '/local_costmap',
            'costmap_resolution': 0.05,
            'costmap_width': 20.0,
            'costmap_height': 20.0,
            'costmap_origin_x': -10.0,
            'costmap_origin_y': -10.0,
            'min_height': 0.05,
            'max_height': 2.0,
            'obstacle_threshold': 0.5,
            'inflation_radius': 0.5,
            'update_rate': 5.0,
            'use_pointcloud': True  # Using pointcloud for better accuracy
        }]
    )
    
    # Main launch file to include
    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(xavier_robotics_dir, 'launch', 'autonomous_airplane_inspection.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Costmap saver node
    costmap_saver_node = Node(
        executable=os.path.join(os.getcwd(), 'src/autonomous_nav/scripts/costmap_saver.py'),
        name='costmap_saver',
        output='screen',
        parameters=[{
            'output_dir': os.path.expanduser('~/maps'),
            'save_frequency': 0.0,  # Save once only
            'map_name': 'autonomous_inspection_costmap',
            'costmap_topic': '/local_costmap'
        }]
    )
    
    # RViz
    rviz_config_file = os.path.join(autonomous_nav_dir, 'rviz', 'navigation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add our TF publisher first
    ld.add_action(static_map_to_odom_node)
    
    # Add local costmap generator
    ld.add_action(local_costmap_generator_node)
    
    # Add costmap saver
    ld.add_action(costmap_saver_node)
    
    # Add RViz node
    ld.add_action(rviz_node)
    
    # Add main launch file
    ld.add_action(main_launch)
    
    return ld
