#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    autonomous_nav_dir = get_package_share_directory('autonomous_nav')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Map publisher node - with dynamic resizing for better navigation
    map_publisher_node = Node(
        package='autonomous_nav',
        executable='map_publisher.py',
        name='map_publisher',
        output='screen',
        parameters=[{
            'resolution': 0.05,
            'initial_width': 150,     # will resize dynamically
            'initial_height': 150,    # will resize dynamically
            'initial_origin_x': -3.75,
            'initial_origin_y': -3.75,
            'publish_rate': 5.0,      # Hz - publish faster for RViz
            'publish_map_topic': '/local_costmap',
            'publish_global_topic': '/global_costmap/costmap',
            'dynamic_resize': True,
            'resize_check_interval': 5,  # check every 5 iterations
            'min_free_border': 20,       # minimum cells from obstacle to edge
            'resize_step': 50            # cells to add when resizing
        }]
    )
    
    # Static transform publisher to ensure the TF tree is complete
    static_map_to_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    
    # RViz with complete display fix
    rviz_config_file = os.path.join(autonomous_nav_dir, 'rviz', 'navigation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        # Ensure proper DISPLAY environment variable
        prefix="bash -c 'export DISPLAY=:1 && export $(dbus-launch) && exec $0 $@'"
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'))
    
    # Add nodes in the order they should start
    ld.add_action(static_map_to_odom_node)
    ld.add_action(map_publisher_node)
    ld.add_action(rviz_node)
    
    return ld
