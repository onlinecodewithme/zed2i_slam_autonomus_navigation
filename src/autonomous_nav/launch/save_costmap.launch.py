#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    autonomous_nav_dir = get_package_share_directory('autonomous_nav')
    
    # Launch configurations
    output_dir = LaunchConfiguration('output_dir', default='~/maps')
    save_frequency = LaunchConfiguration('save_frequency', default='0.0')
    map_name = LaunchConfiguration('map_name', default='costmap')
    costmap_topic = LaunchConfiguration('costmap_topic', default='/local_costmap')
    
    # Declare launch arguments
    declare_output_dir = DeclareLaunchArgument(
        'output_dir',
        default_value='~/maps',
        description='Directory to save costmap files'
    )
    
    declare_save_frequency = DeclareLaunchArgument(
        'save_frequency',
        default_value='0.0',
        description='Frequency to save costmaps (Hz). 0 means save once only when received.'
    )
    
    declare_map_name = DeclareLaunchArgument(
        'map_name',
        default_value='costmap',
        description='Base name for the saved costmap files'
    )
    
    declare_costmap_topic = DeclareLaunchArgument(
        'costmap_topic',
        default_value='/local_costmap',
        description='Topic name for costmap subscription'
    )
    
    # Create costmap saver node
    costmap_saver_node = Node(
        package='autonomous_nav',
        executable='costmap_saver.py',
        name='costmap_saver',
        output='screen',
        parameters=[{
            'output_dir': output_dir,
            'save_frequency': save_frequency,
            'map_name': map_name,
            'costmap_topic': costmap_topic
        }]
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_output_dir)
    ld.add_action(declare_save_frequency)
    ld.add_action(declare_map_name)
    ld.add_action(declare_costmap_topic)
    
    # Add nodes
    ld.add_action(costmap_saver_node)
    
    return ld
