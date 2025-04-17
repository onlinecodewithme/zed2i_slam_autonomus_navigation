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
    
    # Comprehensive cleanup to kill any existing processes that might interfere
    cleanup_cmd = ExecuteProcess(
        cmd=['bash', '-c', 'pkill -f "ros2|zed_|slam_toolbox|nav2|rviz2" || true'],
        shell=True,
        output='screen',
        on_exit=[
            # Add delay to ensure processes are terminated
            ExecuteProcess(cmd=['sleep', '3'], shell=True)
        ]
    )
    
    # Static transform publisher to ensure the TF tree is complete
    static_map_to_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    
    # Static odom publisher
    static_odom_publisher_node = Node(
        package='autonomous_nav',
        executable='static_odom_publisher.py',
        name='static_odom_publisher',
        output='screen'
    )
    
    # Local costmap generator with real obstacles
    local_costmap_node = Node(
        package='autonomous_nav',
        executable='local_costmap_generator.py',
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
            'inflation_radius': 0.7,
            'update_rate': 10.0,
            'use_pointcloud': True  # Using pointcloud for better accuracy
        }]
    )
    
    # Global costmap node - creates a copy of local costmap for global planning
    global_costmap_node = Node(
        package='autonomous_nav',
        executable='global_costmap_relay.py',
        name='global_costmap_relay',
        output='screen',
        parameters=[{
            'local_costmap_topic': '/local_costmap',
            'global_costmap_topic': '/global_costmap/costmap',
            'update_rate': 5.0
        }]
    )
    
    # Updates publisher for local costmap
    local_updates_node = Node(
        package='autonomous_nav',
        executable='map_updates_publisher.py',
        name='local_updates_publisher',
        output='screen',
        parameters=[{
            'map_topic': '/local_costmap',
            'update_topic': '/local_costmap_updates',
            'update_rate': 5.0
        }]
    )
    
    # Costmap saver node
    costmap_saver_node = Node(
        package='autonomous_nav',
        executable='costmap_saver.py',
        name='costmap_saver',
        output='screen',
        parameters=[{
            'output_dir': os.path.expanduser('~/maps'),
            'save_frequency': 0.2,  # Save every 5 seconds
            'map_name': 'real_costmap',
            'costmap_topic': '/local_costmap'
        }]
    )
    
    # Base link to camera_link transform
    base_to_camera_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_transform',
        arguments=['0.35', '0.0', '0.75', '0.0', '0.0', '0.0', 'base_link', 'camera_link'],
        output='screen'
    )
    
    # RViz with customized configuration for just costmap display
    rviz_config_file = os.path.join(autonomous_nav_dir, 'rviz', 'costmap_only.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        # Ensure proper DISPLAY environment variable and reduce graphics requirements
        prefix="bash -c 'export DISPLAY=:1 && export LIBGL_ALWAYS_SOFTWARE=1 && exec $0 $@'"
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'))
    
    # Create maps directory
    maps_dir = os.path.expanduser('~/maps')
    maps_dir_cmd = ExecuteProcess(
        cmd=['bash', '-c', f'mkdir -p {maps_dir}'],
        shell=True
    )
    ld.add_action(maps_dir_cmd)
    
    # First run the cleanup to kill any existing processes
    ld.add_action(cleanup_cmd)
    
    # Add static transforms first (critical for TF tree)
    ld.add_action(static_map_to_odom_node)
    ld.add_action(static_odom_publisher_node)
    ld.add_action(base_to_camera_node)  # Add the camera transform
    
    # Add real costmap generators - order matters!
    ld.add_action(local_costmap_node)  # First generate local costmap
    ld.add_action(local_updates_node)  # Then send updates for local
    ld.add_action(global_costmap_node) # Then create global from local
    ld.add_action(costmap_saver_node)  # Finally save the costmaps
    
    # Add RViz last to make sure all maps are ready
    ld.add_action(rviz_node)
    
    return ld
