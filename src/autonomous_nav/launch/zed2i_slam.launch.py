#!/usr/bin/env python3

import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from threading import Thread

def generate_launch_description():
    # Get the launch directory
    autonomous_nav_dir = get_package_share_directory('autonomous_nav')
    robot_desc_dir = get_package_share_directory('robot_description')
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')
    
    # Configuration files
    slam_params_path = os.path.join(autonomous_nav_dir, 'config', 'slam_params.yaml')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    camera_check_timeout = LaunchConfiguration('camera_check_timeout', default='15.0')
    camera_check_retries = LaunchConfiguration('camera_check_retries', default='3')
    
    # First, check the ZED camera status
    camera_check_node = Node(
        package='autonomous_nav',
        executable='zed_camera_check.py',
        name='zed_camera_check',
        output='screen',
        parameters=[{
            'verbose': True,
            'max_retries': camera_check_retries,
            'retry_delay': 2.0,
            'check_timeout': camera_check_timeout
        }]
    )
    
    # Kill any existing ZED processes that might interfere with camera initialization
    cleanup_cmd = ExecuteProcess(
        cmd=['pkill', '-f', 'zed_'],
        shell=True,
        output='screen',
        on_exit=[
            # Add a small delay to ensure processes are terminated before starting new ones
            ExecuteProcess(cmd=['sleep', '2'], shell=True)
        ]
    )
    
    # Include ZED camera launch, only after camera check has completed
    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('zed_wrapper'), 'launch', 'zed_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name': 'zed2i',
            'publish_tf': 'true',
            'publish_map_tf': 'false',  # We'll handle this with SLAM
            'publish_urdf': 'true',
            'serial_number': '0',
            'base_frame': 'base_link',
            'cam_pos_x': '0.35',  # Front of robot (at 0.35m from center)
            'cam_pos_y': '0.0',   # Center
            'cam_pos_z': '0.75',  # Height above base_link
            'cam_roll': '0.0',
            'cam_pitch': '0.0',
            'cam_yaw': '0.0',
            'verbose': 'true'     # Enable verbose mode for better diagnostics
        }.items()
    )
    
    # Static odom to base_link transform - needed for navigation to work
    static_odom_publisher_node = Node(
        package='autonomous_nav',
        executable='static_odom_publisher.py',
        name='static_odom_publisher',
        output='screen'
    )
    
    # ZED to Laserscan node - converts depth image to laserscan for SLAM
    zed_to_laserscan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='zed_to_laserscan',
        remappings=[
            ('depth', '/zed2i/zed_node/depth/depth_registered'),
            ('depth_camera_info', '/zed2i/zed_node/depth/camera_info'),
            ('scan', '/scan')
        ],
        parameters=[{
            'scan_time': 0.033,
            'range_min': 0.5,
            'range_max': 10.0,
            'output_frame': 'base_link',
        }]
    )
    
    # Point Cloud Processor Node for obstacle detection
    point_cloud_processor_node = Node(
        package='autonomous_nav',
        executable='point_cloud_processor_node.py',
        name='point_cloud_processor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'point_cloud_topic': '/zed2i/zed_node/point_cloud/cloud_registered',
            'markers_topic': '/obstacles/markers',
            'obstacles_topic': '/obstacles',
            'robot_width': 0.82,
            'robot_length': 1.07,
            'robot_height': 0.68,
            'safety_margin': 0.5,
            'ground_threshold': 0.15,
            'cluster_distance': 0.3,
            'min_cluster_size': 10,
            'voxel_size': 0.1,
            'update_rate': 5.0,
        }]
    )
    
    # SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_path],
    )
    
    # Map Saver Node
    map_saver_server = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver_server',
        output='screen',
        parameters=[{
            'save_map_timeout': 5.0,
            'free_thresh_default': 0.25,
            'occupied_thresh_default': 0.65,
        }]
    )
    
    # RViz
    rviz_config_file = os.path.join(autonomous_nav_dir, 'rviz', 'navigation.rviz')
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'))
    
    ld.add_action(DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'))
        
    ld.add_action(DeclareLaunchArgument(
        'camera_check_timeout',
        default_value='15.0',
        description='Timeout for camera check in seconds'))
        
    ld.add_action(DeclareLaunchArgument(
        'camera_check_retries',
        default_value='3',
        description='Number of retries for camera check'))
    
    # First run the cleanup to kill any existing ZED processes
    ld.add_action(cleanup_cmd)
    
    # Then check if camera is properly connected and accessible
    ld.add_action(camera_check_node)
    
    # Define event handler to wait for camera check to complete before launching other nodes
    camera_ready_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=camera_check_node,
            on_exit=[
                # Add nodes in the order they should start once camera is confirmed ready
                zed_wrapper_launch,
                static_odom_publisher_node,
                zed_to_laserscan_node,
                point_cloud_processor_node,
                slam_node,
                map_saver_server,
                rviz_node
            ]
        )
    )
    
    # Add the event handler to ensure proper sequencing
    ld.add_action(camera_ready_event)
    
    return ld
