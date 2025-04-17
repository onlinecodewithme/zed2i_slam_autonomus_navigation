#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node

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
    
    # ZED Camera launch file
    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_dir, 'launch', 'zed2i.launch.py')
        ),
        launch_arguments={
            'svo_path': '',  # Empty string means live camera
            'publish_tf': 'true',
            'base_frame': 'base_link',
            'camera_model': 'zed2i',
            'cam_pos_x': '0.0',
            'cam_pos_y': '0.0',
            'cam_pos_z': '0.2',
            'cam_roll': '0.0',
            'cam_pitch': '0.0',
            'cam_yaw': '0.0'
        }.items()
    )
    
    # Static odom to base_link transform - needed for navigation to work
    static_odom_publisher_node = Node(
        package='autonomous_nav',
        executable='static_odom_publisher.py',
        name='static_odom_publisher',
        output='screen'
    )
    
    # Robot description
    robot_description = Command(['ros2 run xacro xacro ', os.path.join(robot_desc_dir, 'urdf/simple_robot.urdf.xacro')])
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }]
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
    
    # Add nodes in the order they should start
    ld.add_action(zed_wrapper_launch)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(static_odom_publisher_node)
    ld.add_action(zed_to_laserscan_node)
    ld.add_action(slam_node)
    ld.add_action(map_saver_server)
    ld.add_action(rviz_node)
    
    return ld
