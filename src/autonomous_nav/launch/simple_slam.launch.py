#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get the launch directory
    autonomous_nav_dir = get_package_share_directory('autonomous_nav')
    robot_desc_dir = get_package_share_directory('robot_description')
    
    # Configuration files
    slam_params_path = os.path.join(autonomous_nav_dir, 'config', 'slam_params.yaml')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
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
    
    # SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_path],
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
    
    # Add nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(static_odom_publisher_node)
    ld.add_action(slam_node)
    ld.add_action(rviz_node)
    
    return ld
