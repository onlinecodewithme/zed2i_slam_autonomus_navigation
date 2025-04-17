#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Get the launch directory
    autonomous_nav_dir = get_package_share_directory('autonomous_nav')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    waypoint_file = LaunchConfiguration('waypoint_file', 
                                       default=os.path.join(autonomous_nav_dir, 'config', 'waypoints_sample.yaml'))
    loop_waypoints = LaunchConfiguration('loop_waypoints', default='false')
    wait_time = LaunchConfiguration('wait_time', default='1.0')
    obstacle_radius = LaunchConfiguration('obstacle_radius', default='1.0')
    small_obstacle_threshold = LaunchConfiguration('small_obstacle_threshold', default='0.3')
    use_slam = LaunchConfiguration('use_slam', default='true')
    include_nav = LaunchConfiguration('include_nav', default='true')
    start_waypoints = LaunchConfiguration('start_waypoints', default='false')
    
    # Include the SLAM Navigation launch file
    slam_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(autonomous_nav_dir, 'launch', 'slam_navigation.launch.py')
        ]),
        condition=IfCondition(include_nav),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': use_slam,
            'nav': 'true',
        }.items()
    )
    
    # Waypoint Manager Node
    waypoint_manager_node = Node(
        package='autonomous_nav',
        executable='waypoint_manager.py',
        name='waypoint_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'waypoint_file': waypoint_file,
            'loop_waypoints': loop_waypoints,
            'wait_time_at_waypoint': wait_time,
            'obstacle_check_radius': obstacle_radius,
            'small_obstacle_threshold': small_obstacle_threshold,
        }],
    )
    
    # Waypoint Starter Node - Publishes a message to start waypoint following if auto-start is enabled
    waypoint_starter_node = Node(
        condition=IfCondition(start_waypoints),
        package='autonomous_nav',
        executable='waypoint_starter.py',
        name='waypoint_starter',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'start_delay': 10.0,  # Seconds to wait before starting waypoints
        }],
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'))
    
    ld.add_action(DeclareLaunchArgument(
        'waypoint_file',
        default_value=os.path.join(autonomous_nav_dir, 'config', 'waypoints_sample.yaml'),
        description='Full path to the waypoints YAML file'))
    
    ld.add_action(DeclareLaunchArgument(
        'loop_waypoints',
        default_value='false',
        description='Whether to loop through waypoints continuously'))
    
    ld.add_action(DeclareLaunchArgument(
        'wait_time',
        default_value='1.0',
        description='Time to wait at each waypoint in seconds'))
    
    ld.add_action(DeclareLaunchArgument(
        'obstacle_radius',
        default_value='1.0',
        description='Radius to check for obstacles around waypoints in meters'))
    
    ld.add_action(DeclareLaunchArgument(
        'small_obstacle_threshold',
        default_value='0.3',
        description='Height threshold to consider an obstacle small (traversable)'))
    
    ld.add_action(DeclareLaunchArgument(
        'use_slam',
        default_value='true',
        description='Whether to use SLAM for mapping'))
    
    ld.add_action(DeclareLaunchArgument(
        'include_nav',
        default_value='true',
        description='Whether to include the navigation stack'))
    
    ld.add_action(DeclareLaunchArgument(
        'start_waypoints',
        default_value='false',
        description='Whether to automatically start waypoint navigation'))
    
    # Add the included launch files and nodes
    ld.add_action(slam_navigation_launch)
    ld.add_action(waypoint_manager_node)
    ld.add_action(waypoint_starter_node)
    
    return ld
