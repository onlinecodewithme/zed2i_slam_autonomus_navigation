#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Get the launch directory
    autonomous_nav_dir = get_package_share_directory('autonomous_nav')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_pointcloud = LaunchConfiguration('use_pointcloud', default='true')
    
    # Dynamic Costmap Generator Node
    costmap_generator_node = Node(
        package='autonomous_nav',
        executable='local_costmap_generator.py',
        name='local_costmap_generator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'use_pointcloud': use_pointcloud,
            'costmap_resolution': 0.05,
            'initial_costmap_width': 20.0,
            'initial_costmap_height': 20.0,
            'initial_costmap_origin_x': -10.0,
            'initial_costmap_origin_y': -10.0,
            'update_rate': 10.0,
            'inflation_radius': 0.5,
            'map_growth_factor': 1.5,
            'edge_tolerance': 4.0
        }]
    )
    
    # RViz with environment variable set to fix display issues
    rviz_config_file = os.path.join(autonomous_nav_dir, 'rviz', 'costmap_only.rviz')
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        # Setting the DISPLAY environment variable to fix GUI issues
        additional_env={'DISPLAY': ':1'}
    )
    
    # Ensure we have a proper TF between camera and map
    # This is a static transform publisher for testing purposes
    # In a real setup, this would come from SLAM or odometry
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_map_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'zed2i_camera_center'],
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
        'use_pointcloud',
        default_value='true',
        description='Use point cloud instead of depth image for obstacle detection'))
    
    # Add nodes
    ld.add_action(static_tf_node)
    ld.add_action(costmap_generator_node)
    ld.add_action(rviz_node)
    
    return ld
