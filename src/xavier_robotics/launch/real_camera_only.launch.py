#!/usr/bin/env python3

# REAL CAMERA ONLY LAUNCH FILE - NO TEST DATA
# This launch file ONLY uses real ZED camera data with no test patterns

import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Create the launch description
    ld = LaunchDescription()
    
    # Get the share directories
    autonomous_nav_dir = get_package_share_directory('autonomous_nav')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Create message to indicate this is real-camera only mode
    info_msg = LogInfo(
        msg="\n\n" + 
            "********************************************************\n" +
            "*** REAL CAMERA ONLY MODE - NO TEST DATA             ***\n" +
            "*** This launch file only uses real ZED camera data  ***\n" +
            "*** with no test patterns or simulated maps.         ***\n" +
            "********************************************************\n\n"
    )
    ld.add_action(info_msg)
    
    # Only clear temporary costmap files, not log structure
    clear_caches = ExecuteProcess(
        cmd=['bash', '-c', 'rm -rf /tmp/costmap_* /tmp/launch_params_* 2>/dev/null || true'],
        output='screen'
    )
    ld.add_action(clear_caches)
    
    # Ensure log directories exist with proper permissions
    ensure_log_dirs = ExecuteProcess(
        cmd=['bash', '-c', 'if [ -n "$ROS_LOG_DIR" ]; then mkdir -p $ROS_LOG_DIR/launch_logs && chmod -R 755 $ROS_LOG_DIR; fi'],
        output='screen'
    )
    ld.add_action(ensure_log_dirs)
    
    # 1. Static TF publisher (map -> odom)
    static_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    ld.add_action(static_tf_pub)
    
    # 2. Static odom publisher (odom -> base_link)
    static_odom_publisher = Node(
        package='autonomous_nav',
        executable='static_odom_publisher.py',
        name='static_odom_publisher',
        output='screen'
    )
    ld.add_action(static_odom_publisher)
    
    # 3. Local costmap generator - real data only
    local_costmap_generator_node = Node(
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
            'initial_costmap_width': 20.0,
            'initial_costmap_height': 20.0,
            'initial_costmap_origin_x': -10.0,
            'initial_costmap_origin_y': -10.0,
            'min_height': 0.05,
            'max_height': 2.0,
            'obstacle_threshold': 0.5,
            'inflation_radius': 0.5,
            'update_rate': 10.0,
            'use_pointcloud': True,
            'map_growth_threshold': 2.0,
            'map_growth_factor': 1.5,
            'edge_tolerance': 4.0
        }]
    )
    ld.add_action(local_costmap_generator_node)
    
    # 4. Map updates publisher
    map_updates_publisher = Node(
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
    ld.add_action(map_updates_publisher)
    
    # 5. Global costmap relay
    global_costmap_relay = Node(
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
    ld.add_action(global_costmap_relay)
    
    # 6. ZED camera check node
    camera_check_node = Node(
        package='autonomous_nav',
        executable='zed_camera_check.py',
        name='zed_camera_check',
        output='screen',
        parameters=[{
            'verbose': True,
            'max_retries': 3,
            'retry_delay': 2.0,
            'check_timeout': 15.0
        }]
    )
    ld.add_action(camera_check_node)
    
    # 7. ZED camera launch
    try:
        zed_wrapper_dir = get_package_share_directory('zed_wrapper')
        
        zed_camera_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')
            ),
            launch_arguments={
                'camera_model': 'zed2i',
                'camera_name': 'zed2i',
                'publish_tf': 'true',
                'publish_map_tf': 'false',
                'publish_urdf': 'true',
                'serial_number': '0',
                'base_frame': 'base_link'
            }.items()
        )
        ld.add_action(zed_camera_node)
    except Exception as e:
        print(f"Warning: Failed to include ZED camera launch: {e}")
    
    # 8. RViz with custom configuration for real data - showing both costmaps
    rviz_config_file = os.path.join(autonomous_nav_dir, 'rviz', 'costmap_only.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        # Ensure proper DISPLAY environment variable
        prefix="bash -c 'export DISPLAY=:1 && exec $0 $@'"
    )
    ld.add_action(rviz_node)
    
    # 9. Costmap saver
    costmap_saver_node = Node(
        package='autonomous_nav',
        executable='costmap_saver.py',
        name='costmap_saver',
        output='screen',
        parameters=[{
            'output_dir': os.path.expanduser('~/maps'),
            'save_frequency': 0.2,
            'map_name': 'real_camera_costmap',
            'costmap_topic': '/local_costmap'
        }]
    )
    ld.add_action(costmap_saver_node)
    
    return ld
