#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get the launch directory
    autonomous_nav_dir = get_package_share_directory('autonomous_nav')
    robot_desc_dir = get_package_share_directory('robot_description')
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')
    airplane_detection_dir = get_package_share_directory('airplane_detection')
    inspection_planner_dir = get_package_share_directory('inspection_planner')
    xavier_robotics_dir = get_package_share_directory('xavier_robotics')
    
    # Configuration files
    nav_params_path = os.path.join(autonomous_nav_dir, 'config', 'nav2_params.yaml')
    slam_params_path = os.path.join(autonomous_nav_dir, 'config', 'slam_params.yaml')
    airplane_detection_params_path = os.path.join(airplane_detection_dir, 'config', 'detection_params.yaml')
    inspection_params_path = os.path.join(inspection_planner_dir, 'config', 'inspection_params.yaml')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_slam = LaunchConfiguration('use_slam', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    map_yaml_file = LaunchConfiguration('map', default='')
    
    # Camera health check before starting ZED camera
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
    
    # Comprehensive cleanup to kill any existing processes that might interfere
    cleanup_cmd = ExecuteProcess(
        cmd=['bash', '-c', 'pkill -f "ros2|zed_|slam_toolbox|nav2|rviz2" || true'],
        shell=True,
        output='screen',
        on_exit=[
            # Add a small delay to ensure processes are terminated before starting new ones
            ExecuteProcess(cmd=['sleep', '3'], shell=True)
        ]
    )
    
    # Include ZED camera launch
    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name': 'zed2i',
            'publish_tf': 'true',
            'publish_map_tf': 'false',  # We'll handle this with SLAM
            'publish_urdf': 'true',
            'serial_number': '0',
            'base_frame': 'base_link',
            'cam_pos_x': '0.35',  # Front of robot
            'cam_pos_y': '0.0',   # Center
            'cam_pos_z': '0.75',  # Height above base_link
            'cam_roll': '0.0',
            'cam_pitch': '0.0',
            'cam_yaw': '0.0',
            'verbose': 'true'     # Enable verbose mode for better diagnostics
        }.items()
    )
    
    # Camera monitoring for runtime health checks and automatic recovery
    camera_monitor_node = Node(
        package='autonomous_nav',
        executable='zed_camera_monitor.py',
        name='zed_camera_monitor',
        output='screen',
        parameters=[{
            'health_check_period': 5.0,
            'max_restart_attempts': 3,
            'recovery_timeout': 30.0
        }]
    )
    
    # Map to Odom transform publisher - needs to be published first to establish the TF tree
    static_map_to_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    
    # TF publishers - needed for complete TF tree
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
    
    # SLAM Toolbox - adjusted parameters to match lidar capabilities
    slam_node = Node(
        condition=IfCondition(use_slam),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_path,
            {
                'use_sim_time': use_sim_time,
                'max_laser_range': 10.0,  # Match laserscan range
                'min_laser_range': 0.5,   # Match laserscan range
            }
        ],
    )
    
    # Nav2 (used same parameters as in slam_navigation.launch.py)
    # Create our custom nav2 params file with the correct behavior tree
    nav2_bt_navigator_dir = get_package_share_directory('nav2_bt_navigator')
    
    # Define the behavior tree path - use navigate_to_pose which is known to exist
    navigate_to_pose_bt_path = os.path.join(
        nav2_bt_navigator_dir,
        'behavior_trees', 
        'navigate_to_pose_w_replanning_and_recovery.xml'
    )
    
    # Check if navigate_through_poses behavior tree exists
    navigate_through_poses_bt_path = os.path.join(
        nav2_bt_navigator_dir,
        'behavior_trees', 
        'navigate_through_poses_w_replanning_and_recovery.xml'
    )
    
    if not os.path.exists(navigate_through_poses_bt_path):
        # Fall back to the one we know exists
        navigate_through_poses_bt_path = navigate_to_pose_bt_path
    
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_bt_xml_filename': navigate_to_pose_bt_path,
        'navigate_to_pose_bt_xml_filename': navigate_to_pose_bt_path,
        'navigate_through_poses_bt_xml_filename': navigate_through_poses_bt_path
    }
    
    # Process parameter substitutions
    configured_nav_params = RewrittenYaml(
        source_file=nav_params_path,
        param_rewrites=param_substitutions,
        convert_types=True)
        
    # Nav2 Lifecycle Manager
    lifecycle_manager_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'autostart': True,
                    'node_names': ['controller_server',
                                   'planner_server',
                                   'behavior_server',
                                   'bt_navigator',
                                   'waypoint_follower']}]
    )
    
    # Nav2 Components
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[configured_nav_params],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('odom', '/odom')
        ]
    )
    
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_nav_params],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('odom', '/odom')
        ]
    )
    
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_nav_params],
        remappings=[
            ('cmd_vel', '/cmd_vel')
        ]
    )
    
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_nav_params],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('odom', '/odom')
        ]
    )
    
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_nav_params]
    )
    
    # Local costmap generator
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
    
    # Map Server - only needed when using a pre-built map
    map_server = Node(
        condition=UnlessCondition(use_slam),  # Only used when not using SLAM
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file
        }]
    )
    
    # Map Saver to save maps generated with SLAM
    map_saver_server = Node(
        condition=IfCondition(use_slam),
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver_server',
        output='screen',
        parameters=[configured_nav_params]
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
            'map_name': 'autonomous_inspection_costmap',
            'costmap_topic': '/local_costmap'
        }]
    )
    
    # Airplane Detection Node (commented out due to missing torch dependency)
    # airplane_detector_node = Node(
    #     package='airplane_detection',
    #     executable='airplane_detector_node.py',
    #     name='airplane_detector',
    #     output='screen',
    #     parameters=[airplane_detection_params_path]
    # )
    
    # Autonomous Airplane Inspector Node
    autonomous_inspector_node = Node(
        package='autonomous_nav',
        executable='autonomous_airplane_inspector.py',
        name='autonomous_inspector',
        output='screen'
    )
    
    # RViz with complete display fix
    rviz_config_file = os.path.join(autonomous_nav_dir, 'rviz', 'navigation.rviz')
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        # Ensure proper DISPLAY environment variable and force proper X11 authentication
        prefix="bash -c 'export DISPLAY=:1 && export $(dbus-launch) && exec $0 $@'"
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'))
    
    ld.add_action(DeclareLaunchArgument(
        'use_slam',
        default_value='true',
        description='Whether to use SLAM for mapping'))
    
    ld.add_action(DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load'))
    
    ld.add_action(DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'))
    
    # Create a separate directory to store the costmaps
    # This ensures maps are saved whether the mission completes or not
    costmap_dir = os.path.expanduser('~/maps')
    costmap_dir_cmd = ExecuteProcess(
        cmd=['bash', '-c', f'mkdir -p {costmap_dir}'],
        shell=True
    )
    
    # Add nodes in the order they should start
    
    # First run the cleanup to kill any existing processes
    ld.add_action(cleanup_cmd)
    
    # Create maps directory
    ld.add_action(costmap_dir_cmd)
    
    # Publish static transforms first - this is critical for the TF tree to work properly
    ld.add_action(static_map_to_odom_node)
    
    # Start other TF publishers right away
    ld.add_action(static_odom_publisher_node)
    
    # Start the map server early if not using SLAM
    ld.add_action(map_server)
    
    # Local costmap generator - This generates the real costmap from ZED camera data
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
            'costmap_width': 20.0,
            'costmap_height': 20.0,
            'costmap_origin_x': -10.0,
            'costmap_origin_y': -10.0,
            'min_height': 0.05,
            'max_height': 2.0,
            'obstacle_threshold': 0.5,
            'inflation_radius': 0.5,
            'update_rate': 10.0,  # Increased update rate
            'use_pointcloud': True  # Using pointcloud for better accuracy
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

    # Start costmap nodes directly to make them immediately available
    # Note: No map_publisher node here, only real costmap generation from camera data
    ld.add_action(local_costmap_generator_node)
    ld.add_action(local_updates_node)
    ld.add_action(global_costmap_node)
    ld.add_action(costmap_saver_node)
    
    # Then check if camera is properly connected and accessible
    ld.add_action(camera_check_node)
    
    # Define event handler to wait for camera check to complete before launching other nodes
    camera_ready_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=camera_check_node,
            on_exit=[
                # Add remaining nodes once camera is confirmed ready
                zed_wrapper_launch,
                camera_monitor_node,
                zed_to_laserscan_node,
                point_cloud_processor_node,
                slam_node,
                controller_server,
                planner_server,
                behavior_server,
                bt_navigator,
                waypoint_follower,
                lifecycle_manager_nav,
                map_server,
                map_saver_server,
                autonomous_inspector_node,
                rviz_node
            ]
        )
    )
    
    # Add map saving on both SIGINT and on normal SLAM node exit
    # This ensures maps are saved whether the mission completes or is interrupted
    map_save_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=slam_node,
            on_exit=[
                # Save map at end of mission
                Node(
                    package='nav2_map_server',
                    executable='map_saver_cli',
                    name='map_saver_cli',
                    output='screen',
                    arguments=['-f', os.path.expanduser('~/maps/autonomous_inspection_map')],
                    condition=IfCondition(use_slam)
                ),
                # Also save the costmap specifically
                ExecuteProcess(
                    cmd=['bash', '-c', f'cp -f /tmp/costmap_*.pgm {costmap_dir}/ || true'],
                    shell=True
                )
            ]
        )
    )
    
    # Add the event handlers to ensure proper sequencing
    ld.add_action(camera_ready_event)
    ld.add_action(map_save_event)
    
    return ld
