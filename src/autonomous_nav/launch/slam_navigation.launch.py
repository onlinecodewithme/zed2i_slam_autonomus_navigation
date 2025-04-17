#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch.conditions import IfCondition, UnlessCondition
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get the launch directory
    autonomous_nav_dir = get_package_share_directory('autonomous_nav')
    robot_desc_dir = get_package_share_directory('robot_description')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    # Configuration files
    nav_params_path = os.path.join(autonomous_nav_dir, 'config', 'nav2_params.yaml')
    slam_params_path = os.path.join(autonomous_nav_dir, 'config', 'slam_params.yaml')
    default_bt_xml_path = os.path.join(get_package_share_directory('nav2_bt_navigator'),
                                      'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_file = LaunchConfiguration('map', default='')
    use_slam = LaunchConfiguration('slam', default='True')
    use_nav = LaunchConfiguration('nav', default='True')
    autostart = LaunchConfiguration('autostart', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # Create our custom nav2 params file
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_bt_xml_filename': default_bt_xml_path,
    }
    
    # Process parameter substitutions
    configured_nav_params = RewrittenYaml(
        source_file=nav_params_path,
        param_rewrites=param_substitutions,
        convert_types=True)
    
    # Static odom to base_link transform - needed for navigation to work
    static_odom_publisher_node = Node(
        package='autonomous_nav',
        executable='static_odom_publisher.py',
        name='static_odom_publisher',
        output='screen'
    )
    
    # ZED Point Cloud Processor Node for obstacle detection
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
    
    # Process xacro file to generate URDF
    xacro_file = os.path.join(robot_desc_dir, 'urdf/simple_robot.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file])
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
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
    
    # SLAM Toolbox - only when SLAM is enabled
    slam_node = Node(
        condition=IfCondition(use_slam),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_path],
    )
    
    # Nav2 Lifecycle Manager
    lifecycle_manager_nav = Node(
        condition=IfCondition(use_nav),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'autostart': autostart,
                    'node_names': ['controller_server',
                                   'planner_server',
                                   'behavior_server',
                                   'bt_navigator',
                                   'waypoint_follower']}]
    )
    
    # Nav2 Components - only when navigation is enabled
    controller_server = Node(
        condition=IfCondition(use_nav),
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
        condition=IfCondition(use_nav),
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
        condition=IfCondition(use_nav),
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
        condition=IfCondition(use_nav),
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
    
    # Waypoint Follower for multi-goal navigation
    waypoint_follower = Node(
        condition=IfCondition(use_nav),
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_nav_params]
    )
    
    # Nav2 Map Server - only needed when using a pre-built map
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
    
    # Nav2 AMCL - Localization only when not using SLAM
    amcl = Node(
        condition=UnlessCondition(use_slam),  # Only used when not using SLAM
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_nav_params],
        remappings=[
            ('scan', '/scan'),
            ('map', '/map'),
            ('map_metadata', '/map_metadata')
        ]
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
        'map',
        default_value='',
        description='Full path to map yaml file to load'))
    
    ld.add_action(DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Whether to run SLAM'))
    
    ld.add_action(DeclareLaunchArgument(
        'nav',
        default_value='True',
        description='Whether to run navigation'))
    
    ld.add_action(DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start lifecycle nodes'))
    
    ld.add_action(DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'))
    
    # Add nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(static_odom_publisher_node)  # Add static odom publisher
    ld.add_action(point_cloud_processor_node)
    ld.add_action(zed_to_laserscan_node)
    ld.add_action(slam_node)
    ld.add_action(controller_server)
    ld.add_action(planner_server)
    ld.add_action(behavior_server)
    ld.add_action(bt_navigator)
    ld.add_action(waypoint_follower)
    ld.add_action(lifecycle_manager_nav)
    ld.add_action(map_server)
    ld.add_action(amcl)
    ld.add_action(map_saver_server)
    ld.add_action(rviz_node)
    
    return ld
