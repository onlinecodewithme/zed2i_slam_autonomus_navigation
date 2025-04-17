import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter, PushRosNamespace
from launch.conditions import IfCondition

def generate_launch_description():
    # Get the launch directory
    nav_dir = get_package_share_directory('autonomous_nav')
    robot_desc_dir = get_package_share_directory('robot_description')
    airplane_detection_dir = get_package_share_directory('airplane_detection')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file', default=os.path.join(nav_dir, 'config', 'navigation_params.yaml'))
    detection_params_file = LaunchConfiguration('detection_params_file', default=os.path.join(airplane_detection_dir, 'config', 'detection_params.yaml'))
    
    # Robot description
    robot_description = os.path.join(robot_desc_dir, 'urdf/robot.urdf.xacro')
    
    # Declare launch arguments
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to the ROS2 parameters file for navigation'
    )
    
    declare_detection_params_file = DeclareLaunchArgument(
        'detection_params_file',
        default_value=detection_params_file,
        description='Full path to the ROS2 parameters file for airplane detection'
    )

    # Include ZED Wrapper launch file (ensure it's installed)
    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('zed_wrapper'), 'launch', 'zed2i.launch.py')
        ]),
        launch_arguments={
            'publish_urdf': 'false',  # We already publish URDF from robot_description
        }.items()
    )
    
    # Load robot description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(robot_description, 'r').read(),
            'use_sim_time': use_sim_time,
        }]
    )
    
    # Define nodes for autonomous navigation
    point_cloud_processor_node = Node(
        package='autonomous_nav',
        executable='point_cloud_processor_node.py',
        name='point_cloud_processor',
        output='screen',
        parameters=[params_file],
    )
    
    obstacle_avoidance_node = Node(
        package='autonomous_nav',
        executable='obstacle_avoidance.py',
        name='obstacle_avoidance',
        output='screen',
        parameters=[params_file],
    )
    
    path_planner_node = Node(
        package='autonomous_nav',
        executable='path_planner.py',
        name='path_planner',
        output='screen',
        parameters=[params_file],
    )
    
    waypoint_navigator_node = Node(
        package='autonomous_nav',
        executable='waypoint_navigator.py',
        name='waypoint_navigator',
        output='screen',
        parameters=[params_file],
    )
    
    local_costmap_generator_node = Node(
        package='autonomous_nav',
        executable='local_costmap_generator.py',
        name='local_costmap_generator',
        output='screen',
        parameters=[params_file],
    )
    
    # Airplane detection nodes
    airplane_detector_node = Node(
        package='airplane_detection',
        executable='airplane_detector_node.py',
        name='airplane_detector',
        output='screen',
        parameters=[detection_params_file],
    )
    
    detection_visualizer_node = Node(
        package='airplane_detection',
        executable='detection_visualizer.py',
        name='detection_visualizer',
        output='screen',
        parameters=[detection_params_file],
    )
    
    # RViz configuration
    rviz_config_file = os.path.join(nav_dir, 'rviz', 'navigation.rviz')
    if not os.path.exists(rviz_config_file):
        # Default to robot_description's URDF viewer if our specific config doesn't exist
        rviz_config_file = os.path.join(robot_desc_dir, 'rviz', 'urdf_config.rviz')
    
    rviz_node = Node(
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
    ld.add_action(declare_sim_time)
    ld.add_action(declare_params_file)
    ld.add_action(declare_detection_params_file)
    
    # Add robot state publisher
    ld.add_action(robot_state_publisher_node)
    
    # Include ZED Wrapper launch
    ld.add_action(zed_wrapper_launch)
    
    # Add navigation nodes
    ld.add_action(point_cloud_processor_node)
    ld.add_action(obstacle_avoidance_node)
    ld.add_action(path_planner_node)
    ld.add_action(waypoint_navigator_node)
    ld.add_action(local_costmap_generator_node)
    
    # Add airplane detection nodes
    ld.add_action(airplane_detector_node)
    ld.add_action(detection_visualizer_node)
    
    # Add RViz
    ld.add_action(rviz_node)
    
    return ld
