import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter, PushRosNamespace
from launch.conditions import IfCondition

def generate_launch_description():
    # Get package directories
    robot_desc_dir = get_package_share_directory('robot_description')
    airplane_detection_dir = get_package_share_directory('airplane_detection')
    autonomous_nav_dir = get_package_share_directory('autonomous_nav')
    inspection_planner_dir = get_package_share_directory('inspection_planner')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    nav_params_file = LaunchConfiguration('nav_params_file', default=os.path.join(autonomous_nav_dir, 'config', 'navigation_params.yaml'))
    detection_params_file = LaunchConfiguration('detection_params_file', default=os.path.join(airplane_detection_dir, 'config', 'detection_params.yaml'))
    inspection_params_file = LaunchConfiguration('inspection_params_file', default=os.path.join(inspection_planner_dir, 'config', 'inspection_params.yaml'))
    
    # Robot state publisher (URDF)
    robot_description = os.path.join(robot_desc_dir, 'urdf/robot.urdf.xacro')
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
    
    # ZED nodes (directly instead of including possibly missing launch file)
    # This creates a simplified version directly rather than trying to include the ZED wrapper
    zed_node = Node(
        package='zed_wrapper',
        executable='zed_wrapper_node',
        name='zed_node',
        output='screen',
        parameters=[{
            'general.camera_model': 'zed2i',
            'general.camera_name': 'zed2i',
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(LaunchConfiguration('use_zed', default='true'))
    )
    
    # Include Airplane Detection launch
    airplane_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(airplane_detection_dir, 'launch', 'detection.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': detection_params_file,
        }.items()
    )
    
    # Include Autonomous Navigation launch
    autonomous_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(autonomous_nav_dir, 'launch', 'autonomous_navigation.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav_params_file,
        }.items()
    )
    
    # Include Inspection Planner launch
    inspection_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(inspection_planner_dir, 'launch', 'inspection.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': inspection_params_file,
        }.items()
    )
    
    # RViz configuration
    rviz_config_file = os.path.join(autonomous_nav_dir, 'rviz', 'navigation.rviz')
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
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'use_zed',
        default_value='true',
        description='Launch ZED camera node'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'nav_params_file',
        default_value=nav_params_file,
        description='Navigation parameters file'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'detection_params_file',
        default_value=detection_params_file,
        description='Airplane detection parameters file'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'inspection_params_file',
        default_value=inspection_params_file,
        description='Inspection parameters file'
    ))
    
    # Add nodes and included launch files
    ld.add_action(robot_state_publisher_node)
    ld.add_action(zed_node)
    ld.add_action(airplane_detection_launch)
    ld.add_action(autonomous_nav_launch)
    ld.add_action(inspection_planner_launch)
    ld.add_action(rviz_node)
    
    return ld
