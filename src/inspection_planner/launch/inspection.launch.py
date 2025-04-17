import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    inspection_planner_dir = get_package_share_directory('inspection_planner')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file', default=os.path.join(inspection_planner_dir, 'config', 'inspection_params.yaml'))
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to the ROS2 parameters file for inspection'
    )
    
    # Define nodes
    inspection_planner_node = Node(
        package='inspection_planner',
        executable='inspection_planner_node.py',
        name='inspection_planner',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )
    
    inspection_visualizer_node = Node(
        package='inspection_planner',
        executable='inspection_visualizer.py',
        name='inspection_visualizer',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_params_file)
    
    # Add nodes
    ld.add_action(inspection_planner_node)
    ld.add_action(inspection_visualizer_node)
    
    return ld
