import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get directories
    autonomous_nav_dir = get_package_share_directory('autonomous_nav')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    nav_params_file = LaunchConfiguration('params_file', 
                                        default=os.path.join(autonomous_nav_dir, 'config', 'navigation_params.yaml'))
    
    # Point cloud processor node
    point_cloud_processor_node = Node(
        package='autonomous_nav',
        executable='point_cloud_processor_node.py',
        name='point_cloud_processor',
        output='screen',
        parameters=[nav_params_file],
    )
    
    # RViz configuration for visualization
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
        'params_file',
        default_value=nav_params_file,
        description='Navigation parameters file'
    ))
    
    # Add nodes
    ld.add_action(point_cloud_processor_node)
    ld.add_action(rviz_node)
    
    return ld
