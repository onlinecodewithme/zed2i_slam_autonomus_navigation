import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the airplane detection package
    pkg_share = get_package_share_directory('airplane_detection')
    
    # Path to the params file
    params_file = os.path.join(pkg_share, 'config', 'detection_params.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file_arg = LaunchConfiguration('params_file', default=params_file)
    model_path = LaunchConfiguration('model_path', default='')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Path to the parameters file'
    )
    
    declare_model_path = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description='Path to YOLOv5 model file (leave empty for default pretrained model)'
    )
    
    # Airplane detector node
    detector_node = Node(
        package='airplane_detection',
        executable='airplane_detector_node.py',
        name='airplane_detector',
        output='screen',
        parameters=[
            params_file_arg,
            {'use_sim_time': use_sim_time},
            {'model_path': model_path}
        ]
    )
    
    # Detection visualizer node
    visualizer_node = Node(
        package='airplane_detection',
        executable='detection_visualizer.py',
        name='detection_visualizer',
        output='screen',
        parameters=[
            params_file_arg,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_model_path,
        detector_node,
        visualizer_node
    ])
