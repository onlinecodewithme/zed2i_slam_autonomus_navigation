import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    default_model_path = os.path.join(pkg_share, 'urdf/robot.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    model = LaunchConfiguration('model', default=default_model_path)
    rviz_config = LaunchConfiguration('rviz_config', default=default_rviz_config_path)

    # Launch declarations
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_model_path,
        description='Absolute path to robot urdf file'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file'
    )
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Parse URDF through xacro
    robot_description_content = Command([
        'xacro ', LaunchConfiguration('model')
    ])
    robot_description = {'robot_description': robot_description_content}

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Joint state publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Joint state publisher GUI node (useful for moving joints in visualization)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=LaunchConfiguration('gui', default='true')
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        model_arg,
        rviz_config_arg,
        sim_time_arg,
        DeclareLaunchArgument('gui', default_value='true'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
