cmake_minimum_required(VERSION 3.8)
project(autonomous_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ZED interfaces
find_package(zed_msgs REQUIRED)
find_package(zed_components QUIET)
find_package(zed_wrapper QUIET)
find_package(zed_ros2 QUIET)

# Required Nav2/Navigation dependencies
find_package(nav2_msgs QUIET)
find_package(nav2_util QUIET)
find_package(lifecycle_msgs QUIET)
find_package(octomap_msgs QUIET)
find_package(visualization_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/obstacle_avoidance.py
  scripts/waypoint_navigator.py
  scripts/path_planner.py
  scripts/local_costmap_generator.py
  scripts/point_cloud_processor_node.py
  scripts/waypoint_manager.py
  scripts/waypoint_starter.py
  scripts/static_odom_publisher.py
  scripts/autonomous_airplane_inspector.py
  scripts/zed_camera_check.py
  scripts/zed_camera_monitor.py
  scripts/costmap_saver.py
  scripts/map_publisher.py
  scripts/global_costmap_relay.py
  scripts/map_updates_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Make sure RViz configuration files are executable
install(
  FILES
  rviz/navigation.rviz
  rviz/costmap_only.rviz
  DESTINATION share/${PROJECT_NAME}/rviz/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
