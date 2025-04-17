cmake_minimum_required(VERSION 3.8)
project(inspection_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# ZED interfaces (now installed from git)
find_package(zed_interfaces QUIET)
find_package(zed-ros2-interfaces QUIET)
if(NOT zed_interfaces_FOUND AND NOT zed-ros2-interfaces_FOUND)
  message(WARNING "ZED interfaces not found, using standard interfaces")
endif()

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/inspection_planner_node.py
  scripts/inspection_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
