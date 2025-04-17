cmake_minimum_required(VERSION 3.8)
project(airplane_detection)

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
find_package(cv_bridge REQUIRED)
find_package(robot_interfaces REQUIRED)

# Optional ZED dependencies - will use them if available
find_package(zed_components QUIET)
find_package(zed_interfaces QUIET)
find_package(zed_msgs QUIET)

if(NOT zed_components_FOUND)
  message(WARNING "ZED components package not found, using standard depth camera interface")
endif()

if(NOT zed_interfaces_FOUND)
  message(WARNING "ZED interfaces package not found, using custom interfaces")
endif()

if(NOT zed_msgs_FOUND)
  message(WARNING "ZED messages not found, using standard message types")
endif()

# Required for all configurations
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/airplane_detector_node.py
  scripts/detection_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
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
