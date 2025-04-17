cmake_minimum_required(VERSION 3.8)
project(xavier_robotics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files and scripts
install(DIRECTORY
  launch
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# Make sure launch and script files are executable
install(
  FILES
  launch/real_costmap_inspection.launch.py
  scripts/run_real_costmap.sh
  DESTINATION share/${PROJECT_NAME}/
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

# Install README
install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
