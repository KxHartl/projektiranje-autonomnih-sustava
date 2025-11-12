cmake_minimum_required(VERSION 3.8)
project(fanuc_m10ia_support)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install rviz configuration files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDF and mesh files
install(
  DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python test and helper nodes as ROS executables
install(
  PROGRAMS
    test/test_trajectory.py
    test/forward_position_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
