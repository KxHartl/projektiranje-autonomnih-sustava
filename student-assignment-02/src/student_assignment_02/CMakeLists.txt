cmake_minimum_required(VERSION 3.16)
project(student_assignment_02 LANGUAGES NONE)

# Za Python paket s ROS 2
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake REQUIRED)

# Instaliraj Python package
ament_cmake_python_install_package(${PROJECT_NAME})

# Installiraj launch datoteke
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*launch.py"
)

# Installiraj config datoteke  
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Installiraj world datoteke
install(DIRECTORY
  world
  DESTINATION share/${PROJECT_NAME}
)

# Installiraj mapped_maps
install(DIRECTORY
  mapped_maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
