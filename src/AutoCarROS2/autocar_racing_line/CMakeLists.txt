cmake_minimum_required(VERSION 3.5)
project(autocar_racing_line)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/generate_racing_line.py
  scripts/generate_oval_waypoints.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
