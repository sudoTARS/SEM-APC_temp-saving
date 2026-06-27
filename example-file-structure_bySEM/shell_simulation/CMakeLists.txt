cmake_minimum_required(VERSION 3.8)
project(shell_simulation)

# Set C++ standard (required for modern ROS2 packages)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)  # For mixed C++/Python package
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# (rclpy is pulled in by ament_cmake_python)
find_package(rclpy REQUIRED)

# C++ example node\add_executable(example_control src/example_control.cpp)
ament_target_dependencies(example_control
  rclcpp std_msgs geometry_msgs visualization_msgs nav_msgs
)
install(TARGETS example_control
  DESTINATION lib/${PROJECT_NAME}
)

# Python scripts (ensure scripts/example_control.py is executable)
install(PROGRAMS
  scripts/example_control.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files so ROS2 can find them after install
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python module if present
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
