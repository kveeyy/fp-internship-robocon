cmake_minimum_required(VERSION 3.8)
project(finalproject)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Add executable
add_executable(teleop_keyboard_node src/teleop_keyboard_node.cpp)

ament_target_dependencies(teleop_keyboard_node
  rclcpp
  geometry_msgs
  std_msgs
)

# Install other resources (optional)
install(
  DIRECTORY launch rviz config worlds models description
  DESTINATION share/${PROJECT_NAME}
)

# Install executables
install(TARGETS
  teleop_keyboard_node
  DESTINATION lib/${PROJECT_NAME}
)

# Testing and linting (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export package
ament_package()
