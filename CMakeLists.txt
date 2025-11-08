cmake_minimum_required(VERSION 3.8)
project(stack_detector)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(include)

# Stack detector node library (for component)
add_library(stack_detector_component SHARED
  src/stack_detector_node.cpp
)

ament_target_dependencies(stack_detector_component
  rclcpp
  rclcpp_components
  geometry_msgs
  std_msgs
)

# Register the component
rclcpp_components_register_nodes(stack_detector_component "stack_detector::StackDetectorNode")

# Stack detector executable
add_executable(stack_detector_node
  src/stack_detector_main.cpp
)

target_link_libraries(stack_detector_node
  stack_detector_component
)

ament_target_dependencies(stack_detector_node
  rclcpp
  geometry_msgs
  std_msgs
)

# Install targets
install(TARGETS
  stack_detector_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  stack_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(stack_detector_component)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  geometry_msgs
  std_msgs
)

ament_package()
