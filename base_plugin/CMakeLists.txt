cmake_minimum_required(VERSION 3.8)
project(base_plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

# BUILD
include_directories(include)

set(dependencies
  rclcpp
  pluginlib
)

add_executable(
  plugin_loader_node
  src/plugin_loader.cpp
  src/plugin_loader_node.cpp
)

ament_target_dependencies(
  plugin_loader_node
  ${dependencies}
)

# INSTALL
# Make the base_plugin header file available for other packages
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch and config dirs
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install plugin_loader_node
install(
  TARGETS plugin_loader_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
