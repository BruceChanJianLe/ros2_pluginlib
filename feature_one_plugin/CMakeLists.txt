cmake_minimum_required(VERSION 3.8)
project(feature_one_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(base_plugin REQUIRED)

# BUILD
include_directories(include)

set(dependencies
  rclcpp
  pluginlib
  base_plugin
)

add_library(
  feature_one SHARED
  src/feature_one_plugin.cpp
)

ament_target_dependencies(
  feature_one
  ${dependencies}
)

# INSTALL
# Install feature_one
install(
  TARGETS feature_one
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(base_plugin plugins.xml)


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

ament_export_libraries(feature_one)
ament_export_targets(export_${PROJECT_NAME})
ament_package()
