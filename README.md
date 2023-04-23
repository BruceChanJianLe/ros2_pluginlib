# ROS2 Pluginlib

This repository demonstrates the usage of pluginlib and component in ROS2.

## Class Loader

- It is templated with the base class, i.e. `base_plugin::BasePlugin`
- The first argument is a string for the package name of the base class, i.e. `base_plugin`
- The second argument is a string with the fully qualified base class type for the plugin, i.e. `base_plugin::BasePlugin`

## Steps

1. Creating the Base: Create a package for base plugin class with only a single header file. Note that this package will also be using all the other plugins that you have defined. For now just create a base plugin class.

```bash
ros2 pkg create base_plugin --dependencies rclcpp pluginlib
```

2. Create the header file and fill it!

```cpp
#ifndef README_MD
#define README_MD

namespace base_plugin
{
  class BasePlugin
  {
  public:
    virtual void initialize(double length) = 0;
    virtual double area() = 0;
    virtual ~BasePlugin() {}
    void enabled() { enabled_ = true; };
    void disabled() { enabled_ = false; };
  protected:
      BasePlugin() {}
      bool enabled_;
  };

} // namespace base_plugin
#endif /* README_MD */
```

3. Update the CMakeLists.txt, pay attention to the `DO THIS` comment below.

```cmake
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

# INSTALL
# DO THIS
# Make the base_plugin header file available for other packages
install(
  DIRECTORY include/
  DESTINATION include
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

# DO THIS
ament_export_include_directories(include)
ament_package()
```

4. Create Plugin

```bash
ros2 pkg create feature_one_plugin --dependencies rclcpp pluginlib base_plugin
```

5. Fill up the plugin header file. Include base plugin header file in feature_one_plugin and use the `override` keyword, `feature_one_plugin.hpp`

```cpp
#ifndef FEATURE_ONE_PLUGIN_FEATURE_ONE_PLUGIN_HPP
#define FEATURE_ONE_PLUGIN_FEATURE_ONE_PLUGIN_HPP

#include "base_plugin/base_plugin.hpp"
// STL
#include <iostream>

namespace feature_one
{
  class FeatureOnePlugin : public base_plugin::BasePlugin
  {
  public:
    FeatureOnePlugin();
    ~FeatureOnePlugin();

    void initialize(double length) override;
    double area() override;

  private:
    double length_;
  };
} // namespace feature_one
#endif /* FEATURE_ONE_PLUGIN_FEATURE_ONE_PLUGIN_HPP */
```

6. Fill up the source file for the plugin `feature_one_plugin.cpp`.

7. Add `plugin.xml` file with details.
```xml
<library path="feature_one">
  <class type="feature_one::FeatureOnePlugin" base_class_type="base_plugin::BasePlugin">
    <description>This is feature one plugin.</description>
  </class>
</library>
```

8. Update the CMakeLists.txt, pay attention to the `DO THIS` comment below.

```cmake
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

# DO THIS
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
# DO THIS
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

# DO THIS
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

# DO THIS
ament_export_libraries(feature_one)
ament_export_targets(export_${PROJECT_NAME})
ament_package()

```

## Reference

- pluginlib and component in ROS 2 [link_ubuntu](https://ubuntu.com/blog/components-vs-plugins-in-ros-2)
