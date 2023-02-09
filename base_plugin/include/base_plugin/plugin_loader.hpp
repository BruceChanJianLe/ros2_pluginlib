#ifndef BASE_PLUGIN_PLUGIN_LOADER_HPP
#define BASE_PLUGIN_PLUGIN_LOADER_HPP

#include "base_plugin/plugin_loader.hpp"
#include "base_plugin/base_plugin.hpp"
// ROS2
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
// STL
#include <memory>
#include <unordered_map>
#include <string>
#include <vector>

namespace pluginlib
{
  class PluginLoader : public rclcpp::Node
  {
  public:
    PluginLoader(const std::string& node_name);
    ~PluginLoader();

    void start();

  private:
    std::unique_ptr<::pluginlib::ClassLoader<base_plugin::BasePlugin>> plugin_loader_ptr_;
    std::unordered_map<std::string, std::shared_ptr<base_plugin::BasePlugin>> plugins_;
    std::vector<std::string> plugin_names_;
  };

} // namespace pluginlib

#endif /* BASE_PLUGIN_PLUGIN_LOADER_HPP */
