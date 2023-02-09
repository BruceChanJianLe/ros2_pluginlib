#include "base_plugin/plugin_loader.hpp"
// ROS2
#include "rclcpp/rclcpp.hpp"
// STL
#include <string>


constexpr auto node_name = "plugin_loader_node";

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pluginlib::PluginLoader>(node_name);
  // Start Node
  node->start();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}