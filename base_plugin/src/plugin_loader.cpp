#include "base_plugin/plugin_loader.hpp"

namespace pluginlib
{
  PluginLoader::PluginLoader(const std::string& node_name)
  : rclcpp::Node{node_name}
  , plugin_loader_ptr_{std::make_unique<::pluginlib::ClassLoader<base_plugin::BasePlugin>>("base_plugin", "base_plugin::BasePlugin")}
  {
    this->declare_parameter<std::vector<std::string>>("plugins");
    this->get_parameter("plugins", plugin_names_);
    
    // Load available plugins
    for (const auto& name : plugin_names_)
    {
      // Declare plugin related parameters here
      this->declare_parameter<std::string>(name + ".plugin");
      this->declare_parameter<bool>(name + ".enabled");
      this->declare_parameter<double>(name + ".length");

      // Obtain plugin related parameter here
      std::string type; bool enabled; double length;
      this->get_parameter<std::string>(name + ".plugin", type);
      this->get_parameter<bool>(name + ".enabled", enabled);
      this->get_parameter<double>(name + ".length", length);

      // Load plugin
      try
      {
        plugins_.insert({name, plugin_loader_ptr_->createSharedInstance(type)});
      }
      catch (pluginlib::PluginlibException &ex)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "The plugin (" << name << ") failed to load for some reason: " << ex.what());
      }

      auto itr = plugins_.find(name);
      if (itr != plugins_.end())
      {
        itr->second->enabled();
        // Initialize the plugin
        itr->second->initialize(length);
      }
    }
  }

  PluginLoader::~PluginLoader()
  {
    // Good practice
    this->undeclare_parameter("plugins");
    for (const auto& name : plugin_names_)
    {
      // Undeclare plugin related parameters here
      this->undeclare_parameter(name + ".plugin");
      this->undeclare_parameter(name + ".enabled");
    }
  }

  void PluginLoader::start()
  {
    // Run all loaded plugins
    for (auto& plugin : plugins_)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Plugin (" << plugin.first << ") has area : " << plugin.second->area());
    }
  }

} // namespace pluginlib