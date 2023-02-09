#include "feature_one_plugin/feature_one_plugin.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(feature_one::FeatureOnePlugin, base_plugin::BasePlugin)

namespace feature_one
{
  FeatureOnePlugin::FeatureOnePlugin()
  {
  }

  FeatureOnePlugin::~FeatureOnePlugin()
  {
  }

  void FeatureOnePlugin::initialize(double length)
  {
    length_ = length;
  }

  double FeatureOnePlugin::area()
  {
    if (enabled_)
    {
      return 0.5 * length_;
    }
    else
    {
      std::cout << "\nThis plugin has been disabled\n";
      return 0.0;
    }
  }
} // namespace feature_one
