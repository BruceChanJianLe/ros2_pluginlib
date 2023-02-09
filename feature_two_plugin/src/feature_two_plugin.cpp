#include "feature_two_plugin/feature_two_plugin.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(feature_two::FeatureTwoPlugin, base_plugin::BasePlugin)

namespace feature_two
{
  FeatureTwoPlugin::FeatureTwoPlugin()
  {
  }

  FeatureTwoPlugin::~FeatureTwoPlugin()
  {
  }

  void FeatureTwoPlugin::initialize(double length)
  {
    length_ = length;
  }

  double FeatureTwoPlugin::area()
  {
    if (enabled_)
    {
      return 0.2 * length_;
    }
    else
    {
      std::cout << "\nThis plugin has been disabled\n";
      return 0.0;
    }
  }
} // namespace feature_two
