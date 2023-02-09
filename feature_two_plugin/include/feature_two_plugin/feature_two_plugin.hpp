#ifndef FEATURE_TWO_PLUGIN_FEATURE_TWO_PLUGIN_HPP
#define FEATURE_TWO_PLUGIN_FEATURE_TWO_PLUGIN_HPP

#include "base_plugin/base_plugin.hpp"
// STL
#include <iostream>

namespace feature_two
{
  class FeatureTwoPlugin : public base_plugin::BasePlugin
  {
  public:
    FeatureTwoPlugin();
    ~FeatureTwoPlugin();

    void initialize(double length) override;
    double area() override;

  private:
    double length_;
  };
} // namespace feature_two

#endif /* FEATURE_TWO_PLUGIN_FEATURE_TWO_PLUGIN_HPP */
