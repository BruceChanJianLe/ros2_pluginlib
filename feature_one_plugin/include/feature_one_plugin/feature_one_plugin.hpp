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
