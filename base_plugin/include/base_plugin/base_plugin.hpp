#ifndef BASE_PLUGIN_BASE_PLUGIN_HPP
#define BASE_PLUGIN_BASE_PLUGIN_HPP

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

#endif /* BASE_PLUGIN_BASE_PLUGIN_HPP */
