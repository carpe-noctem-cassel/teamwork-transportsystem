#include <pluginlib/class_list_macros.h>
#include <QStringList>

#include "rqt_turtlebutler/TurtleButler.h"

namespace rqt_turtlebutler {
  TurtleButler::TurtleButler()
    : rqt_gui_cpp::Plugin()
    , widget_(0)
  {
    setObjectName("TurtleButler");
  }
  
  void TurtleButler::initPlugin(qt_gui_cpp::PluginContext& context)
  {
    QStringList argv = context.argv();
    widget_ = new QMainWindow();
    ui_.setupUi(widget_);
    context.addWidget(widget_);
  }
  
  void TurtleButler::shutdownPlugin()
  {
    // TODO unregister all publishers here
  }
  
  void TurtleButler::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
  {
    // TODO save intrinsic configurations, usually using:
    // v = instance_settings.value(k);
  }
  
  void TurtleButler::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
  {
    // TODO restore intrinsic configuration, usually using:
    // v = instance_settings.value(k);
  }
  
  /*
  bool hasConfiguration() const
  {
    return true;
  }
  
  void triggerConfiguration()
  {
    // Usually used to open a dialog to offer the user a set of configuration
  }
  */
}
PLUGINLIB_DECLARE_CLASS(butler, TurtleButler, rqt_turtlebutler::TurtleButler, rqt_gui_cpp::Plugin)
