#ifndef butler__TurtleButler_H
#define butler__TurtleButler_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_TurtleButler.h>
#include <QWidget>

namespace rqt_turtlebutler {
  class TurtleButler
    : public rqt_gui_cpp::Plugin
  {
    Q_OBJECT
  public:
    TurtleButler();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

    //bool hasConfiguration() const;
    //void triggerConfiguration();
  private:
    Ui::TurtleButlerWidget ui_;
    QMainWindow* widget_;
  };
}
#endif
