#ifndef butler__TurtleButler_H
#define butler__TurtleButler_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_TurtleButler.h>
#include <QWidget>
#include "ros/ros.h"

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

  private slots:
    void sendData();
    void initComboBoxes();

    //bool hasConfiguration() const;
    //void triggerConfiguration();
  private:
    Ui::TurtleButlerWidget ui_;
    QMainWindow* widget_;
    ros::NodeHandle n;
    ros::Publisher turtleButler_pub;

  };
}
#endif
