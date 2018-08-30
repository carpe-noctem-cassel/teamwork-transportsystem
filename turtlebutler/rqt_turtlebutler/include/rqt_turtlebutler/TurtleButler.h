#ifndef butler__TurtleButler_H
#define butler__TurtleButler_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_TurtleButler.h>
#include <QWidget>
#include "ros/ros.h"
#include <vector>
#include <map>
#include <geometry_msgs/PoseStamped.h>

namespace supplementary
{
  class SystemConfig;
}

namespace rqt_turtlebutler {
  class TurtleButler
    : public rqt_gui_cpp::Plugin
  {
    Q_OBJECT
  public:

    struct Poi {
      std::string id;
      double x;
      double y;
    };

    TurtleButler();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

  private slots:
    void sendData();
    void initComboBoxes();
    void updatePickupComboBox(QString);
    void updateDropoffComboBox(QString);
    
    //bool hasConfiguration() const;
    //void triggerConfiguration();
  private:
    void readConfig();
    Ui::TurtleButlerWidget ui_;
    QMainWindow* widget_;
    ros::NodeHandle n;
    std::map<std::string, ros::Publisher> turtleButler_publishers;
    std::map<std::string, std::vector<Poi>> rooms;
    std::vector<std::string> roomNames;
    std::vector<std::string> splitString(std::string input, std::string delimiter);
    geometry_msgs::PoseStamped getPositions(std::string input, std::string delimeter);
    supplementary::SystemConfig *sc;
  };
}
#endif
