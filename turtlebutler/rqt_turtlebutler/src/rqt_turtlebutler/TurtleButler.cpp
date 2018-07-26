#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QString>

#include "rqt_turtlebutler/TurtleButler.h"
#include <iostream>
#include <fstream>
#include <ros/package.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <vector>


namespace rqt_turtlebutler {
  TurtleButler::TurtleButler()
    : rqt_gui_cpp::Plugin()
    , widget_(0)
  {
    setObjectName("TurtleButler");
  }
  
  void TurtleButler::initPlugin(qt_gui_cpp::PluginContext& context)
  {
    //ros::init(context.argv().length, context.argv(), "rqt_turtlebutler");

    turtleButler_pub = n.advertise<std_msgs::String>("turtlebutler", 1000);
    QStringList argv = context.argv();
    widget_ = new QMainWindow();
    ui_.setupUi(widget_);
    context.addWidget(widget_);
    
    initComboBoxes();
    
    connect(ui_.sendButton, SIGNAL (released()), this, SLOT (sendData()));
  }

  void TurtleButler::initComboBoxes()
  {
    // load turtlebot names and ids from file
    std::string botsPath = ros::package::getPath("rqt_turtlebutler") + "/config/turtlebots.txt";
    std::ifstream botsFile (botsPath.c_str());
    std::string value;
    if(botsFile.is_open())
    {
      bool readPositions = false;
      while(getline(botsFile, value))
      {
        if(value.find("-") != std::string::npos)
        {
          readPositions = true;
          continue;
        }
        if(readPositions)
        {
          std::vector<std::string> data = splitString(value, ";");
          std::string position = data.at(1).append(data.at(2));
          ui_.pickup_comboBox->addItem(data.at(0).c_str(), position.c_str());
          ui_.dropoff_comboBox->addItem(data.at(0).c_str(), position.c_str());
        }
        else
        {
          std::vector<std::string> data = splitString(value, ";");
          ui_.turtlebot_comboBox->addItem(data.at(0).c_str(), data.at(1).c_str());
        }
      }
    }
    botsFile.close();
  }

  std::vector<std::string> TurtleButler::splitString(std::string input, std::string delimeter)
  {
    std::vector<std::string> result;

    while(input.find(delimeter, 0) != std::string::npos)
    {
      int index = -1;
      index = input.find(delimeter, 0);
      result.push_back(input.substr(0, index));
      input.erase(0, index + delimeter.length());
    }
    if(input.length() > 0)
    {
      result.push_back(input);
    }
    return result;
  }

  void TurtleButler::sendData()
  {
    std_msgs::String msg;
    std::string data = ui_.turtlebot_comboBox->currentData().toString().toStdString();
    data.append(ui_.pickup_comboBox->currentData().toString().toStdString());
    data.append(ui_.dropoff_comboBox->currentData().toString().toStdString());
    msg.data = data;
    // TODO: add data on locations and item
    turtleButler_pub.publish(msg);
//    std::cout << ui_.turtlebot_comboBox->currentData().toString().toStdString();
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
PLUGINLIB_EXPORT_CLASS(rqt_turtlebutler::TurtleButler, rqt_gui_cpp::Plugin)
