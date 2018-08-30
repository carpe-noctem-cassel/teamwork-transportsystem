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
#include <string>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <SystemConfig.h>


namespace rqt_turtlebutler {

  const std::string LEONARDO = "Leonardo";
  const std::string RAPHAEL = "Raphael";
  const std::string DONATELLO = "Donatello";
  const std::string LEONARDO_GOAL = "/leonardo/move_base_simple/goal";
  const std::string RAPHAEL_GOAL = "/raphael/move_base_simple/goal";
  const std::string DONATELLO_GOAL = "/donatello/move_base_simple/goal";

  void TurtleButler::readConfig() {
    
    // Read rooms with its connected areas and pois from config
    sc->setConfigPath(ros::package::getPath("rqt_turtlebutler") + "/config");
    auto allRoomNames = (*sc)["TopologicalModel"]->getSections("DistributedSystems.Rooms", NULL);
    for (auto &roomName : *allRoomNames)
    {
      auto pois = (*sc)["TopologicalModel"]->getSections("DistributedSystems.Rooms", roomName.c_str(), "POIs");
      std::vector<Poi> myPois;
      for(auto &poiName: *pois)
      {
        Poi p;
        auto x = (*sc)["TopologicalModel"]->get<double>("DistributedSystems.Rooms", roomName.c_str(), "POIs", poiName.c_str(), "X", NULL);
        auto y = (*sc)["TopologicalModel"]->get<double>("DistributedSystems.Rooms", roomName.c_str(), "POIs", poiName.c_str(), "Y", NULL);
        p.id = poiName;
        p.x = x;
        p.y = y;
        myPois.push_back(p);
      }
      rooms[roomName] = myPois;
      roomNames.push_back(roomName);
    }
  }

  TurtleButler::TurtleButler()
    : rqt_gui_cpp::Plugin()
    , widget_(0)
  {
    setObjectName("TurtleButler");
  }
  
  void checkTurtlebotMessage(const std_msgs::String::ConstPtr& msg)
  {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
  }

  void TurtleButler::initPlugin(qt_gui_cpp::PluginContext& context)
  {
    std::setlocale(LC_NUMERIC, "en_US.UTF-8");
    //ros::init(context.argv().length, context.argv(), "rqt_turtlebutler");
    ROS_INFO("started");
    readConfig();
    QStringList argv = context.argv();
    widget_ = new QMainWindow();
    ui_.setupUi(widget_);
    context.addWidget(widget_);

    ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>(RAPHAEL_GOAL.c_str(), 1000);
    turtleButler_publishers[RAPHAEL_GOAL.c_str()] = pub;
    pub = n.advertise<geometry_msgs::PoseStamped>(DONATELLO_GOAL.c_str(), 1000);
    turtleButler_publishers[DONATELLO_GOAL.c_str()] = pub;
    pub = n.advertise<geometry_msgs::PoseStamped>(LEONARDO_GOAL.c_str(), 1000);
    turtleButler_publishers[LEONARDO_GOAL.c_str()] = pub;
    
    connect(ui_.sendButton, SIGNAL (released()), this, SLOT (sendData()));
    connect(ui_.pickup_comboBox, SIGNAL (currentIndexChanged(QString)), this, SLOT (updatePickupComboBox(QString)));
    connect(ui_.dropoff_comboBox, SIGNAL (currentIndexChanged(QString)), this, SLOT (updateDropoffComboBox(QString)));

    initComboBoxes();
  }

  void TurtleButler::initComboBoxes()
  {
    // load turtlebot names and ids from file
    ui_.turtlebot_comboBox->addItem(RAPHAEL.c_str(), RAPHAEL_GOAL.c_str());
    ui_.turtlebot_comboBox->addItem(LEONARDO.c_str(), LEONARDO_GOAL.c_str());
    ui_.turtlebot_comboBox->addItem(DONATELLO.c_str(), DONATELLO_GOAL.c_str());

    for(std::string room : roomNames)
    {
      ui_.pickup_comboBox->addItem(room.c_str(), room.c_str());
      ui_.dropoff_comboBox->addItem(room.c_str(), room.c_str());
    }
    ui_.pickup_comboBox->setCurrentIndex(0);
    ui_.dropoff_comboBox->setCurrentIndex(0);
  }

  void TurtleButler::updatePickupComboBox(QString roomName)
  {
    std::vector<Poi> currentPois = rooms[roomName.toStdString()];
    ui_.pickup_position_comboBox->clear();
    for(Poi poi : currentPois)
    {
      std::string position = "" + std::to_string(poi.x) + " " + std::to_string(poi.y);
      ui_.pickup_position_comboBox->addItem(poi.id.c_str(), position.c_str());
    }
  }

    void TurtleButler::updateDropoffComboBox(QString roomName)
  {
    std::vector<Poi> currentPois = rooms[roomName.toStdString()];
    ui_.dropoff_position_comboBox->clear();
    for(Poi poi : currentPois)
    {
      std::string position = "" + std::to_string(poi.x) + " " + std::to_string(poi.y);
      ui_.dropoff_position_comboBox->addItem(poi.id.c_str(), position.c_str());
    }
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
    //std_msgs::String msg;
    std::string butler_name = ui_.turtlebot_comboBox->currentData().toString().toStdString();
    std::string pickupPoint = ui_.pickup_position_comboBox->currentData().toString().toStdString();
    std::string dropoffPoint = ui_.dropoff_position_comboBox->currentData().toString().toStdString();


   	geometry_msgs::PoseStamped pose;

   	pose.header.frame_id = "/map";
   	pose.header.stamp = ros::Time::now();
    std::string::size_type offset;
    ROS_INFO("%s", pickupPoint.c_str());
    // ROS_INFO ("y: %d", std::stod (pickupPoint.substr(offset)));

   	pose.pose.position.x = std::stod (pickupPoint, &offset);
   	pose.pose.position.y = std::stod (pickupPoint.substr(offset));

   	tf::Quaternion quat = tf::Quaternion(0,0,1,0);
    
   	tf::quaternionTFToMsg(quat, pose.pose.orientation);
    //msg.data = "Test";
    if (turtleButler_publishers.find(butler_name) != turtleButler_publishers.end())
    {
      turtleButler_publishers[butler_name].publish(pose);
    }
    else
    {
      ROS_ERROR("Could not find robot!");
    }
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
