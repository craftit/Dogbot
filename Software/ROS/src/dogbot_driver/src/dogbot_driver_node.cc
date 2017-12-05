

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <sstream>

#include <thread>
#include <vector>
#include <functional>
#include <assert.h>
#include <mutex>
#include "dogbot/DogBotAPI.hh"


/**
 * Driver node for dogbot
 */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dogbot_driver_node");

  ros::NodeHandle n;
  ros::NodeHandle nPrivate("~");

  std::string configFilename = "";
  std::string devFilename = "local";

  nPrivate.getParam("configFilename",configFilename);
  nPrivate.getParam("devFilename",devFilename);

  ROS_INFO("Using device '%s' ",devFilename.c_str());
  ROS_INFO("Using config '%s' ",configFilename.c_str());

  auto logger = spdlog::stdout_logger_mt("console");
  logger->info("Starting API");

  std::shared_ptr<DogBotN::DogBotAPIC> dogBotAPI = std::make_shared<DogBotN::DogBotAPIC>(devFilename,logger,DogBotN::DogBotAPIC::DMM_ClientOnly);

#if 1
  // Connect and start things going.
  dogBotAPI->Init(configFilename);

  std::vector<std::string> jointNames;
  std::vector<std::shared_ptr<DogBotN::ServoC> > jointServos;

  for(int i = 0;i < 2;i++) {
    std::string prefix1 = i == 0 ? "Front" : "Back";
    for(int j = 0;j < 2;j++) {
      std::string prefix2 = prefix1 + "-" + ((j == 0) ? "Left" : "Right") + "-";
      for(int k = 0;k < 3;k++) {
        std::string jntName = prefix2;
        switch(k) {
          case 0: jntName += "Roll"; break;
          case 1: jntName += "Pitch"; break;
          case 2: jntName += "Knee"; break;
        }
        jointNames.push_back(jntName);
        ROS_INFO("Configuring servo %s ",jntName.c_str());
        jointServos.push_back(dogBotAPI->GetServoByName(jntName));
        if(!jointServos.back()) {
          ROS_WARN("Failed to find servo %s ",jntName.c_str());
        }
      }
    }
  }

  ros::Publisher pubJointState = n.advertise<sensor_msgs::JointState>("joint_states", 1);

  ros::Timer statusTimer = n.createTimer(ros::Duration(0.01),[&pubJointState,&dogBotAPI,&jointNames,&jointServos] (const ros::TimerEvent& event) mutable {
    sensor_msgs::JointState jointState;
    for(int i = 0;i < jointServos.size();i++) {
      jointState.name.push_back(jointNames[i]);
      DogBotN::ServoC::TimePointT theTime = DogBotN::ServoC::TimePointT::clock::now();

      float position = 0;
      float velocity = 0;
      float torque = 0;
      if(jointServos[i])
        jointServos[i]->GetStateAt(theTime,position,velocity,torque);
      jointState.position.push_back(position);
      jointState.velocity.push_back(velocity);
      jointState.effort.push_back(torque);
    }
  });
#endif
  ROS_INFO("Setup complete ");
  ros::spin();

  return 0;
}
