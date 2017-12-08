/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the DogBot
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include "dogbot_control/dogbot_hw_interface.h"

namespace dogbot_control
{

DogBotHWInterface::DogBotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  auto logger = spdlog::stdout_logger_mt("console");
  logger->info("Starting API");

  std::string devFilename = "local";

  m_dogBotAPI = std::make_shared<DogBotN::DogBotAPIC>(devFilename,logger,DogBotN::DogBotAPIC::DMM_ClientOnly);
  m_dogBotAPI->LoadConfig("/home/charles/src/active/BMC2-Firmware/Config/config.json");
  m_dogBotAPI->Init();

  m_actuators.empty();
  for(auto &name : joint_names_) {
    size_t at = name.rfind("_joint");
    std::string actname = name.substr(0,at);
    m_actuators.push_back(m_dogBotAPI->GetServoByName(actname));
    if(m_actuators.back()) {
      ROS_INFO_NAMED("dogbot_hw_interface", "Found hardware interface '%s' for joint '%s' ",actname.c_str(),name.c_str());
    } else {
      ROS_ERROR_NAMED("dogbot_hw_interface", "Failed to find hardware interface '%s' for joint '%s' ",actname.c_str(),name.c_str());
    }
  }
#if 0
  for(auto a : m_dogBotAPI->ListServos()) {
    if(a)
      ROS_INFO_NAMED("dogbot_hw_interface","Found hardware '%s' ",a->Name().c_str());
  }
#endif

  ROS_INFO_NAMED("dogbot_hw_interface", "DogBotHWInterface Ready.");
}

void DogBotHWInterface::read(ros::Duration &elapsed_time)
{
  DogBotN::ServoC::TimePointT theTime = DogBotN::ServoC::TimePointT::clock::now();
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id) {
    std::shared_ptr<DogBotN::JointC> &jnt = m_actuators[joint_id];
    if(!jnt) {
      continue;
    }
    jnt->GetStateAt(theTime,joint_position_[joint_id],joint_velocity_[joint_id],joint_effort_[joint_id]);
    //ROS_INFO_NAMED("dogbot_hw_interface", "Jnt '%s' position %f ",joint_names_[joint_id].c_str(),joint_position_[joint_id]);

  }
}

void DogBotHWInterface::write(ros::Duration &elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);

  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id) {
    std::shared_ptr<DogBotN::JointC> &jnt = m_actuators[joint_id];
    if(!jnt)
      continue;
    jnt->DemandPosition(joint_position_command_[joint_id],5.0);
  }
}

void DogBotHWInterface::enforceLimits(ros::Duration &period)
{
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
  // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
  // DEPENDING ON YOUR CONTROL METHOD
  //
  // EXAMPLES:
  //
  // Saturation Limits ---------------------------
  //
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces velocity and acceleration limits
  // vel_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

  // Soft limits ---------------------------------
  //
  // pos_jnt_soft_limits_.enforceLimits(period);
  // vel_jnt_soft_limits_.enforceLimits(period);
  // eff_jnt_soft_limits_.enforceLimits(period);
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

}  // namespace
