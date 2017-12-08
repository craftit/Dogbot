
#include "dogbot_driver/ROSHardware.hh"
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/four_bar_linkage_transmission.h>

namespace DogBotN {

  ROSJointC::ROSJointC()
  {
    Init();
  }

  ROSJointC::ROSJointC(const std::shared_ptr<DogBotN::ServoC> &servo)
   : m_servo(servo)
  {
    Init();
  }


  void ROSJointC::Init()
  {
    m_actuatorStateData.position.push_back(&m_actuatorPosition);
    m_actuatorStateData.velocity.push_back(&m_actuatorVelocity);
    m_actuatorStateData.effort.push_back(&m_actuatorEffort);


    m_actuatorCommandData.position.push_back(&m_actuatorCommandPosition);
    m_actuatorCommandData.effort.push_back(&m_actuatorCommandEffort);

    m_jointStateData.position.push_back(&m_jointPosition);
    m_jointStateData.velocity.push_back(&m_jointVelocity);
    m_jointStateData.effort.push_back(&m_jointEffort);

    m_jointCommandData.position.push_back(&m_jointCommandPosition);
    m_jointCommandData.effort.push_back(&m_jointCommandEffort);

  }

  //! Update data from robot
  bool ROSJointC::read(const ros::Time &time)
  {
    auto timeNow = std::chrono::steady_clock::now();
    m_servo->GetStateAt(timeNow,m_actuatorPosition,m_actuatorVelocity,m_actuatorEffort);
    return true;
  }

  // =======================================================================

  ROSHardwareC::ROSHardwareC(const std::shared_ptr<DogBotN::DogBotAPIC> &dogBotAPI)
   : m_dogBotAPI(dogBotAPI)
  {
    Init();
  }

  void ROSHardwareC::Init()
  {
    std::vector<std::string> jointNames;

    //transmission_interface::ActuatorData m_actuator_data;
    //transmission_interface::JointData m_joint_data;

    // transmission_interface::ActuatorToJointStateInterface    m_actuatorToJointState; // For propagating current actuator state to joint space
    // transmission_interface::JointToActuatorPositionInterface m_jointToActuatorPosition;   // For propagating joint position commands to actuator space

    for(int i = 0;i < 2;i++) {
      std::string prefix1 = i == 0 ? "Front" : "Back";
      for(int j = 0;j < 2;j++) {
        std::string prefix2 = prefix1 + "-" + ((j == 0) ? "Left" : "Right") + "-";


        {
          std::string jntName = prefix2 + "Roll";

          m_joints.push_back(ROSJointC(m_dogBotAPI->GetServoByName(jntName)));
          ROSJointC &joint = m_joints.back();

          joint.SetTransmition(std::make_shared<transmission_interface::SimpleTransmission>(1.0));

          m_actuatorToJointState.registerHandle(transmission_interface::ActuatorToJointStateHandle(jntName,joint.Transmission(),joint.ActuatorStateData(),joint.JointStateData()));
          m_jointToActuatorPosition.registerHandle(transmission_interface::JointToActuatorPositionHandle(jntName,joint.Transmission(),joint.JointCommandData(),joint.ActuatorCommandData()));
        }
        {
          std::string jntName = prefix2 + "Pitch";

          m_joints.push_back(ROSJointC(m_dogBotAPI->GetServoByName(jntName)));
          ROSJointC &joint = m_joints.back();

          //joint.SetTransmition(std::make_shared<transmission_interface::SimpleTransmission>(1.0));

          m_actuatorToJointState.registerHandle(transmission_interface::ActuatorToJointStateHandle(jntName,joint.Transmission(),joint.ActuatorStateData(),joint.JointStateData()));
          m_jointToActuatorPosition.registerHandle(transmission_interface::JointToActuatorPositionHandle(jntName,joint.Transmission(),joint.JointCommandData(),joint.ActuatorCommandData()));
        }
        {
          std::string jntName = prefix2 + "Knee";

          m_joints.push_back(ROSJointC(m_dogBotAPI->GetServoByName(jntName)));
          ROSJointC &joint = m_joints.back();
          // http://docs.ros.org/kinetic/api/transmission_interface/html/c++/namespacetransmission__interface.html

          joint.SetTransmition(std::make_shared<transmission_interface::FourBarLinkageTransmission>(1.0));

          m_actuatorToJointState.registerHandle(transmission_interface::ActuatorToJointStateHandle(jntName,joint.Transmission(),joint.ActuatorStateData(),joint.JointStateData()));
          m_jointToActuatorPosition.registerHandle(transmission_interface::JointToActuatorPositionHandle(jntName,joint.Transmission(),joint.JointCommandData(),joint.ActuatorCommandData()));
        }

      }
    }

  }


  //! Read and publish actuator position
  void ROSHardwareC::read()
  {

  }

  //! Write demands
  void ROSHardwareC::write()
  {

  }


}
