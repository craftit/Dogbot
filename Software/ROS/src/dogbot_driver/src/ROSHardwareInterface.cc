
#include "dogbot_driver/ROSHardwareInterface.hh"


namespace DogBotN {


  ROSActuatorC::ROSActuatorC()
  {
    Init();
  }

  ROSActuatorC::ROSActuatorC(const std::shared_ptr<DogBotN::ServoC> &servo)
   : m_servo(servo)
  {
    Init();
  }


  void ROSActuatorC::Init()
  {
#if 0
    m_actuatorStateData.position.push_back(&m_actuatorPosition);
    m_actuatorStateData.velocity.push_back(&m_actuatorVelocity);
    m_actuatorStateData.effort.push_back(&m_actuatorEffort);

    m_actuatorCommandData.position.push_back(&m_actuatorCommandPosition);
    m_actuatorCommandData.effort.push_back(&m_actuatorCommandEffort);
#endif
  }


  //! Update data from robot
  bool ROSActuatorC::read(const ros::Time &time)
  {
    auto timeNow = std::chrono::steady_clock::now();
    m_servo->GetStateAt(timeNow,m_actuatorPosition,m_actuatorVelocity,m_actuatorEffort);

    return true;
  }


  // ========================================================================

  ROSHardwareInterface::ROSHardwareInterface(ros::NodeHandle& nh,const std::shared_ptr<DogBotN::DogBotAPIC> &dogBotAPI)
  {
    init(nh,nh);
  }

  /// \brief Initialize the hardware interface

  bool ROSHardwareInterface::init(
      ros::NodeHandle& root_nh,
      ros::NodeHandle &robot_hw_nh
      )
  {
    m_actuators.reserve(12);

    // Populate robot_hw with actuator interfaces (e.g., EffortActuatorInterface)

    for(int i = 0;i < 2;i++) {
      std::string prefix1 = i == 0 ? "Front" : "Back";
      for(int j = 0;j < 2;j++) {
        std::string prefix2 = prefix1 + "-" + ((j == 0) ? "Left" : "Right") + "-";

#if 0
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
#endif
      }
    }

    //hardware_interface::EffortActuatorInterface



    // This is hardware-specific, and not detailed here
    // ...
#if 0
    // Initialise transmission loader
    try
    {
     m_transmission_loader.reset(new transmission_interface::TransmissionInterfaceLoader(this, &m_robot_transmissions));
    }
    catch(const std::invalid_argument& ex)
    {
     ROS_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
     return false;
    }
    catch(const pluginlib::LibraryLoadException& ex)
    {
     ROS_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
     return false;
    }
    catch(...)
    {
     ROS_ERROR_STREAM("Failed to create transmission interface loader. ");
     return false;
    }

    std::string robot_description;
    // ...load URDF from parameter server or file

    // Perform actual transmission loading
    if (!m_transmission_loader->load(robot_description)) {return false;}
#endif
    // We can now query for any of the created interfaces, for example:
    // robot_transmissions_.get<ActuatorToJointStateInterface>();
    // this->get<JointStateInterface>();

    return true;
  }

  /// \brief Read the state from the robot hardware.

  void ROSHardwareInterface::read(
      const ros::Time &         time,
      const ros::Duration &   period
      )
  {

  }

  /// \brief write the command to the robot hardware.

  void ROSHardwareInterface::write(
      const ros::Time &         time,
      const ros::Duration &   period
      )
  {

  }

  /**
   * Check (in non-realtime) if given controllers could be started and stopped from the current state of the RobotHW
   * with regard to necessary hardware interface switches and prepare the switching. Start and stop list are disjoint.
   * This handles the check and preparation, the actual switch is commited in doSwitch()
   */
  bool ROSHardwareInterface::prepareSwitch(
      const std::list<hardware_interface::ControllerInfo>& start_list,
      const std::list<hardware_interface::ControllerInfo>& stop_list
      )
  {
    return true;
  }

  /**
   * Perform (in realtime) all necessary hardware interface switches in order to start and stop the given controllers.
   * Start and stop list are disjoint. The feasability was checked in prepareSwitch() beforehand.
   */
  void ROSHardwareInterface::doSwitch(
      const std::list<hardware_interface::ControllerInfo>& /*start_list*/,
      const std::list<hardware_interface::ControllerInfo>& /*stop_list*/
      )
  {

  }

}
