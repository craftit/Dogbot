#ifndef DOGBOT_ROSDOGBOTHARDWAREINTERFACE_HEADER
#define DOGBOT_ROSDOGBOTHARDWAREINTERFACE_HEADER 1


#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <transmission_interface/effort_joint_interface_provider.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

#include "hardware_interface/actuator_command_interface.h"
#include "hardware_interface/actuator_state_interface.h"

#include <dogbot/DogBotAPI.hh>


namespace DogBotN {

  //! Data about a joint

  class ROSActuatorC
  {
  public:
    ROSActuatorC();

    ROSActuatorC(const std::shared_ptr<DogBotN::ServoC> &servo);

    //! Set servo to use
    void SetServo(const std::shared_ptr<DogBotN::ServoC> &servo)
    { m_servo = servo; }

    //! Update data from robot
    bool read(const ros::Time &time);

#if 0
    hardware_interface::ActuatorStateInterface &ActuatorStateData()
    { return m_actuatorStateData; }

    hardware_interface::ActuatorCommandInterface &ActuatorCommandData()
    { return m_actuatorCommandData; }
#endif

  protected:
    void Init();

    std::shared_ptr<DogBotN::ServoC> m_servo;

    double m_actuatorPosition = 0;
    double m_actuatorVelocity = 0;
    double m_actuatorEffort = 0;
    double m_actuatorCommandPosition = 0;
    double m_actuatorCommandEffort = 0;

    //hardware_interface::ActuatorStateInterface m_actuatorStateData;
    //hardware_interface::ActuatorCommandInterface m_actuatorCommandData;

  };

  //! ROS hardware interface.

  class ROSHardwareInterface
      : public hardware_interface::RobotHW
  {
  public:
    ROSHardwareInterface(ros::NodeHandle& nh,const std::shared_ptr<DogBotN::DogBotAPIC> &dogBotAPI);

    /// \brief Initialize the hardware interface
    virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);

    /// \brief Read the state from the robot hardware.
    virtual void read(const ros::Time &         time,
                      const ros::Duration &   period
                      ) override;

    /// \brief write the command to the robot hardware.
    virtual void write(const ros::Time &         time,
                       const ros::Duration &   period
                       ) override;

    /**
     * Check (in non-realtime) if given controllers could be started and stopped from the current state of the RobotHW
     * with regard to necessary hardware interface switches and prepare the switching. Start and stop list are disjoint.
     * This handles the check and preparation, the actual switch is commited in doSwitch()
     */
    virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                               const std::list<hardware_interface::ControllerInfo>& stop_list) override;

    /**
     * Perform (in realtime) all necessary hardware interface switches in order to start and stop the given controllers.
     * Start and stop list are disjoint. The feasability was checked in prepareSwitch() beforehand.
     */
    virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& /*start_list*/,
                          const std::list<hardware_interface::ControllerInfo>& /*stop_list*/) override;


  protected:
    ros::NodeHandle m_nh;

    //hardware_interface::JointStateInterface joint_state_interface_;
    //hardware_interface::PositionJointInterface position_joint_interface_;

    std::shared_ptr<DogBotN::DogBotAPIC> m_dogBotAPI;

    transmission_interface::RobotTransmissions m_robot_transmissions;
    std::shared_ptr<transmission_interface::TransmissionInterfaceLoader> m_transmission_loader;
    std::vector<ROSActuatorC> m_actuators;

  };

}


#endif
