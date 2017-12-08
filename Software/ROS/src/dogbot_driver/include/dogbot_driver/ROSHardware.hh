#ifndef DOGBOT_DOGBOTHARDWARE_HEADER
#define DOGBOT_DOGBOTHARDWARE_HEADER 1

#include <transmission_interface/transmission_interface.h>
#include <dogbot/DogBotAPI.hh>

namespace DogBotN {

  //! Data about a joint

  class ROSJointC
  {
  public:
    ROSJointC();

    ROSJointC(const std::shared_ptr<DogBotN::ServoC> &servo);

    //! Set servo to use
    void SetServo(const std::shared_ptr<DogBotN::ServoC> &servo)
    { m_servo = servo; }

    void SetTransmition(const std::shared_ptr<transmission_interface::Transmission> &transmission)
    { m_transmission = transmission; }

    //! Update data from robot
    bool UpdateState();

    //! Access transmission.
    transmission_interface::Transmission *Transmission()
    { return m_transmission.get(); }

    transmission_interface::ActuatorData &ActuatorStateData()
    { return m_actuatorStateData; }

    transmission_interface::ActuatorData &ActuatorCommandData()
    { return m_actuatorCommandData; }

    transmission_interface::JointData &JointStateData()
    { return m_jointStateData; }

    transmission_interface::JointData &JointCommandData()
    { return m_actuatorCommandData; }

  protected:
    void Init();

    std::shared_ptr<transmission_interface::Transmission> m_transmission;
    std::shared_ptr<DogBotN::ServoC> m_servo;

    double m_actuatorPosition = 0;
    double m_actuatorVelocity = 0;
    double m_actuatorEffort = 0;
    double m_actuatorCommandPosition = 0;
    double m_actuatorCommandEffort = 0;

    double m_jointPosition = 0;
    double m_jointVelocity = 0;
    double m_jointEffort = 0;
    double m_jointCommandPosition = 0;
    double m_jointCommandEffort = 0;

    transmission_interface::ActuatorData m_actuatorStateData;
    transmission_interface::ActuatorData m_actuatorCommandData;
    transmission_interface::JointData m_jointStateData;
    transmission_interface::JointData m_jointCommandData;

  };




  //! ROS Controller interface
  class ROSHardwareC
  {
  public:
    ROSHardwareC(const std::shared_ptr<DogBotN::DogBotAPIC> &dogBotAPI);

    //! Read and publish actuator position
    void read();

    //! Write demands
    void write();

  protected:
    void Init();

    std::shared_ptr<DogBotN::DogBotAPIC> m_dogBotAPI;
    std::vector<ROSJointC> m_joints;

    // Transmission interfaces
    transmission_interface::ActuatorToJointStateInterface    m_actuatorToJointState; // For propagating current actuator state to joint space
    transmission_interface::JointToActuatorPositionInterface m_jointToActuatorPosition;   // For propagating joint position commands to actuator space

  };

}


#endif
