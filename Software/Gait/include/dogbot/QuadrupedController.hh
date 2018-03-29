#ifndef DOGBOT_QUADRUPEDCONTROLLER_HEADER
#define DOGBOT_QUADRUPEDCONTROLLER_HEADER 1

#include <vector>
#include <iostream>

namespace DogBotN {

  // Position vector.
  //
  //  0- Hip
  //  1- Leg
  //  2- Knee
  //
  //  0-Front left  1-Front right
  //  2-Rear left   3-Read right

  class QuadrupedPoseC
  {
  public:
    // Set the leg goal position
    void SetLegPosition(int legId,float x,float y,float z);

    // Set the leg goal position
    void GetLegPosition(int legId,float &x,float &y,float &z);

    //! Dump pose
    void Dump(std::ostream &out);

    float m_position[12];
  };

  //! Gate generator base class.

  class QuadrupedControllerC
  {
  public:
    QuadrupedControllerC();

    //! Virtual destructor
    virtual ~QuadrupedControllerC();

    //! Do a single timestep
    virtual bool Step(float timeStep,QuadrupedPoseC &pose);

  protected:

  };
}

#endif
