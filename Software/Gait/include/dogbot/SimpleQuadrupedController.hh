#ifndef DOGBOT_QUADRUPEDCONTROLLER_HEADER
#define DOGBOT_QUADRUPEDCONTROLLER_HEADER 1

#include <vector>
#include <iostream>

namespace DogBotN {

  // Cartesian positions for feet.
  //
  //  0-Front left  1-Front right
  //  2-Rear left   3-Read right

  class SimpleQuadrupedPoseC
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

  //! Simple Gate generator base class.

  class SimpleQuadrupedControllerC
  {
  public:
    SimpleQuadrupedControllerC();

    //! Virtual destructor
    virtual ~SimpleQuadrupedControllerC();

    //! Do a single timestep
    virtual bool Step(float timeStep,SimpleQuadrupedPoseC &pose);

  protected:

  };
}

#endif
