
#include "dogbot/QuadrupedController.hh"
#include <cassert>

namespace DogBotN {

  // Set the leg goal position
  void QuadrupedPoseC::SetLegPosition(int legId,float hip,float leg,float knee)
  {
    int off = legId * 3;
    assert(off >= 0);
    assert(off < 12);

    m_position[off+0] = hip;
    m_position[off+1] = leg;
    m_position[off+2] = knee;
  }

  // Set the leg goal position
  void QuadrupedPoseC::GetLegPosition(int legId,float &x,float &y,float &z)
  {
    int off = legId * 3;
    assert(off >= 0);
    assert(off < 12);

    x = m_position[off+0];
    y = m_position[off+1];
    z = m_position[off+2];
  }

  //! Dump pose.

  void QuadrupedPoseC::Dump(std::ostream &out)
  {
    out << m_position[0];
    for(int i = 1;i < 12;i++)
      out << " " << m_position[i];
  }

  // ------------------------------------------------------

  QuadrupedControllerC::QuadrupedControllerC()
  {}

  //! Virtual destructor
  QuadrupedControllerC::~QuadrupedControllerC()
  {}

  //! Do a single timestep
  bool QuadrupedControllerC::Step(float timeStep,QuadrupedPoseC &positions)
  {
    return true;
  }

}
