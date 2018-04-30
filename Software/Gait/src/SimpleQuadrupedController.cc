
#include <cassert>
#include "../include/dogbot/SimpleQuadrupedController.hh"

namespace DogBotN {

  // Set the leg goal position
  void SimpleQuadrupedPoseC::SetLegPosition(int legId,float x,float y,float z)
  {
    int off = legId * 3;
    assert(off >= 0);
    assert(off < 12);

    m_position[off+0] = x;
    m_position[off+1] = y;
    m_position[off+2] = z;
  }

  // Set the leg goal position
  void SimpleQuadrupedPoseC::GetLegPosition(int legId,float &x,float &y,float &z)
  {
    int off = legId * 3;
    assert(off >= 0);
    assert(off < 12);

    x = m_position[off+0];
    y = m_position[off+1];
    z = m_position[off+2];
  }

  //! Dump pose.

  void SimpleQuadrupedPoseC::Dump(std::ostream &out)
  {
    out << m_position[0];
    for(int i = 1;i < 12;i++)
      out << " " << m_position[i];
  }

  // ------------------------------------------------------

  SimpleQuadrupedControllerC::SimpleQuadrupedControllerC()
  {}

  //! Virtual destructor
  SimpleQuadrupedControllerC::~SimpleQuadrupedControllerC()
  {}

  //! Do a single timestep
  bool SimpleQuadrupedControllerC::Step(float timeStep,SimpleQuadrupedPoseC &positions)
  {
    return true;
  }

}
