
#include "dogbot/SplineCatmullRom.hh"
#include <assert.h>
#include <math.h>

namespace DogBotN {

  SplinePoint3dC::SplinePoint3dC(float t,float x,float y,float z)
   : m_timeDelta(t),
     m_point(x,y,z)
  { }

  // -------------------------------------

  SplineCatmullRom3dC::SplineCatmullRom3dC()
  {}


  SplineCatmullRom3dC::SplineCatmullRom3dC(std::vector<SplinePoint3dC> &points)
  {
    Setup(points);
  }

  //! Setup control points
  void SplineCatmullRom3dC::Setup(std::vector<SplinePoint3dC> &points)
  {
    m_trajectory.clear();
    float t = 0;
    for(auto &a : points) {
      m_trajectory.emplace(t,a);
      t += a.m_timeDelta;
    }
    m_totalTime = t;
  }

  bool SplineCatmullRom3dC::Evaluate(float tp,Eigen::Vector3f &pnt) {
#if 1
    if(tp > m_totalTime)
      tp -= floor(tp/m_totalTime) * m_totalTime;
    if(tp < 0)
      tp += floor(tp/m_totalTime) * m_totalTime;
#endif

    auto it = m_trajectory.upper_bound(tp);
    float tOff = 0;
    // Go back 2 points.
    for(int i = 0;i < 2;i++) {
      if(it == m_trajectory.begin()) {
        it = m_trajectory.end();
        tOff -= m_totalTime;
      }
      --it;
    }
    // Pull out points and times
    float t[4];
    Eigen::Vector3f P[4];
    for(int i = 0;i < 4;i++) {
      t[i] = it->first + tOff;
      P[i] = it->second.m_point;
      ++it;
      if(it == m_trajectory.end()) {
        it = m_trajectory.begin();
        tOff += m_totalTime;
      }
    }
    assert(tp >= t[1]);
    assert(tp <= t[2]);
    //std::cout << "t=" << tp << " t0=" << t[0] << " t1=" << t[1] << " t2=" << t[2] << " t3=" << t[3] << "\n";

    Eigen::Vector3f A1 = P[0] * ((t[1]-tp)/(t[1]-t[0])) + P[1] * ((tp-t[0])/(t[1]-t[0]));
    Eigen::Vector3f A2 = P[1] * ((t[2]-tp)/(t[2]-t[1])) + P[2] * ((tp-t[1])/(t[2]-t[1]));
    Eigen::Vector3f A3 = P[2] * ((t[3]-tp)/(t[3]-t[2])) + P[3] * ((tp-t[2])/(t[3]-t[2]));
    Eigen::Vector3f B1 = A1 * ((t[2]-tp)/(t[2]-t[0])) + A2 * ((tp-t[0])/(t[2]-t[0]));
    Eigen::Vector3f B2 = A2 * ((t[3]-tp)/(t[3]-t[1])) + A3 * ((tp-t[1])/(t[3]-t[1]));
    pnt  = B1 * ((t[2]-tp)/(t[2]-t[1])) + B2 *((tp-t[1])/(t[2]-t[1]));

    return true;
  }

  // -------------------------------------

  SplineLinear3dC::SplineLinear3dC()
  {

  }

  //! Construct from a list of positions
  SplineLinear3dC::SplineLinear3dC(std::vector<SplinePoint3dC> &points)
  {
    Setup(points);
  }

  //! Setup control points
  void SplineLinear3dC::Setup(const std::vector<SplinePoint3dC> &points)
  {
    m_trajectory.clear();
    float t = 0;
    for(auto &a : points) {
      m_trajectory.emplace(t,a);
      t += a.m_timeDelta;
    }
    m_totalTime = t;
  }

  //! Evaluate position at time t
  bool SplineLinear3dC::Evaluate(float tp,Eigen::Vector3f &pnt)
  {
#if 1
    if(tp > m_totalTime)
      tp -= floor(tp/m_totalTime) * m_totalTime;
    if(tp < 0)
      tp -= floor(tp/m_totalTime) * m_totalTime;
#endif

    auto it = m_trajectory.upper_bound(tp);

    float tOff = 0;
    // Go back 2 points.
    if(it == m_trajectory.begin()) {
      it = m_trajectory.end();
      tOff -= m_totalTime;
    }
    --it;

    float t[2];
    Eigen::Vector3f P[2];
    for(int i = 0;i < 2;i++) {
      t[i] = it->first + tOff;
      P[i] = it->second.m_point;
      ++it;
      if(it == m_trajectory.end()) {
        it = m_trajectory.begin();
        tOff += m_totalTime;
      }
    }
    assert(tp >= t[0]);
    assert(tp <= t[1]);

    pnt = P[0] * ((t[1]-tp)/(t[1]-t[0])) + P[1] * ((tp-t[0])/(t[1]-t[0]));

    return true;
  }


}
