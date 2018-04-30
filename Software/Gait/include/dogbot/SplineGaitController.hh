#ifndef DOGBOT_SPLINEGATECONTROLLER_HEADER
#define DOGBOT_SPLINEGATECONTROLLER_HEADER 1

#include "dogbot/SimpleQuadrupedController.hh"
#include "dogbot/SplineCatmullRom.hh"
#include "dogbot/LegKinematics.hh"

namespace DogBotN {

  //! Foot trajectory parameters.
  // From 'Locomotion Control for Electrically Powered Quadruped Robot Dynarobin'
  // by Edin Koco

  class FootTrajectoryC
  {
  public:
    FootTrajectoryC();

    //! Setup trajectory
    FootTrajectoryC(float Zc,float Xc,float Lpr,float tpr,float Had,float tad,float tpu);

    std::vector<SplinePoint3dC> GenerateTrajectory(float rz,float xoff,float zoff) const;

    float m_Zc = 0.01;  // z centre
    float m_Xc = -0.06;  // x centre
    float m_Lpr = 0.15; // Length propel
    float m_tpu = 0.1;  // Time push segment

    float m_Had = 0.2; // Height adjust
    float m_tad = 0.2; // Time adjust segment
    float m_tpr = 0.5; // Time propel

    float m_Rpu = 0.03; // Distance push
    float m_apu = 0.8; // Angle push

  };

  //! Gate generator base class.

  class SplineGaitControllerC
    : public SimpleQuadrupedControllerC
  {
  public:
    SplineGaitControllerC();

    //! Do a single timestep
    virtual bool Step(float timeStep,SimpleQuadrupedPoseC &pose) override;

    //! Plot graph of gait
    void PlotGait();

  protected:


    void GenerateFootTrajectory();

    float m_maxSpeed = 6;

    float m_zOffset = 0.7;

    float m_zCentre = 0.0;
    float m_defaultZcentre = 0.0;
    float m_zCentreRange = 0.15;

    float m_Xcentre = -0.00;
    float m_defaultXcentre = -0.06;
    float m_xCentreRange = 0.15;

    float m_timePropel = 0.5;
    float m_defaultTimePropel = 0.5;
    float m_minTimePropel = 0.05;
    float m_maxTimePropel = 1.0;

    float m_hightAdjust = 0.4;
    float m_defaultHightAdjust = 0.15;
    float m_maxHightAdjust = 0.23;

    float m_lengthPropel = 0.15;
    float m_defaultLengthPropel = 0.1;
    float m_maxlengthPropel = 0.16;

    float m_footRotate = 0;
    float m_footRotateMax = M_PI/2;

    float m_footSeperation = 0.00;
    float m_footSeperationDefault = 0.05;
    float m_footSeperationRange = 0.2;

    float m_tiltX = 0;
    float m_tiltXRange = 0.15;

    float m_tiltY = 0;
    float m_tiltYRange = 0.15;

    float m_phase = 0;  //!< Current phase in radians
    float m_defaultOmega = 4;  //!< Radians / second cycle speed
    float m_omega = 2;  //!< Radians / second cycle speed

    float m_phases[4] = { 0,0,0,0 };
    //LegKinematicsC m_legKinematics;

    std::vector<SplineLinear3dC> m_footTrajectories;
    //SplineCatmullRom3dC m_footTrajectory;

  };
}

#endif
