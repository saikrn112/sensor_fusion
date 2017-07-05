#ifndef DEFS_HPP
#define DEFS_HPP
namespace Fusion{
  const int POSITION_SIZE = 3;
  const int VELOCITY_SIZE = 3;
  const int ACCELERATION_SIZE = 3;
  const int QUAT_SIZE = 4;
  const int OMEGA_SIZE = 3;
  const int BIAS_SIZE = 3;
  const int MAGNETIC_NED_SIZE = 3;
  const int MAGNETIC_XYZ_SIZE = 3;
  const int STATE_SIZE = POSITION_SIZE + VELOCITY_SIZE + ACCELERATION_SIZE + QUAT_SIZE + OMEGA_SIZE + BIAS_SIZE  ;
  const int ANGLES_SIZE = 3; //currently used only for anglesCovariance() initialisation
  // Frequently Used Constants
  const double PI = 3.141592653589793;
  const double G = 9.80665; // m/s2
  const double TAU = 6.283185307179587;

  enum States {
    StatePositionX = 0,
    StatePositionY,
    StatePositionZ,
    StateVelocityX,
    StateVelocityY,
    StateVelocityZ,
    StateAcclerationX,
    StateAcclerationY,
    StateAcclerationZ,
    StateQuaternion0,
    StateQuaternion1,
    StateQuaternion2,
    StateQuaternion3,
    StateOmegaX,
    StateOmegaY,
    StateOmegaZ,
    StateDeltaAngleBiasX,
    StateDeltaAngleBiasY,
    StateDeltaAngleBiasZ,
    StateMagneticN,
    StateMagneticE,
    StateMagneticD,
    StateMagneticX,
    StateMagneticY,
    StateMagneticZ
  }; // 24 States


} // namespace Fusion

#endif
