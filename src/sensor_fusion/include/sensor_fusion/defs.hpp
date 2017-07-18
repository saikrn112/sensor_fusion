#ifndef DEFS_HPP
#define DEFS_HPP
namespace Fusion{
  const int POSITION_SIZE = 3; // Size of the position variables
  const int VELOCITY_SIZE = 3; // Size of the velocity variables
  const int ACCELERATION_SIZE = 3; // Size of the Acceleration variables
  const int QUAT_SIZE = 4; // Size of the quaternion variables
  const int OMEGA_SIZE = 3; // Size of the Angular velocity variables
  const int BIAS_SIZE = 3; // Size of the Angle bias variables
  const int MAGNETIC_NED_SIZE = 3; // Size of the Magnetic field in NED coordinate system
  const int MAGNETIC_XYZ_SIZE = 3; // Size of the Magnetic field in XYZ coordinate system (difference is that fixed with respect to earth, this is fixed wrt Initial position)
  const int STATE_SIZE = POSITION_SIZE + VELOCITY_SIZE + ACCELERATION_SIZE + QUAT_SIZE + OMEGA_SIZE + BIAS_SIZE  ; // Total Number of states
  const int ANGLES_SIZE = 3; //currently used only for anglesCovariance() initialisation
  const int POSE_SIZE = POSITION_SIZE + ANGLES_SIZE; // Size of pose
  const int IMU_MEASUREMENT_SIZE = QUAT_SIZE + ACCELERATION_SIZE + OMEGA_SIZE; // Size of measurments from IMU
  // Frequently Used Constants
  const double PI = 3.141592653589793; // constant Pi
  const double G = 9.80665; // m/s2 // constant Acceleration due to gravity

  // For constructing and updating blocks of matrices and vectors enumeration is used for easy coding
  enum States {
    StatePositionX = 0,
    StatePositionY,
    StatePositionZ,
    StateQuaternion0,
    StateQuaternion1,
    StateQuaternion2,
    StateQuaternion3,
    StateVelocityX,
    StateVelocityY,
    StateVelocityZ,
    StateOmegaX,
    StateOmegaY,
    StateOmegaZ,
    StateAcclerationX,
    StateAcclerationY,
    StateAcclerationZ,
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


} // End of namespace Fusion

#endif
