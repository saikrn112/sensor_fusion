#include "sensor_fusion/Defs.hpp"
#include "sensor_fusion/filter_core.h"

using namespace std;
using namespace arma;

namespace Fusion{
namespace FilterCore {
  // Constructor
  EkfCore::EkfCore() {

  }

    // Destructor
  EkfCore::~EkfCore() {

  }

  void EkfCore::predict(const double delta){
    /* Temporary Variables */
    // Quaternion components
    double vx    = state_(StateVelocityX);
    double vy    = state_(StateVelocityY);
    double vz    = state_(StateVelocityZ);
    double ax    = state_(StateAcclerationX);
    double ay    = state_(StateAcclerationY);
    double az    = state_(StateAcclerationZ);
    double q0    = state_(StateQuaternion0);
    double q1    = state_(StateQuaternion1);
    double q2    = state_(StateQuaternion2);
    double q3    = state_(StateQuaternion3);
    double wx    = state_(StateOmegaX);
    double wy    = state_(StateOmegaY);
    double wz    = state_(StateOmegaZ);
    double dax_b = state_(StateDeltaAngleBiasX);
    double day_b = state_(StateDeltaAngleBiasY);
    double daz_b = state_(StateDeltaAngleBiasZ);

    // Rotation matrix components
    double r11 = q0*q0 + q1*q1 - q2*q2 - q3*q3; // X-X velocity component
    double r22 = q0*q0 + q1*q1 - q2*q2 - q3*q3; // Y-Y velocity component
    double r33 = q0*q0 + q1*q1 - q2*q2 - q3*q3; // Z-Z velocity component
    double r12 = 2*(q1*q2 - q3*q0); // X-Y velocity component
    double r21 = 2*(q1*q2 + q3*q0); // Y-X velocity component
    double r13 = 2*(q1*q3 + q2*q0); // X-Z velocity component
    double r31 = 2*(q1*q3 - q2*q0); // Z-X velocity component
    double r23 = 2*(q2*q3 - q1*q0); // Y-Z velocity component
    double r32 = 2*(q2*q3 + q1*q0); // Z-Y velocity component

    // Quaternion based calculations :[Refer Price borough's implementation]
    double dax = wx*delta - dax_b;
    double day = wy*delta - day_b;
    double daz = wz*delta - daz_b;
    double daxHalf = dax/2;
    double dayHalf = day/2;
    double dazHalf = daz/2;

    processMatrix_.eye(STATE_SIZE,STATE_SIZE); // @TODO: send it to Constructor

    /** Note(1)
    * Do the necessary transformations
    * XYZ are in global frame so rotate Velocities, accelerations etc and fill
    * components
    **/
    processMatrix_(StatePositionX,StateVelocityX) = r11*delta;
    processMatrix_(StatePositionX,StateVelocityY) = r12*delta;
    processMatrix_(StatePositionX,StateVelocityZ) = r13*delta;
    processMatrix_(StatePositionY,StateVelocityX) = r21*delta;
    processMatrix_(StatePositionY,StateVelocityY) = r22*delta;
    processMatrix_(StatePositionY,StateVelocityZ) = r23*delta;
    processMatrix_(StatePositionZ,StateVelocityX) = r31*delta;
    processMatrix_(StatePositionZ,StateVelocityY) = r32*delta;
    processMatrix_(StatePositionZ,StateVelocityZ) = r33*delta;
    processMatrix_(StatePositionX,StateAcclerationX) = 0.5*processMatrix_(StatePositionX,StateVelocityX)*delta;
    processMatrix_(StatePositionX,StateAcclerationY) = 0.5*processMatrix_(StatePositionX,StateVelocityY)*delta;
    processMatrix_(StatePositionX,StateAcclerationZ) = 0.5*processMatrix_(StatePositionX,StateVelocityZ)*delta;
    processMatrix_(StatePositionY,StateAcclerationX) = 0.5*processMatrix_(StatePositionY,StateVelocityX)*delta;
    processMatrix_(StatePositionY,StateAcclerationY) = 0.5*processMatrix_(StatePositionY,StateVelocityY)*delta;
    processMatrix_(StatePositionY,StateAcclerationZ) = 0.5*processMatrix_(StatePositionY,StateVelocityZ)*delta;
    processMatrix_(StatePositionZ,StateAcclerationX) = 0.5*processMatrix_(StatePositionZ,StateVelocityX)*delta;
    processMatrix_(StatePositionZ,StateAcclerationY) = 0.5*processMatrix_(StatePositionZ,StateVelocityY)*delta;
    processMatrix_(StatePositionZ,StateAcclerationZ) = 0.5*processMatrix_(StatePositionZ,StateVelocityZ)*delta;
    processMatrix_(StateVelocityX,StateAcclerationX) = delta;
    processMatrix_(StateVelocityY,StateAcclerationY) = delta;
    processMatrix_(StateVelocityZ,StateAcclerationZ) = delta;
    processMatrix_(StateQuaternion0,StateQuaternion1) = -daxHalf;
    processMatrix_(StateQuaternion0,StateQuaternion2) = -dayHalf;
    processMatrix_(StateQuaternion0,StateQuaternion3) = -dazHalf;
    processMatrix_(StateQuaternion1,StateQuaternion0) =  daxHalf;
    processMatrix_(StateQuaternion1,StateQuaternion2) =  dazHalf;
    processMatrix_(StateQuaternion1,StateQuaternion3) = -dayHalf;
    processMatrix_(StateQuaternion2,StateQuaternion0) =  dayHalf;
    processMatrix_(StateQuaternion2,StateQuaternion1) = -dazHalf;
    processMatrix_(StateQuaternion2,StateQuaternion3) =  daxHalf;
    processMatrix_(StateQuaternion3,StateQuaternion0) =  dazHalf;
    processMatrix_(StateQuaternion3,StateQuaternion1) =  dayHalf;
    processMatrix_(StateQuaternion3,StateQuaternion2) = -daxHalf;

    // Much of the processMatrixJacocian is identical to the processMatrix
    processMatrixJacobian_ = processMatrix_;

    double xCoeff = 2*(vx*delta + 0.5*delta*delta*ax);
    double yCoeff = 2*(vy*delta + 0.5*delta*delta*ay);
    double zCoeff = 2*(vz*delta + 0.5*delta*delta*az);

    processMatrixJacobian_(StatePositionX,StateQuaternion0) =  q0*xCoeff - q3*yCoeff + q2*zCoeff;
    processMatrixJacobian_(StatePositionX,StateQuaternion1) =  q1*xCoeff + q2*yCoeff + q3*zCoeff;
    processMatrixJacobian_(StatePositionX,StateQuaternion2) = -q2*xCoeff + q1*yCoeff + q0*zCoeff;
    processMatrixJacobian_(StatePositionX,StateQuaternion3) = -q3*xCoeff - q0*yCoeff + q1*zCoeff;
    processMatrixJacobian_(StatePositionY,StateQuaternion0) =  q3*xCoeff + q0*yCoeff - q1*zCoeff;
    processMatrixJacobian_(StatePositionY,StateQuaternion1) =  q2*xCoeff - q1*yCoeff - q0*zCoeff;
    processMatrixJacobian_(StatePositionY,StateQuaternion2) =  q1*xCoeff + q2*yCoeff + q3*zCoeff;
    processMatrixJacobian_(StatePositionY,StateQuaternion3) =  q0*xCoeff - q3*yCoeff + q2*zCoeff;
    processMatrixJacobian_(StatePositionZ,StateQuaternion0) = -q2*xCoeff + q1*yCoeff + q0*zCoeff;
    processMatrixJacobian_(StatePositionZ,StateQuaternion1) =  q3*xCoeff + q0*yCoeff - q1*zCoeff;
    processMatrixJacobian_(StatePositionZ,StateQuaternion2) = -q0*xCoeff + q3*yCoeff - q2*zCoeff;
    processMatrixJacobian_(StatePositionZ,StateQuaternion3) =  q1*xCoeff + q2*yCoeff + q3*zCoeff;
    processMatrixJacobian_(StateQuaternion0,StateQuaternion1) = -daxHalf;
    processMatrixJacobian_(StateQuaternion0,StateQuaternion2) = -dayHalf;
    processMatrixJacobian_(StateQuaternion0,StateQuaternion3) = -dazHalf;
    processMatrixJacobian_(StateQuaternion1,StateQuaternion0) =  daxHalf;
    processMatrixJacobian_(StateQuaternion1,StateQuaternion2) =  dazHalf;
    processMatrixJacobian_(StateQuaternion1,StateQuaternion3) = -dayHalf;
    processMatrixJacobian_(StateQuaternion2,StateQuaternion0) =  dayHalf;
    processMatrixJacobian_(StateQuaternion2,StateQuaternion1) = -dazHalf;
    processMatrixJacobian_(StateQuaternion2,StateQuaternion3) =  daxHalf;
    processMatrixJacobian_(StateQuaternion3,StateQuaternion0) =  dazHalf;
    processMatrixJacobian_(StateQuaternion3,StateQuaternion1) =  dayHalf;
    processMatrixJacobian_(StateQuaternion3,StateQuaternion2) = -dazHalf;
    processMatrixJacobian_(StateQuaternion0,StateOmegaX) = -q1*delta/2;
    processMatrixJacobian_(StateQuaternion0,StateOmegaY) = -q2*delta/2;
    processMatrixJacobian_(StateQuaternion0,StateOmegaZ) = -q3*delta/2;
    processMatrixJacobian_(StateQuaternion1,StateOmegaX) =  q0*delta/2;
    processMatrixJacobian_(StateQuaternion1,StateOmegaY) = -q3*delta/2;
    processMatrixJacobian_(StateQuaternion1,StateOmegaZ) =  q2*delta/2;
    processMatrixJacobian_(StateQuaternion2,StateOmegaX) =  q3*delta/2;
    processMatrixJacobian_(StateQuaternion2,StateOmegaY) =  q0*delta/2;
    processMatrixJacobian_(StateQuaternion2,StateOmegaZ) = -q1*delta/2;
    processMatrixJacobian_(StateQuaternion3,StateOmegaX) = -q2*delta/2;
    processMatrixJacobian_(StateQuaternion3,StateOmegaY) =  q1*delta/2;
    processMatrixJacobian_(StateQuaternion3,StateOmegaZ) =  q0*delta/2;
    processMatrixJacobian_(StateQuaternion0,StateDeltaAngleBiasX) =  q1/2;
    processMatrixJacobian_(StateQuaternion0,StateDeltaAngleBiasY) =  q2/2;
    processMatrixJacobian_(StateQuaternion0,StateDeltaAngleBiasZ) =  q3/2;
    processMatrixJacobian_(StateQuaternion1,StateDeltaAngleBiasX) = -q0/2;
    processMatrixJacobian_(StateQuaternion1,StateDeltaAngleBiasY) =  q3/2;
    processMatrixJacobian_(StateQuaternion1,StateDeltaAngleBiasZ) = -q2/2;
    processMatrixJacobian_(StateQuaternion2,StateDeltaAngleBiasX) = -q3/2;
    processMatrixJacobian_(StateQuaternion2,StateDeltaAngleBiasY) = -q0/2;
    processMatrixJacobian_(StateQuaternion2,StateDeltaAngleBiasZ) =  q1/2;
    processMatrixJacobian_(StateQuaternion3,StateDeltaAngleBiasX) =  q2/2;
    processMatrixJacobian_(StateQuaternion3,StateDeltaAngleBiasY) = -q1/2;
    processMatrixJacobian_(StateQuaternion3,StateDeltaAngleBiasZ) = -q0/2;

    // projecting state forward till lastFilterTime+delta
    state_ = processMatrix_*state_;

    // Project the process noise covariance forward @TODO need to initialise estimateErrorCovariance_ and processNoiseCovariance_
    // @NOTE need to understand why we need delta in front of processNoiseCovariance_
    // @TODO need to think about pointers (big matrices) atleast for processNoiseCovariance_
    estimateErrorCovariance_ = processMatrixJacobian_*estimateErrorCovariance_*processMatrixJacobian_.t() + delta*processNoiseCovariance_;

  } // method EkfCore::process

  void EkfCore::update(const SensorMeasurement& measurement){
      
  } // method EkfCore::update

}// namespace FilterCore
}// namespace Fusion
