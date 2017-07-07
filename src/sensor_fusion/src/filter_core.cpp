#include "sensor_fusion/defs.hpp"
#include "sensor_fusion/filter_core.h"

using namespace std;
using namespace arma;

namespace Fusion{
  namespace FilterCore{
  // Constructor
  EkfCore::EkfCore():
  state_(STATE_SIZE),
  predictedState_(STATE_SIZE),
  processMatrix_(STATE_SIZE,STATE_SIZE),
  processMatrixJacobian_(STATE_SIZE,STATE_SIZE),
  estimateErrorCovariance_(STATE_SIZE,STATE_SIZE),
  processNoiseCovariance_(STATE_SIZE,STATE_SIZE),
  covarianceEpsilon_(STATE_SIZE,STATE_SIZE),
  identity_(STATE_SIZE,STATE_SIZE),
  lastFilterTime_(0.0),
  lastMeasurementTime_(0.0),
  lastUpdateTime_(0.0),
  isInitialised_(false){
    state_.zeros();
    predictedState_.zeros();
    processMatrix_.eye();
    estimateErrorCovariance_.eye();
    estimateErrorCovariance_*=1e-9;
    processNoiseCovariance_.eye();
    processNoiseCovariance_*=1e-9;
    covarianceEpsilon_.eye();
    covarianceEpsilon_*=1e-9;
    identity_.eye();

    processNoiseCovariance_(StatePositionX, StatePositionX) = 0.05;
    processNoiseCovariance_(StatePositionY, StatePositionY) = 0.05;
    processNoiseCovariance_(StatePositionZ, StatePositionZ) = 0.06;
    processNoiseCovariance_(StateVelocityX, StateVelocityX) = 0.025;
    processNoiseCovariance_(StateVelocityY, StateVelocityY) = 0.025;
    processNoiseCovariance_(StateVelocityZ, StateVelocityZ) = 0.04;
    processNoiseCovariance_(StateQuaternion0, StateQuaternion0) = 0.01;
    processNoiseCovariance_(StateQuaternion1, StateQuaternion1) = 0.01;
    processNoiseCovariance_(StateQuaternion2, StateQuaternion2) = 0.01;
    processNoiseCovariance_(StateQuaternion3, StateQuaternion3) = 0.01;
    processNoiseCovariance_(StateOmegaX, StateOmegaX) = 0.01;
    processNoiseCovariance_(StateOmegaY, StateOmegaY) = 0.01;
    processNoiseCovariance_(StateOmegaZ, StateOmegaZ) = 0.02;
    processNoiseCovariance_(StateAcclerationX, StateAcclerationX) = 0.01;
    processNoiseCovariance_(StateAcclerationY, StateAcclerationY) = 0.01;
    processNoiseCovariance_(StateAcclerationZ, StateAcclerationZ) = 0.015;


  }

    // Destructor
  EkfCore::~EkfCore() {

  } // Destructor

  arma::colvec& EkfCore::getState(){
      return state_;
  }// method getState

  arma::colvec& EkfCore::getPredictedState(){
    return predictedState_;
  }// method getPredictedState

  arma::mat& EkfCore::getProcessNoiseCovariance(){
    return processNoiseCovariance_;
  }// method getProcessNoiseCovariance

  arma::mat& EkfCore::getProcessMatrix(){
    return processMatrix_;
  }// method getProcessMatrix

  arma::mat& EkfCore::getProcessMatrixJacobian(){
    return processMatrixJacobian_;
  }// method getProcessMatrixJacobian

  arma::mat& EkfCore::getEstimateErrorCovariance(){
    return estimateErrorCovariance_;
  }// method getEstimateErrorCovariance

  bool EkfCore::getInitialisedStatus(){
    return isInitialised_;
  }// method getInitialisedStatus

  double EkfCore::getLastFilterTime(){
    return lastFilterTime_;
  } // method getLastFilterTime

  double EkfCore::getLastMeasurementTime(){
    return lastMeasurementTime_;
  }// method getLastMeasurementTime

  void EkfCore::setState(const arma::colvec& state){
    state_ = state;
  }// method setState

  void EkfCore::setProcessMatrix(const arma::mat& msg){
    processMatrix_ = msg;
  }// method setProcessMatrix

  void EkfCore::setProcessMatrixJacobian(const arma::mat& msg){
    processMatrixJacobian_ = msg;
  }// method setProcessMatrixJacobian

  void EkfCore::setEstimateErrorCovariance(const arma::mat& msg){
    estimateErrorCovariance_ = msg;
  }// method setEstimateErrorCovariance

  void EkfCore::setProcessNoiseCovariance(const arma::mat& msg){
    processNoiseCovariance_ = msg;
  }// method setProcessNoiseCovariance

  void EkfCore::setInitialisedStatus(bool msg){
    isInitialised_ = msg;
  }// setInitialisedStatus

  void EkfCore::setLastFilterTime(double msg){
    lastFilterTime_ = msg;
  }// setLastFilterTime

  void EkfCore::setLastMeasurementTime(double msg){
    lastMeasurementTime_ = msg;
  }// setLastMeasurementTime

  void EkfCore::setLastUpdateTime(double msg){
    lastUpdateTime_ = msg;
  }// setLastUpdateTime

  void EkfCore::quatNormalize(){
    double q0 = state_(StateQuaternion0);
    double q1 = state_(StateQuaternion1);
    double q2 = state_(StateQuaternion2);
    double q3 = state_(StateQuaternion3);

    double normFactor = q0*q0 + q1*q1 + q2*q2 + q3*q3;
    normFactor = pow(normFactor,0.5);
    state_(StateQuaternion0) = q0/normFactor;
    state_(StateQuaternion1) = q1/normFactor;
    state_(StateQuaternion2) = q2/normFactor;
    state_(StateQuaternion3) = q3/normFactor;
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
    quatNormalize(); // for normalizing quaternion

    // Project the process noise covariance forward @TODO need to initialise estimateErrorCovariance_ and processNoiseCovariance_
    // @NOTE need to understand why we need delta in front of processNoiseCovariance_
    // @TODO need to think about pointers (big matrices) atleast for processNoiseCovariance_
    estimateErrorCovariance_ = processMatrixJacobian_*estimateErrorCovariance_*processMatrixJacobian_.t() + delta*processNoiseCovariance_;

  } // method EkfCore::process

  void EkfCore::update(const SensorMeasurementPtr& measurement){

    // We don't want to update everything, so we need to build matrices that only update
    // the measured parts of our state vector. Throughout prediction and correction, we
    // attempt to maximize efficiency in Eigen.

    // First, determine how many state vector values we're updating
    // We will create 3 cases - IMU - GPS - Odometry
    // @TODO  preprocessing data;
    // std::vector<size_t> updateIndices;
    // for (size_t i = 0; i < measurement.updateVector_.size(); ++i)
    // {
    // if (measurement.updateVector_[i])
    // {
    //   // Handle nan and inf values in measurements
    //   if (std::isnan(measurement.measurement_(i)))
    //   {
    //     ROS_DEBUG_STREAM("Value at index " << i << " was nan. Excluding from update.\n");
    //
    //   }
    //   else if (std::isinf(measurement.measurement_(i)))
    //   {
    //     ROS_DEBUG_STREAM("Value at index " << i << " was inf. Excluding from update.\n");
    //   }
    //   else
    //   {
    //     updateIndices.push_back(i);
    //   }
    // }
    // }
    //
    // ROS_DEBUG_STREAM("Update indices are:\n" << updateIndices << "\n");
    //
    // size_t updateSize = updateIndices.size();
    //
    // // Now set up the relevant matrices
    // arma::colvec stateSubset(updateSize);                              // x (in most literature)
    // arma::colvec measurementSubset(updateSize);                        // z
    // arma::mat measurementCovarianceSubset(updateSize, updateSize);  // R
    // arma::mat stateToMeasurementSubset(updateSize, state_.rows());  // H
    // arma::mat kalmanGainSubset(state_.rows(), updateSize);          // K
    // arma::colvec innovationSubset(updateSize);                         // z - Hx
    //
    // stateSubset.setZero();
    // measurementSubset.setZero();
    // measurementCovarianceSubset.setZero();
    // stateToMeasurementSubset.setZero();
    // kalmanGainSubset.setZero();
    // innovationSubset.setZero();
    //
    // // Now build the sub-matrices from the full-sized matrices
    // for (size_t i = 0; i < updateSize; ++i)
    // {
    // measurementSubset(i) = measurement.measurement_(updateIndices[i]);
    // stateSubset(i) = state_(updateIndices[i]);
    //
    // for (size_t j = 0; j < updateSize; ++j)
    // {
    //   measurementCovarianceSubset(i, j) = measurement.covariance_(updateIndices[i], updateIndices[j]);
    // }
    //
    // // Handle negative (read: bad) covariances in the measurement. Rather
    // // than exclude the measurement or make up a covariance, just take
    // // the absolute value.
    // if (measurementCovarianceSubset(i, i) < 0)
    // {
    //   measurementCovarianceSubset(i, i) = ::fabs(measurementCovarianceSubset(i, i));
    // }
    //
    // // If the measurement variance for a given variable is very
    // // near 0 (as in e-50 or so) and the variance for that
    // // variable in the covariance matrix is also near zero, then
    // // the Kalman gain computation will blow up. Really, no
    // // measurement can be completely without error, so add a small
    // // amount in that case.
    // if (measurementCovarianceSubset(i, i) < 1e-9)
    // {
    //   measurementCovarianceSubset(i, i) = 1e-9;
    // }
    // }
    //
    // // The state-to-measurement function, h, will now be a measurement_size x full_state_size
    // // matrix, with ones in the (i, i) locations of the values to be updated
    // for (size_t i = 0; i < updateSize; ++i)
    // {
    // stateToMeasurementSubset(i, updateIndices[i]) = 1;
    // }
    //
    // // (1) Compute the Kalman gain: K = (PH') / (HPH' + R)
    // arma::mat pht = estimateErrorCovariance_ * stateToMeasurementSubset.transpose();
    // arma::mat hphrInv  = (stateToMeasurementSubset * pht + measurementCovarianceSubset).inverse();
    // kalmanGainSubset.noalias() = pht * hphrInv;
    //
    // innovationSubset = (measurementSubset - stateSubset);
    //
    // //@TODO  Wrap angles in the innovation
    // // for (size_t i = 0; i < updateSize; ++i)
    // // {
    // // if (updateIndices[i] == StateRoll  ||
    // //     updateIndices[i] == StatePitch ||
    // //     updateIndices[i] == StateYaw)
    // // {
    // //   while (innovationSubset(i) < -PI)
    // //   {
    // //     innovationSubset(i) += TAU;
    // //   }
    // //
    // //   while (innovationSubset(i) > PI)
    // //   {
    // //     innovationSubset(i) -= TAU;
    // //   }
    // // }
    // // }
    //
    // // (3) Apply the gain to the difference between the state and measurement: x = x + K(z - Hx)
    // state_.noalias() += kalmanGainSubset * innovationSubset;
    //
    // // (4) Update the estimate error covariance using the Joseph form: (I - KH)P(I - KH)' + KRK'
    // arma::mat gainResidual = identity_;
    // gainResidual.noalias() -= kalmanGainSubset * stateToMeasurementSubset;
    // estimateErrorCovariance_ = gainResidual * estimateErrorCovariance_ * gainResidual.transpose();
    // estimateErrorCovariance_.noalias() += kalmanGainSubset *
    //                                       measurementCovarianceSubset *
    //                                       kalmanGainSubset.transpose();

  } // method EkfCore::update
} // namespaceFilterCore
}// namespace Fusion
