#include "sensor_fusion/defs.hpp"
#include "sensor_fusion/filter_core.h"

using namespace std;

// Overloading operator for printing vectors
std::ostream &operator<<(std::ostream &output, const std::vector<int>& msg){
  int vecSize = msg.size();
  output << "[";
  for(int i=0; i<vecSize; i++){
    output << msg[i]<<" ";
  }
  output <<"]" << endl;
  return output;
};


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
    identity_(STATE_SIZE,STATE_SIZE),
    lastFilterTime_(0.0),
    lastMeasurementTime_(0.0),
    lastUpdateTime_(0.0),
    isInitialised_(false),
    isDebugMode_(false){
      state_.zeros();
      predictedState_.zeros();
      processMatrix_.eye();
      estimateErrorCovariance_.eye();
      estimateErrorCovariance_*=1e-9;
      processNoiseCovariance_.eye();
      processNoiseCovariance_*=1e-9;
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

    const arma::colvec& EkfCore::getState() const{
        return state_;
    }// Getter getState

    const arma::colvec& EkfCore::getPredictedState() const{
      return predictedState_;
    }// Getter getPredictedState

    const arma::mat& EkfCore::getProcessNoiseCovariance() const{
      return processNoiseCovariance_;
    }// Getter getProcessNoiseCovariance

    const arma::mat& EkfCore::getProcessMatrix() const{
      return processMatrix_;
    }// Getter getProcessMatrix

    const arma::mat& EkfCore::getProcessMatrixJacobian() const{
      return processMatrixJacobian_;
    }// Getter getProcessMatrixJacobian

    const arma::mat& EkfCore::getEstimateErrorCovariance() const{
      return estimateErrorCovariance_;
    }// Getter getEstimateErrorCovariance

    bool EkfCore::getInitialisedStatus() const{
      return isInitialised_;
    }// Getter getInitialisedStatus

    double EkfCore::getLastFilterTime() const{
      return lastFilterTime_;
    }// Getter getLastFilterTime

    double EkfCore::getLastMeasurementTime() const{
      return lastMeasurementTime_;
    }// Getter getLastMeasurementTime

    bool EkfCore::getDebugStatus() const{
      return isDebugMode_;
    }// Getter getDebugStatus

    void EkfCore::setState(const arma::colvec& state){
      state_ = state;
    }// Setter setState

    void EkfCore::setProcessMatrix(const arma::mat& msg){
      processMatrix_ = msg;
    }// Setter setProcessMatrix

    void EkfCore::setProcessMatrixJacobian(const arma::mat& msg){
      processMatrixJacobian_ = msg;
    }// Setter setProcessMatrixJacobian

    void EkfCore::setEstimateErrorCovariance(const arma::mat& msg){
      estimateErrorCovariance_ = msg;
    }// Setter setEstimateErrorCovariance

    void EkfCore::setProcessNoiseCovariance(const arma::mat& msg){
      processNoiseCovariance_ = msg;
    }// Setter setProcessNoiseCovariance

    void EkfCore::setInitialisedStatus(bool msg){
      isInitialised_ = msg;
    }// Setter setInitialisedStatus

    void EkfCore::setLastFilterTime(double msg){
      lastFilterTime_ = msg;
    }// Setter setLastFilterTime

    void EkfCore::setLastMeasurementTime(double msg){
      lastMeasurementTime_ = msg;
    }// Setter setLastMeasurementTime

    void EkfCore::setLastUpdateTime(double msg){
      lastUpdateTime_ = msg;
    }// Setter setLastUpdateTime

    void EkfCore::setDebugStatus(bool msg){
      isDebugMode_ = msg;
    }// Setter setDebugStatus

    /** Definition of quatNormalize
     *  for normalising the quaternion to keep its norm unit
     */
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
    }// method quatNormalize

    /** Definiton of Prediction step in Extended Kalman Filter
     * Parameter: Delta is for projecting the state till current filter time + delta
     * All the processMatrix_ values and processMatrixJacobian_ are defined
     */
    void EkfCore::predict(const double delta){
      DEBUG( "=============================Before Prediction Step=============================\n"
                  << "Delta: " << std::setprecision(14) << delta << endl
                  << "State:\n"<< std::setprecision(2) << state_ << endl
                  << "Process Matrix:\n" << processMatrix_ << endl
                  << "processMatrixJacobian:\n" << processMatrixJacobian_ << endl
                  << "estimateErrorCovariance:\n" << estimateErrorCovariance_ << endl
                  << "processNoiseCovariance:\n" << processNoiseCovariance_ << endl;
      )

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
      double r22 = q0*q0 + q2*q2 - q1*q1 - q3*q3; // Y-Y velocity component
      double r33 = q0*q0 + q3*q3 - q2*q2 - q1*q1; // Z-Z velocity component
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

      // Calculating processMatrix_ or transfer function in discretized form
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

      // Updating the processMatrixJacobian_
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

      // Normalising quaternion
      quatNormalize();

      // Project the process noise covariance forward
      estimateErrorCovariance_ = processMatrixJacobian_*estimateErrorCovariance_*processMatrixJacobian_.t() + delta*processNoiseCovariance_;
      DEBUG("=============================Prediction Step=============================\n"
                  << "Delta: " << std::setprecision(14) << delta << endl
                  << "State:\n"<< std::setprecision(2) <<state_ << endl
                  << "Process Matrix:\n" << processMatrix_ << endl
                  << "processMatrixJacobian:\n" << processMatrixJacobian_ << endl
                  << "estimateErrorCovariance:\n" << estimateErrorCovariance_ << endl
                  << "processNoiseCovariance:\n" << processNoiseCovariance_ << endl;
      )
    } // method EkfCore::process

    /** Definition of the Update step in Extended Kalman Filter
     *  Dynamic update of the measurement Matrix is implemented
     *  Using simple matrix multiplication provided by Armadillo library Update step is performed
     *  Parameter: measurement is given as input to the update function
     *  Parameter: currTime is given to store the time when update step is performed.
     */
    void EkfCore::update(const SensorMeasurementPtr& measurement, const double currTime){
      // This parameter is used to capture which elements from the state does the measurement vector contain.
      vector<int> updateIndices;
      for(int i=0; i<measurement->updateVector_.size(); i++) {
        if(measurement->updateVector_[i]){
          updateIndices.push_back(i);
        }
      }

    	// Updating the measurement time
      lastMeasurementTime_ = measurement->time_;

    	// Construct all the measurement matrices from these udpated indices size;
      arma::colvec innovation(updateIndices.size());                                    // y Vector
      innovation.zeros();

      arma::mat measurementMatrix(updateIndices.size(), state_.n_rows);                 // H Matrix
      measurementMatrix.zeros();

      arma::mat innovationCovariance(updateIndices.size(),updateIndices.size());        // S Matrix
      innovationCovariance.zeros();

      arma::mat kalmanGainMatrix(state_.n_rows, updateIndices.size());                  // K Matrix
      kalmanGainMatrix.zeros();

      arma::mat measurementCovarianceMatrix(updateIndices.size(),updateIndices.size()); // R Matrix
      measurementCovarianceMatrix.zeros();

  	   // Constructing Measurement Matrix
      for(int i=0; i<updateIndices.size(); i++){
        measurementMatrix(i,updateIndices[i]) = 1;
      }

  	   // Extracting measurements into a temperary vector
      arma::colvec tmpMeasurement(updateIndices.size());
      for(int i=0; i<updateIndices.size(); i++){
        tmpMeasurement(i) = measurement->measurement_(updateIndices[i]);
      }

  	   // Calculating innovation Vector
      innovation = tmpMeasurement - measurementMatrix*state_;

    	// Extracting Measurement Matrix from the measurement function got
    	for(int i=0; i<updateIndices.size(); i++){
          measurementCovarianceMatrix(i,i) = measurement->covariance_(updateIndices[i],updateIndices[i]);
        }

    	// Temperory Matrix
      arma::mat pHt = estimateErrorCovariance_*measurementMatrix.t();
      innovationCovariance = measurementMatrix*pHt + measurementCovarianceMatrix;
      kalmanGainMatrix = pHt*innovationCovariance.i();

    	// Calculating new state and estimate Error covariance
    	state_ += kalmanGainMatrix*innovation;
      estimateErrorCovariance_ = (identity_ - kalmanGainMatrix*measurementMatrix)*estimateErrorCovariance_;

    	// Normalizing the quaternion to keep it unit norm
    	quatNormalize();

    	// Updating the filterTime with current time
      lastFilterTime_ = currTime;
      DEBUG("=============================Update Step=============================\n"
                  << "Measurement:\n" << measurement->measurement_ << endl
                  << "Update Indices" << updateIndices << endl
                  << "Measurement Matrix(H)\n:" << measurementMatrix << endl
                  << "Temporary Measurement Vector:\n" << tmpMeasurement << endl
                  << "Measurement Covariance Matrix(R):\n" << measurementCovarianceMatrix << endl
                  << "Innovation Covariance(S):\n" << innovationCovariance << endl
                  << "kalmanGainMatrix(K):\n" << kalmanGainMatrix << endl
                  << "estimateErrorCovariance:\n" << estimateErrorCovariance_ << endl
                  << "State:\n" << state_ << endl)

    } // method EkfCore::update
  } // namespaceFilterCore
}// namespace Fusion
