#ifndef Filter_CORE_H
#define FILTER_CORE_H

#include <armadillo>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <vector>
#include <queue>
// basic methods which will help in debugging
// output vector<int> for matrix and colvec use .print() method
std::ostream &operator<<(std::ostream &output, const std::vector<int>& msg);


namespace Fusion {
  namespace FilterCore{
    struct SensorMeasurement{
      std::string topicName_;
      arma::colvec measurement_;
      arma::mat covariance_;

      std::vector<int> updateVector_;
      double time_;
      bool operator()(const boost::shared_ptr<SensorMeasurement> &a, const boost::shared_ptr<SensorMeasurement> &b){
        return (*this)(*a.get(),*(b.get()));
      }
      bool operator()(const SensorMeasurement &a, const SensorMeasurement &b){
        return a.time_>b.time_;
      }
      SensorMeasurement():
      topicName_(""),
      time_(0.0){

      }
    };
    typedef boost::shared_ptr<SensorMeasurement> SensorMeasurementPtr;
    typedef std::priority_queue<SensorMeasurementPtr,std::vector<SensorMeasurementPtr>,SensorMeasurement> SensorMeasurementPtrQueue;

    struct FusedState{
      arma::colvec state_;
      arma::mat estimateErrorCovariance_;
      double lastMeasurementTime_;
      bool operator()(const FusedState &a, const FusedState &b){
        return a.lastMeasurementTime_<b.lastMeasurementTime_;
      }
      FusedState():
      state_(),
      estimateErrorCovariance_(),
      lastMeasurementTime_(0.0){

      }
    };
    typedef boost::shared_ptr<FusedState> FusedStatePtr;


  /** Extended Kalman Filter Equations
  *** Formulation (This formulation is with Additive Noise formulation)
  * x_k = f(x_k-1,u_k-1,w_k-1)
  * z_k = h(x_k,v_k)
  *** Predict step
  * Predicted state estimate       - x_k|k-1 = f(x_k-1|k-1,u_k-1)
  * Predicted covariance estimate  - P_k|k-1 = F_k-1*P_k-1|k-1*F_k-1' + L_k-1*Q_k-1*L_k-1'
  *** Update step
  * Innovation residual   - y_k = z_k - h(x_k|k-1)
  * Innovation Covariance - S_k = H_k*P_k|k-1*H_k' + M_k*R_k*M_k' * Optimal Kalman Gain   - K_k = P_k|k-1*H_k'*S_k^(-1)
  **/
  class EkfCore {
  private: //@TODO make sure that all the matrices are properly initialised while constructing the class
    arma::colvec state_;
    arma::colvec predictedState_;
    arma::mat processMatrix_;              // f

    arma::mat processMatrixJacobian_;      // F
    arma::mat estimateErrorCovariance_;    // P
    arma::mat processNoiseCovariance_;     // Q
    arma::mat covarianceEpsilon_;
    arma::mat identity_;
    // arma::mat measurementNoiseCovariance_; // R
    /*arma::mat imuNoiseCovariance_;         // R*/
    /*arma::mat disturbanceInfluenceMatrix_; // L need to confirm about this approach*/
    // @NOTE H matrix that is observation matrix is defined dynamically depending on the sensor


    /* Parameter declarations */
    double lastFilterTime_;
    double lastMeasurementTime_;
    double lastUpdateTime_;
    bool isInitialised_;

  public:
    EkfCore(); // all the inital parmeters will be updated as the equations are written
    ~EkfCore(); // Close all the files delete all the new type of pointers

    // Core functions of the Filter
    void predict (const double delta);
    void update (const SensorMeasurementPtr& measurementPtr);

    // Getters and setters;
    arma::colvec& getState();
    arma::colvec& getPredictedState();
    arma::mat& getProcessNoiseCovariance();
    arma::mat& getProcessMatrix();
    arma::mat& getProcessMatrixJacobian();
    arma::mat& getEstimateErrorCovariance();
    // arma::mat& getMeasurementNoiseCovariance();
    bool getInitialisedStatus();
    double getLastFilterTime();
    double getLastMeasurementTime();
    double getLastUpdateTime();

    void setState(const arma::colvec&);
    void setProcessMatrix(const arma::mat&);
    void setProcessMatrixJacobian(const arma::mat&);
    void setEstimateErrorCovariance(const arma::mat&);
    void setProcessNoiseCovariance(const arma::mat&);
    // void setMeasurementNoiseCovariance(const arma::mat&);
    void setInitialisedStatus(bool);
    void setLastFilterTime(double);
    void setLastMeasurementTime(double);
    void setLastUpdateTime(double);
  }; // Class EkfCore

} // namespace FilterCore
} // namespace Fusion
#endif
