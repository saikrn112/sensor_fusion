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

#ifndef Filter_CORE_H
#define FILTER_CORE_H

/* Header file for input and output streams */
#include <iostream>

/* STL vector for storing elements */
#include <vector>

/* STL queue for priority queue container */
#include <queue>

/* Header file for setprecision in streams */
#include <iomanip>

/* shared_ptr from boost for failproof and RAII pointer pass between FilterCore and RosIntegration*/
// [refer: http://www.boost.org/doc/libs/1_62_0/libs/smart_ptr/shared_ptr.htm]
#include <boost/shared_ptr.hpp>

/* Linear Algebra library [refer: http://arma.sourceforge.net/docs.html ] */
#include <armadillo>

/* shortform DEBUG for streaming */
#define DEBUG(msg) if (isDebugMode_) { std::cout << msg; }
// basic methods which will help in debugging
// output vector<int> for matrix and colvec use .print() method
std::ostream &operator<<(std::ostream &output, const std::vector<int>& msg);


namespace Fusion {
  /** This Namespace contains all the core methods of EKF prediction and update
   *  This also contains a DataStructure which stores measurments from various sources
   */
  namespace FilterCore{

    /** DataStructure for storing measurements
     * topicName_    : stores the name of the topic for identifying the sensor
     * time_         : Stores time of the measurment
     * measurment_   : armadillo based column vector for storing measurements
     * updateVector_ : contains list of measurments which the callback has updated. Useful for constructing Measurement Matrices
     * covariance_   : armadillo based matrix for storing covariances of that particular measurement
     * operator ()   : overloads operator for comparing two measurments based on their time of arrival
     */
    struct SensorMeasurement{
      // Members of the structure
      std::string topicName_;
      double time_;
      arma::colvec measurement_;
      std::vector<int> updateVector_;
      arma::mat covariance_;

      // Operator Overload
      bool operator()(const boost::shared_ptr<SensorMeasurement> &a, const boost::shared_ptr<SensorMeasurement> &b){
        return (*this)(*a.get(),*(b.get()));
      }
      bool operator()(const SensorMeasurement &a, const SensorMeasurement &b){
        return a.time_>b.time_;
      }

      // Constructor
      SensorMeasurement():
      topicName_(""),
      time_(0.0){
      }
    };
    // Typedef the pointers and priority queue based Sensor measurement for readability
    typedef boost::shared_ptr<SensorMeasurement> SensorMeasurementPtr;
    typedef std::priority_queue<SensorMeasurementPtr,std::vector<SensorMeasurementPtr>,SensorMeasurement> SensorMeasurementPtrQueue;


  class EkfCore {
  private:
    arma::colvec state_;
    arma::colvec predictedState_;
    arma::mat processMatrix_;              // f

    arma::mat processMatrixJacobian_;      // F
    arma::mat estimateErrorCovariance_;    // P
    arma::mat processNoiseCovariance_;     // Q
    arma::mat covarianceEpsilon_;
    arma::mat identity_;

    /* Parameter declarations */
    double lastFilterTime_;
    double lastMeasurementTime_;
    double lastUpdateTime_;
    bool isInitialised_;
    bool isDebugMode_;

  public:
    EkfCore(); // all the inital parmeters will be updated as the equations are written
    ~EkfCore(); // Close all the files delete all the new type of pointers

    //Getters
    const arma::colvec& getState() const;
    const arma::colvec& getPredictedState() const;
    const arma::mat& getProcessNoiseCovariance() const;
    const arma::mat& getProcessMatrix() const ;
    const arma::mat& getProcessMatrixJacobian() const;
    const arma::mat& getEstimateErrorCovariance() const ;
    bool getInitialisedStatus() const;
    double getLastFilterTime() const;
    double getLastMeasurementTime() const;
    double getLastUpdateTime() const;
    bool getDebugStatus() const;

    //Setters
    void setState(const arma::colvec&);
    void setProcessMatrix(const arma::mat&);
    void setProcessMatrixJacobian(const arma::mat&);
    void setEstimateErrorCovariance(const arma::mat&);
    void setProcessNoiseCovariance(const arma::mat&);
    void setInitialisedStatus(const bool);
    void setLastFilterTime(const double);
    void setLastMeasurementTime(const double);
    void setLastUpdateTime(const double);
    void setDebugStatus(const bool);

    // Core functions of the Filter
    void quatNormalize(void);
    void predict (const double );
    void update (const SensorMeasurementPtr&, const double);

  }; // End of Class EkfCore

} // End of Namespace FilterCore
} // End of Namespace Fusion
#endif
