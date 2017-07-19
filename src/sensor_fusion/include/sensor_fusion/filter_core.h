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
// [refer: http://en.cppreference.com/w/cpp/container/priority_queue]
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

    /** Data Structure SensorMeasurment
     * topicName_    : stores the name of the topic for identifying the sensor
     * time_         : Stores time of the measurment
     * measurment_   : armadillo based column vector for storing measurements
     * updateVector_ : contains list of measurments which the callback has updated. Useful for constructing Measurement Matrices
     * covariance_   : armadillo based matrix for storing covariances of that particular measurement
     * operator ()   : overloads operator for comparing two measurments based on their time of arrival
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
      time_(0.0)
      {
      }
    };

    // Typedef the pointers and priority queue based Sensor measurement for readability
    typedef boost::shared_ptr<SensorMeasurement> SensorMeasurementPtr;
    typedef std::priority_queue<SensorMeasurementPtr,std::vector<SensorMeasurementPtr>,SensorMeasurement> SensorMeasurementPtrQueue;


	/** This class contains the core logic methods and members required for that
     */
    class EkfCore {
    private:

      // Member: Stores the state of the filter_
      arma::colvec state_;
      // Member: Stores the state of the filter_
      arma::colvec predictedState_;

      // Member: Stores the processMatrix_ (f) from Kalman Filter
      arma::mat processMatrix_;              // f

      // Member: Stores the Jacobian of processMatrix_ that is used to compute the Kalman Gain
      arma::mat processMatrixJacobian_;      // F

      // Member: Contains process Modelling noise
      arma::mat estimateErrorCovariance_;    // P

      // Member: Stores the modelling noise covariance (Assumed to be constant Matrix)
      arma::mat processNoiseCovariance_;     // Q

      // Member: Identity matrix which will be useful for calculations
      arma::mat identity_;

   	  // Parameter: last measurment time used to calculate delta
      double lastMeasurementTime_;

	   // Parameter: last Update time to keep track update process
      double lastUpdateTime_;

      // Parameter: To check if the filter state and covariance is initialised or not
      bool isInitialised_;

      // Parameter: variable to check debug mode. If true prints debug statements, vectors, matrices. Note: LOT OF OUTPUT
      bool isDebugMode_;

    public:

	    // Constructer: Initialises the Memebers to default values and also initialises process Noise covariance matrix
      EkfCore();

	    // Destructor
      ~EkfCore();

	    // Getter: gets the current State vector
      const arma::colvec& getState() const;

	    // Getter: gets the predicted state vector
      const arma::colvec& getPredictedState() const;

	    // Getter: gets the Process Noise covariance matrix
      const arma::mat& getProcessNoiseCovariance() const;

	    // Getter: gets the Process matrix
      const arma::mat& getProcessMatrix() const ;

	    // Getter: gets the Process Matrix Jacobian matrix
      const arma::mat& getProcessMatrixJacobian() const;

	    // Getter: gets the estimate error covariance matrix
      const arma::mat& getEstimateErrorCovariance() const ;

	    // Getter: gets the initialised status of the filter
      bool getInitialisedStatus() const;

	    // Getter: gets the Last Filter time of the filter
      double getLastFilterTime() const;

	    // Getter: gets the Last Measurement time of the filter
      double getLastMeasurementTime() const;

	    // Getter: gets the Last Update Time of the filter
      double getLastUpdateTime() const;

	    // Getter: gets the debug status of the filter
      bool getDebugStatus() const;

	    // Setter: sets the state of the filter. Useful for manual setting
      void setState(const arma::colvec&);

	    // Setter: sets the process Matrix
      void setProcessMatrix(const arma::mat&);

	    // Setter: sets the process Matrix Jacobian
      void setProcessMatrixJacobian(const arma::mat&);

	    // Setter: sets the Estimate error Covariance
      void setEstimateErrorCovariance(const arma::mat&);

	    // Setter: sets the process Noise covariance matrix
      void setProcessNoiseCovariance(const arma::mat&);

	    // Setter: sets initialisation status of the filter
      void setInitialisedStatus(const bool);

	    // Setter: sets the last Filter time after prediction and update. This should be equal to last update time,
      //         but delayed measurements can cause problems
      void setLastFilterTime(const double);

	    // Setter: sets the last Measurement time. will be useful for delta calculation
      void setLastMeasurementTime(const double);

	    // Setter: sets the last update time of the filter
      void setLastUpdateTime(const double);

	    // Setter: sets the debug status using parameter server
      void setDebugStatus(const bool);
   	  // Method: normalises the quaternion to keep it within limits, This is equivalent to wrapping angles between -180 to 180 degrees
	    //         but in much more safer way avoiding singularities
      void quatNormalize(void);

	    // Method: performs the prediction step till current filter time + delta
      void predict (const double delta);

	    // Method: Updates the current state estimate using the measurment and sets the current filter time
      void update (const SensorMeasurementPtr&, const double currTime);

    }; // End of Class EkfCore
  } // End of Namespace FilterCore
} // End of Namespace Fusion
#endif
