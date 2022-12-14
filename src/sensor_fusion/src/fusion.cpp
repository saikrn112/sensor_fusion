#include "sensor_fusion/fusion.hpp"

using namespace std;
using namespace arma;

namespace Fusion{
  namespace RosIntegration{

    // Class Ekf
    Ekf::Ekf(ros::NodeHandle* nodeHandle):
    nh_(*nodeHandle),
    nhPriv_("~"),
    isGPSFirstMeasurement_(false){
      ROS_INFO("Got the Node Handle. Initialising IMU, GPS and Odometry Callback");

      // Initialising the subscribers
      imuSub = nh_.subscribe("/imu/data",1,&Ekf::imu_cb,this);
      gpsSub = nh_.subscribe("/navsat/fix",1,&Ekf::gps_cb,this);
      odomSub = nh_.subscribe("/odometry/filtered",1,&Ekf::odom_cb,this);

      // Initialising the publi
      pub = nh_.advertise<nav_msgs::Odometry>("/odomCombined",1,true);
      Ekf::loadParams();

      // Setting filter initial values
      filter_.setLastMeasurementTime(ros::Time::now().toSec());
      filter_.setLastFilterTime(ros::Time::now().toSec());
      arma::colvec state(STATE_SIZE);
      state.zeros();
      filter_.setState(state);


      nav_msgs::Odometry fusedState;

      // Rate at which node should publish
      ros::Rate rate(30);

      while(ros::ok()){
        // Integrate Measurments and update the state
        integrateSensorMeasurements();

        // Get the current state
        getFusedState(fusedState);

        // Publish it in given topic name
        pub.publish(fusedState);

        // Dynamic reconfigure
        dynamic_reconfigure::Server<sensor_fusion::fusionConfig> server;
        dynamic_reconfigure::Server<sensor_fusion::fusionConfig>::CallbackType f;

        f = boost::bind(&Ekf::parameter_cb,this, _1, _2); // [refer: http://www.radmangames.com/programming/how-to-use-boost-bind]
        server.setCallback(f);

        // To publish at specified frequency
        rate.sleep();

        // To update the sensor msgs
        ros::spinOnce();
      }
    } // Constructor

    void Ekf::imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
      // ROS_INFO("in Imu call back");

      // //@TODO if they are just constants why not just hard code or parameterise them instead of reading from msgs;
      // extracting angles covariance from 1D array to 3D array
      arma::mat anglesCovariance(ANGLES_SIZE,ANGLES_SIZE);
      anglesCovariance.eye();
      anglesCovariance(0,0) = msg->orientation_covariance[0];
      anglesCovariance(1,1) = msg->orientation_covariance[4];
      anglesCovariance(2,2) = msg->orientation_covariance[8];

      //quaternion covariance matrix instance
      arma::mat quaternionCovariance(QUAT_SIZE,QUAT_SIZE);
      quaternionCovariance.eye();
      // quaternionCovariance *= 1e-3;

      // extracting omage covariance from an 1D array to 3D array
      arma::mat accelerationCovariance(ACCELERATION_SIZE,ACCELERATION_SIZE);
      accelerationCovariance.eye();
      accelerationCovariance(0,0) = msg->linear_acceleration_covariance[0];
      accelerationCovariance(1,1) = msg->linear_acceleration_covariance[4];
      accelerationCovariance(2,2) = msg->linear_acceleration_covariance[8];

      // extracting omage covariance from an 1D array to 3D array
      arma::mat omegaCovariance(OMEGA_SIZE,OMEGA_SIZE);
      omegaCovariance.eye();
      omegaCovariance(0,0) = msg->angular_velocity_covariance[0];
      omegaCovariance(1,1) = msg->angular_velocity_covariance[4];
      omegaCovariance(2,2) = msg->angular_velocity_covariance[8];

      // Filling the measurement vector with message values
      arma::colvec measurement(STATE_SIZE);
      measurement.zeros();
      measurement(StateQuaternion0) = msg->orientation.x; // @TODO make sure that q0 is scalar
      measurement(StateQuaternion1) = msg->orientation.y;
      measurement(StateQuaternion2) = msg->orientation.z;
      measurement(StateQuaternion3) = msg->orientation.w;
      measurement(StateAcclerationX) = msg->linear_acceleration.x;
      measurement(StateAcclerationY) = msg->linear_acceleration.y;
      measurement(StateAcclerationZ) = msg->linear_acceleration.z;
      measurement(StateOmegaX) = msg->angular_velocity.x;
      measurement(StateOmegaY) = msg->angular_velocity.y;
      measurement(StateOmegaZ) = msg->angular_velocity.z;

      // Preprocessing IMU data - Removing the Gravitational Acceleration. Keeping a parameter just in case
      if(removeGravititionalAcceleration_){
        tf2::Vector3 gravity(0,0,G);
        tf2::Quaternion q(measurement(StateQuaternion0),measurement(StateQuaternion1),measurement(StateQuaternion2),measurement(StateQuaternion3));
        tf2::Matrix3x3 rotMat(q);
        tf2::Vector3 a = rotMat*gravity;
        measurement(StateAcclerationX) -= a.getX();
        measurement(StateAcclerationY) -= a.getY();
        measurement(StateAcclerationZ) -= a.getZ();
      }

      // update covariances with appropriate blocks
      arma::mat covariance(STATE_SIZE,STATE_SIZE);
      covariance.eye();
      covariance.submat(StateQuaternion0,StateQuaternion0,StateQuaternion3,StateQuaternion3) = quaternionCovariance;
      covariance.submat(StateAcclerationX,StateAcclerationX,StateAcclerationZ,StateAcclerationZ) = accelerationCovariance;
      covariance.submat(StateOmegaX,StateOmegaX,StateOmegaZ,StateOmegaZ) = omegaCovariance;

      // updateVector
      std::vector<int> updateVector(STATE_SIZE,0);
      updateVector[StateQuaternion0] = 1;
      updateVector[StateQuaternion1] = 1;
      updateVector[StateQuaternion2] = 1;
      updateVector[StateQuaternion3] = 1;
      updateVector[StateOmegaX] = 1;
      updateVector[StateOmegaY] = 1;
      updateVector[StateOmegaZ] = 1;
      updateVector[StateAcclerationX] = 1;
      updateVector[StateAcclerationY] = 1;
      updateVector[StateAcclerationZ] = 1;

      //Enqueuing the IMU measurement in the priority queue.
      FilterCore::SensorMeasurementPtr measurementPtr = FilterCore::SensorMeasurementPtr(new FilterCore::SensorMeasurement);
      measurementPtr->topicName_ = "IMU";
      measurementPtr->measurement_ = measurement;
      measurementPtr->covariance_ = covariance;
      measurementPtr->updateVector_ = updateVector;
      measurementPtr->time_ = msg->header.stamp.toSec();

      // Adding measurements in the measurement queue
      addMeasurementinQueue(measurementPtr);
    } // method imu_cb

    void Ekf::gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
      if(shouldGPS){
        ROS_INFO("in gps call back");
        std::string zone;

        // Takes the first measurement and subtracts it from subsequent measurements (Assuming coordinate system is NED)
        double initlonN, initlatE=0;
        if(!isGPSFirstMeasurement_){
          gps_common::LLtoUTM(msg->latitude, msg->longitude,initlonN, initlatE,zone);// Converting from lat, long to UTM
          initialLonInNED_ = initlonN;
          initialLatInNED_ = initlatE;
          initialAltitude_ = msg->altitude;
          isGPSFirstMeasurement_ = true;
          // DEBUG("\ninitialgpsX: " << std::setprecision(10) << initialLatInNED_ << "\ninitialgpsY: " << initialLonInNED_ << "\ninitialgpsZ: " << initialAltitude_);
        }

        double latE=0;
        double lonN=0;
        gps_common::LLtoUTM(msg->latitude, msg->longitude,lonN, latE,zone);// Converting from lat, long to UTM
        double gpsX = lonN-initialLonInNED_;
        double gpsY = -latE+initialLatInNED_;
        double gpsZ = msg->altitude - initialAltitude_;
        DEBUG("\ngpsX: "<< std::setprecision(10) << gpsX << "\ngpsY: " << gpsY << "\ngpsZ: " << gpsZ);

        // Filling the Position covariance
        arma::mat positionCovariance(POSITION_SIZE,POSITION_SIZE);
        positionCovariance.eye();
        for(int i=0; i<POSITION_SIZE; i++){
          positionCovariance(i,i) = msg-> position_covariance[i*POSITION_SIZE +i];
        }

        DEBUG("\n" << setprecision(10) <<gpsX << "," << gpsY << "," << gpsZ << endl);

        // Filling the measurement and covariance matrices
        arma::colvec measurement(STATE_SIZE);
        measurement.zeros();
        measurement(StatePositionX) = gpsX;
        measurement(StatePositionY) = gpsY;
        measurement(StatePositionZ) = gpsZ;
        arma::mat covariance(STATE_SIZE,STATE_SIZE);
        covariance.eye();
        covariance.submat(StatePositionX,StatePositionX,StatePositionZ,StatePositionZ) = positionCovariance;

        // updateVector
        std::vector<int> updateVector(STATE_SIZE,0);
        updateVector[StatePositionX] = 1;
        updateVector[StatePositionY] = 1;
        updateVector[StatePositionZ] = 1;

        //Enqueuing the IMU measurement in the priority queue.
        FilterCore::SensorMeasurementPtr measurementPtr = FilterCore::SensorMeasurementPtr(new FilterCore::SensorMeasurement);
        measurementPtr->topicName_ = "GPS";
        measurementPtr->measurement_ = measurement;
        measurementPtr->covariance_ = covariance;
        measurementPtr->updateVector_ = updateVector;
        measurementPtr->time_ = msg->header.stamp.toSec();

        // Adding measurement in the queue
        addMeasurementinQueue(measurementPtr);
      }
    } // method gps_cb

    void Ekf::odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
      ROS_INFO("in Odom call back");

      // Extracting the position covariance
      arma::mat positionCovariance(POSITION_SIZE,POSITION_SIZE);
      positionCovariance.eye();
      for(int i=0; i<POSITION_SIZE; i++){
        positionCovariance(i,i) = msg->pose.covariance[i*POSITION_SIZE +i];
      }

      // Extracting the orientation covariance
      arma::mat orientationCovariance(POSITION_SIZE,POSITION_SIZE);
      orientationCovariance.eye();
      for(int i=POSITION_SIZE; i<POSE_SIZE; i++){
        orientationCovariance(i-POSITION_SIZE,i-POSITION_SIZE) = msg->pose.covariance[i*ANGLES_SIZE +i];
      }
      arma::mat quaternionCovariance(QUAT_SIZE,QUAT_SIZE);
      quaternionCovariance.eye();

      // Extracting the Twist covariance
      arma::mat twistCovariance(POSE_SIZE,POSE_SIZE);
      twistCovariance.eye();
      for(int i=0; i<POSE_SIZE; i++){
        twistCovariance(i,i) = msg->twist.covariance[i*POSE_SIZE+i];
      }

      // Filling measurements
      arma::colvec measurement(STATE_SIZE);
      measurement.zeros();
      measurement(StatePositionX) = msg->pose.pose.position.x;
      measurement(StatePositionY) = msg->pose.pose.position.y;
      measurement(StatePositionZ) = msg->pose.pose.position.z;
      measurement(StateQuaternion0) = msg->pose.pose.orientation.x;
      measurement(StateQuaternion1) = msg->pose.pose.orientation.y;
      measurement(StateQuaternion2) = msg->pose.pose.orientation.z;
      measurement(StateQuaternion3) = msg->pose.pose.orientation.w;
      measurement(StateVelocityX) = msg->twist.twist.linear.x;
      measurement(StateVelocityY) = msg->twist.twist.linear.y;
      measurement(StateVelocityZ) = msg->twist.twist.linear.z;
      measurement(StateOmegaX) = msg->twist.twist.angular.x;
      measurement(StateOmegaY) = msg->twist.twist.angular.y;
      measurement(StateOmegaZ) = msg->twist.twist.angular.z;

      // Filling the covariance from measurement into datastructure in appropriate blocks
      arma::mat covariance(STATE_SIZE,STATE_SIZE);
      covariance.eye();
      covariance.submat(StatePositionX,StatePositionX,StatePositionZ,StatePositionZ) = positionCovariance;
      covariance.submat(StateQuaternion0,StateQuaternion0,StateQuaternion3,StateQuaternion3) = quaternionCovariance;
      covariance.submat(StateVelocityX,StateVelocityX,StateOmegaZ,StateOmegaZ) = twistCovariance;

      // updateVector
      std::vector<int> updateVector(STATE_SIZE,0);
      updateVector[StatePositionX] = 1;
      updateVector[StatePositionY] = 1;
      updateVector[StatePositionZ] = 1;
      updateVector[StateQuaternion0] = 1;
      updateVector[StateQuaternion1] = 1;
      updateVector[StateQuaternion2] = 1;
      updateVector[StateQuaternion3] = 1;
      updateVector[StateVelocityX] = 1;
      updateVector[StateVelocityY] = 1;
      updateVector[StateVelocityZ] = 1;
      updateVector[StateOmegaX] = 1;
      updateVector[StateOmegaY] = 1;
      updateVector[StateOmegaZ] = 1;

      //Enqueuing the IMU measurement in the priority queue.
      FilterCore::SensorMeasurementPtr measurementPtr = FilterCore::SensorMeasurementPtr(new FilterCore::SensorMeasurement);
      measurementPtr->topicName_ = "ODOM";
      measurementPtr->measurement_ = measurement;
      measurementPtr->covariance_ = covariance;
      measurementPtr->updateVector_ = updateVector;
      measurementPtr->time_ = msg->header.stamp.toSec();

      // Adding measurement in the measurement Queue
      addMeasurementinQueue(measurementPtr);

    } // method odom_cb

    // method for changing gps subscription on or off
    void Ekf::parameter_cb(sensor_fusion::fusionConfig &config, uint32_t level) {
      ROS_INFO("Reconfigure Request:%s ",
            config.gps?"True":"False");
            shouldGPS = config.gps?true:false;
    } // method parameter_cb

    // Method for adding measurements into the queue
    void Ekf::addMeasurementinQueue(const FilterCore::SensorMeasurementPtr& measurementPtr){
      measurementPtrQueue_.push(measurementPtr);
    } // method addMeasurementinQueue

    Ekf::~Ekf(){
      ROS_INFO("Destroying the EKF constructor");
    } // Destructor

    // Method for integrating the measurements and updating the state using EKF
    // In this method we are going to integrate and run EKF
    void Ekf::integrateSensorMeasurements(){
      FilterCore::SensorMeasurementPtr measurementPtr;
      while(ros::ok() && !measurementPtrQueue_.empty()){
        // Taking the first measurement from measurement container
        measurementPtr = measurementPtrQueue_.top();
        measurementPtrQueue_.pop();

        // Check if the filter is initialised or not. If not then initialise the filter with measurements and their covariance
        if(filter_.getInitialisedStatus()){
          double delta = measurementPtr->time_ - filter_.getLastMeasurementTime();

          // Asynchronous method for integrating the measurements
          // Delta > 0 ensures that doesnt entertain delayed or out of sequence measurements
          if(delta>0){
            filter_.predict(delta);
            filter_.update(measurementPtr,ros::Time::now().toSec());
          }

        } else {
          // Initialize the filter, but only with the values we're using
          DEBUG("=============================Initialising Filter=============================\n");
          arma::colvec state(STATE_SIZE);
          arma::mat estimateErrorCovariance(STATE_SIZE,STATE_SIZE);
          state.zeros();
          estimateErrorCovariance.eye();
          estimateErrorCovariance *=1e-5;

          // Fill the state vector with measurement
          size_t measurementLength = measurementPtr->updateVector_.size();
          for (size_t i = 0; i < measurementLength; ++i)
          {
            state[i] = (measurementPtr->updateVector_[i] ? measurementPtr->measurement_[i] : state(i));
          }

          // Same for covariance
          for (size_t i = 0; i < measurementLength; ++i)
          {
            for (size_t j = 0; j < measurementLength; ++j)
            {
              estimateErrorCovariance(i, j) = (measurementPtr->updateVector_[i] && measurementPtr->updateVector_[j] ?
                                                measurementPtr->covariance_(i, j) :
                                                estimateErrorCovariance(i, j));
            }
          }

          // Set the filter state and covariances with the initialised values
          filter_.setState(state);
          filter_.setEstimateErrorCovariance(estimateErrorCovariance);
          filter_.setLastMeasurementTime(measurementPtr->time_);
          filter_.setInitialisedStatus(true);
        }
      }
    }// integrateSensorMeasurements


    void Ekf::getFusedState( nav_msgs::Odometry& msg) const {
      arma::colvec state = filter_.getState();
      const arma::mat& covariance = filter_.getProcessNoiseCovariance();
      msg.header.frame_id = "odom";
      msg.child_frame_id = "quad";
      msg.header.stamp = ros::Time(filter_.getLastFilterTime());
      msg.pose.pose.position.x = state(StatePositionX);
      msg.pose.pose.position.y = state(StatePositionY);
      msg.pose.pose.position.z = state(StatePositionZ);
      msg.pose.pose.orientation.x = state(StateQuaternion0);
      msg.pose.pose.orientation.y = state(StateQuaternion1);
      msg.pose.pose.orientation.z = state(StateQuaternion2);
      msg.pose.pose.orientation.w = state(StateQuaternion3);
      msg.twist.twist.linear.x = state(StateVelocityX);
      msg.twist.twist.linear.y = state(StateVelocityY);
      msg.twist.twist.linear.z = state(StateVelocityZ);
      msg.twist.twist.angular.x = state(StateOmegaX);
      msg.twist.twist.angular.y = state(StateOmegaY);
      msg.twist.twist.angular.z = state(StateOmegaZ);

      /* Uncomment the following for printing Euler Angles */ // @TODO some memory issues while printing
      // tf2::Quaternion q(state(StateQuaternion0),state(StateQuaternion1),state(StateQuaternion2),state(StateQuaternion3));
      // tf2::Matrix3x3 mat(q);
      // tf2Scalar yaw, roll, pitch = 0;
      // mat.getEulerYPR(yaw,pitch,roll);
      // ROS_INFO_STREAM("\nROLL: " << roll*180/PI << "\npitch: " << pitch*180/PI << "\nyaw: " << yaw*180/PI << endl);

      // ROS_INFO_STREAM("msg in getFusedState" << endl << state << endl);
      // for(int i=0; i<covariance.size(); i++){
      //   //@TODO convert the quaternion covariance to euler angles covariance.
      //   msg->pose.pose.covariance[i] = covariance[i];
      // }
      for(int i=0; i<6; i++){

      }


    } // getFusedState

    void Ekf::loadParams(){
      nhPriv_.param("debug_mode",isDebugMode_,false);
      nhPriv_.param("remove_gravity",removeGravititionalAcceleration_,true);
      filter_.setDebugStatus(isDebugMode_);
      // Load up the process noise covariance (from the launch file/parameter server)
      arma::mat processNoiseCovariance(3,3);
      processNoiseCovariance.zeros();
      XmlRpc::XmlRpcValue processNoiseCovarConfig;
      if (nhPriv_.hasParam("process_noise_covariance"))
      {

      }

    }// method loadParams
  }// namespace FilterCore
}// namespace Fusion
