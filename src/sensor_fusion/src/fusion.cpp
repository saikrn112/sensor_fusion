#include "sensor_fusion/fusion.hpp"

using namespace std;
using namespace arma;

namespace Fusion{
  namespace RosIntegration{

    Ekf::Ekf(ros::NodeHandle* nodeHandle):
    nh_(*nodeHandle),
    nhPriv_("~")
      {
      ROS_INFO("Got the Node Handle. Initialising IMU, GPS and Odometry Callback");
      imuSub = nh_.subscribe("/imu/data",1,&Ekf::imu_cb,this);
      // gpsSub = nh_.subscribe("/navsat/fix",1,&Ekf::gps_cb,this);
      // odomSub = nh_.subscribe("/odom",1,&Ekf::odom_cb,this);

      pub = nh_.advertise<nav_msgs::Odometry>("/odomCombined",1,true);
      Ekf::loadParams();
      nav_msgs::Odometry fusedState;
      filter_.setLastMeasurementTime(ros::Time::now().toSec());
      filter_.setLastFilterTime(ros::Time::now().toSec());
      while(ros::ok()){
        integrateSensorMeasurements();
        getFusedState(fusedState);
        ros::spinOnce();
      }
    } // Constructor

    void Ekf::imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
      ROS_INFO("in Imu call back");
      // //@TODO convert 3x3 covariance to 4x4 covariance. Temporarily filling it with identity
      // //@TODO confirm that all the covariance matrices are just diagonal matrix;
      // //@TODO if they are just constants why not just hard code or parameterise them instead of reading from msgs;
      arma::mat anglesCovariance(ANGLES_SIZE,ANGLES_SIZE);
      anglesCovariance.eye();
      anglesCovariance(0,0) = msg->orientation_covariance[0];
      anglesCovariance(1,1) = msg->orientation_covariance[4];
      anglesCovariance(2,2) = msg->orientation_covariance[8];


      arma::mat quaternionCovariance(QUAT_SIZE,QUAT_SIZE);
      quaternionCovariance.eye();

      arma::mat accelerationCovariance(ACCELERATION_SIZE,ACCELERATION_SIZE);
      accelerationCovariance.eye();
      accelerationCovariance(0,0) = msg->linear_acceleration_covariance[0];
      accelerationCovariance(1,1) = msg->linear_acceleration_covariance[4];
      accelerationCovariance(2,2) = msg->linear_acceleration_covariance[8];

      arma::mat omegaCovariance(OMEGA_SIZE,OMEGA_SIZE);
      omegaCovariance.eye();
      omegaCovariance(0,0) = msg->angular_velocity_covariance[0];
      omegaCovariance(1,1) = msg->angular_velocity_covariance[4];
      omegaCovariance(2,2) = msg->angular_velocity_covariance[8];

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
        // tf2::Quaternion q(static_cast<tf2Scalar>(measurement(StateQuaternion0)),static_cast<tf2Scalar>(measurement(StateQuaternion1)),static_cast<tf2Scalar>(measurement(StateQuaternion2)),static_cast<tf2Scalar>(measurement(StateQuaternion3)));
        tf2::Quaternion q(measurement(StateQuaternion0),measurement(StateQuaternion1),measurement(StateQuaternion2),measurement(StateQuaternion3));
        tf2::Matrix3x3 rotMat(q);
        tf2::Vector3 a = rotMat*gravity;
        measurement(StateAcclerationX) -= a.getX();
        measurement(StateAcclerationY) -= a.getY();
        measurement(StateAcclerationZ) -= a.getZ();
      }

      arma::mat covariance(STATE_SIZE,STATE_SIZE);
      covariance.eye();
      covariance.submat(StateQuaternion0,StateQuaternion0,StateQuaternion3,StateQuaternion3) = quaternionCovariance;
      covariance.submat(StateAcclerationX,StateAcclerationX,StateAcclerationZ,StateAcclerationZ) = accelerationCovariance;
      covariance.submat(StateOmegaX,StateOmegaX,StateOmegaZ,StateOmegaZ) = omegaCovariance;

      //Enqueuing the IMU measurement in the priority queue.
      FilterCore::SensorMeasurementPtr measurementPtr = FilterCore::SensorMeasurementPtr(new FilterCore::SensorMeasurement);
      measurementPtr->topicName_ = "IMU";
      measurementPtr->measurement_ = measurement;
      measurementPtr->covariance_ = covariance;
      measurementPtr->time_ = msg->header.stamp.toSec();
      if(isDebugMode_){
        ROS_INFO_STREAM("measurement_topic: " << measurementPtr->topicName_ << endl
                    <<  "measurements: " << endl<< measurementPtr->measurement_ << endl
                    <<  "covariances: " << endl << measurementPtr->covariance_ << endl);
      }
      addMeasurementinQueue(measurementPtr);
    } // method imu_cb

    void Ekf::gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
      // addMeasurementinQueue(msg, ros::time::Now());
      ROS_INFO("in gps call back");
    } // method gps_cb

    void Ekf::odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
      // addMeasurementinQueue(msg, ros::time::Now());
      ROS_INFO("in odom call back");
    } // method odom_cb

    void Ekf::addMeasurementinQueue(const FilterCore::SensorMeasurementPtr& measurementPtr){
      measurementPtrQueue_.push(measurementPtr);
      // FilterCore::SensorMeasurementPtr debug = measurementPtrQueue_.top();
      // ROS_INFO_STREAM("debug state:" << debug->measurement_ << endl);
    } // method addMeasurementinQueue

    Ekf::~Ekf(){
      ROS_INFO("Destroying the EKF constructor");
    } // Destructor
    void Ekf::integrateSensorMeasurements(){
      // In this method we are going to integrate and predict the state
      // for that we need to

      FilterCore::SensorMeasurementPtr measurementPtr;
      while(ros::ok() && !measurementPtrQueue_.empty()){
        measurementPtr = measurementPtrQueue_.top();
        ROS_INFO_STREAM("measurements using for integration:" << endl
                    << "measurement_topic: " << measurementPtr->topicName_ << endl
                    <<  "measurements: " << endl<< measurementPtr->measurement_ << endl
                    <<  "covariances: " << endl << measurementPtr->covariance_ << endl);
      }
      // measurementPtrQueue_.pop();
      // double delta = measurementPtr->time_ - filter_.getLastMeasurementTime();
      // filter_.predict(delta);
      // filter_.update(measurementPtr);
    }// integrateSensorMeasurements


    void Ekf::getFusedState( nav_msgs::Odometry& msg){
      arma::colvec state = filter_.getState();
      const arma::mat& covariance = filter_.getProcessNoiseCovariance();
      msg.header.frame_id = "odom";
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

      // for(int i=0; i<covariance.size(); i++){
      //   //@TODO convert the quaternion covariance to euler angles covariance.
      //   msg->pose.pose.covariance[i] = covariance[i];
      // }
      for(int i=0; i<6; i++){

      }


    } // getFusedState
    void Ekf::loadParams(){

      nhPriv_.param("a",placeHolder_);
      nhPriv_.param("debug_mode",isDebugMode_,false);
      nhPriv_.param("remove_gravity",removeGravititionalAcceleration_,true);
      ROS_INFO_STREAM("debug_mode" << isDebugMode_);
      // Load up the process noise covariance (from the launch file/parameter server)
      arma::mat processNoiseCovariance(3,3);
      processNoiseCovariance.zeros();
      XmlRpc::XmlRpcValue processNoiseCovarConfig;
      if (nhPriv_.hasParam("process_noise_covariance"))
      {
        try
        {
          nhPriv_.getParam("process_noise_covariance", processNoiseCovarConfig);
          ROS_ASSERT(processNoiseCovarConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

          int matSize = processNoiseCovariance.n_rows;

          for (int i = 0; i < matSize; i++)
          {
            for (int j = 0; j < matSize; j++)
            {
              try
              {
                // These matrices can cause problems if all the types
                // aren't specified with decimal points. Handle that
                // using string streams.
                std::ostringstream ostr;
                ostr << processNoiseCovarConfig[matSize * i + j];
                std::istringstream istr(ostr.str());
                istr >> processNoiseCovariance(i, j);
              }
              catch(XmlRpc::XmlRpcException &e)
              {
                throw e;
              }
              catch(...)
              {
                throw;
              }
            }
          }
          processNoiseCovariance.print();
          // RF_DEBUG("Process noise covariance is:\n" << processNoiseCovariance << "\n");
        }
        catch (XmlRpc::XmlRpcException &e)
        {
          ROS_ERROR_STREAM("ERROR reading sensor config: " <<
                           e.getMessage() <<
                           " for process_noise_covariance (type: " <<
                           processNoiseCovarConfig.getType() << ")");
        }
        // @TODO comment it out when doen testing
        // filter_.setProcessNoiseCovariance(processNoiseCovariance);
      }
    }// method loadParams


  }// namespace FilterCore
}// namespace Fusion
