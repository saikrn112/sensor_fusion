#include "sensor_fusion/fusion.hpp"

using namespace std;
using namespace arma;

namespace Fusion{
  namespace RosIntegration{

    void Ekf::imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
      ROS_INFO("in Imu call back");
      FilterCore::SensorMeasurementPtr measurement = FilterCore::SensorMeasurementPtr(new FilterCore::SensorMeasurement);
      //@TODO convert 3x3 covariance to 4x4 covariance. Temporarily filling it with identity
      //@TODO confirm that all the covariance matrices are just diagonal matrix;
      //@TODO if they are just constants why not just hard code or parameterise them instead of reading from msgs;
      arma::mat anglesCovariance(ANGLES_SIZE,ANGLES_SIZE);
      anglesCovariance.eye();
      anglesCovariance(0,0) = msg->orientation_covariance[0];
      accelerationCovariance(1,1) = msg->linear_acceleration_covariance[4];
      anglesCovariance(1,1) = msg->orientation_covariance[4];
      anglesCovariance(2,2) = msg->orientation_covariance[8];
      arma::mat quaternionCovariance(QUAT_SIZE,QUAT_SIZE);
      quaternionCovariance.eye();
      arma::mat accelerationCovariance(ACCELERATION_SIZE,ACCELERATION_SIZE);
      accelerationCovariance.eye();
      accelerationCovariance(0,0) = msg->linear_acceleration_covariance[0];
      accelerationCovariance(2,2) = msg->linear_acceleration_covariance[8];
      arma::mat omegaCovariance(OMEGA_SIZE,OMEGA_SIZE);
      omegaCovariance.eye();
      omegaCovariance(0,0) = msg->angular_velocity_covariance[0];
      omegaCovariance(1,1) = msg->angular_velocity_covariance[4];
      omegaCovariance(2,2) = msg->angular_velocity_covariance[8];

      //Enqueuing the IMU measurement in the priority queue.
      measurement->topicName_ = "IMU";
      measurement->measurement_(StateQuaternion0) = msg->orientation.x; // @TODO make sure that q0 is scalar
      measurement->measurement_(StateQuaternion1) = msg->orientation.y;
      measurement->measurement_(StateQuaternion2) = msg->orientation.z;
      measurement->measurement_(StateQuaternion3) = msg->orientation.w;
      measurement->measurement_(StateAcclerationX) = msg->linear_acceleration.x;
      measurement->measurement_(StateAcclerationY) = msg->linear_acceleration.y;
      measurement->measurement_(StateAcclerationZ) = msg->linear_acceleration.z;
      measurement->measurement_(StateOmegaX) = msg->angular_velocity.x;
      measurement->measurement_(StateOmegaY) = msg->angular_velocity.y;
      measurement->measurement_(StateOmegaZ) = msg->angular_velocity.z;

      measurement->covariance_.submat(StateQuaternion0,StateQuaternion0,StateQuaternion3,StateQuaternion3) = quaternionCovariance;
      measurement->covariance_.submat(StateAcclerationX,StateAcclerationX,StateAcclerationZ,StateAcclerationZ) = accelerationCovariance;
      measurement->covariance_.submat(StateOmegaX,StateOmegaX,StateOmegaZ,StateOmegaZ) = omegaCovariance;
      
    }

    void Ekf::gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
      // addMeasurementinQueue(msg, ros::time::Now());
      ROS_INFO("in gps call back");
    }

    void Ekf::odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
      // addMeasurementinQueue(msg, ros::time::Now());
      ROS_INFO("in odom call back");
    }

    Ekf::Ekf(ros::NodeHandle* nodeHandle):
    nh_(*nodeHandle),
    nhPriv_("~")
      {
      ROS_INFO("Got the Node Handle. Initialising IMU, GPS and Odometry Callback");
      imuSub = nh_.subscribe("/imu/data",1,&Ekf::imu_cb,this);
      gpsSub = nh_.subscribe("/navsat/fix",1,&Ekf::gps_cb,this);
      odomSub = nh_.subscribe("/odom",1,&Ekf::odom_cb,this);

      pub = nh_.advertise<nav_msgs::Odometry>("/odomCombined",1,true);
      Ekf::loadParams();
      while(ros::ok()){
        ros::spinOnce();
      }
    }

    Ekf::~Ekf(){
      ROS_INFO("Destroying the EKF constructor");
    }

    void Ekf::loadParams(){

      nhPriv_.param("a",placeHolder_);
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
