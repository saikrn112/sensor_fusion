#include "sensor_fusion/fusion.hpp"

using namespace std;
using namespace arma;

namespace Fusion{
  namespace RosIntegration{

    void Ekf::imu_cb(const sensor_msgs::Imu& msg){
      ROS_INFO("in Imu call back");
      FilterCore::SensorMeasurementPtr measurement;
      //@
    }

    void Ekf::gps_cb(const sensor_msgs::NavSatFix& msg){
      // addMeasurementinQueue(msg, ros::time::Now());
      ROS_INFO("in gps call back");
    }

    void Ekf::odom_cb(const nav_msgs::Odometry& msg){
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
