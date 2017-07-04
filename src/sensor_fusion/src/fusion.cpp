#include "sensor_fusion/fusion.hpp"

using namespace std;
using namespace arma;

namespace Fusion{
  namespace RosIntegration{

    void Ekf::imu_cb(const sensor_msgs::Imu& msg){
      ROS_INFO("in Imu call back");


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
    nh_(*nodeHandle)
      {
      ROS_INFO("Got the Node Handle. Initialising IMU, GPS and Odometry Callback");
      imuSub = nh_.subscribe("/imu/data",1,&Ekf::imu_cb,this);
      gpsSub = nh_.subscribe("/navsat/fix",1,&Ekf::gps_cb,this);
      odomSub = nh_.subscribe("/odom",1,&Ekf::odom_cb,this);

      pub = nh_.advertise<nav_msgs::Odometry>("/odomCombined",1,true);
      while(ros::ok()){
        ros::spinOnce();
      }
    }

    Ekf::~Ekf(){
      ROS_INFO("Destroying the EKF constructor");
    }


  }// namespace FilterCore
}// namespace Fusion
