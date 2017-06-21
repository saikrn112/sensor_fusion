#include "sensor_fusion/Fusion.hpp"

using namespace std;
using namespace arma;

void Fusion::Ekf::imu_cb(const sensor_msgs::Imu& msg){
  ROS_INFO("in Imu call back");
}

void Fusion::Ekf::gps_cb(const sensor_msgs::NavSatFix& msg){
  ROS_INFO("in gps call back");
}

void Fusion::Ekf::odom_cb(const nav_msgs::Odometry& msg){
  ROS_INFO("in odom call back");
}

Fusion::Ekf::Ekf(ros::NodeHandle* nodeHandle):nh_(*nodeHandle){
  ROS_INFO("Got the Node Handle. Initialising IMU, GPS and Odometry Callback");
  imuSub = nh_.subscribe("/imu",1,&Fusion::Ekf::imu_cb,this);
  gpsSub = nh_.subscribe("/fix",1,&Fusion::Ekf::gps_cb,this);
  odomSub = nh_.subscribe("/odom",1,&Fusion::Ekf::odom_cb,this);

  pub = nh_.advertise<nav_msgs::Odometry>("/odomCombined",1,true);
}

Fusion::Ekf::~Ekf(){
  ROS_INFO("Destroying the constructor");
}

void Fusion::Ekf::getVal(void ){
  cout << "Value of a: " << a << endl;
}
