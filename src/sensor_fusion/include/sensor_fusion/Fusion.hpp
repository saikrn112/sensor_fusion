#ifndef FUSION_HPP
#define FUSION_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <armadillo>
#include <include/Defs.hpp>

namespace Fusion{
  class Ekf{
  private:
    int a;
    ros::NodeHandle nh_;
    // Sensor Callbacks
    void imu_cb(const sensor_msgs::Imu& ); // Imu Callback
    void gps_cb(const sensor_msgs::NavSatFix& ); // GPS callback
    void odom_cb(const nav_msgs::Odometry& ); // Odometry Callback
    // Subscribers and Publisher objects
    ros::Subscriber imuSub, odomSub, gpsSub;
    ros::Publisher pub;

    //State Variables vector
    arma::vec<double> state_;
    //Process Covariance
    arma::mat<double> processCovariance(STATE_SIZE,STATE_SIZE);

  public:
    // Constructor
    Ekf(ros::NodeHandle*);
    ~Ekf();
    // Testing function for printing the private value
    void getVal(void);

    bool updateAngularVelocities(const sensor_msgs::Imu& );
    bool updateBodyAccelerations(const sensor_msgs::Imu& );
    bool updateQuaternion(const sensor_msgs::Imu& );
    // Current state vector
    void getCurrentState(void);

    // Previous state vector
    void getPreviousState(void);
  };
}

#endif
