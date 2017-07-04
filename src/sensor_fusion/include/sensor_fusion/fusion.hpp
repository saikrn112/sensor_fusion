#ifndef FUSION_HPP
#define FUSION_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <sensor_fusion/filter_core.h>
#include <sensor_fusion/defs.hpp>
#include <armadillo>

namespace Fusion{
  namespace RosIntegration{


    class Ekf : public Fusion::FilterCore::EkfCore {
    private:
      int a;
      // Local NodeHandle
      ros::NodeHandle nh_;
      // Sensor Callbacks
      void imu_cb(const sensor_msgs::Imu& ); // Imu Callback
      void gps_cb(const sensor_msgs::NavSatFix& ); // GPS callback
      void odom_cb(const nav_msgs::Odometry& ); // Odometry Callback
      // Subscribers and Publisher objects
      ros::Subscriber imuSub, odomSub, gpsSub;
      ros::Publisher pub;

    public:

      Ekf(ros::NodeHandle*); // Constructor
      ~Ekf(); //Destructor

      // The following functions should update the corresponding state with the
      // specified time stamp
      bool updateAngularVelocities(const sensor_msgs::Imu& );
      bool updateBodyAccelerations(const sensor_msgs::Imu& );
      bool updateQuaternion(const sensor_msgs::Imu& , const arma::colvec& state_ );
      void addMeasurementinQueue(const FilterCore::SensorMeasurement&, std::string);

      // This function job is to initialise the filter with mentionend covariances
      // and also check for the initial measurements
      void initialize(void);
      void loadParams(void);

      // This function should integrate the measurements
      void integrate(void);
      // Kalman Filter predict and Update functions
    };

  }// RosIntegration
} // Fusion
#endif
