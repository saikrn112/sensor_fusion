#ifndef FUSION_HPP
#define FUSION_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <sensor_fusion/filter_core.h>
#include <sensor_fusion/defs.hpp>
#include <sensor_fusion/conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <XmlRpcException.h>

#include <armadillo>

namespace Fusion{
  namespace RosIntegration{
    class Ekf  {
    private:
      int placeHolder_;
      // Local NodeHandle
      ros::NodeHandle nh_;
      ros::NodeHandle nhPriv_;
      // Sensor Callbacks
      void imu_cb(const sensor_msgs::Imu::ConstPtr& ); // Imu Callback
      void gps_cb(const sensor_msgs::NavSatFix::ConstPtr& ); // GPS callback
      void odom_cb(const nav_msgs::Odometry::ConstPtr& ); // Odometry Callback
      // Subscribers and Publisher objects
      ros::Subscriber imuSub, odomSub, gpsSub;
      ros::Publisher pub;
      FilterCore::EkfCore filter_;
      bool isDebugMode_;
      bool removeGravititionalAcceleration_;
      FilterCore::SensorMeasurementPtrQueue measurementPtrQueue_;
      bool isGPSFirstMeasurement_;
      double initialLatInNED_;
      double initiallonInNED_;
    public:

      Ekf(ros::NodeHandle*); // Constructor
      ~Ekf(); //Destructor

      // The following functions should update the corresponding state with the
      // specified time stamp
      void addMeasurementinQueue(const FilterCore::SensorMeasurementPtr&);

      // This function job is to initialise the filter with mentionend covariances
      // and also check for the initial measurements
      void initialize(void);
      void loadParams(void);
      void getFusedState( nav_msgs::Odometry& ) const;
      // This function should integrate the measurements
      void integrateSensorMeasurements(void);
      // Kalman Filter predict and Update functions
    };

  }// RosIntegration
} // Fusion
#endif
