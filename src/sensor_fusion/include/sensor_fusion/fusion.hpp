#ifndef FUSION_HPP
#define FUSION_HPP

/* Contains ROS nodehandlers and other essential methods*/
#include <ros/ros.h>
/* ROS message header files */
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
/* Sensor_fusion header files */
#include <sensor_fusion/filter_core.h>
#include <sensor_fusion/defs.hpp>
#include <sensor_fusion/conversions.h>

/* Transformation related header files for rotating accelerations */
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

/* For getting and parsing parameters from parameter yaml files */
#include <XmlRpcException.h>

/* Armadillo Linear Algebra Library [refer: http://arma.sourceforge.net/docs.html ]*/
#include <armadillo>

/* Start of Fusion Namespace */
namespace Fusion{
  /* ROSIntegration namespace contains methods only related to ROS Subscribers, Publisher and Call backs*/
  namespace RosIntegration{

    /** This class contains all the callback methods publihers and
     *  subscribers
     */
    class Ekf  {
    private:

      // Nodehandler for Subscribers and Publisher
      ros::NodeHandle nh_;

      // Private Nodehandler for getting parameters from yaml file
      ros::NodeHandle nhPriv_;

      // Sensor Callbacks
      /** Imu Callback
       * Takes IMU Measurements - Accelerations, Quaternion and Angular velocities.
       * Rotates Acceleration due to gravity to body frame and remove components due to gravity
       * Sets the Measurement values and their covariances and finally sends the pointer to addMeasurementinQueue
       */
      void imu_cb(const sensor_msgs::Imu::ConstPtr& );

      /** GPS Callback
       * Takes GPS Measurements - Latitude, Longitude and Altitude
       * Converts them to UTM coordinate system and translates it to initial position
       * Sets the Measurement values and their covariances and finally sends the pointer to addMeasurementinQueue
       */
      void gps_cb(const sensor_msgs::NavSatFix::ConstPtr& );

      /** Odometry Callback
       * Takes values from Odometry Message - Pose(Position and Quaternion) and Twist(Linear and Angular velocity)
       * Sets the Measurement values and their covariances and finally sends the pointer to addMeasurementinQueue
       */
      void odom_cb(const nav_msgs::Odometry::ConstPtr& );

      // Subscribers and Publisher objects
      ros::Subscriber imuSub, odomSub, gpsSub;
      ros::Publisher pub;

      // Filter related Objects
      /** Object that contains the essential methods of Ekf
       *  Derives it from FilterCore namespace and EkfCore class
       */
      FilterCore::EkfCore filter_; //object for prediction and update

      /** Object that contains that takes the typedefed priority Queue measurement container
       *  Derives it from FilterCore namespace and SensorMeasurement struct with boost::shared_ptr wrapper
       */
      FilterCore::SensorMeasurementPtrQueue measurementPtrQueue_; // object for priority Sensor Measurement Queue container

      /* Class Members */

      // Checks debugMode from parameter server. Used for printing matrices and essential vectors (BEWARE!! LOT OF OUTPUT)
      bool isDebugMode_;

      // Parameter for remove Acceleration due to gravity. Accordingly it rotates and removes gravity from measurement
      bool removeGravititionalAcceleration_;

      // @TODO need to check
      bool isGPSFirstMeasurement_;

      // Stores initial latitude for placing coordinate system at initial location
      double initialLatInNED_;

      // Stores initial longitude for placing coordinate system at initial location
      double initialLonInNED_;

      // Stores initial Altitude for placing coordinate system at initial location
      double initialAltitude_;

    public:
      // Constructor
      Ekf(ros::NodeHandle*);

      //Destructor
      ~Ekf();

      // Method for adding measurements from sensors into the priority queue container.
      void addMeasurementinQueue(const FilterCore::SensorMeasurementPtr&);

      // This function job is to initialise the filter with mentionend covariances
      // and also check for the initial measurements
      void initialize(void);

      void loadParams(void);
      void getFusedState( nav_msgs::Odometry& ) const;

      // This function should integrate the measurements
      void integrateSensorMeasurements(void);
    };

  }// End of namespace RosIntegration
} // End of namespace Fusion
#endif // End of FUSION_HPP
