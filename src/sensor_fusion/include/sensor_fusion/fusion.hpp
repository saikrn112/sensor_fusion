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


#include <dynamic_reconfigure/server.h>
#include <sensor_fusion/fusionConfig.h>

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

      /** Parameter Callback
       *  used to reconfigure the gps topic dynamically
       *  Parameter- fusionConfig - takes the configuration file defined previously
       *  Parameter - unint23_t is defined for level
       */
      void parameter_cb(sensor_fusion::fusionConfig& , uint32_t );

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

      // This member is used to load initial Latitude, Longitude and altitude. which can be used to translation
      bool isGPSFirstMeasurement_;

      // Stores initial latitude for placing coordinate system at initial location
      double initialLatInNED_;

      // Stores initial longitude for placing coordinate system at initial location
      double initialLonInNED_;

      // Stores initial Altitude for placing coordinate system at initial location
      double initialAltitude_;

      // Dynamic parameter for unlocking and locking GPS
      bool shouldGPS;

    public:
      // Constructor
      /** This constructor takes the node Handle and registers IMU, GPS and Odom Callback with rosmaster
       *  This constructor will also load params using method loadParams()
       *  Initialises the filter LastMeasurementTime, LastFilter time and state vector
       *  Integrates sensor measurements using method integrateSensorMeasurements()
       *  Gets the fused state from filter using getFusedState() and publishes it under /odomCombined topic
       */
      Ekf(ros::NodeHandle*);

      //Destructor
      ~Ekf();

      // Method for adding measurements from sensors into the priority queue container.
      // This simply pushes the measurment pointers into measurementPtrQueue_ member
      void addMeasurementinQueue(const FilterCore::SensorMeasurementPtr&);

      // Loads parmeters from parameter server.
      void loadParams(void);

      // gets Fused State from filter_ and loads it into nav_msgs::Odometry&
      void getFusedState( nav_msgs::Odometry& ) const;

      // This function should integrate the measurements
      // Initialises the filter state and covariance with one measurement
      // takes care of asequent observations and does prediction and Update step
      void integrateSensorMeasurements(void);
    };

  }// End of namespace RosIntegration
} // End of namespace Fusion
#endif // End of FUSION_HPP
