#ifndef FUSION_HPP
#define FUSION_HPP

#include <sensor_fusion/State.hpp>
#include <sensor_fusion/Defs.hpp>

namespace Fusion{
  class Ekf{
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
    // Constructor
    Ekf(ros::NodeHandle*);
    ~Ekf();
    // Testing function for printing the private value
    void getVal(void);

    // The following functions should update the corresponding state with the
    // specified time stamp
    bool updateAngularVelocities(const sensor_msgs::Imu& );
    bool updateBodyAccelerations(const sensor_msgs::Imu& );
    bool updateQuaternion(const sensor_msgs::Imu& , State state_ );
    // Current state vector
    void getCurrentState(void);

    // Previous state vector
    void getPreviousState(void);


    // This function job is to initialise the filter with mentionend covariances
    // and also check for the initial measurements
    void initialize(void);

    // This function should synchronize the measurements before passing on to the Filter
    void synchronize(void);

    // This function should integrate the measurements
    void integrate(void);
    // Kalman Filter predict and Update functions
    void predict(void); // Predict function till the next time step.
    void update(void); // Update the measurement once we measure the data.
  };
}

#endif
