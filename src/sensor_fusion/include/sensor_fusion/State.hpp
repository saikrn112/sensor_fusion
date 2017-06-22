#ifndef STATE_HPP
#define STATE_HPP
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion .h>

namespace Fusion{
  class State{
    ros::Time time;
    int size ;
    double pe;
    double pn;
    double pd;
    double ve;
    double vn;
    double vd;
    tf2::Quaternion quat;
    double da_x;
    double da_y;
    double da_z;

  };
}

#endif
