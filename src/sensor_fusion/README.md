# ToDo List
## Completed
- [x] practising and loading parameters from file
- [x] implementing the same in the code
- [x] output operator overload for matrices and vectors of mat and colvec
- [x] adding Measurements
- [x] Interpreting the Measurements
- [x] preprocessing IMU data
- [x] checking git new behaviour
- [x] make sure that q0 is the scalar (basically same )
- [x] priorityMeasurementQueue variable
- [x] print matrices and vectors coming from IMU message
- [x] methods for process and integrate Measurements
- [x] collect bagfiles of imu/data gps/fix from husky simulator for decreasing the load
- [x] Enqueue the message and check its validity in the EkfCore Predict and Update measurement
- [x] Update method update (:P)
- [x] test the IMU data SINS
- [x] check normal odom data from husky simulator
- [x] check combination of both odom and SINS

## Very urgent
- [x] complete documentation literature
- [x] complete comments
- [x] simulate GPS loss for 10 seconds using dynamic parameter or using ROS time or we can use parameter to control ROS timer
- [x] write code for moving the robot in a big circle.
- [x] experiments - screenshots and plots; screenshots should have position of GPS loss and time for which it was lost
  1. [x] circle - rl:with odometry, gps and IMU, fusion- imu and gps alone
  1. [x] straight line - fusion, IMU alone, gps alone(from rl) and gps + IMU
  1. [x] circle - gpsloss fusion alone with RL
  1. [x] straight line - fusion GPS loss no RL
- [x] convert bagfiles to csv files
  1. [x] odometry/filtered_map = ground truth
  1. [x] odomCombined
- [] plot the graphs from the csv files. Pain Sahfeeq!!!!!!
- [x] indicate markings in screenshot like gps loss etc
- [x] update the doc

## YetToDo
- [x] converting GPS lat/long to NED positions using the library that was used in Team Abhiyaan
- [x] check the error accumulation (characterise!!) with robot localisation
- [ ] check the scan matching with this code
- [ ] collect the bag files of the real sensor data with GPS, IMU and laser Scan
  1. [ ] Accelerations
  1. [ ] Angular Velocities
  1. [ ] Magnetic fields
  1. [ ] Quaternions or Euler angles
  1. [ ] Raw Encoder data
  1. [ ] Laser Scans or Point Cloud
  1. [ ] Latitude and Longitude if possible correct altitude :D
- [ ] Covariance mahima -[conversionCode](http://www.mira-project.org/MIRA-doc/YawPitchRoll_8h_source.html)
  1. [ ] Converting 3x3 covaraince to 4x4 covariance (euler angles covariance to quaternion covariance) line - 201
  1. [ ] convariance 4x4 covariance to 3x3 covariance (quaternion to euler angles covariance) line - 318
- [ ] talk to shrikant sir about 4x4 quaternion covariance or derive the jacobian and get the final matrices
- [ ] Correct Noise variances as input both sensor measurements(R) and proecss noise (Q)
  1. [ ] Identify the sensors
  1. [ ] Check their Biases and Variances
  1. [ ] tune those hectice process Noise
- [ ] integrate Magnetic measurements


checking if gitlab is working or not
