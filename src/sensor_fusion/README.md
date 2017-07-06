# ToDo List

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

- [ ] Converting 3x3 covaraince to 4x4 covariance (euler angles to quaternion)
- [ ] talk to shrikant sir about 4x4 quaternion covariance or derive the jacobian and get the final matrices
- [ ] convariance matrix for getFusedState
- [ ] converting GPS lat/long to NED positions using the library that was used in Team Abhiyaan
- [ ] collect bagfiles of imu/data gps/fix from husky simulator for decreasing the load
- [ ] Enqueue the message and check its validity in the EkfCore Predict and Update measurement
