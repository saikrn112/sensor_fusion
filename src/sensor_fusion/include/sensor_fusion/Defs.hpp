#ifndef DEFS_HPP
#define DEFS_HPP

namespace Fusion{
  const int POSITION_SIZE = 3;
  const int VELOCITY_SIZE = 3;
  const int QUAT_SIZE = 4;
  const int BIAS_SIZE = 3;
  const int STATE_SIZE = POSITION_SIZE + VELOCITY_SIZE + QUAT_SIZE + BIAS_SIZE;

  // Frequently Used Constants
  const double PI = 3.141592653589793;
  const double G = 9.80665; // m/s2
  const double TAU = 6.283185307179587;
}


#endif
