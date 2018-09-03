#pragma once

#include <cmath>
#include <iostream>
#include <queue>
#include <vector>

using namespace std;

namespace lib {
namespace pid {

class PIDImpl;
class PID {
 public:
  // Kp -  proportional gain
  // Ki -  Integral gain
  // Kd -  derivative gain
  // dt -  loop interval time
  // max - maximum value of manipulated variable
  // min - minimum value of manipulated variable
  PID(double dt, double max, double min, double Kp, double Kd, double Ki,
      double imax);

  // Returns the manipulated variable given a setpoint and current process value
  double calculate(double setpoint, double pv);
  ~PID();

 private:
  PIDImpl *pimpl;
};

} // namespace pid
} // namespace lib
