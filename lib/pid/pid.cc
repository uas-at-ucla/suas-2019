#include "pid.h"

namespace lib {
namespace pid {

class PIDImpl {
 public:
  PIDImpl(double dt, double max, double min, double Kp, double Kd, double Ki,
          double imax);
  ~PIDImpl();
  double calculate(double setpoint, double pv);

 private:
  double _dt;
  double _max;
  double _min;
  double _Kp;
  double _Kd;
  double _Ki;
  double _imax;
  double _pre_error;
  double _integral;
};

PID::PID(double dt, double max, double min, double Kp, double Kd, double Ki,
         double imax) {
  pimpl = new PIDImpl(dt, max, min, Kp, Kd, Ki, imax);
}
double PID::calculate(double setpoint, double pv) {
  return pimpl->calculate(setpoint, pv);
}
PID::~PID() { delete pimpl; }

/**
 * Implementation
 */
PIDImpl::PIDImpl(double dt, double max, double min, double Kp, double Kd,
                 double Ki, double imax) :
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _imax(imax),
    _pre_error(0),
    _integral(0) {}

double PIDImpl::calculate(double setpoint, double pv) {

  // Calculate error
  double error = setpoint - pv;

  // Proportional term
  double Pout = _Kp * error;

  // Integral term
  _integral += error * _dt;
  double Iout = _Ki * _integral;
  Iout = ::std::max(::std::min(_imax, Iout), -_imax);

  // Derivative term
  double derivative = (error - _pre_error) / _dt;
  double Dout = _Kd * derivative;

  // Calculate total output
  double output = Pout + Iout + Dout;

  // Restrict to max/min
  if (output > _max)
    output = _max;
  else if (output < _min)
    output = _min;

  // Save error to previous error
  _pre_error = error;

  return output;
}

PIDImpl::~PIDImpl() {}

} // namespace pid
} // namespace lib
