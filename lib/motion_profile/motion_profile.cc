#include "motion_profile.h"

namespace lib {
namespace motion_profile {

MotionProfile::MotionProfile(double max_velocity, double max_acceleration,
                             double delta_time) :
    output_(0, 0, 0),
    max_velocity_(max_velocity),
    max_acceleration_(max_acceleration),
    delta_time_(delta_time) {}

::Eigen::Vector3d MotionProfile::Calculate(::Eigen::Vector3d flight_direction) {
  // Limit how fast the drone can accelerate.
  ::Eigen::Vector3d initial_output = output_;
  ::Eigen::Vector3d desired_acceleration = flight_direction - output_;

  double desired_acceleration_magnitude =
      ::std::min(desired_acceleration.norm(), max_acceleration_);

  if (desired_acceleration.norm() > 0) {
    desired_acceleration /= desired_acceleration.norm();
  } else {
    desired_acceleration = ::Eigen::Vector3d(0, 0, 0);
  }

  desired_acceleration *=
      desired_acceleration_magnitude; // direction of acceleration*length of
                                      // acceleration vector

  // Apply change to the output.
  output_ += desired_acceleration * delta_time_;
  // the output vector is in terms of velocity

  // Limit the output.
  double desired_speed = ::std::min(output_.norm(), max_velocity_);

  if (output_.norm() > 0) {
    output_ /= output_.norm(); // creates a unit vector
  } else {
    output_ = ::Eigen::Vector3d(0, 0, 0);
  }

  output_ *= desired_speed;
  // make vector have length equal to the desired speed
  // need to calculate the actual acceleration output.norm() >max_velocity_

  /* code added */
  actual_acceleration_ = (output_ - initial_output) / delta_time_;
  /*end of code added */
  return output_;
}

void MotionProfile::SetOutput(::Eigen::Vector3d output) { output_ = output; }

::Eigen::Vector3d MotionProfile::GetActualAcceleration() const {

  return actual_acceleration_;
}
} // namespace motion_profile
} // namespace lib