#ifndef LIB_MOTION_PROFILE_MOTION_PROFILE_H_
#define LIB_MOTION_PROFILE_MOTION_PROFILE_H_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include <Eigen/Dense>

#include "lib/physics_structs/physics_structs.h"

namespace lib {
namespace motion_profile {

class MotionProfile {
 public:
  MotionProfile(double max_velocity, double max_acceleration,
                double delta_time);

  ::Eigen::Vector3d Calculate(
      ::Eigen::Vector3d flight_direction);

  void SetOutput(::Eigen::Vector3d output);

 private:
  ::Eigen::Vector3d output_;

  double max_velocity_;
  double max_acceleration_;
  double delta_time_;
};

}  // namespace motion_profile
}  // namespace lib

#endif  // LIB_MOTION_PROFILE_MOTION_PROFILE_H_
