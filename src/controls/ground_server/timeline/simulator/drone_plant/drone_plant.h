#pragma once

#include <Eigen/Dense>

#include "lib/motion_profile/motion_profile.h"
#include "lib/physics_structs/physics_structs.h"

namespace src {
namespace controls {
namespace ground_server {
namespace timeline {
namespace simulator {
namespace drone_plant {
namespace {
const double kMetersPerCoordinate = GetDistance2D({0, 0, 0}, {1, 0, 0});
} // namespace

class DronePlant {
 public:
  DronePlant(Position3D init_position, double loop_frequency);

  void MoveDrone(Vector3D flight_direction);
  Position3D position() { return position_; }

 private:
  Position3D position_;
  ::lib::motion_profile::MotionProfile profile_;
};

} // namespace drone_plant
} // namespace simulator
} // namespace timeline
} // namespace ground_server
} // namespace controls
} // namespace src
