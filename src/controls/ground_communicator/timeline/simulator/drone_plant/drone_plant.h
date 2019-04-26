#pragma once

#include "lib/battery/battery.h"
#include <Eigen/Dense>

#include "lib/mission_manager/mission_commands.pb.h"
#include "lib/motion_profile/motion_profile.h"
#include "lib/physics_structs/physics_structs.h"

// using protobuf messages defined in this namespace
using namespace lib::mission_manager;

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
  DronePlant(lib::Position3D init_position, double loop_frequency,
             Battery battery);

  void MoveDrone(Vector3D flight_direction);
  lib::Position3D position() const { return position_; }
  Battery battery() const { return battery_; }
  double
  GetFlightTime() const; // returns the total flight time in seconds (calculated
                         // until the battery reaches drop off)
 private:
  lib::Position3D position_;
  Battery battery_;

  ::lib::motion_profile::MotionProfile profile_;
  double flight_time_;
};

} // namespace drone_plant
} // namespace simulator
} // namespace timeline
} // namespace ground_server
} // namespace controls
} // namespace src