#pragma once

#include "drone_plant.h"

#include <Eigen/Dense>

namespace src {
namespace controls {
namespace ground_server {
namespace timeline {
namespace simulator {
namespace drone_plant {

class DronePlant {
 public:
  DronePlant(Position3D init_position, double loop_frequency);

  void MoveDrone(Vector3D flight_direction);
  Position3D GetPosition() { return drone_position_; }
  double GetLoopFrequency() { return loop_frequency_; }

 private:
  Position3D drone_position_;
  double loop_frequency_;
};

} // namespace drone_plant
} // namespace simulator
} // namespace timeline
} // namespace ground_server
} // namespace controls
} // namespace src
