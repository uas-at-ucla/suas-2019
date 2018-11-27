#include "drone_plant.h"

namespace src {
namespace controls {
namespace ground_server {
namespace timeline {
namespace simulator {
namespace drone_plant {

DronePlant::DronePlant(Position3D init_position, double loop_frequency) :
    drone_position_(init_position),
    loop_frequency_(loop_frequency) {}

void DronePlant::MoveDrone(Vector3D flight_direction) {
  drone_position_.latitude +=
      flight_direction.x / loop_frequency_ / kMetersPerCoordinate;
  drone_position_.longitude +=
      flight_direction.y / loop_frequency_ / kMetersPerCoordinate;
  drone_position_.altitude -= flight_direction.z / loop_frequency_;
}

Position3D DronePlant::GetPosition() { return drone_position_; }

double DronePlant::GetLoopFrequency() { return loop_frequency_; }

} // namespace drone_plant
} // namespace simulator
} // namespace timeline
} // namespace ground_server
} // namespace controls
} // namespace src
