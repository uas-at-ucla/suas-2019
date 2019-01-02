#include "drone_plant.h"

namespace src {
namespace controls {
namespace ground_server {
namespace timeline {
namespace simulator {
namespace drone_plant {

DronePlant::DronePlant(lib::Position3D init_position, double loop_frequency) :
    position_(init_position),
    profile_(0.5, 0.5, 1.0 / loop_frequency) {}

void DronePlant::MoveDrone(Vector3D flight_direction) {
  ::Eigen::Vector3d flight_direction_eigen(
      flight_direction.x, flight_direction.y, flight_direction.z);

  ::Eigen::Vector3d actual_delta_position =
      profile_.Calculate(flight_direction_eigen);

  position_.latitude +=
      actual_delta_position(0) * profile_.delta_time() / kMetersPerCoordinate;

  position_.longitude +=
      actual_delta_position(1) * profile_.delta_time() / kMetersPerCoordinate;

  position_.altitude -= actual_delta_position(2) * profile_.delta_time();
}

} // namespace drone_plant
} // namespace simulator
} // namespace timeline
} // namespace ground_server
} // namespace controls
} // namespace src
