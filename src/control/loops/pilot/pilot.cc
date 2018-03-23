#include "pilot.h"

#include <iostream>

namespace src {
namespace control {
namespace loops {
namespace pilot {

Pilot::Pilot() {}

Vector3D Pilot::Calculate(Position3D drone_position, Position3D goal) {
  constexpr double kSpeed = 15.0;

  Vector3D flight_direction = PointTowards(drone_position, goal);
  flight_direction *= kSpeed;

  return flight_direction;
}

}  // namespace pilot
}  // namespace loops
}  // namespace control
}  // namespace src
