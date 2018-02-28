#include "pilot.h"

#include <iostream>

namespace spinny {
namespace control {
namespace loops {
namespace pilot {

Pilot::Pilot() {}

Vector3D Pilot::Calculate(Position3D drone_position) {
  Position3D goal = {0, 0, 10};

  Vector3D flight_direction = PointTowards(drone_position, goal);

  return flight_direction;
}

}  // namespace pilot
}  // namespace loops
}  // namespace control
}  // namespace spinny
