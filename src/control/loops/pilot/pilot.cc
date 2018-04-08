#include "pilot.h"

namespace src {
namespace control {
namespace loops {
namespace pilot {

Pilot::Pilot() {}

PilotOutput Pilot::Calculate(Position3D drone_position) {
  ::lib::mission_manager::Command cmd =
      mission_message_queue_receiver_.get_mission_manager()
          ->GetCurrentCommand();

  Vector3D flight_direction = {0, 0, 0};
  bool bomb_drop = false;

  if (cmd.has_nothingcommand()) {
    // Do nothing.
  } else if (cmd.has_sleepcommand()) {
    // Sleep.
  } else if (cmd.has_gotocommand()) {
    constexpr double kSpeed = 15.0;

    Position3D goal = {cmd.gotocommand().latitude(),
                       cmd.gotocommand().longitude(),
                       cmd.gotocommand().altitude()};

    flight_direction = PointTowards(drone_position, goal);
    flight_direction *= kSpeed;

    if (GetDistance2D(drone_position, goal) < kSpeed) {
      mission_message_queue_receiver_.get_mission_manager()->PopCommand();
    }
  } else if (cmd.has_bombcommand()) {
    flight_direction = {0, 0, 0};
    bomb_drop = true;
  } else {
    ::std::cout << "ERROR: Unknown command.\n";
  }

  return {flight_direction, bomb_drop};
}

int Pilot::GetCurrentCommandIndex() {
  return mission_message_queue_receiver_.get_mission_manager()
      ->GetCurrentCommandIndex();
}

}  // namespace pilot
}  // namespace loops
}  // namespace control
}  // namespace src
