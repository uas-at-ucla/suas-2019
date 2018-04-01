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

  if (cmd.has_nothing_command()) {
    // Do nothing.
  } else if (cmd.has_sleep_command()) {
    // Sleep.
  } else if (cmd.has_goto_command()) {
    constexpr double kSpeed = 15.0;

    Position3D goal = {cmd.goto_command().latitude(),
                       cmd.goto_command().longitude(),
                       cmd.goto_command().altitude()};

    flight_direction = PointTowards(drone_position, goal);
    flight_direction *= kSpeed;

    if (GetDistance2D(drone_position, goal) < kSpeed) {
      mission_message_queue_receiver_.get_mission_manager()->PopCommand();
    }
  } else if (cmd.has_bomb_command()) {
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
