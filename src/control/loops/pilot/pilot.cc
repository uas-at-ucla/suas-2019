#include "pilot.h"

namespace src {
namespace control {
namespace loops {
namespace pilot {

Pilot::Pilot() {}

PilotOutput Pilot::Calculate(Position3D drone_position) {
  ::std::shared_ptr<::lib::MissionCommand> cmd_ptr =
      mission_message_queue_receiver_.get_mission_manager()
          ->GetCurrentCommand();

  Vector3D flight_direction;
  bool bomb_drop = false;

  switch (cmd_ptr->type()) {
    case ::lib::MissionCommand::GOTO: {
      constexpr double kSpeed = 15.0;

      ::std::shared_ptr<::lib::MissionCommandGoto> goto_cmd_ptr =
          ::std::static_pointer_cast<::lib::MissionCommandGoto>(cmd_ptr);

      Position3D goal = {goto_cmd_ptr->latitude(), goto_cmd_ptr->longitude(),
                         goto_cmd_ptr->altitude()};

      flight_direction = PointTowards(drone_position, goal);
      flight_direction *= kSpeed;

      if (GetDistance2D(drone_position, goal) < kSpeed) {
        mission_message_queue_receiver_.get_mission_manager()->PopCommand();
      }

      break;
    }

    case ::lib::MissionCommand::BOMB_DROP: {
      flight_direction = {0, 0, 0};
      bomb_drop = true;
      break;
    }

    case ::lib::MissionCommand::DO_NOTHING: {
      flight_direction = {0, 0, 0};
      break;
    }
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
