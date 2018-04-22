#include "pilot.h"

namespace src {
namespace control {
namespace loops {
namespace pilot {

Pilot::Pilot()
    : drone_position_set_(false),
      drone_position_semaphore_(1),
      thread_(&Pilot::PreprocessorThread, this) {}

Pilot::~Pilot() {
  Quit();
  thread_.join();
}

void Pilot::PreprocessorThread() {
  // Add test obstacle while developing.
  while (run_) {
    if (drone_position_set_) {
      drone_position_semaphore_.Wait();
      Position3D drone_position = drone_position_;
      drone_position_semaphore_.Notify();

      mission_message_queue_receiver_.RunPreprocessor(drone_position);
    }

    usleep(1e4);
  }
}

PilotOutput Pilot::Calculate(Position3D drone_position) {
  drone_position_semaphore_.Wait();
  drone_position_ = drone_position;
  drone_position_set_ = true;
  drone_position_semaphore_.Notify();

  ::lib::mission_manager::Command cmd =
      mission_message_queue_receiver_.get_mission_manager()
          ->GetCurrentCommand();

  Vector3D flight_direction = {0, 0, 0};
  bool bomb_drop = false;

  if (cmd.has_nothingcommand()) {
    // Do nothing.
    mission_message_queue_receiver_.get_mission_manager()->PopCommand();
  } else if (cmd.has_sleepcommand()) {
    // Sleep.
  } else if (cmd.has_gotorawcommand()) {
    constexpr double kSpeed = 2.0;

    Position3D goal = {cmd.gotorawcommand().goal().latitude(),
                       cmd.gotorawcommand().goal().longitude(),
                       cmd.gotorawcommand().goal().altitude()};

    flight_direction = PointTowards(drone_position, goal);
    flight_direction *= kSpeed;

    if (GetDistance2D(drone_position, goal) < 3) {
      mission_message_queue_receiver_.get_mission_manager()->PopCommand();
    }
  } else if (cmd.has_bombdropcommand()) {
    flight_direction = {0, 0, 0};
    bomb_drop = true;
  } else {
    ::std::cout << "ERROR: Unknown command.\n";
  }

  return {flight_direction, bomb_drop};
}

int Pilot::GetCurrentCommandIndex() {
  // DEPRECATED: DON'T USE.
  return -1;
}

void Pilot::SetMission(::lib::mission_manager::Mission mission) {
  mission_message_queue_receiver_.SetMission(mission);
}

}  // namespace pilot
}  // namespace loops
}  // namespace control
}  // namespace src
