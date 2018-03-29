#ifndef SPINNY_CONTROL_LOOPS_FLIGHT_LOOP_PILOT_PILOT_H_
#define SPINNY_CONTROL_LOOPS_FLIGHT_LOOP_PILOT_PILOT_H_

#include <condition_variable>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "zmq.hpp"

#include "lib/mission_manager/mission_manager.h"
#include "lib/physics_structs/physics_structs.h"

#include "src/control/ground_communicator/mission_commands.pb.h"

namespace src {
namespace control {
namespace loops {
namespace pilot {

struct PilotOutput {
  Vector3D flight_velocities;
  bool bomb_drop;
};

class PilotMissionHandler {
 public:
  PilotMissionHandler(::lib::MissionManager *mission_manager);
  void operator()();
  ::std::vector<::std::shared_ptr<::lib::MissionCommand>> ParseMissionProtobuf(
      ::src::controls::ground_communicator::Mission mission_protobuf);
  void Quit() { run_ = false; }

 private:
  ::lib::MissionManager *mission_manager_;

  ::std::atomic<bool> run_{true};
};

class Pilot {
 public:
  Pilot();

  PilotOutput Calculate(Position3D drone_position);

  void HandleMission();

 private:
  ::lib::MissionManager mission_manager_;
  PilotMissionHandler pilot_mission_handler_;
};

}  // namespace pilot
}  // namespace loops
}  // namespace control
}  // namespace src

#endif  // SPINNY_CONTROL_LOOPS_FLIGHT_LOOP_PILOT_PILOT_H_
