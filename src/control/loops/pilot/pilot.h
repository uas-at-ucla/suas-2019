#ifndef SPINNY_CONTROL_LOOPS_FLIGHT_LOOP_PILOT_PILOT_H_
#define SPINNY_CONTROL_LOOPS_FLIGHT_LOOP_PILOT_PILOT_H_

#include <condition_variable>
#include <map>
#include <string>
#include <vector>
#include <memory>
#include <iostream>

#include "zmq.hpp"

#include "lib/mission_manager/mission_manager.h"
#include "lib/physics_structs/physics_structs.h"

#include "src/control/mission_receiver/mission_commands.pb.h"

namespace spinny {
namespace control {
namespace loops {
namespace pilot {

struct PilotOutput {
  Vector3D flight_velocities;
  bool bomb_drop;
};

class Pilot {
 public:
  Pilot();

  PilotOutput Calculate(Position3D drone_position);

 private:
  MissionManager mission_manager_;
};

class PilotMissionHandler {
 public:
  PilotMissionHandler(MissionManager *mission_manager);
  void operator()();
  ::std::vector<::std::shared_ptr<MissionCommand>> ParseMissionProtobuf(
      ::spinny::control::mission_receiver::Mission mission_protobuf);
  void Quit() { run_ = false; }

 private:
  MissionManager *mission_manager_;

  ::std::atomic<bool> run_{true};
};

}  // namespace pilot
}  // namespace loops
}  // namespace control
}  // namespace spinny

#endif  // SPINNY_CONTROL_LOOPS_FLIGHT_LOOP_PILOT_PILOT_H_
