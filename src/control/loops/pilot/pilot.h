#ifndef SRC_CONTROL_LOOPS_FLIGHT_LOOP_PILOT_PILOT_H_
#define SRC_CONTROL_LOOPS_FLIGHT_LOOP_PILOT_PILOT_H_

#include <atomic>
#include <condition_variable>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "zmq.hpp"

#include "lib/mission_message_queue/mission_message_queue.h"
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

class Pilot {
 public:
  Pilot();

  PilotOutput Calculate(Position3D drone_position);

 private:
  ::lib::mission_message_queue::MissionMessageQueueReceiver
      mission_message_queue_receiver_;
};

}  // namespace pilot
}  // namespace loops
}  // namespace control
}  // namespace src

#endif  // SRC_CONTROL_LOOPS_FLIGHT_LOOP_PILOT_PILOT_H_
