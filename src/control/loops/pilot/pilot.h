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
#include <cmath>
#include <algorithm>
#include <iostream>

#include "zmq.hpp"

#include <google/protobuf/util/message_differencer.h>

#include "lib/mission_message_queue/mission_message_queue.h"
#include "lib/mission_manager/mission_commands.pb.h"
#include "lib/physics_structs/physics_structs.h"
#include "lib/semaphore/semaphore.h"

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
  ~Pilot();

  PilotOutput Calculate(Position3D drone_position);
  void PreprocessorThread();
  void SetMission(::lib::mission_manager::Mission mission);

  void Quit() { run_ = false; }

 private:
  ::lib::mission_message_queue::MissionMessageQueueReceiver
      mission_message_queue_receiver_;

  ::lib::mission_manager::Command cmd_, last_cmd_;
  bool cmd_set_;

  Position3D drone_position_;
  bool drone_position_set_;
  ::lib::Semaphore drone_position_semaphore_;

  ::std::atomic<bool> run_{true};

  ::std::thread thread_;
};

}  // namespace pilot
}  // namespace loops
}  // namespace control
}  // namespace src

#endif  // SRC_CONTROL_LOOPS_FLIGHT_LOOP_PILOT_PILOT_H_
